"""Builds assets/busbar_4m.usd: the four-module fork busbar as one rigid body, lying on its back.

    python tools/make_busbar_usd.py [--out assets/busbar_4m.usd]

The geometry is the task's own estimate (task.py: BODY, TINE_*, FORK_Y), built from boxes: the insulated body and
two copper tines per fork, each with the simulation's latch notch (TINE_NOTCH) in its front face. It is authored in
the lying pose (task.LYING: forks pointing +x), with the prim's origin at the body's centre. Needs only `pxr` and NumPy (run it in Arena's venv); does not start Isaac Sim.
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

TASK_DIR = Path(__file__).resolve().parents[1]


def _task():
    spec = importlib.util.spec_from_file_location("phase_busbar_task", TASK_DIR / "task.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build(out: Path) -> None:
    t = _task()
    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/busbar")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(t.BUSBAR_MASS)

    def material(name, srgb, metallic):
        mat = UsdShade.Material.Define(stage, f"/busbar/looks/{name}")
        shader = UsdShade.Shader.Define(stage, f"/busbar/looks/{name}/shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        lin = [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in srgb]
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*lin))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35 if metallic else 0.6)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(1.0 if metallic else 0.0)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return mat

    looks = {"body": material("insulation", t.BODY_COLOR, False), "tine": material("copper", t.COPPER, True)}

    def box(name, size, centre, kind):
        """A box given in the installed frame, authored in the lying pose."""
        centre = t.LYING @ np.asarray(centre, dtype=float)
        size = np.abs(t.LYING) @ np.asarray(size, dtype=float)
        geom = UsdGeom.Cube.Define(stage, f"/busbar/{name}")
        geom.CreateSizeAttr(1.0)
        geom.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        geom.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(looks[kind])

    box("body", t.BODY, (0.0, 0.0, 0.0), "body")
    z0 = t.BODY[2] / 2 - t.TINE_ROOT
    depth, notch_top, notch_bottom = t.TINE_NOTCH
    tip = t.BODY[2] / 2 + t.TINE_LEN
    for i, y in enumerate(t.FORK_Y):
        for j, side in enumerate((-1, 1)):
            ty = y + side * (t.TINE_GAP + t.TINE_W) / 2
            # below the notch, the notch (the wall-side part of the tine only), above the notch
            lo, hi = z0, tip - notch_bottom
            box(f"fork{i}_tine{j}_low", (t.TINE_T, t.TINE_W, hi - lo), (0.0, ty, (lo + hi) / 2), "tine")
            lo, hi = tip - notch_bottom, tip - notch_top
            rest = t.TINE_T - depth
            box(f"fork{i}_tine{j}_notch", (rest, t.TINE_W, hi - lo), (t.TINE_T / 2 - rest / 2, ty, (lo + hi) / 2), "tine")
            lo, hi = tip - notch_top, tip
            box(f"fork{i}_tine{j}_tip", (t.TINE_T, t.TINE_W, hi - lo), (0.0, ty, (lo + hi) / 2), "tine")

    stage.GetRootLayer().Save()
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"]).ComputeWorldBound(root.GetPrim()).ComputeAlignedRange()
    lo, hi = bounds.GetMin(), bounds.GetMax()
    print(f"{out}: {t.BUSBAR_MASS * 1000:.0f} g, lying bounds x {lo[0]*1000:.1f}..{hi[0]*1000:.1f} "
          f"y {lo[1]*1000:.1f}..{hi[1]*1000:.1f} z {lo[2]*1000:.1f}..{hi[2]*1000:.1f} mm")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=TASK_DIR / "assets" / "busbar_4m.usd")
    args = ap.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    if args.out.exists():
        args.out.unlink()
    build(args.out)


if __name__ == "__main__":
    sys.exit(main())
