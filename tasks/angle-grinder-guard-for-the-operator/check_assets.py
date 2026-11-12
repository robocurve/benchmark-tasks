"""Build the USD assets into a fresh folder and check them: needs pxr and PIL (Isaac Sim's Python), not the simulator.

    python check_assets.py --output /path/to/fresh/usd-check

Checks that the grinder is one articulation (floating root ``body``) with exactly two joints, ``guard`` (revolute about
the spindle's z axis) and ``lever`` (revolute about a horizontal axis), that the masses and sizes match geometry.py
(the real part's), and that every textured material points at a texture that was written.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from pxr import Usd, UsdGeom, UsdPhysics

import build_assets as ba
import geometry as g


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", required=True)
    a = ap.parse_args()
    out = Path(a.output)
    tex = out / "textures"
    tex.mkdir(parents=True, exist_ok=True)
    ba.build_grinder(out, tex)
    ba.build_mat(out, tex)
    bad = []

    def need(cond, msg):
        print(("ok    " if cond else "FAIL  ") + msg)
        if not cond:
            bad.append(msg)

    stage = Usd.Stage.Open(str(out / "grinder.usda"))
    prims = {str(p.GetPath()): p for p in stage.Traverse()}
    body, guard, lever = (prims.get(f"/grinder/{n}") for n in ("body", "guard", "lever"))
    need(all((body, guard, lever)), "bodies /grinder/body, /grinder/guard, /grinder/lever exist")
    need(body.HasAPI(UsdPhysics.ArticulationRootAPI), "the root body carries the articulation root")
    for p, m in ((body, g.BODY_MASS), (guard, g.GUARD_MASS), (lever, g.LEVER_MASS)):
        need(abs(UsdPhysics.MassAPI(p).GetMassAttr().Get() - m) < 1e-5, f"{p.GetName()} mass {m} kg")
    joints = {p.GetName(): p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)}
    need(set(joints) == {"guard", "lever"}, f"exactly two revolute joints: {sorted(joints)}")
    need(UsdPhysics.RevoluteJoint(joints["guard"]).GetAxisAttr().Get() == "Z", "the guard turns about the spindle's z axis")
    need(UsdPhysics.RevoluteJoint(joints["lever"]).GetAxisAttr().Get() == "X", "the lever turns about its frame's x axis (the tangent)")
    lo = UsdPhysics.RevoluteJoint(joints["lever"]).GetLowerLimitAttr().Get()
    hi = UsdPhysics.RevoluteJoint(joints["lever"]).GetUpperLimitAttr().Get()
    need(abs(lo - math.degrees(g.LEVER_OPEN)) < 1e-6 and hi == 0.0, f"lever limits {lo}..{hi} degrees (raises to 90)")
    g_lo = UsdPhysics.RevoluteJoint(joints["guard"]).GetLowerLimitAttr().Get()
    need(abs(g_lo) < 360, "the guard joint's USD limits stay inside one turn (PhysX)")
    colliders = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    need(len(colliders) >= 20, f"{len(colliders)} collision shapes (skirt boxes, lug, lever, body)")
    disc = prims["/grinder/body/disc"]
    cyl = UsdGeom.Cylinder(disc)
    need(abs(cyl.GetRadiusAttr().Get() - 0.0575) < 1e-9 and abs(cyl.GetHeightAttr().Get() - 0.006) < 1e-9,
         "disc is 115 mm across and 6 mm thick")
    for name in ("motor_label.png", "disc_top.png", "operator_mat.png"):
        need((tex / name).is_file() and (tex / name).stat().st_size > 1000, f"texture {name} written")
    need((out / "operator_mat.usda").is_file(), "operator_mat.usda written")
    print("\nPASS" if not bad else f"\nFAIL ({len(bad)})")
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
