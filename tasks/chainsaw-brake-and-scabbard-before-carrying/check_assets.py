"""Check the built USD assets against geometry.py (run after build_assets.py; needs pxr, not the simulator).

    python check_assets.py
"""

from __future__ import annotations

import sys
from pathlib import Path

from pxr import Usd, UsdGeom, UsdPhysics

import geometry as g

HERE = Path(__file__).resolve().parent / "assets"
bad: list[str] = []


def need(cond: bool, msg: str) -> None:
    print(("ok   " if cond else "FAIL ") + msg)
    if not cond:
        bad.append(msg)


def colliders(stage, root):
    return [p for p in Usd.PrimRange(stage.GetPrimAtPath(root)) if p.HasAPI(UsdPhysics.CollisionAPI)]


def main() -> int:
    saw = Usd.Stage.Open(str(HERE / "chainsaw.usda"))
    need(saw.GetDefaultPrim().GetName() == "chainsaw", "chainsaw.usda default prim is /chainsaw")
    need(saw.GetPrimAtPath("/chainsaw/saw").HasAPI(UsdPhysics.ArticulationRootAPI), "the saw body is the articulation root")
    joints = [p for p in saw.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    need([p.GetName() for p in joints] == ["guard"], f"one revolute joint, named guard: {[p.GetName() for p in joints]}")
    j = UsdPhysics.RevoluteJoint(joints[0])
    need(j.GetAxisAttr().Get() == "Z", "the guard hinge is about the vertical axis")
    lo, hi = j.GetLowerLimitAttr().Get(), j.GetUpperLimitAttr().Get()
    import math

    need(abs(lo - math.degrees(g.GUARD_LIMITS[0])) < 1e-3 and abs(hi - math.degrees(g.GUARD_LIMITS[1])) < 1e-3,
         f"guard limits {lo:.1f} to {hi:.1f} deg match geometry")
    masses = {p.GetName(): UsdPhysics.MassAPI(p).GetMassAttr().Get() for p in saw.Traverse()
              if p.HasAPI(UsdPhysics.MassAPI)}
    need(abs(sum(masses.values()) - g.SAW_MASS) < 1e-6, f"masses add up to {g.SAW_MASS} kg: {masses}")
    names = {p.GetName() for p in colliders(saw, "/chainsaw")}
    for want in ("housing", "battery", "sprocket_cover", "bar_plate", "bar_nose", "chain_L", "chain_R", "chain_nose",
                 "front_handle", "plate", "grip_bar"):
        need(want in names, f"collider {want}")
    need("hinge_bracket" not in names, "the hinge bracket is visual only")
    top = max(UsdGeom.Imageable(p).ComputeWorldBound(0, "default").ComputeAlignedRange().GetMax()[2]
              for p in colliders(saw, "/chainsaw"))
    need(abs(top - (g.BAR_Z + g.CHAIN_H / 2)) < 0.03, f"the highest collider is at {top * 1000:.0f} mm")

    mir = Usd.Stage.Open(str(HERE / "chainsaw_b.usda"))       # layout B: the mirror image
    need(mir.GetDefaultPrim().GetName() == "chainsaw", "chainsaw_b.usda default prim is /chainsaw")
    mj = [p for p in mir.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    need([p.GetName() for p in mj] == ["guard"], "chainsaw_b: one revolute joint, named guard")
    need(UsdPhysics.RevoluteJoint(mj[0]).GetLowerLimitAttr().Get() == lo and UsdPhysics.RevoluteJoint(mj[0]).GetUpperLimitAttr().Get() == hi,
         "chainsaw_b: the guard's limits are the same (its hinge axis is turned over)")
    need(UsdPhysics.RevoluteJoint(mj[0]).GetLocalRot0Attr().Get().GetReal() == 0.0, "chainsaw_b: hinge frame turned 180 degrees about x")
    ya = UsdGeom.Xformable(saw.GetPrimAtPath("/chainsaw/saw/housing")).ComputeLocalToWorldTransform(0).ExtractTranslation()[1]
    yb = UsdGeom.Xformable(mir.GetPrimAtPath("/chainsaw/saw/housing")).ComputeLocalToWorldTransform(0).ExtractTranslation()[1]
    need(abs(ya + yb) < 1e-9 and ya > 0, f"the housing is at y = {ya * 1000:.0f} mm in A and {yb * 1000:.0f} mm in B")

    sc = Usd.Stage.Open(str(HERE / "scabbard.usda"))
    need(sc.GetPrimAtPath("/scabbard").HasAPI(UsdPhysics.RigidBodyAPI), "the scabbard is a rigid body")
    need(abs(UsdPhysics.MassAPI(sc.GetPrimAtPath("/scabbard")).GetMassAttr().Get() - g.SCAB_MASS) < 1e-6, "scabbard mass")
    need(len(colliders(sc, "/scabbard")) == 5, "five colliders: top, bottom, two walls, closed end")
    for tex in ("warning", "brake", "bar", "battery", "scabbard"):
        need((HERE / "textures" / f"{tex}.png").is_file(), f"texture {tex}.png")
    print("PASS" if not bad else f"FAIL ({len(bad)})")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
