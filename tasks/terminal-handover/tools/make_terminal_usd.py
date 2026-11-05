"""Builds assets/terminal_flat.usd: the terminal block as one rigid body, lying flat.

    python tools/make_terminal_usd.py SOURCE.usd [--out assets/terminal_flat.usd]

SOURCE is a terminal block made from the manufacturer's STEP file: housing and latch foot as two meshes
(`/root/Gehaeuse/Gehaeuse`, `/root/Rastfuss/Rastfuss`), an articulation with the foot on a prismatic joint.
For a handover the latch never moves, so both meshes become colliders of a single rigid body and the joint is
dropped. The geometry is rotated -90 deg about y so the part lies on its side: the source's thickness axis (x)
points up, its height axis (z, latch foot at the bottom) points along -x, its length axis (y) stays along y.

Needs only `pxr` (usd-core, or the Isaac Sim venv); does not start Isaac Sim.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

PARTS = {"housing": "/root/Gehaeuse/Gehaeuse", "latch_foot": "/root/Rastfuss/Rastfuss"}
# Source -> flat: x -> z, z -> -x, y -> y (rotation of -90 deg about y).
FLAT = Gf.Matrix3d(Gf.Rotation(Gf.Vec3d(0, 1, 0), -90.0))


def build(source: Path, out: Path) -> None:
    src = Usd.Stage.Open(str(source))
    dst = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageUpAxis(dst, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(dst, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(dst, 1.0)

    root = UsdGeom.Xform.Define(dst, "/terminal")
    dst.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())

    total_mass = 0.0
    for name, path in PARTS.items():
        mesh = UsdGeom.Mesh(src.GetPrimAtPath(path))
        if not mesh:
            raise SystemExit(f"{source}: no mesh at {path}")
        mass = UsdPhysics.MassAPI(mesh.GetPrim()).GetMassAttr().Get() or 0.0
        total_mass += mass
        points = [FLAT.GetTranspose() * Gf.Vec3d(p) for p in mesh.GetPointsAttr().Get()]  # row-vector convention
        part = UsdGeom.Mesh.Define(dst, f"/terminal/{name}")
        part.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(p) for p in points]))
        part.CreateFaceVertexCountsAttr(mesh.GetFaceVertexCountsAttr().Get())
        part.CreateFaceVertexIndicesAttr(mesh.GetFaceVertexIndicesAttr().Get())
        part.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        part.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.62, 0.63, 0.62)]))
        UsdPhysics.CollisionAPI.Apply(part.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(part.GetPrim()).CreateApproximationAttr(
            UsdPhysics.Tokens.convexDecomposition
        )

    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(total_mass)
    root.GetPrim().SetCustomDataByKey("source", source.name)
    dst.GetRootLayer().Save()

    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"]).ComputeWorldBound(root.GetPrim()).ComputeAlignedRange()
    size = bounds.GetMax() - bounds.GetMin()
    print(f"{out}: {total_mass * 1000:.2f} g, flat size {size[0]*1000:.1f} x {size[1]*1000:.1f} x {size[2]*1000:.1f} mm")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", type=Path)
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parents[1] / "assets" / "terminal_flat.usd")
    args = ap.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    if args.out.exists():
        args.out.unlink()
    build(args.source, args.out)


if __name__ == "__main__":
    main()
