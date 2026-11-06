"""Make a self-contained rigid tool from Arena's existing spatula_15 mesh.

Run with usd-core. Source meshes are unchanged; two convex hulls replace authored
mesh decomposition, filling small visual slots. A rigid grip sleeve is added.
No textures are required.
"""
from pathlib import Path
import urllib.request
from pxr import Usd, UsdGeom, UsdPhysics, Gf

URL = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/objects/vomp/spatula_15/spatula_15.usd"


def build(source, destination):
    src = Usd.Stage.Open(str(source))
    out = Usd.Stage.CreateNew(str(destination))
    UsdGeom.SetStageMetersPerUnit(out, 1.0)
    UsdGeom.SetStageUpAxis(out, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(out, "/Spatula")
    out.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(0.08)
    xf = UsdGeom.XformCache()
    for prim in src.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        name = "blade" if "Blade" in prim.GetName() else "handle"
        dest = UsdGeom.Mesh.Define(out, "/Spatula/" + name)
        transform = xf.GetLocalToWorldTransform(prim)
        dest.CreatePointsAttr([Gf.Vec3f(transform.Transform(Gf.Vec3d(p))) for p in mesh.GetPointsAttr().Get()])
        dest.CreateFaceVertexCountsAttr(mesh.GetFaceVertexCountsAttr().Get())
        dest.CreateFaceVertexIndicesAttr(mesh.GetFaceVertexIndicesAttr().Get())
        dest.CreateSubdivisionSchemeAttr("none")
        dest.CreateDoubleSidedAttr(True)
        dest.CreateDisplayColorAttr([Gf.Vec3f(.65, .68, .70) if name == "blade" else Gf.Vec3f(.12, .18, .22)])
        UsdPhysics.CollisionAPI.Apply(dest.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(dest.GetPrim()).CreateApproximationAttr("convexHull")
    sleeve = UsdGeom.Cube.Define(out, "/Spatula/grip_sleeve")
    sleeve.CreateSizeAttr(1.0)
    sleeve.AddTranslateOp().Set(Gf.Vec3d(-.10, 0, .006))
    sleeve.AddScaleOp().Set(Gf.Vec3f(.035, .024, .018))
    sleeve.CreateDisplayColorAttr([Gf.Vec3f(.08,.12,.14)])
    UsdPhysics.CollisionAPI.Apply(sleeve.GetPrim())
    out.GetRootLayer().customLayerData = {"source_url": URL, "source_registry_name": "spatula_15_vomp_robolab", "modification": "35 x 24 x 18 mm rigid rubber grip sleeve centred at local (-100,0,6) mm"}
    out.GetRootLayer().Save()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path)
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    source = args.source or here / "assets" / "source-spatula.usd"
    if not source.exists():
        urllib.request.urlretrieve(URL, source)
    build(source, here / "assets" / "spatula.usdc")
