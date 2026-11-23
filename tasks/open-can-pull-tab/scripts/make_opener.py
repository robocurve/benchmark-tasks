"""Writes tasks/open-can-pull-tab/assets/opener.usda: a 3D-printable ring-pull tab lifter, a rigid body standing like a
screwdriver: a 16 x 16 x 90 mm handle on top (offset 3.75 mm towards the tongue), a 3 x 8 mm plate (30 mm long) down from its bottom, and at the plate's
foot a 5 x 8 x 1.8 mm tongue pointing +x. The plate goes down through the tab's hole, the tongue slides under the
tab's side bar, and lifting the handle lifts the ring. Origin: the foot's bottom, on the plate's centre line; z up.

    python make_opener.py <out.usda>
"""
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

MM = 0.001
HANDLE = (-4.25, 11.75, -8, 8, 30, 120)  # centred over where the tongue bears on the ring's bar, so the pull goes through the pinch  # x0 x1 y0 y1 z0 z1 (mm)
PLATE = (-1.5, 1.5, -4, 4, 0, 30)
TONGUE = (1.5, 6.5, -4, 4, 0, 1.8)
MASS = 0.030
PLASTIC = (0.15, 0.45, 0.80)


def box(st, path, b, coll, color):
    x0, x1, y0, y1, z0, z1 = (v * MM for v in b)
    c = UsdGeom.Cube.Define(st, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    c.AddScaleOp().Set(Gf.Vec3f(x1 - x0, y1 - y0, z1 - z0))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if coll:
        p = c.GetPrim()
        UsdPhysics.CollisionAPI.Apply(p)
        p.AddAppliedSchema("PhysxCollisionAPI")
        p.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.0004)
        p.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.0)
        c.CreatePurposeAttr("guide")
    return c


def main(out):
    st = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    r = UsdGeom.Xform.Define(st, "/Opener")
    st.SetDefaultPrim(r.GetPrim())
    p = r.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    m = UsdPhysics.MassAPI.Apply(p)
    m.CreateMassAttr(MASS)
    m.CreateCenterOfMassAttr(Gf.Vec3f(0.00375, 0, 0.07))
    mat = UsdShade.Material.Define(st, "/Opener/Material")
    a = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    a.CreateStaticFrictionAttr(0.6)
    a.CreateDynamicFrictionAttr(0.5)
    a.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(p).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    UsdGeom.Scope.Define(st, "/Opener/visual")
    UsdGeom.Scope.Define(st, "/Opener/collision")
    for n, b in (("handle", HANDLE), ("plate", PLATE), ("tongue", TONGUE)):
        box(st, f"/Opener/visual/{n}", b, False, PLASTIC)
        box(st, f"/Opener/collision/{n}", b, True, PLASTIC)
    st.GetRootLayer().Save()
    print("wrote", out)


if __name__ == "__main__":
    main(sys.argv[1])
