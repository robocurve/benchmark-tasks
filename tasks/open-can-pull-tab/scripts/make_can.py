"""Writes tasks/open-can-pull-tab/assets/can.usda: the NVIDIA USDCraft-10K 'Standard 330 ml Can' (ds_R043_01, Apache-2.0)
as a floating-base articulation: the can (root) and its ring-pull tab on a revolute joint about x at the rivet. The
visual meshes are the asset's own (can wall, end, tab); the colliders are rebuilt:

  * the can: a cylinder body (66 mm), the neck, the end panel 6 mm under the rim with a 3 mm deeper finger well under
    the tab's ring (as stay-on ends have), and the rim as 32 slabs;
  * the tab: four boxes round the ring's 11.8 x 13 mm hole (measured from the asset's tab mesh), 1.6 mm thick.

    python make_can2.py <model.usdc> <out.usda>
"""
import math
import sys

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from make_can_helpers import circle, clip, collider, extrusion, material  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

MM = 0.001
BODY_R, BODY_H = 33.0 * MM, 96.0 * MM
NECK_R, NECK_TOP = 29.3 * MM, 110.0 * MM
PANEL_TOP = 114.0 * MM
WELL_DEPTH = 3.0 * MM
WELL_X, WELL_Y0 = 10.0 * MM, 4.0 * MM
PANEL_R = 28.0 * MM
RIM_TOP = 120.0 * MM
RIM_R0, RIM_R1 = 28.0 * MM, 31.5 * MM
RIVET = (0.0, -1.0 * MM, 115.0 * MM)
TAB_Z0, TAB_Z1 = 114.8 * MM, 116.4 * MM
RING = {  # x0 x1 y0 y1 (mm, can frame)
    "near": (-9.4, 9.4, -0.6, 6.0),
    "far": (-7.0, 7.0, 19.0, 22.2),
    "left": (-9.4, -5.9, 6.0, 19.0),
    "right": (5.9, 9.4, 6.0, 19.0),
}
CAN_MASS, TAB_MASS = 0.35, 0.0007


def box(st, path, x0, x1, y0, y1, z0, z1):
    b = UsdGeom.Cube.Define(st, path)
    b.CreateSizeAttr(1.0)
    b.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    b.AddScaleOp().Set(Gf.Vec3f(x1 - x0, y1 - y0, z1 - z0))
    b.CreatePurposeAttr("guide")
    collider(b.GetPrim())


def prism(st, path, poly, z0, z1):
    extrusion(st, path, poly, z0, z1, (0.8, 0.8, 0.8), coll=True)
    UsdGeom.Imageable(st.GetPrimAtPath(path)).CreatePurposeAttr("guide")


def main(model, out):
    st = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = UsdGeom.Xform.Define(st, "/Can")
    st.SetDefaultPrim(root.GetPrim())
    p = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    UsdPhysics.ArticulationRootAPI.Apply(p)
    p.AddAppliedSchema("PhysxArticulationAPI")
    p.AddAppliedSchema("PhysxRigidBodyAPI")
    m = UsdPhysics.MassAPI.Apply(p)
    m.CreateMassAttr(CAN_MASS)
    m.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0.055))
    I = CAN_MASS * (3 * BODY_R**2 + 0.12**2) / 12
    m.CreateDiagonalInertiaAttr(Gf.Vec3f(I, I, CAN_MASS * BODY_R**2 / 2))
    for k, (t, v) in {"enabledSelfCollisions": (Sdf.ValueTypeNames.Bool, False), "solverPositionIterationCount": (Sdf.ValueTypeNames.Int, 32),
                      "solverVelocityIterationCount": (Sdf.ValueTypeNames.Int, 4), "sleepThreshold": (Sdf.ValueTypeNames.Float, 0.0),
                      "stabilizationThreshold": (Sdf.ValueTypeNames.Float, 0.0)}.items():
        p.CreateAttribute(f"physxArticulation:{k}", t, custom=True).Set(v)
    mat = material(st, "/Can/Material", 0.8, 0.7)
    UsdShade.MaterialBindingAPI.Apply(p).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    # the asset's visuals and looks
    src = Sdf.Layer.FindOrOpen(model)
    dst = st.GetRootLayer()
    Sdf.CreatePrimInLayer(dst, "/Can/Looks")
    Sdf.CopySpec(src, "/Asset/Looks", dst, "/Can/Looks")
    UsdGeom.Xform.Define(st, "/Can/tab")
    tp = st.GetPrimAtPath("/Can/tab")
    tab = UsdGeom.Xform(tp)
    tab.AddTranslateOp().Set(Gf.Vec3d(*RIVET))
    UsdGeom.Xform.Define(st, "/Can/visual")
    UsdGeom.Xform.Define(st, "/Can/tab/visual")  # the asset's tab geometry is already relative to its link origin, the rivet
    for kind in ("wall", "end"):
        Sdf.CopySpec(src, f"/Asset/Links/can_body/Visuals/{kind}", dst, f"/Can/visual/{kind}")
    Sdf.CopySpec(src, "/Asset/Links/ring_tab/Visuals/tab", dst, "/Can/tab/visual/tab")
    for path, look in (("/Can/visual/wall/Geom/can_wall_obj", "smooth_aluminium"), ("/Can/visual/end/Geom/can_end_obj", "matte_aluminium"),
                       ("/Can/tab/visual/tab/Geom/can_pull_tab_obj", "tab_aluminium")):
        UsdShade.MaterialBindingAPI.Apply(st.GetPrimAtPath(path)).Bind(UsdShade.Material(st.GetPrimAtPath(f"/Can/Looks/{look}")))

    # the finger well in the visible end, to match the colliders: the panel under the ring is pressed 3.5 mm lower
    # (sloping walls over 2.5 mm), so the lifter's tongue does not pass through the surface
    import numpy as np
    endp = st.GetPrimAtPath("/Can/visual/end/Geom/can_end_obj")
    pts = np.array(endp.GetAttribute("points").Get(), dtype=np.float64)
    x, y, z = pts[:, 0] / MM, pts[:, 1] / MM, pts[:, 2] / MM
    r = np.hypot(x, y)
    def ramp(v, a, b):  # 1 for v <= a, 0 for v >= b, smooth between
        t = np.clip((v - a) / (b - a), 0.0, 1.0)
        return 1.0 - t * t * (3 - 2 * t)
    w = ramp(abs(x), WELL_X / MM - 1.0, WELL_X / MM + 1.5) * (1.0 - ramp(y, WELL_Y0 / MM - 1.5, WELL_Y0 / MM + 1.0)) * ramp(r, 24.5, 27.5)
    panel = (z > 113.5) & (z < 116.2) & (r < 27.6)
    pts[:, 2] -= np.where(panel, w * 3.5 * MM, 0.0)
    endp.GetAttribute("points").Set(pts.astype(np.float32).tolist() if False else __import__("pxr").Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    nrm = endp.GetAttribute("normals")
    if nrm:
        endp.RemoveProperty("normals")
    # the tab: a dark anodised aluminium, so it shows on the lid
    for sh in ("tab_aluminium",):
        shader = st.GetPrimAtPath(f"/Can/Looks/{sh}/PreviewSurface")
        shader.GetAttribute("inputs:diffuseColor").Set(Gf.Vec3f(0.10, 0.10, 0.12))
        if shader.GetAttribute("inputs:roughness"):
            shader.GetAttribute("inputs:roughness").Set(0.35)
    # the can's colliders
    UsdGeom.Scope.Define(st, "/Can/collision")
    def cyl(path, r, z0, z1):
        c = UsdGeom.Cylinder.Define(st, path)
        c.CreateAxisAttr("Z"); c.CreateRadiusAttr(r); c.CreateHeightAttr(z1 - z0)
        c.AddTranslateOp().Set(Gf.Vec3d(0, 0, (z0 + z1) / 2))
        c.CreatePurposeAttr("guide"); collider(c.GetPrim())
    cyl("/Can/collision/body", BODY_R, 0.0, BODY_H)
    cyl("/Can/collision/neck", NECK_R, BODY_H, NECK_TOP)
    cyl("/Can/collision/floor", PANEL_R, NECK_TOP, PANEL_TOP - WELL_DEPTH)  # under the finger well
    disc = circle(PANEL_R)
    near = clip(disc, 0, 1, WELL_Y0)
    far_r = clip(clip(disc, 0, -1, -WELL_Y0), -1, 0, -WELL_X)
    far_l = clip(clip(disc, 0, -1, -WELL_Y0), 1, 0, -WELL_X)
    for k, poly in enumerate((near, far_r, far_l)):
        prism(st, f"/Can/collision/panel{k}", poly, PANEL_TOP - WELL_DEPTH, PANEL_TOP)
    rc, n = (RIM_R0 + RIM_R1) / 2, 32
    for k in range(n):
        a = 2 * math.pi * k / n
        b = UsdGeom.Cube.Define(st, f"/Can/collision/rim{k:02d}")
        b.CreateSizeAttr(1.0)
        b.AddTranslateOp().Set(Gf.Vec3d(rc * math.cos(a), rc * math.sin(a), (PANEL_TOP + RIM_TOP) / 2))
        b.AddRotateZOp().Set(math.degrees(a))
        b.AddScaleOp().Set(Gf.Vec3f(RIM_R1 - RIM_R0, 2 * math.pi * rc / n * 1.04, RIM_TOP - PANEL_TOP))
        b.CreatePurposeAttr("guide"); collider(b.GetPrim())

    # the tab
    UsdPhysics.RigidBodyAPI.Apply(tp)
    tp.AddAppliedSchema("PhysxRigidBodyAPI")
    tm = UsdPhysics.MassAPI.Apply(tp)
    tm.CreateMassAttr(TAB_MASS)
    tm.CreateCenterOfMassAttr(Gf.Vec3f(0, 11 * MM, 0.0))
    ti = TAB_MASS * 0.02**2 / 12
    tm.CreateDiagonalInertiaAttr(Gf.Vec3f(ti, ti, 2 * ti))
    tp.CreateAttribute("physxRigidBody:maxAngularVelocity", Sdf.ValueTypeNames.Float, custom=True).Set(300.0)
    tmat = material(st, "/Can/tab/Material", 0.8, 0.7)
    UsdShade.MaterialBindingAPI.Apply(tp).Bind(tmat, UsdShade.Tokens.weakerThanDescendants, "physics")
    UsdGeom.Scope.Define(st, "/Can/tab/collision")
    for name, (x0, x1, y0, y1) in RING.items():
        box(st, f"/Can/tab/collision/{name}", x0 * MM, x1 * MM, (y0 * MM - RIVET[1]), (y1 * MM - RIVET[1]), TAB_Z0 - RIVET[2], TAB_Z1 - RIVET[2])
    j = UsdPhysics.RevoluteJoint.Define(st, "/Can/tab_rivet")
    j.CreateAxisAttr("X")
    j.CreateBody0Rel().SetTargets([Sdf.Path("/Can")])
    j.CreateBody1Rel().SetTargets([Sdf.Path("/Can/tab")])
    j.CreateLocalPos0Attr(Gf.Vec3f(*RIVET))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLowerLimitAttr(0.0)
    j.CreateUpperLimitAttr(90.0)
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    d.CreateTypeAttr("force"); d.CreateStiffnessAttr(0.0); d.CreateDampingAttr(0.0); d.CreateTargetPositionAttr(0.0)
    st.GetRootLayer().Save()
    print("wrote", out)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
