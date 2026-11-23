"""Writes tasks/open-can-pull-tab/assets/can.usda: a standard 330 ml aluminium beverage can (66 mm across, 115 mm tall,
a 202 end: 52 mm across the chime) with a stay-on ring-pull tab riveted on its end, as a floating-base articulation:
the can (root) and the tab on a revolute joint about x at the rivet (``tab_rivet``, 0 shut to 90 degrees).

    python make_can.py <out.usda>

Modelled the way the real end is made: the chime (rim) stands 2.3 mm over the end panel; under the tab's ring the panel
has a finger well (the stay-on end's recess) so a fingertip can go under the ring's far bar; the tab is a ring (a
18.8 x 22.8 mm loop round a 11 x 13.5 mm hole) held 1.5 mm over the panel. Everything is rigid; the score line is
not modelled (tabkit.py stands in for its resistance). Frame: origin at the can's foot, z up; the tab's ring points +y.
"""

import math
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

MM = 0.001
BODY_R, BODY_H = 33.0 * MM, 100.0 * MM  # 66 mm can body
NECK_R = 29.5 * MM  # the shoulder's top, under the chime
PANEL_TOP = 112.7 * MM  # the end panel's upper face
CHIME_TOP = 115.0 * MM  # the can's height
CHIME_R0, CHIME_R1 = 23.8 * MM, 26.0 * MM  # the seam's inner and outer radius (52 mm across the 202 end)
WELL_DEPTH = 4.0 * MM  # the finger well under the ring
WELL_X, WELL_Y0, WELL_Y1 = 10.5 * MM, 4.0 * MM, 24.0 * MM  # its half-width, and where it runs in y (to the chime)
TAB_Z = PANEL_TOP + 1.5 * MM + 0.5 * MM  # the tab's mid-plane (the rivet): the ring's underside 1.5 mm over the panel
TAB_T = 1.0 * MM  # the ring's thickness (as modelled: a rolled sheet edge)
RING = {  # name: (x0, x1, y0, y1) in mm, in the tab frame (origin at the rivet, ring towards +y)
    "near": (-10.0, 10.0, -0.6, 4.5),
    "far": (-7.0, 7.0, 18.5, 22.5),
    "left": (-10.0, -6.0, 4.5, 18.5),
    "right": (6.0, 10.0, 4.5, 18.5),
}
CAN_MASS = 0.35  # kg: 330 ml of water and a 15 g can
TAB_MASS = 0.0007  # kg: about 0.7 g
ALU = (0.78, 0.79, 0.81)
TABC = (0.42, 0.44, 0.47)  # the tab, a darker aluminium so it shows on the end
RED = (0.72, 0.10, 0.10)


def material(stage, path, static, dynamic):
    m = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
    api.CreateStaticFrictionAttr(static)
    api.CreateDynamicFrictionAttr(dynamic)
    api.CreateRestitutionAttr(0.0)
    return m


def collider(prim, mesh=False):
    UsdPhysics.CollisionAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.0004)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float, custom=True).Set(0.0)
    if mesh:
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexDecomposition")


def cyl(stage, path, r, z0, z1, color, coll=True, visual=True):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateAxisAttr("Z")
    c.CreateRadiusAttr(r)
    c.CreateHeightAttr(z1 - z0)
    c.AddTranslateOp().Set(Gf.Vec3d(0, 0, (z0 + z1) / 2))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if coll:
        collider(c.GetPrim())
    if not visual:
        c.CreatePurposeAttr("guide")
    return c


def box(stage, path, x0, x1, y0, y1, z0, z1, color, coll=True):
    b = UsdGeom.Cube.Define(stage, path)
    b.CreateSizeAttr(1.0)
    b.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    b.AddScaleOp().Set(Gf.Vec3f(x1 - x0, y1 - y0, z1 - z0))
    b.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if coll:
        collider(b.GetPrim())
    return b


def extrusion(stage, path, poly, z0, z1, color, coll=True):
    """A prism: the polygon ``poly`` ([(x, y)] counter-clockwise) from z0 to z1, as a mesh."""
    n = len(poly)
    pts = [Gf.Vec3f(x, y, z0) for x, y in poly] + [Gf.Vec3f(x, y, z1) for x, y in poly]
    counts, idx = [n, n], [*range(n - 1, -1, -1), *range(n, 2 * n)]
    counts = [n, n] + [4] * n
    idx = list(range(n - 1, -1, -1)) + list(range(n, 2 * n))
    for i in range(n):
        j = (i + 1) % n
        idx += [i, j, n + j, n + i]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr(counts)
    m.CreateFaceVertexIndicesAttr(idx)
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if coll:
        collider(m.GetPrim(), mesh=True)
    return m


def circle(r, n=40):
    return [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def clip(poly, a, b, c):
    """Sutherland-Hodgman: the part of the polygon with a*x + b*y <= c."""
    out = []
    n = len(poly)
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        dp, dq = a * p[0] + b * p[1] - c, a * q[0] + b * q[1] - c
        if dp <= 0:
            out.append(p)
        if (dp < 0 < dq) or (dq < 0 < dp):
            t = dp / (dp - dq)
            out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
    return out


def panel_pieces(r, n=64):
    """The end panel as convex pieces: the disc of radius r with the finger well's strip (|x| < WELL_X, y > WELL_Y0) cut out."""
    disc = circle(r, n)
    near = clip(disc, 0, 1, WELL_Y0)  # y <= WELL_Y0
    far_r = clip(clip(disc, 0, -1, -WELL_Y0), -1, 0, -WELL_X)  # y >= WELL_Y0, x >= WELL_X
    far_l = clip(clip(disc, 0, -1, -WELL_Y0), 1, 0, -WELL_X)  # y >= WELL_Y0, x <= -WELL_X
    return [near, far_r, far_l]


def main(out):
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
    mass = UsdPhysics.MassAPI.Apply(p)
    mass.CreateMassAttr(CAN_MASS)
    mass.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0.055))
    I = CAN_MASS * (3 * BODY_R**2 + BODY_H**2) / 12
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(I, I, CAN_MASS * BODY_R**2 / 2))
    p.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool, custom=True).Set(False)
    p.CreateAttribute("physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int, custom=True).Set(32)
    p.CreateAttribute("physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, custom=True).Set(4)
    p.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float, custom=True).Set(0.0)
    p.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float, custom=True).Set(0.0)
    mat = material(st, "/Can/Material", 0.8, 0.7)
    UsdShade.MaterialBindingAPI.Apply(p).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    UsdGeom.Scope.Define(st, "/Can/visual")
    cyl(st, "/Can/visual/body", BODY_R, 0.0, BODY_H, RED, coll=False)
    cyl(st, "/Can/visual/neck", NECK_R, BODY_H, PANEL_TOP - WELL_DEPTH, ALU, coll=False)
    for k, poly in enumerate(panel_pieces(CHIME_R0 + 0.3 * MM)):
        extrusion(st, f"/Can/visual/panel{k}", poly, PANEL_TOP - WELL_DEPTH, PANEL_TOP, ALU, coll=False)
    extrusion(st, "/Can/visual/chime_up", circle(CHIME_R1), PANEL_TOP - 0.1 * MM, CHIME_TOP, ALU, coll=False)
    UsdGeom.Scope.Define(st, "/Can/collision")
    cyl(st, "/Can/collision/body", BODY_R, 0.0, BODY_H, RED, visual=False)
    cyl(st, "/Can/collision/neck", NECK_R, BODY_H, PANEL_TOP - WELL_DEPTH, ALU, visual=False)
    for k, poly in enumerate(panel_pieces(CHIME_R0 + 0.3 * MM)):
        extrusion(st, f"/Can/collision/panel{k}", poly, PANEL_TOP - WELL_DEPTH, PANEL_TOP, ALU)
    # the chime: a ring of 32 slabs
    n = 32
    rc = (CHIME_R0 + CHIME_R1) / 2
    for k in range(n):
        a = 2 * math.pi * k / n
        w = 2 * math.pi * rc / n * 1.04
        b = UsdGeom.Cube.Define(st, f"/Can/collision/chime{k:02d}")
        b.CreateSizeAttr(1.0)
        b.AddTranslateOp().Set(Gf.Vec3d(rc * math.cos(a), rc * math.sin(a), (PANEL_TOP + CHIME_TOP) / 2))
        b.AddRotateZOp().Set(math.degrees(a))
        b.AddScaleOp().Set(Gf.Vec3f(CHIME_R1 - CHIME_R0, w, CHIME_TOP - PANEL_TOP))
        b.CreatePurposeAttr("guide")
        collider(b.GetPrim())

    tab = UsdGeom.Xform.Define(st, "/Can/tab")
    tp = tab.GetPrim()
    tab.AddTranslateOp().Set(Gf.Vec3d(0, 0, TAB_Z))
    UsdPhysics.RigidBodyAPI.Apply(tp)
    tp.AddAppliedSchema("PhysxRigidBodyAPI")
    tm = UsdPhysics.MassAPI.Apply(tp)
    tm.CreateMassAttr(TAB_MASS)
    tm.CreateCenterOfMassAttr(Gf.Vec3f(0, 11 * MM, 0))
    ti = TAB_MASS * (0.02**2) / 12
    tm.CreateDiagonalInertiaAttr(Gf.Vec3f(ti, ti, ti * 2))
    tp.CreateAttribute("physxRigidBody:maxAngularVelocity", Sdf.ValueTypeNames.Float, custom=True).Set(300.0)
    tmat = material(st, "/Can/tab/Material", 0.8, 0.7)
    UsdShade.MaterialBindingAPI.Apply(tp).Bind(tmat, UsdShade.Tokens.weakerThanDescendants, "physics")
    UsdGeom.Scope.Define(st, "/Can/tab/visual")
    UsdGeom.Scope.Define(st, "/Can/tab/collision")
    for name, (x0, x1, y0, y1) in RING.items():
        box(st, f"/Can/tab/visual/{name}", x0 * MM, x1 * MM, y0 * MM, y1 * MM, -TAB_T / 2, TAB_T / 2, TABC, coll=False)
        b = box(st, f"/Can/tab/collision/{name}", x0 * MM, x1 * MM, y0 * MM, y1 * MM, -TAB_T / 2, TAB_T / 2, ALU)
        b.CreatePurposeAttr("guide")

    j = UsdPhysics.RevoluteJoint.Define(st, "/Can/tab_rivet")
    j.CreateAxisAttr("X")
    j.CreateBody0Rel().SetTargets([Sdf.Path("/Can")])
    j.CreateBody1Rel().SetTargets([Sdf.Path("/Can/tab")])
    j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, TAB_Z))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLowerLimitAttr(0.0)
    j.CreateUpperLimitAttr(90.0)
    jp = j.GetPrim()
    d = UsdPhysics.DriveAPI.Apply(jp, "angular")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(0.0)
    d.CreateDampingAttr(0.0)
    d.CreateTargetPositionAttr(0.0)
    st.GetRootLayer().Save()
    print("wrote", out)


if __name__ == "__main__":
    main(sys.argv[1])
