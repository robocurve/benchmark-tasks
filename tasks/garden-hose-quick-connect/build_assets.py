"""Write the scene's USD assets from geometry.py.

    python build_assets.py          # needs pxr (Isaac Sim's Python), not the simulator

- assets/coupler.usda: the tap-side quick-connect coupler, an articulation fixed where it is spawned (task.py fixes its
  root): the brass ``body`` (its round bore built from 16 wall segments, the spout and the tap as visuals), the sprung
  ``collar`` on the ``collar`` prismatic joint along the bore, and three locking balls on radial prismatic joints
  ``pin0``..``pin2``. task.py's monitor turns the collar's position into the lock (see README).
- assets/hose.usda: the hose, an articulation fixed at its table clamp: 12 links of 23 mm hose on alternating yaw and
  pitch joints ``j0``..``j11`` (soft springs), and the brass plug on ``j12``: a round 15.5 mm shaft with an 11 mm
  groove and rounded nose, a flange, and a hex body.

Each asset's origin: the coupler's at the centre of its mouth (x into the bore); the hose's at its clamp (x along it).
"""

from __future__ import annotations

import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"

BRASS = (0.78, 0.60, 0.26)
STEEL = (0.70, 0.72, 0.75)
BLACK = (0.10, 0.10, 0.11)
GREEN = (0.10, 0.45, 0.20)
GREY = (0.45, 0.46, 0.48)
RED = (0.70, 0.08, 0.06)


def stage_for(name: str):
    path = OUT / f"{name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    return stage


def material(stage, path, color, roughness=0.5, metallic=0.0):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(mat)


def collide(prim, offset=0.0008):
    p = prim.GetPrim()
    UsdPhysics.CollisionAPI.Apply(p)
    p.AddAppliedSchema("PhysxCollisionAPI")
    p.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(offset)
    p.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)


def body(stage, path, mass, pos=(0, 0, 0), rot_x_deg=0.0, root=False, inertia=None, rot_z_deg=0.0):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if rot_z_deg:
        xf.AddRotateZOp().Set(rot_z_deg)
    if rot_x_deg:
        xf.AddRotateXOp().Set(rot_x_deg)
    p = xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    m = UsdPhysics.MassAPI.Apply(p)
    m.CreateMassAttr(mass)
    if inertia is not None:
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia, inertia, inertia))
    if root:
        UsdPhysics.ArticulationRootAPI.Apply(p)
        p.AddAppliedSchema("PhysxArticulationAPI")
        p.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
        p.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
        p.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    return xf


def box(stage, path, centre, size, mat, rot_x_deg=0.0, coll=False, visible=True):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*centre))
    if rot_x_deg:
        xf.AddRotateXOp().Set(rot_x_deg)
    c = UsdGeom.Cube.Define(stage, f"{path}/shape")
    c.CreateSizeAttr(1.0)
    c.AddScaleOp().Set(Gf.Vec3f(*size))
    bind(c, mat)
    if coll:
        collide(c)
    if not visible:
        UsdGeom.Imageable(c).MakeInvisible()
    return xf


def cylinder_x(stage, path, x0, x1, radius, mat, coll=False):
    """A solid cylinder along x from x0 to x1."""
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(x1 - x0)
    c.CreateAxisAttr("X")
    c.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) / 2, 0, 0))
    bind(c, mat)
    if coll:
        collide(c)
    return c


def hex_x(stage, path, x0, x1, flat, mat):
    """A hexagonal prism along x from x0 to x1, flats (``flat`` from the axis) facing +-y: the plug's spanner hex,
    which the fingers take on two flats."""
    r = flat / math.cos(math.pi / 6)
    pts = [(r * math.cos(math.pi / 6 + k * math.pi / 3), r * math.sin(math.pi / 6 + k * math.pi / 3)) for k in range(6)]
    pts = [(z, y) for y, z in pts]                     # rotate so two flats face +-y
    verts = [Gf.Vec3f(x, y, z) for x in (x0, x1) for y, z in pts]
    faces = [list(reversed(range(6))), list(range(6, 12))] + [[k, (k + 1) % 6, (k + 1) % 6 + 6, k + 6] for k in range(6)]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(verts)
    m.CreateFaceVertexCountsAttr([len(f) for f in faces])
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    bind(m, mat)
    collide(m)
    UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
    p = m.GetPrim()                                     # pinched on two flats, it resists twisting over the patch
    p.CreateAttribute("physxCollision:torsionalPatchRadius", Sdf.ValueTypeNames.Float).Set(0.008)
    p.CreateAttribute("physxCollision:minTorsionalPatchRadius", Sdf.ValueTypeNames.Float).Set(0.006)
    return m


def ring_x(stage, path, x0, x1, r_in, r_out, mat, segments=16, coll=True):
    """A hollow ring along x from x0 to x1, as overlapping boxes round the axis."""
    width = 2 * r_out * math.tan(math.pi / segments) * 1.08
    for i in range(segments):
        a = 360.0 * i / segments
        r = (r_in + r_out) / 2
        # a box whose local y points radially at angle a (about x, from +y toward +z)
        box(stage, f"{path}/seg{i:02d}", ((x0 + x1) / 2, r * math.cos(math.radians(a)), r * math.sin(math.radians(a))),
            (x1 - x0, r_out - r_in, width), mat, rot_x_deg=a, coll=coll)


def joint(stage, path, kind, body0, body1, axis, pos0=(0, 0, 0), rot0_x_deg=0.0, pos1=(0, 0, 0), limits=(0.0, 0.0),
          rot0_z_deg=0.0):
    j = (UsdPhysics.PrismaticJoint if kind == "linear" else UsdPhysics.RevoluteJoint).Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    h = math.radians(rot0_x_deg) / 2
    hz = math.radians(rot0_z_deg) / 2
    q = Gf.Quatf(math.cos(hz), 0, 0, math.sin(hz)) * Gf.Quatf(math.cos(h), math.sin(h), 0, 0)
    j.CreateLocalRot0Attr(q)
    j.CreateLocalPos1Attr(Gf.Vec3f(*pos1))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), kind)       # gains come from task.py's actuators
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(0.0)
    return j


# -- the coupler -------------------------------------------------------------------------------------------------------
def build_coupler():
    stage = stage_for("coupler")
    r = "/coupler"
    brass = material(stage, f"{r}/Looks/brass", BRASS, 0.3, 0.9)
    steel = material(stage, f"{r}/Looks/steel", STEEL, 0.3, 0.9)
    black = material(stage, f"{r}/Looks/black", BLACK, 0.5, 0.3)
    red = material(stage, f"{r}/Looks/red", RED, 0.5)

    body(stage, f"{r}/body", g.BODY_MASS, root=True)
    # greased: low friction (the lower of the two in a contact counts) for the steel balls and the wet brass bore, so the
    # plug's cone lifts the balls and its shaft slides in the bore
    greased = UsdShade.Material.Define(stage, f"{r}/Looks/greased")
    gp = greased.GetPrim()
    gm = UsdPhysics.MaterialAPI.Apply(gp)
    gm.CreateStaticFrictionAttr(g.BALL_FRICTION)
    gm.CreateDynamicFrictionAttr(g.BALL_FRICTION)
    gm.CreateRestitutionAttr(0.0)
    gp.AddAppliedSchema("PhysxMaterialAPI")
    gp.CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set("min")

    # the bore's wall (collides with the plug) inside the brass body (visual)
    x0 = 0.0
    for k, (x1, r_in) in enumerate(g.LEAD_IN):          # the chamfered mouth, as steps down to the bore
        ring_x(stage, f"{r}/body/lead_in{k}", x0, x1, r_in, g.BODY_R - 0.001, brass)
        x0 = x1
    ring_x(stage, f"{r}/body/bore", x0, g.BODY_LEN, g.BORE_R, g.BORE_R + 0.003, brass)
    for prim in stage.Traverse():                       # the bore and its mouth are wet brass
        path = str(prim.GetPath())
        if prim.HasAPI(UsdPhysics.CollisionAPI) and ("/body/bore/" in path or "/body/lead_in" in path):
            UsdShade.MaterialBindingAPI.Apply(prim)
            UsdShade.MaterialBindingAPI(prim).Bind(greased, materialPurpose="physics")
    ring_x(stage, f"{r}/body/shell", g.LEAD_IN[-1][0], g.BODY_LEN, g.BORE_R + 0.003, g.BODY_R, brass, coll=False)
    cylinder_x(stage, f"{r}/body/back", g.BODY_LEN - 0.004, g.BODY_LEN, g.BODY_R, brass)
    # the spout and the tap behind it, back to the post (visual)
    cylinder_x(stage, f"{r}/body/hex", g.BODY_LEN, g.BODY_LEN + 0.012, g.BODY_R + 0.002, brass)
    cylinder_x(stage, f"{r}/body/spout", g.BODY_LEN + 0.012, g.POST_BACK - g.POST_SIZE[0] / 2 + 0.005,
               0.010, brass)
    tap_x = g.BODY_LEN + 0.030
    box(stage, f"{r}/body/tap", (tap_x, 0, 0.004), (0.026, 0.030, 0.034), brass)
    box(stage, f"{r}/body/stem", (tap_x, 0, 0.030), (0.008, 0.008, 0.022), steel)
    box(stage, f"{r}/body/handle_a", (tap_x, 0, 0.043), (0.008, 0.050, 0.006), red)
    box(stage, f"{r}/body/handle_b", (tap_x, 0, 0.043), (0.050, 0.008, 0.006), red)

    # the collar: a knurled black sleeve over the front of the body, sliding back toward the tap
    body(stage, f"{r}/collar", g.COLLAR_MASS)
    ring_x(stage, f"{r}/collar/sleeve", g.COLLAR_X0, g.COLLAR_X0 + g.COLLAR_LEN, g.COLLAR_R_IN, g.COLLAR_R_OUT, black,
           segments=20)
    joint(stage, f"{r}/joints/collar", "linear", f"{r}/body", f"{r}/collar", "X", limits=(0.0, g.COLLAR_TRAVEL))

    # the locking balls, each on a radial joint (its frame's y is outward)
    for i, a in enumerate(g.PIN_ANGLES):
        body(stage, f"{r}/pin{i}", g.PIN_MASS, pos=(g.PIN_X, 0, 0), rot_x_deg=a)
        ball = UsdGeom.Sphere.Define(stage, f"{r}/pin{i}/ball")
        ball.CreateRadiusAttr(g.PIN_BALL)
        ball.AddTranslateOp().Set(Gf.Vec3d(0, g.PIN_IN + g.PIN_BALL, 0))
        bind(ball, steel)
        collide(ball)
        UsdShade.MaterialBindingAPI(ball.GetPrim()).Bind(greased, materialPurpose="physics")
        joint(stage, f"{r}/joints/pin{i}", "linear", f"{r}/body", f"{r}/pin{i}", "Y", pos0=(g.PIN_X, 0, 0),
              rot0_x_deg=a, limits=(0.0, g.PIN_TRAVEL))
    stage.GetRootLayer().Save()


# -- the hose ----------------------------------------------------------------------------------------------------------
def build_hose():
    stage = stage_for("hose")
    r = "/hose"
    green = material(stage, f"{r}/Looks/hose", GREEN, 0.6)
    grey = material(stage, f"{r}/Looks/clamp", GREY, 0.4, 0.6)
    brass = material(stage, f"{r}/Looks/brass", BRASS, 0.3, 0.9)
    black = material(stage, f"{r}/Looks/boot", BLACK, 0.7)

    body(stage, f"{r}/clamp", 0.3, root=True)
    box(stage, f"{r}/clamp/block", (-0.01, 0, 0.0), (0.03, 0.05, 2 * g.HOSE_R + 0.008), grey, coll=True)
    # The hose is laid in a gentle curve: each yaw joint turns HOSE_BEND at rest (its zero), so the links are placed
    # along the curve and the joints' springs keep that shape.
    prev, at = f"{r}/clamp", (0.005, 0.0, 0.0)
    L, R = g.HOSE_LINK_L, g.HOSE_R
    heading, px, py = 0.0, 0.005, 0.0
    for i in range(g.HOSE_LINKS):
        link = f"{r}/h{i:02d}"
        axis = "Z" if i % 2 == 0 else "Y"
        bend = g.HOSE_BEND if axis == "Z" else 0.0
        heading += bend
        body(stage, link, g.HOSE_LINK_MASS, pos=(px, py, 0), rot_z_deg=math.degrees(heading))
        cap = UsdGeom.Capsule.Define(stage, f"{link}/tube")
        cap.CreateRadiusAttr(R)
        cap.CreateHeightAttr(L - 2 * R + 0.004)
        cap.CreateAxisAttr("X")
        cap.AddTranslateOp().Set(Gf.Vec3d(L / 2, 0, 0))
        bind(cap, green)
        collide(cap)
        lim = 60.0 if axis == "Z" else 45.0
        joint(stage, f"{r}/joints/j{i}", "angular", prev, link, axis, pos0=at, limits=(-lim, lim),
              rot0_z_deg=math.degrees(bend))
        prev, at = link, (L, 0, 0)
        px, py = px + L * math.cos(heading), py + L * math.sin(heading)

    # the plug: its frame at the flange face, x toward the nose
    back = g.FLANGE_LEN + g.PLUG_BODY_LEN + g.BOOT_LEN
    plug = f"{r}/plug"
    body(stage, plug, g.PLUG_MASS, pos=(px + back * math.cos(heading), py + back * math.sin(heading), 0),
         rot_z_deg=math.degrees(heading))
    s0, s1 = g.GROOVE
    cylinder_x(stage, f"{plug}/shaft_a", 0.0, s0, g.SHAFT_R, brass, coll=True)
    cylinder_x(stage, f"{plug}/groove", s0, s1, g.GROOVE_R, brass, coll=True)
    cylinder_x(stage, f"{plug}/shaft_b", s1, g.SHAFT_LEN - g.NOSE_LEN, g.SHAFT_R, brass, coll=True)
    nose = UsdGeom.Cone.Define(stage, f"{plug}/nose")                    # apex toward +x
    nose.CreateRadiusAttr(g.SHAFT_R)
    nose.CreateHeightAttr(g.NOSE_LEN)
    nose.CreateAxisAttr("X")
    nose.AddTranslateOp().Set(Gf.Vec3d(g.SHAFT_LEN - g.NOSE_LEN / 2, 0, 0))
    bind(nose, brass)
    collide(nose)
    cylinder_x(stage, f"{plug}/flange", -g.FLANGE_LEN, 0.0, g.FLANGE_R, brass, coll=True)
    hex_x(stage, f"{plug}/body", -g.FLANGE_LEN - g.PLUG_BODY_LEN, -g.FLANGE_LEN, g.PLUG_BODY_R, brass)
    cylinder_x(stage, f"{plug}/boot", -back, -g.FLANGE_LEN - g.PLUG_BODY_LEN, 0.0105, black, coll=True)
    joint(stage, f"{r}/joints/j{g.HOSE_LINKS}", "angular", prev, plug, "Z", pos0=(L, 0, 0), pos1=(-back, 0, 0),
          limits=(-60.0, 60.0))
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    build_coupler()
    build_hose()
    for p in sorted(OUT.rglob("*.usda")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
