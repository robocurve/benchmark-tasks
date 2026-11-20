"""Write the scene's USD assets from geometry.py.

    python build_assets.py          # needs pxr (Isaac Sim's Python), not the simulator

- assets/frame.usda: the microscope's fixed frame (base, pillar, stage bracket, condenser, stage and focus knobs, the
  limb's front), kinematic. Origin at the optical axis on the stage top (M frame).
- assets/optics.usda: the nosepiece, the four objectives (4x in the path) and the binocular head, kinematic, its own
  body so that contacts with it can be read. Same origin.
- assets/stage.usda: the mechanical stage, an articulation with a fixed base: the fixed lower plate, the Y carriage
  (prismatic along x), the X carriage (prismatic along y: the upper plate with the template's back strip and fixed jaw),
  and the caliper (revolute about z: the curved arm, its tip, the lever link and knob). Same origin.
- assets/slide.usda: the prepared slide, one rigid body of three boxes (label end, specimen part, cover glass). Origin
  at its centre; x toward the label end, z out of the coverslip face.
- assets/holder.usda: the angled bench slide holder (15 slots), kinematic; colliders round the middle slot only.
  Origin at the middle slot's floor centre.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"
CREAM = (0.86, 0.85, 0.80)          # the stand's painted cast body
BLACK = (0.07, 0.07, 0.08)          # the stage plates, knobs
STEEL = (0.62, 0.63, 0.65)          # template, caliper
CHROME = (0.78, 0.79, 0.80)         # objectives
GLASS = (0.82, 0.90, 0.92)
FROST = (0.95, 0.95, 0.93)
COVER = (0.75, 0.85, 0.90)
STAIN = (0.18, 0.30, 0.70)          # the methylene blue spot
HOLDER = (0.93, 0.93, 0.92)         # white polyethylene
SLOTC = (0.30, 0.30, 0.32)


def stage_for(name: str):
    path = OUT / f"{name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    return stage, root


def material(stage, path: str, color, roughness=0.5, metallic=0.0, opacity=1.0):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    if opacity < 1.0:
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(mat)


def contact_offsets(prim, contact=0.0002, rest=0.0):
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(contact)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(rest)


def box(stage, path, size, center, mat=None, collide=True, visible=True, rot=None, offsets=None):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    if rot is not None:
        c.AddOrientOp().Set(Gf.Quatf(*rot))
    c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        if offsets is not None:
            contact_offsets(c.GetPrim(), *offsets)
    if not visible:
        c.CreatePurposeAttr(UsdGeom.Tokens.guide)
    return c


def span_box(stage, path, xr, yr, zr, mat=None, **kw):
    size = (xr[1] - xr[0], yr[1] - yr[0], zr[1] - zr[0])
    center = ((xr[0] + xr[1]) / 2, (yr[0] + yr[1]) / 2, (zr[0] + zr[1]) / 2)
    return box(stage, path, size, center, mat, **kw)


def cylinder(stage, path, radius, height, center, mat, axis="Z", collide=False, rot=None, offsets=None):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(float(radius))
    c.CreateHeightAttr(float(height))
    c.CreateAxisAttr(axis)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    if rot is not None:
        c.AddOrientOp().Set(Gf.Quatf(*rot))
    bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        if offsets is not None:
            contact_offsets(c.GetPrim(), *offsets)
    return c


def cone_mesh(stage, path, r0, r1, z0, z1, mat, n=20, collide=False):
    """A closed frustum along local z (radius r0 at z0, r1 at z1), convex hull collider when ``collide``."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    V = [(r0 * math.cos(t), r0 * math.sin(t), z0) for t in a] + [(r1 * math.cos(t), r1 * math.sin(t), z1) for t in a]
    V += [(0, 0, z0), (0, 0, z1)]
    F = []
    for i in range(n):
        j = (i + 1) % n
        F += [[i, j, n + j], [i, n + j, n + i], [2 * n, j, i], [2 * n + 1, n + i, n + j]]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in V])
    m.CreateFaceVertexCountsAttr([3] * len(F))
    m.CreateFaceVertexIndicesAttr([int(i) for f in F for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    bind(m, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
    return m


def quat_between(z_to) -> tuple:
    """(w, x, y, z) rotating +z onto the unit vector ``z_to``."""
    z = np.array([0.0, 0.0, 1.0])
    v = np.asarray(z_to, float) / np.linalg.norm(z_to)
    axis = np.cross(z, v)
    s = np.linalg.norm(axis)
    if s < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    ang = math.atan2(s, float(z @ v))
    axis /= s
    return (math.cos(ang / 2), *(axis * math.sin(ang / 2)))


def kinematic(prim):
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)


def xform(stage, path, pos=(0.0, 0.0, 0.0), rot=None):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    if rot is not None:
        xf.AddOrientOp().Set(Gf.Quatf(*rot))
    return xf


# -- the fixed frame ------------------------------------------------------------------------------------------------------
def build_frame():
    stage, root = stage_for("frame")
    kinematic(root.GetPrim())
    r = "/frame"
    cream = material(stage, f"{r}/Looks/cream", CREAM, roughness=0.5)
    black = material(stage, f"{r}/Looks/black", BLACK, roughness=0.45)
    chrome = material(stage, f"{r}/Looks/chrome", CHROME, roughness=0.25, metallic=0.8)
    span_box(stage, f"{r}/base", *g.BASE, cream)
    span_box(stage, f"{r}/pillar", *g.PILLAR, cream)
    span_box(stage, f"{r}/bracket", *g.BRACKET, cream)
    span_box(stage, f"{r}/limb", *g.LIMB, cream)
    rc, (z0, z1) = g.CONDENSER
    cylinder(stage, f"{r}/condenser", rc, z1 - z0, (0, 0, (z0 + z1) / 2), black, collide=True)
    (kx, ky), kr, (kz0, kz1) = g.XY_KNOBS
    cylinder(stage, f"{r}/xy_knobs", kr, kz1 - kz0, (kx, ky, (kz0 + kz1) / 2), black, collide=True)
    span_box(stage, f"{r}/xy_rack", (kx - 0.004, 0.040), (ky - 0.004, ky + 0.004), (-0.020, -0.012), black)
    (fx, fz), fr, (fy0, fy1) = g.FOCUS_KNOBS
    for s in (-1, 1):
        cylinder(stage, f"{r}/focus_{'lr'[s > 0]}", fr, fy1 - fy0, (fx, s * (fy0 + fy1) / 2, fz), black, axis="Y",
                 collide=True)
    # lamp window on the base under the condenser (visual)
    cylinder(stage, f"{r}/lamp", 0.016, 0.002, (0, 0, g.BASE[2][1] + 0.001), chrome)
    stage.Save()


# -- the optics: nosepiece, objectives, head ---------------------------------------------------------------------------
def build_optics():
    stage, root = stage_for("optics")
    kinematic(root.GetPrim())
    r = "/optics"
    cream = material(stage, f"{r}/Looks/cream", CREAM, roughness=0.5)
    black = material(stage, f"{r}/Looks/black", BLACK, roughness=0.4)
    chrome = material(stage, f"{r}/Looks/chrome", CHROME, roughness=0.2, metallic=0.85)
    red = material(stage, f"{r}/Looks/red", (0.75, 0.12, 0.10), roughness=0.4)
    # the 4x objective in the path (vertical, tip OBJ4_TIP above the stage top)
    t4 = g.OBJ4_TIP
    xf = xform(stage, f"{r}/obj4x", (0, 0, t4))
    cone_mesh(stage, f"{r}/obj4x/body", g.OBJ4_R[0], g.OBJ4_R[1], 0.0, g.OBJ4_LEN, chrome, collide=True)
    cylinder(stage, f"{r}/obj4x/band", g.OBJ4_R[1] + 0.0003, 0.002, (0, 0, g.OBJ4_LEN - 0.004), red)
    # the turret (tilted disc) and the three idle objectives, leaning back
    cx, cy, cz = g.TURRET_C
    tilt = g.TURRET_TILT
    q = quat_between((math.sin(tilt), 0.0, math.cos(tilt)))
    cylinder(stage, f"{r}/turret", g.TURRET_R, g.TURRET_T, (cx, cy, cz), black, collide=True, rot=q)
    cylinder(stage, f"{r}/turret_hub", 0.022, 0.030, (cx + 0.004, cy, cz + 0.020), cream, collide=True)
    sh0 = np.array([0.0, 0.0, t4 + g.OBJ4_LEN])          # the 4x's shoulder
    for k, (off, lean, az, length) in enumerate(g.OBJ_OTHERS):
        d = np.array([math.cos(az), math.sin(az), 0.0])   # +x = toward the back
        shoulder = sh0 + off * d + np.array([0, 0, 0.004 * k])
        axis = np.array([-math.sin(lean) * math.cos(az), -math.sin(lean) * math.sin(az), math.cos(lean)])  # tip outward
        xf = xform(stage, f"{r}/obj_{k}", tuple(shoulder - axis * length), quat_between(axis))
        cone_mesh(stage, f"{r}/obj_{k}/body", 0.0065, g.OBJ_OTHER_R, 0.0, length, chrome, collide=True)
    # the head: body over the limb, two eyepiece tubes up and toward the user (-x)
    span_box(stage, f"{r}/head", *g.HEAD, cream)
    er, ey, (ex0, ez0), etilt, elen = g.EYEPIECES
    axis = np.array([-math.sin(etilt), 0.0, math.cos(etilt)])
    for s in (-1, 1):
        c = np.array([ex0, s * ey, ez0]) + axis * elen / 2
        cylinder(stage, f"{r}/tube_{'lr'[s > 0]}", er, elen, tuple(c), black, collide=True, rot=quat_between(axis))
    stage.Save()


# -- the mechanical stage (articulation) -------------------------------------------------------------------------------
def body(stage, path, mass, pos=(0.0, 0.0, 0.0)):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    UsdPhysics.MassAPI.Apply(xf.GetPrim()).CreateMassAttr(mass)
    return xf


def joint(stage, kind, path, body0, body1, pos0, axis, limits):
    j = (UsdPhysics.RevoluteJoint if kind == "revolute" else UsdPhysics.PrismaticJoint).Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(float(limits[0]))
    j.CreateUpperLimitAttr(float(limits[1]))
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if kind == "revolute" else "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.0)
    drive.CreateTargetPositionAttr(0.0)
    return j


def build_stage():
    stage, root = stage_for("stage")
    r = "/stage"
    black = material(stage, f"{r}/Looks/black", BLACK, roughness=0.45)
    steel = material(stage, f"{r}/Looks/steel", STEEL, roughness=0.35, metallic=0.7)
    dark = material(stage, f"{r}/Looks/dark", (0.02, 0.02, 0.02), roughness=0.8)
    knob = material(stage, f"{r}/Looks/knob", (0.12, 0.12, 0.13), roughness=0.5, metallic=0.3)
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    root.GetPrim().AddAppliedSchema("PhysxArticulationAPI")
    root.GetPrim().CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    off = (0.0002, 0.0)
    # fixed lower plate
    lower = f"{r}/lower"
    body(stage, lower, 2.0)
    UsdPhysics.FixedJoint.Define(stage, f"{r}/joints/root_joint").CreateBody1Rel().SetTargets([Sdf.Path(lower)])
    zl = (-g.STAGE_UPPER_T - g.STAGE_LOWER_T, -g.STAGE_UPPER_T - 0.0002)
    span_box(stage, f"{lower}/plate", (g.STAGE_X[0] + 0.004, g.STAGE_X[1] - 0.004), (g.STAGE_Y[0] + 0.006, g.STAGE_Y[1]),
             zl, black)
    # Y carriage (moves along x): a guide under the right edge (small, visual + mass carrier)
    ycar = f"{r}/ycar"
    body(stage, ycar, 0.15)
    span_box(stage, f"{ycar}/guide", (g.STAGE_X[0] + 0.006, g.STAGE_X[1] - 0.006), (g.STAGE_Y[0], g.STAGE_Y[0] + 0.006),
             zl, knob, collide=False)
    joint(stage, "prismatic", f"{r}/joints/stage_y", lower, ycar, (0, 0, 0), "X", g.STAGE_TRAVEL[0])
    # X carriage (moves along y): the upper plate, the template's back strip and fixed jaw
    xcar = f"{r}/xcar"
    body(stage, xcar, 0.35)
    span_box(stage, f"{xcar}/plate", g.STAGE_X, g.STAGE_Y, (-g.STAGE_UPPER_T, 0.0), black, offsets=off)
    cylinder(stage, f"{xcar}/aperture", g.APERTURE_R, 0.0002, (0, 0, 0.0001), dark)
    zt = (0.0, g.TEMPLATE_T)
    span_box(stage, f"{xcar}/back_strip", *g.BACK_STRIP, zt, steel, offsets=off)
    span_box(stage, f"{xcar}/jaw", *g.JAW, zt, steel, offsets=off)
    # scale marks on the stage edge (visual)
    for i in range(-6, 7):
        span_box(stage, f"{xcar}/tick_{i + 6}", (g.STAGE_X[0] + 0.001, g.STAGE_X[0] + 0.004 + (0.002 if i % 5 == 0 else 0)),
                 (i * 0.005 - 0.0002, i * 0.005 + 0.0002), (0.0, 0.0002), steel, collide=False)
    joint(stage, "prismatic", f"{r}/joints/stage_x", ycar, xcar, (0, 0, 0), "Y", g.STAGE_TRAVEL[1])
    # the caliper: origin at its pivot (closed pose)
    px, py = g.PIVOT
    cal = f"{r}/caliper"
    body(stage, cal, g.CAL_MASS, (px, py, 0.0))

    def local(xr, yr):
        return (xr[0] - px, xr[1] - px), (yr[0] - py, yr[1] - py)

    span_box(stage, f"{cal}/arm", *local(*g.ARM_A), g.ARM_Z, steel, offsets=off)
    span_box(stage, f"{cal}/tip", *local(*g.ARM_TIP), g.ARM_Z, steel, offsets=off)
    span_box(stage, f"{cal}/lever", *local(*g.LEVER_LINK), g.ARM_Z, steel, offsets=off)
    cylinder(stage, f"{cal}/hub", g.HUB_R, g.ARM_Z[1] + 0.0012 - g.ARM_Z[0], (0, 0, (g.ARM_Z[0] + g.ARM_Z[1] + 0.0012) / 2),
             steel, collide=True, offsets=off)
    kx, ky = g.KNOB
    kz0, kz1 = g.KNOB_Z
    cylinder(stage, f"{cal}/knob", g.KNOB_R, kz1 - kz0, (kx - px, ky - py, (kz0 + kz1) / 2), knob, collide=True,
             offsets=(0.0005, 0.0))
    joint(stage, "revolute", f"{r}/joints/caliper", xcar, cal, (px, py, 0.0), "Z",
          (math.degrees(g.CAL_SHUT), math.degrees(g.CAL_OPEN)))
    stage.Save()


# -- the slide ------------------------------------------------------------------------------------------------------------
def build_slide():
    stage, root = stage_for("slide")
    p = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    UsdPhysics.MassAPI.Apply(p).CreateMassAttr(g.SLIDE_MASS)
    UsdPhysics.MassAPI(p).CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    p.AddAppliedSchema("PhysxRigidBodyAPI")
    p.CreateAttribute("physxRigidBody:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)
    r = "/slide"
    glass = material(stage, f"{r}/Looks/glass", GLASS, roughness=0.1, opacity=0.55)
    frost = material(stage, f"{r}/Looks/frost", FROST, roughness=0.9)
    cover = material(stage, f"{r}/Looks/cover", COVER, roughness=0.05, opacity=0.5)
    stain = material(stage, f"{r}/Looks/stain", STAIN, roughness=0.6)
    L, W, T = g.SLIDE_L, g.SLIDE_W, g.SLIDE_T
    lab = g.LABEL_L
    off = (0.0002, 0.0)
    box(stage, f"{r}/label_end", (lab, W, T), (L / 2 - lab / 2, 0, 0), frost, offsets=off)
    box(stage, f"{r}/specimen", (L - lab, W, T), (-lab / 2, 0, 0), glass, offsets=off)
    box(stage, f"{r}/coverslip", (g.COVER_S, g.COVER_S, g.COVER_T), (0, 0, T / 2 + g.COVER_T / 2), cover, offsets=off)
    cylinder(stage, f"{r}/spot", 0.0035, 0.00005, (0, 0, T / 2 + 0.00003), stain)
    # pencil label marks on the frosted end (visual)
    for i in range(3):
        box(stage, f"{r}/mark_{i}", (0.0012, 0.012 - 0.003 * i, 0.00005), (L / 2 - 0.006 - 0.003 * i, 0, T / 2 + 0.00003),
            material(stage, f"{r}/Looks/pencil", (0.25, 0.25, 0.27), roughness=0.8), collide=False)
    stage.Save()


# -- the bench slide holder --------------------------------------------------------------------------------------------
def build_holder():
    stage, root = stage_for("holder")
    kinematic(root.GetPrim())
    r = "/holder"
    white = material(stage, f"{r}/Looks/white", HOLDER, roughness=0.75)
    slotc = material(stage, f"{r}/Looks/slot", SLOTC, roughness=0.8)
    X, Y, Z = g.HOLDER_SIZE
    a = g.SLOT_LEAN
    ex = np.array([math.sin(a), 0.0, math.cos(a)])
    ez = np.array([-math.cos(a), 0.0, math.sin(a)])
    zfloor = Z - g.SLOT_DEPTH * math.cos(a)         # the floor centre's height above the table
    # origin: the middle slot's floor centre; the block below/around it (visual only, no collider near the slot)
    span_box(stage, f"{r}/block", (-X / 2, X / 2), (-Y / 2, Y / 2), (-zfloor, Z - zfloor), white, collide=False)
    # colliders: the block below the slot floor, and the slot's walls and floor in the slot frame
    span_box(stage, f"{r}/c_base", (-X / 2, X / 2), (-Y / 2, Y / 2), (-zfloor, -0.004), visible=False)
    R = np.column_stack([-ez, np.array([0, 1.0, 0]), ex])     # local x: toward +x (back), y, z: up the slot
    w = math.sqrt(max(1e-12, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    qx = (R[2, 1] - R[1, 2]) / (4 * w)
    qy = (R[0, 2] - R[2, 0]) / (4 * w)
    qz = (R[1, 0] - R[0, 1]) / (4 * w)
    rot = (w, qx, qy, qz)
    gap, sw = g.SLOT_GAP, g.SLOT_W
    th = 0.006
    depth = g.SLOT_DEPTH + 0.002
    for name, xr, yr, zr in (
        ("c_back", (gap / 2, gap / 2 + th), (-sw / 2 - th, sw / 2 + th), (-0.004, depth)),
        ("c_front", (-gap / 2 - th, -gap / 2), (-sw / 2 - th, sw / 2 + th), (-0.004, depth)),
        ("c_floor", (-gap / 2, gap / 2), (-sw / 2, sw / 2), (-0.004, 0.0)),
        ("c_left", (-gap / 2, gap / 2), (sw / 2, sw / 2 + th), (-0.004, depth)),
        ("c_right", (-gap / 2, gap / 2), (-sw / 2 - th, -sw / 2), (-0.004, depth)),
    ):
        size = np.array([xr[1] - xr[0], yr[1] - yr[0], zr[1] - zr[0]])
        cl = np.array([(xr[0] + xr[1]) / 2, (yr[0] + yr[1]) / 2, (zr[0] + zr[1]) / 2])
        box(stage, f"{r}/{name}", size, R @ cl, rot=rot, visible=False, offsets=(0.0005, 0.0))
    # the 15 slots drawn on the top (dark lines along y, leaning), and the slot mouths
    for k in range(g.SLOTS):
        dx = (k - g.SLOTS // 2) * g.SLOT_PITCH
        top = np.array([dx, 0.0, 0.0]) + ex * g.SLOT_DEPTH
        box(stage, f"{r}/slot_{k:02d}", (gap + 0.0006, sw, 0.0004), top + np.array([0, 0, 0.0002]), slotc,
            collide=False)
    stage.Save()


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    build_frame()
    build_optics()
    build_stage()
    build_slide()
    build_holder()
    for p in sorted(OUT.glob("*.usda")):
        print(p.name, p.stat().st_size, "bytes")
