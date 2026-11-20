"""Write the scene's USD assets and textures from geometry.py.

    python build_assets.py          # needs pxr, PIL and the DejaVu fonts (Isaac Sim's Python), not the simulator

- assets/oven.usda: the BLACK+DECKER TO1313SBD toaster oven, one articulation with a fixed base:
  - base: the case (stainless sides and roof, the black control column with its three knobs, the curved dark
    interior), the four heating elements (they collide), the crumb tray (visual);
  - door (revolute about y at its bottom edge): the black frame, the glass window and the handle bar on two posts;
  - rack (prismatic along x): the wire slide rack, 4 mm wires;
  - timer (revolute about x): the Toast/Timer knob with its grip ridge.
  The file's origin is the station's origin at table height (the spawn pose is (0, 0, TOP)), so every part is placed
  in station coordinates.
- assets/pan.usda: the bake pan, one rigid body (3 mm walls); origin under the middle of its front wall.
- assets/plate.usda: the paper plate, one rigid body: a 2 mm disc and its sloped rim as 36 thin plates; the visual
  rim is fluted; origin at the centre of its underside.
- assets/half.usda: one bagel half, a rigid ring of 24 convex segments, cut side up; origin at the centre of its
  underside.
- assets/textures/panel_3.png: the control panel's printing (the only texture; everything else is plain material).
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"
TEX = OUT / "textures"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf"
STEEL = (0.62, 0.63, 0.64)
BLACK = (0.03, 0.03, 0.035)
DARK = (0.10, 0.10, 0.11)
ENAMEL = (0.16, 0.15, 0.15)
GLASS = (0.05, 0.05, 0.06)
ELEMENT = (0.35, 0.33, 0.31)
RACK = (0.72, 0.72, 0.70)
PAN = (0.42, 0.42, 0.43)
PAPER = (1.0, 1.0, 0.985)
CRUST = (0.58, 0.34, 0.12)


# -- textures ------------------------------------------------------------------------------------------------------------
def panel_texture(path: Path) -> None:
    """The control column's front: black, white printing, seen from the front (from -x): image left is the robot's
    left (+y), image up is +z."""
    w_m, h_m = g.COLUMN_W, g.OVEN_H - g.FOOT_H
    ppm = 4000
    W, H = int(w_m * ppm), int(h_m * ppm)
    im = Image.new("RGB", (W, H), (10, 10, 11))
    d = ImageDraw.Draw(im)
    f_small = ImageFont.truetype(FONT, int(0.0042 * ppm))
    f_tiny = ImageFont.truetype(FONT, int(0.0034 * ppm))

    def px(y, z):   # panel point (y in the station frame, z above the table) -> image pixel
        return (g.COLUMN_Y1 - y) * ppm, (g.OVEN_H - z) * ppm

    cx, _ = px(g.KNOB_Y, 0)
    # Toast/Timer ring round the bottom knob: OFF at the left (9 o'clock, where the pointer rests), then clockwise
    # (as the robot sees it) the toast shades and the minutes to 30 (300 deg); STAY ON just anticlockwise of OFF
    zc = g.KNOB_Z[0]
    _, cz = px(0, zc)
    r_txt = (g.KNOB_R + 0.0085) * ppm

    def at(deg, r):            # image point at ``deg`` clockwise from OFF (OFF = 9 o'clock)
        a = math.radians(deg - 90.0)
        return cx + r * math.sin(a), cz - r * math.cos(a)

    for deg, label in ((0, "OFF"), (50, "5"), (100, "10"), (150, "15"), (200, "20"), (300, "30"), (-35, "ON")):
        d.text(at(deg, r_txt), label, font=f_tiny, fill=(235, 235, 230), anchor="mm")
    for deg in range(50, 301, 25):
        d.line([at(deg, (g.KNOB_R + 0.0015) * ppm), at(deg, (g.KNOB_R + 0.0035) * ppm)], fill=(235, 235, 230), width=4)
    for k in range(30):        # toast shades, light to dark, from 15 to 45 degrees
        shade = int(235 - 5 * k)
        d.line([at(15 + k, (g.KNOB_R + 0.0012) * ppm), at(15 + k, (g.KNOB_R + 0.0040) * ppm)],
               fill=(shade, int(shade * 0.78), int(shade * 0.45)), width=6)
    d.text((cx, px(0, zc - g.KNOB_R - 0.0065)[1]), "TOAST/TIMER", font=f_small, fill=(235, 235, 230), anchor="mm")
    for zk, label, marks in ((g.KNOB_Z[1], "FUNCTION", ("BAKE", "BROIL", "TOAST", "WARM")),
                             (g.KNOB_Z[2], "TEMPERATURE", ("WARM", "250", "350", "450"))):
        _, cz2 = px(0, zk)
        d.text((cx, px(0, zk - g.KNOB_R - 0.0065)[1]), label, font=f_small, fill=(235, 235, 230), anchor="mm")
        for i, m in enumerate(marks):
            a = math.radians(-90 + 60 * i)
            x, y = cx + r_txt * math.sin(a), cz2 - r_txt * math.cos(a) - 0.006 * ppm
            d.text((x, y), m, font=f_tiny, fill=(235, 235, 230), anchor="mm")
    # power light (off) and the brand line
    d.ellipse([cx - 14, px(0, 0.198)[1] - 14, cx + 14, px(0, 0.198)[1] + 14], fill=(60, 20, 18))
    im.save(path)


# -- USD helpers -------------------------------------------------------------------------------------------------------
def stage_for(name: str):
    path = OUT / f"{name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    return stage, root


def material(stage, path: str, color, roughness=0.5, texture: str | None = None, metallic=0.0, opacity=None,
             matte=False, emissive=None):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    if opacity is not None:
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    if matte:                                   # no specular highlight at all (uncoated paper)
        sh.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
        sh.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
    if emissive is not None:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
    if texture:
        st = UsdShade.Shader.Define(stage, f"{path}/st")
        st.CreateIdAttr("UsdPrimvarReader_float2")
        st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tx = UsdShade.Shader.Define(stage, f"{path}/texture")
        tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture)
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
        tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
    else:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(mat)


def offsets(prim, contact=0.002, rest=0.0):
    prim = prim.GetPrim()
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(contact)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(rest)


def box(stage, path, size, center, mat=None, collide=True, rot=None, off=(0.002, 0.0)):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    if rot is not None:
        c.AddOrientOp().Set(Gf.Quatf(*map(float, rot)))
    c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        if off is not None:
            offsets(c, *off)
    return c


def cylinder(stage, path, radius, length, center, axis, mat, collide=False, off=(0.002, 0.0)):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(float(radius))
    c.CreateHeightAttr(float(length))
    c.CreateAxisAttr(axis)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        if off is not None:
            offsets(c, *off)
    return c


def capsule(stage, path, radius, length, center, axis, mat, collide=True, off=(0.002, 0.0)):
    c = UsdGeom.Capsule.Define(stage, path)
    c.CreateRadiusAttr(float(radius))
    c.CreateHeightAttr(float(length))
    c.CreateAxisAttr(axis)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        if off is not None:
            offsets(c, *off)
    return c


def mesh(stage, path, V, F, mat=None, collide=False, off=(0.002, 0.0), uv=None, normals=None, smooth=False):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in V])
    m.CreateFaceVertexCountsAttr([len(f) for f in F])
    m.CreateFaceVertexIndicesAttr([int(i) for f in F for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDoubleSidedAttr(True)
    if uv is not None:
        UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                             UsdGeom.Tokens.vertex).Set([Gf.Vec2f(*map(float, t)) for t in uv])
    if normals is not None:
        m.CreateNormalsAttr([Gf.Vec3f(*map(float, n)) for n in normals])
        m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    if mat is not None:
        bind(m, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
        if off is not None:
            offsets(m, *off)
    return m


def quad(stage, path, corners, mat, uv=((0, 0), (1, 0), (1, 1), (0, 1))):
    """A textured quad from four corners (counterclockwise seen from its front)."""
    return mesh(stage, path, corners, [[0, 1, 2, 3]], mat, uv=uv)


def body(stage, path, mass, pos=(0.0, 0.0, 0.0), com=None, inertia=None):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    m = UsdPhysics.MassAPI.Apply(xf.GetPrim())
    m.CreateMassAttr(float(mass))
    if com is not None:
        m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
    if inertia is not None:
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
    return xf


def joint(stage, kind, path, body0, body1, pos0, axis, limits=None):
    j = (UsdPhysics.RevoluteJoint if kind == "revolute" else UsdPhysics.PrismaticJoint).Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    if limits is not None:
        j.CreateLowerLimitAttr(float(limits[0]))
        j.CreateUpperLimitAttr(float(limits[1]))
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if kind == "revolute" else "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.0)
    drive.CreateTargetPositionAttr(0.0)
    return j


def bx(stage, path, lo, hi, mat, collide=True):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return box(stage, path, hi - lo, (lo + hi) / 2, mat, collide)


# -- the oven ------------------------------------------------------------------------------------------------------------
def build_oven():
    stage, root = stage_for("oven")
    r = "/oven"
    steel = material(stage, f"{r}/Looks/steel", STEEL, roughness=0.30, metallic=0.85)
    black = material(stage, f"{r}/Looks/black", BLACK, roughness=0.45)
    enamel = material(stage, f"{r}/Looks/enamel", ENAMEL, roughness=0.55)
    glass = material(stage, f"{r}/Looks/glass", GLASS, roughness=0.05, opacity=0.15)
    element = material(stage, f"{r}/Looks/element", ELEMENT, roughness=0.5, metallic=0.6)
    rackm = material(stage, f"{r}/Looks/rack", RACK, roughness=0.3, metallic=0.9)
    panel = material(stage, f"{r}/Looks/panel", (1, 1, 1), roughness=0.5, texture="./textures/panel_3.png")
    knobm = material(stage, f"{r}/Looks/knob", (0.09, 0.09, 0.095), roughness=0.4)
    knobside = material(stage, f"{r}/Looks/knobside", (0.05, 0.05, 0.055), roughness=0.4)

    prim = root.GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxArticulationAPI")
    prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    prim.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    prim.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)

    X0, X1, Y0, Y1, H = g.OVEN_X, g.OVEN_X1, g.OVEN_Y0, g.OVEN_Y1, g.OVEN_H
    b = f"{r}/base"
    body(stage, b, g.OVEN_MASS - g.DOOR_MASS - g.RACK_MASS)
    UsdPhysics.FixedJoint.Define(stage, f"{r}/joints/root_joint").CreateBody1Rel().SetTargets([Sdf.Path(b)])
    F = g.FOOT_H
    # outer case: the control column (right), left side, roof, bottom, back; the front frame round the door
    dy0, dy1 = g.CAV_YC - g.DOOR_W / 2, g.CAV_YC + g.DOOR_W / 2
    door_top = g.HINGE_Z + g.DOOR_H
    bx(stage, f"{b}/column", (X0, Y0, F), (X1, g.CAV_Y0, H), black)
    bx(stage, f"{b}/left", (X0, g.CAV_Y1, F), (X1, Y1, H), steel)
    bx(stage, f"{b}/roof", (g.CAV_X0, g.CAV_Y0, g.CAV_Z1), (X1, g.CAV_Y1, H), steel)
    bx(stage, f"{b}/front_top", (X0, g.CAV_Y0, door_top), (g.CAV_X0, g.CAV_Y1, H), steel)
    bx(stage, f"{b}/bottom", (X0, g.CAV_Y0, F), (X1, g.CAV_Y1, g.CAV_Z0), steel)
    bx(stage, f"{b}/back", (g.CAV_X1, g.CAV_Y0, g.CAV_Z0), (X1, g.CAV_Y1, g.CAV_Z1), steel)
    # the dark interior (visual liners) and the crumb tray's front lip
    e = 0.0006
    bx(stage, f"{b}/in_floor", (g.CAV_X0, g.CAV_Y0, g.CAV_Z0), (g.CAV_X1, g.CAV_Y1, g.CAV_Z0 + e), enamel, False)
    bx(stage, f"{b}/in_roof", (g.CAV_X0, g.CAV_Y0, g.CAV_Z1 - e), (g.CAV_X1, g.CAV_Y1, g.CAV_Z1), enamel, False)
    bx(stage, f"{b}/in_back", (g.CAV_X1 - e, g.CAV_Y0, g.CAV_Z0), (g.CAV_X1, g.CAV_Y1, g.CAV_Z1), enamel, False)
    bx(stage, f"{b}/in_left", (g.CAV_X0, g.CAV_Y1 - e, g.CAV_Z0), (g.CAV_X1, g.CAV_Y1, g.CAV_Z1), enamel, False)
    bx(stage, f"{b}/in_right", (g.CAV_X0, g.CAV_Y0, g.CAV_Z0), (g.CAV_X1, g.CAV_Y0 + e, g.CAV_Z1), enamel, False)
    bx(stage, f"{b}/front_left", (X0, dy1, F), (g.CAV_X0, Y1, H), steel, False)
    bx(stage, f"{b}/front_right", (X0, g.CAV_Y0, F), (g.CAV_X0, dy0, H), steel, False)
    bx(stage, f"{b}/crumb_lip", (X0 - 0.002, dy0 + 0.01, F + 0.004), (X0, dy1 - 0.01, g.HINGE_Z - 0.008), black, False)
    for k, (fx, fy) in enumerate(((X0 + 0.02, Y0 + 0.02), (X0 + 0.02, Y1 - 0.02), (X1 - 0.02, Y0 + 0.02),
                                  (X1 - 0.02, Y1 - 0.02))):
        bx(stage, f"{b}/foot_{k}", (fx - 0.012, fy - 0.012, 0.0), (fx + 0.012, fy + 0.012, F), black, False)
    # heating elements, two over and two under the rack (along y; they collide)
    L = g.CAV_Y1 - g.CAV_Y0 - 0.02
    for k, (ex, ez) in enumerate(((g.CAV_X0 + 0.07, g.ELEM_Z_TOP), (g.CAV_X0 + 0.17, g.ELEM_Z_TOP),
                                  (g.CAV_X0 + 0.07, g.ELEM_Z_BOT), (g.CAV_X0 + 0.17, g.ELEM_Z_BOT))):
        capsule(stage, f"{b}/element_{k}", g.ELEM_R, L - 2 * g.ELEM_R, (ex, g.CAV_YC, ez), "Y", element)
    # the power cord (visual): out of the back near the column, along the table and down over its far edge, the plug
    # hanging below the table top, out of the cameras' view (the oven is never plugged in)
    cz = g.CORD_R
    cord_x0 = X1 + 0.004
    bx(stage, f"{b}/cord_boot", (X1, g.CORD_Y - 0.008, 0.018), (X1 + 0.012, g.CORD_Y + 0.008, 0.034), black, False)
    run = g.TABLE_X1 - cord_x0
    capsule(stage, f"{b}/cord_back", g.CORD_R, run, (cord_x0 + run / 2, g.CORD_Y, cz), "X", black, collide=False)
    capsule(stage, f"{b}/cord_rise", g.CORD_R, 0.020, (X1 + 0.006, g.CORD_Y, 0.014), "Z", black, collide=False)
    capsule(stage, f"{b}/cord_drop", g.CORD_R, 0.20, (g.TABLE_X1 + g.CORD_R, g.CORD_Y, -0.10), "Z", black,
            collide=False)
    bx(stage, f"{b}/plug", (g.TABLE_X1 - 0.008, g.CORD_Y - 0.018, -0.25), (g.TABLE_X1 + 0.018, g.CORD_Y + 0.018,
                                                                              -0.20), black, False)
    # the control panel's printing and the two fixed knobs (function and temperature, both at TOAST)
    quad(stage, f"{b}/panel_print", [(X0 - 0.0004, g.COLUMN_Y1, F), (X0 - 0.0004, Y0, F), (X0 - 0.0004, Y0, H),
                                      (X0 - 0.0004, g.COLUMN_Y1, H)], panel)
    for k, zk in enumerate(g.KNOB_Z[1:]):
        knob(stage, f"{b}/knob_{k + 1}", (X0, g.KNOB_Y, zk), knobm, knobside, turn=math.radians(120 if k == 0 else 60))

    # the door: origin on the hinge axis; at angle 0 its axes are the station's
    d = f"{r}/door"
    hx, hz = g.HINGE_X, g.HINGE_Z
    com = (-0.004, 0.0, g.DOOR_H * 0.48)
    body(stage, d, g.DOOR_MASS, (hx, g.CAV_YC, hz), com=com,
         inertia=(g.DOOR_MASS * (g.DOOR_H ** 2) / 3, g.DOOR_MASS * (g.DOOR_H ** 2) / 3,
                  g.DOOR_MASS * (g.DOOR_W ** 2) / 12))
    t2 = g.DOOR_T / 2
    wy, wz = g.WINDOW
    w0 = g.WINDOW_UP
    # frame round the window (black) and the glass (one collider for the whole slab)
    box(stage, f"{d}/slab", (g.DOOR_T, g.DOOR_W, g.DOOR_H), (0.0, 0.0, g.DOOR_H / 2), None, True)
    UsdGeom.Imageable(stage.GetPrimAtPath(f"{d}/slab")).CreatePurposeAttr(UsdGeom.Tokens.guide)
    side = (g.DOOR_W - wy) / 2
    box(stage, f"{d}/frame_l", (g.DOOR_T, side, g.DOOR_H), (0.0, wy / 2 + side / 2, g.DOOR_H / 2), black, False)
    box(stage, f"{d}/frame_r", (g.DOOR_T, side, g.DOOR_H), (0.0, -wy / 2 - side / 2, g.DOOR_H / 2), black, False)
    box(stage, f"{d}/frame_b", (g.DOOR_T, wy, w0), (0.0, 0.0, w0 / 2), black, False)
    top_h = g.DOOR_H - w0 - wz
    box(stage, f"{d}/frame_t", (g.DOOR_T, wy, top_h), (0.0, 0.0, g.DOOR_H - top_h / 2), black, False)
    box(stage, f"{d}/glass", (0.004, wy, wz), (0.0, 0.0, w0 + wz / 2), glass, False)
    # the handle bar on two posts
    bxp = -(t2 + g.BAR_PROUD)
    capsule(stage, f"{d}/bar", g.BAR_R, g.BAR_L - 2 * g.BAR_R, (bxp, 0.0, g.BAR_UP), "Y", black)
    for s in (-1, 1):
        box(stage, f"{d}/post_{'ab'[s > 0]}", (g.BAR_PROUD, 0.012, 0.014),
            (-(t2 + g.BAR_PROUD / 2), s * (g.BAR_L / 2 - 0.008), g.BAR_UP), black)
    joint(stage, "revolute", f"{r}/joints/door", b, d, (hx, g.CAV_YC, hz), "Y", (-math.degrees(g.DOOR_OPEN), 0.0))

    # the slide rack: origin at its front wire's centre, rack home
    k = f"{r}/rack"
    body(stage, k, g.RACK_MASS, (g.RACK_X0, g.CAV_YC, g.RACK_Z))
    W2, D = g.RACK_W / 2, g.RACK_D
    w = 2 * g.WIRE_R
    box(stage, f"{k}/front", (w, g.RACK_W, w), (0.0, 0.0, 0.0), rackm)
    box(stage, f"{k}/back", (w, g.RACK_W, w), (D, 0.0, 0.0), rackm)
    for s in (-1, 1):
        box(stage, f"{k}/side_{'ab'[s > 0]}", (D, w, w), (D / 2, s * (W2 - g.WIRE_R), 0.0), rackm)
    n = 11
    for i in range(n):
        y = -W2 + 0.02 + i * (g.RACK_W - 0.04) / (n - 1)
        box(stage, f"{k}/wire_{i:02d}", (D, w, w), (D / 2, y, 0.0), rackm)
    joint(stage, "prismatic", f"{r}/joints/rack", b, k, (g.RACK_X0, g.CAV_YC, g.RACK_Z), "X", (-g.RACK_OUT, 0.0))

    # the Toast/Timer knob: revolute about x; positive turns clockwise as the robot sees it
    t = f"{r}/timer"
    body(stage, t, g.KNOB_MASS, (g.OVEN_X, g.KNOB_Y, g.KNOB_Z[0]))
    knob(stage, f"{t}/knob", (0.0, 0.0, 0.0), knobm, knobside, collide=True)
    joint(stage, "revolute", f"{r}/joints/timer", b, t, (g.OVEN_X, g.KNOB_Y, g.KNOB_Z[0]), "X",
          (-30.0, math.degrees(g.TIMER_MAX)))
    stage.Save()


def knob(stage, path, at, face_mat, side_mat, turn=0.0, collide=True):
    """A 35 mm knob on the front face (axis along -x) with a grip ridge across it, its pointer at the ridge's +y end
    (at angle 0 the ridge is level and the pointer points to the robot's left); ``turn`` about x (rad)."""
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, at)))
    xf.AddRotateXOp().Set(math.degrees(turn))
    depth = g.KNOB_DEPTH - g.RIDGE[2]
    cylinder(stage, f"{path}/body", g.KNOB_R, depth, (-depth / 2, 0.0, 0.0), "X", side_mat, collide)
    rt, rl, rp = g.RIDGE          # thickness (z), length (y), proud (x)
    box(stage, f"{path}/ridge", (rp, rl, rt), (-depth - rp / 2, 0.0, 0.0), side_mat, collide)
    # the face print, facing -x (image up = +z, image left = +y)
    s = g.KNOB_R * 0.98
    xq = -depth - 0.0003
    quad(stage, f"{path}/face", [(xq, s, -s), (xq, -s, -s), (xq, -s, s), (xq, s, s)], face_mat)
    xl = -g.KNOB_DEPTH - 0.0003
    quad(stage, f"{path}/pointer", [(xl, rl / 2 - 0.001, -0.0015), (xl, 0.002, -0.0015), (xl, 0.002, 0.0015),
                                     (xl, rl / 2 - 0.001, 0.0015)],
         material(stage, f"{path}/Looks/line", (0.95, 0.95, 0.92), roughness=0.5))
    return xf


# -- the bake pan --------------------------------------------------------------------------------------------------------
def build_pan():
    stage, root = stage_for("pan")
    p = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    UsdPhysics.MassAPI.Apply(p).CreateMassAttr(g.PAN_MASS)
    r = "/pan"
    m = material(stage, f"{r}/Looks/steel", PAN, roughness=0.45, metallic=0.6)
    T, Wd, Dp, Hh = g.PAN_T, g.PAN_W, g.PAN_D, g.PAN_H
    # origin: centre of the underside; x from the front wall (x 0) to the back (x D)
    box(stage, f"{r}/bottom", (Dp, Wd, T), (Dp / 2, 0.0, T / 2), m)
    box(stage, f"{r}/front", (T, Wd, Hh), (T / 2, 0.0, Hh / 2), m)
    box(stage, f"{r}/back", (T, Wd, Hh), (Dp - T / 2, 0.0, Hh / 2), m)
    for s in (-1, 1):
        box(stage, f"{r}/side_{'ab'[s > 0]}", (Dp - 2 * T, T, Hh), (Dp / 2, s * (Wd / 2 - T / 2), Hh / 2), m)
    # rolled rim bead (visual)
    for s in (-1, 1):
        capsule(stage, f"{r}/bead_x{'ab'[s > 0]}", 0.0025, Dp - 0.005, (Dp / 2, s * (Wd / 2 - 0.0015), Hh), "X", m,
                collide=False)
        capsule(stage, f"{r}/bead_y{'ab'[s > 0]}", 0.0025, Wd - 0.005, (Dp / 2 + s * (Dp / 2 - 0.0015), 0.0, Hh), "Y",
                m, collide=False)
    stage.Save()


# -- the paper plate -----------------------------------------------------------------------------------------------------
def rim_z(r: float) -> float:
    if r <= g.PLATE_FLAT_R:
        return 0.0
    return g.PLATE_RIM_H * (r - g.PLATE_FLAT_R) / (g.PLATE_R - g.PLATE_FLAT_R)


def build_plate():
    stage, root = stage_for("plate")
    p = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    UsdPhysics.MassAPI.Apply(p).CreateMassAttr(g.PLATE_MASS)
    r = "/plate"
    m = material(stage, f"{r}/Looks/paper", PAPER, roughness=1.0, matte=True, emissive=(0.07, 0.07, 0.07))
    shade = material(stage, f"{r}/Looks/flute", (0.74, 0.74, 0.71), roughness=1.0, matte=True)
    T = g.PLATE_T
    # colliders: a 2 mm disc (the flat) and the sloped rim as 36 thin plates
    cylinder(stage, f"{r}/flat", g.PLATE_FLAT_R, T, (0.0, 0.0, T / 2), "Z", None, collide=True)
    UsdGeom.Imageable(stage.GetPrimAtPath(f"{r}/flat")).CreatePurposeAttr(UsdGeom.Tokens.guide)
    n = 36
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        V = []
        for (rr, zz) in ((g.PLATE_FLAT_R - 0.002, 0.0), (g.PLATE_R, g.PLATE_RIM_H)):
            for a in (a0, a1):
                V.append((rr * math.cos(a), rr * math.sin(a), zz))
                V.append((rr * math.cos(a), rr * math.sin(a), zz + T))
        F = [[0, 2, 6, 4], [1, 5, 7, 3], [0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6]]
        c = mesh(stage, f"{r}/rim_{k:02d}", V, F, None, collide=True)
        c.CreatePurposeAttr(UsdGeom.Tokens.guide)
    # visual: the flat (with an embossed ring at its edge) and a fluted rim with a rolled lip
    rings_r = np.concatenate([np.linspace(0.0, g.PLATE_FLAT_R - 0.004, 12), [g.PLATE_FLAT_R - 0.003,
                              g.PLATE_FLAT_R - 0.0015, g.PLATE_FLAT_R], np.linspace(g.PLATE_FLAT_R + 0.002,
                              g.PLATE_R - 0.004, 14), [g.PLATE_R - 0.002, g.PLATE_R]])
    na = g.PLATE_FLUTES * 6
    V, UV, N = [], [], []
    for rr in rings_r:
        for j in range(na):
            a = 2 * math.pi * j / na
            z = T + rim_z(rr)
            if g.PLATE_FLAT_R - 0.0035 < rr < g.PLATE_FLAT_R - 0.001:
                z -= 0.0006          # the embossed ring
            if rr > g.PLATE_FLAT_R + 0.001:
                frac = (rr - g.PLATE_FLAT_R) / (g.PLATE_R - g.PLATE_FLAT_R)
                z += 0.0025 * math.sin(g.PLATE_FLUTES * a) * min(1.0, 2.5 * frac) * (1 - 0.4 * frac)
            if rr >= g.PLATE_R - 0.0021:
                z += 0.0012       # the rolled lip
            V.append((rr * math.cos(a), rr * math.sin(a), z))
            UV.append((0.5 + 0.5 * rr / g.PLATE_R * math.cos(a), 0.5 + 0.5 * rr / g.PLATE_R * math.sin(a)))
    Fv = []
    for i in range(len(rings_r) - 1):
        for j in range(na):
            a, b2 = i * na + j, i * na + (j + 1) % na
            Fv.append([a, b2, b2 + na, a + na])
    mv = mesh(stage, f"{r}/visual", V, Fv, m, uv=UV)
    # smooth normals from the faces
    Vn = np.asarray(V)
    acc = np.zeros_like(Vn)
    for f in Fv:
        nrm = np.cross(Vn[f[1]] - Vn[f[0]], Vn[f[3]] - Vn[f[0]])
        for i in f:
            acc[i] += nrm
    acc /= np.linalg.norm(acc, axis=1, keepdims=True) + 1e-12
    mv.CreateNormalsAttr([Gf.Vec3f(*map(float, v)) for v in acc])
    mv.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    # the flutes' shade: a thin strip down each valley of the rim (geometry, so no texture is needed)
    for k in range(g.PLATE_FLUTES):
        a = 2 * math.pi * (k + 0.75) / g.PLATE_FLUTES
        w = 0.40 * 2 * math.pi / g.PLATE_FLUTES
        pts = []
        for rr in (g.PLATE_FLAT_R + 0.004, g.PLATE_R - 0.003):
            for da in (-w / 2, w / 2):
                z = T + rim_z(rr) + 0.0003 - 0.0025 * min(1.0, 2.5 * (rr - g.PLATE_FLAT_R) / (g.PLATE_R - g.PLATE_FLAT_R)) * (1 - 0.4 * (rr - g.PLATE_FLAT_R) / (g.PLATE_R - g.PLATE_FLAT_R))
                pts.append(((rr) * math.cos(a + da), (rr) * math.sin(a + da), z))
        mesh(stage, f"{r}/flute_{k:02d}", [pts[0], pts[1], pts[3], pts[2]], [[0, 1, 2, 3]], shade)
    # underside (visual)
    mesh(stage, f"{r}/under", [(g.PLATE_FLAT_R * math.cos(a), g.PLATE_FLAT_R * math.sin(a), 0.0002)
                               for a in np.linspace(0, 2 * math.pi, 64, endpoint=False)],
         [list(range(64))[::-1]], m)
    stage.Save()


# -- a bagel half ----------------------------------------------------------------------------------------------------------
def build_half():
    stage, root = stage_for("half")
    p = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    UsdPhysics.MassAPI.Apply(p).CreateMassAttr(g.HALF_MASS)
    r = "/half"
    crust = material(stage, f"{r}/Looks/crust", CRUST, roughness=0.8)
    cut = material(stage, f"{r}/Looks/cut", (0.80, 0.65, 0.43), roughness=0.9)
    ring = material(stage, f"{r}/Looks/ring", (0.66, 0.42, 0.18), roughness=0.85)
    R, Ri, T = g.BAGEL_R, g.HOLE_R, g.HALF_T
    n = g.HALF_SEGS
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        V = []
        for z in (0.0, T):
            for rr, a in ((Ri, a0), (R, a0), (R, a1), (Ri, a1)):
                V.append((rr * math.cos(a) / math.cos(math.pi / n) if rr == R else rr * math.cos(a),
                          rr * math.sin(a) / math.cos(math.pi / n) if rr == R else rr * math.sin(a), z))
        F = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
        c = mesh(stage, f"{r}/seg_{k:02d}", V, F, None, collide=True, off=(0.002, 0.0))
        c.CreatePurposeAttr(UsdGeom.Tokens.guide)
    # visual: crust round the outside and underneath (rounded lower edge), crumb on the cut face (top)
    na = 96
    prof = []                     # (r, z) round the outer side from the cut face down to the underside
    fr = 0.006
    for i in range(0, 7):
        prof.append((R, T - 0.0005 * i / 6))
    for i in range(0, 9):
        t = (math.pi / 2) * i / 8
        prof.append((R - fr + fr * math.cos(t), fr - fr * math.sin(t)))
    prof.append((Ri + 0.004, 0.0))
    prof.append((Ri, 0.004))
    prof.append((Ri, T))
    V, Fv = [], []
    for (rr, z) in prof:
        for j in range(na):
            a = 2 * math.pi * j / na
            V.append((rr * math.cos(a), rr * math.sin(a), z))
    for i in range(len(prof) - 1):
        for j in range(na):
            a, b2 = i * na + j, i * na + (j + 1) % na
            Fv.append([a, a + na, b2 + na, b2])
    mesh(stage, f"{r}/crust", V, Fv, crust)
    V, UV, Fv = [], [], []
    for rr in (Ri, R):
        for j in range(na):
            a = 2 * math.pi * j / na
            V.append((rr * math.cos(a), rr * math.sin(a), T + 0.0002))
            UV.append((0.5 + 0.5 * rr / R * math.cos(a), 0.5 + 0.5 * rr / R * math.sin(a)))
    for j in range(na):
        Fv.append([j, (j + 1) % na, na + (j + 1) % na, na + j])
    mesh(stage, f"{r}/cut", V, Fv, cut, uv=UV)
    Vr, Fr = [], []
    for rr in (R - 0.0045, R):
        for j in range(na):
            a = 2 * math.pi * j / na
            Vr.append((rr * math.cos(a), rr * math.sin(a), T + 0.0004))
    for j in range(na):
        Fr.append([j, (j + 1) % na, na + (j + 1) % na, na + j])
    mesh(stage, f"{r}/cut_edge", Vr, Fr, ring)
    stage.Save()


if __name__ == "__main__":
    TEX.mkdir(parents=True, exist_ok=True)
    panel_texture(TEX / "panel_3.png")
    build_oven()
    build_pan()
    build_plate()
    build_half()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
