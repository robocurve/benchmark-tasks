"""Write the scene's USD asset and texture atlas from geometry.py.

    python build_assets.py          # needs pxr, PIL and the DejaVu fonts (Isaac Sim's Python), not the simulator

- assets/panel.usda: one fixed-base articulation, origin on the table top at the station frame's origin:
  - base (fixed): the plywood wall with its sill and shelf brackets, the load center's steel box, its cover (the dead
    front, with the breakers' faces, ON / OFF marks and circuit numbers as a texture) and its door, fixed open flat
    with the circuit directory on its inside;
  - h1 .. h8 (revolute about a vertical axis): the breakers' handles, black, +-25 degrees, + toward the middle (ON);
  - loop (prismatic along space 4's handle), tie_a and tie (revolute about the handle's axis, then about the
    horizontal axis across it), tag_a and tag (revolute about the horizontal axis at the grommet, then about the
    tag's length): the nylon tie and the Master Lock 497A tag. Only the handles, the tag and the base collide; the
    handles do not collide with the base (they pass through the dead front's slots).
- assets/textures/bt_atlas_1.png: the dead front, the tag's two faces and the circuit directory.
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
FONT_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_C = "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf"
FONT_R = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
OFF_SMALL = (0.0005, 0.0)           # contact and rest offsets: handles and the tag
OFF_BIG = (0.001, 0.0)


def lin(srgb):
    """sRGB 0-255 to linear 0-1 (constant material colours; textures are sRGB files)."""
    out = []
    for c in srgb:
        c = c / 255.0
        out.append(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4)
    return tuple(out)


WALL = (236, 234, 228)              # painted wall white
PINE = (214, 182, 134)
STEEL = (120, 122, 126)             # shelf brackets
BOX_GREY = (170, 172, 172)          # ANSI 49 grey enamel
HANDLE = (22, 22, 24)
BRASS = (196, 160, 82)
TIE = (232, 232, 226)               # natural nylon
RED = (205, 22, 30)
BLACK = (18, 18, 20)
WHITE = (246, 246, 242)

# -- textures ------------------------------------------------------------------------------------------------------------
ATLAS = "bt_atlas_2.png"
ATLAS_PX = 2048
REGIONS = {"front": (0, 0, 1024, 1434), "tag_front": (1024, 0, 512, 946), "tag_back": (1536, 0, 512, 946),
           "directory": (1024, 946, 860, 1024)}


def _font(path, px):
    return ImageFont.truetype(path, max(6, int(px)))


def front(path: Path) -> None:
    """The cover's face seen from the front (image left = +y): grey enamel, the dead-front opening with the eight
    breakers' black faces, ON toward the middle and OFF toward the sides on each, the circuit numbers beside them."""
    W, Z0, Z1 = g.COVER_W, g.COVER_Z[0], g.COVER_Z[1]
    w = 1024
    ppm = w / W
    h = round((Z1 - Z0) * ppm)
    im = Image.new("RGB", (w, h), BOX_GREY)
    d = ImageDraw.Draw(im)

    def px(y, z):
        return ((W / 2 - (y - g.PANEL_Y)) * ppm, (Z1 - z) * ppm)

    # the opening (darker recess) and the breakers' faces
    x0, y0 = px(g.PANEL_Y + 0.070, g.ROWS[0] + 0.020)
    x1, y1 = px(g.PANEL_Y - 0.070, g.ROWS[-1] - 0.020)
    d.rectangle([x0, y0, x1, y1], fill=(60, 60, 62))
    f_on = _font(FONT_C, 0.0042 * ppm)
    f_num = _font(FONT_B, 0.0075 * ppm)
    for n in range(1, 9):
        col, row, y, z = g.space(n)
        s = 1 if col == "L" else -1
        a0, b0 = px(y + 0.030, z + 0.0092)
        a1, b1 = px(y - 0.030, z - 0.0092)
        d.rectangle([min(a0, a1), b0, max(a0, a1), b1], fill=(28, 28, 30))
        # the handle's slot
        c0, e0 = px(y + 0.013, z + 0.0058)
        c1, e1 = px(y - 0.013, z - 0.0058)
        d.rectangle([min(c0, c1), e0, max(c0, c1), e1], fill=(8, 8, 9))
        # ON toward the middle, OFF toward the side
        for word, dy in (("ON", -s * 0.021), ("OFF", s * 0.021)):
            cx, cy = px(y + dy, z)
            d.text((cx, cy), word, font=f_on, fill=(225, 225, 220), anchor="mm")
        # the circuit number beside the breaker, on the cover
        cx, cy = px(y + s * 0.050, z)
        d.text((cx, cy), str(n), font=f_num, fill=(25, 25, 25), anchor="mm")
    # a label strip above the opening
    lx, ly = px(g.PANEL_Y, g.ROWS[0] + 0.045)
    d.text((lx, ly), "QO", font=_font(FONT_B, 0.010 * ppm), fill=(40, 40, 40), anchor="mm")
    im.save(path)


def tag_face(path: Path, back: bool) -> None:
    """The 497A tag: front, the ANSI DANGER header (a red oval with DANGER in white, on black) over DO NOT OPERATE on
    white; back, write-on lines (Dept: Electrical, Expected completion: 4 PM). A brass grommet near the top."""
    w = 512
    ppm = w / g.TAG_W
    h = round(g.TAG_L * ppm)
    im = Image.new("RGB", (w, h), WHITE)
    d = ImageDraw.Draw(im)
    hole_y = g.TAG_HOLE * ppm
    if not back:
        top, bot = 0.024 * ppm, 0.060 * ppm
        d.rectangle([0, top, w, bot], fill=BLACK)
        d.ellipse([0.05 * w, top + 0.004 * ppm, 0.95 * w, bot - 0.004 * ppm], fill=RED)
        d.text((w / 2, (top + bot) / 2), "DANGER", font=_font(FONT_B, 0.0125 * ppm), fill=WHITE, anchor="mm")
        f = _font(FONT_B, 0.0150 * ppm)
        for k, word in enumerate(("DO NOT", "OPERATE")):
            d.text((w / 2, (0.080 + 0.022 * k) * ppm), word, font=f, fill=BLACK, anchor="mm")
        d.rectangle([0.05 * w, 0.122 * ppm, 0.95 * w, 0.140 * ppm], outline=BLACK, width=3)
    else:
        f = _font(FONT_C, 0.0058 * ppm)
        fh = _font("/usr/share/fonts/truetype/dejavu/DejaVuSans-Oblique.ttf", 0.0068 * ppm)
        rows = (("NAME:", ""), ("DEPT:", "Electrical"), ("EXPECTED", ""), ("COMPLETION:", "4 PM"))
        for k, (lab, val) in enumerate(rows):
            y = (0.040 + 0.020 * k) * ppm
            d.text((0.05 * w, y), lab, font=f, fill=BLACK, anchor="ls")
            d.line([0.40 * w, y + 2, 0.95 * w, y + 2], fill=BLACK, width=2)
            if val:
                d.text((0.45 * w, y - 3), val, font=fh, fill=(20, 30, 120), anchor="ls")
        d.text((w / 2, 0.128 * ppm), "DO NOT REMOVE THIS TAG", font=_font(FONT_B, 0.0052 * ppm), fill=RED, anchor="mm")
    # the grommet
    r_out, r_in = g.GROMMET_OD / 2 * ppm, 0.0055 * ppm
    d.ellipse([w / 2 - r_out, hole_y - r_out, w / 2 + r_out, hole_y + r_out], fill=BRASS)
    d.ellipse([w / 2 - r_in, hole_y - r_in, w / 2 + r_in, hole_y + r_in], fill=(30, 30, 30))
    im.save(path)


def directory(path: Path) -> None:
    """The circuit directory card inside the door."""
    w, h = 860, 1024
    im = Image.new("RGB", (w, h), (244, 244, 236))
    d = ImageDraw.Draw(im)
    f = _font(FONT_B, 40)
    d.text((w / 2, 60), "CIRCUIT DIRECTORY", font=f, fill=BLACK, anchor="mm")
    names = {1: "FURNACE", 2: "KITCHEN COUNTER L", 3: "BATH", 4: "KITCHEN COUNTER R", 5: "BEDROOMS",
             6: "REFRIGERATOR", 7: "LIVING ROOM", 8: "LIGHTS"}
    fl = _font(FONT_C, 34)
    for k, n in enumerate(range(1, 9)):
        y = 140 + 105 * k
        d.line([30, y + 45, w - 30, y + 45], fill=(120, 120, 130), width=2)
        d.text((40, y + 30), f"{n}", font=fl, fill=BLACK, anchor="ls")
        d.text((110, y + 30), names[n], font=fl, fill=(20, 30, 120), anchor="ls")
    im.save(path)


def build_atlas(path: Path) -> None:
    tmp = TEX / "_parts"
    tmp.mkdir(exist_ok=True)
    front(tmp / "front.png")
    tag_face(tmp / "tag_front.png", back=False)
    tag_face(tmp / "tag_back.png", back=True)
    directory(tmp / "directory.png")
    im = Image.new("RGB", (ATLAS_PX, ATLAS_PX), (128, 128, 128))
    for name, (x, y, w, h) in REGIONS.items():
        im.paste(Image.open(tmp / f"{name}.png").convert("RGB").resize((w, h), Image.LANCZOS), (x, y))
    for f in tmp.iterdir():
        f.unlink()
    tmp.rmdir()
    im.save(path)


def auv(region: str, uv):
    """uv in [0, 1]^2 of one artwork (v up) mapped into its region of the atlas."""
    x, y, w, h = REGIONS[region]
    u, v = uv
    return ((x + u * w) / ATLAS_PX, 1.0 - (y + (1.0 - v) * h) / ATLAS_PX)


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


def material(stage, path: str, color, roughness=0.5, texture: str | None = None, metallic=0.0):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
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
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*lin(color)))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(mat)


def offsets(prim, contact, rest):
    prim = prim.GetPrim()
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(contact)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(rest)


def box(stage, path, lo, hi, mat=None, collide=True, off=OFF_BIG):
    """An axis-aligned box in its body's frame."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, (lo + hi) / 2)))
    c.AddScaleOp().Set(Gf.Vec3f(*map(float, hi - lo)))
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        offsets(c, *off)
    return c


def quad(stage, path, corners, region, mat):
    """A textured quad: corners in order (image) bottom-left, bottom-right, top-right, top-left."""
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in corners])
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    m.CreateSubdivisionSchemeAttr("none")
    uv = [auv(region, t) for t in ((0, 0), (1, 0), (1, 1), (0, 1))]
    UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                         UsdGeom.Tokens.vertex).Set([Gf.Vec2f(*map(float, t)) for t in uv])
    m.CreateDoubleSidedAttr(True)
    bind(m, mat)
    return m


def cylinder_x(stage, path, radius, x0, x1, center_yz, mat):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(float(radius))
    c.CreateHeightAttr(float(abs(x1 - x0)))
    c.CreateAxisAttr("X")
    c.AddTranslateOp().Set(Gf.Vec3d(float((x0 + x1) / 2), float(center_yz[0]), float(center_yz[1])))
    bind(c, mat)
    return c


def quat_wxyz(R) -> Gf.Quatf:
    """A rotation matrix as a USD quaternion."""
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return Gf.Quatf(float(w), float(x), float(y), float(z))


def oriented_box(stage, path, center, R, size, mat, collide=True):
    """A box at ``center`` turned by the rotation matrix ``R`` (its axes), ``size`` along them."""
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    c.AddOrientOp().Set(quat_wxyz(R))
    c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        offsets(c, *OFF_BIG)
    return c


def rod(stage, path, a, b, radius, mat, collide=True):
    """A round rod from ``a`` to ``b`` (a box collider of its square section)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    z = (b - a) / np.linalg.norm(b - a)
    x = np.cross([0.0, 1.0, 0.0], z) if abs(z[1]) < 0.9 else np.cross([1.0, 0.0, 0.0], z)
    x /= np.linalg.norm(x)
    R = np.column_stack([x, np.cross(z, x), z])
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(float(radius))
    cyl.CreateHeightAttr(float(np.linalg.norm(b - a)))
    cyl.CreateAxisAttr("Z")
    cyl.AddTranslateOp().Set(Gf.Vec3d(*map(float, (a + b) / 2)))
    cyl.AddOrientOp().Set(quat_wxyz(R))
    bind(cyl, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        offsets(cyl, *OFF_BIG)
    return cyl


def body(stage, path, mass, pos, inertia=None, com=None):
    """A link: an Xform at ``pos`` (in the asset's frame), world-aligned."""
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    m = UsdPhysics.MassAPI.Apply(xf.GetPrim())
    m.CreateMassAttr(float(mass))
    if inertia is not None:
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
    if com is not None:
        m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
    return xf


Q_FLIP_X = Gf.Quatf(0.0, 1.0, 0.0, 0.0)     # 180 degrees about x: a joint axis "Z" becomes -z
Q_FLIP_Z = Gf.Quatf(0.0, 0.0, 0.0, 1.0)     # 180 degrees about z: a joint axis "X" becomes -x


def joint(stage, kind, path, body0, body1, pos0, axis, limits=None, rot=None):
    """A joint whose frame sits at body1's origin (``pos0`` in body0's frame); both bodies are world-aligned as
    authored. ``rot`` turns the joint frame in both (to flip the axis)."""
    j = (UsdPhysics.RevoluteJoint if kind == "revolute" else UsdPhysics.PrismaticJoint).Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    r = rot if rot is not None else Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    j.CreateLocalRot0Attr(r)
    j.CreateLocalRot1Attr(r)
    if limits is not None:
        j.CreateLowerLimitAttr(float(limits[0]))
        j.CreateUpperLimitAttr(float(limits[1]))
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if kind == "revolute" else "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.0)
    drive.CreateTargetPositionAttr(0.0)
    return j


# -- the panel -----------------------------------------------------------------------------------------------------------
def build_panel(name="panel"):
    stage, root = stage_for(name)
    r = f"/{name}"
    wall_m = material(stage, f"{r}/Looks/wall", WALL, roughness=0.8)
    pine_m = material(stage, f"{r}/Looks/pine", PINE, roughness=0.8)
    steel_m = material(stage, f"{r}/Looks/steel", STEEL, roughness=0.5, metallic=0.6)
    grey_m = material(stage, f"{r}/Looks/grey", BOX_GREY, roughness=0.45)
    atlas_m = material(stage, f"{r}/Looks/atlas", (255, 255, 255), roughness=0.45, texture=f"./textures/{ATLAS}")
    handle_m = material(stage, f"{r}/Looks/handle", HANDLE, roughness=0.4)
    brass_m = material(stage, f"{r}/Looks/brass", BRASS, roughness=0.35, metallic=0.8)
    tie_m = material(stage, f"{r}/Looks/tie", TIE, roughness=0.5)
    edge_m = material(stage, f"{r}/Looks/tag_edge", WHITE, roughness=0.5)

    prim = root.GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxArticulationAPI")
    prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(True)
    prim.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    prim.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)

    # base: wall, sill, brackets, box, cover, door
    b = f"{r}/base"
    body(stage, b, 20.0, (0.0, 0.0, 0.0))
    UsdPhysics.FixedJoint.Define(stage, f"{r}/joints/root_joint").CreateBody1Rel().SetTargets([Sdf.Path(b)])
    X, T = g.WALL_X, g.WALL_T
    box(stage, f"{b}/wall", (X, -g.WALL_W / 2, 0.0), (X + T, g.WALL_W / 2, g.WALL_H), wall_m)
    box(stage, f"{b}/sill", (X + T, -g.WALL_W / 2 + 0.01, 0.0), (X + T + g.SILL[1], g.WALL_W / 2 - 0.01, g.SILL[0]),
        pine_m)
    for k, y in enumerate(g.BRACKETS_Y):
        box(stage, f"{b}/bracket_{k}_up", (X + T, y - 0.012, g.SILL[0]), (X + T + 0.004, y + 0.012, 0.300), steel_m,
            collide=False)
        box(stage, f"{b}/bracket_{k}_foot", (X + T, y - 0.012, g.SILL[0]), (X + T + 0.200, y + 0.012, g.SILL[0] + 0.004),
            steel_m, collide=False)
    hw, P = g.BOX_W / 2, g.PANEL_Y
    box(stage, f"{b}/box", (g.BOX_X[0], P - hw, g.BOX_Z[0]), (g.BOX_X[1], P + hw, g.BOX_Z[1]), grey_m)
    cw = g.COVER_W / 2
    box(stage, f"{b}/cover", (g.FRONT_X, P - cw, g.COVER_Z[0]), (g.BOX_X[0], P + cw, g.COVER_Z[1]), grey_m)
    e = 0.0002
    xf = g.FRONT_X - e
    quad(stage, f"{b}/cover_face", [(xf, P + cw, g.COVER_Z[0]), (xf, P - cw, g.COVER_Z[0]), (xf, P - cw, g.COVER_Z[1]),
                                    (xf, P + cw, g.COVER_Z[1])], "front", atlas_m)
    # the door, open flat: hinged on the cover's right edge on a 15 mm standoff, its inside (the directory) to the robot
    hx, hy = g.DOOR_HINGE
    z0, z1 = g.DOOR_Z
    box(stage, f"{b}/door", (hx - g.DOOR_T, hy - g.DOOR_W, z0), (hx, hy, z1), grey_m)
    box(stage, f"{b}/door_standoff", (hx, hy, z0 + 0.02), (g.FRONT_X, hy + 0.004, z1 - 0.02), grey_m, collide=False)
    dx = hx - g.DOOR_T - e
    m = 0.016
    quad(stage, f"{b}/directory", [(dx, hy - m, z0 + 0.03), (dx, hy - g.DOOR_W + m, z0 + 0.03),
                                   (dx, hy - g.DOOR_W + m, z1 - 0.03), (dx, hy - m, z1 - 0.03)], "directory", atlas_m)
    for k, zc in enumerate((z0 + 0.03, z1 - 0.03)):
        cyl = UsdGeom.Cylinder.Define(stage, f"{b}/hinge_{k}")
        cyl.CreateRadiusAttr(0.003)
        cyl.CreateHeightAttr(0.03)
        cyl.CreateAxisAttr("Z")
        cyl.AddTranslateOp().Set(Gf.Vec3d(hx - 0.001, hy, zc))
        bind(cyl, grey_m)

    # the trial camera: a phone on a gooseneck clamped to the wall's top edge (fixed)
    phone_m = material(stage, f"{r}/Looks/phone", (24, 24, 28), roughness=0.3)
    neck_m = material(stage, f"{r}/Looks/neck", (40, 40, 44), roughness=0.5, metallic=0.4)
    cam, aim = np.array(g.TRIAL_CAM), np.array(g.TRIAL_AIM)
    v = (aim - cam) / np.linalg.norm(aim - cam)                # the view: the phone's back faces it
    side = np.cross(v, [0.0, 0.0, 1.0])
    side /= np.linalg.norm(side)
    up = np.cross(side, v)
    Rp = np.column_stack([up, side, v])                       # x: the phone's long side, z: through its back
    oriented_box(stage, f"{b}/phone", cam - v * (g.PHONE[2] / 2 + 0.001), Rp, (g.PHONE[0], g.PHONE[1], g.PHONE[2]),
                 phone_m)
    cx, cy, cz = g.CLAMP
    box(stage, f"{b}/clamp", (cx - 0.020, cy - 0.020, cz - 0.015), (cx + 0.020, cy + 0.020, cz + 0.015), neck_m)
    back = cam - v * (g.PHONE[2] + 0.004)
    rod(stage, f"{b}/neck_a", (cx, cy, cz - 0.010), g.NECK_ELBOW, 0.006, neck_m)
    rod(stage, f"{b}/neck_b", g.NECK_ELBOW, back, 0.006, neck_m)

    # the handles
    handles = []
    for n in range(1, 9):
        col, row, y, z = g.space(n)
        h = f"{r}/h{n}"
        handles.append(h)
        body(stage, h, g.HANDLE_MASS, (g.PIVOT_X, y, z))
        box(stage, f"{h}/grip", (-g.HANDLE_R, -g.HANDLE_T / 2, -g.HANDLE_HT / 2),
            (-g.HANDLE_IN, g.HANDLE_T / 2, g.HANDLE_HT / 2), handle_m, off=OFF_SMALL)
        joint(stage, "revolute", f"{r}/joints/h{n}", b, h, (g.PIVOT_X, y, z), "Z",
              (-math.degrees(g.HANDLE_STOP), math.degrees(g.HANDLE_STOP)), rot=None if col == "L" else Q_FLIP_X)
    fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(b))
    fp.CreateFilteredPairsRel().SetTargets([Sdf.Path(h) for h in handles])

    # the tie and the tag, on space 4's handle (authored with the handle straight, the tie and tag hanging straight)
    _, _, y4, z4 = g.space(g.TAGGED)
    p4 = np.array([g.PIVOT_X, y4, z4])
    s0 = g.LOOP_BASE + g.LOOP_S0
    loop_p = p4 + np.array([-s0, 0.0, 0.0])
    tiny = (5e-9, 5e-9, 5e-9)
    lp = f"{r}/loop"
    body(stage, lp, g.DUMMY_MASS, loop_p, tiny)
    # the loop round the stem (visual): four thin strips round the 9 x 10 handle
    hw4, hh4, tw, tt = g.HANDLE_T / 2 + 0.0008, g.HANDLE_HT / 2 + 0.0008, g.TIE_W / 2, 0.0008
    box(stage, f"{lp}/top", (-tw, -hw4, hh4 - tt), (tw, hw4, hh4), tie_m, collide=False)
    box(stage, f"{lp}/left", (-tw, hw4 - tt, -hh4 - 0.004), (tw, hw4, hh4), tie_m, collide=False)
    box(stage, f"{lp}/right", (-tw, -hw4, -hh4 - 0.004), (tw, -hw4 + tt, hh4), tie_m, collide=False)
    joint(stage, "prismatic", f"{r}/joints/loop", f"{r}/h{g.TAGGED}", lp, (-g.LOOP_BASE, 0.0, 0.0), "X",
          (0.0, g.LOOP_MAX), rot=Q_FLIP_Z)
    ta = f"{r}/tie_a"
    body(stage, ta, g.DUMMY_MASS, loop_p, tiny)
    joint(stage, "revolute", f"{r}/joints/tie_x", lp, ta, (0.0, 0.0, 0.0), "X", (-90.0, 90.0))
    ti = f"{r}/tie"
    body(stage, ti, g.DUMMY_MASS, loop_p, tiny)
    box(stage, f"{ti}/strap", (-tt, -tw, -g.TIE_L), (tt, tw, -g.HANDLE_HT / 2 - 0.001), tie_m, collide=False)
    joint(stage, "revolute", f"{r}/joints/tie_y", ta, ti, (0.0, 0.0, 0.0), "Y")
    hole = loop_p + np.array([0.0, 0.0, -g.TIE_L])
    ga = f"{r}/tag_a"
    body(stage, ga, g.DUMMY_MASS, hole, tiny)
    joint(stage, "revolute", f"{r}/joints/tag_y", ti, ga, (0.0, 0.0, -g.TIE_L), "Y")
    tg = f"{r}/tag"
    L, W, Tt = g.TAG_L, g.TAG_W, g.TAG_T
    zc = g.TAG_HOLE - L / 2
    inertia = (g.TAG_MASS * (W ** 2 + L ** 2) / 12, g.TAG_MASS * (Tt ** 2 + L ** 2) / 12,
               g.TAG_MASS * (Tt ** 2 + W ** 2) / 12)
    body(stage, tg, g.TAG_MASS, hole, inertia, com=(0.0, 0.0, zc))
    box(stage, f"{tg}/card", (-Tt / 2, -W / 2, g.TAG_HOLE - L), (Tt / 2, W / 2, g.TAG_HOLE), edge_m, off=OFF_SMALL)
    xf, xb = -Tt / 2 - 0.0001, Tt / 2 + 0.0001
    zb, zt = g.TAG_HOLE - L, g.TAG_HOLE
    # front (toward the robot, -x): image left = +y; back (+x): image left = -y
    quad(stage, f"{tg}/front", [(xf, W / 2, zb), (xf, -W / 2, zb), (xf, -W / 2, zt), (xf, W / 2, zt)], "tag_front",
         atlas_m)
    quad(stage, f"{tg}/back", [(xb, -W / 2, zb), (xb, W / 2, zb), (xb, W / 2, zt), (xb, -W / 2, zt)], "tag_back",
         atlas_m)
    cylinder_x(stage, f"{tg}/grommet", g.GROMMET_OD / 2, -Tt / 2 - 0.0006, Tt / 2 + 0.0006, (0.0, 0.0), brass_m)
    joint(stage, "revolute", f"{r}/joints/tag_z", ga, tg, (0.0, 0.0, 0.0), "Z", (-120.0, 120.0))
    stage.Save()


if __name__ == "__main__":
    TEX.mkdir(parents=True, exist_ok=True)
    for f in TEX.glob("*.png"):
        f.unlink()
    build_atlas(TEX / ATLAS)
    build_panel()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
