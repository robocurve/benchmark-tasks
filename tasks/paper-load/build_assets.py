"""Write the scene's USD assets and textures from geometry.py.

    python build_assets.py          # needs pxr, PIL and the DejaVu fonts (Isaac Sim's Python), not the simulator

- assets/printer.usda: the all-in-one, an articulation fixed to the world. Its body: the base, the scanner's frame
  and glass under the shut document cover, the operation panel on the top's left side, the rear housing with the
  rear tray's slot (its floor, its right side), the output tray pulled out. Its moving parts: the document cover on
  a hinge along its back edge, the paper support on a hinge along the housing's back (a stop at SUPPORT_STOP), the
  paper guide on a slide in the slot (a raised grip on top), and the OK, Black and Color keys (sprung, 1.5 mm of
  travel).
- assets/sheet.usda: the five sheets as one limp stack, an articulation with a free root: N_STRIPS strips side by
  side, each the stack's full width, on hinges along their long edges, each with a bending spring toward flat (set
  from task.py). Each strip's collider is a COLL_T box (not rendered); the stack is drawn DRAW_T thick in its middle.
- assets/stand.usda: the IKEA LACK side table.
- assets/textures/: the operation panel and the two copy-button tops (drawn here; names unique to this task and
  versioned, since the renderer caches textures by file name).
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"
TEX = OUT / "textures"
SANS_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
SANS = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

PANEL_TEX = "pl_panel_1.png"
BLACK_TEX = "pl_key_black_1.png"
COLOR_TEX = "pl_key_color_1.png"

WHITE = (226, 226, 222)             # the printer's white plastic
WHITE_SHADE = (214, 214, 210)
GREY = (150, 152, 155)
DARK = (38, 40, 44)
GLASS = (34, 40, 46)                # the glass over the dark scanner bed
PAD = (244, 244, 242)               # the cover's white pad
PAPER = (252, 252, 255)             # bright-white copy paper


def lin(srgb):
    out = []
    for c in srgb:
        c = c / 255.0
        out.append(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4)
    return tuple(out)


# -- textures ---------------------------------------------------------------------------------------------------------
def panel_texture(path: Path) -> None:
    """The operation panel (PANEL_X by PANEL_Y), seen from above with the printer's front at the image's bottom: ON at
    the back, the LCD (the copy count 1 and LTR), Paper Select, OK and Stop; the Black and Color buttons themselves are
    separate parts at the front."""
    px = 8                                                          # px per mm
    W = round((g.PANEL_Y[1] - g.PANEL_Y[0]) * 1000 * px)            # image x: robot's right (-y) to the right
    H = round((g.PANEL_X[1] - g.PANEL_X[0]) * 1000 * px)            # image y: from the back (top) to the front
    im = Image.new("RGB", (W, H), (226, 227, 226))
    d = ImageDraw.Draw(im)

    def at(x, y):                                                   # station x, y -> pixel
        return ((g.PANEL_Y[1] - y) * 1000 * px, (g.PANEL_X[1] - x) * 1000 * px)

    def circle(x, y, r_mm, **k):
        u, v = at(x, y)
        r = r_mm * px
        d.ellipse([u - r, v - r, u + r, v + r], **k)

    def key(x, y, s_mm):
        u, v = at(x, y)
        r = s_mm * px / 2
        d.rounded_rectangle([u - r, v - r, u + r, v + r], radius=1.5 * px, fill=(242, 242, 240),
                            outline=(170, 170, 170))

    yc = (g.PANEL_Y[0] + g.PANEL_Y[1]) / 2
    circle(0.385, yc, 5.5, fill=(242, 242, 240), outline=(160, 160, 160), width=2)       # ON
    circle(0.385, yc, 2.5, outline=(40, 40, 40), width=3)
    key(0.362, yc + 0.012, 11)                                                          # wireless
    key(0.362, yc - 0.012, 11)
    key(0.340, yc + 0.012, 11)                                                          # information, setup
    key(0.340, yc - 0.012, 11)
    # the LCD: the copy count and the paper size
    u0, v0 = at(0.318, yc + 0.020)
    u1, v1 = at(0.278, yc - 0.020)
    d.rectangle([u0, v0, u1, v1], fill=(28, 34, 30), outline=(120, 120, 120), width=2)
    seg = ImageFont.truetype(SANS_B, round(22 * px))
    b = d.textbbox((0, 0), "1", font=seg)
    d.text(((u0 + u1) / 2 - (b[2] - b[0]) / 2 - b[0] + 6 * px, v0 + 3 * px - b[1]), "1", font=seg, fill=(180, 230, 190))
    sm = ImageFont.truetype(SANS_B, round(6 * px))
    d.text((u1 - 17 * px, v1 - 9 * px), "LTR", font=sm, fill=(180, 230, 190))
    key(0.262, yc + 0.012, 11)                                                          # paper select, OK
    key(0.262, yc - 0.012, 11)
    circle(0.240, yc, 5.5, fill=(242, 242, 240), outline=(160, 160, 160), width=2)      # stop
    u, v = at(0.240, yc)
    d.regular_polygon((u, v, 3 * px), 8, fill=(200, 40, 40))
    # sockets round the Black and Color buttons
    for xy in (g.BLACK_XY, g.COLOR_XY):
        u, v = at(*xy)
        r = (g.BTN / 2 + 0.0015) * 1000 * px
        d.rounded_rectangle([u - r, v - r, u + r, v + r], radius=2 * px, fill=(196, 198, 198))
    im.save(path)


def key_texture(path: Path, colour: bool) -> None:
    """A button top: a pale key with a diamond (black, or in the four ink colours)."""
    n = 128
    im = Image.new("RGB", (n, n), (240, 240, 238))
    d = ImageDraw.Draw(im)
    c, r = n / 2, n * 0.34
    pts = [(c, c - r), (c + r, c), (c, c + r), (c - r, c)]
    if colour:
        d.polygon([(c, c - r), (c + r, c), (c, c)], fill=(0, 160, 220))
        d.polygon([(c + r, c), (c, c + r), (c, c)], fill=(220, 0, 130))
        d.polygon([(c, c + r), (c - r, c), (c, c)], fill=(250, 210, 0))
        d.polygon([(c - r, c), (c, c - r), (c, c)], fill=(30, 30, 30))
    else:
        d.polygon(pts, fill=(25, 25, 25))
    d.polygon(pts, outline=(60, 60, 60))
    im.save(path)


# -- USD helpers -------------------------------------------------------------------------------------------------------
def stage_for(name: str):
    from pxr import Usd, UsdGeom

    path = OUT / f"{name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    return stage, root


def material(stage, path: str, color, roughness=0.5, texture: str | None = None, metallic=0.0, matte=False):
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    if matte:
        sh.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
        sh.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
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
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(mat)


def offsets(prim, contact=0.001, rest=0.0):
    from pxr import Sdf

    prim = prim.GetPrim()
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(contact)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(rest)


def box(stage, path, lo, hi, mat=None, collide=True, visible=True, off=(0.001, 0.0)):
    """An axis-aligned box from corner ``lo`` to ``hi`` (in its parent's frame)."""
    from pxr import Gf, UsdGeom, UsdPhysics

    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, (lo + hi) / 2)))
    c.AddScaleOp().Set(Gf.Vec3f(*map(float, hi - lo)))
    if mat is not None:
        bind(c, mat)
    if not visible:
        c.CreatePurposeAttr(UsdGeom.Tokens.guide)                    # a collider that isn't rendered
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        offsets(c, *off)
    return c


def tilted_box(stage, path, centre, size, rot_q, mat, collide=True):
    from pxr import Gf, UsdGeom, UsdPhysics

    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
    c.AddOrientOp().Set(Gf.Quatf(*map(float, rot_q)))
    c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        offsets(c)
    return c


def mesh(stage, path, V, F, mat=None, uv=None):
    from pxr import Gf, Sdf, UsdGeom

    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in V])
    m.CreateFaceVertexCountsAttr([len(f) for f in F])
    m.CreateFaceVertexIndicesAttr([int(i) for f in F for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDoubleSidedAttr(True)
    if uv is not None:
        UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                             UsdGeom.Tokens.vertex).Set([Gf.Vec2f(*map(float, t)) for t in uv])
    if mat is not None:
        bind(m, mat)
    return m


def quad(stage, path, corners, mat, uv=((0, 0), (1, 0), (1, 1), (0, 1))):
    """A quad from four corners, counterclockwise seen from its front."""
    return mesh(stage, path, corners, [[0, 1, 2, 3]], mat, uv=uv)


def body(stage, path, mass, pos=(0.0, 0.0, 0.0), damping=None):
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
    xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
    p = xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    UsdPhysics.MassAPI.Apply(p).CreateMassAttr(float(mass))
    p.AddAppliedSchema("PhysxRigidBodyAPI")
    p.CreateAttribute("physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    p.CreateAttribute("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float).Set(0.3)
    if damping is not None:
        p.CreateAttribute("physxRigidBody:linearDamping", Sdf.ValueTypeNames.Float).Set(damping[0])
        p.CreateAttribute("physxRigidBody:angularDamping", Sdf.ValueTypeNames.Float).Set(damping[1])
    return xf


def joint(stage, kind, path, body0, body1, pos0, pos1, axis, limits, stiffness=0.0, damping=0.0):
    """A revolute (limits in degrees, drive per degree) or prismatic joint (limits in m)."""
    from pxr import Gf, Sdf, UsdPhysics

    j = (UsdPhysics.RevoluteJoint if kind == "revolute" else UsdPhysics.PrismaticJoint).Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, pos1)))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(float(limits[0]))
    j.CreateUpperLimitAttr(float(limits[1]))
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if kind == "revolute" else "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(stiffness))
    drive.CreateDampingAttr(float(damping))
    drive.CreateTargetPositionAttr(0.0)
    return j


def quat_y(angle):
    return (math.cos(angle / 2), 0.0, math.sin(angle / 2), 0.0)


# -- the printer -------------------------------------------------------------------------------------------------------
def build_printer():
    """Origin: the station origin on the table top (x, y as the station frame, z above the table)."""
    from pxr import Sdf, UsdPhysics

    stage, root = stage_for("printer")
    r = "/printer"
    white = material(stage, f"{r}/Looks/white", WHITE, roughness=0.45)
    shade = material(stage, f"{r}/Looks/shade", WHITE_SHADE, roughness=0.5)
    grey = material(stage, f"{r}/Looks/grey", GREY, roughness=0.5)
    dark = material(stage, f"{r}/Looks/dark", DARK, roughness=0.5)
    glass = material(stage, f"{r}/Looks/glass", GLASS, roughness=0.08)
    pad = material(stage, f"{r}/Looks/pad", PAD, roughness=0.8, matte=True)
    paper = material(stage, f"{r}/Looks/paper", PAPER, roughness=0.8, matte=True)
    mark = material(stage, f"{r}/Looks/mark", (20, 20, 22), roughness=0.6)
    panel = material(stage, f"{r}/Looks/panel", (255, 255, 255), roughness=0.5, texture=f"./textures/{PANEL_TEX}")
    k_black = material(stage, f"{r}/Looks/key_black", (255, 255, 255), roughness=0.5,
                       texture=f"./textures/{BLACK_TEX}")
    k_color = material(stage, f"{r}/Looks/key_color", (255, 255, 255), roughness=0.5,
                       texture=f"./textures/{COLOR_TEX}")

    prim = root.GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxArticulationAPI")
    prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    prim.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    prim.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)

    b = f"{r}/body"
    body(stage, b, g.PR_MASS - g.COVER_MASS)
    UsdPhysics.FixedJoint.Define(stage, f"{r}/joints/root_joint").CreateBody1Rel().SetTargets([Sdf.Path(b)])
    X0, X1 = g.PR_X
    Y0, Y1 = g.PR_Y
    GX0, GX1 = g.GLASS_X
    GY0, GY1 = g.GLASS_Y
    LO = 0.100                                     # the scanner unit's floor under the glass and frame
    REAR = g.REAR_X[0]                             # the rear tray housing starts here (behind the cover's hinge)
    side_y = g.COVER_Y[1]                          # the panel side of the top starts here
    box(stage, f"{b}/base", (X0, Y0, 0.004), (X1, Y1, g.LIP_FLOOR_H), white)    # its top in front: the recess
    box(stage, f"{b}/base_upper", (g.LIP_X[1], Y0, g.LIP_FLOOR_H), (X1, Y1, LO), white)
    box(stage, f"{b}/feet", (X0 + 0.01, Y0 + 0.01, 0.0), (X1 - 0.01, Y1 - 0.01, 0.004), dark)
    box(stage, f"{b}/glass", (GX0, GY0, LO), (GX1, GY1, g.GLASS_H), glass)
    box(stage, f"{b}/frame_front", (g.LIP_X[1], Y0, LO), (GX0, side_y, g.FRAME_H), shade)
    box(stage, f"{b}/frame_back", (GX1, Y0, LO), (REAR, side_y, g.FRAME_H), shade)
    box(stage, f"{b}/frame_right", (GX0, Y0, LO), (GX1, GY0, g.FRAME_H), shade)
    box(stage, f"{b}/frame_left", (GX0, GY1, LO), (GX1, side_y, g.FRAME_H), shade)
    box(stage, f"{b}/side", (X0, side_y, g.LIP_FLOOR_H), (REAR, Y1, g.COVER_TOP_H), white)
    # the rear tray: a slot along the housing (its right end the right side), the floor 112 up
    SX0, SX1 = g.SLOT_X
    SY0, SY1 = g.SLOT_Y
    box(stage, f"{b}/rear_front", (REAR, Y0, LO), (SX0, Y1, g.PR_H), white)
    box(stage, f"{b}/slot_floor", (SX0, SY0, LO), (SX1, SY1, g.SLOT_FLOOR_H), dark)
    box(stage, f"{b}/slot_right", (SX0, Y0, LO), (SX1, SY0, g.PR_H), white)
    box(stage, f"{b}/slot_left", (SX0, SY1, LO), (SX1, Y1, g.PR_H), white)
    box(stage, f"{b}/rear_back", (SX1, Y0, LO), (X1, Y1, g.SUPPORT_HINGE[1] - 0.004), white)
    box(stage, f"{b}/right_mark", (SX0, SY0 - 0.0006, g.PR_H - 0.012), (SX1, SY0 + 0.0002, g.PR_H - 0.002), mark,
        collide=False)
    # the operation panel (a texture on the side's top) and the sockets of the two buttons
    z = g.COVER_TOP_H + 0.0002
    quad(stage, f"{b}/panel", [(g.PANEL_X[1], g.PANEL_Y[1], z), (g.PANEL_X[1], g.PANEL_Y[0], z),
                               (g.PANEL_X[0], g.PANEL_Y[0], z), (g.PANEL_X[0], g.PANEL_Y[1], z)], panel,
         uv=((0, 1), (1, 1), (1, 0), (0, 0)))
    # the alignment mark: an arrow on the frame pointing into the back-left corner of the glass
    zf = g.FRAME_H + 0.0002
    cx, cy = g.CORNER
    tri = [(cx + 0.0015, cy + 0.0015, zf), (cx + 0.0095, cy + 0.0015, zf), (cx + 0.0015, cy + 0.0095, zf)]
    mesh(stage, f"{b}/mark", tri, [[0, 1, 2]], mark)
    stem = [(cx + 0.0055, cy + 0.0085, zf), (cx + 0.0085, cy + 0.0055, zf), (cx + 0.0135, cy + 0.0105, zf),
            (cx + 0.0105, cy + 0.0135, zf)]
    mesh(stage, f"{b}/mark_stem", stem, [[0, 1, 2, 3]], mark)
    # the front: the paper exit (dark) above the output tray, pulled out
    box(stage, f"{b}/exit", (X0 - 0.0008, -0.130, 0.030), (X0 + 0.002, 0.030, 0.062), dark, collide=False)
    box(stage, f"{b}/tray", (g.TRAY_X[0], g.TRAY_Y[0], g.TRAY_H - 0.003), (X0, g.TRAY_Y[1], g.TRAY_H), shade)
    box(stage, f"{b}/tray_stop", (g.TRAY_X[0], g.TRAY_Y[0], g.TRAY_H), (g.TRAY_X[0] + 0.004, g.TRAY_Y[1],
                                                                       g.TRAY_H + 0.012), shade)

    # the document cover: its frame on the hinge line; shut, it runs from there toward -x
    c = f"{r}/cover"
    body(stage, c, g.COVER_MASS, (g.HINGE[0], 0.0, g.HINGE[1]))
    L = g.HINGE[0] - g.COVER_X[0]
    box(stage, f"{c}/plate", (-L, g.COVER_Y[0], -g.COVER_T), (0.0, g.COVER_Y[1], 0.0), white)
    box(stage, f"{c}/pad", (-L + 0.006, g.GLASS_Y[0] - 0.010, -g.COVER_T - 0.0004),
        (-0.010, g.GLASS_Y[1] + 0.010, -g.COVER_T + 0.0001), pad, collide=False)
    box(stage, f"{c}/grip", (-L - 0.0005, -0.120, -0.012), (-L + 0.0003, 0.020, -0.006), shade, collide=False)
    joint(stage, "revolute", f"{r}/joints/cover", b, c, (g.HINGE[0], 0.0, g.HINGE[1]), (0, 0, 0), "Y",
          (0.0, math.degrees(g.COVER_STOP)))

    # the paper support: a plate on a hinge along the housing's back; shut (0) it lies forward over the rear tray
    p = f"{r}/support"
    hx, hz = g.SUPPORT_HINGE
    body(stage, p, g.SUPPORT_MASS, (hx, 0.0, hz))
    box(stage, f"{p}/plate", (-g.SUPPORT_L, g.SUPPORT_Y[0], 0.0), (0.0, g.SUPPORT_Y[1], g.SUPPORT_T), white)
    joint(stage, "revolute", f"{r}/joints/support", b, p, (hx, 0.0, hz), (0, 0, 0), "Y",
          (0.0, math.degrees(g.SUPPORT_STOP)))
    # the paper guide: a slider in the slot, its face (frame origin, at mid height) toward -y; widest at q 0
    p = f"{r}/guide"
    gx = (g.SLOT_X[0] + g.SLOT_X[1]) / 2
    body(stage, p, g.GUIDE_MASS, (gx, g.GUIDE_OPEN_Y, 0.0))
    hw = (g.SLOT_X[1] - g.SLOT_X[0]) / 2 - 0.001
    box(stage, f"{p}/fence", (-hw, 0.0, g.GUIDE_H[0]), (hw, g.GUIDE_T, g.GUIDE_H[1]), grey, off=(0.0005, 0.0))
    box(stage, f"{p}/grip", (-hw, 0.0, g.GUIDE_H[1] - g.GUIDE_GRIP_H), (hw, 0.014, g.GUIDE_H[1]), grey,
        off=(0.0005, 0.0))
    joint(stage, "prismatic", f"{r}/joints/guide", b, p, (gx, g.GUIDE_OPEN_Y, 0.0), (0, 0, 0), "Y",
          (-g.GUIDE_TRAVEL, 0.0))

    # the Black, Color and OK buttons: frame at the key's top centre, at rest
    for name, xy, mat in (("btn_black", g.BLACK_XY, k_black), ("btn_color", g.COLOR_XY, k_color),
                          ("btn_ok", g.OK_XY, grey)):
        p = f"{r}/{name}"
        body(stage, p, 0.002, (xy[0], xy[1], g.BTN_TOP_H))
        s = g.BTN / 2
        box(stage, f"{p}/key", (-s, -s, -0.006), (s, s, -0.0002), grey, off=(0.0005, 0.0))
        quad(stage, f"{p}/face", [(s, s, 0.0), (s, -s, 0.0), (-s, -s, 0.0), (-s, s, 0.0)], mat,
             uv=((0, 1), (1, 1), (1, 0), (0, 0)))
        joint(stage, "prismatic", f"{r}/joints/{name}", b, p, (xy[0], xy[1], g.BTN_TOP_H), (0, 0, 0), "Z",
              (-g.BTN_TRAVEL, 0.0))
    stage.Save()


# -- the paper stack ---------------------------------------------------------------------------------------------------
ROOT_STRIP = g.N_STRIPS // 2


def build_sheet():
    """Root frame = the middle strip's centre; strip i at ((i - ROOT_STRIP) * STRIP_W, 0, 0)."""
    from pxr import Sdf, UsdPhysics

    stage, root = stage_for("sheet")
    r = "/sheet"
    face = material(stage, f"{r}/Looks/face", PAPER, roughness=0.85, matte=True)
    w, L, t = g.STRIP_W, g.SHEET_L, g.DRAW_T
    for i in range(g.N_STRIPS):
        s = f"{r}/strip_{i}"
        x = body(stage, s, g.STRIP_MASS, ((i - ROOT_STRIP) * w, 0.0, 0.0), damping=(0.05, 0.05))
        if i == ROOT_STRIP:
            UsdPhysics.ArticulationRootAPI.Apply(x.GetPrim())
            x.GetPrim().AddAppliedSchema("PhysxArticulationAPI")
            x.GetPrim().CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
            x.GetPrim().CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
        box(stage, f"{s}/collider", (-w / 2, -L / 2, -g.COLL_T / 2), (w / 2, L / 2, g.COLL_T / 2), visible=False,
            off=(0.001, 0.0))
        quad(stage, f"{s}/top", [(-w / 2, L / 2, t / 2), (-w / 2, -L / 2, t / 2), (w / 2, -L / 2, t / 2),
                                 (w / 2, L / 2, t / 2)], face)
        quad(stage, f"{s}/bottom", [(-w / 2, L / 2, -t / 2), (w / 2, L / 2, -t / 2), (w / 2, -L / 2, -t / 2),
                                    (-w / 2, -L / 2, -t / 2)], face)
    for i in range(g.N_STRIPS - 1):                # h{i}: between strip i and strip i + 1
        if i >= ROOT_STRIP:
            b0, b1, p0, p1 = f"{r}/strip_{i}", f"{r}/strip_{i + 1}", (w / 2, 0, 0), (-w / 2, 0, 0)
        else:
            b0, b1, p0, p1 = f"{r}/strip_{i + 1}", f"{r}/strip_{i}", (-w / 2, 0, 0), (w / 2, 0, 0)
        lim = math.degrees(g.HINGE_LIMIT)
        joint(stage, "revolute", f"{r}/joints/h{i}", b0, b1, p0, p1, "Y", (-lim, lim),
              stiffness=g.HINGE_K * math.pi / 180, damping=g.HINGE_C * math.pi / 180)
    stage.Save()


def build_stand():
    """The IKEA LACK side table (fixed): origin on the floor below the station origin."""
    from pxr import UsdPhysics

    stage, root = stage_for("stand")
    r = "/stand"
    white = material(stage, f"{r}/Looks/white", (240, 240, 238), roughness=0.5)
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.RigidBodyAPI(root.GetPrim()).CreateKinematicEnabledAttr(True)
    x0, y0 = g.LACK_X0, g.LACK_Y - g.LACK[1] / 2
    x1, y1 = x0 + g.LACK[0], y0 + g.LACK[1]
    box(stage, f"{r}/top", (x0, y0, g.STAND_H - g.LACK[2]), (x1, y1, g.STAND_H), white)
    for i, (lx, ly) in enumerate(((x0, y0), (x1 - 0.05, y0), (x0, y1 - 0.05), (x1 - 0.05, y1 - 0.05))):
        box(stage, f"{r}/leg_{i}", (lx, ly, 0.0), (lx + 0.05, ly + 0.05, g.STAND_H - g.LACK[2]), white)
    stage.Save()


def main():
    TEX.mkdir(parents=True, exist_ok=True)
    panel_texture(TEX / PANEL_TEX)
    key_texture(TEX / BLACK_TEX, colour=False)
    key_texture(TEX / COLOR_TEX, colour=True)
    build_printer()
    build_sheet()
    build_stand()
    print("written:", sorted(p.name for p in OUT.rglob("*") if p.is_file()))


if __name__ == "__main__":
    main()
