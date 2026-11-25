"""Write the scene's USD assets and textures from geometry.py.

    python build_assets.py          # needs pxr, PIL and the DejaVu fonts (Isaac Sim's Python), not the simulator

- assets/phone.usda: the iPhone 12, one rigid body: the body (a convex prism with the outline's rounded corners), the
  camera plateau and the two lens bumps as colliders; the screen shows a still of the Camera app in Pano.
- assets/fixtures.usda: everything fixed, one kinematic body: the mat and its tape cross, the desk stand (base, arm,
  reclined back plate, seat and lip), the three boards in their T-base holders and the three rods in their blocks.
- assets/textures/pano_screen_1.png, pano_board_{1,2,3}_1.png: drawn here.
- assets/print/board_{1,2,3}.pdf: the boards' prints at true size (A4, 100 %), for the real build.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"
TEX = OUT / "textures"
PRINT = OUT / "print"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
OFF = (0.0005, 0.0)                 # contact and rest offsets of the colliders

WHITE_BACK = (236, 236, 232)        # iPhone 12 in white: back glass, aluminium band, plateau
BAND = (214, 214, 210)
LENS_RING = (60, 62, 66)
LENS_GLASS = (12, 12, 16)
ALU = (168, 172, 178)               # the stand: bead-blasted aluminium
MAT = (24, 24, 26)
TAPE = (240, 240, 236)
ACRYLIC = (226, 232, 236)           # the T-base holders (clear acrylic, rendered frosted)
STEEL = (150, 152, 156)
BLOCK = (196, 160, 110)
SCREEN_TEX = "pano_screen_1.png"


def lin(srgb):
    out = []
    for c in srgb:
        c = c / 255.0
        out.append(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4)
    return tuple(out)


def board_tex(i: int) -> str:
    return f"pano_board_{i + 1}_1.png"


# -- textures -----------------------------------------------------------------------------------------------------------
def font_for_cap(path: str, cap_px: float):
    size = int(cap_px / 0.73)
    for _ in range(20):
        f = ImageFont.truetype(path, size)
        box = f.getbbox("8")
        h = box[3] - box[1]
        if abs(h - cap_px) <= 1:
            break
        size = max(8, int(round(size * cap_px / h)))
    return ImageFont.truetype(path, size)


def board_image(i: int, px_per_mm: float) -> Image.Image:
    """An A4 sheet, portrait, as seen from the cross: the centre line with its two ticks, the coloured stripe at lens
    height and the board's number at the upper left. Image row 0 is the sheet's top edge."""
    W, H = round(g.SHEET_W * 1000 * px_per_mm), round(g.SHEET_H * 1000 * px_per_mm)
    im = Image.new("RGB", (W, H), (250, 250, 248))
    d = ImageDraw.Draw(im)
    k = px_per_mm

    def row(z_table_mm):            # an image row from a height above the table
        return (g.SHEET_Z0 * 1000 + g.SHEET_H * 1000 - z_table_mm) * k

    col = g.BOARD_COLOURS[i]
    zs = g.LENS_H * 1000
    d.rectangle([0, row(zs + g.STRIPE_H * 500), W, row(zs - g.STRIPE_H * 500)], fill=col)
    cx = W / 2
    d.rectangle([cx - g.LINE_W * 500 * k, 0, cx + g.LINE_W * 500 * k, H], fill=(0, 0, 0))
    for t in g.TICKS:
        y = row(t * 1000)
        d.rectangle([cx - g.TICK_L * 500 * k, y - 1.0 * k, cx + g.TICK_L * 500 * k, y + 1.0 * k], fill=(0, 0, 0))
    s = g.DIGIT_H * 1000 * k
    x0, y0 = 10 * k, 10 * k
    d.rectangle([x0, y0, x0 + s, y0 + s], fill=col)
    f = font_for_cap(FONT_B, 0.70 * s)
    txt = str(i + 1)
    l, t, r, b = f.getbbox(txt)
    d.text((x0 + s / 2 - (l + r) / 2, y0 + s / 2 - (t + b) / 2), txt, font=f, fill=(255, 255, 255))
    return im


def screen_image() -> Image.Image:
    """A still of the iPhone Camera app in Pano, portrait, the glass's full face (69.42 x 144.61 mm) at 6 px/mm:
    the black surround, the preview, the guide band with its yellow centre line and the arrow at its left end, the
    mode labels with PANO selected, and the white shutter button 21 mm above the phone's bottom edge."""
    k = 6.0
    gw, gh = 69.42, 144.61
    W, H = round(gw * k), round(gh * k)
    im = Image.new("RGB", (W, H), (6, 6, 8))
    d = ImageDraw.Draw(im)
    inset = (g.PW * 1000 - gw) / 2                    # glass to housing edge
    bottom = (g.PH * 1000 - gh) / 2

    def y_from_bottom(mm):                            # image row of a height above the phone's bottom edge
        return H - (mm - bottom) * k

    # the preview: from 32 to 128 mm above the bottom edge
    y0, y1 = y_from_bottom(128), y_from_bottom(34)
    for yy in range(int(y0), int(y1)):
        f = (yy - y0) / (y1 - y0)
        c = tuple(int(v) for v in (np.array([150, 160, 170]) * (1 - f) + np.array([110, 98, 82]) * f))
        d.line([(0, yy), (W, yy)], fill=c)
    # the guide band and the yellow centre line, the arrow at its left end
    ym = (y0 + y1) / 2
    d.rectangle([0, ym - 9 * k, W, ym + 9 * k], fill=(40, 40, 44))
    d.line([(6 * k, ym), (W - 4 * k, ym)], fill=(255, 204, 0), width=max(2, int(0.5 * k)))
    ax = 8 * k
    d.polygon([(ax - 3 * k, ym - 4 * k), (ax + 4 * k, ym), (ax - 3 * k, ym + 4 * k)], fill=(255, 255, 255))
    small = ImageFont.truetype(FONT, int(2.2 * k))
    msg = "Move iPhone continuously when taking a panorama."
    tw = d.textlength(msg, font=small)
    d.text(((W - tw) / 2, y0 + 6 * k), msg, font=small, fill=(255, 255, 255))
    # mode labels
    lab = ImageFont.truetype(FONT_B, int(2.3 * k))
    yl = y_from_bottom(31.5)
    modes = ["VIDEO", "PHOTO", "PORTRAIT", "PANO"]
    xs = [W / 2 - 34 * k, W / 2 - 18 * k, W / 2 - 1 * k, W / 2 + 18 * k]
    for m, x in zip(modes, xs):
        d.text((x, yl), m, font=lab, fill=(255, 204, 0) if m == "PANO" else (230, 230, 230))
    # the shutter: a white ring and a white disc, centre on the centre line 21 mm above the bottom edge
    cx, cy = W / 2, y_from_bottom(g.SHUTTER_UP * 1000)
    ro = g.SHUTTER_D * 1000 / 2 * k
    d.ellipse([cx - ro, cy - ro, cx + ro, cy + ro], fill=(250, 250, 250))
    d.ellipse([cx - 0.86 * ro, cy - 0.86 * ro, cx + 0.86 * ro, cy + 0.86 * ro], fill=(6, 6, 8))
    d.ellipse([cx - 0.78 * ro, cy - 0.78 * ro, cx + 0.78 * ro, cy + 0.78 * ro], fill=(250, 250, 250))
    # the last photo's thumbnail, lower left
    tx = cx - 22 * k
    d.rectangle([tx - 4.5 * k, cy - 4.5 * k, tx + 4.5 * k, cy + 4.5 * k], fill=(120, 140, 120))
    # status bar items
    st = ImageFont.truetype(FONT_B, int(2.4 * k))
    d.text((6 * k, 3 * k), "9:41", font=st, fill=(255, 255, 255))
    d.rounded_rectangle([W / 2 - 13 * k, 0, W / 2 + 13 * k, 4.5 * k], radius=2 * k, fill=(0, 0, 0))
    return im


def write_textures() -> None:
    TEX.mkdir(parents=True, exist_ok=True)
    for f in TEX.glob("*.png"):
        f.unlink()
    screen_image().save(TEX / SCREEN_TEX)
    for i in range(3):
        board_image(i, 4.0).save(TEX / board_tex(i))


def printables() -> None:
    """The three boards at true size: A4 portrait PDFs, print at 100 % (no scaling)."""
    PRINT.mkdir(parents=True, exist_ok=True)
    dpi = 300
    for i in range(3):
        im = board_image(i, dpi / 25.4)
        im.save(PRINT / f"board_{i + 1}.pdf", "PDF", resolution=dpi)


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


def offsets(prim, contact=OFF[0], rest=OFF[1]):
    prim = prim.GetPrim()
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(contact)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(rest)


def mesh(stage, path, V, F, mat=None, collide=False, uv=None):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in V])
    m.CreateFaceVertexCountsAttr([len(f) for f in F])
    m.CreateFaceVertexIndicesAttr([int(i) for f in F for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    if uv is not None:
        UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                             UsdGeom.Tokens.faceVarying).Set([Gf.Vec2f(*map(float, t)) for t in uv])
    m.CreateDoubleSidedAttr(True)
    if mat is not None:
        bind(m, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
        offsets(m)
    return m


def box_mesh(stage, path, centre, R, size, mat=None, collide=True):
    """An oriented box (centre, rotation, edge lengths) as a mesh (convex hull collider)."""
    h = np.asarray(size, float) / 2
    V = [np.asarray(centre) + R @ (h * np.array([sx, sy, sz])) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    F = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]]
    return mesh(stage, path, V, F, mat, collide)


def prism(stage, path, outline_2d, axis_lo, axis_hi, frame, mat=None, collide=True):
    """A convex prism: a convex outline (u, v) swept along w from axis_lo to axis_hi; frame = (origin, U, V, W)."""
    o, U, Vv, Wv = (np.asarray(a, float) for a in frame)
    n = len(outline_2d)
    P = [o + U * a + Vv * b + Wv * w for w in (axis_lo, axis_hi) for a, b in outline_2d]
    F = [list(range(n))[::-1], list(range(n, 2 * n))]
    for k in range(n):
        j = (k + 1) % n
        F.append([k, j, n + j, n + k])
    return mesh(stage, path, P, F, mat, collide)


def quad(stage, path, corners, uv, mat):
    return mesh(stage, path, corners, [[0, 1, 2, 3]], mat, uv=uv)


def disc(r, n=16, c=(0.0, 0.0)):
    return [(c[0] + r * math.cos(2 * math.pi * k / n), c[1] + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def rounded_rect(w, h, r, n=6):
    """A rectangle w x h centred on the origin with corner radius r, counterclockwise."""
    pts = []
    for cx, cy, a0 in ((w / 2 - r, h / 2 - r, 0), (-w / 2 + r, h / 2 - r, 90), (-w / 2 + r, -h / 2 + r, 180),
                       (w / 2 - r, -h / 2 + r, 270)):
        for k in range(n + 1):
            a = math.radians(a0 + 90 * k / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


# -- the phone ----------------------------------------------------------------------------------------------------------
def build_phone():
    """Origin at the body's centre; x out of the screen, z toward the top edge, y to the front view's right (the
    camera's side)."""
    stage, root = stage_for("phone")
    p = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    m = UsdPhysics.MassAPI.Apply(p)
    m.CreateMassAttr(g.PHONE_MASS)
    m.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
    a, b, c = g.PT, g.PW, g.PH
    M = g.PHONE_MASS
    m.CreateDiagonalInertiaAttr(Gf.Vec3f(M * (b * b + c * c) / 12, M * (a * a + c * c) / 12, M * (a * a + b * b) / 12))
    p.AddAppliedSchema("PhysxContactReportAPI")
    p.CreateAttribute("physxContactReport:threshold", Sdf.ValueTypeNames.Float).Set(0.0)
    r = "/phone"
    band = material(stage, f"{r}/Looks/band", BAND, roughness=0.35, metallic=0.6)
    back = material(stage, f"{r}/Looks/back", WHITE_BACK, roughness=0.15)
    ring = material(stage, f"{r}/Looks/ring", LENS_RING, roughness=0.3, metallic=0.5)
    glass = material(stage, f"{r}/Looks/glass", LENS_GLASS, roughness=0.05)
    flash = material(stage, f"{r}/Looks/flash", (240, 226, 170), roughness=0.3)
    screen = material(stage, f"{r}/Looks/screen", (1, 1, 1), roughness=0.08, texture=f"./textures/{SCREEN_TEX}")
    # the body: the front-view outline (y, z) with rounded corners, through the thickness (x)
    outline = rounded_rect(g.PW, g.PH, g.CORNER_R)
    X, Y, Z = np.eye(3)
    prism(stage, f"{r}/body", outline, -g.PT / 2, g.PT / 2, (np.zeros(3), Y, Z, X), band)
    # the back glass (visual, 0.1 mm proud) and the screen (visual, 0.1 mm proud)
    inner = rounded_rect(g.PW - 0.0012, g.PH - 0.0012, g.CORNER_R - 0.0006)
    prism(stage, f"{r}/back_glass", inner, -g.PT / 2 - 0.0001, -g.PT / 2 + 0.0002, (np.zeros(3), Y, Z, X), back,
          collide=False)
    gw, gh = 0.06942, 0.14461
    xs = g.PT / 2 + 0.0002
    cs = [(xs, -gw / 2, -gh / 2), (xs, gw / 2, -gh / 2), (xs, gw / 2, gh / 2), (xs, -gw / 2, gh / 2)]
    quad(stage, f"{r}/screen", cs, [(0, 0), (1, 0), (1, 1), (0, 1)], screen)
    # the camera plateau (collider), 0.73 mm proud, 3.43 mm in from the top and side edges to 36.30 / 34.13 mm
    y0, y1 = g.PW / 2 - g.PLATEAU_SIDE, g.PW / 2 - 0.00343
    z0, z1 = g.PH / 2 - g.PLATEAU_TOP, g.PH / 2 - 0.00343
    pl = rounded_rect(y1 - y0, z1 - z0, 0.006)
    prism(stage, f"{r}/plateau", pl, -g.PT / 2 - g.PLATEAU_PROUD, -g.PT / 2,
          (np.array([0.0, (y0 + y1) / 2, (z0 + z1) / 2]), Y, Z, X), back)
    # the wide and ultra-wide lenses: rings 1.51 mm proud of the back glass (colliders), black glass (visual)
    yl = g.PW / 2 - g.LENS_FROM_SIDE
    for name, zt in (("lens_wide", g.LENS_FROM_TOP), ("lens_ultra", g.ULTRA_FROM_TOP)):
        zl = g.PH / 2 - zt
        prism(stage, f"{r}/{name}", disc(g.LENS_D / 2 + 0.0008), -g.PT / 2 - g.LENS_PROUD, -g.PT / 2,
              (np.array([0.0, yl, zl]), Y, Z, X), ring)
        prism(stage, f"{r}/{name}_glass", disc(g.LENS_D / 2 - 0.0015), -g.PT / 2 - g.LENS_PROUD - 0.0001,
              -g.PT / 2 - g.LENS_PROUD + 0.0003, (np.array([0.0, yl, zl]), Y, Z, X), glass, collide=False)
    prism(stage, f"{r}/flash", disc(0.00283, 12), -g.PT / 2 - g.PLATEAU_PROUD - 0.0001, -g.PT / 2,
          (np.array([0.0, g.PW / 2 - g.FLASH[1], g.PH / 2 - g.FLASH[0]]), Y, Z, X), flash, collide=False)
    # the buttons (visual): volume up / down on the -y edge, the side button on the +y edge
    for name, (a, bnd), side in (("vol_up", g.VOL_UP, -1), ("vol_down", g.VOL_DOWN, -1), ("side", g.SIDE_BTN, 1)):
        zc = g.PH / 2 - (a + bnd) / 2
        box_mesh(stage, f"{r}/{name}", (0.0, side * (g.PW / 2 + 0.0003), zc), np.eye(3),
                 (0.0026, 0.0008, bnd - a), band, collide=False)
    stage.Save()


# -- the fixtures -------------------------------------------------------------------------------------------------------
def build_fixtures():
    """Origin at the station frame's origin on the table top (x, y as in the station frame, z above the table)."""
    stage, root = stage_for("fixtures")
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.RigidBodyAPI(root.GetPrim()).CreateKinematicEnabledAttr(True)
    r = "/fixtures"
    mat_m = material(stage, f"{r}/Looks/mat", MAT, roughness=0.95)
    tape = material(stage, f"{r}/Looks/tape", TAPE, roughness=0.7)
    alu = material(stage, f"{r}/Looks/alu", ALU, roughness=0.45, metallic=0.6)
    acr = material(stage, f"{r}/Looks/acrylic", ACRYLIC, roughness=0.2)
    steel = material(stage, f"{r}/Looks/steel", STEEL, roughness=0.3, metallic=0.8)
    wood = material(stage, f"{r}/Looks/block", BLOCK, roughness=0.8)
    paper_back = material(stage, f"{r}/Looks/paper_back", (240, 240, 236), roughness=0.9)
    I3 = np.eye(3)
    # the mat and its cross
    mx, my, mt = g.MAT
    box_mesh(stage, f"{r}/mat", (g.MAT_XY[0], g.MAT_XY[1], mt / 2), I3, g.MAT, mat_m)
    for k, size in enumerate(((g.CROSS_ARM, g.CROSS_W, 0.0003), (g.CROSS_W, g.CROSS_ARM, 0.0003))):
        box_mesh(stage, f"{r}/cross_{k}", (g.CROSS[0], g.CROSS[1], mt + 0.00015), I3, size, tape, collide=False)
    # the stand
    sx, sy = g.STAND_XY
    box_mesh(stage, f"{r}/stand_base", (sx, sy, mt + g.BASE_T / 2), I3, (g.BASE_L, g.BASE_W, g.BASE_T), alu)
    _, R = g.phone_in_stand()
    n, yv, u = R[:, 0], R[:, 1], R[:, 2]
    O = np.array([g.STAND_SEAT_X, sy, g.SEAT_UP])
    T = g.PLATE_T
    w = 0.070

    def cradle_box(name, n0, n1, u0, u1, width):
        c = O + n * (n0 + n1) / 2 + u * (u0 + u1) / 2
        box_mesh(stage, f"{r}/{name}", c, np.column_stack([n, yv, u]), (n1 - n0, width, u1 - u0), alu)

    slot = g.SLOT
    cradle_box("stand_back", -T, 0.0, -T, g.BACK_L, w)
    cradle_box("stand_seat", 0.0, slot, -T, 0.0, w)
    cradle_box("stand_lip", slot, slot + T, -T, g.LIP_H, 0.060)
    a = np.array([sx - 0.030, sy, mt + g.BASE_T])           # the arm: the base's rear to the back plate's rear
    b = O - n * T + u * 0.050
    d = b - a
    L = float(np.linalg.norm(d))
    ax = d / L
    side = np.cross(yv, ax)
    box_mesh(stage, f"{r}/stand_arm", (a + b) / 2 - side * 0.003, np.column_stack([ax, yv, side]), (L, 0.040, 0.006),
             alu)
    # the boards, their holders and rods
    for i in range(3):
        u2 = g.bearing_vec(g.BEARINGS[i])
        th = math.atan2(u2[1], u2[0])
        Rb = g.rot_z(th)                                    # x along the sight line (away from the cross), y to its left
        c = g.board_xy(i)
        sheet_c = np.array([c[0], c[1], g.SHEET_Z0 + g.SHEET_H / 2])
        box_mesh(stage, f"{r}/board_{i + 1}", sheet_c + Rb @ np.array([0.003, 0, 0]), Rb, (0.006, g.SHEET_W, g.SHEET_H),
                 paper_back)
        tex = material(stage, f"{r}/Looks/board_{i + 1}", (1, 1, 1), roughness=0.85,
                       texture=f"./textures/{board_tex(i)}")
        # the print's face toward the cross; seen from the cross, image left is the sight line's +y side
        f = sheet_c - Rb @ np.array([0.0002, 0, 0])
        hw, hh = g.SHEET_W / 2, g.SHEET_H / 2
        ly = Rb[:, 1]
        cs = [f + ly * hw - np.array([0, 0, hh]), f - ly * hw - np.array([0, 0, hh]), f - ly * hw + np.array([0, 0, hh]),
              f + ly * hw + np.array([0, 0, hh])]
        quad(stage, f"{r}/print_{i + 1}", cs, [(0, 0), (1, 0), (1, 1), (0, 1)], tex)
        box_mesh(stage, f"{r}/holder_{i + 1}", np.array([c[0], c[1], g.HOLDER_H / 2]) + Rb @ np.array([0.003, 0, 0]), Rb,
                 (g.HOLDER_D, g.HOLDER_W, g.HOLDER_H), acr)
        for s, nm in ((-1, "front"), (1, "back")):
            box_mesh(stage, f"{r}/holder_{i + 1}_{nm}", sheet_c + Rb @ np.array([0.003 + s * 0.0035, 0, 0])
                     - np.array([0, 0, g.SHEET_H / 2 - 0.035]), Rb, (0.001, g.SHEET_W + 0.01, 0.070), acr,
                     collide=False)
        rx, ry = g.rod_xy(i)
        bx, by, bz = g.ROD_BLOCK
        box_mesh(stage, f"{r}/rod_block_{i + 1}", (rx, ry, bz / 2), Rb, g.ROD_BLOCK, wood)
        prism(stage, f"{r}/rod_{i + 1}", disc(g.ROD_D / 2, 20), 0.0, g.ROD_LEN,
              (np.array([rx, ry, 0.0]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])), steel)
    stage.Save()


if __name__ == "__main__":
    printables()
    if "--printables" in sys.argv:
        raise SystemExit
    write_textures()
    build_phone()
    build_fixtures()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
