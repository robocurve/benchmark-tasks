"""Write the scene's textures and USD assets from geometry.py.

    python build_assets.py          # needs pxr, PIL (Isaac Sim's Python) and the DejaVu fonts, not the simulator

- assets/textures/*.png: the printed decals (the housing's warning label, the brake label beside the hand guard, the bar
  and chain, the battery, the scabbard's top).
- assets/chainsaw.usda: a floating articulation, the saw lying on its side. Its root is the housing, bar and handles
  (colliders: housing, battery, cover, bar plate, chain, handles); its one joint, ``guard``, is the hand guard: a
  revolute hinge about a vertical axis (the chain brake lever). task.py turns the guard's angle into the brake.
- assets/scabbard.usda: the bar cover, a free rigid body: a plastic channel (colliders: top, bottom, two walls and a
  closed end), open at its +y end.

The saw's origin is the point on the table under the bar's root axis (the housing rests on its z = 0 face). The
scabbard's origin is the middle of its length and of its channel. Colliders are boxes and cylinders; the printed decals
and visual-only shapes are meshes.
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
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_PLAIN = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

ORANGE = (0.93, 0.40, 0.05)
GREY_D = (0.15, 0.16, 0.17)
GREY_M = (0.34, 0.35, 0.37)
STEEL = (0.62, 0.64, 0.67)
CHAIN = (0.09, 0.09, 0.10)
RED = (0.82, 0.10, 0.07)
PLASTIC = (0.96, 0.78, 0.08)       # the scabbard: yellow polypropylene
INK = (24, 27, 31)


# -- textures ----------------------------------------------------------------------------------------------------------
def _font(size, bold=True):
    return ImageFont.truetype(FONT if bold else FONT_PLAIN, size)


def warning_label(path: Path) -> None:
    """On the housing's exposed top face: the carry warning. Image right is -y (toward the bar), up is +x."""
    w, h = 900, 1000
    im = Image.new("RGB", (w, h), (246, 244, 236))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, w, 150], fill=(232, 150, 20))
    d.polygon([(70, 128), (160, 14), (250, 128)], fill=INK)
    d.text((160, 96), "!", font=_font(88), fill=(232, 150, 20), anchor="mm")
    d.text((290, 76), "WARNING", font=_font(96), fill=INK, anchor="lm")
    lines = ["Before carrying or", "transporting this saw:", "", "1  Engage the chain brake", "    (push the hand guard",
             "    forward until it clicks)", "2  Fit the bar cover", "", "Never carry it with the",
             "chain uncovered."]
    y = 200
    for ln in lines:
        d.text((50, y), ln, font=_font(54, bold=ln[:1].isdigit()), fill=INK, anchor="lm")
        y += 78
    d.rectangle([0, h - 70, w, h], fill=(32, 36, 42))
    d.text((w / 2, h - 35), "READ THE MANUAL BEFORE USE", font=_font(40), fill=(246, 244, 236), anchor="mm")
    im.save(path)


def brake_label(path: Path) -> None:
    """Beside the hand guard: CHAIN BRAKE with an arrow toward the bar tip (image right = -y)."""
    w, h = 900, 400
    im = Image.new("RGB", (w, h), (236, 236, 232))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, w, 92], fill=RED_RGB)
    d.text((w / 2, 46), "CHAIN BRAKE", font=_font(66), fill=(255, 255, 255), anchor="mm")
    d.text((60, 190), "ON", font=_font(70), fill=INK, anchor="lm")
    d.line([(190, 190), (700, 190)], fill=INK, width=24)
    d.polygon([(690, 130), (830, 190), (690, 250)], fill=INK)
    d.text((60, 310), "OFF", font=_font(70), fill=INK, anchor="lm")
    d.line([(700, 310), (250, 310)], fill=(120, 124, 130), width=16)
    d.polygon([(260, 262), (170, 310), (260, 358)], fill=(120, 124, 130))
    im.save(path)


RED_RGB = tuple(int(255 * v) for v in RED)


def bar_texture(path: Path) -> None:
    """The bar and its chain seen from above: bar plate in the middle, the chain's drive links along both edges, text
    along the bar. Image right is -y (toward the tip), up is +x."""
    w, h = 2500, 320
    im = Image.new("RGB", (w, h), tuple(int(255 * v) for v in STEEL))
    d = ImageDraw.Draw(im)
    edge = int(h * g.CHAIN_OVER / (2 * g.BAR_OUT_HALF_W))
    for y0, y1 in ((0, edge), (h - edge, h)):
        d.rectangle([0, y0, w, y1], fill=tuple(int(255 * v) for v in CHAIN))
        for k in range(0, w, 34):                                   # the cutters' tops
            d.rectangle([k, y0, k + 15, y1], fill=(150, 152, 158))
    d.text((120, h / 2), "SAW CHAIN 3/8 LP  |  GUIDE BAR 10 IN / 25 CM", font=_font(86), fill=INK, anchor="lm")
    d.ellipse([w - 150, h / 2 - 28, w - 94, h / 2 + 28], outline=INK, width=8)
    im.save(path)


def battery_label(path: Path) -> None:
    w, h = 460, 880
    im = Image.new("RGB", (w, h), tuple(int(255 * v) for v in GREY_D))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, w, 120], fill=tuple(int(255 * v) for v in ORANGE))
    d.text((w / 2, 60), "FORESTAR", font=_font(80), fill=(255, 255, 255), anchor="mm")
    d.text((w / 2, 330), "18V  4.0Ah", font=_font(72), fill=(240, 240, 240), anchor="mm")
    d.text((w / 2, 450), "LI-ION", font=_font(60), fill=(200, 200, 200), anchor="mm")
    d.text((w / 2, 760), "CAUTION: remove for", font=_font(40, False), fill=(200, 200, 200), anchor="mm")
    d.text((w / 2, 810), "chain service", font=_font(40, False), fill=(200, 200, 200), anchor="mm")
    im.save(path)


def scabbard_top(path: Path) -> None:
    """The scabbard's top face: text from the open end (left) to the closed end (right); right is -y, up is +x."""
    w, h = 2650, 520
    im = Image.new("RGB", (w, h), tuple(int(255 * v) for v in PLASTIC))
    d = ImageDraw.Draw(im)
    for k in range(-h, w, 120):                                      # hazard stripes along the closed end
        if k > w - 520:
            d.polygon([(k, h), (k + 60, h), (k + 60 + h, 0), (k + h, 0)], fill=INK)
    d.text((110, h / 2 - 60), "BAR COVER  10 IN / 25 CM", font=_font(120), fill=INK, anchor="lm")
    d.text((110, h / 2 + 90), "Fit before carrying or storing.  Keep the chain covered.", font=_font(70, False),
           fill=INK, anchor="lm")
    d.polygon([(2200, 80), (2330, 250), (2200, 420)], fill=INK)      # points to the closed end (-y)
    im.save(path)


# -- USD helpers (as in the reference task) --------------------------------------------------------------------------
MIRROR = 1.0        # -1 while building the mirror-image saw (layout B): every y is reflected through the saw's origin


def stage_for(name: str, file: str | None = None):
    path = OUT / f"{file or name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    return stage, root


def material(stage, path: str, color, roughness=0.5, texture: str | None = None):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
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


def refl(p):
    """A saw-frame point as built: y reflected in the mirror-image saw."""
    return (p[0], MIRROR * p[1], p[2])


def body(stage, path: str, mass: float, pos=(0.0, 0.0, 0.0)):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*refl(pos)))
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    m = UsdPhysics.MassAPI.Apply(xf.GetPrim())
    m.CreateMassAttr(mass)
    return xf


def box(stage, path, center, size, color, mat=None, collide=True):
    """An axis-aligned box in its parent's frame: centre and size (x, y, z). Colliders are real boxes."""
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*refl(center)))
    c.AddScaleOp().Set(Gf.Vec3f(*size))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def cyl(stage, path, center, radius, height, axis, color, mat=None, collide=True):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(height)
    c.CreateAxisAttr(axis)
    c.AddTranslateOp().Set(Gf.Vec3d(*refl(center)))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def decal(stage, path, center, w, h, mat, u=(0.0, -1.0, 0.0), v=(1.0, 0.0, 0.0)):
    """A flat textured quad facing up (+z): ``u`` is the image's right in the parent's frame, ``v`` its up (the top
    camera's view: right is -y, up is +x), ``w`` x ``h`` metres, centred on ``center``. Visual only."""
    cu, cv = np.asarray(refl(u), float), np.asarray(v, float)
    c = np.asarray(refl(center), float)
    pts = [c - cu * w / 2 - cv * h / 2, c + cu * w / 2 - cv * h / 2, c + cu * w / 2 + cv * h / 2,
           c - cu * w / 2 + cv * h / 2]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    m.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    UsdGeom.Mesh(m).SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    m.CreateDoubleSidedAttr(True)
    bind(m, mat)
    return m


def revolute(stage, path, body0, body1, pos0, axis="Z", limits_deg=None, stiffness=0.0, damping=0.0):
    j = UsdPhysics.RevoluteJoint.Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    # the mirror-image saw's hinge axis is turned over (a 180 degree turn about x), so the angles keep their meaning:
    # negative still swings the guard toward the bar tip
    flip = Gf.Quatf(1, 0, 0, 0) if MIRROR > 0 else Gf.Quatf(0, 1, 0, 0)
    j.CreateLocalPos0Attr(Gf.Vec3f(*refl(pos0)))
    j.CreateLocalRot0Attr(flip)
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(flip)
    if limits_deg is not None:
        j.CreateLowerLimitAttr(limits_deg[0])
        j.CreateUpperLimitAttr(limits_deg[1])
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateTargetPositionAttr(0.0)
    return j


# -- the saw ---------------------------------------------------------------------------------------------------------
def build_chainsaw(mirror: bool = False):
    global MIRROR
    MIRROR = -1.0 if mirror else 1.0
    stage, root = stage_for("chainsaw", "chainsaw_b" if mirror else "chainsaw")
    r = "/chainsaw"
    saw_mass = g.SAW_MASS - g.GUARD_MASS
    prim = body(stage, f"{r}/saw", saw_mass).GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxArticulationAPI")
    prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    prim.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    prim.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    s = f"{r}/saw"

    orange = material(stage, f"{r}/Looks/orange", ORANGE, roughness=0.45)
    dark = material(stage, f"{r}/Looks/dark", GREY_D, roughness=0.55)
    mid = material(stage, f"{r}/Looks/mid", GREY_M, roughness=0.5)
    steel = material(stage, f"{r}/Looks/steel", STEEL, roughness=0.3)
    chain = material(stage, f"{r}/Looks/chain", CHAIN, roughness=0.4)
    red = material(stage, f"{r}/Looks/red", RED, roughness=0.4)

    def mm(a):                                  # centre and size from (lo, hi) pairs
        return (a[0] + a[1]) / 2, a[1] - a[0]

    hx, sx = mm(g.HOUSING_X); hy, sy = mm(g.HOUSING_Y); hz, sz = mm(g.HOUSING_Z)
    box(stage, f"{s}/housing", (hx, hy, hz), (sx, sy, sz), ORANGE, orange)
    bx, sbx = mm(g.BATTERY_X); by, sby = mm(g.BATTERY_Y); bz, sbz = mm(g.BATTERY_Z)
    box(stage, f"{s}/battery", (bx, by, bz), (sbx, sby, sbz), GREY_D, dark)
    cx, scx = mm(g.COVER_X); cy, scy = mm(g.COVER_Y); cz, scz = mm(g.COVER_Z)
    box(stage, f"{s}/sprocket_cover", (cx, cy, cz), (scx, scy, scz), GREY_M, mid)

    # the guide bar: its plate (inside and out to the tip) and the chain round its rim, in the horizontal plane
    plate_y0, plate_y1 = -(g.BAR_LEN - g.BAR_WIDTH / 2), g.BAR_INSIDE
    box(stage, f"{s}/bar_plate", (0.0, (plate_y0 + plate_y1) / 2, g.BAR_Z), (g.BAR_WIDTH, plate_y1 - plate_y0, g.BAR_PLATE_T),
        STEEL, steel)
    cyl(stage, f"{s}/bar_nose", (0.0, plate_y0, g.BAR_Z), g.BAR_WIDTH / 2, g.BAR_PLATE_T, "Z", STEEL, steel)
    edge_x = g.BAR_WIDTH / 2 + g.CHAIN_OVER / 2 - 0.0005
    for sign, nm in ((1, "L"), (-1, "R")):
        box(stage, f"{s}/chain_{nm}", (sign * edge_x, (plate_y0 + 0.0) / 2, g.BAR_Z),
            (g.CHAIN_OVER + 0.001, -plate_y0, g.CHAIN_H), CHAIN, chain)
    cyl(stage, f"{s}/chain_nose", (0.0, plate_y0, g.BAR_Z), g.BAR_OUT_HALF_W, g.CHAIN_H, "Z", CHAIN, chain)

    # handles: the front handle over the housing's top edge, the rear handle behind
    f = g.FRONT_HANDLE
    for y in (f["y0"], f["y1"]):
        box(stage, f"{s}/front_post_{int(round(y * 1000))}", ((g.HOUSING_X[1] + f["x"]) / 2, y, f["z"]),
            (f["x"] - g.HOUSING_X[1], 0.026, 0.050), GREY_D, dark)
    cyl(stage, f"{s}/front_handle", (f["x"], (f["y0"] + f["y1"]) / 2, f["z"]), f["r"], f["y1"] - f["y0"] + 0.026, "Y",
        GREY_D, dark)
    rh = g.REAR_HANDLE
    box(stage, f"{s}/rear_handle", (rh["x"] - 0.0, (rh["y0"] + rh["y1"]) / 2, rh["z"]),
        (0.040, rh["y1"] - rh["y0"], 0.070), GREY_D, dark)
    # the hinge's bracket on the housing's edge (visual only)
    px, py = g.GUARD_PIVOT
    box(stage, f"{s}/hinge_bracket", (g.HOUSING_X[1] + (px - g.HOUSING_X[1]) / 2, py, 0.0425),
        (px - g.HOUSING_X[1] + 0.012, 0.030, 0.050), GREY_D, dark, collide=False)

    # printed decals
    warning_label(TEX / "warning.png")
    brake_label(TEX / "brake.png")
    bar_texture(TEX / "bar.png")
    battery_label(TEX / "battery.png")
    warn = material(stage, f"{r}/Looks/warning", (1, 1, 1), 0.6, "./textures/warning.png")
    brk = material(stage, f"{r}/Looks/brake", (1, 1, 1), 0.6, "./textures/brake.png")
    bart = material(stage, f"{r}/Looks/bar_tex", (1, 1, 1), 0.35, "./textures/bar.png")
    batt = material(stage, f"{r}/Looks/battery_tex", (1, 1, 1), 0.6, "./textures/battery.png")
    ztop = g.HOUSING_Z[1] + 0.0004
    decal(stage, f"{s}/warning", (0.020, 0.1425, ztop), 0.110, 0.100 * 1.1, warn)     # w along y (-y right), h along x
    decal(stage, f"{s}/brake_label", (0.078, 0.040, ztop), 0.090, 0.040, brk)
    decal(stage, f"{s}/bar_top", (0.0, (g.BAR_INSIDE - g.BAR_LEN) / 2, g.BAR_Z + g.CHAIN_H / 2 + 0.0003),
          g.BAR_LEN + g.BAR_INSIDE, 2 * g.BAR_OUT_HALF_W, bart)
    decal(stage, f"{s}/battery_top", (0.0275, (g.BATTERY_Y[0] + g.BATTERY_Y[1]) / 2, g.BATTERY_Z[1] + 0.0004),
          g.BATTERY_Y[1] - g.BATTERY_Y[0] - 0.004, 0.100, batt)

    # the hand guard: a plate on a vertical hinge; its free end has a thick grip bar
    gprim = body(stage, f"{r}/guard", g.GUARD_MASS, pos=(px, py, 0.0425)).GetPrim()
    plate_len = g.GUARD_LEN - g.GUARD_BAR_LEN
    box(stage, f"{r}/guard/plate", (plate_len / 2 + 0.002, 0.0, 0.0), (plate_len, g.GUARD_T, g.GUARD_H), RED, red)
    box(stage, f"{r}/guard/grip_bar", (g.GUARD_LEN - g.GUARD_BAR_LEN / 2, 0.0, 0.0),
        (g.GUARD_BAR_LEN, g.GUARD_BAR_T, g.GUARD_H), RED, red)
    cyl(stage, f"{r}/guard/boss", (0.0, 0.0, 0.0), 0.0060, g.GUARD_H, "Z", RED, red)
    revolute(stage, f"{r}/joints/guard", s, f"{r}/guard", (px, py, 0.0425), "Z",
             limits_deg=(math.degrees(g.GUARD_LIMITS[0]), math.degrees(g.GUARD_LIMITS[1])),
             stiffness=0.0, damping=g.GUARD_DAMP)
    stage.GetRootLayer().Save()
    MIRROR = 1.0


# -- the scabbard -----------------------------------------------------------------------------------------------------
def build_scabbard():
    stage, root = stage_for("scabbard")
    r = "/scabbard"
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(g.SCAB_MASS)
    yel = material(stage, f"{r}/Looks/plastic", PLASTIC, roughness=0.45)
    iw, ih, w, L = g.SCAB_IN_W, g.SCAB_IN_H, g.SCAB_WALL, g.SCAB_LEN
    ow, oh = g.SCAB_OUT_W, g.SCAB_OUT_H
    box(stage, f"{r}/bottom", (0, 0, -(ih / 2 + w / 2)), (ow, L, w), PLASTIC, yel)
    box(stage, f"{r}/top", (0, 0, ih / 2 + w / 2), (ow, L, w), PLASTIC, yel)
    box(stage, f"{r}/wall_left", (-(iw / 2 + w / 2), 0, 0), (w, L, ih), PLASTIC, yel)
    box(stage, f"{r}/wall_right", (iw / 2 + w / 2, 0, 0), (w, L, ih), PLASTIC, yel)
    box(stage, f"{r}/end_cap", (0, -L / 2 + w / 2, 0), (iw, w, ih), PLASTIC, yel)
    scabbard_top(TEX / "scabbard.png")
    top = material(stage, f"{r}/Looks/top_tex", (1, 1, 1), 0.5, "./textures/scabbard.png")
    decal(stage, f"{r}/label", (0.0, 0.0, oh / 2 + 0.0004), L - 0.004, ow - 0.004, top)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    TEX.mkdir(parents=True, exist_ok=True)
    build_chainsaw()
    build_chainsaw(mirror=True)                 # layout B
    build_scabbard()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
