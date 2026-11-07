"""Write the scene's textures and USD assets from geometry.py.

    python build_assets.py          # needs pxr, PIL (Isaac Sim's Python) and the DejaVu fonts, not the simulator

- assets/rack.usda: the rack, fixed (kinematic). Two posts whose front faces have real square holes: each post is
  built from strips and bridges with gaps where the holes are, and solid behind, so a clip goes in only at a hole.
  The installed units (a 2U server at U1-U2, a 1U switch at U3, a 1U patch panel at U8) are part of it. Each post
  carries its unit numbers and unit lines on the strip beside its holes.
- assets/stand.usda: two slotted wooden uprights on a base board, fixed, holding the panel upright.
- assets/panel.usda: the 1U blanking panel, a free rigid body. Its origin is the middle of its back face; a clip on
  each end (8 mm square, chamfered to 5 mm at the tip) sticks 12 mm out of the back at mid-height.

Every asset's origin is the station origin on the table top (the panel's is its own), so task.py places the rack
and the stand at (0, 0, TOP).
"""

from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"
TEX = OUT / "textures"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_REG = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

POST = (0.20, 0.21, 0.23)
POST_INSIDE = (0.03, 0.03, 0.035)
PANEL = (0.05, 0.05, 0.055)
WOOD = (0.62, 0.47, 0.30)
PX_PER_M = 4000


# -- textures ------------------------------------------------------------------------------------------------------------
def post_strip(path: Path) -> None:
    """The strip beside each post's holes: a white line at every unit boundary and the unit number in each unit."""
    lo, hi = 0.020, g.TOP_Z + 0.012
    w = round((g.POST_OUT - (g.PY + g.HOLE / 2)) * PX_PER_M)
    h = round((hi - lo) * PX_PER_M)
    im = Image.new("RGB", (w, h), tuple(round(c * 255) for c in POST))
    d = ImageDraw.Draw(im)
    f = ImageFont.truetype(FONT, 40)
    row = lambda z: (hi - z) * PX_PER_M                  # noqa: E731
    for u in range(g.NU + 1):
        y = row(g.RZ0 + u * g.U)
        d.line([(0, y), (w, y)], fill=(235, 235, 230), width=4)
    for u in range(1, g.NU + 1):
        d.text((w / 2, row(g.RZ0 + (u - 0.5) * g.U)), str(u), font=f, fill=(245, 245, 240), anchor="mm")
    im.save(path)


def bezel(path: Path, kind: str, units: int) -> None:
    """The front of an installed unit, 19 inches wide; image left is the robot's left."""
    w = round(g.PANEL_W * PX_PER_M / 2)
    h = round((units * g.U - 0.0008) * PX_PER_M / 2)
    base = {"server": (52, 56, 62), "switch": (40, 52, 66), "patch": (28, 29, 31)}[kind]
    im = Image.new("RGB", (w, h), base)
    d = ImageDraw.Draw(im)
    s = PX_PER_M / 2                                      # pixels per metre at this size
    ear = (g.PANEL_W / 2 - g.POST_IN) * s
    for x in (ear / 2 + 4, w - ear / 2 - 4):              # a screw in the middle hole of each unit, both ears
        for k in range(units):
            cy = h - (k * g.U + g.HOLE_OFF[1]) * s
            d.ellipse([x - 9, cy - 9, x + 9, cy + 9], fill=(170, 174, 180), outline=(90, 92, 96), width=2)
            d.line([(x - 6, cy), (x + 6, cy)], fill=(80, 82, 86), width=3)
    d.line([(ear, 0), (ear, h)], fill=(20, 20, 22), width=3)
    d.line([(w - ear, 0), (w - ear, h)], fill=(20, 20, 22), width=3)
    x0, x1 = ear + 30, w - ear - 30
    if kind == "server":
        bw = (x1 - x0 - 3 * 14) / 4
        for r in range(2):
            for c in range(4):
                bx = x0 + c * (bw + 14)
                by = 14 + r * (h - 28) / 2
                d.rectangle([bx, by, bx + bw, by + (h - 42) / 2], fill=(118, 124, 132), outline=(30, 30, 34), width=3)
                d.rectangle([bx + 10, by + 10, bx + 70, by + 22], fill=(40, 42, 46))
                d.ellipse([bx + bw - 26, by + 10, bx + bw - 14, by + 22], fill=(70, 220, 120))
    elif kind == "switch":
        pw, gap = 30, 6
        for r in range(2):
            for c in range(12):
                px = x0 + 40 + c * (pw + gap) + (c // 6) * 24
                py = 18 + r * 36
                d.rectangle([px, py, px + pw, py + 26], fill=(8, 9, 11))
        for c in range(4):
            px = x1 - 4 * 54 + c * 54
            d.rectangle([px, 26, px + 40, 52], fill=(8, 9, 11), outline=(120, 124, 130), width=2)
        for c in range(0, 12, 2):
            d.ellipse([x0 + 46 + c * 36, 6, x0 + 54 + c * 36, 14], fill=(240, 180, 60) if c % 4 else (80, 220, 120))
    else:
        fs = ImageFont.truetype(FONT_REG, 16)
        pw = (x1 - x0) / 24
        for c in range(24):
            px = x0 + c * pw
            d.rectangle([px + 6, 30, px + pw - 6, 62], fill=(6, 6, 7), outline=(90, 92, 96), width=1)
            d.text((px + pw / 2, 18), str(c + 1), font=fs, fill=(220, 220, 214), anchor="mm")
    im.save(path)


# -- USD helpers -------------------------------------------------------------------------------------------------------
def stage_for(name: str, mass: float | None = None):
    path = OUT / f"{name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    if mass is not None:
        UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(mass)
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
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(mat)


def box(stage, path, lo, hi, mat, collide=True):
    """An axis-aligned box from corner ``lo`` to corner ``hi``."""
    size = [b - a for a, b in zip(lo, hi)]
    center = [(a + b) / 2 for a, b in zip(lo, hi)]
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*center))
    c.AddScaleOp().Set(Gf.Vec3f(*size))
    bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def face(stage, path, x, y_left, y_right, z0, z1, mat):
    """A textured quad facing the arms (-x), visual only; image left is the robot's left (+y)."""
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(x, y_left, z0), Gf.Vec3f(x, y_right, z0), Gf.Vec3f(x, y_right, z1),
                        Gf.Vec3f(x, y_left, z1)])
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    m.CreateSubdivisionSchemeAttr("none")
    st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    m.CreateDoubleSidedAttr(True)
    bind(m, mat)
    return m


def frustum_x(stage, path, x0, x1, half0, half1, cy, cz, mat):
    """A square frustum along +x (a chamfered clip tip): half-width ``half0`` at x0 narrowing to ``half1`` at x1."""
    v = []
    for x, h in ((x0, half0), (x1, half1)):
        v += [(x, cy - h, cz - h), (x, cy + h, cz - h), (x, cy + h, cz + h), (x, cy - h, cz + h)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in v])
    m.CreateFaceVertexCountsAttr([4] * 6)
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    bind(m, mat)
    UsdPhysics.CollisionAPI.Apply(m.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
    return m


# -- assets ---------------------------------------------------------------------------------------------------------------
def build_rack():
    stage, _ = stage_for("rack")
    r = "/rack"
    post = material(stage, f"{r}/Looks/post", POST, roughness=0.45, metallic=0.3)
    inside = material(stage, f"{r}/Looks/inside", POST_INSIDE, roughness=0.8)
    post_strip(TEX / "post_strip.png")
    strip = material(stage, f"{r}/Looks/strip", (1, 1, 1), roughness=0.5, texture="./textures/post_strip.png")
    z_lo, z_hi = 0.020, g.TOP_Z + 0.012
    x0, x1, x2 = g.RX, g.RX + g.POST_D, g.RX + g.POST_D + g.POST_BACK
    h0, h1 = g.PY - g.HOLE / 2, g.PY + g.HOLE / 2
    holes = [g.hole_z(h) for h in range(1, 3 * g.NU + 1)]
    for s, side in ((1, "left"), (-1, "right")):
        def ys(a, b):
            return (min(s * a, s * b), max(s * a, s * b))
        p = f"{r}/{side}_post"
        ya, yb = ys(g.POST_IN, h0)
        box(stage, f"{p}/inner", (x0, ya, z_lo), (x1, yb, z_hi), post)
        ya, yb = ys(h1, g.POST_OUT)
        box(stage, f"{p}/outer", (x0, ya, z_lo), (x1, yb, z_hi), post)
        ya, yb = ys(h0, h1)
        edges = [z_lo] + [z for hz in holes for z in (hz - g.HOLE / 2, hz + g.HOLE / 2)] + [z_hi]
        for i in range(0, len(edges), 2):                 # the bridges between the holes
            box(stage, f"{p}/bridge_{i // 2:02d}", (x0, ya, edges[i]), (x1, yb, edges[i + 1]), post)
        ya, yb = ys(g.POST_IN, g.POST_OUT)
        box(stage, f"{p}/back", (x1, ya, z_lo), (x2, yb, z_hi), inside)
        yl, yr = (g.POST_OUT, h1) if s > 0 else (-h1, -g.POST_OUT)
        face(stage, f"{p}/numbers", x0 - 0.0003, yl, yr, z_lo, z_hi, strip)
        fy = s * (g.POST_IN + g.POST_OUT) / 2
        box(stage, f"{p}/foot", (x0 - 0.10, fy - 0.02, 0.0), (x0 + 0.20, fy + 0.02, z_lo), post)
    box(stage, f"{r}/top_bar", (x0, -g.POST_OUT, z_hi), (x2, g.POST_OUT, z_hi + g.BAR_H), post)

    for kind, u0, n in g.INSTALLED:                       # the installed units: a bezel with ears, a body behind
        bezel(TEX / f"{kind}.png", kind, n)
        look = material(stage, f"{r}/Looks/{kind}", (1, 1, 1), roughness=0.5, texture=f"./textures/{kind}.png")
        dark = material(stage, f"{r}/Looks/{kind}_body", (0.12, 0.13, 0.14), roughness=0.6)
        zb, zt = g.RZ0 + (u0 - 1) * g.U + 0.0004, g.RZ0 + (u0 - 1 + n) * g.U - 0.0004
        depth = {"server": 0.28, "switch": 0.20, "patch": 0.06}[kind]
        box(stage, f"{r}/{kind}/bezel", (x0 - 0.004, -g.PANEL_W / 2, zb), (x0, g.PANEL_W / 2, zt), dark)
        face(stage, f"{r}/{kind}/front", x0 - 0.0042, g.PANEL_W / 2, -g.PANEL_W / 2, zb, zt, look)
        box(stage, f"{r}/{kind}/body", (x0, -g.POST_IN + 0.001, zb), (x0 + depth, g.POST_IN - 0.001, zt), dark)
    stage.GetRootLayer().Save()


def build_stand():
    stage, _ = stage_for("stand")
    r = "/stand"
    wood = material(stage, f"{r}/Looks/wood", WOOD, roughness=0.75)
    xb, xf = g.STAND_X, g.STAND_X - g.PANEL_T              # the panel's back and front faces in the slot
    s0, s1 = xf - (g.SLOT - g.PANEL_T) / 2, xb + (g.SLOT - g.PANEL_T) / 2
    front, back = s0 - (g.STAND_D - g.SLOT) / 2, s1 + (g.STAND_D - g.SLOT) / 2
    box(stage, f"{r}/board", (front - 0.01, -g.STAND_Y - 0.035, 0.0), (back + 0.01, g.STAND_Y + 0.035, 0.012), wood)
    for s, side in ((1, "left"), (-1, "right")):
        y0, y1 = s * g.STAND_Y - g.STAND_W / 2, s * g.STAND_Y + g.STAND_W / 2
        box(stage, f"{r}/{side}_base", (front, y0, 0.012), (back, y1, g.SLOT_BOTTOM), wood)
        box(stage, f"{r}/{side}_front", (front, y0, g.SLOT_BOTTOM), (s0, y1, g.STAND_H), wood)
        box(stage, f"{r}/{side}_back", (s1, y0, g.SLOT_BOTTOM), (back, y1, g.STAND_H), wood)
    stage.GetRootLayer().Save()


def build_panel():
    stage, _ = stage_for("panel", g.PANEL_MASS)
    r = "/panel"
    black = material(stage, f"{r}/Looks/black", PANEL, roughness=0.55)
    groove = material(stage, f"{r}/Looks/groove", (0.015, 0.015, 0.017), roughness=0.7)
    clip = material(stage, f"{r}/Looks/clip", (0.80, 0.82, 0.84), roughness=0.4)
    w, h, t = g.PANEL_W / 2, g.PANEL_H / 2, g.PANEL_T
    box(stage, f"{r}/plate", (-t, -w, -h), (0.0, w, h), black)
    for i in range(-3, 4):                                # vent grooves across the middle, visual only
        box(stage, f"{r}/groove_{i + 3}", (-t - 0.0003, -0.17, i * 0.005 - 0.0011), (-t, 0.17, i * 0.005 + 0.0011),
            groove, collide=False)
    half = g.PEG / 2
    for s, side in ((1, "left"), (-1, "right")):
        y = s * g.PY
        box(stage, f"{r}/{side}_clip", (0.0, y - half, -half), (g.PEG_L - 0.003, y + half, half), clip)
        frustum_x(stage, f"{r}/{side}_clip_tip", g.PEG_L - 0.003, g.PEG_L, half, 0.0025, y, 0.0, clip)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    TEX.mkdir(parents=True, exist_ok=True)
    build_rack()
    build_stand()
    build_panel()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
