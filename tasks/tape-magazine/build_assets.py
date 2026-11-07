"""Write the scene's textures and USD assets from geometry.py.

    python build_assets.py          # needs pxr, PIL (Isaac Sim's Python) and the DejaVu fonts, not the simulator

- assets/tape_1.usda .. tape_4.usda: LTO cartridges, free rigid bodies. Each one's origin is the middle of its label
  face's bottom edge, x toward its front end; the barcode label (AI0001L9 ...) is on the label face, printed along its
  height, as on a cartridge standing on edge.
- assets/magazine.usda: the five-slot magazine, fixed. Each slot has a stop at the back and lead-in chamfers on its
  walls, and a ramp leads up to the floor's front edge. The top stops 15 mm short of the front, so fingers can push
  a cartridge home; the slot numbers are printed along the top's front edge.
- assets/tote.usda: a tote with four bays, fixed; low rails either side of each bay keep its cartridge upright.

Every fixed asset's origin is the station origin on the table top, so task.py places them at (0, 0, TOP).
"""

from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"
TEX = OUT / "textures"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"
FONT_SANS = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

SHELL = (0.10, 0.11, 0.13)
DOOR = (0.40, 0.42, 0.45)
MAG = (0.17, 0.18, 0.20)
LIP = (0.82, 0.84, 0.86)
TOTE = (0.30, 0.42, 0.55)

# Code 39 patterns (wide/narrow, bar first) for the characters a barcode label uses
CODE39 = {"0": "nnnwwnwnn", "1": "wnnwnnnnw", "2": "nnwwnnnnw", "3": "wnwwnnnnn", "4": "nnnwwnnnw", "5": "wnnwwnnnn",
          "6": "nnwwwnnnn", "7": "nnnwnnwnw", "8": "wnnwnnwnn", "9": "nnwwnnwnn", "A": "wnnnnwnnw", "I": "nnwnnwwnn",
          "L": "nnnnnwwnw", "*": "nwnnwnwnn"}


# -- textures ------------------------------------------------------------------------------------------------------------
def label(path: Path, code: str) -> None:
    """An LTO barcode label (text above a Code 39 barcode), turned to run up the face of a cartridge on edge: the text
    reads from the bottom up."""
    w, h = 1200, 240
    im = Image.new("RGB", (w, h), (248, 248, 244))
    d = ImageDraw.Draw(im)
    f = ImageFont.truetype(FONT, 92)
    d.text((w / 2, 62), code, font=f, fill=(12, 12, 12), anchor="mm")
    bars = "*" + code + "*"
    unit = (w - 80) / (len(bars) * 16)                  # 3 wide (3 units) + 6 narrow + a gap, per character
    x = 40.0
    for ch in bars:
        for i, kind in enumerate(CODE39[ch]):
            bw = unit * (3 if kind == "w" else 1)
            if i % 2 == 0:
                d.rectangle([x, 122, x + bw, 222], fill=(12, 12, 12))
            x += bw
        x += unit
    im.rotate(90, expand=True).save(path)


def lip_numbers(path: Path) -> None:
    """The slot numbers along the front of the magazine's top, 1 on the robot's left (image left), read from the arms."""
    half = g.SLOTS / 2 * g.SLOT_PITCH + 0.0035
    w, h = 1200, round(1200 * 0.022 / (2 * half))
    im = Image.new("RGB", (w, h), tuple(round(c * 255) for c in LIP))
    d = ImageDraw.Draw(im)
    f = ImageFont.truetype(FONT_SANS, round(h * 0.8))
    for k in range(1, g.SLOTS + 1):
        x = (half - g.slot_y(k)) / (2 * half) * w
        d.text((x, h / 2), str(k), font=f, fill=(20, 22, 26), anchor="mm")
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


def hull(stage, path, points, mat):
    """A convex solid through eight corners (a box's, in the order -x face then +x face, each going round)."""
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    m.CreateFaceVertexCountsAttr([4] * 6)
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDoubleSidedAttr(True)
    bind(m, mat)
    UsdPhysics.CollisionAPI.Apply(m.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
    return m


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


def up(stage, path, x_near, x_far, y_left, y_right, z, mat):
    """A textured quad facing up, visual only; read from the arms: image left is +y, image top is the far edge."""
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(x_near, y_left, z), Gf.Vec3f(x_near, y_right, z), Gf.Vec3f(x_far, y_right, z),
                        Gf.Vec3f(x_far, y_left, z)])
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    m.CreateSubdivisionSchemeAttr("none")
    st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    m.CreateDoubleSidedAttr(True)
    bind(m, mat)
    return m


# -- assets ---------------------------------------------------------------------------------------------------------------
def build_tape(k: int):
    code = g.BARCODES[k - 1]
    stage, _ = stage_for(f"tape_{k}", g.TAPE_MASS)
    r = f"/tape_{k}"
    shell = material(stage, f"{r}/Looks/shell", SHELL, roughness=0.45)
    door = material(stage, f"{r}/Looks/door", DOOR, roughness=0.4, metallic=0.4)
    tab = material(stage, f"{r}/Looks/tab", (0.80, 0.12, 0.10), roughness=0.5)
    label(TEX / f"label_{k}.png", code)
    lab = material(stage, f"{r}/Looks/label", (1, 1, 1), roughness=0.6, texture=f"./textures/label_{k}.png")
    L, H, T = g.TAPE_L, g.TAPE_H, g.TAPE_T
    box(stage, f"{r}/shell", (0.0, -T / 2, 0.0), (L, T / 2, H), shell)
    box(stage, f"{r}/door", (L - 0.040, -T / 2 - 0.0004, 0.30 * H), (L - 0.004, -T / 2, 0.85 * H), door, collide=False)
    box(stage, f"{r}/write_protect", (-0.0012, -0.003, 0.006), (0.0, 0.003, 0.014), tab, collide=False)
    face(stage, f"{r}/label", -0.0004, 0.42 * T, -0.42 * T, 0.12 * H, 0.90 * H, lab)
    stage.GetRootLayer().Save()


def build_magazine():
    stage, _ = stage_for("magazine")
    r = "/magazine"
    body = material(stage, f"{r}/Looks/body", MAG, roughness=0.55)
    lip_numbers(TEX / "slots.png")
    lip = material(stage, f"{r}/Looks/lip", (1, 1, 1), roughness=0.5, texture="./textures/slots.png")
    x0, x1 = g.MAG_X, g.MAG_X + g.MAG_D
    z0, z1 = g.MAG_FLOOR, g.MAG_FLOOR + g.SLOT_H
    half = g.SLOTS / 2 * g.SLOT_PITCH
    wall = g.SLOT_PITCH - g.SLOT_W
    box(stage, f"{r}/floor", (x0, -half - 0.0035, 0.0), (x1, half + 0.0035, z0), body)
    box(stage, f"{r}/top", (x0 + g.NOTCH, -half - 0.0035, z1), (x1, half + 0.0035, z1 + 0.004), body)   # finger room
    stop = g.MAG_X + g.SLOT_IN + g.TAPE_L
    box(stage, f"{r}/back", (stop, -half - 0.0035, z0), (x1 + 0.004, half + 0.0035, z1), body)
    lead, tip = 0.016, 0.0005                              # the walls taper to 0.5 mm over their front 16 mm
    for i in range(g.SLOTS + 1):                           # the walls between and beside the slots
        y = half - i * g.SLOT_PITCH
        lo_y, hi_y = y - wall / 2, y + wall / 2
        if i == 0:
            lo_y, hi_y = y - wall / 2, y + 0.0035
        elif i == g.SLOTS:
            lo_y, hi_y = y - 0.0035, y + wall / 2
        box(stage, f"{r}/wall_{i}", (x0 + lead, lo_y, z0), (stop, hi_y, z1), body)
        a, b = y - tip / 2, y + tip / 2
        if i == 0:                                         # the outer walls taper on their inner side only
            a, b = hi_y - tip, hi_y
        elif i == g.SLOTS:
            a, b = lo_y, lo_y + tip
        hull(stage, f"{r}/lead_{i}", [(x0, a, z0), (x0, b, z0), (x0, b, z1), (x0, a, z1),
                                      (x0 + lead, lo_y, z0), (x0 + lead, hi_y, z0), (x0 + lead, hi_y, z1),
                                      (x0 + lead, lo_y, z1)], body)
    hull(stage, f"{r}/ramp", [(x0 - 0.008, -half - 0.0035, 0.0), (x0 - 0.008, half + 0.0035, 0.0),
                              (x0 - 0.008, half + 0.0035, 0.0005), (x0 - 0.008, -half - 0.0035, 0.0005),
                              (x0, -half - 0.0035, 0.0), (x0, half + 0.0035, 0.0), (x0, half + 0.0035, z0),
                              (x0, -half - 0.0035, z0)], body)      # a ramp up to the floor's front edge
    up(stage, f"{r}/numbers", x0 + g.NOTCH + 0.002, x0 + g.NOTCH + 0.024, half + 0.0035, -half - 0.0035, z1 + 0.0043, lip)
    stage.GetRootLayer().Save()


def build_tote():
    stage, _ = stage_for("tote")
    r = "/tote"
    blue = material(stage, f"{r}/Looks/tote", TOTE, roughness=0.7)
    x0, x1 = g.TOTE_X - 0.006, g.TOTE_X + g.TAPE_L + 0.006
    hy = g.TOTE_HALF_W
    box(stage, f"{r}/floor", (x0, -hy, 0.0), (x1, hy, g.TOTE_FLOOR), blue)
    box(stage, f"{r}/front", (x0 - 0.003, -hy, 0.0), (x0, hy, g.TOTE_WALL), blue)
    box(stage, f"{r}/back", (x1, -hy, 0.0), (x1 + 0.003, hy, g.TOTE_WALL), blue)
    for s, side in ((1, "left"), (-1, "right")):
        box(stage, f"{r}/{side}", (x0 - 0.003, min(s * hy, s * (hy + 0.003)), 0.0),
            (x1 + 0.003, max(s * hy, s * (hy + 0.003)), g.TOTE_WALL), blue)
    for i, y in enumerate((0.07, 0.0, -0.07)):
        box(stage, f"{r}/divider_{i}", (x0, y - 0.001, g.TOTE_FLOOR), (x1, y + 0.001, g.DIVIDER_H), blue)
    for i, y in enumerate(g.BAY_Y):                         # rails either side of each cartridge, 2 mm clear
        for s in (1, -1):
            c = y + s * (g.TAPE_T / 2 + 0.002 + 0.002)
            box(stage, f"{r}/rail_{i}_{'l' if s > 0 else 'r'}", (x0, c - 0.002, g.TOTE_FLOOR),
                (x1, c + 0.002, g.TOTE_FLOOR + 0.010), blue)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    TEX.mkdir(parents=True, exist_ok=True)
    for k in range(1, 5):
        build_tape(k)
    build_magazine()
    build_tote()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
