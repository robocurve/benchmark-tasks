"""USD assets, written with plain ``pxr`` and PIL after Isaac Sim has started: the blister cards (articulations: the
card and one foil flap per filled cell), the medicine cup (a rigid body) and the printed medication record (a fixed
sheet).

A card's stage (default prim ``/Card``, the articulation root, a floating base):
- ``/Card/card``: the root body, origin at the centre of its underside; the card is a slab with a square opening
  through each cell, built from boxes.
- ``/Card/flap_<cell>_link``: the foil under a filled cell, hinged at the cell's far edge
  (``/Card/joints/flap_<cell>``, revolute about y, 0 = flat, negative = bent down).
- ``/Card/dome_<cell>_link``: the bubble over a filled cell, its face just above the tablet; it slides down into the
  opening (``/Card/joints/dome_<cell>``, prismatic along z, 0 at rest, negative = pressed), so the tablet is enclosed
  and a fingertip pressing the bubble works against its spring.
"""

from __future__ import annotations

import math
import tempfile
import textwrap
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from blister import design as d

FRICTION = (1.2, 1.0)  # static, dynamic: as yam_arena's graspable objects
FONTS = Path(__file__).resolve().parent.parent / "assets" / "fonts"
OUT = Path(tempfile.gettempdir()) / "yam_tasks" / "blister"
PX_PER_M = 4000  # texture resolution: 40 px per cm


def linear_rgb(srgb) -> tuple[float, float, float]:
    return tuple(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in srgb)


class Stage:
    """A new stage with materials and primitive helpers (boxes and quads in a body's frame)."""

    def __init__(self, root: str):
        self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(self.stage, 1.0)
        self.root = root
        self.stage.SetDefaultPrim(UsdGeom.Xform.Define(self.stage, root).GetPrim())
        self._mats: dict[str, UsdShade.Material] = {}
        self.grip = UsdShade.Material.Define(self.stage, f"{root}/Looks/grip")
        api = UsdPhysics.MaterialAPI.Apply(self.grip.GetPrim())
        api.CreateStaticFrictionAttr(FRICTION[0])
        api.CreateDynamicFrictionAttr(FRICTION[1])
        api.CreateRestitutionAttr(0.0)

    def material(self, rgb=(1, 1, 1), texture: str | None = None, opacity: float = 1.0) -> UsdShade.Material:
        key = texture or "c" + "".join(f"{round(c * 255):02x}" for c in rgb) + f"_{round(opacity * 100)}"
        key = key.replace(".", "_").replace("/", "_")
        if key in self._mats:
            return self._mats[key]
        base = f"{self.root}/Looks/{key}"
        mat = UsdShade.Material.Define(self.stage, base)
        surf = UsdShade.Shader.Define(self.stage, f"{base}/surface")
        surf.CreateIdAttr("UsdPreviewSurface")
        surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
        if texture:
            reader = UsdShade.Shader.Define(self.stage, f"{base}/st")
            reader.CreateIdAttr("UsdPrimvarReader_float2")
            reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            tex = UsdShade.Shader.Define(self.stage, f"{base}/image")
            tex.CreateIdAttr("UsdUVTexture")
            tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./{texture}")
            tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
            tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
            surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
                tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))  # fmt: skip
        else:
            surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(rgb)))
        if opacity < 1.0:
            surf.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
        mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
        self._mats[key] = mat
        return mat

    def body(self, path: str, pos, mass: float) -> str:
        xf = UsdGeom.Xform.Define(self.stage, path)
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
        UsdPhysics.MassAPI.Apply(xf.GetPrim()).CreateMassAttr(float(mass))
        return path

    def box(self, parent: str, name: str, size, pos, rgb, yaw: float = 0.0, collide: bool = True) -> None:
        cube = UsdGeom.Cube.Define(self.stage, f"{parent}/{name}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*pos))
        if yaw:
            cube.AddRotateZOp().Set(math.degrees(yaw))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(self.material(rgb))
        if collide:
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(self.grip, UsdShade.Tokens.weakerThanDescendants, "physics")

    def quad(self, parent: str, name: str, size_xy, z: float, mat) -> None:
        """A visual rectangle facing up, centred on the parent's origin: image up along +x, image right along -y."""
        sx, sy = size_xy
        mesh = UsdGeom.Mesh.Define(self.stage, f"{parent}/{name}")
        mesh.CreatePointsAttr([(-sx / 2, sy / 2, z), (-sx / 2, -sy / 2, z), (sx / 2, -sy / 2, z), (sx / 2, sy / 2, z)])
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        st.Set([(0, 0), (1, 0), (1, 1), (0, 1)])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)

    def save(self, name: str) -> str:
        path = OUT / f"{name}.usda"
        path.parent.mkdir(parents=True, exist_ok=True)
        self.stage.GetRootLayer().Export(str(path))
        return str(path)


def _font(size: int, mono: bool = False):
    from PIL import ImageFont

    return ImageFont.truetype(str(FONTS / ("DejaVuSansMono-Bold.ttf" if mono else "DejaVuSans-Bold.ttf")), size)


def _px(m: float) -> int:
    return round(m * PX_PER_M)


# -- blister card ----------------------------------------------------------------------------------------------------


def card_image(card: d.Card):
    """The card's printed face: pharmacy label at the bottom (the card's -x end), the 30 numbered cells above it."""
    from PIL import Image, ImageDraw

    L, W, _ = d.CARD_SIZE
    img = Image.new("RGB", (_px(W), _px(L)), (250, 250, 247))
    draw = ImageDraw.Draw(img)

    def to_px(x, y):  # card frame -> image (right = -y, up = +x)
        return _px(W / 2 - y), _px(L / 2 - x)

    band = tuple(round(c * 255) for c in card.band_color)
    top = _px(L - d.LABEL_LENGTH)
    draw.rectangle((0, top, img.width, top + 26), fill=band)
    draw.text((img.width / 2, top + 13), f"{d.RESIDENT}  {d.ROOM}", font=_font(17), fill=(255, 255, 255), anchor="mm")
    draw.text((img.width / 2, top + 52), card.drug, font=_font(22 if len(card.drug) < 14 else 15), fill=(10, 10, 10),
              anchor="mm")  # fmt: skip
    draw.text((img.width / 2, top + 82), card.strength, font=_font(18), fill=(10, 10, 10), anchor="mm")
    for k, line in enumerate(textwrap.wrap(card.directions, 22)):
        draw.text((img.width / 2, top + 112 + 22 * k), line, font=_font(15), fill=(40, 40, 40), anchor="mm")
    draw.rectangle((0, 0, img.width - 1, img.height - 1), outline=band, width=6)
    h = _px(d.HOLE) / 2
    for cell in range(1, d.ROWS * d.COLS + 1):
        u, v = to_px(*d.cell_xy(cell))
        full = cell >= card.first_full
        draw.rectangle((u - h, v - h, u + h, v + h), fill=(200, 202, 206) if full else (70, 70, 74),
                       outline=(150, 150, 155), width=2)  # fmt: skip
        draw.text((u, v + h + 8), str(cell), font=_font(13, mono=True), fill=(30, 30, 30), anchor="mm")
    return img


def build_card(card: d.Card) -> str:
    """The card articulation for ``card``; returns the USD path."""
    s = Stage("/Card")
    UsdPhysics.ArticulationRootAPI.Apply(s.stage.GetPrimAtPath("/Card"))
    L, W, T = d.CARD_SIZE
    body = s.body("/Card/card", (0, 0, 0), d.CARD_MASS)
    white = (0.96, 0.96, 0.94)
    # The slab around the openings: full-width strips between rows, and short blocks between a row's openings.
    rows_x = [d.ROW0_X + r * d.CELL_PITCH[0] for r in range(d.ROWS)]
    edges_x = [-L / 2] + [e for x in rows_x for e in (x - d.HOLE / 2, x + d.HOLE / 2)] + [L / 2]
    for k in range(0, len(edges_x), 2):
        a, b = edges_x[k], edges_x[k + 1]
        s.box(body, f"strip_{k // 2}", (b - a, W, T), ((a + b) / 2, 0, T / 2), white)
    cols_y = [(1 - c) * d.CELL_PITCH[1] for c in range(d.COLS)]
    edges_y = [-W / 2] + [e for y in sorted(cols_y) for e in (y - d.HOLE / 2, y + d.HOLE / 2)] + [W / 2]
    for r, x in enumerate(rows_x):
        for k in range(0, len(edges_y), 2):
            a, b = edges_y[k], edges_y[k + 1]
            s.box(body, f"block_{r}_{k // 2}", (d.HOLE, b - a, T), (x, (a + b) / 2, T / 2), white)
    tex = f"card_{card.name}.png"
    OUT.mkdir(parents=True, exist_ok=True)
    card_image(card).save(OUT / tex)
    s.quad(body, "print", (L, W), T + 0.0002, s.material(texture=tex))
    UsdGeom.Scope.Define(s.stage, "/Card/joints")
    dome = s.material((0.92, 0.95, 1.0), opacity=0.35)
    for cell in card.full_cells():
        x, y = d.cell_xy(cell)
        hinge = (x + d.HOLE / 2, y, d.FLAP_T / 2)
        link = s.body(f"/Card/flap_{cell}_link", hinge, d.FLAP_MASS)
        s.box(link, "foil", (d.HOLE, d.HOLE, d.FLAP_T), (-d.HOLE / 2, 0, 0), (0.78, 0.79, 0.82))
        j = UsdPhysics.RevoluteJoint.Define(s.stage, f"/Card/joints/{d.flap_joint(cell)}")
        j.CreateAxisAttr("Y")
        j.CreateBody0Rel().SetTargets([Sdf.Path(body)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(link)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*hinge))
        j.CreateLocalRot0Attr(Gf.Quatf(1.0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0))
        j.CreateLowerLimitAttr(-math.degrees(d.FLAP_LIMIT))
        j.CreateUpperLimitAttr(0.0)
        UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular").CreateTypeAttr("force")
        # The bubble: a dome that slides down into the opening, its face just above the tablet.
        at = (x, y, d.DOME_Z)
        link = s.body(f"/Card/dome_{cell}_link", at, d.DOME_MASS)
        s.box(link, "face", d.DOME, (0, 0, 0), (0.92, 0.95, 1.0))
        cyl = UsdGeom.Cylinder.Define(s.stage, f"{link}/bubble")
        cyl.CreateRadiusAttr(d.HOLE / 2)
        top, bottom = d.DOME[2] / 2, T - d.DOME_Z  # from its face down to the card's top face (link frame)
        cyl.CreateHeightAttr(top - bottom)
        cyl.AddTranslateOp().Set(Gf.Vec3d(0, 0, (top + bottom) / 2))
        UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(dome)
        j = UsdPhysics.PrismaticJoint.Define(s.stage, f"/Card/joints/{d.dome_joint(cell)}")
        j.CreateAxisAttr("Z")
        j.CreateBody0Rel().SetTargets([Sdf.Path(body)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(link)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*at))
        j.CreateLocalRot0Attr(Gf.Quatf(1.0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0))
        j.CreateLowerLimitAttr(-d.DOME_TRAVEL)
        j.CreateUpperLimitAttr(0.0)
        UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear").CreateTypeAttr("force")
    return s.save(f"card_{card.name}")


# -- medicine cup ----------------------------------------------------------------------------------------------------


def build_cup() -> str:
    """A 1 oz paper cup as one rigid body: a disc floor and a ring of wall staves. Origin at the floor's centre."""
    s = Stage("/Cup")
    prim = s.stage.GetPrimAtPath("/Cup")
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(d.CUP_MASS)
    prim.AddAppliedSchema("PhysxRigidBodyAPI")  # PhysX solver settings, authored in this USD
    prim.CreateAttribute("physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.UInt).Set(
        d.CUP_SOLVER_ITERATIONS)
    prim.CreateAttribute("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float).Set(0.5)
    white = (0.97, 0.97, 0.95)
    r, h, t, n = d.CUP_RADIUS, d.CUP_HEIGHT, d.CUP_WALL, d.CUP_STAVES
    s.box("/Cup", "floor", (2 * r, 2 * r, t), (0, 0, t / 2), white)
    width = 2 * (r + t) * math.tan(math.pi / n) * 1.05
    for k in range(n):
        a = 2 * math.pi * k / n
        s.box("/Cup", f"wall_{k}", (t, width, h), ((r + t / 2) * math.cos(a), (r + t / 2) * math.sin(a), h / 2),
              white, yaw=a)  # fmt: skip
    return s.save("cup")


# -- the medication administration record ---------------------------------------------------------------------------

def _fit(draw, text: str, size: int, width: float, mono: bool = False):
    """The font at ``size``, or smaller so ``text`` fits ``width`` px."""
    while draw.textlength(text, font=_font(size, mono)) > width:
        size -= 2
    return _font(size, mono)


def mar_image():
    """Large print (drug names about 1 cm tall, doses and times 1.1 cm), so a wrist camera can read it."""
    from PIL import Image, ImageDraw

    w, h = _px(d.MAR_SIZE[1]), _px(d.MAR_SIZE[0])
    img = Image.new("RGB", (w, h), (248, 248, 244))
    draw = ImageDraw.Draw(img)
    ink, grey = (10, 10, 10), (70, 70, 70)
    draw.rectangle((0, 0, w - 1, h - 1), outline=ink, width=8)
    draw.text((w / 2, 44), "MEDICATION ADMINISTRATION RECORD", font=_fit(draw, "M" * 33, 40, w - 80), fill=grey,
              anchor="mm")  # fmt: skip
    draw.text((w / 2, 112), f"{d.RESIDENT}   {d.ROOM}", font=_font(60), fill=ink, anchor="mm")
    cols = (36, 650, 870)  # medication, dose, times
    for x, head in zip(cols, ("MEDICATION", "DOSE", "TIME"), strict=True):
        draw.text((x, 190), head, font=_font(34), fill=grey, anchor="lm")
    draw.line((24, 218, w - 24, 218), fill=ink, width=5)
    row = (h - 230) / len(d.CARDS)
    for k, card in enumerate(d.CARDS):
        n, times = d.MAR[card.name]
        y = 230 + row * (k + 0.5)
        draw.text((cols[0], y - 34), card.drug, font=_fit(draw, card.drug, 58, cols[1] - cols[0] - 24), fill=ink,
                  anchor="lm")  # fmt: skip
        draw.text((cols[0], y + 36), card.strength + ", by mouth", font=_font(38), fill=grey, anchor="lm")
        draw.text((cols[1], y), f"{n} tab", font=_font(62), fill=ink, anchor="lm")
        for j, t in enumerate(times):
            draw.text((cols[2], y + 64 * (j - (len(times) - 1) / 2)), t, font=_font(62, mono=True), fill=ink,
                      anchor="lm")  # fmt: skip
        if k:
            draw.line((24, 230 + row * k, w - 24, 230 + row * k), fill=(150, 150, 150), width=3)
    return img


def build_mar() -> str:
    s = Stage("/Sheet")
    sx, sy, sz = d.MAR_SIZE
    s.box("/Sheet", "paper", (sx, sy, sz), (0, 0, sz / 2), (0.95, 0.95, 0.93))
    OUT.mkdir(parents=True, exist_ok=True)
    mar_image().save(OUT / "mar.png")
    s.quad("/Sheet", "print", (sx, sy), sz + 0.0002, s.material(texture="mar.png"))
    return s.save("mar")
