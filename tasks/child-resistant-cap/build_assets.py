"""Write the scene's textures and USD asset from geometry.py.

    python build_assets.py          # needs pxr, PIL (Isaac Sim's Python) and the DejaVu fonts, not the simulator

- assets/textures/cap_top.png: the cap's top, printed PUSH DOWN & TURN around an anticlockwise arrow.
- assets/textures/cap_side.png: the cap's ribbed side.
- assets/textures/label.png: the vial's pharmacy label.
- assets/textures/neck.png, pills.png: the vial's threaded neck, and the tablets seen through its open mouth.
- assets/pill_bottle.usda: a floating articulation (the vial stands free on the table). The cap hangs off the vial
  on three joints: ``lift`` (prismatic, up its axis; task.py keeps it locked until the cap is free), ``spin``
  (revolute, free, about its axis) and ``press`` (prismatic, down 3 mm against a spring). task.py turns the
  turning while pressed into unscrewing. The vial's collider stops below the cap, so the two never overlap.

The asset's origin is the centre of the vial's base, so task.py places it at (BOTTLE_X, BOTTLE_Y, TOP).
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

AMBER = (0.60, 0.30, 0.06)
CAP_WHITE = (0.94, 0.94, 0.92)
INK = (24, 27, 31)


# -- textures ------------------------------------------------------------------------------------------------------------
def cap_top(path: Path) -> None:
    """The cap's top, as seen from above with the robot's forward (+x) up the image."""
    n = 1024
    c = n / 2
    im = Image.new("RGB", (n, n), tuple(int(255 * v) for v in CAP_WHITE))
    d = ImageDraw.Draw(im)
    d.ellipse([18, 18, n - 18, n - 18], outline=(178, 182, 188), width=10)
    f = ImageFont.truetype(FONT, 78)
    text = "PUSH DOWN & TURN  •  PUSH DOWN & TURN  •  "
    r = 380
    widths = [d.textlength(ch, font=f) for ch in text]
    total = sum(widths)
    a = math.pi / 2 + (total / r) / 2                      # centred on the top, running clockwise like the hands
    for ch, w in zip(text, widths):
        a -= w / 2 / r
        x, y = c + r * math.cos(a), c - r * math.sin(a)
        glyph = Image.new("RGBA", (120, 120), (0, 0, 0, 0))
        ImageDraw.Draw(glyph).text((60, 60), ch, font=f, fill=INK, anchor="mm")
        glyph = glyph.rotate(math.degrees(a - math.pi / 2), resample=Image.BICUBIC)
        im.paste(glyph, (int(x - 60), int(y - 60)), glyph)
        a -= w / 2 / r
    # an anticlockwise arrow round the middle, and a down arrow inside it
    d.arc([c - 200, c - 200, c + 200, c + 200], start=-60, end=200, fill=INK, width=34)
    ax, ay = c + 200 * math.cos(math.radians(-60)), c + 200 * math.sin(math.radians(-60))
    d.polygon([(ax - 62, ay - 8), (ax + 30, ay - 52), (ax + 26, ay + 56)], fill=INK)
    d.rectangle([c - 22, c - 120, c + 22, c + 40], fill=(180, 92, 18))
    d.polygon([(c - 70, c + 30), (c + 70, c + 30), (c, c + 120)], fill=(180, 92, 18))
    im.save(path)


def cap_side(path: Path) -> None:
    """The cap's side unrolled: fine vertical ribs."""
    w, h = 1024, 96
    im = Image.new("RGB", (w, h), tuple(int(255 * v) for v in CAP_WHITE))
    d = ImageDraw.Draw(im)
    for k in range(72):
        x = k * w / 72
        d.rectangle([x, 6, x + 4, h - 6], fill=(196, 199, 204))
    im.save(path)


def neck_thread(path: Path) -> None:
    """The neck unrolled: amber with a lighter two-start thread climbing round it."""
    w, h = 1024, 64
    im = Image.new("RGB", (w, h), tuple(int(255 * v) for v in AMBER))
    d = ImageDraw.Draw(im)
    for start in (0, w // 2):
        for k in (-1, 0, 1):
            x0 = start + k * w
            d.line([(x0, h - 6), (x0 + w // 2, 6)], fill=(222, 150, 64), width=7)
    im.save(path)


def pills(path: Path) -> None:
    """Looking down into the open vial: tablets on the dark amber bottom (image up is +x, like the cap's top)."""
    n = 512
    im = Image.new("RGB", (n, n), (58, 26, 6))
    rng = np.random.default_rng(7)
    for _ in range(14):
        x, y = rng.uniform(70, n - 70, 2)
        ang = rng.uniform(0, 180)
        tab = Image.new("RGBA", (150, 150), (0, 0, 0, 0))
        td = ImageDraw.Draw(tab)
        td.ellipse([18, 48, 132, 102], fill=(238, 234, 224), outline=(196, 190, 178), width=4)
        td.line([(75, 52), (75, 98)], fill=(206, 200, 188), width=3)
        tab = tab.rotate(ang, resample=Image.BICUBIC)
        im.paste(tab, (int(x - 75), int(y - 75)), tab)
    im.save(path)


def label(path: Path) -> None:
    """The pharmacy label, unrolled round the vial (image x runs anticlockwise seen from above)."""
    w, h = 1400, 520
    im = Image.new("RGB", (w, h), (250, 249, 244))
    d = ImageDraw.Draw(im)
    big, mid, small = ImageFont.truetype(FONT, 96), ImageFont.truetype(FONT, 46), ImageFont.truetype(FONT_PLAIN, 40)
    d.rectangle([0, 0, w, 92], fill=(40, 84, 140))
    d.text((40, 46), "PHARMACY", font=mid, fill=(255, 255, 255), anchor="lm")
    d.text((w - 40, 46), "Rx 0427", font=mid, fill=(255, 255, 255), anchor="rm")
    d.text((60, 190), "Rx", font=big, fill=INK, anchor="lm")
    d.text((250, 160), "TAKE 1 TABLET", font=mid, fill=INK, anchor="lm")
    d.text((250, 222), "BY MOUTH DAILY", font=mid, fill=INK, anchor="lm")
    for k, wd in enumerate((900, 760, 820)):
        d.rectangle([60, 290 + 44 * k, 60 + wd, 306 + 44 * k], fill=(170, 175, 182))
    d.rectangle([0, h - 92, w, h], fill=(214, 74, 32))
    d.text((w / 2, h - 46), "KEEP OUT OF REACH OF CHILDREN", font=small, fill=(255, 255, 255), anchor="mm")
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


def body(stage, path: str, mass: float, pos=(0.0, 0.0, 0.0), inertia=None):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    m = UsdPhysics.MassAPI.Apply(xf.GetPrim())
    m.CreateMassAttr(mass)
    if inertia is not None:                             # links with no collider need their inertia given
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia, inertia, inertia))
        m.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    return xf


def cylinder(stage, path, radius, height, z, color, collide=True):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(height)
    c.CreateAxisAttr("Z")
    c.AddTranslateOp().Set(Gf.Vec3d(0, 0, z))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def band(stage, path, radius, z0, z1, a0, a1, mat, segs=96):
    """A textured band round the z axis, from angle a0 to a1 (anticlockwise seen from above); image x runs with the
    angle, image y down from z1 to z0."""
    pts, uvs, idx, counts = [], [], [], []
    for j in range(segs + 1):
        a = a0 + (a1 - a0) * j / segs
        for z in (z0, z1):
            pts.append(Gf.Vec3f(radius * math.cos(a), radius * math.sin(a), z))
    for j in range(segs):
        p, q, r, s = 2 * j, 2 * j + 2, 2 * j + 3, 2 * j + 1
        counts.append(4)
        idx += [p, q, r, s]
        u0, u1 = j / segs, (j + 1) / segs
        uvs += [Gf.Vec2f(u0, 0), Gf.Vec2f(u1, 0), Gf.Vec2f(u1, 1), Gf.Vec2f(u0, 1)]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr(counts)
    m.CreateFaceVertexIndicesAttr(idx)
    st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    st.Set(uvs)
    m.CreateDoubleSidedAttr(True)
    bind(m, mat)
    return m


def disc(stage, path, radius, z, mat, segs=96):
    """A textured disc facing up; the image's up is +x (forward) and its right is -y (the robot's right)."""
    pts = [Gf.Vec3f(0, 0, z)] + [Gf.Vec3f(radius * math.cos(a), radius * math.sin(a), z)
                                 for a in np.linspace(0, 2 * math.pi, segs, endpoint=False)]
    uv = [Gf.Vec2f(0.5, 0.5)] + [Gf.Vec2f(0.5 - 0.5 * math.sin(a), 0.5 + 0.5 * math.cos(a))
                                 for a in np.linspace(0, 2 * math.pi, segs, endpoint=False)]
    idx = [i for k in range(segs) for i in (0, 1 + k, 1 + (k + 1) % segs)]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr([3] * segs)
    m.CreateFaceVertexIndicesAttr(idx)
    m.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * len(idx))
    UsdGeom.Mesh(m).SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    st.Set([uv[i] for i in idx])
    m.CreateDoubleSidedAttr(True)
    bind(m, mat)
    return m


def ring(stage, path, r_in, r_out, z, mat, segs=96):
    """A flat ring facing up, between two radii."""
    pts, idx = [], []
    for j in range(segs):
        a = 2 * math.pi * j / segs
        pts += [Gf.Vec3f(r_in * math.cos(a), r_in * math.sin(a), z), Gf.Vec3f(r_out * math.cos(a), r_out * math.sin(a), z)]
    for j in range(segs):
        k = (j + 1) % segs
        idx += [2 * j, 2 * j + 1, 2 * k + 1, 2 * k]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr([4] * segs)
    m.CreateFaceVertexIndicesAttr(idx)
    m.CreateDoubleSidedAttr(True)
    bind(m, mat)
    return m


def joint(stage, path, kind, body0, body1, pos0, limits=None, stiffness=0.0, damping=0.0):
    j = (UsdPhysics.PrismaticJoint if kind == "linear" else UsdPhysics.RevoluteJoint).Define(stage, path)
    j.CreateAxisAttr("Z")
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    if limits is not None:
        j.CreateLowerLimitAttr(limits[0])
        j.CreateUpperLimitAttr(limits[1])
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), kind)
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateTargetPositionAttr(0.0)
    return j


# -- the asset ---------------------------------------------------------------------------------------------------------
def build_pill_bottle():
    stage, root = stage_for("pill_bottle")
    r = "/pill_bottle"
    # the vial: amber body and neck, its label round the front. It is the articulation's root (floating), so the
    # task places it by its base
    prim = body(stage, f"{r}/bottle", g.BOTTLE_MASS).GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxArticulationAPI")
    prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    prim.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    prim.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    amber = material(stage, f"{r}/Looks/amber", AMBER, roughness=0.3)
    inside = material(stage, f"{r}/Looks/inside", (0.20, 0.09, 0.02), roughness=0.5)
    neck_thread(TEX / "neck.png")
    pills(TEX / "pills.png")
    neck_mat = material(stage, f"{r}/Looks/neck", (1, 1, 1), roughness=0.3, texture="./textures/neck.png")
    pills_mat = material(stage, f"{r}/Looks/pills", (1, 1, 1), roughness=0.6, texture="./textures/pills.png")
    mouth = g.NECK_R - 0.0015                                                   # the neck's wall is 1.5 mm thick
    band(stage, f"{r}/bottle/body", g.BODY_R, 0.0, g.BODY_H, 0.0, 2 * math.pi, amber)
    ring(stage, f"{r}/bottle/base", 0.0, g.BODY_R, 0.0005, amber)
    ring(stage, f"{r}/bottle/shoulder", g.NECK_R, g.BODY_R, g.BODY_H, amber)
    band(stage, f"{r}/bottle/neck", g.NECK_R, g.BODY_H, g.NECK_TOP, 0.0, 2 * math.pi, neck_mat)
    ring(stage, f"{r}/bottle/rim", mouth, g.NECK_R, g.NECK_TOP, amber)
    band(stage, f"{r}/bottle/mouth", mouth, g.NECK_TOP - 0.012, g.NECK_TOP, 0.0, 2 * math.pi, inside)
    disc(stage, f"{r}/bottle/tablets", mouth, g.NECK_TOP - 0.008, pills_mat)
    # its collider stops short of the cap pressed all the way down, so the two never overlap
    top = g.CAP_Z0 - g.TRAVEL - 0.001
    UsdGeom.Imageable(cylinder(stage, f"{r}/bottle/collider", g.BODY_R, top, top / 2, AMBER)).MakeInvisible()
    label(TEX / "label.png")
    label_mat = material(stage, f"{r}/Looks/label", (1, 1, 1), roughness=0.6, texture="./textures/label.png")
    band(stage, f"{r}/bottle/label", g.BODY_R + 0.0004, *g.LABEL_Z,
         math.pi - g.LABEL_SPAN / 2, math.pi + g.LABEL_SPAN / 2, label_mat)

    # the cap, on its lift, spin and press joints
    tiny = 1e-6
    body(stage, f"{r}/cap_lift", 0.004, (0, 0, g.CAP_C), inertia=tiny)
    body(stage, f"{r}/cap_spin", 0.004, (0, 0, g.CAP_C), inertia=tiny)
    body(stage, f"{r}/cap", g.CAP_MASS, (0, 0, g.CAP_C))
    white = material(stage, f"{r}/Looks/cap", CAP_WHITE, roughness=0.45)
    bind(cylinder(stage, f"{r}/cap/shell", g.CAP_R, g.CAP_H, 0.0, CAP_WHITE), white)
    cap_top(TEX / "cap_top.png")
    cap_side(TEX / "cap_side.png")
    top_mat = material(stage, f"{r}/Looks/cap_top", (1, 1, 1), roughness=0.45, texture="./textures/cap_top.png")
    side_mat = material(stage, f"{r}/Looks/cap_side", (1, 1, 1), roughness=0.45, texture="./textures/cap_side.png")
    disc(stage, f"{r}/cap/top", g.CAP_R - 0.0003, g.CAP_H / 2 + 0.0003, top_mat)
    band(stage, f"{r}/cap/ribs", g.CAP_R + 0.0003, -g.CAP_H / 2 + 0.001, g.CAP_H / 2 - 0.001, 0, 2 * math.pi, side_mat)

    joint(stage, f"{r}/joints/lift", "linear", f"{r}/bottle", f"{r}/cap_lift", (0, 0, g.CAP_C), (0.0, g.LIFT_MAX),
          damping=0.5)
    joint(stage, f"{r}/joints/spin", "angular", f"{r}/cap_lift", f"{r}/cap_spin", (0, 0, 0),
          damping=g.SPIN_DAMPING)
    joint(stage, f"{r}/joints/press", "linear", f"{r}/cap_spin", f"{r}/cap", (0, 0, 0), (-g.TRAVEL, 0.0),
          stiffness=g.SPRING, damping=2.0)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    TEX.mkdir(parents=True, exist_ok=True)
    build_pill_bottle()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
