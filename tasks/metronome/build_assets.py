"""Write the scene's textures and USD asset from geometry.py.

    python build_assets.py          # needs pxr, PIL (Isaac Sim's Python) and the DejaVu fonts, not the simulator

- assets/textures/scale.png: the tempo scale, 40 to 208, with the Italian tempo terms.
- assets/metronome.usda: a floating articulation (the metronome stands free on the table). The case is the root.
  The pendulum hangs on ``pendulum`` (revolute, about x, in front of the scale); its bob has no shape, it is mass
  only, hidden in the case. The weight slides on the rod on ``weight`` (prismatic, up the rod); a drive holds it on
  the nearest notch (task.py moves the drive's target). task.py also runs the escapement.

The asset's origin is the centre of the case's base, so task.py places it at (MET_X, MET_Y, TOP).
"""

from __future__ import annotations

import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"
TEX = OUT / "textures"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_ITALIC = "/usr/share/fonts/truetype/dejavu/DejaVuSerif-Italic.ttf"

WALNUT = (0.30, 0.16, 0.08)
WALNUT_DARK = (0.20, 0.10, 0.05)
BRASS = (0.78, 0.60, 0.25)
STEEL = (0.62, 0.64, 0.67)
INK = (40, 34, 26)
TERMS = (("Largo", 40, 60), ("Larghetto", 60, 66), ("Adagio", 66, 76), ("Andante", 76, 108), ("Moderato", 108, 120),
         ("Allegro", 120, 168), ("Presto", 168, 200))


# -- textures ------------------------------------------------------------------------------------------------------------
def scale(path: Path) -> None:
    """The tempo scale plate as the arms see it (image left is the robot's left). Each mark is a line out to 14 mm
    either side of the rod, past the weight's sides, numbered on alternating sides; the weight's top edge reads it."""
    lo, hi = g.SCALE_Z
    w = 512
    ppm = w / (2 * g.SCALE_HALF_W)                         # pixels per metre
    h = round((hi - lo) * ppm)
    im = Image.new("RGB", (w, h), (244, 236, 214))
    d = ImageDraw.Draw(im)
    f = ImageFont.truetype(FONT, 30)
    fi = ImageFont.truetype(FONT_ITALIC, 22)
    c, rod, reach = w / 2, g.ROD_W / 2 * ppm, 0.014 * ppm
    row = lambda x: (hi - x) * ppm                         # noqa: E731
    left = True
    for b, x in zip(g.MARKS, g.MARK_X):
        y = row(x)
        d.line([(c - reach, y), (c - rod - 4, y)], fill=INK, width=3)
        d.line([(c + rod + 4, y), (c + reach, y)], fill=INK, width=3)
        if b < g.START and b not in (40, 44, 48, 52, 56):  # the slow end is too crowded to number every mark
            continue
        if left:
            d.text((c - reach - 8, y), str(b), font=f, fill=INK, anchor="rm")
        else:
            d.text((c + reach + 8, y), str(b), font=f, fill=INK, anchor="lm")
        left = not left
    for name, b0, b1 in TERMS:                             # the tempo terms up the right edge, between their marks
        y0, y1 = row(g.mark_height(b0)), row(g.mark_height(b1))
        tw = d.textlength(name, font=fi)
        if abs(y1 - y0) < tw + 8:
            continue
        glyph = Image.new("RGBA", (int(tw) + 8, 30), (0, 0, 0, 0))
        ImageDraw.Draw(glyph).text((4, 15), name, font=fi, fill=(120, 84, 40), anchor="lm")
        glyph = glyph.rotate(90, expand=True)
        im.paste(glyph, (w - 34, int((y0 + y1) / 2 - glyph.height / 2)), glyph)
    d.rectangle([2, 2, w - 3, h - 3], outline=(150, 128, 92), width=4)
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


def body(stage, path: str, mass: float, pos=(0.0, 0.0, 0.0), com=None, inertia=None):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    m = UsdPhysics.MassAPI.Apply(xf.GetPrim())
    m.CreateMassAttr(mass)
    if com is not None:                                 # set by hand: the bob is mass with no shape
        m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
    return xf


def box(stage, path, size, center, mat, collide=True):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*center))
    c.AddScaleOp().Set(Gf.Vec3f(*size))
    bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def cylinder(stage, path, radius, height, center, axis, mat, collide=False):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(height)
    c.CreateAxisAttr(axis)
    c.AddTranslateOp().Set(Gf.Vec3d(*center))
    bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def frustum(stage, path, mat, collide=True):
    """The case: a four-sided frustum with a vertical front face (at FRONT_X) and the other three faces leaning in."""
    x0, x1, w0 = g.FRONT_X, g.FRONT_X + g.CASE_D, g.CASE_W / 2
    tx1, tw = g.FRONT_X + g.TOP_D, g.TOP_W / 2
    h = g.CASE_H
    v = {"b0": (x0, -w0, 0), "b1": (x1, -w0, 0), "b2": (x1, w0, 0), "b3": (x0, w0, 0),
         "t0": (x0, -tw, h), "t1": (tx1, -tw, h), "t2": (tx1, tw, h), "t3": (x0, tw, h)}
    faces = [("b3", "b2", "b1", "b0"), ("t0", "t1", "t2", "t3"), ("b0", "t0", "t3", "b3"), ("b1", "b2", "t2", "t1"),
             ("b0", "b1", "t1", "t0"), ("b2", "b3", "t3", "t2")]
    pts, idx = [], []
    for f in faces:                                      # vertices per face, so each face is shaded flat
        for k in f:
            idx.append(len(pts))
            pts.append(Gf.Vec3f(*v[k]))
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr([4] * len(faces))
    m.CreateFaceVertexIndicesAttr(idx)
    m.CreateSubdivisionSchemeAttr("none")
    bind(m, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
    return m


def plate(stage, path, mat):
    """The scale plate on the front face, facing the arms; image left is +y (the robot's left)."""
    x = g.FRONT_X - 0.0004
    lo, hi = g.PIVOT_Z + g.SCALE_Z[0], g.PIVOT_Z + g.SCALE_Z[1]
    w = g.SCALE_HALF_W
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(x, w, lo), Gf.Vec3f(x, -w, lo), Gf.Vec3f(x, -w, hi), Gf.Vec3f(x, w, hi)])
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    m.CreateSubdivisionSchemeAttr("none")
    st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    m.CreateDoubleSidedAttr(True)
    bind(m, mat)
    return m


def joint(stage, path, kind, axis, body0, body1, pos0, limits, stiffness=0.0, damping=0.0, target=0.0):
    j = (UsdPhysics.PrismaticJoint if kind == "linear" else UsdPhysics.RevoluteJoint).Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), kind)
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateTargetPositionAttr(target)
    return j


# -- the asset ---------------------------------------------------------------------------------------------------------
def build_metronome():
    stage, root = stage_for("metronome")
    r = "/metronome"
    walnut = material(stage, f"{r}/Looks/walnut", WALNUT, roughness=0.45)
    dark = material(stage, f"{r}/Looks/walnut_dark", WALNUT_DARK, roughness=0.5)
    brass = material(stage, f"{r}/Looks/brass", BRASS, roughness=0.3, metallic=0.6)
    steel = material(stage, f"{r}/Looks/steel", STEEL, roughness=0.3, metallic=0.7)
    black = material(stage, f"{r}/Looks/black", (0.04, 0.04, 0.04), roughness=0.6)
    scale(TEX / "scale.png")
    scale_mat = material(stage, f"{r}/Looks/scale", (1, 1, 1), roughness=0.7, texture="./textures/scale.png")

    # the case: the articulation's root (floating), so the task places it by its base
    prim = body(stage, f"{r}/case", g.CASE_MASS).GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxArticulationAPI")
    prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    prim.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    prim.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    frustum(stage, f"{r}/case/shell", walnut)
    plate(stage, f"{r}/case/scale", scale_mat)
    top_x = g.FRONT_X + g.TOP_D / 2
    box(stage, f"{r}/case/cap", (g.TOP_D + 0.002, g.TOP_W + 0.002, 0.005), (top_x, 0, g.CASE_H + 0.0025), dark)
    box(stage, f"{r}/case/plinth", (g.CASE_D + 0.008, g.CASE_W + 0.008, 0.008), (0, 0, 0.004), dark)
    hub = g.FRONT_X - g.PIVOT_X
    cylinder(stage, f"{r}/case/hub", 0.005, hub, (g.FRONT_X - hub / 2, 0, g.PIVOT_Z), "X", steel)
    cylinder(stage, f"{r}/case/key", 0.004, 0.016, (0.0, -g.CASE_W / 2 + 0.002, 0.06), "Y", brass,
             collide=True)                                                                        # winding key
    box(stage, f"{r}/case/key_wing", (0.004, 0.003, 0.024), (0.0, -g.CASE_W / 2 - 0.007, 0.06), brass)

    # the pendulum: the rod above the pivot, and the bob (mass only) below it
    rod_c = (g.ROD_TOP + g.ROD_LOW) / 2
    mass = g.BOB_MASS + g.ROD_MASS
    com_z = (g.ROD_MASS * rod_c - g.BOB_MASS * g.BOB_L) / mass
    length = g.ROD_TOP - g.ROD_LOW
    i_pivot = (g.BOB_MASS * g.BOB_L ** 2 + 0.4 * g.BOB_MASS * g.BOB_R ** 2
               + g.ROD_MASS * (length ** 2 / 12 + rod_c ** 2))
    i_com = i_pivot - mass * com_z ** 2
    body(stage, f"{r}/pendulum", mass, (g.PIVOT_X, 0, g.PIVOT_Z), com=(0, 0, com_z),
         inertia=(i_com, i_com, 0.4 * g.BOB_MASS * g.BOB_R ** 2))
    box(stage, f"{r}/pendulum/rod", (g.ROD_T, g.ROD_W, length), (0, 0, rod_c), steel)

    # the weight: its frame is at its top edge, which reads the scale
    x0 = g.MARK_X[g.MARKS.index(g.START)]
    i_w = g.WEIGHT_MASS * (g.WEIGHT_W ** 2 + g.WEIGHT_H ** 2) / 12
    body(stage, f"{r}/weight", g.WEIGHT_MASS, (g.PIVOT_X, 0, g.PIVOT_Z + x0), com=(0, 0, -g.WEIGHT_H / 2),
         inertia=(i_w, i_w, i_w))
    box(stage, f"{r}/weight/block", (g.WEIGHT_T, g.WEIGHT_W, g.WEIGHT_H), (0, 0, -g.WEIGHT_H / 2), brass)
    box(stage, f"{r}/weight/top", (g.WEIGHT_T + 0.0002, g.WEIGHT_W + 0.0002, 0.0010), (0, 0, -0.0004),
        black, collide=False)                  # a dark top: from above, where it meets the scale is the reading
    cylinder(stage, f"{r}/weight/screw", 0.0028, 0.002, (-g.WEIGHT_T / 2 - 0.001, 0, -g.WEIGHT_H * 0.6), "X", dark)

    joint(stage, f"{r}/joints/pendulum", "angular", "X", f"{r}/case", f"{r}/pendulum", (g.PIVOT_X, 0, g.PIVOT_Z),
          (-math.degrees(g.SWING_LIMIT), math.degrees(g.SWING_LIMIT)), damping=g.DAMP_STOP)
    joint(stage, f"{r}/joints/weight", "linear", "Z", f"{r}/pendulum", f"{r}/weight", (0, 0, 0), (g.X_MIN, g.X_MAX),
          stiffness=g.DETENT_K, damping=g.DETENT_D, target=x0 + g.SAG)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    TEX.mkdir(parents=True, exist_ok=True)
    build_metronome()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
