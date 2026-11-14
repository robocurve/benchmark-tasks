"""Write the scene's textures and USD assets from geometry.py.

    python build_assets.py [--out DIR]      # needs pxr, PIL (Isaac Sim's Python) and the DejaVu fonts, not the simulator

- assets/textures/motor_label.png: the sticker round the motor housing (brand, model, ratings, the guard warning).
- assets/textures/disc_top.png: the print on the grinding disc.
- assets/textures/operator_mat.png: the OPERATOR mat's print.
- assets/grinder.usda: one floating articulation. Root ``body``: motor housing, rear cap, gear head, collar, spindle,
  side handle, flange, disc and clamp nut. Link ``guard``: the hood (plate, skirt, adjusting lug), on the revolute
  joint ``guard`` about the spindle axis. Link ``lever``: the clamp lever, on the revolute joint ``lever`` (about a
  horizontal axis tangent to the skirt) at the guard's skirt. task.py turns the lever's position into the guard's
  lock (it holds the guard joint's limits).
- assets/operator_mat.usda: a flat printed mat (kinematic, no collision).

The grinder's origin is on the spindle axis at table height, its motor along -y, so task.py places it at
(spindle_x, spindle_y, TOP) with the layout's yaw.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

HERE = Path(__file__).resolve().parent
OUT = HERE / "assets"
TEX = OUT / "textures"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_PLAIN = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

HOUSING = (0.13, 0.35, 0.40)        # teal housing
HEAD_GREY = (0.30, 0.31, 0.33)      # cast gear head
STEEL = (0.62, 0.64, 0.67)
GUARD_PAINT = (0.74, 0.74, 0.76)    # zinc-plated steel hood
LEVER_RED = (0.82, 0.12, 0.10)
LUG_YELLOW = (0.95, 0.78, 0.10)
BLACK = (0.08, 0.08, 0.09)
INK = (24, 27, 31)


# -- textures ------------------------------------------------------------------------------------------------------------
def motor_label(path: Path) -> None:
    """The sticker unrolled round the motor housing (image x runs along the circle, image y along the motor)."""
    w, h = 1500, 700
    im = Image.new("RGB", (w, h), (232, 236, 238))
    d = ImageDraw.Draw(im)
    big, mid, small = (ImageFont.truetype(FONT, 78), ImageFont.truetype(FONT, 44), ImageFont.truetype(FONT_PLAIN, 36))
    d.rectangle([0, 0, w, 110], fill=(20, 80, 92))
    d.text((40, 55), "TORQMASTER", font=big, fill=(255, 255, 255), anchor="lm")
    d.text((w - 40, 55), "TG-115/750", font=mid, fill=(255, 255, 255), anchor="rm")
    d.text((40, 165), "ANGLE GRINDER  115 mm", font=mid, fill=INK, anchor="lm")
    d.text((40, 220), "750 W    230 V~    11 000 min-1", font=small, fill=INK, anchor="lm")
    d.rectangle([0, 270, w, 275], fill=(190, 194, 198))
    # the warning panel: a yellow triangle with a guard diagram
    d.polygon([(70, 300), (190, 500), (-50 + 70, 500)], fill=(250, 205, 20), outline=INK)
    d.text((70, 440), "!", font=big, fill=INK, anchor="mm")
    d.text((250, 330), "WARNING", font=mid, fill=INK, anchor="lm")
    d.text((250, 385), "THE GUARD'S CLOSED SIDE MUST FACE", font=small, fill=INK, anchor="lm")
    d.text((250, 430), "THE OPERATOR. LOCK THE CLAMP LEVER", font=small, fill=INK, anchor="lm")
    d.text((250, 475), "BEFORE USE.", font=small, fill=INK, anchor="lm")
    # a diagram: a disc with a half-ring guard, an arrow to a stick figure
    cx, cy, r = 1130, 440, 70
    d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=INK, width=6)
    d.arc([cx - r - 14, cy - r - 14, cx + r + 14, cy + r + 14], start=90, end=270, fill=(200, 60, 40), width=16)
    d.line([(cx - r - 70, cy), (cx - r - 150, cy)], fill=INK, width=6)
    d.polygon([(cx - r - 150, cy), (cx - r - 118, cy - 18), (cx - r - 118, cy + 18)], fill=INK)
    d.ellipse([cx - r - 205, cy - 50, cx - r - 160, cy - 5], outline=INK, width=6)
    d.line([(cx - r - 182, cy - 5), (cx - r - 182, cy + 50)], fill=INK, width=6)
    d.text((750, 600), "Made in a generic factory   CE   IP20   2.1 kg", font=small, fill=(90, 94, 98), anchor="lm")
    im.save(path)


def disc_top(path: Path) -> None:
    """The grinding disc's printed face (image up is +x, image right is -y)."""
    n = 1024
    c = n / 2
    im = Image.new("RGB", (n, n), (38, 40, 44))
    d = ImageDraw.Draw(im)
    d.ellipse([20, 20, n - 20, n - 20], outline=(210, 160, 40), width=14)
    d.ellipse([150, 150, n - 150, n - 150], outline=(120, 124, 130), width=5)
    d.ellipse([c - 62, c - 62, c + 62, c + 62], fill=(150, 152, 156))
    f1, f2 = ImageFont.truetype(FONT, 56), ImageFont.truetype(FONT_PLAIN, 40)
    d.text((c, 250), "GRINDING DISC", font=f1, fill=(240, 230, 200), anchor="mm")
    d.text((c, 320), "115 x 6 x 22.23   A 24 R", font=f2, fill=(240, 230, 200), anchor="mm")
    d.text((c, n - 330), "MAX 13 300 min-1   80 m/s", font=f2, fill=(240, 230, 200), anchor="mm")
    d.text((c, n - 260), "METAL", font=f1, fill=(240, 230, 200), anchor="mm")
    im.save(path)


def operator_mat(path: Path) -> None:
    """The OPERATOR mat's print: 1000 px across (the mat's y extent, 0.10 m) and 1400 px tall (its x extent, 0.14 m).
    Image up is +x (the arms' forward), image right is -y, so it reads left to right from the arms."""
    w, h = 1000, 1400
    im = Image.new("RGB", (w, h), (30, 33, 36))
    d = ImageDraw.Draw(im)
    d.rectangle([14, 14, w - 14, h - 14], outline=(250, 205, 20), width=22)
    f = ImageFont.truetype(FONT, 128)
    d.text((w / 2, 190), "OPERATOR", font=f, fill=(250, 205, 20), anchor="mm")
    for cx in (w / 2 - 150, w / 2 + 150):                      # two footprints, toes up the image
        d.ellipse([cx - 105, 360, cx + 105, 840], fill=(250, 250, 250))
        d.ellipse([cx - 88, 900, cx + 88, 1180], fill=(250, 250, 250))
    im.save(path)


# -- USD helpers -------------------------------------------------------------------------------------------------------
def stage_for(out: Path, name: str):
    path = out / f"{name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    return stage, root


def material(stage, path: str, color, roughness=0.5, texture: str | None = None, metallic: float = 0.0):
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


def rigid(stage, path: str, mass: float, pos=(0.0, 0.0, 0.0), rot_z: float | None = None, inertia=None):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if rot_z is not None:
        xf.AddRotateZOp().Set(math.degrees(rot_z))
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    m = UsdPhysics.MassAPI.Apply(xf.GetPrim())
    m.CreateMassAttr(mass)
    if inertia is not None:
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        m.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    return xf


def cylinder(stage, path, radius, height, pos, color, mat=None, collide=True, axis="Z"):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(height)
    c.CreateAxisAttr(axis)
    c.AddTranslateOp().Set(Gf.Vec3d(*pos))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def box(stage, path, size, pos, color, mat=None, rot_z: float | None = None, collide=True):
    """A box of the given full size, centred at pos, turned rot_z (radians) about z."""
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if rot_z is not None:
        c.AddRotateZOp().Set(math.degrees(rot_z))
    c.AddScaleOp().Set(Gf.Vec3f(*size))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def band(stage, path, radius, z0, z1, a0, a1, mat, segs=96):
    """A textured band round the z axis from angle a0 to a1 (anticlockwise seen from above): image x runs with the
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


def disc_face(stage, path, radius, z, mat, segs=96):
    """A textured disc facing up; the image's up is +x and its right is -y."""
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


def sector(stage, path, r_in, r_out, z, a0, a1, mat, segs=48):
    """A flat annular sector facing up between two radii and two angles."""
    pts, idx = [], []
    for j in range(segs + 1):
        a = a0 + (a1 - a0) * j / segs
        pts += [Gf.Vec3f(r_in * math.cos(a), r_in * math.sin(a), z), Gf.Vec3f(r_out * math.cos(a), r_out * math.sin(a), z)]
    for j in range(segs):
        idx += [2 * j, 2 * j + 1, 2 * j + 3, 2 * j + 2]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr([4] * segs)
    m.CreateFaceVertexIndicesAttr(idx)
    m.CreateDoubleSidedAttr(True)
    bind(m, mat)
    return m


def joint(stage, path, kind, body0, body1, pos0, axis="Z", rot0=(1, 0, 0, 0), limits=None, damping=0.0,
          stiffness=0.0):
    """A prismatic ('linear') or revolute ('angular') joint. Limits are metres or DEGREES (USD's unit)."""
    j = (UsdPhysics.PrismaticJoint if kind == "linear" else UsdPhysics.RevoluteJoint).Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    j.CreateLocalRot0Attr(Gf.Quatf(*rot0))
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


def quat_z(a: float):
    """A rotation by angle a about z as a (w, x, y, z) quaternion."""
    return (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


# -- the grinder ---------------------------------------------------------------------------------------------------------
def build_grinder(out: Path, tex: Path) -> None:
    stage, root = stage_for(out, "grinder")
    r = "/grinder"

    # ----- the body (the articulation's floating root) -----
    prim = rigid(stage, f"{r}/body", g.BODY_MASS).GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxArticulationAPI")
    prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    prim.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    prim.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)

    housing = material(stage, f"{r}/Looks/housing", HOUSING, roughness=0.45)
    head_m = material(stage, f"{r}/Looks/head", HEAD_GREY, roughness=0.35, metallic=0.6)
    steel = material(stage, f"{r}/Looks/steel", STEEL, roughness=0.3, metallic=0.8)
    black = material(stage, f"{r}/Looks/black", BLACK, roughness=0.6)
    motor_label(tex / "motor_label.png")
    disc_top(tex / "disc_top.png")
    label_m = material(stage, f"{r}/Looks/label", (1, 1, 1), roughness=0.5, texture="./textures/motor_label.png")
    disc_m = material(stage, f"{r}/Looks/disc", (1, 1, 1), roughness=0.8, texture="./textures/disc_top.png")
    disc_edge = material(stage, f"{r}/Looks/disc_edge", (0.16, 0.17, 0.19), roughness=0.9)

    # the motor housing lies along y: a group turned so its local z runs along the asset's +y
    motor_len = g.MOTOR_Y1 - g.MOTOR_Y0
    mg = UsdGeom.Xform.Define(stage, f"{r}/body/motor")
    mg.AddTranslateOp().Set(Gf.Vec3d(0, (g.MOTOR_Y0 + g.MOTOR_Y1) / 2, g.MOTOR_R))
    mg.AddRotateXOp().Set(-90.0)
    cylinder(stage, f"{r}/body/motor/shell", g.MOTOR_R, motor_len, (0, 0, 0), HOUSING, housing)
    # the sticker: 90 degrees of the circle on the top (local angle -90), over most of the housing's length
    z0, z1 = -motor_len / 2 + 0.012, motor_len / 2 - 0.012
    band(stage, f"{r}/body/motor/sticker", g.MOTOR_R + 0.0004, z0, z1, math.radians(-90 - 45), math.radians(-90 + 45), label_m)
    cg = UsdGeom.Xform.Define(stage, f"{r}/body/cap")
    cg.AddTranslateOp().Set(Gf.Vec3d(0, (g.CAP_Y0 + g.MOTOR_Y0) / 2, g.MOTOR_R))
    cg.AddRotateXOp().Set(-90.0)
    cylinder(stage, f"{r}/body/cap/shell", g.CAP_R, g.MOTOR_Y0 - g.CAP_Y0, (0, 0, 0), BLACK, black)

    # gear head, collar, spindle
    box(stage, f"{r}/body/head", (2 * g.HEAD_HX, g.HEAD_Y1 - g.HEAD_Y0, g.HEAD_H),
        (0, (g.HEAD_Y0 + g.HEAD_Y1) / 2, g.HEAD_H / 2), HEAD_GREY, head_m)
    cylinder(stage, f"{r}/body/neck", g.NECK_R, g.NECK_Z1 - g.NECK_Z0, (0, 0, (g.NECK_Z0 + g.NECK_Z1) / 2), STEEL, steel)
    cylinder(stage, f"{r}/body/spindle", g.SPINDLE_R, g.SPINDLE_Z1 - g.NECK_Z1,
             (0, 0, (g.NECK_Z1 + g.SPINDLE_Z1) / 2), STEEL, steel, collide=False)
    # side handle on the -x side of the head, along x
    hg = UsdGeom.Xform.Define(stage, f"{r}/body/handle")
    hg.AddTranslateOp().Set(Gf.Vec3d(-(g.HEAD_HX + g.HANDLE_LEN / 2), 0.0, g.HANDLE_Z))
    hg.AddRotateYOp().Set(90.0)
    cylinder(stage, f"{r}/body/handle/grip", g.HANDLE_R, g.HANDLE_LEN, (0, 0, 0), BLACK, black)

    # flange, disc, nut
    cylinder(stage, f"{r}/body/flange", g.FLANGE_R, g.FLANGE_Z1 - g.FLANGE_Z0,
             (0, 0, (g.FLANGE_Z0 + g.FLANGE_Z1) / 2), STEEL, steel)
    cylinder(stage, f"{r}/body/disc", g.DISC_R, g.DISC_T, (0, 0, (g.DISC_Z0 + g.DISC_Z1) / 2), BLACK, disc_edge)
    disc_face(stage, f"{r}/body/disc_print", g.DISC_R - 0.0003, g.DISC_Z1 + 0.0003, disc_m)
    cylinder(stage, f"{r}/body/nut", g.NUT_R, g.NUT_Z1 - g.NUT_Z0, (0, 0, (g.NUT_Z0 + g.NUT_Z1) / 2), STEEL, steel)

    # ----- the guard link: origin on the spindle axis at PLATE_Z0, closed-side bisector along +x -----
    rigid(stage, f"{r}/guard", g.GUARD_MASS, (0, 0, g.PLATE_Z0))
    paint = material(stage, f"{r}/Looks/guard", GUARD_PAINT, roughness=0.35, metallic=0.7)
    yellow = material(stage, f"{r}/Looks/lug", LUG_YELLOW, roughness=0.5)
    a0, a1 = -g.SKIRT_SPAN / 2, g.SKIRT_SPAN / 2
    skirt_h = g.SKIRT_Z1 - g.PLATE_Z0
    # visual: the hood's plate and skirt (meshes, no collision); collision: boxes along the skirt
    sector(stage, f"{r}/guard/plate", 0.0225, g.SKIRT_R_IN + g.SKIRT_T, g.PLATE_T, a0, a1, paint)
    for k, rad in enumerate((g.SKIRT_R_IN, g.SKIRT_R_IN + g.SKIRT_T)):          # inner and outer wall, both faces
        m = UsdGeom.Mesh.Define(stage, f"{r}/guard/wall{k}")
        segs = 48
        pts, idx = [], []
        for j in range(segs + 1):
            a = a0 + (a1 - a0) * j / segs
            pts += [Gf.Vec3f(rad * math.cos(a), rad * math.sin(a), 0.0), Gf.Vec3f(rad * math.cos(a), rad * math.sin(a), skirt_h)]
        for j in range(segs):
            idx += [2 * j, 2 * j + 2, 2 * j + 3, 2 * j + 1]
        m.CreatePointsAttr(pts)
        m.CreateFaceVertexCountsAttr([4] * segs)
        m.CreateFaceVertexIndicesAttr(idx)
        m.CreateDoubleSidedAttr(True)
        bind(m, paint)
    n_boxes = 12
    r_c = g.SKIRT_R_IN + g.SKIRT_T / 2
    chord = 2 * r_c * math.sin(g.SKIRT_SPAN / n_boxes / 2) * 1.08
    for k in range(n_boxes):
        a = a0 + (k + 0.5) * g.SKIRT_SPAN / n_boxes
        box(stage, f"{r}/guard/skirt{k:02d}", (g.SKIRT_T + 0.0005, chord, skirt_h),
            (r_c * math.cos(a), r_c * math.sin(a), skirt_h / 2), GUARD_PAINT, paint, rot_z=a)
    # the adjusting lug: a yellow tab standing up from the skirt rim at LUG_PHI
    lug_r = g.SKIRT_R_IN + g.SKIRT_T + g.LUG_L / 2
    box(stage, f"{r}/guard/lug", (g.LUG_L, g.LUG_W, g.LUG_H),
        (lug_r * math.cos(g.LUG_PHI), lug_r * math.sin(g.LUG_PHI), g.LUG_MID - g.PLATE_Z0),
        LUG_YELLOW, yellow, rot_z=g.LUG_PHI)
    # the clamp collar round the neck (visual): a ring under the plate
    cyl_vis = cylinder(stage, f"{r}/guard/collar", g.NECK_R + 0.004, g.NECK_Z1 - g.NECK_Z0 - 0.001,
                       (0, 0, -(g.NECK_Z1 - g.NECK_Z0) / 2 - 0.0005), STEEL, steel, collide=False)

    # ----- the lever link: origin at its hinge, frame turned so local x is the tangent (the joint axis), -y radial out -----
    hx, hy = g.LEVER_HINGE_R * math.cos(g.LEVER_PHI), g.LEVER_HINGE_R * math.sin(g.LEVER_PHI)
    rigid(stage, f"{r}/lever", g.LEVER_MASS, (hx, hy, g.LEVER_HINGE_Z), rot_z=g.LEVER_PHI + math.pi / 2)
    red = material(stage, f"{r}/Looks/lever", LEVER_RED, roughness=0.5)
    box(stage, f"{r}/lever/arm", (g.LEVER_W, g.LEVER_L, g.LEVER_T), (0, -g.LEVER_L / 2, 0), LEVER_RED, red)
    cylinder(stage, f"{r}/lever/barrel", 0.0045, g.LEVER_W + 0.004, (0, 0, 0), STEEL, steel, collide=False, axis="X")

    # ----- joints -----
    # the guard turns about the spindle (z) on the body, almost a full turn each way (task.py holds the limits to lock it)
    joint(stage, f"{r}/joints/guard", "angular", f"{r}/body", f"{r}/guard", (0, 0, g.PLATE_Z0), axis="Z",
          limits=(-355.0, 355.0), damping=g.GUARD_DAMPING)
    # the lever turns about the tangent at its hinge, on the guard; negative angles raise it
    joint(stage, f"{r}/joints/lever", "angular", f"{r}/guard", f"{r}/lever",
          (hx, hy, g.LEVER_HINGE_Z - g.PLATE_Z0), axis="X", rot0=quat_z(g.LEVER_PHI + math.pi / 2),
          limits=(math.degrees(g.LEVER_OPEN), 0.0), damping=g.LEVER_DAMPING)
    stage.GetRootLayer().Save()


def build_mat(out: Path, tex: Path) -> None:
    stage, root = stage_for(out, "operator_mat")
    r = "/operator_mat"
    operator_mat(tex / "operator_mat.png")
    m = material(stage, f"{r}/Looks/mat", (1, 1, 1), roughness=0.9, texture="./textures/operator_mat.png")
    rigid(stage, f"{r}/body", 0.5)
    sx, sy = g.MAT_SIZE
    quad = UsdGeom.Mesh.Define(stage, f"{r}/body/print")
    quad.CreatePointsAttr([Gf.Vec3f(-sx / 2, -sy / 2, 0), Gf.Vec3f(sx / 2, -sy / 2, 0), Gf.Vec3f(sx / 2, sy / 2, 0),
                           Gf.Vec3f(-sx / 2, sy / 2, 0)])
    quad.CreateFaceVertexCountsAttr([4])
    quad.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    quad.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    UsdGeom.Mesh(quad).SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    # image up is +x, image right is -y: corner (+x, +y) is the image's upper left
    st = UsdGeom.PrimvarsAPI(quad).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1), Gf.Vec2f(0, 0)])
    quad.CreateDoubleSidedAttr(True)
    bind(quad, m)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(OUT))
    a = ap.parse_args()
    out = Path(a.out)
    tex = out / "textures"
    tex.mkdir(parents=True, exist_ok=True)
    build_grinder(out, tex)
    build_mat(out, tex)
    for p in sorted(out.rglob("*.*")):
        print(p.relative_to(out), p.stat().st_size, "bytes")
