"""Write the scene's texture and USD asset from geometry.py.

    python build_assets.py          # needs pxr, PIL (Isaac Sim's Python) and the DejaVu fonts, not the simulator

- assets/textures/dial.png: the gauge's dial, 0 to 300 mmHg, with a green arc from 250.
- assets/tourniquet.usda: an articulation fixed to the world. Its base holds the training thigh, the chocks, the band,
  the bladder and the gauge. The windlass turns on ``twist`` (revolute, about the vertical through the band's top);
  the rod tips on ``press`` (revolute, about the horizontal across it at its middle), so either end can be pushed down
  into one of the clip's hooks. The gauge's needle turns on ``needle``. task.py applies the band's unwinding torque,
  sets the needle from the twist, and adds the clip's hooks as fixed shapes (the rod is part of the articulation, so
  only shapes outside it can stop it).

The asset's origin is on the table under the thigh's axis, at the band, so task.py places it at (LEG_X, BAND_Y, TOP).
"""

from __future__ import annotations

import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"
TEX = OUT / "textures"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_REG = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

FOAM = (0.78, 0.62, 0.50)          # the trainer's skin-toned foam
CHOCK = (0.55, 0.40, 0.24)         # wood
BAND = (0.10, 0.30, 0.62)          # a blue training tourniquet
ROD = (0.05, 0.05, 0.06)
BLADDER = (0.14, 0.14, 0.16)       # dark grey nylon
CASE = (0.16, 0.17, 0.19)
CHROME = (0.80, 0.82, 0.85)
TUBE = (0.12, 0.12, 0.13)
NEEDLE = (0.80, 0.08, 0.06)


# -- texture ---------------------------------------------------------------------------------------------------------------
def dial(path: Path) -> None:
    """The dial face as seen from the front: 0 at lower left, 300 at lower right, clockwise; numbers every 20 mmHg,
    ticks every 10, a green arc from GREEN to the end."""
    n = 1024
    im = Image.new("RGBA", (n, n), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    c, R = n / 2, n / 2 - 4
    d.ellipse([c - R, c - R, c + R, c + R], fill=(246, 244, 238, 255), outline=(40, 40, 44, 255), width=10)

    def at(p, r):
        a = g.needle_angle(p)                          # clockwise from up
        return c + r * math.sin(a), c - r * math.cos(a)

    def arc(p0, p1, r, width, color):
        a0, a1 = (math.degrees(g.needle_angle(p)) - 90 for p in (p0, p1))
        d.arc([c - r, c - r, c + r, c + r], a0, a1, fill=color, width=width)

    arc(g.GREEN, g.GAUGE_MAX, R - 70, 64, (40, 160, 80, 255))
    for p in range(0, int(g.GAUGE_MAX) + 1, 10):
        big = p % 20 == 0
        d.line([at(p, R - (110 if big else 80)), at(p, R - 30)], fill=(30, 30, 34, 255), width=9 if big else 5)
    f = ImageFont.truetype(FONT, 62)
    for p in range(0, int(g.GAUGE_MAX) + 1, 40):
        d.text(at(p, R - 175), str(p), font=f, fill=(25, 25, 28, 255), anchor="mm")
    d.text((c, c + 200), "mmHg", font=ImageFont.truetype(FONT_REG, 54), fill=(60, 60, 64, 255), anchor="mm")
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


def material(stage, path: str, color, roughness=0.6, texture: str | None = None, metallic=0.0):
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


def body(stage, path: str, mass: float, pos=(0.0, 0.0, 0.0), orient=None, inertia=None):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if orient is not None:
        xf.AddOrientOp().Set(orient)
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    m = UsdPhysics.MassAPI.Apply(xf.GetPrim())
    m.CreateMassAttr(mass)
    if inertia is not None:                           # bodies with no colliders need it set by hand
        m.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
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


def cylinder(stage, path, radius, height, center, axis, mat, collide=False, orient=None):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(height)
    c.CreateAxisAttr(axis)
    c.AddTranslateOp().Set(Gf.Vec3d(*center))
    if orient is not None:
        c.AddOrientOp().Set(orient)
    bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def rod_between(stage, path, a, b, radius, mat):
    """A thin visual cylinder from a to b (the gauge's tube)."""
    a, b = Gf.Vec3d(*a), Gf.Vec3d(*b)
    d = b - a
    rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), d.GetNormalized())
    q = rot.GetQuat()
    return cylinder(stage, path, radius, d.GetLength(), (a + b) / 2, "Z", mat,
                    orient=Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary())))


def joint(stage, kind, path, body0, body1, pos0, rot0, axis, limits=None):
    j = (UsdPhysics.RevoluteJoint if kind == "revolute" else UsdPhysics.PrismaticJoint).Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    j.CreateLocalRot0Attr(rot0)
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    if limits is not None:
        j.CreateLowerLimitAttr(limits[0])
        j.CreateUpperLimitAttr(limits[1])
    # a force drive with zero gains; the task's actuator config sets the real ones
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if kind == "revolute" else "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.0)
    drive.CreateTargetPositionAttr(0.0)
    return j


def dial_frame():
    """The dial's frame in the asset: x to the dial's right (the robot's right), y to its 12 o'clock, z out of its
    face (toward the arms, tilted up by GAUGE_TILT)."""
    t = g.GAUGE_TILT
    x, y, z = Gf.Vec3d(0, -1, 0), Gf.Vec3d(math.sin(t), 0, math.cos(t)), Gf.Vec3d(-math.cos(t), 0, math.sin(t))
    m = Gf.Matrix3d(x[0], x[1], x[2], y[0], y[1], y[2], z[0], z[1], z[2])   # rows: the frame's axes (Gf's convention)
    q = Gf.Matrix4d(m, Gf.Vec3d(0, 0, 0)).ExtractRotation().GetQuat()
    return Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary())), (x, y, z)


# -- the asset ---------------------------------------------------------------------------------------------------------
def build_tourniquet():
    stage, _ = stage_for("tourniquet")
    r = "/tourniquet"
    foam = material(stage, f"{r}/Looks/foam", FOAM, roughness=0.9)
    wood = material(stage, f"{r}/Looks/wood", CHOCK, roughness=0.8)
    band = material(stage, f"{r}/Looks/band", BAND, roughness=0.85)
    rod_mat = material(stage, f"{r}/Looks/rod", ROD, roughness=0.5)
    case = material(stage, f"{r}/Looks/case", CASE, roughness=0.4, metallic=0.3)
    chrome = material(stage, f"{r}/Looks/chrome", CHROME, roughness=0.25, metallic=0.8)
    tube = material(stage, f"{r}/Looks/tube", TUBE, roughness=0.7)
    bladder = material(stage, f"{r}/Looks/bladder", BLADDER, roughness=0.8)
    needle_mat = material(stage, f"{r}/Looks/needle", NEEDLE, roughness=0.5)
    dial(TEX / "dial.png")
    dial_mat = material(stage, f"{r}/Looks/dial", (1, 1, 1), roughness=0.6, texture="./textures/dial.png")

    # the base: fixed to the world (the thigh is strapped down on its chocks)
    base = f"{r}/base"
    body(stage, base, 2.0)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(r))
    prim = stage.GetPrimAtPath(r)
    prim.AddAppliedSchema("PhysxArticulationAPI")
    prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    fixed = UsdPhysics.FixedJoint.Define(stage, f"{r}/joints/root_joint")
    fixed.CreateBody1Rel().SetTargets([Sdf.Path(base)])

    leg_c = (0.0, -g.BAND_Y, g.LEG_Z)                  # the asset's origin is at the band
    cylinder(stage, f"{base}/thigh", g.LEG_R, g.LEG_L, leg_c, "Y", foam, collide=True)
    for y in g.CHOCK_Y:
        for s in (-1, 1):
            box(stage, f"{base}/chock_{'lr'[y > 0]}{'ab'[s > 0]}", g.CHOCK,
                (s * (g.CHOCK_IN + g.CHOCK[0] / 2), y - g.BAND_Y, g.CHOCK[2] / 2), wood)   # touching the thigh
    cylinder(stage, f"{base}/band", g.LEG_R + g.BAND_T, g.BAND_W, (0.0, 0.0, g.LEG_Z), "Y", band, collide=True)
    # the pressure bladder under the band, wrapped once round the thigh, showing either side of it
    cylinder(stage, f"{base}/bladder", g.LEG_R + g.BLADDER_T, g.BLADDER_W, (0.0, 0.0, g.LEG_Z), "Y", bladder)
    # the band's tail, fastened round the side toward the arms
    box(stage, f"{base}/band_tail", (0.004, g.BAND_W * 0.9, 0.07), (-(g.LEG_R + g.BAND_T) - 0.002, 0.0, g.LEG_Z + 0.01),
        band, collide=False)
    # the gauge: a dial on a short post, tilted up toward the arms, its tube running to the band's bladder
    q_dial, (dx, dy, dz) = dial_frame()
    centre = Gf.Vec3d(g.GAUGE_XY[0] - g.LEG_X, g.GAUGE_XY[1] - g.BAND_Y, g.GAUGE_Z)
    cylinder(stage, f"{base}/gauge_case", g.DIAL_R + 0.006, 0.03, centre - dz * 0.016, "Z", case, collide=True,
             orient=q_dial)
    cylinder(stage, f"{base}/gauge_bezel", g.DIAL_R + 0.007, 0.004, centre + dz * 0.0005, "Z", chrome, orient=q_dial)
    face = UsdGeom.Mesh.Define(stage, f"{base}/gauge_face")
    R, n = g.DIAL_R, 64
    ring = [(math.cos(2 * math.pi * k / n), math.sin(2 * math.pi * k / n)) for k in range(n)]
    pts = [centre + dz * 0.003] + [centre + dz * 0.003 + dx * cx * R + dy * cy * R for cx, cy in ring]
    face.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    face.CreateFaceVertexCountsAttr([3] * n)
    idx, uv = [], []
    for k in range(n):
        for v in (0, 1 + k, 1 + (k + 1) % n):
            idx.append(v)
            cx, cy = (0.0, 0.0) if v == 0 else ring[v - 1]
            uv.append(Gf.Vec2f(0.5 + 0.5 * cx * (R - 0.001) / R, 0.5 + 0.5 * cy * (R - 0.001) / R))
    face.CreateFaceVertexIndicesAttr(idx)
    face.CreateSubdivisionSchemeAttr("none")
    st = UsdGeom.PrimvarsAPI(face).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    st.Set(uv)
    face.CreateDoubleSidedAttr(True)
    bind(face, dial_mat)
    post_top = centre - dz * 0.03 - dy * 0.02
    box(stage, f"{base}/gauge_post", (0.014, 0.014, post_top[2]), (post_top[0], post_top[1], post_top[2] / 2), case)
    box(stage, f"{base}/gauge_foot", (0.07, 0.07, 0.008), (post_top[0], post_top[1], 0.004), case)
    side = Gf.Vec3d(0.0, -g.BAND_W / 2 - 0.004, g.LEG_Z - g.LEG_R * 0.5)
    pts = [side, Gf.Vec3d(0.02, -0.06, 0.012), Gf.Vec3d(post_top[0] - 0.04, post_top[1] + 0.03, 0.012),
           Gf.Vec3d(post_top[0], post_top[1], 0.03)]
    for i in range(len(pts) - 1):
        rod_between(stage, f"{base}/tube_{i}", pts[i], pts[i + 1], 0.0025, tube)

    # the windlass: the twisted strap (mass only, no shape) turning about the vertical on the band's top; its x axis
    # is the rod's direction, THETA0 at the start (joint 0)
    hub = f"{r}/windlass"
    hub_z = g.BAND_TOP
    h = g.THETA0 / 2
    q0 = Gf.Quatf(math.cos(h), Gf.Vec3f(0, 0, math.sin(h)))
    body(stage, hub, 0.01, (0.0, 0.0, hub_z), orient=q0, inertia=(2e-6, 2e-6, 2e-6))
    box(stage, f"{hub}/strap", g.HUB, (0.0, 0.0, g.HUB[2] / 2), band, collide=False)
    joint(stage, "revolute", f"{r}/joints/twist", base, hub, (0.0, 0.0, hub_z), q0, "Z")

    # the rod: across the strap; an end pushed down tips it on ``press`` (positive tips its +x end down)
    rod = f"{r}/rod"
    body(stage, rod, g.ROD_MASS, (0.0, 0.0, g.ROD_Z), orient=q0)
    cylinder(stage, f"{rod}/bar", g.ROD_R, g.ROD_L, (0, 0, 0), "X", rod_mat, collide=True)
    for s in (-1, 1):
        cylinder(stage, f"{rod}/rib_{'ab'[s > 0]}", g.ROD_R + 0.0008, 0.004, (s * (g.ROD_L / 2 - 0.004), 0, 0), "X",
                 rod_mat)
    lim = math.degrees(g.TILT_MAX)
    joint(stage, "revolute", f"{r}/joints/press", hub, rod, (0.0, 0.0, g.ROD_Z - hub_z), Gf.Quatf(1, 0, 0, 0), "Y",
          limits=(-lim, lim))

    # the needle: turns about the dial's axis; at joint 0 it points to 12 o'clock
    needle = f"{r}/needle"
    pivot = centre + dz * 0.006
    body(stage, needle, 0.002, tuple(pivot), orient=q_dial, inertia=(1e-7, 1e-7, 1e-7))
    box(stage, f"{needle}/pointer", (0.0035, R * 0.85, 0.0015), (0, R * 0.85 / 2 - 0.008, 0), needle_mat,
        collide=False)
    cylinder(stage, f"{needle}/cap", 0.006, 0.003, (0, 0, 0.001), "Z", case)
    lim = math.degrees(g.NEEDLE_SWEEP / 2) + 2
    joint(stage, "revolute", f"{r}/joints/needle", base, needle, tuple(pivot), q_dial, "Z", limits=(-lim, lim))
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    TEX.mkdir(parents=True, exist_ok=True)
    build_tourniquet()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
