"""Write the scene's textures and USD assets from geometry.py.

    python build_assets.py          # needs pxr and PIL (Isaac Sim's Python), not the simulator

- assets/textures/analog_face.png: the analog clock's face, with its hands at 4:40.
- assets/textures/clock_top.png: the bedside clock's top, printed HOUR and MIN beside its buttons.
- assets/bedside_clock.usda: an articulation with a fixed body, two buttons that slide down on spring-loaded
  prismatic joints (``hour_btn``, ``min_btn``) and three number drums on revolute joints (``drum_hours``,
  ``drum_tens``, ``drum_ones``), which task.py turns to the time to show. The drums have no colliders.
- assets/textures/drum_*.png: each drum's numbers, unrolled.
- assets/analog_clock.usda: one rigid body (kinematic in the task): the face, its rim and a wooden stand.

The asset origins are the station's (x, y) origin on the table top, so task.py places both at (0, 0, TOP).
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

BODY = (0.11, 0.11, 0.12)
CAP_GREY = (0.55, 0.57, 0.60)
WOOD = (0.50, 0.37, 0.24)
RIM = (0.16, 0.17, 0.19)
RIM_DEPTH = 0.035                       # the analog clock's rim, front to back


# -- textures ------------------------------------------------------------------------------------------------------------
def _font(size: int):
    return ImageFont.truetype(FONT, size)


def analog_face(path: Path) -> None:
    n = 1024
    c = n / 2
    im = Image.new("RGB", (n, n), (247, 246, 241))
    d = ImageDraw.Draw(im)

    def at(a, r):                       # a clock angle (clockwise from 12) and a radius, in pixels
        return c + r * math.sin(a), c - r * math.cos(a)

    for m in range(60):
        a, big = m / 60 * 2 * math.pi, m % 5 == 0
        d.line([at(a, c - 24), at(a, c - (78 if big else 48))], fill=(29, 32, 36), width=12 if big else 4)
    f = _font(112)
    for h in range(1, 13):
        x, y = at(h / 12 * 2 * math.pi, c - 165)
        d.text((x, y), str(h), font=f, fill=(29, 32, 36), anchor="mm")
    hour, minute = g.TARGET

    def hand(a, length, back, width):
        d.line([at(a + math.pi, back), at(a, length)], fill=(17, 19, 22), width=width)
        d.ellipse([at(a, length)[0] - width / 2, at(a, length)[1] - width / 2,
                   at(a, length)[0] + width / 2, at(a, length)[1] + width / 2], fill=(17, 19, 22))

    hand((hour + minute / 60) / 12 * 2 * math.pi, 250, 50, 30)
    hand(minute / 60 * 2 * math.pi, 410, 60, 18)
    d.ellipse([c - 26, c - 26, c + 26, c + 26], fill=(17, 19, 22))
    im.save(path)


def clock_top(path: Path) -> tuple[int, int]:
    """The flat top behind the slope: image right is the robot's right (-y), image up is away from it (+x)."""
    depth, width = g.BACK_X - g.SLOPE_X, g.CLOCK_W
    w, h = 1100, int(round(1100 * depth / width))
    im = Image.new("RGB", (w, h), (28, 30, 33))
    d = ImageDraw.Draw(im)
    f = _font(46)
    for name, label in (("hour", "HOUR"), ("min", "MIN")):
        y = g.BUTTON_Y[name]
        px = (g.CLOCK_W / 2 - y) / g.CLOCK_W * w
        py = (g.BACK_X - (g.BUTTON_X - g.CAP / 2 - 0.009)) / depth * h
        d.text((px, py), label, font=f, fill=(233, 236, 239), anchor="mm")
    im.save(path)
    return w, h


# -- USD helpers -------------------------------------------------------------------------------------------------------
def never_sleep(prim) -> None:
    """Keep an articulation awake: drums that are only teleported would otherwise fall asleep at once, and a sleeping
    body's new pose never reaches the rendered image."""
    prim.AddAppliedSchema("PhysxArticulationAPI")
    prim.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    prim.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)


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


def body(stage, path: str, mass: float | None, pos=(0.0, 0.0, 0.0)):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
        UsdPhysics.MassAPI.Apply(xf.GetPrim()).CreateMassAttr(mass)
    return xf


def cube(stage, path, size, pos, color, collide=True, orient=None):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(c)
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if orient is not None:
        x.AddOrientOp().Set(orient)
    x.AddScaleOp().Set(Gf.Vec3f(*size))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def basis_quat(x_axis, y_axis, z_axis) -> Gf.Quatf:
    """The rotation taking the local x, y, z axes to these (orthonormal, right-handed) directions."""
    m = Gf.Matrix3d(*x_axis, *y_axis, *z_axis)      # rows: images of the local axes (USD's row-vector convention)
    q = Gf.Rotation(Gf.Matrix4d(m, Gf.Vec3d(0, 0, 0)).ExtractRotation()).GetQuat()
    out = Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary()))
    rot = Gf.Rotation(Gf.Quatd(out.GetReal(), Gf.Vec3d(out.GetImaginary())))
    assert (rot.TransformDir(Gf.Vec3d(1, 0, 0)) - Gf.Vec3d(*x_axis)).GetLength() < 1e-6
    assert (rot.TransformDir(Gf.Vec3d(0, 0, 1)) - Gf.Vec3d(*z_axis)).GetLength() < 1e-6
    return out


def quad(stage, path, corners, uvs, mat):
    """A textured quad: corners in order bottom-left, bottom-right, top-right, top-left of the image."""
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in corners])
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    nrm = np.cross(np.subtract(corners[1], corners[0]), np.subtract(corners[3], corners[0]))
    m.CreateNormalsAttr([Gf.Vec3f(*(nrm / np.linalg.norm(nrm)))] * 4)
    st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    st.Set([Gf.Vec2f(*uv) for uv in uvs])
    m.CreateDoubleSidedAttr(True)
    bind(m, mat)
    return m


def prismatic(stage, path, body0, body1, pos0, rot0, axis, limits, stiffness=0.0, damping=0.0):
    j = UsdPhysics.PrismaticJoint.Define(stage, path)
    j.CreateAxisAttr(axis)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    j.CreateLocalRot0Attr(rot0)
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateTargetPositionAttr(0.0)
    return j


# -- assets ------------------------------------------------------------------------------------------------------------
def drum_texture(name: str, path: Path) -> None:
    """The drum's surface unrolled: image x runs round the drum (the turn about +y, from the front toward the top of
    the slope), image y runs along the axis from +y (top) to -y. A number reads upright at the front when it is drawn
    with its top toward image x and its lines running down the image. At joint angle 0 the start time faces the
    front; turning the drum by -step brings the next number round."""
    numbers, _, length = g.DRUMS[name]
    circ = 2 * math.pi * g.drum_radius(name)
    w = 2048
    px = w / circ                       # pixels per metre, the same along both image axes
    h = int(round(length * px))
    im = Image.new("RGB", (w, h), tuple(int(255 * c) for c in g.PAPER))
    font = _font(int(0.024 * px))
    band = int(round(g.PITCH * px))
    for k, text in enumerate(numbers):
        tile = Image.new("RGBA", (h, band), (0, 0, 0, 0))     # upright: x along the number's line, y its height
        ImageDraw.Draw(tile).text((h / 2, band / 2), text, font=font, fill=tuple(int(255 * c) for c in g.INK),
                                  anchor="mm")
        tile = tile.rotate(-90, expand=True)                   # top toward image x, line running down the image
        x = ((k - g.drum_start(name)) % len(numbers)) / len(numbers) * w     # the start number faces the front
        for dx in (-w, 0, w):
            im.paste(tile, (int(round(x + dx - tile.width / 2)), 0), tile)
    im.save(path)


def drum(stage, r: str, name: str):
    numbers, _, length = g.DRUMS[name]
    rad, centre = g.drum_radius(name), g.drum_centre(name)
    link = f"{r}/drum_{name}"
    body(stage, link, 0.05, tuple(centre))
    tex = f"drum_{name}.png"
    drum_texture(name, TEX / tex)
    mat = material(stage, f"{r}/Looks/drum_{name}", (1, 1, 1), roughness=0.55, texture=f"./textures/{tex}")
    segs = 96
    n0 = np.asarray(g.N)
    pts, uvs, idx, counts = [], [], [], []
    for j in range(segs + 1):
        th = 2 * math.pi * j / segs
        c, s_ = math.cos(th), math.sin(th)
        radial = np.array([n0[0] * c + n0[2] * s_, 0.0, -n0[0] * s_ + n0[2] * c])     # N turned by th about +y
        for side in (1, -1):
            pts.append(Gf.Vec3f(*(radial * rad + np.array([0.0, side * length / 2, 0.0]))))
    for j in range(segs):
        a, b2, c2, d2 = 2 * j, 2 * j + 1, 2 * j + 3, 2 * j + 2
        counts.append(4)
        idx += [a, b2, c2, d2]
        u0, u1 = j / segs, (j + 1) / segs
        uvs += [Gf.Vec2f(u0, 1), Gf.Vec2f(u0, 0), Gf.Vec2f(u1, 0), Gf.Vec2f(u1, 1)]
    m = UsdGeom.Mesh.Define(stage, f"{link}/surface")
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr(counts)
    m.CreateFaceVertexIndicesAttr(idx)
    st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    st.Set(uvs)
    m.CreateDoubleSidedAttr(True)
    bind(m, mat)
    j = UsdPhysics.RevoluteJoint.Define(stage, f"{r}/joints/drum_{name}")
    j.CreateAxisAttr("Y")
    j.CreateBody0Rel().SetTargets([Sdf.Path(f"{r}/base")])
    j.CreateBody1Rel().SetTargets([Sdf.Path(link)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*centre))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.0)
    drive.CreateTargetPositionAttr(0.0)
    prim = stage.GetPrimAtPath(link)
    prim.AddAppliedSchema("PhysxRigidBodyAPI")
    prim.CreateAttribute("physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool).Set(True)


def build_bedside_clock():
    stage, root = stage_for("bedside_clock")
    r = "/bedside_clock"
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    never_sleep(root.GetPrim())
    body(stage, f"{r}/base", 0.6)
    fixed = UsdPhysics.FixedJoint.Define(stage, f"{r}/joints/root_joint")
    fixed.CreateBody1Rel().SetTargets([Sdf.Path(f"{r}/base")])

    # the case: the profile extruded along y (one convex mesh)
    prof = list(g.PROFILE)
    y0, y1 = g.CLOCK_Y - g.CLOCK_W / 2, g.CLOCK_Y + g.CLOCK_W / 2
    pts = [Gf.Vec3f(x, y0, z) for x, z in prof] + [Gf.Vec3f(x, y1, z) for x, z in prof]
    k = len(prof)
    faces = [list(range(k))[::-1], list(range(k, 2 * k))]
    faces += [[i, (i + 1) % k, k + (i + 1) % k, k + i] for i in range(k)]
    case = UsdGeom.Mesh.Define(stage, f"{r}/base/case")
    case.CreatePointsAttr(pts)
    case.CreateFaceVertexCountsAttr([len(f) for f in faces])
    case.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    case.CreateDisplayColorAttr([Gf.Vec3f(*BODY)])
    UsdPhysics.CollisionAPI.Apply(case.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(case.GetPrim()).CreateApproximationAttr("convexHull")
    bind(case, material(stage, f"{r}/Looks/case", BODY, roughness=0.6))

    rot_disp = basis_quat(g.N, g.U, g.V)            # local x: out of the slope, y: display right, z: display up
    bezel_mat = material(stage, f"{r}/Looks/bezel", g.PAPER, roughness=0.6)
    for i, sv in enumerate((1, -1)):
        dv = sv * (g.WINDOW_H / 2 + g.BEZEL[1] / 2)
        bar = cube(stage, f"{r}/base/bezel{i}", (g.BEZEL[2], g.BEZEL[0], g.BEZEL[1]),
                   tuple(g.DISP_C + g.V * dv + g.N * g.BEZEL[2] / 2), g.PAPER, orient=rot_disp)
        bind(bar, bezel_mat)
    ink = material(stage, f"{r}/Looks/ink", g.INK, roughness=0.5)
    for i, dv in enumerate((0.006, -0.006)):
        dot = cube(stage, f"{r}/base/colon{i}", (0.002, 0.004, 0.004),
                   tuple(g.DISP_C + g.U * g.COLON_U + g.V * dv + g.N * 0.001), g.INK, collide=False, orient=rot_disp)
        bind(dot, ink)

    tw, th = clock_top(TEX / "clock_top.png")
    top_mat = material(stage, f"{r}/Looks/top", (1, 1, 1), roughness=0.6, texture="./textures/clock_top.png")
    zt = g.TOP_Z + 0.0005
    quad(stage, f"{r}/base/top_print", [(g.SLOPE_X, y1, zt), (g.SLOPE_X, y0, zt), (g.BACK_X, y0, zt), (g.BACK_X, y1, zt)],
         [(0, 0), (1, 0), (1, 1), (0, 1)], top_mat)

    # number drums: free-turning on axes along y, driven by task.py to the number to show
    for name in g.DRUMS:
        drum(stage, r, name)

    # buttons: spring-loaded, sliding down
    ident = Gf.Quatf(1, 0, 0, 0)
    cap_mat = material(stage, f"{r}/Looks/cap", CAP_GREY, roughness=0.5)
    for name in ("hour", "min"):
        pos = (g.BUTTON_X, g.BUTTON_Y[name], g.TOP_Z + g.CAP_H / 2)
        body(stage, f"{r}/{name}_btn", 0.01, pos)
        c = cube(stage, f"{r}/{name}_btn/cap", (g.CAP, g.CAP, g.CAP_H), (0, 0, 0), CAP_GREY)
        bind(c, cap_mat)
        prismatic(stage, f"{r}/joints/{name}_btn", f"{r}/base", f"{r}/{name}_btn", pos, ident, "Z", (-g.TRAVEL, 0.0))

    stage.GetRootLayer().Save()


def build_analog_clock():
    stage, root = stage_for("analog_clock")
    r = "/analog_clock"
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    c = g.ANALOG_C
    n = np.array([-math.cos(g.ANALOG_LEAN), 0.0, math.sin(g.ANALOG_LEAN)])
    up = np.array([math.sin(g.ANALOG_LEAN), 0.0, math.cos(g.ANALOG_LEAN)])
    right = np.array([0.0, -1.0, 0.0])
    rot = basis_quat(n, right, up)
    frame = UsdGeom.Xform.Define(stage, f"{r}/face_frame")
    frame.AddTranslateOp().Set(Gf.Vec3d(*c))
    frame.AddOrientOp().Set(rot)
    rim = UsdGeom.Cylinder.Define(stage, f"{r}/face_frame/rim")
    rim.CreateRadiusAttr(g.ANALOG_R + 0.01)
    rim.CreateHeightAttr(RIM_DEPTH)
    rim.CreateAxisAttr("X")
    rim.AddTranslateOp().Set(Gf.Vec3d(-RIM_DEPTH / 2 - 0.0005, 0, 0))
    rim.CreateDisplayColorAttr([Gf.Vec3f(*RIM)])
    UsdPhysics.CollisionAPI.Apply(rim.GetPrim())
    bind(rim, material(stage, f"{r}/Looks/rim", RIM, roughness=0.35))
    analog_face(TEX / "analog_face.png")
    face_mat = material(stage, f"{r}/Looks/face", (1, 1, 1), roughness=0.55, texture="./textures/analog_face.png")
    m = UsdGeom.Mesh.Define(stage, f"{r}/face_frame/face")
    segs = 96
    pts = [Gf.Vec3f(0.0005, 0, 0)] + [Gf.Vec3f(0.0005, g.ANALOG_R * math.cos(a), g.ANALOG_R * math.sin(a))
                                      for a in np.linspace(0, 2 * math.pi, segs, endpoint=False)]
    uv0 = [Gf.Vec2f(0.5, 0.5)] + [Gf.Vec2f(0.5 + 0.5 * math.cos(a), 0.5 + 0.5 * math.sin(a))
                                  for a in np.linspace(0, 2 * math.pi, segs, endpoint=False)]
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr([3] * segs)
    idx = [i for k in range(segs) for i in (0, 1 + k, 1 + (k + 1) % segs)]
    m.CreateFaceVertexIndicesAttr(idx)
    st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    st.Set([uv0[i] for i in idx])
    m.CreateDoubleSidedAttr(True)
    bind(m, face_mat)
    wood = material(stage, f"{r}/Looks/wood", WOOD, roughness=0.8)
    # an easel: a back board under the rim's back, from the foot to the clock's centre, and a ledge the rim stands on
    back = c - n * (RIM_DEPTH + 0.0005 + 0.006)              # the board's mid-plane: its front on the rim's back
    s_low = (0.012 - back[2]) / up[2]                        # where the board meets the foot, along up
    for path, size, pos in ((f"{r}/board", (0.012, 0.06, -s_low), back + up * s_low / 2),
                            (f"{r}/ledge", (RIM_DEPTH + 0.0225, 0.10, 0.012),
                             c - n * (RIM_DEPTH / 2 + 0.00075) - up * (g.ANALOG_R + 0.01 + 0.006))):
        bind(cube(stage, path, size, tuple(pos), WOOD, orient=rot), wood)
    foot = cube(stage, f"{r}/foot", (0.14, 0.16, 0.012), (c[0] + 0.02, c[1], 0.006), WOOD)
    bind(foot, wood)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    TEX.mkdir(parents=True, exist_ok=True)
    build_bedside_clock()
    build_analog_clock()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
