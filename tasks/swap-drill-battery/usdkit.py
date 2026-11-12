"""usdkit: small helpers for writing articulated USD assets with pxr, shared by build_assets.py.

Pure pxr + PIL (Isaac Sim's Python, no simulator needed):  python build_assets.py

Conventions
- Stage units are metres, z up. Every asset's origin is where task.py places it (usually its base on the table).
- Colliders are primitives (box, cylinder, capsule), so PhysX gets convex shapes and contact is predictable.
- Every joint carries a zero-gain drive with a finite max force (DRIVE_MAX_FORCE): Isaac Lab reads that as the
  joint's effort limit, which is 0 without it (measured: springs then push with no force). Springs and dampers are
  set in task.py through ``UsdBody.drives`` (scenekit), in SI units (N/m, N m/rad).
- Revolute limits are given to :func:`joint` in RADIANS and converted to the degrees USD expects (measured: a
  105-degree limit reads 1.8326 rad in the running sim).
- A fixed base: ArticulationRootAPI on the asset's top Xform plus a world->base fixed joint (see articulation_root).
- Filtered pairs (no_collide) do stop collisions (measured: the printer lid's lip shuts inside the shell).
"""

from __future__ import annotations

import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

_AXES = {"X": Gf.Vec3d(1, 0, 0), "Y": Gf.Vec3d(0, 1, 0), "Z": Gf.Vec3d(0, 0, 1)}


# -- stage, materials ------------------------------------------------------------------------------------------------
def stage_for(out: Path, name: str):
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"{name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    return stage


def material(stage, path: str, color, roughness: float = 0.5, metallic: float = 0.0, texture: str | None = None):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    if texture:
        st = UsdShade.Shader.Define(stage, f"{path}/st")
        st.CreateIdAttr("UsdPrimvarReader_float2")
        st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        st.CreateOutput("result", Sdf.ValueTypeNames.Float2)
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
    prim = prim.GetPrim() if hasattr(prim, "GetPrim") else prim
    UsdShade.MaterialBindingAPI.Apply(prim)
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


DRIVE_MAX_FORCE = 1000.0   # N or N m: far above any spring force in these tasks


# -- bodies and shapes -----------------------------------------------------------------------------------------------
def body(stage, path: str, mass: float | None, pos=(0.0, 0.0, 0.0), rot_wxyz=(1.0, 0.0, 0.0, 0.0),
         inertia: float | None = None):
    """A rigid body Xform. ``mass=None`` leaves the mass to be computed from colliders (density 1000)."""
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddOrientOp().Set(Gf.Quatf(*rot_wxyz))
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    if mass is not None:
        m = UsdPhysics.MassAPI.Apply(xf.GetPrim())
        m.CreateMassAttr(mass)
        if inertia is not None:  # bodies without colliders need their inertia given
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia, inertia, inertia))
            m.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    return xf


def articulation_root(stage, root_body: str, fixed: bool) -> None:
    """Make ``root_body`` the articulation's root link. ``fixed`` bolts it to the world (the task places the asset).

    A fixed base needs the ArticulationRootAPI on the asset's top Xform (not on the body) and a world->base fixed
    joint: PhysX then makes that joint the articulation's root joint (measured: is_fixed_base True). With the API on
    the body itself the articulation is floating and the joint a soft external constraint, so the printer could be
    shoved about (measured: is_fixed_base False, a 6.5 kg base pushed 1.3 cm by its own door)."""
    body_prim = stage.GetPrimAtPath(root_body)
    prim = body_prim.GetParent() if fixed else body_prim
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxArticulationAPI")
    prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    prim.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    prim.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    if fixed:
        j = UsdPhysics.FixedJoint.Define(stage, f"{root_body}_to_world")
        # body0 = the top Xform (not a rigid body, so this is still the world), so the anchor sits where the task
        # spawns the asset rather than at the world origin (measured: anchored at the origin, the printer first sat at
        # (0, 0, 0), its seated cartridge fell out, and the reset then teleported the printer into it)
        j.CreateBody0Rel().SetTargets([prim.GetPath()])
        j.CreateBody1Rel().SetTargets([Sdf.Path(root_body)])


def _shape(prim_def, color, collide: bool, mat, pos, rot_wxyz, visible=True):
    prim_def.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if rot_wxyz is not None:
        prim_def.AddOrientOp().Set(Gf.Quatf(*rot_wxyz))
    prim_def.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collide:
        UsdPhysics.CollisionAPI.Apply(prim_def.GetPrim())
    if mat is not None:
        bind(prim_def, mat)
    if not visible:
        UsdGeom.Imageable(prim_def).MakeInvisible()
    return prim_def


def box(stage, path, size, pos=(0, 0, 0), color=(0.5, 0.5, 0.5), collide=True, mat=None, rot_wxyz=None,
        visible=True):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    p = _shape(c, color, collide, mat, pos, rot_wxyz, visible)
    p.AddScaleOp().Set(Gf.Vec3f(*size))
    return p


def cylinder(stage, path, radius, height, pos=(0, 0, 0), color=(0.5, 0.5, 0.5), axis="Z", collide=True, mat=None,
             rot_wxyz=None, visible=True):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(height)
    c.CreateAxisAttr(axis)
    return _shape(c, color, collide, mat, pos, rot_wxyz, visible)


def capsule(stage, path, radius, height, pos=(0, 0, 0), color=(0.5, 0.5, 0.5), axis="Z", collide=True, mat=None,
            rot_wxyz=None, visible=True):
    c = UsdGeom.Capsule.Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(height)
    c.CreateAxisAttr(axis)
    return _shape(c, color, collide, mat, pos, rot_wxyz, visible)


def sphere(stage, path, radius, pos=(0, 0, 0), color=(0.5, 0.5, 0.5), collide=True, mat=None, visible=True):
    c = UsdGeom.Sphere.Define(stage, path)
    c.CreateRadiusAttr(radius)
    return _shape(c, color, collide, mat, pos, None, visible)


def arc_of_boxes(stage, prefix, radius, a0, a1, n, size, z, color, axis_normal="Z", collide=True, mat=None,
                 center=(0.0, 0.0, 0.0)):
    """``n`` boxes along an arc of ``radius`` (centre at the parent's origin) from angle a0 to a1, each tangent to
    it: a cheap convex approximation of a curved wall (hook, ring, roll cradle). ``axis_normal`` is the arc's
    plane normal ("Z": arc in the xy plane at height z; "Y": arc in the xz plane at y=z), offset by ``center``."""
    prims = []
    for k in range(n):
        a = a0 + (a1 - a0) * (k + 0.5) / n
        if axis_normal == "Z":
            pos = (radius * math.cos(a), radius * math.sin(a), z)
            rot = quat_axis_angle((0, 0, 1), a)
        else:  # arc in the xz plane, offset z along y
            pos = (radius * math.cos(a), z, radius * math.sin(a))
            rot = quat_axis_angle((0, 1, 0), -a)
        pos = (pos[0] + center[0], pos[1] + center[1], pos[2] + center[2])
        prims.append(box(stage, f"{prefix}_{k:02d}", size, pos, color, collide, mat, rot))
    return prims


# -- joints ---------------------------------------------------------------------------------------------------------
def quat_axis_angle(axis, angle: float):
    """(w, x, y, z) for a rotation of ``angle`` rad about unit ``axis``."""
    s = math.sin(angle / 2)
    return (math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def joint(stage, path, kind: str, body0: str | None, body1: str, pos0, axis: str = "Z", pos1=(0.0, 0.0, 0.0),
          rot0_wxyz=(1.0, 0.0, 0.0, 0.0), rot1_wxyz=(1.0, 0.0, 0.0, 0.0), limits=None):
    """A ``"linear"`` (prismatic, metres) or ``"angular"`` (revolute, limits given in RADIANS) joint along ``axis`` in
    body0's frame at ``pos0``. ``body0=None`` attaches body1 to the world."""
    j = (UsdPhysics.PrismaticJoint if kind == "linear" else UsdPhysics.RevoluteJoint).Define(stage, path)
    j.CreateAxisAttr(axis)
    if body0:
        j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    j.CreateLocalRot0Attr(Gf.Quatf(*rot0_wxyz))
    j.CreateLocalPos1Attr(Gf.Vec3f(*pos1))
    j.CreateLocalRot1Attr(Gf.Quatf(*rot1_wxyz))
    # A drive with a finite max force: Isaac Lab reads it as the joint's effort limit. Without one the limit is 0 and
    # the task's springs (UsdBody.drives) push with no force at all. Stiffness/damping here are overridden by task.py.
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), kind)
    drive.CreateTypeAttr("force")
    drive.CreateMaxForceAttr(DRIVE_MAX_FORCE)
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.0)
    if limits is not None:
        lo, hi = limits
        if kind == "angular":
            lo, hi = math.degrees(lo), math.degrees(hi)
        j.CreateLowerLimitAttr(lo)
        j.CreateUpperLimitAttr(hi)
    return j


def fixed(stage, path, body0: str, body1: str, pos0=(0.0, 0.0, 0.0), pos1=(0.0, 0.0, 0.0)):
    j = UsdPhysics.FixedJoint.Define(stage, path)
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    j.CreateLocalPos1Attr(Gf.Vec3f(*pos1))
    return j


def no_collide(stage, a: str, b: str) -> None:
    """Stop two bodies' colliders from touching each other (for parts that overlap by design)."""
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(a)).CreateFilteredPairsRel().AddTarget(Sdf.Path(b))


# -- printed labels ------------------------------------------------------------------------------------------------
def text_png(path: Path, size_px, lines, bg=(250, 250, 248), fg=(20, 22, 26), font_px=48,
             font="/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf") -> None:
    """A label image: ``lines`` of text centred top to bottom. Fonts: DejaVu (Bitstream Vera licence), list it in
    the README's asset table."""
    from PIL import Image, ImageDraw, ImageFont

    w, h = size_px
    im = Image.new("RGB", (w, h), bg)
    d = ImageDraw.Draw(im)
    f = ImageFont.truetype(font, font_px)
    step = h / (len(lines) + 1)
    for k, line in enumerate(lines):
        d.text((w / 2, step * (k + 1)), line, font=f, fill=fg, anchor="mm")
    path.parent.mkdir(parents=True, exist_ok=True)
    im.save(path)


def quad(stage, path, w, h, pos, normal: str, mat):
    """A flat textured rectangle (w along the first in-plane axis, h along the second) facing ``normal``
    ("+Z", "-X", "+X", "+Y", "-Y"), for printed labels. Visual only."""
    hw, hh = w / 2, h / 2
    if normal == "+Z":
        pts = [(-hh, -hw, 0), (-hh, hw, 0), (hh, hw, 0), (hh, -hw, 0)]
    elif normal == "-X":
        pts = [(0, hw, -hh), (0, -hw, -hh), (0, -hw, hh), (0, hw, hh)]
    elif normal == "+X":
        pts = [(0, -hw, -hh), (0, hw, -hh), (0, hw, hh), (0, -hw, hh)]
    elif normal == "+Y":
        pts = [(hw, 0, -hh), (-hw, 0, -hh), (-hw, 0, hh), (hw, 0, hh)]
    else:  # "-Y"
        pts = [(-hw, 0, -hh), (hw, 0, -hh), (hw, 0, hh), (-hw, 0, hh)]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(p[0] + pos[0], p[1] + pos[1], p[2] + pos[2]) for p in pts])
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    m.CreateDoubleSidedAttr(True)
    bind(m, mat)
    return m
