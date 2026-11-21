"""partkit: rigid parts built from boxes, cylinders and spheres (optionally hinged), with optional compliant contact,
for yam_arena task files. Adapted from a part-building helper of an earlier yam_arena task and extended
for the charger tasks: fixed parts without hinges, sphere colliders, per-part contact offset, link poses,
on_physics / on_pre_reset / on_spawn hooks, prismatic links, a textured screen quad, contact reports on every part
body and a gripper force cap. Never writes object poses or scores.

Keep this file next to the task file:

    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    from partkit import Hinge, part, on_step, on_reset, weld

yam_arena's task-file objects are single primitives. A ``Part`` is one rigid body made of several box and cylinder
colliders. Give it ``hinges`` and it becomes an articulation: each hinge is a link on a revolute joint about an axis
of the part frame, hung from the part's body or from another hinge, with optional angle limits and a spring and
damper drive (a spring toward its start angle, or a pure damper). A part without a mass is a fixed base: its body is
fixed to the world.

``weld(part, to_part, link)`` joins a loose part to a link of another part with a fixed joint that the task can
release later with ``set_weld(part, False)``.

Importing this module hooks ``yam_arena.scene`` (and the embodiment's step and reset) without modifying yam_arena:
``success()/score()`` receive a :class:`PartState` (the body origin's pose plus joint angles) for parts.
``on_step`` and ``on_reset`` register callbacks that run after every control step and after every reset.

Pure Python at import time; everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena import task_file
from yam_arena.task_file import Box, ObjectState

__all__ = ["Hinge", "Part", "PartState", "part", "weld", "set_weld", "set_targets", "finger_points", "on_reset", "on_step",
           "on_spawn", "set_screen", "set_grip_force", "env"]

Color = tuple[float, float, float]
# box: (centre, size, colour); cylinder: (centre, radius, height, colour[, axis "X" | "Y" | "Z"]); part frame, m


@dataclass(frozen=True)
class Hinge:
    """A link on a revolute joint. Its shapes are given in the part frame with the joint at 0; ``anchor`` is a point on
    the joint axis (part frame) and ``axis`` one of "X", "Y", "Z". The link starts at ``start`` (rad). A drive of
    ``rate`` N m/rad pulls it toward ``start`` and ``damping`` N m s/rad resists its speed."""

    name: str
    anchor: tuple[float, float, float]
    axis: str | tuple = "Z"  # "X" | "Y" | "Z" or a direction vector in the part frame
    boxes: tuple = ()
    cylinders: tuple = ()
    spheres: tuple = ()  # (centre, radius, colour), part frame
    capsules: tuple = ()  # (centre, radius, cylinder length, colour, axis), part frame; PhysX native capsule shape
    parent: str = ""  # "" hangs it from the part's body, otherwise from the hinge of that name
    lower: float | None = None  # rad
    upper: float | None = None
    start: float = 0.0
    rate: float = 0.0
    damping: float = 0.0
    mass: float = 0.05
    linear: bool = False  # prismatic along ``axis`` (limits, start and target in m; rate in N/m)
    target: float | None = None  # drive target if not ``start`` (e.g. a preloaded spring against a stop)


@dataclass(frozen=True)
class Part(Box):
    """A rigid part. It subclasses Box only so yam_arena's task-file checks accept it; ``size`` is its bounding box
    and ``pos`` is the body origin. ``mass=None`` fixes its body to the world."""

    boxes: tuple = ()
    cylinders: tuple = ()
    spheres: tuple = ()
    hinges: tuple[Hinge, ...] = ()
    friction: float = 0.8
    contact_offset: float = 0.0005  # m, PhysX contactOffset of every collider of this part
    # m, PhysX (min) torsional patch radius of every collider: torsional friction over a finite contact patch (rubber
    # pads on a round handle). 0 = PhysX default (point contacts carry no torsional friction).
    torsional_patch: float = 0.0
    body_mass: float = 1.0
    self_collide: bool = False  # links collide with each other (joined pairs never do)
    # passive compliant contact (PhysX spring-damper contact on this part's material): normal force = stiffness x
    # penetration + damping x approach speed, per contact point. 0 = rigid contact.
    stiffness: float = 0.0  # N/m
    contact_damping: float = 0.0  # N s/m
    visuals: tuple = ()  # (centre, size, colour) boxes drawn but not colliding
    screen: tuple | None = None  # (centre, size_xy, png path): textured quad facing +Z on the body (an LCD)
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not (self.boxes or self.cylinders or self.spheres):
            raise ValueError(f"{self.name}: a Part needs at least one shape")


def _turn(base, v):
    """(w, x, y, z) of the shortest rotation taking unit ``base`` onto the direction of ``v``."""
    a = np.asarray(base, float)
    b = np.asarray(v, float) / np.linalg.norm(v)
    c = float(np.clip(a @ b, -1.0, 1.0))
    if c > 1 - 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    if c < -1 + 1e-12:
        k = np.cross(a, [1.0, 0.0, 0.0])
        k = k if np.linalg.norm(k) > 1e-6 else np.cross(a, [0.0, 1.0, 0.0])
        k /= np.linalg.norm(k)
        return (0.0, float(k[0]), float(k[1]), float(k[2]))
    k = np.cross(a, b)
    k /= np.linalg.norm(k)
    h = math.acos(c) / 2
    return (math.cos(h), *(float(x) * math.sin(h) for x in k))


def _cyl(spec):
    centre, radius, height, color = spec[:4]
    return centre, radius, height, color, (spec[4] if len(spec) > 4 else "Z")


def _cyl_half(spec):
    _, r, h, _, axis = _cyl(spec)
    if not isinstance(axis, str):  # any direction: conservative bound
        return np.full(3, r + h / 2)
    return np.array([h / 2 if axis == "X" else r, h / 2 if axis == "Y" else r, h / 2 if axis == "Z" else r])


def part(name, pos, boxes=(), mass=None, yaw=0.0, hinges=(), color=(0.5, 0.5, 0.5), friction=0.8, cylinders=(),
         body_mass=1.0, self_collide=False, stiffness=0.0, contact_damping=0.0, visuals=(), spheres=(),
         contact_offset=0.0005, torsional_patch=0.0, screen=None) -> Part:
    """Build a Part; ``size`` is filled in from the body's shapes."""
    half = ([np.array(b[1]) / 2 for b in boxes] + [_cyl_half(c) for c in cylinders]
            + [np.full(3, sp[1]) for sp in spheres])
    centres = [np.array(b[0]) for b in boxes] + [np.array(c[0]) for c in cylinders] + [np.array(sp[0]) for sp in spheres]
    lo = np.min([c - h for c, h in zip(centres, half)], axis=0)
    hi = np.max([c + h for c, h in zip(centres, half)], axis=0)
    return Part(name=name, pos=tuple(float(v) for v in pos), size=tuple((hi - lo).tolist()), mass=mass,
                color=tuple(float(c) for c in color), yaw=float(yaw),
                boxes=tuple(boxes), cylinders=tuple(cylinders), spheres=tuple(spheres), hinges=tuple(hinges),
                friction=friction, contact_offset=float(contact_offset), torsional_patch=float(torsional_patch),
                body_mass=body_mass if mass is None else mass, self_collide=self_collide, stiffness=float(stiffness),
                contact_damping=float(contact_damping), visuals=tuple(visuals), screen=screen)


@dataclass(frozen=True)
class PartState(ObjectState):
    """The body origin's pose (``pos``, ``quat_xyzw``), velocities, bounds, and each hinge's angle (rad)."""

    joints: dict = field(default_factory=dict)

    def rotation(self) -> np.ndarray:
        x, y, z, w = self.quat_xyzw
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])

    def to_local(self, world_point) -> np.ndarray:
        return self.rotation().T @ (np.asarray(world_point, float) - self.pos)

    def to_world(self, local_point) -> np.ndarray:
        return self.pos + self.rotation() @ np.asarray(local_point, float)


# -- welds --------------------------------------------------------------------------------------------------------

_WELDS: dict[str, tuple[str, str]] = {}  # loose part -> (other part, link name)
_WELD_PATHS: dict[str, list[str]] = {}


def weld(loose: Part, to: Part, link: str) -> None:
    """Join ``loose`` (at its start pose) to ``link`` of ``to`` with a fixed joint. List ``to`` before ``loose``."""
    _WELDS[loose.name] = (to.name, link)


def set_weld(name: str, enabled: bool) -> None:
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    for path in _WELD_PATHS.get(name, []):
        stage.GetPrimAtPath(path).GetAttribute("physics:jointEnabled").Set(bool(enabled))


# -- spawning -----------------------------------------------------------------------------------------------------

_SPECS: dict[str, Part] = {}


def _link_anchor(spec: Part, link: str) -> np.ndarray:
    if not link:
        return np.zeros(3)
    return np.array(next(h.anchor for h in spec.hinges if h.name == link), float)


def _spawn_part(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the part under ``prim_path`` (pxr only): a plain Xform holding the body and links as siblings."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Part = _SPECS[cfg.part_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))

    phys = f"{prim_path}/Looks/physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    pm.func(phys, pm)
    if spec.stiffness > 0:
        mat = PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(phys))
        mat.CreateCompliantContactStiffnessAttr(float(spec.stiffness))
        mat.CreateCompliantContactDampingAttr(float(spec.contact_damping))
        mat.CreateFrictionCombineModeAttr("average")
    looks: dict[Color, str] = {}

    def look(color: Color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.45)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def body(path: str, pos, mass: float):
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
        xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateAngularDampingAttr(0.5)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)  # gripper-contact measurement
        return prim

    def collide(prim, color):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(spec.contact_offset)
        cp.CreateRestOffsetAttr(0.0)
        if spec.torsional_patch > 0:
            cp.CreateTorsionalPatchRadiusAttr(spec.torsional_patch)
            cp.CreateMinTorsionalPatchRadiusAttr(spec.torsional_patch)
        bind_physics_material(str(prim.GetPath()), phys, stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)

    def shapes(path: str, box_specs, cyl_specs, offset=(0.0, 0.0, 0.0), sph_specs=(), cap_specs=()):
        off = np.array(offset, float)
        for i, spec_i in enumerate(box_specs):
            centre, size, color = spec_i[:3]
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - off)))
            if len(spec_i) > 3:  # (axis "X" | "Y" | "Z", angle rad): the box turned about its centre
                axis, angle = spec_i[3]
                v = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[axis]
                sh = math.sin(angle / 2)
                cube.AddOrientOp().Set(Gf.Quatf(math.cos(angle / 2), v[0] * sh, v[1] * sh, v[2] * sh))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            collide(cube.GetPrim(), color)
        for i, c in enumerate(cyl_specs):
            centre, radius, height, color, axis = _cyl(c)
            cyl = UsdGeom.Cylinder.Define(stage, f"{path}/cyl_{i}")
            cyl.CreateAxisAttr(axis if isinstance(axis, str) else "Z")
            cyl.CreateRadiusAttr(radius)
            cyl.CreateHeightAttr(height)
            cyl.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - off)))
            if not isinstance(axis, str):  # a direction vector (part frame): turn the Z cylinder onto it
                cyl.AddOrientOp().Set(Gf.Quatf(*_turn((0.0, 0.0, 1.0), axis)))
            collide(cyl.GetPrim(), color)
        for i, (centre, radius, length, color, axis) in enumerate(cap_specs):
            cap = UsdGeom.Capsule.Define(stage, f"{path}/cap_{i}")
            cap.CreateAxisAttr(axis if isinstance(axis, str) else "Z")
            cap.CreateRadiusAttr(radius)
            cap.CreateHeightAttr(length)
            cap.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - off)))
            if not isinstance(axis, str):
                cap.AddOrientOp().Set(Gf.Quatf(*_turn((0.0, 0.0, 1.0), axis)))
            collide(cap.GetPrim(), color)
        for i, (centre, radius, color) in enumerate(sph_specs):
            sph = UsdGeom.Sphere.Define(stage, f"{path}/sph_{i}")
            sph.CreateRadiusAttr(radius)
            sph.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - off)))
            collide(sph.GetPrim(), color)

    main = f"{prim_path}/body"
    body(main, (0.0, 0.0, 0.0), spec.body_mass)
    if spec.mass is None and not spec.hinges:  # fixed rigid part: a kinematic body that never moves
        PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath(main))
        UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(main)).CreateKinematicEnabledAttr(True)
    shapes(main, spec.boxes, spec.cylinders, sph_specs=spec.spheres)
    for i, (centre, size, color) in enumerate(spec.visuals):  # drawn only: no CollisionAPI
        cube = UsdGeom.Cube.Define(stage, f"{main}/vis_{i}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        bind_visual_material(str(cube.GetPrim().GetPath()), look(color), stage=stage)
    if spec.screen:
        _screen(stage, prim_path, main, spec)
    paths = {"": main}
    for h in spec.hinges:
        path = f"{prim_path}/{h.name}"
        paths[h.name] = path
        body(path, h.anchor, h.mass)
        shapes(path, h.boxes, h.cylinders, offset=h.anchor, sph_specs=h.spheres, cap_specs=h.capsules)
        parent_anchor = _link_anchor(spec, h.parent)
        j = (UsdPhysics.PrismaticJoint if h.linear else UsdPhysics.RevoluteJoint).Define(stage, f"{path}/{h.name}_joint")
        # every body of a part has the part's orientation, so equal joint frames on both sides are consistent;
        # a vector axis (part frame) is the joint's X axis turned onto it
        jq = Gf.Quatf(1, 0, 0, 0) if isinstance(h.axis, str) else Gf.Quatf(*_turn((1.0, 0.0, 0.0), h.axis))
        j.CreateAxisAttr(h.axis if isinstance(h.axis, str) else "X")
        j.CreateBody0Rel().SetTargets([Sdf.Path(paths[h.parent])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(np.array(h.anchor) - parent_anchor)))
        j.CreateLocalRot0Attr(jq)
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(jq)
        u = 1.0 if h.linear else 180 / math.pi  # USD: metres, or degrees with drives per degree
        if h.lower is not None:
            j.CreateLowerLimitAttr(h.lower * u)
            j.CreateUpperLimitAttr(h.upper * u)
        kind = "linear" if h.linear else "angular"
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), kind)
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(h.rate / u)
        drive.CreateDampingAttr(h.damping / u)
        drive.CreateTargetPositionAttr((h.start if h.target is None else h.target) * u)
        state = PhysxSchema.JointStateAPI.Apply(j.GetPrim(), kind)
        state.CreatePositionAttr(h.start * u)
        state.CreateVelocityAttr(0.0)
    if spec.mass is None and spec.hinges:
        fix = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/fix_to_world")
        fix.CreateBody1Rel().SetTargets([Sdf.Path(main)])
        world = UsdGeom.Xformable(stage.GetPrimAtPath(main)).ComputeLocalToWorldTransform(0)
        fix.CreateLocalPos0Attr(Gf.Vec3f(*world.ExtractTranslation()))
        fix.CreateLocalRot0Attr(Gf.Quatf(world.ExtractRotationQuat()))
        fix.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        fix.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    if spec.hinges:
        art_prim = stage.GetPrimAtPath(prim_path if spec.mass is None else main)
        UsdPhysics.ArticulationRootAPI.Apply(art_prim)
        art = PhysxSchema.PhysxArticulationAPI.Apply(art_prim)
        art.CreateEnabledSelfCollisionsAttr(bool(spec.self_collide))
        art.CreateSolverPositionIterationCountAttr(32)
        art.CreateSolverVelocityIterationCountAttr(4)
    for fn in _SPAWN:
        fn(stage, spec, prim_path, paths)
    if spec.name in _WELDS:
        other, link = _WELDS[spec.name]
        ospec = _SPECS[other]
        other_path = prim_path.rsplit("/", 1)[0] + "/" + other
        target = f"{other_path}/body" if not link else f"{other_path}/{link}"
        # the loose part's origin in the link's frame at the start pose (the other part is not turned)
        rel = np.array(spec.pos) - np.array(ospec.pos) - _link_anchor(ospec, link)
        wj = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/weld")
        wj.CreateBody0Rel().SetTargets([Sdf.Path(target)])
        wj.CreateBody1Rel().SetTargets([Sdf.Path(main)])
        wj.CreateLocalPos0Attr(Gf.Vec3f(*rel))
        wj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        wj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        wj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        wj.CreateExcludeFromArticulationAttr(True)
        wj.CreateJointEnabledAttr(True)
        wj.CreateCollisionEnabledAttr(True)  # the parts still touch where they meet, and after the weld is released
        _WELD_PATHS.setdefault(spec.name, []).append(str(wj.GetPrim().GetPath()))
    return root.GetPrim()


def _screen(stage, prim_path, body_path, spec):
    from pxr import Gf, Sdf, UsdGeom, UsdShade
    (cx, cy, cz), (sx, sy), png = spec.screen
    mesh = UsdGeom.Mesh.Define(stage, f"{body_path}/screen")
    mesh.CreatePointsAttr([Gf.Vec3f(cx + sx / 2, cy + sy / 2, cz), Gf.Vec3f(cx - sx / 2, cy + sy / 2, cz),
                           Gf.Vec3f(cx - sx / 2, cy - sy / 2, cz), Gf.Vec3f(cx + sx / 2, cy - sy / 2, cz)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying)
    st.Set([(0, 0), (1, 0), (1, 1), (0, 1)])  # image up = part -Y
    mat = UsdShade.Material.Define(stage, f"{prim_path}/Looks/screen")
    sh = UsdShade.Shader.Define(stage, f"{prim_path}/Looks/screen/shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    tex = UsdShade.Shader.Define(stage, f"{prim_path}/Looks/screen/tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(png)
    reader = UsdShade.Shader.Define(stage, f"{prim_path}/Looks/screen/st")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    rgb = tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    _SCREENS[spec.name] = f"{prim_path}/Looks/screen/tex"


_SCREENS: dict[str, str] = {}


def set_screen(name: str, png: str) -> None:
    """Swap the texture of part ``name``'s screen (display content only, no physics)."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdShade
    UsdShade.Shader(get_current_stage().GetPrimAtPath(_SCREENS[name])).GetInput("file").Set(png)


def _arena_object(spec: Part):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class PartSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_part)
        part_name: str = ""

    class PartObject(Object):
        def _generate_articulation_cfg(self):
            # Implicit actuators carry the springs and dampers; their target is each hinge's start angle.
            actuators = {h.name: ImplicitActuatorCfg(joint_names_expr=[f"{h.name}_joint"], stiffness=h.rate,
                                                     damping=h.damping, friction=0.0) for h in spec.hinges}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            cfg = self._add_initial_pose_to_cfg(cfg)
            cfg.init_state.joint_pos = {f"{h.name}_joint": h.start for h in spec.hinges}
            cfg.init_state.joint_vel = {".*": 0.0}
            return cfg

    kind = ObjectType.ARTICULATION if spec.hinges else ObjectType.RIGID
    return PartObject(
        name=spec.name,
        object_type=kind,
        spawner_cfg=PartSpawnerCfg(part_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2),
                                                                       math.cos(spec.yaw / 2))),
    )


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _part_state(environment, spec: Part) -> PartState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    probe = PartState(pos=pose[:3] - origin, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6],
                      low=np.zeros(3), high=np.zeros(3))
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * np.array(spec.size) / 2
    pts = np.array([probe.to_world(c) for c in corners])
    joints = {}
    if spec.hinges:
        names = list(asset.joint_names)
        q = _np(asset.data.joint_pos)[0]
        joints = {h.name: float(q[names.index(f"{h.name}_joint")]) for h in spec.hinges}
    return PartState(pos=probe.pos, quat_xyzw=probe.quat_xyzw, lin_vel=probe.lin_vel, ang_vel=probe.ang_vel,
                     low=pts.min(0), high=pts.max(0), joints=joints)


def set_targets(environment, name: str, targets: dict[str, float]) -> None:
    """Set drive targets (rad) of some hinges of an articulated part (env 0)."""
    import torch

    asset = environment.unwrapped.scene[name]
    names = list(asset.joint_names)
    ids = [names.index(f"{k}_joint") for k in targets]
    t = torch.tensor([list(targets.values())], dtype=torch.float32, device=asset.device)
    asset.set_joint_position_target(t, joint_ids=ids)


def finger_points(environment) -> list:
    """Points on both arms' finger pads (world, relative to the env origin): at the TCP and at the pad tips, 6 mm
    outside each pad face. For "released" checks in an ``on_step`` hook; score() does not see the robot."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        jp, jn = _np(robot.data.joint_pos)[0], list(robot.joint_names)
        half_gap = max(abs(jp[jn.index(j)]) for j in ("joint7", "joint8") if j in jn)
        r = PartState(pos=pose[:3], quat_xyzw=pose[3:7], lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.zeros(3),
                      high=np.zeros(3)).rotation()
        for along in (0.0, PAD_TIP_PAST_TCP):
            centre = pose[:3] + r @ (np.array(TCP_OFFSET) + [0.0, 0.0, along]) - origin
            pts += [centre + r[:, 0] * side * (half_gap + 0.006) for side in (1.0, -1.0)]
    return pts


# -- hooking yam_arena -------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_PRE: list = []
_PHYS: list = []
_SPAWN: list = []
_ENV: dict = {"env": None, "wrapped": None}
_CFG: dict = {"physics_dt": None, "grip_n": None, "pad_friction": None}


def set_physics_dt(dt: float) -> None:
    """Override the station's 240 Hz physics step for this task (decimation follows control_hz)."""
    _CFG["physics_dt"] = float(dt)


def set_grip_force(newtons: float) -> None:
    """Cap each finger drive of both YAM grippers at ``newtons`` (the SDK default is 50 N)."""
    _CFG["grip_n"] = float(newtons)


def set_pad_friction(friction) -> None:
    """(static, dynamic) friction of a physics material bound to every YAM finger-pad collider when the arms are
    spawned (before play); None keeps the asset as it is (no pad material)."""
    _CFG["pad_friction"] = friction


def _pad_spawner(orig, friction):
    from isaaclab.sim.utils import clone
    inner = getattr(orig, "__wrapped__", orig)

    @clone
    def spawn(prim_path, cfg, *a, **k):
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Usd, UsdPhysics
        prim = inner(prim_path, cfg, *a, **k)
        stage = get_current_stage()
        mat = sim_utils.RigidBodyMaterialCfg(static_friction=friction[0], dynamic_friction=friction[1])
        mat.func(f"{prim_path}/PadMaterial", mat)
        for p in Usd.PrimRange(stage.GetPrimAtPath(prim_path)):
            if p.GetName().startswith(("tip_left", "tip_right")) and p.HasAPI(UsdPhysics.CollisionAPI):
                bind_physics_material(str(p.GetPath()), f"{prim_path}/PadMaterial", stage=stage)
        return prim

    return spawn


def pad_readback(stage, root="/World/envs/env_0") -> dict:
    """Physics material actually bound to the finger-pad colliders: {material path: (static, dynamic, count)}."""
    from pxr import Usd, UsdPhysics, UsdShade
    out = {}
    for p in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies()):
        if p.GetName().startswith(("tip_left", "tip_right")) and p.HasAPI(UsdPhysics.CollisionAPI):
            path = str(UsdShade.MaterialBindingAPI(p).GetDirectBinding("physics").GetMaterialPath()) or "none"
            m = UsdPhysics.MaterialAPI(stage.GetPrimAtPath(path)) if path != "none" else None
            fr = (m.GetStaticFrictionAttr().Get(), m.GetDynamicFrictionAttr().Get()) if m else (None, None)
            n = out.get(path, (None, None, 0))[2]
            out[path] = (*fr, n + 1)
    return out


def on_spawn(fn):
    """Register ``fn(stage, spec, prim_path, paths)`` to author extra USD (e.g. joints) after each part is spawned;
    ``paths`` maps link names ("" = body) to prim paths."""
    _SPAWN.append(fn)
    return fn


def on_pre_reset(fn):
    """Register ``fn()`` to run at the start of every reset, before the settling steps (task state)."""
    _PRE.append(fn)
    return fn


def on_physics(fn):
    """Register ``fn(env, dt)`` to run before every physics substep (also during reset settling). It sees the state
    left by the previous substep; forces it applies through a PhysX tensor view act during the coming substep."""
    _PHYS.append(fn)
    return fn


def link_pose(environment, name: str, link: str) -> tuple[np.ndarray, np.ndarray]:
    """World pose (relative to the env origin) of a link of an articulated part: (pos, 3x3 rotation)."""
    base = environment.unwrapped
    asset = base.scene[name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.body_link_pose_w)[0][list(asset.body_names).index(link)]
    st = PartState(pos=pose[:3] - origin, quat_xyzw=pose[3:7], lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                   low=np.zeros(3), high=np.zeros(3))
    return st.pos, st.rotation()


def _wrap_sim(environment) -> None:
    base = environment.unwrapped
    sim = base.sim
    if _ENV["wrapped"] is sim:
        return
    orig = sim.step

    def step(*a, **k):
        # read the state left by the previous substep, apply this substep's forces, then step
        for fn in _PHYS:
            fn(environment, base.physics_dt)
        return orig(*a, **k)

    sim.step = step
    _ENV["wrapped"] = sim


def on_step(fn):
    """Register ``fn(env)`` to run after every control step, before the task file is scored."""
    _STEP.append(fn)
    return fn


def on_reset(fn):
    """Register ``fn(env)`` to run after every reset, once objects have settled."""
    _RESET.append(fn)
    return fn


def env():
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_hingekit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Part):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = orig_states(environment, plain) if plain else {}
        for n in names:
            if n in _SPECS:
                out[n] = _part_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    orig_cb = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        configure = orig_cb(control_hz)

        def wrapped(env_cfg):
            env_cfg = configure(env_cfg)
            if _CFG["physics_dt"]:
                dt = _CFG["physics_dt"]
                env_cfg.sim.dt = dt
                env_cfg.decimation = max(1, round(1.0 / ((control_hz or 10.0) * dt)))
                env_cfg.sim.render_interval = env_cfg.decimation
            if _CFG["pad_friction"]:
                for arm in ("left_robot", "right_robot"):
                    sp = getattr(env_cfg.scene, arm).spawn
                    sp.make_uninstanceable = True  # pad colliders must be editable prims to take the material
                    sp.func = _pad_spawner(sp.func, _CFG["pad_friction"])
            if _CFG["grip_n"]:
                for arm in ("left_robot", "right_robot"):
                    getattr(env_cfg.scene, arm).actuators["gripper"].joint_effort_limit = _CFG["grip_n"]
            return env_cfg

        return wrapped

    scene.task_object = task_object
    scene.object_states = object_states
    scene.env_cfg_callback = env_cfg_callback
    scene._hingekit_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_hingekit_patched", False):
        return
    base_task_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        orig_ensure = self._ensure_sim

        def ensure(*aa, **kk):  # the physics hook must be live before the reset's settling steps
            r = orig_ensure(*aa, **kk)
            _ENV["env"] = self._env
            _wrap_sim(self._env)
            return r

        self._ensure_sim = ensure
        for fn in _PRE:
            fn()
        try:
            obs = base_reset(self, *a, **k)
        finally:
            del self._ensure_sim
        _ENV["env"] = self._env
        for spec in _SPECS.values():  # drive targets back to each hinge's start angle (springs, dampers)
            if spec.hinges:
                set_targets(self._env, spec.name, {h.name: h.start if h.target is None else h.target for h in spec.hinges})
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._hingekit_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, patch = spec.loader, self.targets[fullname]
        orig_exec = loader.exec_module

        def exec_module(module):
            orig_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena (now for modules already imported, otherwise when they are). Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
assert task_file
