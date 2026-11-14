"""lockkit: rigid parts built from boxes, cylinders and capsules, optionally with hinged links, for yam_arena task
files.

Keep this file next to the task file:

    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    from lockkit import Hinge, part, on_step, on_reset, weld

yam_arena's task-file objects are single primitives. A ``Part`` is one rigid body made of several box and cylinder
colliders. Give it ``hinges`` and it becomes an articulation: each hinge is a link on a revolute joint about an axis
of the part frame, hung from the part's body or from another hinge, with optional angle limits and a spring and
damper drive (a spring toward its start angle, or a pure damper). A part without a mass is a fixed base: its body is
fixed to the world.

``weld(name, part, to_part, rel_pos, rel_quat_wxyz)`` authors a fixed joint, disabled at the start, that holds a loose
part at a given pose in another loose part's frame; the task switches it on with ``set_weld(name, True)``.
``finger_points(env)`` gives points on both grippers' finger pads for hands-off checks.

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

__all__ = ["Hinge", "Part", "PartState", "part", "weld", "set_weld", "set_targets", "on_reset", "on_step", "env",
           "finger_points"]

Color = tuple[float, float, float]
# box: (centre, size, colour[, quat_wxyz]); cylinder: (centre, radius, height, colour[, axis "X" | "Y" | "Z"]);
# capsule: (end_a, end_b, radius, colour), the axis running between the two end centres; part frame, m


@dataclass(frozen=True)
class Hinge:
    """A link on a revolute joint. Its shapes are given in the part frame with the joint at 0; ``anchor`` is a point on
    the joint axis (part frame) and ``axis`` one of "X", "Y", "Z". The link starts at ``start`` (rad). A drive of
    ``rate`` N m/rad pulls it toward ``start`` and ``damping`` N m s/rad resists its speed."""

    name: str
    anchor: tuple[float, float, float]
    axis: str = "Z"
    boxes: tuple = ()
    cylinders: tuple = ()
    capsules: tuple = ()
    parent: str = ""  # "" hangs it from the part's body, otherwise from the hinge of that name
    lower: float | None = None  # rad
    upper: float | None = None
    start: float = 0.0
    rate: float = 0.0
    damping: float = 0.0
    mass: float = 0.05


@dataclass(frozen=True)
class Part(Box):
    """A rigid part. It subclasses Box only so yam_arena's task-file checks accept it; ``size`` is its bounding box
    and ``pos`` is the body origin. ``mass=None`` fixes its body to the world."""

    boxes: tuple = ()
    cylinders: tuple = ()
    capsules: tuple = ()
    hinges: tuple[Hinge, ...] = ()
    friction: float = 0.8
    combine: str = "average"  # PhysX friction combine mode against other materials ("max": this material's wins)
    body_mass: float = 1.0
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not (self.boxes or self.cylinders or self.capsules):
            raise ValueError(f"{self.name}: a Part needs at least one shape")
        if self.mass is None and not self.hinges:
            raise ValueError(f"{self.name}: a fixed Part needs hinges (use a plain Box otherwise)")


def _cyl(spec):
    centre, radius, height, color = spec[:4]
    return centre, radius, height, color, (spec[4] if len(spec) > 4 else "Z")


def _cyl_half(spec):
    _, r, h, _, axis = _cyl(spec)
    return np.array([h / 2 if axis == "X" else r, h / 2 if axis == "Y" else r, h / 2 if axis == "Z" else r])


def part(name, pos, boxes=(), mass=None, yaw=0.0, hinges=(), color=(0.5, 0.5, 0.5), friction=0.8, cylinders=(),
         capsules=(), body_mass=1.0, combine="average") -> Part:
    """Build a Part; ``size`` is filled in from the body's shapes (box rotations ignored: a loose bound)."""
    half = [np.full(3, np.linalg.norm(b[1]) / 2) if len(b) > 3 else np.array(b[1]) / 2 for b in boxes]
    half += [_cyl_half(c) for c in cylinders]
    centres = [np.array(b[0]) for b in boxes] + [np.array(c[0]) for c in cylinders]
    for a, b, r, _ in capsules:
        for e in (a, b):
            centres.append(np.array(e, float))
            half.append(np.full(3, r))
    lo = np.min([c - h for c, h in zip(centres, half)], axis=0)
    hi = np.max([c + h for c, h in zip(centres, half)], axis=0)
    return Part(name=name, pos=tuple(float(v) for v in pos), size=tuple((hi - lo).tolist()), mass=mass, color=color,
                yaw=yaw, boxes=tuple(boxes), cylinders=tuple(cylinders), capsules=tuple(capsules),
                hinges=tuple(hinges), friction=friction, combine=combine,
                body_mass=body_mass if mass is None else mass)


def _quat_x_to(d) -> tuple[float, float, float, float]:
    """(w, x, y, z) of the rotation taking +x onto direction ``d``."""
    d = np.asarray(d, float) / np.linalg.norm(d)
    c = float(d[0])
    axis = np.cross([1.0, 0.0, 0.0], d)
    s = float(np.linalg.norm(axis))
    if s < 1e-9:
        return (1.0, 0.0, 0.0, 0.0) if c > 0 else (0.0, 0.0, 0.0, 1.0)
    ang = math.atan2(s, c)
    axis /= s
    return (math.cos(ang / 2), *(axis * math.sin(ang / 2)))


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

_WELDS: dict[str, list] = {}  # loose part -> [(weld name, other part, rel_pos, rel_quat_wxyz)]
_WELD_PATHS: dict[str, list[str]] = {}


def weld(name: str, loose: Part, to: Part, rel_pos, rel_quat_wxyz=(1.0, 0.0, 0.0, 0.0)) -> None:
    """A fixed joint, disabled at the start, holding ``loose``'s origin at ``rel_pos`` / ``rel_quat_wxyz`` in
    ``to``'s body frame. List ``to`` before ``loose`` in OBJECTS."""
    _WELDS.setdefault(loose.name, []).append((name, to.name, tuple(rel_pos), tuple(rel_quat_wxyz)))


def set_weld(name: str, enabled: bool) -> None:
    """Switch the weld called ``name`` on or off."""
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    for path in _WELD_PATHS.get(name, []):
        stage.GetPrimAtPath(path).GetAttribute("physics:jointEnabled").Set(bool(enabled))


# -- spawning -----------------------------------------------------------------------------------------------------

_SPECS: dict[str, Part] = {}


def _link_anchor(spec: Part, link: str) -> np.ndarray:  # noqa: D103
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
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction,
                                        friction_combine_mode=spec.combine)
    pm.func(phys, pm)
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
        return prim

    def collide(prim, color):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(0.0005)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), phys, stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)

    def shapes(path: str, box_specs, cyl_specs, cap_specs=(), offset=(0.0, 0.0, 0.0)):
        off = np.array(offset, float)
        for i, b in enumerate(box_specs):
            centre, size, color = b[:3]
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - off)))
            if len(b) > 3:
                cube.AddOrientOp().Set(Gf.Quatf(*[float(v) for v in b[3]]))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            collide(cube.GetPrim(), color)
        for i, (a, b, radius, color) in enumerate(cap_specs):
            a, b = np.array(a, float), np.array(b, float)
            cap = UsdGeom.Capsule.Define(stage, f"{path}/cap_{i}")
            cap.CreateAxisAttr("X")
            cap.CreateRadiusAttr(float(radius))
            cap.CreateHeightAttr(float(np.linalg.norm(b - a)))
            cap.AddTranslateOp().Set(Gf.Vec3d(*((a + b) / 2 - off)))
            cap.AddOrientOp().Set(Gf.Quatf(*_quat_x_to(b - a)))
            collide(cap.GetPrim(), color)
        for i, c in enumerate(cyl_specs):
            centre, radius, height, color, axis = _cyl(c)
            cyl = UsdGeom.Cylinder.Define(stage, f"{path}/cyl_{i}")
            cyl.CreateAxisAttr(axis)
            cyl.CreateRadiusAttr(radius)
            cyl.CreateHeightAttr(height)
            cyl.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - off)))
            collide(cyl.GetPrim(), color)

    main = f"{prim_path}/body"
    body(main, (0.0, 0.0, 0.0), spec.body_mass)
    shapes(main, spec.boxes, spec.cylinders, spec.capsules)
    paths = {"": main}
    for h in spec.hinges:
        path = f"{prim_path}/{h.name}"
        paths[h.name] = path
        body(path, h.anchor, h.mass)
        shapes(path, h.boxes, h.cylinders, h.capsules, offset=h.anchor)
        parent_anchor = _link_anchor(spec, h.parent)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{path}/{h.name}_joint")
        j.CreateAxisAttr(h.axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(paths[h.parent])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(np.array(h.anchor) - parent_anchor)))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        if h.lower is not None:
            j.CreateLowerLimitAttr(math.degrees(h.lower))
            j.CreateUpperLimitAttr(math.degrees(h.upper))
        # USD drives are per degree; the spring pulls toward the start angle
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(h.rate * math.pi / 180)
        drive.CreateDampingAttr(h.damping * math.pi / 180)
        drive.CreateTargetPositionAttr(math.degrees(h.start))
        state = PhysxSchema.JointStateAPI.Apply(j.GetPrim(), "angular")
        state.CreatePositionAttr(math.degrees(h.start))
        state.CreateVelocityAttr(0.0)
    if spec.mass is None:
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
        art.CreateEnabledSelfCollisionsAttr(False)
        art.CreateSolverPositionIterationCountAttr(32)
        art.CreateSolverVelocityIterationCountAttr(4)
    for wname, other, rel_pos, rel_quat in _WELDS.get(spec.name, []):
        other_path = prim_path.rsplit("/", 1)[0] + "/" + other
        wj = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/weld_{wname}")
        wj.CreateBody0Rel().SetTargets([Sdf.Path(f"{other_path}/body")])
        wj.CreateBody1Rel().SetTargets([Sdf.Path(main)])
        wj.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in rel_pos]))
        wj.CreateLocalRot0Attr(Gf.Quatf(*[float(v) for v in rel_quat]))
        wj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        wj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        wj.CreateExcludeFromArticulationAttr(True)
        wj.CreateJointEnabledAttr(False)
        wj.CreateCollisionEnabledAttr(True)
        _WELD_PATHS.setdefault(wname, []).append(str(wj.GetPrim().GetPath()))
    return root.GetPrim()


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
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2),
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


def finger_points(environment) -> list:
    """Points on both arms' finger pads (world, relative to the env origin): at the TCP and at the pad tips, 6 mm
    outside each pad face. For hands-off checks in an ``on_step`` hook; score() does not see the robot."""
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


def set_targets(environment, name: str, targets: dict[str, float]) -> None:
    """Set drive targets (rad) of some hinges of an articulated part (env 0)."""
    import torch

    asset = environment.unwrapped.scene[name]
    names = list(asset.joint_names)
    ids = [names.index(f"{k}_joint") for k in targets]
    t = torch.tensor([list(targets.values())], dtype=torch.float32, device=asset.device)
    asset.set_joint_position_target(t, joint_ids=ids)


# -- hooking yam_arena -------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


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
    if getattr(scene, "_lockkit_patched", False):
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

    scene.task_object = task_object
    scene.object_states = object_states
    scene._lockkit_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_lockkit_patched", False):
        return
    base_task_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = base_reset(self, *a, **k)
        _ENV["env"] = self._env
        for spec in _SPECS.values():  # drive targets back to each hinge's start angle (springs, dampers)
            if spec.hinges:
                set_targets(self._env, spec.name, {h.name: h.start for h in spec.hinges})
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._lockkit_patched = True


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
