"""dynkit: scripted moving bodies, an episode clock and robot geometry for a yam_arena task file.

Built on scenekit (USD bodies, articulations and posed fixed shapes), which an accepted task uses, with additions for
scenes that change while the robot works: kinematic bodies that follow a schedule, lamps that switch, and the robot's
links as capsules for distance and speed checks.

From scenekit:

yam_arena's task files spawn single primitives (yaw only) and library assets. This module adds three object kinds,
for the hinged platform, the lock pin and the mannequin hand:

- :class:`UsdBody`: a body loaded from a USD file next to the task file. With ``articulation=True`` it is an
  articulation (here, the stand and the platform on its hinge, fixed base) whose joints start at ``joint_pos``.
- :class:`Shape`: a fixed box or capsule in any orientation.

They subclass ``yam_arena.task_file.Box`` only so the task-file checks accept them. Importing this module hooks
``yam_arena.scene`` and the embodiment's step and reset, without changing yam_arena:

- ``success()`` and ``score()`` receive a :class:`BodyState` for these objects: the body origin's pose (not a
  bounding-box centre) and, for articulations, the joint positions;
- callbacks registered with :func:`on_step` run after every control step, before the task file is scored, and
  ones registered with :func:`on_reset` run after every reset; both get the Isaac Lab environment.

Pure Python at import time; everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import math
import importlib.util
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["BodyState", "Mover", "Shape", "UsdBody", "drive", "now", "show_when", "arm_joints", "env", "on_reset", "on_step", "robot_links"]

HERE = Path(__file__).resolve().parent


@dataclass(frozen=True)
class UsdBody(Box):
    """A body from ``usd`` (relative to this folder). ``pos``/``rot_xyzw`` place the file's origin. ``size`` is only
    nominal. With ``mass=None`` a rigid body is kinematic; an articulation keeps the fixed base its file declares."""

    usd: str = ""
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    joint_pos: tuple[tuple[str, float], ...] = ()
    joint_damping: float = 0.03
    friction: tuple[float, float] = (0.8, 0.7)  # static, dynamic
    friction_min: bool = False  # combine friction with "min", so this surface's friction rules its contacts
    solver_iters: int = 16
    torsional_patch: float | None = None  # m, for bodies held in a pinch (lets the pads resist twisting)
    # per-joint position drives for an articulation: ((joint name regex, stiffness, damping, effort limit), ...);
    # joints not listed get the free drive (stiffness 0, joint_damping)
    joint_drives: tuple = ()
    contact_offset: float | None = None  # m, for small or thin bodies (default PhysX offset is a few mm)

    def _check(self) -> None:
        if not (HERE / self.usd).is_file():
            raise FileNotFoundError(f"{self.name}: {HERE / self.usd} not found")


@dataclass(frozen=True)
class Shape(Box):
    """A fixed box (``size``) or capsule (``radius``, ``height`` of the cylindrical part, along ``axis``), posed
    with ``rot_xyzw``. It collides unless ``collide`` is False."""

    kind: str = "box"
    radius: float = 0.01
    height: float = 0.02
    axis: str = "Z"
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    collide: bool = True

    def _check(self) -> None:
        if self.kind not in ("box", "capsule"):
            raise ValueError(f"{self.name}: kind must be 'box' or 'capsule'")
        if self.mass is not None:
            raise ValueError(f"{self.name}: a Shape is fixed (no mass)")


@dataclass(frozen=True)
class BodyState(ObjectState):
    """The body origin's pose, velocities, bounds and joint positions (rad or m)."""

    joints: dict = field(default_factory=dict)

    def rotation(self) -> np.ndarray:
        x, y, z, w = self.quat_xyzw
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


# -- Arena objects -------------------------------------------------------------------------------------------------

_SPECS: dict[str, UsdBody | Shape] = {}


def _usd_object(spec: UsdBody):
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    addon = {}
    if spec.friction is not None:      # None: the USD file binds its own physics materials, part by part
        addon["physics_material"] = sim_utils.RigidBodyMaterialCfg(
            static_friction=spec.friction[0], dynamic_friction=spec.friction[1],
            friction_combine_mode="min" if spec.friction_min else "average")
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=spec.solver_iters, enabled_self_collisions=False)
    else:
        addon["rigid_props"] = sim_utils.RigidBodyPropertiesCfg(solver_position_iteration_count=spec.solver_iters,
                                                                kinematic_enabled=spec.mass is None)
        col = {}
        if spec.torsional_patch:
            col.update(torsional_patch_radius=spec.torsional_patch,
                       min_torsional_patch_radius=0.8 * spec.torsional_patch)
        if spec.contact_offset is not None:
            col.update(contact_offset=spec.contact_offset, rest_offset=0.0)
        if col:
            addon["collision_props"] = sim_utils.CollisionPropertiesCfg(**col)
        if spec.mass is not None:
            addon["mass_props"] = sim_utils.MassPropertiesCfg(mass=spec.mass)

    class UsdObject(Object):
        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            joints = dict(spec.joint_pos)
            cfg.init_state.joint_pos = joints
            if spec.joint_drives:
                cfg.actuators = {f"drive{i}": ImplicitActuatorCfg(joint_names_expr=[expr], stiffness=k, damping=d,
                                                                  joint_effort_limit=e, friction=0.0)
                                 for i, (expr, k, d, e) in enumerate(spec.joint_drives)}
            else:
                cfg.actuators = {"joints": ImplicitActuatorCfg(joint_names_expr=list(joints) or [".*"],
                                                               stiffness=0.0, damping=spec.joint_damping,
                                                               friction=0.0)}
            return cfg

    obj = UsdObject(name=spec.name, usd_path=str(HERE / spec.usd),
                    object_type=ObjectType.ARTICULATION if spec.articulation else ObjectType.RIGID,
                    spawn_cfg_addon=addon)
    obj.set_initial_pose(Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=tuple(spec.rot_xyzw)))
    return obj


def _shape_object(spec: Shape):
    import isaaclab.sim as sim_utils
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    from yam_arena.station import linear_rgb

    body = {"visual_material": sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.6)}
    if spec.collide:
        body["collision_props"] = sim_utils.CollisionPropertiesCfg()
    if spec.kind == "box":
        cfg = sim_utils.CuboidCfg(size=tuple(spec.size), **body)
    else:
        cfg = sim_utils.CapsuleCfg(radius=spec.radius, height=spec.height, axis=spec.axis, **body)
    obj = Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg)
    obj.set_initial_pose(Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=tuple(spec.rot_xyzw)))
    return obj


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _state(environment, spec: UsdBody | Shape) -> BodyState:
    if isinstance(spec, Shape):
        half = np.asarray(spec.size) / 2
        pos = np.asarray(spec.pos, float)
        return BodyState(pos=pos, quat_xyzw=np.asarray(spec.rot_xyzw, float), lin_vel=np.zeros(3),
                         ang_vel=np.zeros(3), low=pos - half, high=pos + half)
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0]
    joints = {}
    if getattr(spec, "articulation", False):
        q = _np(asset.data.joint_pos)[0]
        joints = {n: float(q[i]) for i, n in enumerate(asset.joint_names)}
    pos = pose[:3] - origin
    return BodyState(pos=pos, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=pos, high=pos,
                     joints=joints)


def robot_links(environment) -> dict[str, dict[str, np.ndarray]]:
    """Each arm's link origins by link name (station frame, env 0), for proximity checks."""
    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    out = {}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        pos = _np(robot.data.body_pos_w)[0] - origin
        out[side] = {name: pos[i] for i, name in enumerate(robot.body_names)}
    return out


def arm_joints(environment, side: str, names) -> np.ndarray:
    """One arm's measured joint positions, in the order of ``names``."""
    robot = environment.unwrapped.scene[f"{side}_robot"]
    ids = robot.find_joints(list(names), preserve_order=True)[0]
    return _np(robot.data.joint_pos)[0, ids]


# -- hooking yam_arena -----------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    """Run ``fn(env)`` after every control step, before the task file is scored."""
    _STEP.append(fn)
    return fn


def on_reset(fn):
    """Run ``fn(env)`` after every reset, once the scene has settled."""
    _RESET.append(fn)
    return fn


def env():
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_scenekit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def task_object_hook(spec):
        if isinstance(spec, UsdBody | Shape | Mover):
            _SPECS[spec.name] = spec
            if isinstance(spec, Mover):
                return _mover_object(spec)
            return _usd_object(spec) if isinstance(spec, UsdBody) else _shape_object(spec)
        return task_object(spec)

    def object_states_hook(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _state(environment, _SPECS[n]) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = task_object_hook, object_states_hook
    scene._scenekit = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_scenekit", False):
        return
    task_step, reset = cls._task_step, cls.reset

    def task_step_hook(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return task_step(self, *a, **k)

    def reset_hook(self, *a, **k):
        obs = reset(self, *a, **k)
        _ENV["env"] = self._env
        for fn in _RESET:
            fn(self._env)
        return obs

    def prepare_trial(self, scene):
        from inspect_robots.embodiment import Readiness

        _TRIAL[0] += 1
        print(f"[dynkit] trial {_TRIAL[0]}: lot {lot()}", file=sys.stderr, flush=True)
        return Readiness(ready=True)

    cls._task_step, cls.reset, cls.prepare_trial = task_step_hook, reset_hook, prepare_trial
    cls._scenekit = True


# -- lots: trial k of a run (Inspect Robots calls prepare_trial before every trial) uses lot k mod N ------------------

_TRIAL = [-1]
_LOTS: list = ["A"]


def declare_lots(*names: str) -> None:
    """The task's lots in order; with ``--epochs N`` trial k uses lot k mod N (lot A first). ``TASK_LOT=B`` pins
    every trial to one lot (to record that lot's demo)."""
    _LOTS[:] = list(names)


def lot() -> str:
    import os

    pinned = os.environ.get("TASK_LOT")
    if pinned:
        if pinned not in _LOTS:
            raise ValueError(f"TASK_LOT={pinned!r}: the lots are {_LOTS}")
        return pinned
    return _LOTS[max(_TRIAL[0], 0) % len(_LOTS)]


class _Finder(importlib.abc.MetaPathFinder):
    """Patches yam_arena's scene and embodiment modules as they are imported (they need Isaac Sim, which starts after
    the task file is loaded)."""

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
        exec_module = loader.exec_module

        def exec_and_patch(module):
            exec_module(module)
            patch(module)

        loader.exec_module = exec_and_patch
        return spec


def install() -> None:
    """Hook yam_arena now for modules already imported, and on import for the rest. Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()


# -- dynamic scenes: an episode clock, scripted kinematic bodies and colours that change ------------------------------
#
# A Mover is a kinematic rigid body (box, cylinder, sphere or capsule) whose pose a task sets after every control step
# with drive(name, fn): fn(t) returns (pos, quat_xyzw) in the station frame at episode time t (s); it shows from the
# next rendered frame. show_when() switches a body on and off (lamps). RTX does not pick up USD material edits made
# while the episode runs (tried 2026-11-12), so a lamp that changes colour is two bulbs.

_CLOCK = {"steps": 0, "dt": 0.05}
_DRIVES: dict = {}


@dataclass(frozen=True)
class Mover(Box):
    """A kinematic body: ``kind`` box (``size``), cylinder or capsule (``radius``, ``height`` along z) or sphere
    (``radius``). ``pos``/``rot_xyzw`` give its pose at reset; a drive moves it. It collides unless ``collide`` is
    False (then it is only seen)."""

    kind: str = "box"
    radius: float = 0.01
    height: float = 0.02
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    collide: bool = True
    emissive: float = 0.0  # >0 makes the body glow (lamps), in nits-ish units of the RTX preview surface

    def _check(self) -> None:
        if self.kind not in ("box", "cylinder", "capsule", "sphere"):
            raise ValueError(f"{self.name}: kind must be box, cylinder, capsule or sphere")
        if self.mass is not None:
            raise ValueError(f"{self.name}: a Mover is kinematic (no mass)")


def now() -> float:
    """Episode time (s): control steps since the last reset times the control period."""
    return _CLOCK["steps"] * _CLOCK["dt"]


def drive(name: str, fn) -> None:
    """Pose the Mover ``name`` at ``fn(t) -> (pos, quat_xyzw)`` after every control step."""
    _DRIVES[name] = fn


def show_when(spec: "Mover", fn) -> None:
    """Drive ``spec`` to stay at its own pose while ``fn(t)`` is true, and 1 m under the table otherwise (an indicator
    lamp is two bulbs, one per colour, in the same place)."""
    hidden = (spec.pos[0], spec.pos[1], spec.pos[2] - 1.0)
    drive(spec.name, lambda t: (spec.pos if fn(t) else hidden, spec.rot_xyzw))


def _mover_object(spec: Mover):
    import isaaclab.sim as sim_utils
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    from yam_arena.station import linear_rgb

    look = dict(diffuse_color=linear_rgb(spec.color), roughness=0.5)
    if spec.emissive > 0:
        look["emissive_color"] = tuple(c * spec.emissive for c in linear_rgb(spec.color))
    body = {"visual_material": sim_utils.PreviewSurfaceCfg(**look),
            "rigid_props": sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
            "mass_props": sim_utils.MassPropertiesCfg(mass=1.0),
            "collision_props": sim_utils.CollisionPropertiesCfg(collision_enabled=spec.collide)}
    if spec.kind == "box":
        cfg = sim_utils.CuboidCfg(size=tuple(spec.size), **body)
    elif spec.kind == "cylinder":
        cfg = sim_utils.CylinderCfg(radius=spec.radius, height=spec.height, axis="Z", **body)
    elif spec.kind == "capsule":
        cfg = sim_utils.CapsuleCfg(radius=spec.radius, height=spec.height, axis="Z", **body)
    else:
        cfg = sim_utils.SphereCfg(radius=spec.radius, **body)
    obj = Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg)
    pos, rot = tuple(float(v) for v in spec.pos), tuple(float(v) for v in spec.rot_xyzw)
    if spec.name in _DRIVES:  # spawn (and reset) where the schedule puts it at t = 0, so the first frame shows it there
        p0, q0 = _DRIVES[spec.name](0.0)
        pos, rot = tuple(float(v) for v in p0), tuple(float(v) for v in q0)
    obj.set_initial_pose(Pose(position_xyz=pos, rotation_xyzw=rot))
    return obj


def _apply_drives(environment) -> None:
    import torch

    base = environment.unwrapped
    t = now()
    for name, fn in _DRIVES.items():
        pos, quat = fn(t)
        asset = base.scene[name]
        origin = base.scene.env_origins[0]
        pose = torch.tensor([*pos, *quat], dtype=torch.float32, device=origin.device).unsqueeze(0)
        pose[:, :3] += origin
        asset.write_root_pose_to_sim_index(root_pose=pose)
        asset.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=origin.device))


@on_reset
def _dyn_reset(environment) -> None:
    _CLOCK["steps"] = 0
    _CLOCK["dt"] = float(environment.unwrapped.step_dt)
    _apply_drives(environment)


def _dyn_step(environment) -> None:
    _CLOCK["steps"] += 1
    _apply_drives(environment)


_STEP.insert(0, _dyn_step)  # tick the clock before the task's own monitors run


# -- the robot as capsules ---------------------------------------------------------------------------------------------

CHAIN = ["link1", "link2", "link3", "link4", "link5", "gripper"]
ARM_R = 0.03      # m, capsule radius along the arm's links
PAD_R = 0.012     # m, capsule radius from the gripper to the fingertips
PAD_TIP = 0.013   # m, fingertips past the tool point (yam_arena.constants.PAD_TIP_PAST_TCP)
_PREV_TCP: dict = {}


def arms(environment, track: bool = True) -> dict:
    """Per arm (station frame): ``caps`` [(a, b, r)] along the links and out to the fingertips, ``tcp`` (tool point),
    ``tip`` (fingertips), ``rot`` (tool frame), ``opening`` (m between the fingers) and ``speed`` (tool point, m/s,
    over the last control step). The task's monitor calls it once per step (``track``); others pass track=False so
    the speed stays measured over whole control steps."""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    links = robot_links(environment)
    out = {}
    for side in ("left", "right"):
        q = arm_joints(environment, side, ARM_JOINTS)
        tcp, rot = ik.tcp_pose(q, station.BASES[side])
        tip = tcp + rot[:, 2] * PAD_TIP
        chain = [links[side][n] for n in CHAIN if n in links[side]]
        caps = [(a, b, ARM_R) for a, b in zip(chain[:-1], chain[1:])]
        caps.append((chain[-1], tip, PAD_R))
        opening = float(np.abs(arm_joints(environment, side, FINGER_JOINTS)).sum())
        prev = _PREV_TCP.get(side)
        speed = 0.0 if prev is None else float(np.linalg.norm(tcp - prev) / _CLOCK["dt"])
        if track:
            _PREV_TCP[side] = tcp.copy()
        out[side] = {"caps": caps, "tcp": tcp, "tip": tip, "rot": rot, "opening": opening, "speed": speed}
    return out


@on_reset
def _arms_reset(environment) -> None:
    _PREV_TCP.clear()


def seg_point_dist(a, b, p) -> float:
    """Distance from point p to segment ab."""
    a, b, p = (np.asarray(v, float) for v in (a, b, p))
    ab = b - a
    t = 0.0 if not ab.any() else float(np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0.0, 1.0))
    return float(np.linalg.norm(a + t * ab - p))


def seg_seg_dist(a, b, c, d, n: int = 16) -> float:
    """Distance between segments ab and cd (sampled along ab, exact along cd)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    return min(seg_point_dist(c, d, a + (b - a) * k / n) for k in range(n + 1))


def seg_box_dist(a, b, lo, hi, n: int = 16) -> float:
    """Distance from segment ab to the axis-aligned box [lo, hi] (sampled along ab)."""
    a, b, lo, hi = (np.asarray(v, float) for v in (a, b, lo, hi))
    best = np.inf
    for k in range(n + 1):
        p = a + (b - a) * k / n
        best = min(best, float(np.linalg.norm(np.maximum(np.maximum(lo - p, 0.0), p - hi))))
    return best


def holding(arm, centre, radius: float = 0.03, max_open: float = 0.06) -> bool:
    """A gripper holds the object at ``centre`` when its tool point is within ``radius`` and its fingers are closed
    to under ``max_open`` apart."""
    return bool(np.linalg.norm(arm["tcp"] - np.asarray(centre)) < radius and arm["opening"] < max_open)


def smoothstep(u: float) -> float:
    u = min(max(u, 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


# -- a mannequin hand and forearm, moved along a scripted path ------------------------------------------------------------

SKIN = (0.86, 0.68, 0.55)


def _quat_z_to(direction) -> tuple[float, float, float, float]:
    """xyzw quaternion turning the z axis onto the unit vector ``direction``."""
    d = np.asarray(direction, float)
    d = d / np.linalg.norm(d)
    z = np.array([0.0, 0.0, 1.0])
    c = float(np.dot(z, d))
    if c < -0.999999:
        return (1.0, 0.0, 0.0, 0.0)
    axis = np.cross(z, d)
    s = math.sqrt((1.0 + c) * 2.0)
    return (axis[0] / s, axis[1] / s, axis[2] / s, s / 2.0)


class Hand:
    """A right or left mannequin hand, palm down, and its forearm, as capsules. ``path(t)`` returns the wrist point
    (station frame) and the heading (unit vector, wrist to fingertips, horizontal). ``objects`` go in OBJECTS.

    Parts (hand frame: u along the heading, v to the hand's left, w up), as (name, start, end, radius):
    - four fingers 7 cm long, 9 mm radius, from 8.5 cm past the wrist;
    - the palm as three 12.5 mm capsules 6 cm long side by side, from 1.5 cm past the wrist;
    - the thumb, 5 cm, angled 40 degrees out from the palm's side;
    - the forearm, 24 cm long, 3 cm radius, behind the wrist, rising 4 cm toward the elbow."""

    def __init__(self, prefix: str, path, sleeve=(0.25, 0.35, 0.60), skin=SKIN, side: str = "right", visible=None,
                 parts=None):
        """``visible(t)`` (optional) hides the hand while false (two Hands, open and closed, make a hand that grips);
        ``parts`` replaces the default open-hand parts."""
        self.prefix, self.path, self.visible = prefix, path, visible
        thumb_v = 1.0 if side == "right" else -1.0  # palm down: a right hand's thumb is on its left
        a40 = math.radians(40)
        self.parts = [
            *[(f"{prefix}_finger{i}", (0.085, dv, 0.0), (0.155, dv, 0.0), 0.009)
              for i, dv in enumerate((-0.03, -0.01, 0.01, 0.03))],
            *[(f"{prefix}_palm{i}", (0.015, dv, 0.0), (0.075, dv, 0.0), 0.0125)
              for i, dv in enumerate((-0.025, 0.0, 0.025))],
            (f"{prefix}_thumb", (0.03, thumb_v * 0.04, 0.0),
             (0.03 + 0.05 * math.cos(a40), thumb_v * (0.04 + 0.05 * math.sin(a40)), 0.0), 0.01),
            (f"{prefix}_forearm", (-0.02, 0.0, 0.005), (-0.26, 0.0, 0.045), 0.03),
        ]
        if parts is not None:
            self.parts = [(f"{prefix}_{n}", p0, p1, r) for n, p0, p1, r in parts]
        self.objects = []
        for name, p0, p1, r in self.parts:
            color = sleeve if name.endswith("forearm") else skin
            length = float(np.linalg.norm(np.subtract(p1, p0)))
            self.objects.append(Mover(name, pos=(0.0, 0.0, -1.0), kind="capsule", radius=r, height=length,
                                      color=color))
            drive(name, lambda t, p0=p0, p1=p1: self._pose(t, p0, p1))

    def _frame(self, t):
        wrist, heading = self.path(t)
        u = np.zeros(3)
        u[:len(heading)] = np.asarray(heading, float)[:3]  # a 2-D heading (x, y) is horizontal
        u = u / np.linalg.norm(u)
        v = np.cross([0.0, 0.0, 1.0], u)
        v = v / np.linalg.norm(v)
        w = np.cross(u, v)
        return np.asarray(wrist, float), u, v, w

    def _world(self, t, p):
        o, u, v, w = self._frame(t)
        return o + p[0] * u + p[1] * v + p[2] * w

    def _pose(self, t, p0, p1):
        if self.visible is not None and not self.visible(t):
            return (0.0, 0.0, -1.0), (0.0, 0.0, 0.0, 1.0)
        a, b = self._world(t, p0), self._world(t, p1)
        return tuple((a + b) / 2), _quat_z_to(b - a)

    def capsules(self, t):
        """[(a, b, r)] of every part at time t (station frame); none while the hand is hidden."""
        if self.visible is not None and not self.visible(t):
            return []
        return [(self._world(t, p0), self._world(t, p1), r) for _, p0, p1, r in self.parts]

    def distance(self, t, a, b, r) -> float:
        """Surface distance from the robot capsule (a, b, r) to the hand at time t (negative when they overlap)."""
        return min((seg_seg_dist(a, b, c, d) - r - rc for c, d, rc in self.capsules(t)), default=9.0)

    def robot_distance(self, t, arm_list) -> float:
        """Surface distance from every robot capsule (arms() values) to the hand."""
        return min(self.distance(t, a, b, r) for arm in arm_list for a, b, r in arm["caps"])


def path_through(waypoints):
    """A wrist path from [(t, (x, y, z), heading)]: holds each pose and blends (smoothstep) to the next. A heading is
    (hx, hy), horizontal, or (hx, hy, hz) for a hand tilted down or up."""
    pts = [(float(t), np.asarray(p, float), np.asarray((*h, 0.0) if len(h) == 2 else h, float))
           for t, p, h in waypoints]

    def path(t):
        if t <= pts[0][0]:
            return pts[0][1], pts[0][2]
        for (t0, p0, h0), (t1, p1, h1) in zip(pts, pts[1:]):
            if t <= t1:
                s = smoothstep((t - t0) / (t1 - t0)) if t1 > t0 else 1.0
                h = h0 + s * (h1 - h0)
                return p0 + s * (p1 - p0), h / (np.linalg.norm(h) or 1.0)
        return pts[-1][1], pts[-1][2]

    return path


# -- push buttons -------------------------------------------------------------------------------------------------------


def press_depth(arm_list, xy, top: float, radius: float, travel: float) -> float:
    """How far (m, up to ``travel``) the deepest fingertip pushes a button whose cap (``radius``) has its top at
    ``top`` over ``xy``. A button's cap is drawn as a non-colliding Mover that follows this depth down to a fixed stop,
    so the fingertip meets the stop at the end of the travel."""
    depth = 0.0
    for arm in arm_list:
        tip = arm["tip"]
        if math.hypot(tip[0] - xy[0], tip[1] - xy[1]) <= radius + 0.008:
            depth = max(depth, min(travel, top - (float(tip[2]) - 0.004)))
    return max(0.0, depth)
