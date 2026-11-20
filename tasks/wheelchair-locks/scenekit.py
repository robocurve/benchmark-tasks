"""scenekit: USD bodies, articulations and posed fixed shapes for a yam_arena task file.

yam_arena's task files spawn single primitives (yaw only) and library assets. This module adds two object kinds:

- :class:`UsdBody`: a body loaded from a USD file next to the task file, rigid (with a mass), kinematic (without
  one), or with ``articulation=True`` an articulation (for example a lid on a hinge, fixed base) whose joints start at
  ``joint_pos``.
- :class:`Shape`: a fixed box or capsule in any orientation.

Keep this file next to your task file, and import it from there (see the template's task.py).

They subclass ``yam_arena.task_file.Box`` only so that the task-file checks pass them. Importing this module hooks
``yam_arena.scene`` and the embodiment's step and reset, without changing yam_arena:

- ``success()`` and ``score()`` receive a :class:`BodyState` for these objects: the body origin's pose (not a
  bounding-box centre) and, for articulations, the joint positions;
- callbacks registered with :func:`on_step` run after every control step, before the task file is scored, and
  ones registered with :func:`on_reset` run after every reset; both get the Isaac Lab environment.

Pure Python at import time; everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["BodyState", "Shape", "UsdBody", "after_spawn", "arm_joints", "before_reset", "env", "on_reset", "on_step",
           "physics", "robot_links", "set_joint_targets", "write_joint_positions"]

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
    drives: tuple[tuple, ...] = ()  # (joint name or regex, stiffness, damping[, effort limit]) overriding the default
    friction: tuple[float, float] | None = (0.8, 0.7)  # static, dynamic; None keeps the file's own materials
    friction_min: bool = False  # combine friction with "min", so this surface's friction rules its contacts
    solver_iters: int = 16
    torsional_patch: float | None = None  # m, for bodies held in a pinch (lets the pads resist twisting)

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

_ITEMS: dict[str, UsdBody | Shape] = {}


def _usd_object(item: UsdBody):
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    addon = {}
    if item.friction is not None:
        addon["physics_material"] = sim_utils.RigidBodyMaterialCfg(
            static_friction=item.friction[0], dynamic_friction=item.friction[1],
            friction_combine_mode="min" if item.friction_min else "average")
    if item.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=item.solver_iters, enabled_self_collisions=False)
    else:
        addon["rigid_props"] = sim_utils.RigidBodyPropertiesCfg(solver_position_iteration_count=item.solver_iters,
                                                                kinematic_enabled=item.mass is None)
        if item.torsional_patch:
            addon["collision_props"] = sim_utils.CollisionPropertiesCfg(
                torsional_patch_radius=item.torsional_patch, min_torsional_patch_radius=0.8 * item.torsional_patch)
        if item.mass is not None:
            addon["mass_props"] = sim_utils.MassPropertiesCfg(mass=item.mass)

    class UsdObject(Object):
        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            joints = dict(item.joint_pos)
            cfg.init_state.joint_pos = joints
            driven = {d[0] for d in item.drives}
            free = [name for name in joints if not any(re.fullmatch(d, name) for d in driven)]
            cfg.actuators = {}
            for name, k, d, *lim in item.drives:     # (name, stiffness, damping[, effort limit])
                extra = {"joint_effort_limit": lim[0]} if lim else {}
                cfg.actuators[name] = ImplicitActuatorCfg(joint_names_expr=[name], stiffness=k, damping=d, friction=0.0,
                                                          **extra)
            if free or not driven:
                cfg.actuators["joints"] = ImplicitActuatorCfg(joint_names_expr=free or [".*"], stiffness=0.0,
                                                              damping=item.joint_damping, friction=0.0)
            return cfg

    obj = UsdObject(name=item.name, usd_path=str(HERE / item.usd),
                    object_type=ObjectType.ARTICULATION if item.articulation else ObjectType.RIGID,
                    spawn_cfg_addon=addon)
    obj.set_initial_pose(Pose(position_xyz=tuple(float(v) for v in item.pos), rotation_xyzw=tuple(item.rot_xyzw)))
    if item.name in _SPAWN_HOOKS:
        spawn = obj.object_cfg.spawn
        func, hook = spawn.func, _SPAWN_HOOKS[item.name]

        def spawn_and_hook(prim_path, cfg, *a, **k):
            prim = func(prim_path, cfg, *a, **k)
            hook(prim)
            return prim

        spawn.func = spawn_and_hook
    return obj


def _shape_object(item: Shape):
    import isaaclab.sim as sim_utils
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    from yam_arena.station import linear_rgb

    body = {"visual_material": sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(item.color), roughness=0.6)}
    if item.collide:
        body["collision_props"] = sim_utils.CollisionPropertiesCfg()
    if item.kind == "box":
        cfg = sim_utils.CuboidCfg(size=tuple(item.size), **body)
    else:
        cfg = sim_utils.CapsuleCfg(radius=item.radius, height=item.height, axis=item.axis, **body)
    obj = Object(name=item.name, object_type=ObjectType.BASE, spawner_cfg=cfg)
    obj.set_initial_pose(Pose(position_xyz=tuple(float(v) for v in item.pos), rotation_xyzw=tuple(item.rot_xyzw)))
    return obj


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _state(environment, item: UsdBody | Shape) -> BodyState:
    if isinstance(item, Shape):
        half = np.asarray(item.size) / 2
        pos = np.asarray(item.pos, float)
        return BodyState(pos=pos, quat_xyzw=np.asarray(item.rot_xyzw, float), lin_vel=np.zeros(3),
                         ang_vel=np.zeros(3), low=pos - half, high=pos + half)
    base = environment.unwrapped
    asset = base.scene[item.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0]
    joints = {}
    if item.articulation:
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


def robot_link_poses(environment) -> dict[str, dict[str, np.ndarray]]:
    """Each arm's link poses as 4x4 matrices by link name (station frame, env 0), fingers included, for placing the
    URDF's collision hulls (``yam_arena.analysis.world_hulls``)."""
    from yam_arena.runtime import link_poses

    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    out = {}
    for side in ("left", "right"):
        poses = link_poses(base.scene[f"{side}_robot"])
        for p in poses.values():
            p[:3, 3] -= origin
        out[side] = poses
    return out


def set_joint_targets(environment, name: str, targets: dict) -> None:
    """Position targets for an articulation's driven joints (see ``UsdBody.drives``), by joint name."""
    import torch

    asset = environment.unwrapped.scene[name]
    ids = asset.find_joints(list(targets), preserve_order=True)[0]
    value = torch.tensor([list(targets.values())], dtype=torch.float32, device=asset.device)
    asset.actuators.target_command.set_position_index(value=value, joint_ids=ids)


def write_joint_positions(environment, name: str, positions: dict) -> None:
    """Move an articulation's joints to these positions (by joint name), at rest, for example to enforce a stop."""
    import torch

    asset = environment.unwrapped.scene[name]
    ids = asset.find_joints(list(positions), preserve_order=True)[0]
    value = torch.tensor([list(positions.values())], dtype=torch.float32, device=asset.device)
    asset.write_joint_position_to_sim_index(position=value, joint_ids=ids)
    asset.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(value), joint_ids=ids)


def arm_joints(environment, side: str, names) -> np.ndarray:
    """One arm's measured joint positions, in the order of ``names``."""
    robot = environment.unwrapped.scene[f"{side}_robot"]
    ids = robot.find_joints(list(names), preserve_order=True)[0]
    return _np(robot.data.joint_pos)[0, ids]


# -- hooking yam_arena -----------------------------------------------------------------------------------------------

_SPAWN_HOOKS: dict = {}


def after_spawn(name: str):
    """Run ``fn(prim)`` right after the UsdBody ``name`` is spawned, before the simulation starts (for example to add a
    joint between two bodies)."""
    def register(fn):
        _SPAWN_HOOKS[name] = fn
        return fn
    return register


_STEP: list = []
_RESET: list = []
_PRE_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    """Run ``fn(env)`` after every control step, before the task file is scored."""
    _STEP.append(fn)
    return fn


def on_reset(fn):
    """Run ``fn(env)`` after every reset, once the scene has settled."""
    _RESET.append(fn)
    return fn


def before_reset(fn):
    """Run ``fn(env)`` at the start of every reset, before the scene is reset and settles (on the first, as soon as
    the scene is built). Use it to put a UsdBody articulation's joints back: the reset moves it only by its root, and
    its joints and drive targets keep the last trial's values (on the first, the drives start out targeting 0), which
    drives would chase while the reset scene settles."""
    _PRE_RESET.append(fn)
    return fn


def env():
    return _ENV["env"]


_PHYSICS: dict = {}


def physics(hz: float) -> None:
    """Run PhysX at ``hz`` (yam_arena's default is 240 Hz). Call from the task file at import, before the scene is
    built (yam_arena's scene reads the physics step when it builds the environment config). GPU PhysX has no CCD, so
    a higher rate is how a thin part is kept from tunnelling."""
    import yam_arena.constants as C

    _PHYSICS["dt"] = 1.0 / hz
    C.PHYSICS_DT = 1.0 / hz
    if "yam_arena.scene" in sys.modules:
        _apply_physics(sys.modules["yam_arena.scene"])


def _apply_physics(scene) -> None:
    if "dt" in _PHYSICS:
        scene.PHYSICS_DT = _PHYSICS["dt"]


_ROOM: dict = {"floor_bay": False}
FLOOR_FRICTION = (0.55, 0.55)        # vinyl sheet flooring: static, dynamic
PEDESTAL, PEDESTAL_FOOT = (0.10, 0.10), (0.24, 0.24, 0.015)
FLOOR_BAY_DOCS = """Simulated rig (Isaac Lab-Arena 0.3.1, PhysX): Dream's calibrated YAM station without its table. Both arms
are mounted 0.61 m apart at the station's height (0.76 m above the floor), each on a steel pedestal, left arm at +y,
both facing +x over a level vinyl floor; each arm's base frame is its own mount. Cameras: top_cam looks down over the
arms from a mast behind them; left_cam and right_cam are on the wrists, looking out along the fingers."""


def floor_bay() -> None:
    """Use the station's arms and cameras over a floor bay instead of its table: the same arms, mounts, gains, cameras and
    control, the table taken away, each arm on a steel pedestal from the floor to its mount, the top camera on a floor
    mast, a vinyl floor. Call from the task file at import (SCENE = "station")."""
    _ROOM["floor_bay"] = True
    if "yam_arena.inspect_embodiment" in sys.modules:
        _patch_docs(sys.modules["yam_arena.inspect_embodiment"])


def _patch_docs(module) -> None:
    if _ROOM["floor_bay"]:
        module.ROOM_DOCS["station"] = FLOOR_BAY_DOCS


def _floor_bay_room(scene, top_cam) -> list:
    """Floor, lights, the arms' pedestals and the top camera's floor mast (no table)."""
    import isaaclab.sim as sim_utils
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_library import DomeLight
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg

    from yam_arena import station

    floor_z = station.FLOOR_Z
    vinyl = sim_utils.RigidBodyMaterialCfg(static_friction=FLOOR_FRICTION[0], dynamic_friction=FLOOR_FRICTION[1])
    floor = Object(name="floor", object_type=ObjectType.BASE,
                   spawner_cfg=sim_utils.CuboidCfg(
                       size=(200.0, 200.0, 0.02), collision_props=PhysxCollisionPropertiesCfg(), physics_material=vinyl,
                       visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=scene.STATION_FLOOR_COLOR,
                                                                   roughness=0.6)),
                   initial_pose=Pose(position_xyz=(0.0, 0.0, floor_z - 0.01)))
    steel = (0.30, 0.31, 0.33)
    room = [floor,
            DomeLight(spawner_cfg=sim_utils.DomeLightCfg(intensity=scene.STATION_DOME_INTENSITY,
                                                         color=station.linear_rgb(station.SKY))),
            Object(name="lamp", object_type=ObjectType.BASE,
                   spawner_cfg=sim_utils.SphereLightCfg(intensity=scene.STATION_LAMP_INTENSITY,
                                                        radius=scene.STATION_LAMP_RADIUS),
                   initial_pose=Pose(position_xyz=station.LAMP_POS))]
    for side, (x, y, z) in station.BASES.items():
        top = z - 0.004
        room += [scene._box(f"{side}_pedestal_foot", PEDESTAL_FOOT, (x, y, floor_z + PEDESTAL_FOOT[2] / 2), steel),
                 scene._box(f"{side}_pedestal", (*PEDESTAL, top - floor_z - PEDESTAL_FOOT[2]),
                            (x, y, (top + floor_z + PEDESTAL_FOOT[2]) / 2), steel)]
    stand = station.camera_stand(top_cam)
    cx, cy, cz = top_cam.pos
    mast_top = cz + 0.02
    room += [scene._box("camera_floor_foot", (0.24, 0.24, 0.015), (cx - 0.07, cy, floor_z + 0.0075), steel),
             scene._box("camera_floor_mast", (0.03, 0.03, mast_top - floor_z - 0.015),
                        (cx - 0.07, cy, (mast_top + floor_z + 0.015) / 2), steel),
             scene._stand_part("stand_arm", *stand["stand_arm"]),
             scene._stand_part("camera_housing", *stand["camera_housing"])]
    return room


def _patch_scene(scene) -> None:
    _apply_physics(scene)
    if getattr(scene, "_scenekit", False):
        return
    task_object, object_states, task_room = scene.task_object, scene.object_states, scene.task_room

    def task_room_hook(name, top_cam):
        if _ROOM["floor_bay"] and name == "station":
            return _floor_bay_room(scene, top_cam)
        return task_room(name, top_cam)

    scene.task_room = task_room_hook

    def task_object_hook(item):
        if isinstance(item, UsdBody | Shape):
            _ITEMS[item.name] = item
            return _usd_object(item) if isinstance(item, UsdBody) else _shape_object(item)
        return task_object(item)

    def object_states_hook(environment, names):
        plain = [n for n in names if n not in _ITEMS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _state(environment, _ITEMS[n]) for n in names if n in _ITEMS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = task_object_hook, object_states_hook
    scene._scenekit = True


def _patch_embodiment(module) -> None:
    _patch_docs(module)
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_scenekit", False):
        return
    task_step, reset, ensure_sim = cls._task_step, cls.reset, cls._ensure_sim

    def task_step_hook(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return task_step(self, *a, **k)

    def ensure_sim_hook(self, *a, **k):
        first = getattr(self, "_env", None) is None
        out = ensure_sim(self, *a, **k)
        if first and getattr(self, "_env", None) is not None:      # the first reset: the scene was just built
            for fn in _PRE_RESET:
                fn(self._env)
        return out

    def reset_hook(self, *a, **k):
        if getattr(self, "_env", None) is not None:
            for fn in _PRE_RESET:
                fn(self._env)
        obs = reset(self, *a, **k)
        _ENV["env"] = self._env
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset, cls._ensure_sim = task_step_hook, reset_hook, ensure_sim_hook
    cls._scenekit = True


class _Finder(importlib.abc.MetaPathFinder):
    """Patches yam_arena's scene and embodiment modules as they are imported (they need Isaac Sim, which starts after
    the task file is loaded)."""

    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            item = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if item is None or item.loader is None:
            return None
        loader, patch = item.loader, self.targets[fullname]
        exec_module = loader.exec_module

        def exec_and_patch(module):
            exec_module(module)
            patch(module)

        loader.exec_module = exec_and_patch
        return item


def install() -> None:
    """Hook yam_arena now for modules already imported, and on import for the rest. Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
