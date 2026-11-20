"""scenekit: USD bodies, articulations and posed fixed shapes for a yam_arena task file.

yam_arena's task files spawn primitives and library assets. This vendored
adapter adds USD rigid bodies, articulations and fully posed fixed shapes.
The split-ring task uses a fixed-base USD articulation for the workholder and
spring spiral. The free key has no attachment to that articulation.

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
import importlib.util
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["BodyState", "Shape", "UsdBody", "arm_joints", "body_link_poses", "env", "on_reset", "on_step", "robot_links", "set_joint_gains", "set_joint_targets"]

HERE = Path(__file__).resolve().parent


@dataclass(frozen=True)
class UsdBody(Box):
    """A body from ``usd`` (relative to this folder). ``pos``/``rot_xyzw`` place the file's origin. ``size`` is only
    nominal. With ``mass=None`` a rigid body is kinematic; an articulation keeps the fixed base its file declares."""

    usd: str = ""
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    joint_pos: tuple[tuple[str, float], ...] = ()
    # None reads the authored USD value. Explicit angular gains use SI per-radian
    # units (unlike raw USD angular-drive attributes, which use per-degree gains).
    joint_stiffness: float | dict | None = None
    joint_damping: float | dict | None = None
    joint_friction: float | dict | None = None
    joint_effort_limit: float | dict | None = None
    # Absolute spring rest targets, radians/metres. Empty preserves USD targets.
    joint_targets: tuple[tuple[str, float], ...] = ()
    friction: tuple[float, float] = (0.8, 0.7)  # static, dynamic
    friction_min: bool = False  # combine friction with "min", so this surface's friction rules its contacts
    solver_iters: int = 16
    solver_velocity_iters: int = 8
    torsional_patch: float | None = None  # m, for bodies held in a pinch (lets the pads resist twisting)

    def _check(self) -> None:
        super()._check()
        if not (HERE / self.usd).is_file():
            raise FileNotFoundError(f"{self.name}: {HERE / self.usd} not found")
        quat = np.asarray(self.rot_xyzw, float)
        if quat.shape != (4,) or not np.isfinite(quat).all() or not np.isclose(np.linalg.norm(quat), 1.0, atol=1e-5):
            raise ValueError(f"{self.name}: rot_xyzw must be a unit quaternion")


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
        super()._check()
        if self.kind not in ("box", "capsule"):
            raise ValueError(f"{self.name}: kind must be 'box' or 'capsule'")
        if self.mass is not None:
            raise ValueError(f"{self.name}: a Shape is fixed (no mass)")


@dataclass(frozen=True)
class BodyState(ObjectState):
    """The body origin's pose, velocities, bounds and joint positions (rad or m)."""

    joints: dict = field(default_factory=dict)
    joint_velocities: dict = field(default_factory=dict)

    def rotation(self) -> np.ndarray:
        x, y, z, w = self.quat_xyzw
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])

    def to_local(self, world_point) -> np.ndarray:
        return self.rotation().T @ (np.asarray(world_point, float) - self.pos)

    def to_world(self, local_point) -> np.ndarray:
        return self.pos + self.rotation() @ np.asarray(local_point, float)


# -- Arena objects -------------------------------------------------------------------------------------------------

_SPECS: dict[str, UsdBody | Shape] = {}
_DRIVE_TARGETS: dict[str, dict[str, float]] = {}
_BODY_BOUND_CORNERS: dict[str, dict[str, np.ndarray]] = {}


def _usd_object(spec: UsdBody):
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import (PhysxArticulationRootPropertiesCfg,
        PhysxCollisionPropertiesCfg, PhysxRigidBodyPropertiesCfg)

    material = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0], dynamic_friction=spec.friction[1],
                                              friction_combine_mode="min" if spec.friction_min else "average")
    addon = {"physics_material": material}
    if spec.articulation:
        addon["articulation_props"] = PhysxArticulationRootPropertiesCfg(
            solver_position_iteration_count=spec.solver_iters,
            solver_velocity_iteration_count=spec.solver_velocity_iters)
        # Isaac Lab command buffers start at zero. Preserve nonzero USD drive
        # targets explicitly as well as the stiffness/damping values.
        from pxr import Usd, UsdGeom, UsdPhysics
        usd_stage = Usd.Stage.Open(str(HERE / spec.usd))
        targets = {}
        bound_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        bounds = {}
        for prim in usd_stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                box = bound_cache.ComputeUntransformedBound(prim).ComputeAlignedRange()
                lo, hi = box.GetMin(), box.GetMax()
                bounds[prim.GetName()] = np.array([(x,y,z) for x in (lo[0],hi[0])
                    for y in (lo[1],hi[1]) for z in (lo[2],hi[2])], float)
            if not prim.IsA(UsdPhysics.Joint):
                continue
            kind = "angular" if prim.IsA(UsdPhysics.RevoluteJoint) else "linear"
            drive = UsdPhysics.DriveAPI(prim, kind)
            if drive and drive.GetTargetPositionAttr().HasAuthoredValueOpinion():
                value = float(drive.GetTargetPositionAttr().Get())
                targets[prim.GetName()] = float(np.deg2rad(value)) if kind == "angular" else value
        targets.update(dict(spec.joint_targets))
        _DRIVE_TARGETS[spec.name] = targets
        _BODY_BOUND_CORNERS[spec.name] = bounds
    else:
        addon["rigid_props"] = PhysxRigidBodyPropertiesCfg(solver_position_iteration_count=spec.solver_iters,
                solver_velocity_iteration_count=spec.solver_velocity_iters,
                kinematic_enabled=spec.mass is None)
        if spec.torsional_patch:
            addon["collision_props"] = PhysxCollisionPropertiesCfg(
                torsional_patch_radius=spec.torsional_patch, min_torsional_patch_radius=0.8 * spec.torsional_patch)
        if spec.mass is not None:
            addon["mass_props"] = sim_utils.MassPropertiesCfg(mass=spec.mass)

    class UsdObject(Object):
        def _build_reset_event(self):
            event = super()._build_reset_event()
            if event is not None and spec.articulation:
                # Arena's generic object event resets only the root pose. A
                # configured init_state joint position is merely a default
                # buffer until it is explicitly written at episode reset.
                from isaaclab_arena.terms.events import reset_articulation_pose_and_joints
                event.func = reset_articulation_pose_and_joints
            return event

        def _get_spawn_cfg(self, activate_contact_sensors=False):
            cfg = super()._get_spawn_cfg(activate_contact_sensors)
            if not spec.articulation and spec.mass is None:
                spawn = cfg.func

                def spawn_kinematic(*args, **kwargs):
                    from pxr import PhysxSchema, Usd, UsdPhysics
                    prim = spawn(*args, **kwargs)
                    # CCD is meaningful only for dynamic actors. Override it in
                    # the spawned stage; the source USD remains unchanged.
                    for child in Usd.PrimRange(prim):
                        if child.HasAPI(UsdPhysics.RigidBodyAPI):
                            PhysxSchema.PhysxRigidBodyAPI.Apply(child).CreateEnableCCDAttr(False)
                    return prim

                cfg.func = spawn_kinematic
            return cfg

        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            joints = dict(spec.joint_pos)
            cfg.init_state.joint_pos = joints or {".*": 0.0}
            cfg.actuators = {"joints": ImplicitActuatorCfg(joint_names_expr=[".*"],
                stiffness=spec.joint_stiffness, damping=spec.joint_damping,
                friction=spec.joint_friction, joint_effort_limit=spec.joint_effort_limit)}
            return cfg

    obj = UsdObject(name=spec.name, usd_path=str(HERE / spec.usd),
                    object_type=ObjectType.ARTICULATION if spec.articulation else ObjectType.RIGID,
                    spawn_cfg_addon=addon)
    obj.set_initial_pose(Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=tuple(float(v) for v in spec.rot_xyzw)))
    return obj


def _shape_object(spec: Shape):
    import isaaclab.sim as sim_utils
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg

    from yam_arena.station import linear_rgb

    body = {"visual_material": sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.6)}
    if spec.collide:
        body["collision_props"] = PhysxCollisionPropertiesCfg()
    if spec.kind == "box":
        cfg = sim_utils.CuboidCfg(size=tuple(spec.size), **body)
    else:
        cfg = sim_utils.CapsuleCfg(radius=spec.radius, height=spec.height, axis=spec.axis, **body)
    obj = Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg)
    obj.set_initial_pose(Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=tuple(float(v) for v in spec.rot_xyzw)))
    return obj


def _np(x) -> np.ndarray:
    value = getattr(x, "torch", x)
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=np.float64)


def _state(environment, spec: UsdBody | Shape) -> BodyState:
    if isinstance(spec, Shape):
        pos = np.asarray(spec.pos, float)
        state = BodyState(pos=pos, quat_xyzw=np.asarray(spec.rot_xyzw, float), lin_vel=np.zeros(3),
                          ang_vel=np.zeros(3), low=pos, high=pos)
        half = np.abs(state.rotation()) @ (np.asarray(spec.size) / 2)
        return BodyState(pos=pos, quat_xyzw=state.quat_xyzw, lin_vel=np.zeros(3),
                         ang_vel=np.zeros(3), low=pos-half, high=pos+half)
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    # Isaac Lab 3.0 root_link_pose_w is explicitly XYZW; use actor-frame velocity
    # rather than the older root_vel_w alias (which denotes centre-of-mass velocity).
    vel = _np(asset.data.root_link_vel_w)[0]
    joints, joint_velocities = {}, {}
    if spec.articulation:
        q = _np(asset.data.joint_pos)[0]
        dq = _np(asset.data.joint_vel)[0]
        joints = {n: float(q[i]) for i, n in enumerate(asset.joint_names)}
        joint_velocities = {n: float(dq[i]) for i, n in enumerate(asset.joint_names)}
    pos = pose[:3] - origin
    if spec.articulation:
        # Arena's cached root AABB assumes rigidly fixed descendants, so it cannot
        # represent an opened cam. Transform each authored link's bounds by its
        # measured physics pose instead.
        points = []
        for name, link_pose in body_link_poses(environment, spec.name).items():
            corners = _BODY_BOUND_CORNERS[spec.name][name]
            x,y,z,w = link_pose[3:7]
            rot = np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                            [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                            [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
            points.append(corners @ rot.T + link_pose[:3])
        vertices = np.concatenate(points)
        low,high = vertices.min(axis=0),vertices.max(axis=0)
    else:
        box = base.arena_world.get_aabb_w(spec.name)
        low, high = _np(box.min_point)[0]-origin, _np(box.max_point)[0]-origin
    return BodyState(pos=pos, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=low, high=high,
                     joints=joints, joint_velocities=joint_velocities)


def body_link_poses(environment, name: str) -> dict[str, np.ndarray]:
    """Measured per-link pose7 [x,y,z,qx,qy,qz,qw] in the task frame.

    Uses PhysX-backed pose buffers, never stale USD transforms. Returns copies.
    """
    base = environment.unwrapped
    asset = base.scene[name]
    origin = _np(base.scene.env_origins[0])
    if hasattr(asset.data, "body_link_pose_w"):
        poses = _np(asset.data.body_link_pose_w)[0].copy()
        poses[:, :3] -= origin
        return {key: poses[i].copy() for i, key in enumerate(asset.body_names)}
    pose = _np(asset.data.root_link_pose_w)[0].copy()
    pose[:3] -= origin
    return {name: pose}


def set_joint_targets(environment, name: str, targets: dict[str, float]) -> None:
    """Set physical actuator rest targets (rad/m), without changing measured poses."""
    if not targets:
        return
    import torch
    asset = environment.unwrapped.scene[name]
    names = list(targets)
    ids, resolved = asset.find_joints(names, preserve_order=True)
    if list(resolved) != names:
        raise ValueError(f"{name}: joint targets {names} resolved to {resolved}")
    values = torch.tensor([[float(targets[n]) for n in names]], dtype=torch.float32, device=asset.device)
    values = values.repeat(asset.num_instances, 1)
    asset.actuators.target_command.set_position_index(value=values, joint_ids=list(ids))


def set_joint_gains(environment, name: str, stiffness: dict[str, float], damping: dict[str, float]) -> None:
    """Update native implicit drive gains in SI units, without changing poses.

    For angular joints, stiffness is N m/rad and damping is N m s/rad.
    The installed Isaac Lab 3.0 setters update both PhysX and the articulation
    data buffers. This helper is intended for native implicit actuators.
    """
    import torch
    asset = environment.unwrapped.scene[name]
    cache = getattr(asset, "_scenekit_joint_gain_cache", None)
    if cache is None:
        cache = asset._scenekit_joint_gain_cache = {}
    for kind, gains in (("stiffness", stiffness), ("damping", damping)):
        if not gains:
            continue
        names = list(gains)
        raw = [float(gains[joint]) for joint in names]
        if not all(np.isfinite(value) and value >= 0.0 for value in raw):
            raise ValueError(f"{name}: {kind} must be finite and nonnegative")
        # PhysX stores float32 gains. Skip identical native commands before
        # allocating GPU tensors or synchronizing its parameter buffers.
        raw = [float(np.float32(value)) for value in raw]
        if all(cache.get((kind, joint)) == value for joint, value in zip(names, raw)):
            continue
        ids, resolved = asset.find_joints(names, preserve_order=True)
        if list(resolved) != names:
            raise ValueError(f"{name}: joint gains {names} resolved to {resolved}")
        values = torch.tensor([raw], dtype=torch.float32, device=asset.device).repeat(asset.num_instances, 1)
        setter = getattr(asset, f"write_joint_{kind}_to_sim_index")
        setter(**{kind: values}, joint_ids=list(ids))
        cache.update({(kind, joint): value for joint, value in zip(names, raw)})


def _apply_drive_targets(environment) -> None:
    for name, targets in _DRIVE_TARGETS.items():
        if name in environment.unwrapped.scene.articulations:
            set_joint_targets(environment, name, targets)


def robot_links(environment) -> dict[str, dict[str, np.ndarray]]:
    """Each arm's link origins by link name (station frame, env 0), for proximity checks."""
    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    out = {}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        pos = _np(robot.data.body_link_pose_w)[0, :, :3] - origin
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
    # Millimetre steel contacts need smaller native substeps; applies equally
    # to scripted and inspect-robots model runs. Control remains 20 Hz.
    scene.PHYSICS_DT = 1.0 / 1000.0
    if getattr(scene, "_scenekit", False):
        return
    # A declared fixed witness camera is created with the scene, before native
    # sensor initialization. It is recorded separately, not a controller input.
    original_config_factory = scene.env_cfg_callback
    def config_factory(control_hz=None):
        configure = original_config_factory(control_hz)
        def with_witness(cfg):
            cfg = configure(cfg)
            from witness import camera_cfg
            cfg.scene.witness_cam = camera_cfg()
            return cfg
        return with_witness
    scene.env_cfg_callback = config_factory
    task_object, object_states = scene.task_object, scene.object_states

    def task_object_hook(spec):
        if isinstance(spec, UsdBody | Shape):
            _SPECS[spec.name] = spec
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
    task_step, reset, ensure_sim = cls._task_step, cls.reset, cls._ensure_sim

    def ensure_sim_hook(self, *a, **k):
        result = ensure_sim(self, *a, **k)
        _apply_drive_targets(self._env)
        return result

    def task_step_hook(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return task_step(self, *a, **k)

    def reset_hook(self, *a, **k):
        obs = reset(self, *a, **k)
        _ENV["env"] = self._env
        # A native reset or a reset callback may restore drive parameters. The
        # first subsequent command must therefore always reach PhysX.
        for asset in self._env.unwrapped.scene.articulations.values():
            cache = getattr(asset, "_scenekit_joint_gain_cache", None)
            if cache is not None:
                cache.clear()
        _apply_drive_targets(self._env)
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

