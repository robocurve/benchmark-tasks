"""guardkit: adapted research fixture bridge for yam_arena / Isaac Lab-Arena.

Based on the on-step/object-state bridge from the accepted RoboCurve
lock-before-loading example (original implementation supplied in the user
repository snapshot). The native integration must be requalified on the
installed YAM and Isaac Lab versions; import hooks are version-sensitive.

USD bodies, articulations and posed fixed shapes for a yam_arena task file.

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
import importlib.util
import json
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["BodyState", "Shape", "UsdBody", "arm_joints", "env", "on_reset", "on_step", "robot_links"]

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
    joint_actuators: tuple[tuple[str, float, float], ...] = ()  # joint, kp, kd
    joint_actuator_limits: tuple[tuple[str, float, float], ...] = ()  # joint, effort, velocity
    friction: tuple[float, float] = (0.8, 0.7)  # static, dynamic
    friction_min: bool = False  # combine friction with "min", so this surface's friction rules its contacts
    solver_iters: int = 16
    self_collisions: bool = False  # enable contacts between parts of the same articulation
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

_SPECS: dict[str, UsdBody | Shape] = {}


def _usd_object(spec: UsdBody):
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    material = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0], dynamic_friction=spec.friction[1],
                                              friction_combine_mode="min" if spec.friction_min else "average")
    addon = {"physics_material": material}
    if spec.articulation:
        addon["activate_contact_sensors"] = True
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=spec.solver_iters,
            enabled_self_collisions=spec.self_collisions)
    else:
        addon["rigid_props"] = sim_utils.RigidBodyPropertiesCfg(solver_position_iteration_count=spec.solver_iters,
                                                                kinematic_enabled=spec.mass is None)
        if spec.torsional_patch:
            addon["collision_props"] = sim_utils.CollisionPropertiesCfg(
                torsional_patch_radius=spec.torsional_patch, min_torsional_patch_radius=0.8 * spec.torsional_patch)
        if spec.mass is not None:
            addon["mass_props"] = sim_utils.MassPropertiesCfg(mass=spec.mass)

    class UsdObject(Object):
        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            joints = dict(spec.joint_pos)
            cfg.init_state.joint_pos = joints
            if spec.joint_actuators:
                limits=dict((n,(effort,velocity)) for n,effort,velocity in spec.joint_actuator_limits)
                cfg.actuators = {
                    n: ImplicitActuatorCfg(
                        joint_names_expr=[n], stiffness=kp, damping=kd, friction=0.0,
                        joint_effort_limit=limits[n][0], joint_velocity_limit=limits[n][1])
                    for n, kp, kd in spec.joint_actuators
                }
            else:
                cfg.actuators = {"joints": ImplicitActuatorCfg(joint_names_expr=list(joints) or [".*"], stiffness=0.0,
                                                               damping=spec.joint_damping, friction=0.0)}
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
    value = getattr(x, "torch", x)
    if hasattr(value, "detach"):
        value = value.detach().cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value, dtype=np.float64)


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
    if spec.articulation:
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



# -- controlled episode reset for the articulated guard ------------------------
# Arena release/0.3.1 can return an articulation in its USD-authored (closed)
# rest pose even when the task's Isaac Lab cfg.init_state.joint_pos requests an
# open hinge. The robot must not receive credit for a preclosed safety guard.
# Resetting joint state is permitted ONLY as part of env.reset() BEFORE Arena's
# 0.5-second physics-settling period. Never teleport joints during a trial.
_RESET_PHASES: list[dict] = []


def _guard_joint_snapshot(environment) -> dict[str, float]:
    asset = environment.unwrapped.scene['machine_guard']
    q = _np(asset.data.joint_pos)[0]
    return {name: float(q[i]) for i, name in enumerate(asset.joint_names)}


def drive_state(environment):
    """Record Isaac's configured targets and estimates separately from measured positions."""
    asset = environment.unwrapped.scene['machine_guard']
    data = asset.data
    collection = getattr(asset, 'actuators', None)
    command = getattr(collection, 'target_command', None)
    targets = getattr(command, 'position', None)
    if targets is None:
        targets = data.joint_pos_target
    def by_joint(value):
        values = _np(value)[0]
        return {str(name): float(values[i]) for i, name in enumerate(asset.joint_names)}
    state = {'position_targets': by_joint(targets)}
    fields = (('joint_vel', 'measured_velocities'), ('joint_stiffness', 'stiffness'),
              ('joint_damping', 'damping'), ('applied_torque', 'estimated_efforts'))
    for attr, key in fields:
        try:
            value = getattr(data, attr, None)
            if value is not None:
                state[key] = by_joint(value)
        except (AttributeError, TypeError, IndexError, RuntimeError) as exc:
            state[key+'_error'] = str(exc)[:180]
    return state


def _reset_phase(environment, name: str) -> None:
    try:
        row = {'phase': name, 'joint_positions': _guard_joint_snapshot(environment)}
        try:
            base=environment.unwrapped
            asset=base.scene['machine_guard']
            pose=_np(asset.data.root_link_pose_w)[0]
            root_vel=_np(asset.data.root_vel_w)[0]
            row['root_pose_w']=[float(v) for v in pose[:7]]
            row['root_velocity_w']=[float(v) for v in root_vel[:6]]

            # Full body poses reveal whether joint coordinates match the link
            # transforms.  This distinguishes joint-drive effects from deep
            # contact penetration during native settling.
            for attr, key in (('body_pose_w', 'body_poses_w'),
                              ('body_vel_w', 'body_velocities_w')):
                try:
                    data = _np(getattr(asset.data, attr))[0]
                    row[key] = {str(n): [float(v) for v in data[i]]
                                for i, n in enumerate(asset.body_names)}
                except Exception as error:
                    row[key+'_unavailable'] = str(error)[:100]
            try:
                vel = _np(asset.data.joint_vel)[0]
                row['joint_velocities'] = {str(n):float(vel[i])
                        for i,n in enumerate(asset.joint_names)}
            except Exception as error:
                row['joint_velocities_unavailable'] = str(error)[:100]
            for attr, key in (('joint_pos_target', 'joint_targets'),
                              ('applied_torque', 'applied_joint_efforts'),
                              ('computed_torque', 'computed_joint_efforts')):
                try:
                    value = getattr(asset.data, attr)
                    if value is None: continue
                    arr=_np(value)[0]
                    row[key]={str(n):float(arr[i]) for i,n in enumerate(asset.joint_names)}
                except Exception:
                    pass

            # Link origins are approximate proximity diagnostics, not measured
            # contacts, and must not replace the native contact/visual audit.
            links=robot_links(environment)
            q=row['joint_positions']
            import math
            theta=q['hinge']
            x0,y0=0.315,0.104
            x,y=-0.025,-0.149
            handle=np.asarray([x0+x*math.cos(theta)-y*math.sin(theta),
                               y0+x*math.sin(theta)+y*math.cos(theta),
                               pose[2]+0.102],float)
            from geometry import BOLT_PAD_LOCAL
            bolt_tab=np.asarray([x0-.014,y0-.218+q['bolt_slide'],pose[2]+.068],float)+BOLT_PAD_LOCAL
            row['nearest_arm_link_origin_m']={}
            for side, body in links.items():
                points=np.stack([np.asarray(p,float) for p in body.values()])
                row['nearest_arm_link_origin_m'][side]={
                    'handle':float(np.min(np.linalg.norm(points-handle,axis=1))),
                    'bolt_tab':float(np.min(np.linalg.norm(points-bolt_tab,axis=1)))}
        except Exception as diagnostic_exc:
            row['scene_diagnostic_error']=str(diagnostic_exc)[:180]
    except Exception as exc:
        row = {'phase': name, 'error': str(exc)}
    _RESET_PHASES.append(row)
    output = os.environ.get('GUARD_RESET_PHASES_JSONL')
    if output:
        with open(output, 'a', encoding='utf8') as stream:
            stream.write(json.dumps(row, sort_keys=True) + '\n')


def _set_guard_episode_initial_joints(environment) -> None:
    """Reset all three physical DOFs, with zero velocity, before settling.

    Isaac Lab 3.0 exposes write_joint_position_to_sim_index / velocity_index.
    Older releases expose write_joint_state_to_sim. Fail closed if neither is
    supported: silently skipping the write invalidates the task.
    """
    spec = _SPECS.get('machine_guard')
    if not isinstance(spec, UsdBody) or not spec.articulation:
        raise RuntimeError('machine_guard articulation not registered for episode reset')
    asset = environment.unwrapped.scene['machine_guard']
    names = list(asset.joint_names)
    required = dict(spec.joint_pos)
    if set(required) != {'hinge', 'bolt_slide', 'switch_slide'}:
        raise RuntimeError('Unexpected machine guard reset joint specification')
    if not set(required).issubset(names):
        raise RuntimeError(f'Cannot reset guard: expected {sorted(required)}, got {names}')
    # Clone existing runtime tensors so all indexing, device and batch dimensions
    # remain unchanged. All other joints (if present) retain their current value.
    q = asset.data.joint_pos.torch.clone()
    v = asset.data.joint_vel.torch.clone()
    v.zero_()
    for joint, target in required.items():
        q[:, names.index(joint)] = float(target)
    if hasattr(asset, 'write_joint_position_to_sim_index') and hasattr(asset, 'write_joint_velocity_to_sim_index'):
        asset.write_joint_position_to_sim_index(position=q)
        asset.write_joint_velocity_to_sim_index(velocity=v)
    elif hasattr(asset, 'write_joint_state_to_sim'):
        asset.write_joint_state_to_sim(q, v)
    else:
        raise RuntimeError('No supported Isaac Lab articulation joint-state reset API')
    # This is NOT manipulation and must not run inside _task_step or on_step.
    # Zero velocity and reset the physical spring's target to its home position.
    # Only the reset sets the passive spring rest positions.
    if hasattr(asset, 'set_joint_position_target'):
        target = q.clone()
        target[:, names.index('switch_slide')] = 0.0
        asset.set_joint_position_target(target)


def _install_guard_reset_before_settle(environment) -> None:
    if getattr(environment, '_machine_guard_joint_reset_v3', False):
        return
    original_reset = environment.reset

    def guarded_reset(*args, **kwargs):
        # Environment reset can overwrite task-specific initial joints in
        # Arena 0.3.1. Sample before & after the explicit state restoration.
        _RESET_PHASES.clear()
        obs = original_reset(*args, **kwargs)
        _reset_phase(environment, 'after_environment_reset')
        if os.environ.get('GUARD_DISABLE_EXPLICIT_RESET') != '1':
            _set_guard_episode_initial_joints(environment)
            _reset_phase(environment, 'after_explicit_joint_reset')
        else:
            _reset_phase(environment, 'explicit_joint_reset_disabled')
        environment._guard_settle_ticks = 0
        environment._guard_record_settle = True
        return obs

    environment.reset = guarded_reset
    # YAM's embodiment advances the environment for 0.5 s after reset. Sample
    # every native env.step to locate the precise first jump, not merely the
    # initial and final snapshots. This wrapper NEVER changes the actions.
    original_step = getattr(environment, 'step', None)
    def diagnostic_step(*args, **kwargs):
        result = original_step(*args, **kwargs)
        if getattr(environment, '_guard_record_settle', False):
            environment._guard_settle_ticks += 1
            tick = environment._guard_settle_ticks
            _reset_phase(environment, f'settle_step_{tick:03d}')
            dest = os.environ.get('GUARD_SETTLE_MEDIA_DIR')
            if dest:
                try:
                    from PIL import Image
                    camera_root = Path(dest)
                    camera_root.mkdir(parents=True, exist_ok=True)
                    for cam in ('top_cam', 'left_cam', 'right_cam'):
                        frame = environment.unwrapped.scene[cam].data.output['rgb']
                        pixels = _np(frame)[0]
                        import numpy as np
                        pixels=np.asarray(pixels[:,:,:3]).clip(0,255).astype('uint8')
                        Image.fromarray(pixels).save(camera_root/f'{cam}_{tick:03d}.png')
                except Exception as exc:
                    camera_root = Path(dest)
                    camera_root.mkdir(parents=True, exist_ok=True)
                    with (camera_root/'media_errors.log').open('a') as stream:
                        stream.write(f'{tick}: {type(exc).__name__}: {exc}\n')
        return result
    if callable(original_step):
        environment.step = diagnostic_step
    environment._guard_record_settle = False
    environment._guard_settle_ticks = 0
    environment._machine_guard_joint_reset_v3 = True

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
        if isinstance(spec, UsdBody | Shape):
            _SPECS[spec.name] = spec
            return _usd_object(spec) if isinstance(spec, UsdBody) else _shape_object(spec)
        return task_object(spec)

    def object_states_hook(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _state(environment, _SPECS[n]) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    # Select CPU rigid-body dynamics only for this task. Kit keeps the station RTX
    # device for all camera rendering. Stock Arena still builds and owns the scene.
    make_env = scene.make_env
    def guard_environment(embodiment, **kwargs):
        spec = kwargs.get("task")
        if any(isinstance(obj, UsdBody) and obj.name == "machine_guard"
               for obj in getattr(spec, "objects", ())):
            import carb.settings
            carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
            kwargs["device"] = "cpu"
        return make_env(embodiment, **kwargs)
    scene.make_env = guard_environment
    scene.task_object, scene.object_states = task_object_hook, object_states_hook
    scene._scenekit = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_scenekit", False):
        return
    task_step, reset, ensure_sim = cls._task_step, cls.reset, cls._ensure_sim

    def ensure_sim_hook(self, *a, **k):
        result = ensure_sim(self, *a, **k)
        # This runs before YamArenaEmbodiment.reset calls self._env.reset(),
        # and therefore before the embodiment's 0.5 s settling loop.
        _install_guard_reset_before_settle(self._env)
        return result

    def task_step_hook(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return task_step(self, *a, **k)

    def reset_hook(self, *a, **k):
        obs = reset(self, *a, **k)
        _ENV["env"] = self._env
        self._env._guard_record_settle = False
        _reset_phase(self._env, 'after_embodiment_settle')
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
