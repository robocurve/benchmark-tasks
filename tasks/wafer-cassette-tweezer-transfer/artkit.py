"""artkit: USD parts, sprung joints and contact sensors for a yam_arena task file, without modifying yam_arena.

A task file lists ``UsdPart`` objects in ``OBJECTS`` next to the usual primitives. Importing this module hooks
``yam_arena.scene`` (the object factory, the per-step object states and the env config) and the embodiment's reset:

- ``UsdPart(name, pos, usd=..., kind="rigid" | "kinematic" | "articulation", ...)`` spawns the USD file. An
  articulation's joints are springs: implicit PD actuators holding ``joint_pos0`` with the given stiffness and damping
  (Isaac Lab ``ImplicitActuatorCfg``), so a gripper must push against them.
- ``sensor(name, prim, filters)`` adds an Isaac Lab ``ContactSensorCfg`` (normal forces filtered to ``filters``).
- ``force(name)`` reads the largest filtered normal force over the last control step's physics substeps (N).
- ``joints(name)``, ``body_pose(name, body)`` read articulation state; ``on_reset(fn)`` runs fn at every reset.

Pure Python at import; Isaac Lab is imported lazily when the scene is built.
"""

from __future__ import annotations

from dataclasses import dataclass

import importlib.abc
import importlib.machinery
import sys

import numpy as np
from yam_arena import task_file
from yam_arena.task_file import Box, ObjectState

__all__ = ["UsdPart", "sensor", "force", "forces", "joints", "body_pose", "on_reset", "env"]


@dataclass(frozen=True)
class UsdPart(Box):
    """A part spawned from a USD file. ``size`` is only its nominal bounding box (for object states)."""

    usd: str = ""
    kind: str = "rigid"  # rigid | kinematic | articulation
    joint_pos0: tuple = ()  # ((joint, value), ...)
    springs: tuple = ()  # ((joint_regex, stiffness, damping), ...)
    rot_xyzw: tuple = (0.0, 0.0, 0.0, 1.0)

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if self.kind not in ("rigid", "kinematic", "articulation"):
            raise ValueError(f"{self.name}: kind must be rigid, kinematic or articulation")

    @property
    def free(self) -> bool:
        return self.kind != "kinematic"


_SENSORS: dict[str, dict] = {}
_RESET: list = []
_ENV = None
_PARTS: dict[str, UsdPart] = {}


def sensor(name: str, prim: str, filters: list[str]) -> None:
    """A contact sensor at ``{ENV_REGEX_NS}/<prim>`` reporting normal forces against ``filters`` (prim paths)."""
    _SENSORS[name] = dict(prim=prim, filters=list(filters))


def on_reset(fn):
    _RESET.append(fn)
    return fn


def env():
    return _ENV


def _t(x) -> np.ndarray:
    x = x.torch if hasattr(x, "torch") else x
    return x.detach().cpu().numpy().astype(np.float64)


def forces(name: str) -> np.ndarray:
    """Per-substep max filtered normal force (N) over the last control step, shape (substeps,)."""
    if _ENV is None:
        return np.zeros(1)
    data = _ENV.unwrapped.scene[name].data
    hist = getattr(data, "normal_force_matrix_w_history", None)
    if hist is None:
        now = _t(data.normal_force_matrix_w)[0]
        return np.array([np.linalg.norm(now, axis=-1).max()])
    h = _t(hist)[0]  # (T, bodies, filters, 3)
    return np.linalg.norm(h, axis=-1).max(axis=(1, 2))


def force(name: str) -> float:
    return float(forces(name).max())


def joints(name: str) -> dict[str, float]:
    a = _ENV.unwrapped.scene[name]
    return dict(zip(a.joint_names, _t(a.data.joint_pos)[0].tolist()))


def body_pose(name: str, body: str) -> tuple[np.ndarray, np.ndarray]:
    """(position, quaternion xyzw) of one link of an articulation, env frame."""
    a = _ENV.unwrapped.scene[name]
    i = a.body_names.index(body)
    p = _t(a.data.body_link_pose_w)[0, i]
    origin = _t(_ENV.unwrapped.scene.env_origins)[0]
    return p[:3] - origin, p[3:]


# -- hooks into yam_arena --------------------------------------------------------------------------------------------


def _arena_object(spec: UsdPart):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    pose = Pose(position_xyz=tuple(spec.pos), rotation_xyzw=tuple(spec.rot_xyzw))
    if spec.kind != "articulation":
        return Object(name=spec.name, usd_path=spec.usd, object_type=ObjectType.RIGID, initial_pose=pose)

    j0 = dict(spec.joint_pos0)
    acts = {
        f"spring{i}": ImplicitActuatorCfg(joint_names_expr=[rx], stiffness=k, damping=d, effort_limit_sim=1.0e4)
        for i, (rx, k, d) in enumerate(spec.springs)
    }

    class _Sprung(Object):
        def _generate_articulation_cfg(self):
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self._get_spawn_cfg(activate_contact_sensors=True),
                                  init_state=ArticulationCfg.InitialStateCfg(joint_pos=j0), actuators=acts)
            return self._add_initial_pose_to_cfg(cfg)

    return _Sprung(name=spec.name, usd_path=spec.usd, object_type=ObjectType.ARTICULATION, initial_pose=pose)


_ORIG: dict = {}


def _task_object(spec):
    if isinstance(spec, UsdPart):
        _PARTS[spec.name] = spec
        return _arena_object(spec)
    return _ORIG["task_object"](spec)


def _object_states(environment, names):
    global _ENV
    _ENV = environment
    plain = [n for n in names if n not in _PARTS]
    out = _ORIG["object_states"](environment, plain) if plain else {}
    sc = environment.unwrapped.scene
    origin = _t(sc.env_origins)[0]
    for n in names:
        if n not in _PARTS:
            continue
        d = sc[n].data
        pose = _t(d.root_pose_w)[0] if hasattr(d, "root_pose_w") else _t(d.root_link_pose_w)[0]
        pos = pose[:3] - origin
        vel = _t(d.root_lin_vel_w)[0] if hasattr(d, "root_lin_vel_w") else np.zeros(3)
        ang = _t(d.root_ang_vel_w)[0] if hasattr(d, "root_ang_vel_w") else np.zeros(3)
        half = np.asarray(_PARTS[n].size) / 2
        out[n] = ObjectState(pos=pos, quat_xyzw=pose[3:7], lin_vel=vel, ang_vel=ang, low=pos - half, high=pos + half)
    return out


def _env_cfg_callback(control_hz=None):
    base = _ORIG["env_cfg_callback"](control_hz)

    def configure(env_cfg):
        env_cfg = base(env_cfg)
        if _SENSORS:
            from isaaclab.sensors import ContactSensorCfg

            dec = int(env_cfg.decimation)
            for name, s in _SENSORS.items():
                setattr(env_cfg.scene, name, ContactSensorCfg(
                    prim_path="{ENV_REGEX_NS}/" + s["prim"],
                    filter_prim_paths_expr=["{ENV_REGEX_NS}/" + f for f in s["filters"]],
                    history_length=dec, update_period=0.0))
        return env_cfg

    return configure


def _patch_reset():
    from yam_arena import inspect_embodiment as ie

    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_artkit_patched", False):
        return
    orig = cls.reset

    def reset(self, scene, *, seed=None):
        global _ENV
        for fn in _RESET:
            fn()
        obs = orig(self, scene, seed=seed)
        _ENV = self._env
        for fn in _RESET:
            fn()
        return obs

    cls.reset = reset
    cls._artkit_patched = True


def _install(scene) -> None:
    """Patch yam_arena.scene (imported only after the simulator has started)."""
    if getattr(scene, "_artkit", False):
        return
    for k in ("task_object", "object_states", "env_cfg_callback"):
        _ORIG[k] = getattr(scene, k)
    scene.task_object = _task_object
    scene.object_states = _object_states
    scene.env_cfg_callback = _env_cfg_callback
    scene._artkit = True


class _PostImport(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    """Runs _install right after yam_arena.scene executes, wherever it is imported from."""

    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        spec = importlib.machinery.PathFinder.find_spec(fullname, path)
        if spec is None:
            return None
        self._inner = spec.loader
        spec.loader = self
        return spec

    def create_module(self, spec):
        return self._inner.create_module(spec)

    def exec_module(self, module):
        self._inner.exec_module(module)
        _install(module)


if "yam_arena.scene" in sys.modules:
    _install(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _PostImport) for f in sys.meta_path):
    sys.meta_path.insert(0, _PostImport())
_patch_reset()

# task_file.load type-checks OBJECTS against these classes; UsdPart is a Box, so it passes.
assert issubclass(UsdPart, task_file.Box)
