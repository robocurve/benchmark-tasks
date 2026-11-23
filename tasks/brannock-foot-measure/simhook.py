"""simhook: lets a yam_arena task file spawn USD assets (rigid bodies and a fixed-base articulation with passive,
friction-only joints) and read their state, without modifying yam_arena.

Importing it installs a post-import hook on ``yam_arena.scene`` that wraps three of its functions:
``task_object`` (builds the Isaac Lab asset for a :class:`UsdObject`), ``object_states`` (adds the USD objects'
root states) and ``env_cfg_callback`` (adds an overhead close-up camera only when BRANNOCK_CLOSEUP is set, to render
the setup image; trials never set it). Joints of an articulation get an Isaac Lab ``ImplicitActuatorCfg`` with zero
stiffness and zero damping, i.e. no drive at all, and the given Coulomb friction (N): the joints move only when
something pushes them. Pure Python at import; Isaac Lab is imported lazily when the scene is built.
"""

from __future__ import annotations

import importlib.abc
import importlib.machinery
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena import task_file
from yam_arena.task_file import Box, ObjectState

_SPECS: dict = {}
_STATE: dict = {"env": None}
_RESET: list = []


@dataclass(frozen=True)
class UsdObject(Box):
    """A USD asset. ``size`` is its nominal bounding box, ``rot_xyzw`` its spawn orientation."""

    usd: str = ""
    articulated: bool = False
    rot_xyzw: tuple = (0.0, 0.0, 0.0, 1.0)
    joint_pos0: tuple = ()  # ((joint name, value), ...)
    joint_friction: tuple = (0.0, 0.0)  # (static, dynamic) Coulomb friction of every joint, N

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")

    @property
    def free(self) -> bool:
        return True


def on_reset(fn):
    _RESET.append(fn)
    return fn


def env():
    return _STATE["env"]


def _np(x) -> np.ndarray:
    x = x.torch if hasattr(x, "torch") else x
    return x.detach().cpu().numpy().astype(np.float64)


def joints(name: str) -> tuple[dict, dict]:
    """(positions, velocities) of an articulation's joints by name."""
    a = env().unwrapped.scene[name]
    names = list(a.joint_names)
    return dict(zip(names, _np(a.data.joint_pos)[0])), dict(zip(names, _np(a.data.joint_vel)[0]))


def robot_points(bodies=("tip_left", "tip_right", "gripper")) -> np.ndarray:
    """World positions (env frame) of the named links of both arms, stacked."""
    sc = env().unwrapped.scene
    origin = _np(sc.env_origins)[0]
    out = []
    for robot in ("left_robot", "right_robot"):
        r = sc[robot]
        poses = _np(r.data.body_link_pose_w)[0]
        for b in bodies:
            if b in r.body_names:
                out.append(poses[r.body_names.index(b), :3] - origin)
    return np.array(out)


def _isaac_object(spec: UsdObject):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    pose = Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=tuple(float(v) for v in spec.rot_xyzw))
    if not spec.articulated:
        return Object(name=spec.name, usd_path=spec.usd, object_type=ObjectType.RIGID, initial_pose=pose)
    fs, fd = spec.joint_friction
    acts = {"passive": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=0.0, damping=0.0, friction=fs,
                                          dynamic_friction=fd, effort_limit_sim=1.0e4)}
    j0 = dict(spec.joint_pos0)

    class _Passive(Object):
        def _generate_articulation_cfg(self):
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self._get_spawn_cfg(activate_contact_sensors=False),
                                  init_state=ArticulationCfg.InitialStateCfg(joint_pos=j0), actuators=acts)
            return self._add_initial_pose_to_cfg(cfg)

    return _Passive(name=spec.name, usd_path=spec.usd, object_type=ObjectType.ARTICULATION, initial_pose=pose)


_ORIG: dict = {}


def _task_object(spec):
    if isinstance(spec, UsdObject):
        _SPECS[spec.name] = spec
        return _isaac_object(spec)
    return _ORIG["task_object"](spec)


def _object_states(environment, names):
    _STATE["env"] = environment
    plain = [n for n in names if n not in _SPECS]
    out = _ORIG["object_states"](environment, plain) if plain else {}
    sc = environment.unwrapped.scene
    origin = _np(sc.env_origins)[0]
    for n in names:
        if n not in _SPECS:
            continue
        d = sc[n].data
        pose = _np(d.root_link_pose_w)[0] if hasattr(d, "root_link_pose_w") else _np(d.root_pose_w)[0]
        pos = pose[:3] - origin
        vel = _np(d.root_lin_vel_w)[0] if hasattr(d, "root_lin_vel_w") else np.zeros(3)
        ang = _np(d.root_ang_vel_w)[0] if hasattr(d, "root_ang_vel_w") else np.zeros(3)
        half = np.asarray(_SPECS[n].size) / 2
        out[n] = ObjectState(pos=pos, quat_xyzw=pose[3:7], lin_vel=vel, ang_vel=ang, low=pos - half, high=pos + half)
    return out


CLOSEUP = dict(pos=(0.25, -0.035, 1.22), size=(1920, 1080))  # overhead still of the device (setup image only)


def _env_cfg_callback(control_hz=None):
    """Adds one overhead close-up camera when BRANNOCK_CLOSEUP is set (used only to render the setup image)."""
    base = _ORIG["env_cfg_callback"](control_hz)

    def configure(env_cfg):
        env_cfg = base(env_cfg)
        import os

        if os.environ.get("BRANNOCK_CLOSEUP"):
            import isaaclab.sim as sim_utils
            from isaaclab.sensors import CameraCfg

            s = 2 ** -0.5
            env_cfg.scene.closeup_cam = CameraCfg(
                prim_path="{ENV_REGEX_NS}/closeup_cam", update_period=0.0, height=CLOSEUP["size"][1],
                width=CLOSEUP["size"][0], data_types=["rgb"],
                spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, clipping_range=(0.05, 5.0)),
                offset=CameraCfg.OffsetCfg(pos=CLOSEUP["pos"], rot=(0.0, s, 0.0, s), convention="world"),
                renderer_cfg=_rig_renderer())
        return env_cfg

    return configure


def _rig_renderer():
    """The same renderer settings as the rig's cameras (they are process-global)."""
    from yam_arena import embodiment

    return embodiment.RENDERER


def closeup_image():
    sc = env().unwrapped.scene
    return _np(sc["closeup_cam"].data.output["rgb"])[0][..., :3].astype(np.uint8)


def _patch_reset():
    from yam_arena import inspect_embodiment as ie

    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_simhook_patched", False):
        return
    orig = cls.reset

    def reset(self, scene, *, seed=None):
        for fn in _RESET:
            fn()
        obs = orig(self, scene, seed=seed)
        _STATE["env"] = self._env
        for fn in _RESET:
            fn()
        return obs

    cls.reset = reset
    cls._simhook_patched = True


def _install(scene) -> None:
    if getattr(scene, "_simhook", False):
        return
    for k in ("task_object", "object_states", "env_cfg_callback"):
        _ORIG[k] = getattr(scene, k)
    scene.task_object = _task_object
    scene.object_states = _object_states
    scene.env_cfg_callback = _env_cfg_callback
    scene._simhook = True


class _AfterImport(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    """Calls _install as soon as yam_arena.scene has executed."""

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
elif not any(isinstance(f, _AfterImport) for f in sys.meta_path):
    sys.meta_path.insert(0, _AfterImport())
_patch_reset()
assert issubclass(UsdObject, task_file.Box)
