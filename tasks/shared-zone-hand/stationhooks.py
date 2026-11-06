"""stationhooks: per-step logic for a yam_arena task file (Team T-13's own helper, written for this task).

yam_arena only calls a task file's score()/success() with object states. This module adds three things a task file
can use, without changing yam_arena itself:

    from stationhooks import every_step, at_reset, arm_link_positions, place_body, body_position

    @at_reset
    def start(env): ...            # called after each reset
    @every_step
    def tick(env, step): ...       # called once per control step, with the step count since the reset

    arm_link_positions(env)        # {"left": (n, 3) array, "right": (n, 3) array}, table frame, metres
    place_body(env, "hand", xyz)   # move a loose object to a position (keeps its orientation, zeroes its velocity)
    body_position(env, "hand")     # its current position, table frame

How it works: when yam_arena's embodiment module is imported (after the task file, once Isaac Sim starts), the
embodiment class's per-step and reset methods are wrapped so the registered functions run alongside them.
Everything that needs Isaac Sim or torch is imported inside the functions.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import sys

import numpy as np

EMBODIMENT_MODULE = "yam_arena.inspect_embodiment"
STEP_METHOD_CANDIDATES = ("_task_step", "step")
RESET_METHOD = "reset"

_step_fns: list = []
_reset_fns: list = []
_state = {"env": None, "step": 0}


def every_step(fn):
    _step_fns.append(fn)
    return fn


def at_reset(fn):
    _reset_fns.append(fn)
    return fn


def current_env():
    return _state["env"]


# ---- reading and moving things in the Isaac Lab scene ---------------------------------------------------------------
def _to_numpy(t) -> np.ndarray:
    t = getattr(t, "torch", t)
    return t.detach().cpu().numpy().astype(np.float64)


def _origin(env) -> np.ndarray:
    return _to_numpy(env.unwrapped.scene.env_origins[0])


def arm_link_positions(env) -> dict:
    """Every link origin of each arm, in the table frame (env 0)."""
    scene, origin = env.unwrapped.scene, _origin(env)
    return {side: _to_numpy(scene[f"{side}_robot"].data.body_pos_w)[0] - origin for side in ("left", "right")}


def body_position(env, name: str) -> np.ndarray:
    data = env.unwrapped.scene[name].data
    pose = getattr(data, "root_link_pose_w", None)
    if pose is None:
        pose = data.root_state_w[:, :7]
    return _to_numpy(pose)[0, :3] - _origin(env)


def place_body(env, name: str, xyz) -> None:
    """Teleport a loose object to xyz (table frame), keeping its current orientation and zeroing its velocity."""
    import torch

    obj = env.unwrapped.scene[name]
    data = obj.data
    pose = getattr(data, "root_link_pose_w", None)
    if pose is None:
        pose = data.root_state_w[:, :7]
    pose = getattr(pose, "torch", pose).clone()
    origin = getattr(env.unwrapped.scene.env_origins, "torch", env.unwrapped.scene.env_origins)[0]
    pose[0, :3] = origin + torch.as_tensor(np.asarray(xyz, dtype=np.float32), device=pose.device)
    write_pose = getattr(obj, "write_root_link_pose_to_sim", None) or obj.write_root_pose_to_sim
    write_pose(pose)
    write_vel = getattr(obj, "write_root_com_velocity_to_sim", None) or obj.write_root_velocity_to_sim
    write_vel(torch.zeros((1, 6), device=pose.device))


# ---- wrapping the embodiment ----------------------------------------------------------------------------------------
def _wrap_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_stationhooks_wrapped", False):
        return
    step_name = next((n for n in STEP_METHOD_CANDIDATES if hasattr(cls, n)), None)
    if step_name is None or not hasattr(cls, RESET_METHOD):
        raise RuntimeError(f"stationhooks: {cls.__name__} has none of {STEP_METHOD_CANDIDATES} or no {RESET_METHOD}()")
    original_step, original_reset = getattr(cls, step_name), getattr(cls, RESET_METHOD)

    def stepped(self, *args, **kwargs):
        env = getattr(self, "_env", None)
        _state["env"] = env
        if env is not None:
            _state["step"] += 1
            for fn in _step_fns:
                fn(env, _state["step"])
        return original_step(self, *args, **kwargs)

    def reset(self, *args, **kwargs):
        result = original_reset(self, *args, **kwargs)
        env = getattr(self, "_env", None)
        _state["env"], _state["step"] = env, 0
        if env is not None:
            for fn in _reset_fns:
                fn(env)
        return result

    setattr(cls, step_name, stepped)
    setattr(cls, RESET_METHOD, reset)
    cls._stationhooks_wrapped = True


class _WrapAfterImport(importlib.abc.Loader):
    def __init__(self, inner, after):
        self.inner, self.after = inner, after

    def create_module(self, spec):
        return self.inner.create_module(spec)

    def exec_module(self, module):
        self.inner.exec_module(module)
        self.after(module)


class _ImportWatcher(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != EMBODIMENT_MODULE:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is not None and spec.loader is not None:
            spec.loader = _WrapAfterImport(spec.loader, _wrap_embodiment)
        return spec


def _install() -> None:
    if EMBODIMENT_MODULE in sys.modules:
        _wrap_embodiment(sys.modules[EMBODIMENT_MODULE])
    elif not any(isinstance(f, _ImportWatcher) for f in sys.meta_path):
        sys.meta_path.insert(0, _ImportWatcher())


_install()
