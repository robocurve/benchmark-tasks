"""trialkit: per-trial layouts, labelled boxes and staged demos for yam_arena task files. MIT licence.

Keep this file next to the task file and import it before declaring objects:

- ``on_reset(fn)``: ``fn(env, rng)`` runs after every reset, before the trial starts. ``rng`` is seeded from the
  trial's seed, so each trial can lay the objects out differently and a fixed eval seed repeats the same layouts.
- ``place(env, name, xyz)``: move a free object (keeping its orientation) and stop it.
- ``env()``: the running Isaac Lab environment, or None outside a simulation.
- ``top_image()``: the top camera's current RGB image (640 x 360). ``grippers()``: each gripper's TCP and opening.
- ``Labelled``: a Box with thin coloured panels fixed to it; the panels are visual only, the box is the collider.
- ``staged(stages)``: a scripted demo in stages. ``stages`` is a generator: it yields the first plan, then receives a
  ``View`` of the policy's observation each time a stage ends and yields the next plan. A ``View`` holds only what
  the policy observes (camera images, joint positions), so a stage is chosen from observations, not simulator state.
  Every arm a later stage moves needs a waypoint in the first stage.
"""

import importlib.abc
import importlib.machinery
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena import ik
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box

ARMS = ("left", "right")
_RESETS, _PANELS, _S = {}, {}, {"env": None, "gen": None}


@dataclass(frozen=True)
class Labelled(Box):
    """``panels``: ((dx, dy, dz), (sx, sy, sz), (r, g, b)) per panel, centre and size in the box's frame."""

    panels: tuple = ()

    def __post_init__(self):
        _PANELS[self.name] = self.panels


def on_reset(fn):
    _RESETS[fn.__qualname__] = fn
    return fn


def env():
    return _S["env"]


def place(environment, name, xyz):
    import torch

    base = environment.unwrapped
    obj = base.scene[name]
    pose = obj.data.default_root_pose.torch.clone()
    pose[:, :3] = torch.tensor(xyz, dtype=pose.dtype, device=pose.device) + base.scene.env_origins
    obj.write_root_pose_to_sim_index(root_pose=pose)
    obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))


def top_image():
    if _S["env"] is None:
        return None
    out = _S["env"].unwrapped.scene["top_cam"].data.output["rgb"]
    out = out.torch if hasattr(out, "torch") else out
    return out[0, ..., :3].cpu().numpy()


def grippers():
    """{arm: (TCP position, opening 0..1)} in the task frame, or {} outside a simulation."""
    if _S["env"] is None:
        return {}
    from yam_arena import runtime
    from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN

    base = _S["env"].unwrapped
    origin = base.scene.env_origins[0].cpu().numpy()
    out = {}
    for arm in ARMS:
        robot = base.scene[f"{arm}_robot"]
        ids = robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        opening = float(robot.data.joint_pos.torch[0, ids].mean().item()) / FINGER_OPEN
        out[arm] = (runtime.tcp_world(robot) - origin, opening)
    return out


class View:
    """What the scripted policy observes when a stage ends."""

    def __init__(self, observation, bases):
        self.images = observation.images
        self.joints = np.asarray(next(iter(observation.state.values())), dtype=float).reshape(-1)
        self.bases = bases

    def tcp(self, arm):
        q = self.joints[:7] if arm == "left" else self.joints[7:]
        return ik.tcp_pose(q[:6], self.bases[arm])[0]


def staged(stages):
    def demo():
        _S["gen"] = stages()
        return next(_S["gen"])

    return demo


def _patch_scene(scene):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils import clone

    @clone
    def spawn_labelled(prim_path, cfg, *args, **kwargs):
        from pxr import Gf, UsdGeom

        prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)
        stage = prim.GetStage()
        for i, (offset, size, color) in enumerate(_PANELS[prim_path.rsplit("/", 1)[-1]]):
            path = f"{prim_path}/panel_{i}"
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*map(float, offset)))
            cube.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
            look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(color), roughness=0.4)
            look.func(f"{path}_look", look)
            sim_utils.bind_visual_material(path, f"{path}_look", stage=stage)
        return prim

    base_object = scene.task_object

    def task_object(spec):
        obj = base_object(spec)
        if spec.name in _PANELS:
            obj.spawner_cfg.func = obj.object_cfg.spawn.func = spawn_labelled
        return obj

    scene.task_object = task_object


def _patch_embodiment(mod):
    cls = mod.YamArenaEmbodiment
    base_reset = cls.reset

    def reset(self, scene, *, seed=None):
        obs = base_reset(self, scene, seed=seed)
        _S["env"] = self._env
        if not _RESETS:
            return obs
        rng = np.random.default_rng(seed)
        for fn in _RESETS.values():
            fn(self._env, rng)
        for _ in range(max(1, round(mod.SETTLE_S / self._base.step_dt))):
            self._env.step(self._ready)
        for _ in range(mod.RESET_RENDERS):
            self._base.sim.render()
        return self._observation()

    cls.reset = reset


def _patch_scripted(mod):
    cls = mod.YamScriptedPolicy
    base_act = cls.act

    def act(self, observation):
        t = float(observation.state_time) + self._dt
        if _S["gen"] is not None and t > self._end:
            try:
                plan = _S["gen"].send(View(observation, self._bases))
            except StopIteration:
                _S["gen"] = None
            else:
                for arm, waypoints in plan.items():
                    p = self._plans[arm]
                    if self._end - p.duration > 1e-6:
                        w = p.waypoints[-1]
                        p.waypoints.append(Waypoint(self._end - p.duration, w.pos, w.finger, "wait", w.yaw))
                    p.waypoints.extend(waypoints)
                self._end = max(p.duration for p in self._plans.values())
        return base_act(self, observation)

    cls.act = act


_PATCHES = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment,
            "yam_arena.scripted": _patch_scripted}


class _AfterImport(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path, target=None):
        if name not in _PATCHES:
            return None
        spec = importlib.machinery.PathFinder.find_spec(name, path)
        if spec is not None and spec.loader is not None:
            run = spec.loader.exec_module

            def exec_module(module):
                run(module)
                _PATCHES[name](module)

            spec.loader.exec_module = exec_module
        return spec


if not getattr(sys, "_trialkit", False):
    sys._trialkit = True
    for _name, _patch in _PATCHES.items():
        if _name in sys.modules:
            _patch(sys.modules[_name])
    sys.meta_path.insert(0, _AfterImport())
