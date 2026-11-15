"""A ``yam_arena`` embodiment with task-built mechanism bodies and native joints (reusable kit module).

Subclass :class:`MechanismEmbodiment` and set:

- ``ENV_NAME``: the Arena environment name;
- ``builders()``: {part name: () -> USD path}, every part a rigid body (kinematic for fixed frames);
- ``layout()``: {part name: start pose (xyz + quat xyzw, world, env 0)}, consistent with every joint;
- ``make_joints(js)``: define joints on a :class:`~.joints.JointSet` (created disabled);
- ``measure()``: the per-step measured state and score (returns ``info``, ``done``).

The robot, cameras, 14-D joint action and observations are the stock adapter's, unchanged. Reset order:
joints off -> stock reset (Arena writes every part's initial pose, arms to READY, settle) -> joints on -> the start
layout written again and settled. Nothing is teleported after that; parts move only through contact and joints.

Adapted from the ratchet-lowering task plugin (robocurve/benchmark-tasks PR #538, MIT, Copyright (c) 2026 YAMbition
contributors; full notice in THIRD_PARTY_NOTICES.txt).
"""

from __future__ import annotations

import atexit
import json
from pathlib import Path

import numpy as np
from inspect_robots.types import StepResult
from yam_arena import inspect_embodiment as upstream
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN, READY
from yam_arena.inspect_embodiment import ARMS, YamArenaEmbodiment

from .joints import JointSet

ACTIVE = None  # the live embodiment; the demo policies do not read it (they use only joint angles)


class MechanismEmbodiment(YamArenaEmbodiment):
    ENV_NAME = "yam_mechanism"
    TASK_FILE: Path | None = None
    SETTLE_S = 0.5

    def __init__(self, telemetry=None, **kwargs):
        global ACTIVE
        if self.TASK_FILE is not None:
            kwargs.setdefault("task", str(self.TASK_FILE))
        kwargs.setdefault("object_jitter", 0)
        super().__init__(**kwargs)
        self.telemetry = Path(telemetry).expanduser() if telemetry else None
        self._trace = None
        self.js: JointSet | None = None
        self.start = self.layout()
        ACTIVE = self

    # -- to override -----------------------------------------------------------------------
    def builders(self) -> dict:
        raise NotImplementedError

    def layout(self) -> dict[str, np.ndarray]:
        raise NotImplementedError

    def make_joints(self, js: JointSet, origin: np.ndarray) -> None:
        raise NotImplementedError

    def on_reset(self) -> None:
        """After the start layout is restored: reset the scorer."""

    def configure_robot(self, robot):
        """Add task sensors before the environment starts."""

    def measure(self) -> tuple[dict, bool]:
        return {}, False

    # -- simulator --------------------------------------------------------------------------
    def _ensure_sim(self, goal):
        if self._env is not None:
            return
        if upstream._APP is None:
            from isaaclab.app import AppLauncher

            upstream._APP = AppLauncher(headless=self.headless, device=self.device, enable_cameras=True).app
            atexit.register(upstream._close_app_at_exit)
        import torch
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
        from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
        from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
        from isaaclab_arena.scene.scene import Scene
        from isaaclab_arena.utils.pose import Pose
        from yam_arena.embodiment import BimanualYamEmbodiment
        from yam_arena.scene import env_cfg_callback, task_object, task_room

        robot = BimanualYamEmbodiment(enable_cameras=True, rig=self.rig, wrist_cam_mount=self.wrist_cam_mount,
                                      contact_sensors=False)
        self.configure_robot(robot)
        objects = []
        for name, build in self.builders().items():
            pose = self.start[name]
            objects.append(Object(name=name, object_type=ObjectType.RIGID, usd_path=str(build()),
                                  initial_pose=Pose(position_xyz=tuple(map(float, pose[:3])),
                                                    rotation_xyzw=tuple(map(float, pose[3:])))))
        objects.extend(task_object(o) for o in self.task.objects)
        for obj in objects:
            obj.initial_pose = Pose.from_dict(obj.initial_pose.to_dict())
            obj.enable_reset_pose()
        environment = IsaacLabArenaEnvironment(
            name=self.ENV_NAME, scene=Scene([*task_room("station", self.rig.top_cam), *objects]),
            embodiment=robot, env_cfg_callback=env_cfg_callback(self.info.control_hz))
        self._env = ArenaEnvBuilder(environment, ArenaEnvBuilderCfg(
            num_envs=1, solve_relations=False, device=self.device)).make_registered()
        self._base = self._env.unwrapped
        self._torch, self._goal = torch, goal
        self._free_objects = self.task.free_objects
        self._ready = torch.tensor([[*READY, 1.0, *READY, 1.0]], dtype=torch.float32, device=self._base.device)
        self._ids = {n: (self._base.scene[n].find_joints(ARM_JOINTS, preserve_order=True)[0],
                         self._base.scene[n].find_joints(FINGER_JOINTS, preserve_order=True)[0]) for n in ARMS}
        self.js = JointSet(self._base.sim.stage)
        self.make_joints(self.js, self.origin())

    def origin(self) -> np.ndarray:
        return self._base.scene.env_origins[0].cpu().numpy().astype(float)

    def reset(self, scene, *, seed=None):
        if self.telemetry and self._trace is None:
            self.telemetry.parent.mkdir(parents=True, exist_ok=True)
            self._trace = self.telemetry.open("a", buffering=1)
        if self._env is None:
            self._ensure_sim(None)
        self.js.enable(False)
        super().reset(scene, seed=seed)
        self.js.enable(True)
        self.restore_layout()
        self.on_reset()
        self._t = 0.0
        if self._trace:
            self._trace.write(json.dumps({"reset": True, "poses": {k: v.round(6).tolist()
                                                                   for k, v in self.poses().items()}}) + "\n")
        return self._observation()

    def restore_layout(self, repeats: int = 2) -> None:
        torch, base = self._torch, self._base
        for _ in range(repeats):
            for name, pose in self.start.items():
                p = torch.tensor(pose[None], dtype=torch.float32, device=base.device)
                p[:, :3] += base.scene.env_origins
                base.scene[name].write_root_pose_to_sim_index(root_pose=p)
                base.scene[name].write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=base.device))
            for _ in range(max(1, round(self.SETTLE_S / base.step_dt))):
                self._env.step(self._ready)
        for _ in range(upstream.RESET_RENDERS):
            base.sim.render()

    # -- measured state -----------------------------------------------------------------------
    def poses(self) -> dict[str, np.ndarray]:
        """Every part's pose (xyz relative to the env origin + quat xyzw)."""
        o = self.origin()
        out = {}
        for n in self.start:
            p = self._base.scene[n].data.root_pose_w.torch[0].cpu().numpy().astype(float)
            p[:3] -= o
            out[n] = p
        return out

    def velocity(self, name: str) -> np.ndarray:
        """(linear xyz, angular xyz) world velocity of a part."""
        data = self._base.scene[name].data
        lin = data.root_lin_vel_w.torch[0].cpu().numpy().astype(float)
        ang = data.root_ang_vel_w.torch[0].cpu().numpy().astype(float)
        return np.r_[lin, ang]

    def grippers(self) -> tuple[list[float], list[np.ndarray]]:
        """Each arm's gripper opening (0 closed .. 1 open) and TCP position (env frame), from measured joints."""
        from yam_arena import ik, station

        opening, tcp = [], []
        for name, side in zip(ARMS, ("left", "right")):
            arm_ids, finger_ids = self._ids[name]
            q = self._base.scene[name].data.joint_pos.torch[0]
            opening.append(float(np.clip(q[finger_ids].mean().item() / FINGER_OPEN, 0.0, 1.0)))
            tcp.append(ik.tcp_pose(q[arm_ids].cpu().numpy().astype(float), station.BASES[side])[0])
        return opening, tcp

    def _task_step(self):
        info, done = self.measure()
        if self._trace:
            self._trace.write(json.dumps({"t": round(self._t, 3), **info}, default=_jsonable) + "\n")
        return StepResult(observation=self._observation(), terminated=done,
                          termination_reason="success" if done else None, info=info)

    def close(self):
        if self._trace:
            self._trace.close()
            self._trace = None
        super().close()


def _jsonable(v):
    if isinstance(v, np.ndarray):
        return v.round(6).tolist()
    if isinstance(v, (np.floating, np.integer, np.bool_)):
        return v.item()
    return str(v)
