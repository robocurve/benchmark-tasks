"""Inspect Robots YAM embodiment. Free utensils interact through articulated elastic bodies and contact."""

import os

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MUJOCO_EGL_DEVICE_ID", os.environ.get("ROBOCURVE_GPU", "0"))

import mujoco
import numpy as np
from inspect_robots import Observation
from inspect_robots.embodiment import (
    EmbodimentBase,
    EmbodimentInfo,
    PRIVILEGED_SUCCESS,
    RESETTABLE,
    SEEDABLE,
)
from inspect_robots.types import StepResult
from inspect_robots_yam.config import DEFAULT_CAMERAS, action_box, observation_space
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN

from scene import build
from scoring import evaluate, step_score

HOME_POSE = [0.0, 0.65, 1.2, -0.3, 0.0, 0.0]
HZ, LIMIT = 20, 1000
INSTRUCTION = "Separate the whisk, wooden spoon, and spatula, then put them in the holder so I can take any one out on its own."


class KitchenUtensils(EmbodimentBase):
    def __init__(self, cameras=True):
        self.model = mujoco.MjModel.from_xml_string(build())
        self.data = mujoco.MjData(self.model)
        self.cameras = cameras
        self.renderers = {}
        self.steps_per_tick = round(1 / HZ / self.model.opt.timestep)
        self.arm_joints = np.array(
            [
                [self.model.joint(f"{s}_joint{i}").qposadr[0] for i in range(1, 9)]
                for s in ("left", "right")
            ]
        )
        self.arm_drives = np.array(
            [[self.model.actuator(f"{s}_joint{i}").id for i in range(1, 9)] for s in ("left", "right")]
        )
        self.low = np.tile(np.r_[ik.LOWER, 0], 2)
        self.high = np.tile(np.r_[ik.UPPER, 1], 2)
        self.info = EmbodimentInfo(
            name="utensils_yam",
            action_space=action_box(self.low, self.high),
            observation_space=observation_space(176, 320, DEFAULT_CAMERAS),
            control_hz=HZ,
            is_simulated=True,
            capabilities=frozenset({RESETTABLE, SEEDABLE, PRIVILEGED_SUCCESS}),
            environment_id="untangle-kitchen-utensils",
        )
        self.opt = mujoco.MjvOption()
        self.opt.geomgroup[3] = 0
        self.opt.sitegroup[:] = 0

    def reset(self, scene, *, seed=None):
        mujoco.mj_resetData(self.model, self.data)
        self.instruction = scene.instruction or INSTRUCTION
        arm = np.r_[HOME_POSE, FINGER_OPEN, FINGER_OPEN]
        self.data.qpos[self.arm_joints] = arm
        self.data.ctrl[self.arm_drives] = arm
        mujoco.mj_forward(self.model, self.data)
        settle_seconds = 1.0
        mujoco.mj_step(self.model, self.data, nstep=round(settle_seconds / self.model.opt.timestep))
        self.steps = 0
        self.metrics = {}
        return self.observe()

    def render(self, name, width=320, height=176, *, data=None):
        if (width, height) not in self.renderers:
            self.renderers[width, height] = mujoco.Renderer(self.model, height, width)
        renderer = self.renderers[width, height]
        renderer.update_scene(self.data if data is None else data, camera=name, scene_option=self.opt)
        return renderer.render().copy()

    def observe(self):
        joints = self.data.qpos[self.arm_joints]
        opening = np.clip(joints[:, 6:].mean(axis=1) / FINGER_OPEN, 0, 1)
        state = np.column_stack((joints[:, :6], opening)).ravel()
        images = {n: self.render(n) for n in DEFAULT_CAMERAS} if self.cameras else {}
        return Observation(
            images=images,
            state={"joint_pos": state},
            instruction=self.instruction,
            state_time=self.steps / HZ,
            image_times={n: self.steps / HZ for n in images},
        )

    def step(self, action):
        target = np.clip(np.asarray(action.data, dtype=float), self.low, self.high).reshape(2, 7)
        self.data.ctrl[self.arm_drives[:, :6]] = target[:, :6]
        self.data.ctrl[self.arm_drives[:, 6:]] = target[:, 6, None] * FINGER_OPEN
        mujoco.mj_step(self.model, self.data, nstep=self.steps_per_tick)
        self.steps += 1
        ended = self.steps >= LIMIT
        if ended:
            self.metrics = evaluate(self.model, self.data)
            score = self.metrics["score"]
        else:
            score = step_score(self.model, self.data)
        info = {"success": score == 1.0, "score": score}
        return StepResult(
            observation=self.observe(),
            terminated=ended,
            termination_reason="action_limit" if ended else None,
            info=info,
        )

    def close(self):
        for renderer in self.renderers.values():
            renderer.close()
        self.renderers.clear()
