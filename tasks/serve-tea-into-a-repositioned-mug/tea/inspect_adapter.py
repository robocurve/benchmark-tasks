"""Inspect Robots' simulated embodiment; policy observations are three RGB images and 14 joint coordinates."""

import gzip
import json
import os
from pathlib import Path

import numpy as np
from inspect_robots.embodiment import (
    PRIVILEGED_SUCCESS,
    RESETTABLE,
    EmbodimentBase,
    EmbodimentInfo,
)
from inspect_robots.spaces import (
    ActionSemantics,
    Box,
    CameraSpec,
    ObservationSpace,
    StateField,
    StateSpec,
)
from inspect_robots.types import Action, ActionChunk, Observation, StepResult
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.scorer import Score
from .model import CAMS, Kitchen
from .render import Camera
from .run import revision

INSTRUCTION = "Pour my tea to the line on my mug and put the pitcher down."
LABELS = tuple(
    f"{s}_{part}"
    for s in ("left", "right")
    for part in [*(f"j{i}" for i in range(6)), "gripper"]
)


def action_space(limits=None):
    low, high = (None, None) if limits is None else limits
    return Box(
        (14,),
        low=low,
        high=high,
        semantics=ActionSemantics(
            control_mode="joint_pos",
            gripper="continuous",
            frame="base",
            dim_labels=LABELS,
        ),
    )


class Embodiment(EmbodimentBase):
    def __init__(self, profile="reposition", cam_height=224, cam_width=224):
        self.profile = str(profile)
        self.height = int(cam_height)
        self.width = int(cam_width)
        self.plant = Kitchen(self.profile)
        self.camera = None
        self.trace = None
        self.revision = revision()
        bounds = self.plant.m.jnt_range[:6]
        limits = tuple(np.tile(np.r_[bounds[:, i], i], 2) for i in range(2))
        self.info = EmbodimentInfo(
            name="yam_tea_service",
            morphology="yam-bimanual",
            action_space=action_space(limits),
            observation_space=ObservationSpace(
                cameras=tuple(CameraSpec(n, self.height, self.width) for n in CAMS),
                state_keys=frozenset(["joint_pos"]),
                state=StateSpec((StateField("joint_pos", (14,), "rad+normalized"),)),
            ),
            control_hz=20.0,
            is_simulated=True,
            capabilities=frozenset([PRIVILEGED_SUCCESS, RESETTABLE]),
            docs="Two YAM arms on a kitchen counter. Absolute joint targets, then gripper opening from 0 to 1, for each arm. A resident holds a clear mug with a serving line. A handled pitcher contains tea.",
            environment_id="yam-household-tea-mujoco",
            environment_revision=self.revision,
        )

    def obs(self):
        e = self.plant
        return Observation(
            images={n: self.camera.frame(n) for n in CAMS},
            state={"joint_pos": e.state()},
            instruction=self.instruction,
            image_times={n: e.d.time for n in CAMS},
            state_time=e.d.time,
        )

    def reset(self, scene, *, seed=None):
        self.close()
        self.plant = Kitchen(self.profile)
        self.camera = Camera(self.plant, self.width, self.height)
        self.instruction = scene.instruction or INSTRUCTION
        self.steps = 0
        path = os.environ.get("TEA_OBSERVATION_TRACE")
        if path:
            self.trace = gzip.open(path, "xt")
            self.trace.write(
                json.dumps(
                    dict(
                        kind="reset",
                        source_sha256=self.revision,
                        profile=self.profile,
                        seed=seed,
                    )
                )
                + "\n"
            )
        return self.obs()

    def step(self, action):
        a = np.asarray(action.data, dtype=float)
        self.plant.action(a)
        self.steps += 1
        r = self.plant.report()
        if self.trace:
            self.trace.write(
                json.dumps(dict(kind="step", tick=self.steps, action=a.tolist(), **r))
                + "\n"
            )
        return StepResult(
            observation=self.obs(),
            terminated=r["score"] == 1.0,
            termination_reason="success" if r["score"] == 1.0 else None,
            info={"success": r["score"] == 1.0, "score": r["score"]},
        )

    def close(self):
        if self.camera:
            self.camera.close()
            self.camera = None
        if self.trace:
            self.trace.close()
            self.trace = None


class Replay(PolicyBase):
    def __init__(self, actions):
        self.actions = np.load(Path(actions), allow_pickle=False)
        if self.actions.ndim != 2 or self.actions.shape[1] != 14:
            raise ValueError("Expected rows of 14 commands")
        self.info = PolicyInfo(name="tea_replay", action_space=action_space())

    def reset(self, scene):
        self.i = 0

    def act(self, observation):
        a = self.actions[min(self.i, len(self.actions) - 1)]
        self.i += 1
        return ActionChunk(actions=[Action(data=a)], control_hz=20.0)


class TerminalScore:
    name = "tea_outcome"

    def __call__(self, record, target):
        score = float(record.steps[-1].result.info["score"]) if record.steps else 0.0
        return Score(
            value=score,
            explanation="Terminal collected masses, two-second visible dwell, pitcher placement and fixture witnesses.",
        )
