"""Inspect Robots adapters: the MuJoCo YAM embodiment and the scripted oracle policy.

Action contract matches Robocurve's ``yam_arms``: 14-D absolute ``joint_pos``,
``[left j1..j6, left gripper, right j1..j6, right gripper]``, at 10 Hz.
The gripper value is the finger slide in metres (0 closed, 0.0469 open).
Every step's ``info`` carries the task's full rubric so scorers need no sim access.
"""

from __future__ import annotations

from collections.abc import Iterator
from typing import Any

import numpy as np
from inspect_robots import (
    Action,
    ActionChunk,
    ActionSemantics,
    Box,
    CameraSpec,
    EmbodimentInfo,
    Observation,
    ObservationSpace,
    PolicyConfig,
    PolicyInfo,
    Scene,
    StepResult,
)
from inspect_robots.embodiment import PRIVILEGED_SUCCESS, RENDERABLE, SEEDABLE

from robosci import scene as sc
from robosci.env import SimEnv
from robosci.tasks import POLICIES
from robosci.tasks import TASKS

def _action_space() -> Box:
    """Joint limits straight from the robot model, gripper slide 0..open."""
    lo, hi = [], []
    model = sc.base_spec().compile()
    for s in sc.SIDES:
        for jn in sc.ARM_JOINTS[s]:
            r = model.jnt_range[model.joint(jn).id]
            lo.append(r[0])
            hi.append(r[1])
        lo.append(0.0)
        hi.append(sc.GRIPPER_OPEN)
    labels = tuple(f"{s}_{j}" for s in sc.SIDES for j in [*(f"joint{i}" for i in range(1, 7)), "gripper"])
    return Box(shape=(14,), low=np.array(lo), high=np.array(hi),
               semantics=ActionSemantics(control_mode="joint_pos", gripper="continuous",
                                         frame="joint", dim_labels=labels))


_W, _H = 640, 360
# Live envs, so the privileged oracle policy can find the sim it is driving.
_LIVE: dict[int, SimEnv] = {}


class RoboSciEmbodiment:
    """Two YAM arms in MuJoCo; the scene's metadata names the robosci task."""

    def __init__(self, *, difficulty: str = "easy") -> None:
        self.difficulty = difficulty
        self.env: SimEnv | None = None
        self.info = EmbodimentInfo(
            name="robosci_mujoco",
            action_space=_action_space(),
            observation_space=ObservationSpace(
                cameras=tuple(CameraSpec(name=c, height=_H, width=_W, channels=3) for c in sc.CAMERAS),
                state_keys=frozenset({"joint_pos", "sim_handle"}),
            ),
            control_hz=sc.CONTROL_HZ,
            is_simulated=True,
            capabilities=frozenset({SEEDABLE, RENDERABLE, PRIVILEGED_SUCCESS}),
            docs=("Two i2RT YAM arms facing +x; left base at y=0, right at y=-0.61 m. "
                  "Action: absolute joint radians per arm plus gripper slide in metres "
                  "(0 closed, 0.0469 open, 9.4 cm between pads). Cameras: top, left_wrist, right_wrist."),
        )

    def reset(self, scene: Scene, *, seed: int | None = None) -> Observation:
        if self.env is not None:
            _LIVE.pop(id(self.env), None)
            self.env.close()
        key = scene.metadata["task"]
        self.env = SimEnv(TASKS[key](difficulty=scene.metadata.get("difficulty", self.difficulty)),
                          seed=seed if seed is not None else 0, width=_W, height=_H)
        self.env.settle(0.5)
        self._instruction = scene.instruction
        _LIVE[id(self.env)] = self.env
        return self._observe()

    def step(self, action: Action) -> StepResult:
        assert self.env is not None, "reset() first"
        self.env.step(np.asarray(action.data, dtype=float))
        info: dict[str, Any] = self.env.score()
        # End the episode only at the top rubric stage: a lucky correct placement before
        # the experiment has proven the answer must not cut the experiment short.
        done = int(info["stage"]) >= 4
        return StepResult(observation=self._observe(), reward=float(info["stage"]),
                          terminated=done, termination_reason="success" if done else None,
                          truncated=False, info=info)

    def close(self) -> None:
        if self.env is not None:
            _LIVE.pop(id(self.env), None)
            self.env.close()
            self.env = None

    def _observe(self) -> Observation:
        assert self.env is not None
        obs = self.env.observe()
        return Observation(images={c: obs[c] for c in sc.CAMERAS},
                           state={"joint_pos": obs["joint_pos"],
                                  "sim_handle": np.array([id(self.env)], dtype=np.int64)},
                           instruction=self._instruction)


class RoboSciOraclePolicy:
    """Scripted reference solution; reads privileged sim state via ``sim_handle``."""

    def __init__(self, *, chunk_size: int = 1, which: str = "oracle") -> None:
        # chunk_size 1: scripted policies read live sim state when computing each action,
        # so pre-computing a chunk would act on stale observations.

        self.chunk_size = chunk_size
        self.which = which
        self.info = PolicyInfo(
            name=f"robosci_{which}",
            action_space=Box(shape=(14,), semantics=ActionSemantics(control_mode="joint_pos",
                                                                    gripper="continuous", frame="joint")),
            observation_space=ObservationSpace(state_keys=frozenset({"sim_handle"})),
        )
        self.config = PolicyConfig(action_horizon=chunk_size)
        self._gen: Iterator[np.ndarray] | None = None
        self._last: np.ndarray | None = None

    def reset(self, scene: Scene) -> None:
        self._gen, self._last = None, None
        self._task = scene.metadata["task"]

    def act(self, observation: Observation) -> ActionChunk:
        if self._gen is None:
            env = _LIVE[int(observation.state["sim_handle"][0])]
            self._gen = POLICIES[self._task][self.which](env, np.random.default_rng(env.seed))
        acts = []
        for _ in range(self.chunk_size):
            a = next(self._gen, None)
            if a is None:
                a = self._last if self._last is not None else np.asarray(observation.state.get("joint_pos"))
            self._last = a
            acts.append(Action(data=np.asarray(a, dtype=float).copy()))
        return ActionChunk(actions=acts)


def make_embodiment(**kw: Any) -> RoboSciEmbodiment:
    return RoboSciEmbodiment(**kw)


def make_oracle_policy(**kw: Any) -> RoboSciOraclePolicy:
    return RoboSciOraclePolicy(**kw)


def make_guess_policy(**kw: Any) -> RoboSciOraclePolicy:
    return RoboSciOraclePolicy(which="guess", **kw)
