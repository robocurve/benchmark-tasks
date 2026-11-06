"""YAM task adapter: privileged evaluation and raw traces stay outside observations."""

from __future__ import annotations

import dataclasses
import json
import math
import uuid
from pathlib import Path
from typing import Any

import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from .evaluation_io import RUN_LIMIT_STEPS, evaluator_fingerprint, release_state


def encode_objects(objects: dict) -> dict:
    return {name: {field: np.asarray(getattr(state, field), dtype=float).tolist()
                   for field in ("pos", "quat_xyzw", "lin_vel", "ang_vel", "low", "high")}
            for name, state in objects.items()}


class SpacerExtractionEmbodiment(YamArenaEmbodiment):
    def __init__(self, *, trace_path: str | None = None, **kwargs: Any) -> None:
        if "task" in kwargs:
            raise ValueError("spacer_extraction_yam uses its packaged, frozen task file")
        for key, value in (("control_hz", 20), ("object_jitter", 0)):
            if key in kwargs and float(kwargs[key]) != value:
                raise ValueError(f"this fixed-layout task requires {key}={value}")
            kwargs[key] = value
        kwargs.setdefault("cam_width", 320)
        kwargs.setdefault("cam_height", 176)
        super().__init__(task=str(Path(__file__).with_name("task.py")), **kwargs)
        self._trace_path = Path(trace_path) if trace_path else Path("/tmp/spacer-extraction-traces") / f"{uuid.uuid4().hex}.jsonl"
        self._trace = None
        self._episode = 0
        self._tracker = self.task.module.SpacerTrialTracker()
        self._step_count = 0
        self._last_success = False

    def _write(self, record: dict) -> None:
        if self._trace is not None:
            self._trace.write(json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n")
            self._trace.flush()

    def reset(self, scene, *, seed: int | None = None):
        if self._trace is not None:
            self._trace.close()
        self._episode += 1
        path = self._trace_path
        if self._episode > 1:
            path = path.with_name(f"{path.stem}.episode{self._episode:03d}{path.suffix}")
        path.parent.mkdir(parents=True, exist_ok=True)
        self._trace = path.open("w", encoding="utf-8")
        observation = super().reset(scene, seed=seed)
        self._tracker.reset()
        self._step_count = 0
        self._last_success = False
        self._write({"type": "header", "schema": 1, "task_sha256": self.task.sha256,
                     "evaluator_sha256": evaluator_fingerprint(), "run_limit_steps": RUN_LIMIT_STEPS,
                     "seed": seed, "control_hz": self.info.control_hz, "physics_hz": 240,
                     "bases": self.rig.bases, "instruction": scene.instruction})
        from yam_arena.scene import object_states
        states = object_states(self._env, self.task.names)
        release = release_state(states, observation.state["joint_pos"], self.rig.bases)
        snapshot = self._tracker.update(states, time_s=0.0, released=release["released"])
        if float(snapshot.score) != 0.0:
            raise RuntimeError("reset unexpectedly starts with nonzero task score")
        self._record(states, observation, release, snapshot)
        return observation

    def _record(self, states, observation, release, snapshot) -> None:
        self._write({"type": "frame", "time_s": self._t,
                     "objects": encode_objects(states),
                     "joint_pos": np.asarray(observation.state["joint_pos"]).tolist(),
                     "release": release, "evaluation": dataclasses.asdict(snapshot)})

    def _task_step(self):
        from inspect_robots.types import StepResult
        from yam_arena.scene import object_states
        states = object_states(self._env, self.task.names)
        observation = self._observation()
        release = release_state(states, observation.state["joint_pos"], self.rig.bases)
        snapshot = self._tracker.update(states, time_s=self._t, released=release["released"])
        score = float(snapshot.score)
        if not math.isfinite(score) or not 0.0 <= score <= 1.0:
            raise ValueError("invalid task score")
        success = bool(snapshot.success_ready)
        self._step_count += 1
        if self._step_count > RUN_LIMIT_STEPS:
            raise RuntimeError("task received actions beyond its frozen run limit")
        self._last_success = success
        self._record(states, observation, release, snapshot)
        at_limit = self._step_count >= RUN_LIMIT_STEPS
        return StepResult(observation=observation, terminated=success or at_limit,
                          termination_reason="success" if success else "run_limit" if at_limit else None,
                          info={"success": success, "score": score})

    def close(self) -> None:
        if self._trace is not None:
            reason = "success" if self._last_success else "run_limit" if self._step_count >= RUN_LIMIT_STEPS else "incomplete"
            self._write({"type": "end", "time_s": self._t, "steps": self._step_count,
                         "reason": reason, "complete": reason != "incomplete"})
            self._trace.close()
            self._trace = None
        super().close()


def spacer_extraction_yam(**kwargs):
    return SpacerExtractionEmbodiment(**kwargs)


def spacer_extraction_demo(**kwargs):
    from inspect_robots.registry import resolve
    return resolve("policy", "yam_scripted", **kwargs)

