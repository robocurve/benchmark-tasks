"""Scoring ladder and verification for stack-crates-largest-first. Pure Python + NumPy.

Ladder:
  0.00: Initial / nothing correctly placed.
  0.25: Large crate resting on pallet mark (within 15 mm).
  0.50: Medium crate resting on Large crate (within 12 mm).
  0.75: Small crate resting on Medium crate (within 12 mm).
  1.00: Complete stack stable for 2.0 s (40 consecutive steps at 20 Hz):
        all 3 offsets <= 12 mm, tilt < 5 deg, both arms clear (> 60 mm).

Zero-gates:
  - Floor fault: any crate falling off the table (z < TOP - 0.05 m) latches score to 0.0.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping
import numpy as np

import geometry as g
import logic


@dataclass(frozen=True)
class ScoreResult:
    score: float
    success: bool
    details: dict


class Scorer:
    """Stateful scorer evaluating palletising stack ladder and zero-gates."""

    def __init__(self, hold_s: float = 2.0, hz: float = 20.0) -> None:
        self.hold_steps = int(round(hold_s * hz))
        self.dt = 1.0 / hz
        self.logic = logic.StackLogic()
        self.reset()

    def reset(self) -> None:
        self.logic = logic.StackLogic()
        self.consecutive_success_steps = 0
        self.best_score = 0.0
        self.latched_floor_fault = False

    def update(
        self,
        large_pos: tuple[float, float, float] | np.ndarray,
        large_quat: tuple[float, float, float, float] | np.ndarray = (0.0, 0.0, 0.0, 1.0),
        large_vel: tuple[float, float, float] | np.ndarray = (0.0, 0.0, 0.0),
        medium_pos: tuple[float, float, float] | np.ndarray = (0.0, 0.0, 0.0),
        medium_quat: tuple[float, float, float, float] | np.ndarray = (0.0, 0.0, 0.0, 1.0),
        medium_vel: tuple[float, float, float] | np.ndarray = (0.0, 0.0, 0.0),
        small_pos: tuple[float, float, float] | np.ndarray = (0.0, 0.0, 0.0),
        small_quat: tuple[float, float, float, float] | np.ndarray = (0.0, 0.0, 0.0, 1.0),
        small_vel: tuple[float, float, float] | np.ndarray = (0.0, 0.0, 0.0),
        arm_tcps: Mapping[str, tuple[float, float, float] | np.ndarray] | None = None,
    ) -> ScoreResult:
        st = self.logic.step(
            dt=self.dt,
            large_pos=large_pos,
            large_quat=large_quat,
            large_vel=large_vel,
            medium_pos=medium_pos,
            medium_quat=medium_quat,
            medium_vel=medium_vel,
            small_pos=small_pos,
            small_quat=small_quat,
            small_vel=small_vel,
            arm_tcps=arm_tcps,
        )

        if st.floor_fault:
            self.latched_floor_fault = True
            return ScoreResult(score=0.0, success=False, details={"floor_fault": True})

        if self.latched_floor_fault:
            return ScoreResult(score=0.0, success=False, details={"floor_fault": True})

        score = 0.0
        success = False

        if st.large_on_pallet:
            score = max(score, 0.25)

        if st.medium_on_large:
            score = max(score, 0.50)

        if st.small_on_medium:
            score = max(score, 0.75)

        if st.solved:
            self.consecutive_success_steps += 1
            if self.consecutive_success_steps >= self.hold_steps:
                score = 1.00
                success = True
        else:
            self.consecutive_success_steps = 0

        self.best_score = max(self.best_score, score)

        out_details = dict(st.details)
        out_details["consecutive_steps"] = self.consecutive_success_steps
        out_details["best_score"] = self.best_score

        return ScoreResult(
            score=self.best_score,
            success=success,
            details=out_details,
        )

