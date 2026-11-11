"""Scoring definitions and evaluation glue for clear-lane-then-push-block-to-dock."""

from __future__ import annotations

import numpy as np

import geometry as g
from logic import EvalResult, TaskLogic


LADDER = [
    (0.25, "Both loose blocks cleared out of the lane (|y| > 0.075 m)"),
    (0.50, "Cargo block moved 80 mm along x without loose block in the lane"),
    (0.75, "Cargo block at dock (within 25 mm of mark center x=0.340 m)"),
    (1.00, "Cargo block on mark within 12 mm, yaw within 10 deg, red post undisturbed, held 2 s, arms clear"),
]

ZERO_GATES = [
    ("post_hit", 0.25, "Red post tilted > 10 degrees or moved > 10 mm caps score at 0.25"),
]


class Scorer:
    def __init__(self, hold_s: float = g.HOLD_TIME_S, hz: float = 20.0):
        self.logic = TaskLogic(hold_s=hold_s, hz=hz)

    def reset(self) -> None:
        self.logic.reset()

    def step(
        self,
        cargo_pos: np.ndarray,
        cargo_quat: np.ndarray,
        blue_pos: np.ndarray,
        yellow_pos: np.ndarray,
        post_pos: np.ndarray,
        post_quat: np.ndarray,
        arm_tcps: dict | None = None,
    ) -> EvalResult:
        return self.logic.update(
            cargo_pos=cargo_pos,
            cargo_quat=cargo_quat,
            blue_pos=blue_pos,
            yellow_pos=yellow_pos,
            post_pos=post_pos,
            post_quat=post_quat,
            arm_tcps=arm_tcps,
        )

