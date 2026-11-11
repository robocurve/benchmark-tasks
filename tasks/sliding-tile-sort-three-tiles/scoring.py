"""Scorer wrapper for sliding-tile-sort-three-tiles."""

from __future__ import annotations

import numpy as np

import geometry as g
import logic


class Scorer:
    def __init__(self, hold_s: float = g.HOLD_TIME_S, hz: float = 20.0):
        self.logic = logic.TaskLogic(hold_s=hold_s, hz=hz)

    def reset(self) -> None:
        self.logic.reset()

    def update(
        self,
        blue_pos: np.ndarray,
        blue_quat: np.ndarray,
        blue_vel: np.ndarray,
        orange_pos: np.ndarray,
        orange_quat: np.ndarray,
        orange_vel: np.ndarray,
        yellow_pos: np.ndarray,
        yellow_quat: np.ndarray,
        yellow_vel: np.ndarray,
        arm_tcps: dict | None = None,
    ) -> logic.EvalResult:
        return self.logic.update(
            blue_pos=blue_pos,
            blue_quat=blue_quat,
            blue_vel=blue_vel,
            orange_pos=orange_pos,
            orange_quat=orange_quat,
            orange_vel=orange_vel,
            yellow_pos=yellow_pos,
            yellow_quat=yellow_quat,
            yellow_vel=yellow_vel,
            arm_tcps=arm_tcps,
        )

