"""Scorer wrapper for squeeze-in-the-last-slab."""

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
        s1_pos: np.ndarray, s1_quat: np.ndarray,
        s2_pos: np.ndarray, s2_quat: np.ndarray,
        s3_pos: np.ndarray, s3_quat: np.ndarray,
        s4_pos: np.ndarray, s4_quat: np.ndarray,
        s5_pos: np.ndarray, s5_quat: np.ndarray,
        arm_tcps: dict | None = None,
    ) -> logic.EvalResult:
        return self.logic.update(
            s1_pos=s1_pos, s1_quat=s1_quat,
            s2_pos=s2_pos, s2_quat=s2_quat,
            s3_pos=s3_pos, s3_quat=s3_quat,
            s4_pos=s4_pos, s4_quat=s4_quat,
            s5_pos=s5_pos, s5_quat=s5_quat,
            arm_tcps=arm_tcps,
        )

