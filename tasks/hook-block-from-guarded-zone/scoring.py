"""Scoring ladder and verification for hook-block-from-guarded-zone. Pure Python + NumPy.

Ladder:
  0.00: Initial / invalid state.
  0.25: Hooked rod held / lifted by arm.
  0.50: Block moved >= 60 mm along -X (x <= 0.52).
  0.75: Block outside guarded zone (x < 0.38 resting).
  1.00: Block resting in green tray, no fault, rod set down outside zone,
        both arms clear, held stable for 2.0 s (40 steps at 20 Hz).

Zero-gates:
  - Any arm TCP or gripper body crossing past yellow line (x > 0.41 m)
    permanently latches FAULT = score 0.0.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence
import numpy as np

import geometry as g
import logic


@dataclass(frozen=True)
class ScoreResult:
    score: float
    success: bool
    details: dict


class Scorer:
    """Stateful scorer evaluating hazardous zone retrieval ladder and boundary safety."""

    def __init__(self, hold_s: float = g.HOLD_TIME_S, rest_s: float = g.REST_TIME_S, hz: float = 20.0) -> None:
        self.dt = 1.0 / hz
        self.logic = logic.HookBlockLogic(hold_s=hold_s, rest_s=rest_s)
        self.reset()

    def reset(self) -> None:
        self.logic.reset()
        self.best_score = 0.0

    def update(
        self,
        block_pos: tuple[float, float, float] | np.ndarray | Sequence[float],
        rod_pos: tuple[float, float, float] | np.ndarray | Sequence[float],
        block_vel: tuple[float, float, float] | np.ndarray | Sequence[float] = (0.0, 0.0, 0.0),
        rod_vel: tuple[float, float, float] | np.ndarray | Sequence[float] = (0.0, 0.0, 0.0),
        arm_tcps: Mapping[str, tuple[float, float, float] | np.ndarray | Sequence[float]] | None = None,
        gripper_poses: Mapping[str, Any] | None = None,
    ) -> ScoreResult:
        st = self.logic.step(
            dt=self.dt,
            block_pos=block_pos,
            rod_pos=rod_pos,
            block_vel=block_vel,
            rod_vel=rod_vel,
            arm_tcps=arm_tcps,
            gripper_poses=gripper_poses,
        )

        dbg_details = {
            "dbg_fault": st.fault,
            "dbg_fault_reason": st.fault_reason,
            "dbg_rod_held": st.rod_held,
            "dbg_block_moved_60mm": st.block_moved_60mm,
            "dbg_block_outside_zone": st.block_outside_zone,
            "dbg_block_in_tray": st.block_in_tray,
            "dbg_block_pos": [round(float(x), 4) for x in block_pos[:3]],
            "dbg_rod_pos": [round(float(x), 4) for x in rod_pos[:3]],
            "dbg_rod_outside_zone": st.rod_outside_zone,
            "dbg_arms_clear": st.arms_clear,
            "dbg_tray_hold_time": round(st.tray_hold_time, 2),
            "dbg_solved": st.solved,
        }

        if arm_tcps is not None:
            if "left" in arm_tcps:
                dbg_details["dbg_left_tcp"] = [round(float(x), 4) for x in arm_tcps["left"][:3]]
            if "right" in arm_tcps:
                dbg_details["dbg_right_tcp"] = [round(float(x), 4) for x in arm_tcps["right"][:3]]

        if st.fault:
            self.best_score = 0.0
            return ScoreResult(score=0.0, success=False, details=dbg_details)

        self.best_score = max(self.best_score, st.score)
        success = st.solved and (self.best_score >= 1.00) and (not st.fault)

        dbg_details["best_score"] = self.best_score
        return ScoreResult(
            score=self.best_score,
            success=success,
            details=dbg_details,
        )
