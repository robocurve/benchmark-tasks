"""Scoring ladder and verification for interlocked-drawer-transfer. Pure Python + NumPy.

Ladder:
  0.00: Initial / nothing completed.
  0.25: Left drawer opened >= 70 mm and block lifted out (above rim, clear of drawer).
  0.50: Left drawer closed again (<= 5 mm) while block is held outside or parked on cabinet top.
  0.75: Right drawer opened >= 70 mm.
  1.00: Block resting inside right drawer, both drawers closed (<= 5 mm),
        grippers open, arms still and clear of cabinet, no TIP fault,
        held for 20 consecutive steps (1.0 s at 20 Hz).

Zero-gates:
  - TIP FAULT: both drawers open simultaneously for >= 0.2 s (exceed 15 mm simultaneously).
    Locks score to at most 0.25 and permanently disables success.
  - Dropped block: block falls off table (z < TOP - 0.05 m). Score = 0.00 permanently.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping
import numpy as np

import geometry as g
import logic

STILL_LIN_VEL = 0.005  # 5 mm/s
STILL_ANG_VEL = 0.200  # 0.2 rad/s


@dataclass(frozen=True)
class ScoreResult:
    score: float
    success: bool
    details: dict


class Scorer:
    """Stateful scorer evaluating interlocked drawer transfer ladder and zero-gates."""

    def __init__(self, hold_s: float = 1.0, hz: float = 20.0) -> None:
        self.hold_steps = int(round(hold_s * hz))
        self.dt = 1.0 / hz
        self.logic = logic.DrawerTransferLogic()
        self.reset()

    def reset(self) -> None:
        self.logic = logic.DrawerTransferLogic()
        self.consecutive_success_steps = 0
        self.best_score = 0.0
        self.latched_tip_fault = False
        self.latched_drop = False

    def update(
        self,
        disp_left: float,
        disp_right: float,
        block_pos: tuple[float, float, float] | np.ndarray,
        grippers_open: bool = True,
        arm_tcps: Mapping[str, tuple[float, float, float] | np.ndarray] | None = None,
        arm_velocities: Mapping[str, tuple[np.ndarray, np.ndarray]] | None = None,
    ) -> ScoreResult:
        st = self.logic.step(
            dt=self.dt,
            disp_left=disp_left,
            disp_right=disp_right,
            block_pos=block_pos,
            arm_tcps=dict(arm_tcps) if arm_tcps else None,
        )

        if st.tip_fault:
            self.latched_tip_fault = True
        if st.block_dropped:
            self.latched_drop = True

        if self.latched_drop:
            return ScoreResult(
                score=0.0,
                success=False,
                details={
                    "reason": "dropped",
                    "block_dropped": True,
                    "tip_fault": self.latched_tip_fault,
                    "lamp_ok": False,
                    "lamp_tip": self.latched_tip_fault,
                },
            )

        arms_still = True
        if arm_velocities is not None:
            for _arm_name, (lin_v, ang_v) in arm_velocities.items():
                if np.linalg.norm(lin_v) > STILL_LIN_VEL or np.linalg.norm(ang_v) > STILL_ANG_VEL:
                    arms_still = False
                    break

        score = 0.0
        success = False

        if st.stage_0_25:
            score = max(score, 0.25)

        if st.stage_0_50:
            score = max(score, 0.50)

        if st.stage_0_75:
            score = max(score, 0.75)

        if self.latched_tip_fault:
            # TIP fault caps score at 0.25 and permanently prevents higher scores
            score = min(score, 0.25)
            self.best_score = min(self.best_score, 0.25) if self.best_score > 0.25 else max(self.best_score, score)
            self.consecutive_success_steps = 0
            return ScoreResult(
                score=self.best_score,
                success=False,
                details={
                    "reason": "tip_fault",
                    "tip_fault": True,
                    "stage_0_25": st.stage_0_25,
                    "stage_0_50": st.stage_0_50,
                    "stage_0_75": st.stage_0_75,
                    "lamp_ok": False,
                    "lamp_tip": True,
                },
            )

        at_1_00 = (
            st.solved
            and grippers_open
            and arms_still
        )

        if at_1_00:
            self.consecutive_success_steps += 1
            if self.consecutive_success_steps >= self.hold_steps:
                score = 1.00
                success = True
        else:
            self.consecutive_success_steps = 0

        self.best_score = max(self.best_score, score)

        return ScoreResult(
            score=self.best_score,
            success=success,
            details={
                "stage_0_25": st.stage_0_25,
                "stage_0_50": st.stage_0_50,
                "stage_0_75": st.stage_0_75,
                "block_in_right_drawer": st.block_in_right_drawer,
                "both_drawers_closed": st.both_drawers_closed,
                "arms_clear": st.arms_clear,
                "lamp_ok": st.lamp_ok,
                "lamp_tip": st.lamp_tip,
                "consecutive_steps": self.consecutive_success_steps,
            },
        )
