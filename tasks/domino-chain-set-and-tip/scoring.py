"""Scoring ladder and verification for domino-chain-set-and-tip. Pure Python + NumPy.

Ladder:
  0.00: Initial / invalid state.
  0.25: Two dominoes standing on end on marks (tilt < 6 deg, resting 0.3 s, on mark within 8 mm).
  0.50: All four dominoes standing on end on marks.
  0.75: First domino tipped (tilt > 20 deg) while others are standing (cascade initiated).
  1.00: All four dominoes fallen toward +x within 5.0 s of tip, with both arms clear (> 60 mm).

Zero-gates:
  - Premature fall: Any domino falling (tilt > 45 deg) before all four are standing caps score at 0.25 max.
  - Floor fault: Any domino falling off the table (z < TOP - 0.05 m) permanently zeroes the score.
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
    """Stateful scorer evaluating domino setup ladder and toppling zero-gates."""

    def __init__(
        self,
        rest_s: float = g.REST_TIME_S,
        max_tip_time_s: float = g.MAX_TIP_TO_FALL_TIME_S,
        hz: float = 20.0,
    ) -> None:
        self.dt = 1.0 / hz
        self.logic = logic.DominoLogic(rest_s=rest_s, max_tip_time_s=max_tip_time_s)
        self.reset()

    def reset(self) -> None:
        self.logic.reset()
        self.best_score = 0.0

    def update(
        self,
        domino_poses: list[tuple[float, float, float] | np.ndarray],
        domino_quats: list[tuple[float, float, float, float] | np.ndarray],
        domino_vels: list[tuple[float, float, float] | np.ndarray],
        arm_tcps: Mapping[str, tuple[float, float, float] | np.ndarray] | None = None,
    ) -> ScoreResult:
        st = self.logic.step(
            dt=self.dt,
            domino_poses=domino_poses,
            domino_quats=domino_quats,
            domino_vels=domino_vels,
            arm_tcps=dict(arm_tcps) if arm_tcps is not None else None,
        )

        dbg_details = {
            "dbg_standing_count": st.standing_count,
            "dbg_marks_occupied": list(st.marks_occupied),
            "dbg_fallen_count": st.fallen_count,
            "dbg_premature": st.premature,
            "dbg_ever_standing_2": st.ever_standing_2,
            "dbg_ever_standing_4": st.ever_standing_4,
            "dbg_first_tipped": st.first_tipped,
            "dbg_ever_fallen_4": st.ever_fallen_4,
            "dbg_tilts": [round(float(t), 2) for t in st.tilts],
            "dbg_tip_timer": round(float(st.tip_timer), 2),
            "dbg_arms_clear": st.arms_clear,
            "dbg_floor_fault": st.floor_fault,
        }

        if arm_tcps is not None:
            if "left" in arm_tcps:
                dbg_details["dbg_left_tcp"] = [round(float(x), 4) for x in arm_tcps["left"]]
            if "right" in arm_tcps:
                dbg_details["dbg_right_tcp"] = [round(float(x), 4) for x in arm_tcps["right"]]

        if st.floor_fault:
            return ScoreResult(score=0.0, success=False, details=dbg_details)

        # Milestone score calculation
        score = 0.0
        if st.ever_standing_2:
            score = max(score, 0.25)
        if st.ever_standing_4:
            score = max(score, 0.50)
        if st.first_tipped:
            score = max(score, 0.75)
        if st.ever_fallen_4:
            score = 1.00

        # Zero-gate: premature fall caps score at 0.25 max
        if st.premature:
            score = min(score, 0.25)

        self.best_score = max(self.best_score, score)
        if st.premature:
            self.best_score = min(self.best_score, 0.25)

        success = (self.best_score >= 1.00) and (not st.premature) and st.solved

        dbg_details["best_score"] = self.best_score
        return ScoreResult(
            score=self.best_score,
            success=success,
            details=dbg_details,
        )
