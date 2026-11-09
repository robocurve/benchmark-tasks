"""Scoring ladder and verification for tower-transfer-no-big-on-small. Pure Python + NumPy.

Ladder:
  0.00: Initial / invalid state.
  0.25: Small block on MIDDLE or RIGHT mark (first move done).
  0.50: Large block off LEFT and resting on the table at MIDDLE or RIGHT.
  0.75: Large block on RIGHT with a smaller block on it or both others on MIDDLE.
  1.00: All three blocks on RIGHT in order (Large bottom, Med middle, Small top),
        offsets <= 12 mm from the mark centre, tilt < 5 deg, arms clear (> 60 mm),
        held stable for 2.0 s (40 steps at 20 Hz).

Zero-gates:
  - Bigger block resting on smaller block (centre within 40 mm in xy and higher by >= 20 mm
    after resting < 5 mm/s for 0.3 s) latches rule_broken = score 0.0 permanently.
  - Floor fault: any block falling off table (z < TOP - 0.05 m) latches score 0.0 permanently.
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
    """Stateful scorer evaluating tower transfer ladder and rule zero-gates."""

    def __init__(self, hold_s: float = g.HOLD_TIME_S, hz: float = 20.0) -> None:
        self.dt = 1.0 / hz
        self.logic = logic.TowerLogic(hold_s=hold_s)
        self.reset()

    def reset(self) -> None:
        self.logic.reset()
        self.best_score = 0.0

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

        dbg_details = {
            "dbg_rule_broken": st.rule_broken,
            "dbg_floor_fault": st.floor_fault,
            "dbg_large_pos": [round(float(x), 4) for x in large_pos],
            "dbg_medium_pos": [round(float(x), 4) for x in medium_pos],
            "dbg_small_pos": [round(float(x), 4) for x in small_pos],
            "dbg_large_mark": st.large_mark,
            "dbg_med_mark": st.med_mark,
            "dbg_small_mark": st.small_mark,
            "dbg_hold_time": round(st.hold_time, 2),
            "dbg_ever_25": st.ever_25,
            "dbg_ever_50": st.ever_50,
            "dbg_ever_75": st.ever_75,
            "dbg_ever_100": st.ever_100,
        }

        if arm_tcps is not None:
            if "left" in arm_tcps:
                dbg_details["dbg_left_tcp"] = [round(float(x), 4) for x in arm_tcps["left"]]
            if "right" in arm_tcps:
                dbg_details["dbg_right_tcp"] = [round(float(x), 4) for x in arm_tcps["right"]]

        if st.rule_broken or st.floor_fault:
            return ScoreResult(score=0.0, success=False, details=dbg_details)

        score = 0.0
        if st.ever_25:
            score = max(score, 0.25)
        if st.ever_50:
            score = max(score, 0.50)
        if st.ever_75:
            score = max(score, 0.75)
        if st.ever_100:
            score = 1.00

        self.best_score = max(self.best_score, score)
        success = st.solved and (self.best_score >= 1.00)

        dbg_details["best_score"] = self.best_score
        return ScoreResult(
            score=self.best_score,
            success=success,
            details=dbg_details,
        )

