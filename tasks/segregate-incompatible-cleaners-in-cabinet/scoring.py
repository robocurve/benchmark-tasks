"""Scoring ladder and verification for segregate-incompatible-cleaners-in-cabinet. Pure Python + NumPy.

Ladder:
  0.00: Neither bottle placed in storage cabinet.
  0.25: One bottle placed in either bay (tilt < 6 deg, seated in recessed well).
  0.50: Both bottles picked/moved from staging pads.
  0.75: Both bottles placed into separate bays, seated, tilt < 6 deg.
  1.00: Both bottles segregated (different bays), fully seated, tilt < 5 deg, grippers open and retracted to standby, held 2.0 s.

Hazard Zero-Gate / Latch:
  - Incompatible mixing: If both bottles are placed into the same bay or come within 0.120 m of each other,
    the toxic chloramine gas hazard violation latches permanently, capping the score at 0.25 max.
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
    """Stateful scorer evaluating chemical cleaner segregation and cabinet placement."""

    def __init__(self, variant: str = "A", hold_s: float = g.HOLD_TIME_S, hz: float = 20.0) -> None:
        self.variant = variant
        self.hold_s = hold_s
        self.hz = hz
        self.dt = 1.0 / hz
        self.logic = logic.CleanersCabinetLogic(variant=variant, hold_s=hold_s, hz=hz)
        self.reset()

    def reset(self) -> None:
        self.logic.reset()
        self.last_score = 0.0

    def update(
        self,
        bleach_pos: tuple[float, float, float] | np.ndarray,
        bleach_quat: tuple[float, float, float, float] | np.ndarray = (0.0, 0.0, 0.0, 1.0),
        bleach_vel: tuple[float, float, float] | np.ndarray = (0.0, 0.0, 0.0),
        ammonia_pos: tuple[float, float, float] | np.ndarray = g.STAGING_RIGHT_POS,
        ammonia_quat: tuple[float, float, float, float] | np.ndarray = (0.0, 0.0, 0.0, 1.0),
        ammonia_vel: tuple[float, float, float] | np.ndarray = (0.0, 0.0, 0.0),
        arm_tcps: Mapping[str, tuple[float, float, float] | np.ndarray] | None = None,
    ) -> ScoreResult:
        st = self.logic.step(
            dt=self.dt,
            bleach_pos=bleach_pos,
            bleach_quat=bleach_quat,
            bleach_vel=bleach_vel,
            ammonia_pos=ammonia_pos,
            ammonia_quat=ammonia_quat,
            ammonia_vel=ammonia_vel,
            arm_tcps=dict(arm_tcps) if arm_tcps else None,
        )

        self.last_score = st.score
        success = (self.last_score >= 1.00)

        bp = [round(float(v), 4) for v in bleach_pos]
        ap = [round(float(v), 4) for v in ammonia_pos]
        l_tcp = [round(float(v), 4) for v in arm_tcps["left"]] if arm_tcps and "left" in arm_tcps else None
        r_tcp = [round(float(v), 4) for v in arm_tcps["right"]] if arm_tcps and "right" in arm_tcps else None

        if success:
            reason = "Task fully succeeded: Incompatible cleaning chemicals safely segregated into separate cabinet bays, held 2.0 s."
        elif st.incompatible_mix_violation:
            reason = st.violation_reason
        elif st.both_placed_separate:
            reason = f"Partial credit 0.75: Both bottles placed into separate bays; hold pending ({st.hold_time_s:.1f}s / {self.hold_s:.1f}s)."
        elif st.both_moved:
            reason = "Partial credit 0.50: Both bottles picked and moved from staging."
        elif st.one_placed:
            reason = "Partial credit 0.25: One bottle placed in cabinet bay."
        else:
            reason = "Neither bottle placed in cabinet bay."

        details = {
            "dbg_variant": self.variant,
            "dbg_bleach_pos": bp,
            "dbg_ammonia_pos": ap,
            "dbg_bleach_in_left": st.bleach_in_left,
            "dbg_bleach_in_right": st.bleach_in_right,
            "dbg_ammonia_in_left": st.ammonia_in_left,
            "dbg_ammonia_in_right": st.ammonia_in_right,
            "dbg_both_placed_separate": st.both_placed_separate,
            "dbg_both_placed_same_bay": st.both_placed_same_bay,
            "dbg_incompatible_mix_violation": st.incompatible_mix_violation,
            "dbg_bottles_still": st.bottles_still,
            "dbg_arms_clear": st.arms_clear,
            "dbg_hold_time_s": round(st.hold_time_s, 2),
            "dbg_score": round(self.last_score, 2),
            "dbg_left_tcp": l_tcp,
            "dbg_right_tcp": r_tcp,
            "dbg_reason": reason,
        }

        return ScoreResult(score=self.last_score, success=success, details=details)
