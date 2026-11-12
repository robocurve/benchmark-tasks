"""Scoring ladder and verification for hot-swap-failed-drive-sled. Pure Python + NumPy.

Ladder:
  0.0:  Initial / nothing completed.
  0.25: Failed sled pulled out >= 60 mm with latch held (no fault).
  0.50: Failed sled completely clear of the rack and resting on the discard pad.
  0.75: Blue spare seated >= 95% in the failed bay (latch may still be held).
  1.00: Status lamp green (blue spare seated >= 95%, latch released, both arms clear,
        healthy sleds undisturbed, held for 20 consecutive steps / 1.0 s).

Zero-gates:
  - FAULT: forced pull without latch (moved > 3 mm in detent zone 0-25 mm without latch held),
    healthy sled moved > 5 mm. Score latches to 0.0.
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
    """Stateful scorer evaluating hot-swap drive sled ladder and zero-gates."""

    def __init__(self, hold_s: float = 1.0, hz: float = 20.0) -> None:
        self.hold_steps = int(round(hold_s * hz))
        self.dt = 1.0 / hz
        self.logic = logic.HotSwapLogic()
        self.reset()

    def reset(self) -> None:
        self.logic = logic.HotSwapLogic()
        self.consecutive_success_steps = 0
        self.last_score = 0.0
        self.latched_fault = False

    def update(
        self,
        latch_disps: Mapping[str, float],
        sled_positions: Mapping[str, tuple[float, float, float] | np.ndarray],
        spare_positions: Mapping[str, tuple[float, float, float] | np.ndarray],
        grippers_open: bool,
        arm_tcps: Mapping[str, tuple[float, float, float] | np.ndarray] | None = None,
        arm_velocities: Mapping[str, tuple[np.ndarray, np.ndarray]] | None = None,
    ) -> ScoreResult:
        st = self.logic.step(
            dt=self.dt,
            latch_disps=dict(latch_disps),
            sled_positions=dict(sled_positions),
            spare_positions=dict(spare_positions),
            arm_tcps=dict(arm_tcps) if arm_tcps else None,
        )

        if st.fault:
            self.latched_fault = True
            return ScoreResult(
                score=0.0,
                success=False,
                details={"reason": f"fault_{st.fault_reason}"},
            )

        if self.latched_fault:
            return ScoreResult(
                score=0.0,
                success=False,
                details={"reason": "latched_fault"},
            )

        arms_still = True
        if arm_velocities is not None:
            for arm_name, (lin_v, ang_v) in arm_velocities.items():
                if np.linalg.norm(lin_v) > STILL_LIN_VEL or np.linalg.norm(ang_v) > STILL_ANG_VEL:
                    arms_still = False
                    break

        score = 0.0
        success = False

        if st.pulled_60mm:
            score = max(score, 0.25)

        if st.old_sled_on_discard:
            score = max(score, 0.50)

        # Blue spare seated >= 95% gives 0.75
        if st.spare_inserted_95pct and st.blue_spare_used and st.old_sled_on_discard:
            score = max(score, 0.75)

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

        # End-state scoring: the score is that of the current step (faults stay latched; goals must still hold).
        self.last_score = score

        return ScoreResult(
            score=self.last_score,
            success=success,
            details={
                "pulled_60mm": st.pulled_60mm,
                "old_sled_on_discard": st.old_sled_on_discard,
                "spare_inserted_95pct": st.spare_inserted_95pct,
                "blue_spare_used": st.blue_spare_used,
                "orange_spare_used": st.orange_spare_used,
                "lamp_green": st.lamp_green,
                "consecutive_steps": self.consecutive_success_steps,
            },
        )

