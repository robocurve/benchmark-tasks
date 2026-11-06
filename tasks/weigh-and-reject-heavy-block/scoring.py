"""Scoring ladder and verification for weigh-and-reject-heavy-block. Pure Python + NumPy.

Ladder:
  0.0:  Nothing completed.
  0.25: All three blocks weighed on the scale (each >= 1.0 s steady, platform sag within tolerance).
  0.50: Heavy block in reject tray AND it was weighed.
  0.75: Additionally both light blocks rest on the green shelf.
  1.00: Additionally scale is empty, nothing on the floor, grippers open, arms still,
        held for 20 consecutive steps (1.0 s at 20 Hz).

Zero-gates:
  - Wrong rejection: a light block placed in the reject tray after being weighed caps score at 0.25.
  - Shortcut penalty: putting the heavy block in reject tray without weighing gives no reject credit (caps at 0.25).
  - Dropped block: any block falling off table drops score to 0.0.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping
import numpy as np

import geometry as g
import logic

STILL_LIN_VEL = 0.005          # 5 mm/s
STILL_ANG_VEL = 0.200          # 0.2 rad/s


@dataclass(frozen=True)
class ScoreResult:
    score: float
    success: bool
    details: dict


class Scorer:
    """Stateful scorer evaluating weighing inspection ladder and zero-gates."""

    def __init__(self, hold_s: float = 1.0, hz: float = 20.0) -> None:
        self.hold_steps = int(round(hold_s * hz))
        self.dt = 1.0 / hz
        self.logic = logic.WeighLogic()
        self.reset()

    def reset(self) -> None:
        self.logic = logic.WeighLogic()
        self.consecutive_success_steps = 0
        self.best_score = 0.0
        self.latched_wrong_rejection = False

    def update(
        self,
        scale_disp: float,
        block_positions: Mapping[str, tuple[float, float, float] | np.ndarray],
        grippers_open: bool,
        arm_tcps: Mapping[str, tuple[float, float, float] | np.ndarray] | None = None,
        arm_velocities: Mapping[str, tuple[np.ndarray, np.ndarray]] | None = None,
    ) -> ScoreResult:
        st = self.logic.step(
            dt=self.dt,
            scale_disp=scale_disp,
            block_positions=dict(block_positions),
            arm_tcps=dict(arm_tcps) if arm_tcps else None,
        )

        if st.wrong_rejection:
            self.latched_wrong_rejection = True

        if st.block_dropped:
            return ScoreResult(
                score=0.0,
                success=False,
                details={"reason": "block_dropped"},
            )

        arms_still = True
        if arm_velocities is not None:
            for arm_name, (lin_v, ang_v) in arm_velocities.items():
                if np.linalg.norm(lin_v) > STILL_LIN_VEL or np.linalg.norm(ang_v) > STILL_ANG_VEL:
                    arms_still = False
                    break

        score = 0.0
        success = False

        if not self.latched_wrong_rejection:
            if st.all_three_weighed:
                score = max(score, 0.25)

            # Heavy block in reject tray gets credit ONLY if it was weighed
            if st.heavy_in_tray and st.block_weighed["block_1"]:
                score = max(score, 0.50)

                if st.both_lights_on_shelf:
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
            self.best_score = max(self.best_score, score)
        else:
            # Wrong rejection caps score at 0.25
            score = 0.25 if st.all_three_weighed else 0.0
            self.consecutive_success_steps = 0
            self.best_score = min(self.best_score, score)

        return ScoreResult(
            score=self.best_score,
            success=success,
            details={
                "weighed": dict(st.block_weighed),
                "all_weighed": st.all_three_weighed,
                "heavy_in_tray": st.heavy_in_tray,
                "lights_on_shelf": st.both_lights_on_shelf,
                "scale_empty": st.scale_empty,
                "wrong_rejection": st.wrong_rejection,
                "consecutive_steps": self.consecutive_success_steps,
            },
        )

