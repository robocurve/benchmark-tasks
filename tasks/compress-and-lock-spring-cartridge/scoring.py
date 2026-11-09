"""Scoring ladder and verification for compress-and-lock-spring-cartridge. Pure Python + NumPy.

Ladder:
  0.0:  Initial / idle.
  0.25: Plunger compressed >= 38 mm held for >= 1.0 s.
  0.50: + Lock bar across >= 55 mm while plunger compressed (ever_locked latched per lesson 8).
  0.75: + Plunger released (no arm contact) and stays compressed >= 38 mm for >= 2.0 s.
  1.00: + Both arms clear and lamp green held for >= 1.0 s (20 control steps at 20 Hz).

Zero-gates:
  None (physical stop box prevents early bar advance; no zero-gate penalty needed).
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
    """Stateful scorer evaluating cartridge compression ladder."""

    def __init__(self, hold_s: float = 1.0, hz: float = 20.0) -> None:
        self.hold_steps = int(round(hold_s * hz))
        self.dt = 1.0 / hz
        self.logic = logic.CartridgeLogic()
        self.reset()

    def reset(self) -> None:
        self.logic = logic.CartridgeLogic()
        self.consecutive_success_steps = 0
        self.best_score = 0.0

    def update(
        self,
        plunger_disp: float,
        bar_disp: float,
        grippers_open: bool = True,
        arm_tcps: Mapping[str, tuple[float, float, float] | np.ndarray] | None = None,
        arm_velocities: Mapping[str, tuple[np.ndarray, np.ndarray]] | None = None,
        forced_plunger_released: bool | None = None,
    ) -> ScoreResult:
        st = self.logic.step(
            dt=self.dt,
            raw_plunger_disp=plunger_disp,
            raw_bar_disp=bar_disp,
            arm_tcps=dict(arm_tcps) if arm_tcps else None,
            forced_plunger_released=forced_plunger_released,
        )

        arms_still = True
        if arm_velocities is not None:
            for _arm_name, (lin_v, ang_v) in arm_velocities.items():
                if np.linalg.norm(lin_v) > STILL_LIN_VEL or np.linalg.norm(ang_v) > STILL_ANG_VEL:
                    arms_still = False
                    break

        score = 0.0

        if st.plunger_compressed_1s:
            score = max(score, 0.25)

        if st.ever_locked and st.plunger_compressed_1s:
            score = max(score, 0.50)

        if st.released_and_held_2s and st.ever_locked and st.plunger_compressed_1s:
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
        else:
            self.consecutive_success_steps = 0

        self.best_score = max(self.best_score, score)
        success = (self.best_score >= 1.00)

        details = {
            "plunger_disp": st.plunger_disp,
            "bar_disp": st.bar_disp,
            "bar_blocked": st.bar_blocked,
            "plunger_compressed_1s": st.plunger_compressed_1s,
            "compress_time": st.compress_time,
            "ever_locked": st.ever_locked,
            "plunger_released": st.plunger_released,
            "released_and_held_2s": st.released_and_held_2s,
            "release_hold_time": st.release_hold_time,
            "arms_clear": st.arms_clear,
            "arms_clear_1s": st.arms_clear_1s,
            "lamp_green": st.lamp_green,
            "consecutive_success": self.consecutive_success_steps,
        }

        return ScoreResult(score=self.best_score, success=success, details=details)
