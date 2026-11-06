"""Scoring ladder and verification for lockout-tagout-before-service. Pure Python + NumPy.

Ladder:
  0.0:  Initial / nothing completed.
  0.25: Breaker switched OFF (LIVE lamp off).
  0.50: Red lock pin seated in breaker hole.
  0.75: TEST pressed and verified DEAD (green lamp shown).
  1.00: SERVICE button pressed while cover held open >= 70 mm for >= 1.0 s,
        held for 20 consecutive steps (1.0 s at 20 Hz).

Zero-gates:
  - FAULT: cover opened >= 20 mm while breaker is ON, or pin not seated, or before TEST DEAD.
    Score latches permanently to 0.0.
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
    """Stateful scorer evaluating LOTO ladder and zero-gates."""

    def __init__(self, hold_s: float = 1.0, hz: float = 20.0) -> None:
        self.hold_steps = int(round(hold_s * hz))
        self.dt = 1.0 / hz
        self.logic = logic.LotoLogic()
        self.reset()

    def reset(self) -> None:
        self.logic = logic.LotoLogic()
        self.consecutive_success_steps = 0
        self.best_score = 0.0
        self.latched_fault = False

    def update(
        self,
        breaker_disp: float,
        pin_pos: tuple[float, float, float] | np.ndarray,
        test_btn_disp: float,
        cover_disp: float,
        service_btn_disp: float,
        grippers_open: bool = True,
        arm_tcps: Mapping[str, tuple[float, float, float] | np.ndarray] | None = None,
        arm_velocities: Mapping[str, tuple[np.ndarray, np.ndarray]] | None = None,
    ) -> ScoreResult:
        st = self.logic.step(
            dt=self.dt,
            breaker_disp=breaker_disp,
            pin_pos=pin_pos,
            test_btn_disp=test_btn_disp,
            cover_disp=cover_disp,
            service_btn_disp=service_btn_disp,
            arm_tcps=dict(arm_tcps) if arm_tcps else None,
        )

        if st.fault:
            self.latched_fault = True

        score = 0.0
        success = False

        if st.breaker_off:
            score = max(score, 0.25)

        if st.pin_seated and st.breaker_off:
            score = max(score, 0.50)

        if st.test_pressed_dead and st.pin_seated:
            score = max(score, 0.75)

        if not self.latched_fault and st.solved:
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
                "breaker_off": st.breaker_off,
                "pin_seated": st.pin_seated,
                "test_pressed_dead": st.test_pressed_dead,
                "dead_lamp_lit": st.dead_lamp_lit,
                "live_lamp_lit": st.live_lamp_lit,
                "cover_open_70mm": st.cover_open_70mm,
                "service_pressed": st.service_pressed,
                "consecutive_steps": self.consecutive_success_steps,
            },
        )
