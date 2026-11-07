"""Scoring ladder and zero-gate verification for estop-before-clearing-jam. Pure Python + NumPy.

Ladder:
  0.00: Initial state / nothing completed.
  0.25: E-STOP counted (pusher stopped, no fault, latched ever_stopped).
  0.50: + stuck block removed from lane danger zone into green tray (no fault),
        or RESET pressed after E-STOP without fault.
  0.75: + RESET then START counted in that order after block is out of lane
        (RUN lamp on, pusher moving).
  1.00: + arms outside danger zone, end state held for 2.0 s (40 steps at 20 Hz).

Zero-gates:
  - FAULT: any arm TCP inside danger zone while pusher is moving.
  - FAULT: START pressed with block still in lane.
  Latched FAULT freezes the score permanently at the tier reached before the fault.
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
    """Stateful scorer evaluating E-STOP sequence ladder and safety zero-gates."""

    def __init__(self, hold_s: float = 2.0, hz: float = 20.0) -> None:
        self.hold_steps = int(round(hold_s * hz))
        self.dt = 1.0 / hz
        self.logic = logic.JamClearLogic()
        self.reset()

    def reset(self) -> None:
        self.logic = logic.JamClearLogic()
        self.consecutive_success_steps = 0
        self.best_score = 0.0
        self.latched_fault = False

    def update(
        self,
        estop_disp: float,
        reset_disp: float,
        start_disp: float,
        block_pos: tuple[float, float, float] | np.ndarray,
        arm_tcps: Mapping[str, tuple[float, float, float] | np.ndarray] | None = None,
    ) -> ScoreResult:
        st = self.logic.step(
            dt=self.dt,
            estop_disp=estop_disp,
            reset_disp=reset_disp,
            start_disp=start_disp,
            block_pos=block_pos,
            arm_tcps=dict(arm_tcps) if arm_tcps else None,
        )

        score = 0.0
        success = False

        if st.ever_stopped:
            score = max(score, 0.25)

        if st.ever_stopped and (st.block_cleared or st.reset_pressed):
            score = max(score, 0.50)

        if st.ever_stopped and st.block_cleared and st.restarted:
            score = max(score, 0.75)

        if not self.latched_fault and not st.fault and st.solved:
            self.consecutive_success_steps += 1
            if self.consecutive_success_steps >= self.hold_steps:
                score = 1.00
                success = True
        else:
            self.consecutive_success_steps = 0

        if st.fault:
            self.latched_fault = True

        if not self.latched_fault:
            self.best_score = max(self.best_score, score)
        else:
            # Score cannot advance after a fault
            pass

        return ScoreResult(
            score=self.best_score,
            success=success,
            details={
                "pusher_moving": st.pusher_moving,
                "ever_stopped": st.ever_stopped,
                "estop_pressed": st.estop_pressed,
                "reset_pressed": st.reset_pressed,
                "start_pressed": st.start_pressed,
                "block_in_lane": st.block_in_lane,
                "block_cleared": st.block_cleared,
                "run_lamp_lit": st.run_lamp_lit,
                "stop_lamp_lit": st.stop_lamp_lit,
                "fault_lamp_lit": st.fault_lamp_lit,
                "fault": st.fault,
                "fault_reason": st.fault_reason,
                "restarted": st.restarted,
                "pusher_x": st.pusher_x,
                "consecutive_steps": self.consecutive_success_steps,
            },
        )
