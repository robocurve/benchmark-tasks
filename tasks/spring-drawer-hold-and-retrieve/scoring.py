"""Scoring ladder and verification for spring-drawer-hold-and-retrieve. Pure Python + NumPy.

Ladder:
  0.0:  Initial / nothing completed.
  0.25: Drawer opened >= 90 mm (cyan lamp lit) for >= 1.0 s.
  0.50: Block grasped and lifted >= 30 mm clear of the drawer floor.
  0.75: Block resting on the green shelf.
  1.00: Drawer closed (< 5 mm) afterwards with both arms clear of cabinet,
        held for 20 consecutive steps (1.0 s at 20 Hz).

Zero-gates:
  - PINCH: drawer closing (< 20 mm) while block is within 20 mm of drawer front lip. Score = 0.0.
  - Dropped block: block falls off table. Score = 0.0.
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
    """Stateful scorer evaluating spring drawer ladder and zero-gates."""

    def __init__(self, hold_s: float = 1.0, hz: float = 20.0) -> None:
        self.hold_steps = int(round(hold_s * hz))
        self.dt = 1.0 / hz
        self.logic = logic.DrawerLogic()
        self.reset()

    def reset(self) -> None:
        self.logic = logic.DrawerLogic()
        self.consecutive_success_steps = 0
        self.best_score = 0.0
        self.latched_pinch = False
        self.latched_drop = False

    def update(
        self,
        drawer_disp: float,
        block_pos: tuple[float, float, float] | np.ndarray,
        grippers_open: bool,
        arm_tcps: Mapping[str, tuple[float, float, float] | np.ndarray] | None = None,
        arm_velocities: Mapping[str, tuple[np.ndarray, np.ndarray]] | None = None,
    ) -> ScoreResult:
        st = self.logic.step(
            dt=self.dt,
            drawer_disp=drawer_disp,
            block_pos=block_pos,
            arm_tcps=dict(arm_tcps) if arm_tcps else None,
        )

        if st.pinch:
            self.latched_pinch = True
        if st.block_dropped:
            self.latched_drop = True

        if self.latched_pinch:
            return ScoreResult(score=0.0, success=False, details={"reason": "pinch"})
        if self.latched_drop:
            return ScoreResult(score=0.0, success=False, details={"reason": "dropped"})

        arms_still = True
        if arm_velocities is not None:
            for arm_name, (lin_v, ang_v) in arm_velocities.items():
                if np.linalg.norm(lin_v) > STILL_LIN_VEL or np.linalg.norm(ang_v) > STILL_ANG_VEL:
                    arms_still = False
                    break

        score = 0.0
        success = False

        if st.drawer_opened_1s:
            score = max(score, 0.25)

        if st.block_lifted_30mm and st.drawer_opened_1s:
            score = max(score, 0.50)

        if st.block_on_shelf and st.block_lifted_30mm:
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

        return ScoreResult(
            score=self.best_score,
            success=success,
            details={
                "drawer_opened_1s": st.drawer_opened_1s,
                "block_lifted_30mm": st.block_lifted_30mm,
                "block_on_shelf": st.block_on_shelf,
                "drawer_closed": st.drawer_closed,
                "cyan_lamp": st.cyan_lamp,
                "consecutive_steps": self.consecutive_success_steps,
            },
        )

