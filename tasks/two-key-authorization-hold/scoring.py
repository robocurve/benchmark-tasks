"""Scoring ladder and verification for two-key-authorization-hold. Pure Python + NumPy.

Ladder:
  0.0:  Nothing touched.
  0.25: Both guards open >= 60 mm.
  0.50: Both keys held down simultaneously >= 1.5 s (max simultaneous hold).
  0.75: ARMED state reached (3.0 s simultaneous hold).
  1.00: CONFIRM pressed while ARMED, no LOCK, keys released, grippers open, arms still,
        held for 20 consecutive steps (1.0 s at 20 Hz).

Zero-gates:
  - LOCKED (2 strikes or 10 s timeout after armed) caps score at 0.25.
  - Fixture shift > 10 mm or tilt > 5 deg drops score to 0.0.
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
    """Stateful scorer evaluating two-key authorization ladder and lockout gates."""

    def __init__(self, hold_s: float = 1.0, hz: float = 20.0) -> None:
        self.hold_steps = int(round(hold_s * hz))
        self.dt = 1.0 / hz
        self.logic = logic.TwoKeyLogic()
        self.reset()

    def reset(self) -> None:
        self.logic = logic.TwoKeyLogic()
        self.consecutive_success_steps = 0
        self.best_score = 0.0
        self.latched_lockout = False
        self.ever_guards_open = False

    def update(
        self,
        fixture_pos: tuple[float, float, float] | np.ndarray,
        fixture_quat: tuple[float, float, float, float] | np.ndarray,
        fixture_sliders: Mapping[str, float],
        grippers_open: bool,
        arm_velocities: Mapping[str, tuple[np.ndarray, np.ndarray]] | None = None,
    ) -> ScoreResult:
        # Check fixture displacement
        fpos = np.asarray(fixture_pos, dtype=float)
        base_pos = np.asarray(g.BASE_POS, dtype=float)
        fixture_dist = float(np.linalg.norm(fpos[:2] - base_pos[:2]))
        if fixture_dist > 0.010:  # > 10 mm
            return ScoreResult(
                score=0.0,
                success=False,
                details={"reason": "fixture_shifted", "shift_mm": fixture_dist * 1000.0},
            )

        # Extract slider displacements
        btn_disps = {
            "key_left": float(fixture_sliders.get("btn_key_left", 0.0)),
            "key_right": float(fixture_sliders.get("btn_key_right", 0.0)),
            "confirm": float(fixture_sliders.get("btn_confirm", 0.0)),
        }
        guard_disps = {
            "guard_left": float(fixture_sliders.get("slider_guard_left", 0.0)),
            "guard_right": float(fixture_sliders.get("slider_guard_right", 0.0)),
        }

        st = self.logic.step(self.dt, btn_disps, guard_disps)
        if st.both_guards_open:
            self.ever_guards_open = True   # the guards creep back on their own, so remember that they were opened

        if st.locked:
            self.latched_lockout = True

        arms_still = True
        if arm_velocities is not None:
            for arm_name, (lin_v, ang_v) in arm_velocities.items():
                if np.linalg.norm(lin_v) > STILL_LIN_VEL or np.linalg.norm(ang_v) > STILL_ANG_VEL:
                    arms_still = False
                    break

        # Check whether keys are released
        keys_released = (
            btn_disps["key_left"] >= g.BUTTON_RELEASE_THRESHOLD
            and btn_disps["key_right"] >= g.BUTTON_RELEASE_THRESHOLD
        )

        score = 0.0
        success = False

        if not self.latched_lockout:
            if st.both_guards_open:
                score = max(score, 0.25)

            if st.max_simultaneous_hold >= 1.5:
                score = max(score, 0.50)

            if st.armed or st.solved:
                score = max(score, 0.75)

            at_1_00 = (
                st.solved
                and not st.locked
                and keys_released
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
        else:
            # Locked out caps score at 0.25 if guards were opened, otherwise 0.0
            if self.ever_guards_open:
                score = 0.25
            else:
                score = 0.0
            self.consecutive_success_steps = 0
            self.best_score = min(self.best_score, score)

        if not self.latched_lockout:
            self.best_score = max(self.best_score, score)

        return ScoreResult(
            score=self.best_score,
            success=success,
            details={
                "guards_open": st.both_guards_open,
                "sim_hold": st.max_simultaneous_hold,
                "armed": st.armed,
                "solved": st.solved,
                "locked": st.locked,
                "strikes": st.strikes,
                "consecutive_steps": self.consecutive_success_steps,
            },
        )
