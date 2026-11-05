"""Scorer for the star-pattern cold-plate bolt torque task. Pure Python + numpy.

Call ``Scorer.update`` once per control step (20 Hz, dt = 0.05 s).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping

import numpy as np

import geometry as g
from logic import StarPatternLogic, LogicState, CORNERS

TOL = 1e-9  # Slack for floating point comparisons

# Movement and velocity thresholds
FIXTURE_MOVE_LIMIT = 0.005  # 5 mm fixture shift causes failure (>)
FIXTURE_TILT_LIMIT = 5.0    # 5 deg fixture tilt causes failure (>)
STILL_LIN = 0.005           # 5 mm/s linear velocity limit for "still" (<=)
STILL_ANG = 0.2             # 0.2 rad/s angular velocity limit for "still" (<=)

SUCCESS_HOLD_STEPS = 20     # 1.0 s at 20 Hz

SCORE_NONE = 0.0
SCORE_TOUCHED = 0.25
SCORE_PASS1 = 0.50
SCORE_PASS2 = 0.75
SCORE_SUCCESS = 1.0


def _vec(x, n: int, name: str) -> np.ndarray:
    a = np.array(x, dtype=float)
    if a.shape != (n,):
        raise ValueError(f"{name} must have shape ({n},), got {a.shape}")
    if not np.all(np.isfinite(a)):
        raise ValueError(f"{name} contains non-finite values: {a}")
    return a


def quat_to_matrix(q_xyzw) -> np.ndarray:
    q = _vec(q_xyzw, 4, "quaternion")
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        raise ValueError("zero-length quaternion")
    x, y, z, w = q / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotation_angle_deg(R1, R2) -> float:
    rel = R1.T @ R2
    c = (float(np.trace(rel)) - 1.0) / 2.0
    return math.degrees(math.acos(min(1.0, max(-1.0, c))))


@dataclass
class StepResult:
    score: float
    success: bool
    details: dict = field(default_factory=dict)


class Scorer:
    def __init__(self) -> None:
        self.logic = StarPatternLogic()
        self.reset()

    def reset(self) -> None:
        self.logic.reset()
        self.fixture_start_pos: np.ndarray | None = None
        self.failed: bool = False
        self.fail_reason: str | None = None
        self.highest_tier: float = 0.0
        self.hold_steps: int = 0
        self.button_touched: bool = False

    def update(
        self,
        fixture_pos,
        fixture_quat,
        fixture_sliders: Mapping[str, float],
        fixture_lin_vel=(0.0, 0.0, 0.0),
        fixture_ang_vel=(0.0, 0.0, 0.0),
        grippers_open: bool = True,
    ) -> StepResult:
        # 1. Capture fixture start position on first step
        fpos = np.asarray(fixture_pos, dtype=float)
        if self.fixture_start_pos is None:
            self.fixture_start_pos = fpos.copy()

        # 2. Check fixture movement zero-gates
        fixture_shift = float(np.linalg.norm(fpos - self.fixture_start_pos))
        if fixture_shift > FIXTURE_MOVE_LIMIT + TOL:
            self.failed = True
            self.fail_reason = f"Fixture moved {fixture_shift*1000:.1f} mm > 5.0 mm"

        if fixture_quat is not None:
            try:
                R_fix = quat_to_matrix(fixture_quat)
                tilt = rotation_angle_deg(R_fix, np.eye(3))
                if tilt > FIXTURE_TILT_LIMIT + TOL:
                    self.failed = True
                    self.fail_reason = f"Fixture tilted {tilt:.1f} deg > 5.0 deg"
            except Exception:
                pass

        # 3. Check if any button was depressed
        for c in CORNERS:
            q = float(fixture_sliders.get(f"btn_{c}", 0.0))
            if q <= -0.002:  # depressed beyond 2 mm
                self.button_touched = True

        # 4. Update hidden-state logic
        logic_state = self.logic.update(fixture_sliders)

        # 5. Check WARP zero-gate
        if logic_state.warp:
            self.failed = True
            if not self.fail_reason:
                self.fail_reason = logic_state.warp_reason

        # If zero-gated: score 0.0 permanently
        if self.failed:
            return StepResult(
                score=SCORE_NONE,
                success=False,
                details={
                    "failed": True,
                    "fail_reason": self.fail_reason,
                    "fixture_shift": fixture_shift,
                    "warp": logic_state.warp,
                    "stages": logic_state.stages,
                    "status_lamp": logic_state.status_lamp,
                },
            )

        # 6. Evaluate partial credit tiers
        tier = SCORE_NONE
        if self.button_touched or any(s >= 1 for s in logic_state.stages.values()):
            tier = max(tier, SCORE_TOUCHED)

        if logic_state.completed_passes >= 1 and all(s >= 1 for s in logic_state.stages.values()):
            tier = max(tier, SCORE_PASS1)

        if logic_state.completed_passes >= 2 and all(s >= 2 for s in logic_state.stages.values()):
            tier = max(tier, SCORE_PASS2)

        self.highest_tier = max(self.highest_tier, tier)

        # 7. Check final 1.0 criteria
        all_stage_3 = all(s == 3 for s in logic_state.stages.values())
        lamp_green = (logic_state.status_lamp == "green")
        l_spd = float(np.linalg.norm(fixture_lin_vel))
        a_spd = float(np.linalg.norm(fixture_ang_vel))
        still = (l_spd <= STILL_LIN + TOL and a_spd <= STILL_ANG + TOL)

        full_conditions = (
            all_stage_3
            and lamp_green
            and not logic_state.warp
            and grippers_open
            and still
        )

        if full_conditions:
            self.hold_steps += 1
        else:
            self.hold_steps = 0

        success = (self.hold_steps >= SUCCESS_HOLD_STEPS)
        current_score = SCORE_SUCCESS if success else self.highest_tier

        return StepResult(
            score=current_score,
            success=success,
            details={
                "failed": False,
                "stages": logic_state.stages,
                "warp": logic_state.warp,
                "status_lamp": logic_state.status_lamp,
                "completed_passes": logic_state.completed_passes,
                "current_pass": logic_state.current_pass,
                "highest_tier": self.highest_tier,
                "hold_steps": self.hold_steps,
                "fixture_shift": fixture_shift,
                "grippers_open": grippers_open,
                "still": still,
                "cyan_lamps": logic_state.cyan_lamps,
            },
        )
