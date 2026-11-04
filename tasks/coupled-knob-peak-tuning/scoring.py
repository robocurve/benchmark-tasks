"""Scoring ladder and verification for coupled knob peak tuning. Pure Python + NumPy.

Ladder:
  0.0:  Nothing touched (knobs near start positions, READ not pressed).
  0.25: A knob moved by >= 3 mm or READ pressed.
  0.50: r at final settings >= 0.5.
  0.75: r at final settings >= 0.8 with at most 8 READ presses in total and at least 2 separated
        counted presses (settings >= 10 mm apart in max(|dx|, |dy|)).
  1.0:  r at final settings >= 0.95 (needle on the green band), at most 8 READ presses in total, at
        least 3 separated counted presses, both grippers open, both knobs still (speed <= 5 mm/s),
        held for 20 consecutive control steps (1.0 s at 20 Hz). The scorer runs once per control step.

Zero-gates:
  - Any knob forced out of its channel (lateral displacement > 10 mm or lifted out) -> 0.0
  - Fixture base displaced > 5 mm -> 0.0
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

import geometry as g
import logic

# Thresholds
KNOB_MOVED_THRESHOLD_MM = 3.0  # mm
STILL_LIN_VEL = 0.005  # m/s (5 mm/s)
FIXTURE_SHIFT_LIMIT = 0.005  # 5 mm
KNOB_LATERAL_LIMIT = 0.010  # 10 mm lateral deviation from channel centerline
KNOB_MAX_Z = g.BASE_TOP_Z + g.TRACK_WALL_H + 0.015  # 15 mm above wall top
KNOB_MIN_Z = g.TOP - 0.050  # table drop limit

R_PARTIAL_0_50 = 0.50
R_PARTIAL_0_75 = 0.80
R_SUCCESS_1_00 = 0.95

MAX_ALLOWED_PRESSES = 8


@dataclass(frozen=True)
class ScoreResult:
    score: float
    success: bool
    details: dict


class Scorer:
    """Stateful scorer evaluating the 5-level partial credit ladder and zero-gates."""

    def __init__(self, hold_s: float = 1.0, hz: float = 20.0) -> None:
        self.hold_steps = int(round(hold_s * hz))
        self.logic = logic.KnobTuningLogic()
        self.reset()

    def reset(self) -> None:
        self.logic.reset()
        self.consecutive_success_steps = 0
        self.knob_moved = False
        self.zero_gate_triggered = False
        self.zero_gate_reason = ""

    def update(
        self,
        fixture_pos: tuple[float, float, float] | np.ndarray,
        fixture_sliders: dict[str, float],
        knob_left_data: tuple,  # (pos, quat, lin_vel, ang_vel)
        knob_right_data: tuple,
        grippers_open: bool,
    ) -> ScoreResult:
        """Step the scorer and return ScoreResult."""
        pos_fix = np.asarray(fixture_pos, dtype=float)
        fix_shift = float(np.linalg.norm(pos_fix - np.asarray(g.BASE_POS, dtype=float)))

        pos_l, _q_l, vel_l, _w_l = knob_left_data
        pos_r, _q_r, vel_r, _w_r = knob_right_data

        pos_l = np.asarray(pos_l, dtype=float)
        pos_r = np.asarray(pos_r, dtype=float)
        vel_l = np.asarray(vel_l, dtype=float)
        vel_r = np.asarray(vel_r, dtype=float)

        # Derive knob settings in mm from world x
        setting_l = g.knob_setting_mm_from_cx(pos_l[0])
        setting_r = g.knob_setting_mm_from_cx(pos_r[0])

        # Step the button and reading logic
        btn_z = float(fixture_sliders.get("read_btn", 0.0))
        logic_state = self.logic.step(btn_z, setting_l, setting_r)

        # Evaluate current r from physical knob positions
        r_current = logic.reading(setting_l, setting_r)

        # Movement from start positions (start settings are 30.0 mm)
        dl = abs(setting_l - g.KNOB_START_SETTING_MM)
        dr = abs(setting_r - g.KNOB_START_SETTING_MM)
        if dl >= KNOB_MOVED_THRESHOLD_MM or dr >= KNOB_MOVED_THRESHOLD_MM:
            self.knob_moved = True

        # Check zero-gates
        zero_gate = False
        reason = ""

        # 1. Base fixture shifted > 5 mm
        if fix_shift > FIXTURE_SHIFT_LIMIT:
            zero_gate = True
            reason = f"fixture base shifted {fix_shift * 1000:.1f} mm > {FIXTURE_SHIFT_LIMIT * 1000:.1f} mm"

        # 2. Left knob out of channel
        dy_l = abs(pos_l[1] - g.TRACK_Y["left"])
        if dy_l > KNOB_LATERAL_LIMIT or pos_l[2] > KNOB_MAX_Z or pos_l[2] < KNOB_MIN_Z:
            zero_gate = True
            reason = f"left knob escaped channel: dy={dy_l*1000:.1f} mm, z={pos_l[2]:.3f} m"

        # 3. Right knob out of channel
        dy_r = abs(pos_r[1] - g.TRACK_Y["right"])
        if dy_r > KNOB_LATERAL_LIMIT or pos_r[2] > KNOB_MAX_Z or pos_r[2] < KNOB_MIN_Z:
            zero_gate = True
            reason = f"right knob escaped channel: dy={dy_r*1000:.1f} mm, z={pos_r[2]:.3f} m"

        if zero_gate:
            self.zero_gate_triggered = True
            self.zero_gate_reason = reason

        # Stillness check
        speed_l = float(np.linalg.norm(vel_l))
        speed_r = float(np.linalg.norm(vel_r))
        knobs_still = (speed_l <= STILL_LIN_VEL) and (speed_r <= STILL_LIN_VEL)

        # READ press budget check: at most 8 presses
        within_press_budget = logic_state.press_count <= MAX_ALLOWED_PRESSES

        # Ladder evaluation
        score = 0.0

        if not self.zero_gate_triggered:
            if self.knob_moved or logic_state.press_count > 0:
                score = 0.25

            if r_current >= R_PARTIAL_0_50:
                score = 0.50

            # 0.75 requires r >= 0.80, within 8 press budget, AND at least 2 distinct READ presses
            if r_current >= R_PARTIAL_0_75 and within_press_budget and logic_state.has_distinct_readings:
                score = 0.75

            # Success condition for 1.0:
            # r >= 0.95, presses <= 8, at least 2 distinct READ presses, both grippers open, knobs still, held for 1.0 s
            at_peak = r_current >= R_SUCCESS_1_00
            ready_for_1_0 = (
                at_peak
                and within_press_budget
                and logic_state.has_three_separated
                and grippers_open
                and knobs_still
            )

            if ready_for_1_0:
                self.consecutive_success_steps += 1
            else:
                self.consecutive_success_steps = 0

            if self.consecutive_success_steps >= self.hold_steps:
                score = 1.0
        else:
            score = 0.0

        is_success = (score >= 1.0)

        details = {
            "score": score,
            "success": is_success,
            "r_current": r_current,
            "needle_r": logic_state.needle_r,
            "press_count": logic_state.press_count,
            "counted_presses": logic_state.counted_presses,
            "has_distinct_readings": logic_state.has_distinct_readings,
            "separated_presses": logic_state.separated_presses,
            "within_press_budget": within_press_budget,
            "setting_l_mm": setting_l,
            "setting_r_mm": setting_r,
            "speed_l": speed_l,
            "speed_r": speed_r,
            "grippers_open": grippers_open,
            "knobs_still": knobs_still,
            "zero_gate": self.zero_gate_triggered,
            "zero_gate_reason": self.zero_gate_reason,
            "hold_progress": f"{self.consecutive_success_steps}/{self.hold_steps}",
        }

        return ScoreResult(score=score, success=is_success, details=details)
