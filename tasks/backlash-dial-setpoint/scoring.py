"""Scoring ladder and verification for backlash dial setpoint. Pure Python + NumPy.

Ladder:
  0.0:  Nothing touched (knobs near start positions, neither locked).
  0.25: A knob moved by >= 5 mm.
  0.50: One dial locked in band (displayed value within setpoint +- 2.5 mm).
  0.75: Both dials locked in band.
  1.0:  Both dials locked in band, both grippers open,
        both knobs still (speed <= 5 mm/s), held for 20 consecutive control steps (1.0 s at 20 Hz).

Zero-gates:
  - Fixture base displaced > 5 mm -> 0.0
  - Third lock attempt on a dial -> 0.0
  - Second lock outside band on a dial -> 0.0
  - Any knob forced out of its channel (lateral displacement > 10 mm or lifted out) -> 0.0
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import geometry as g
import logic

# Thresholds
KNOB_MOVED_THRESHOLD_MM = 5.0  # 5 mm
STILL_LIN_VEL = 0.005  # m/s (5 mm/s)
FIXTURE_SHIFT_LIMIT = 0.005  # 5 mm
KNOB_LATERAL_LIMIT = 0.010  # 10 mm lateral deviation from channel centerline
KNOB_MAX_Z = g.BASE_TOP_Z + g.TRACK_WALL_H + 0.015  # 15 mm above wall top
KNOB_MIN_Z = g.TOP - 0.050  # table drop limit


@dataclass(frozen=True)
class ScoreResult:
    score: float
    success: bool
    details: dict


class Scorer:
    """Stateful scorer evaluating the 5-level partial credit ladder and zero-gates."""

    def __init__(self, hold_s: float = 1.0, hz: float = 20.0) -> None:
        self.hold_steps = int(round(hold_s * hz))
        self.logic = logic.BacklashDialLogic()
        self.reset()

    def reset(self) -> None:
        self.logic.reset()
        self.consecutive_success_steps = 0
        self.knob_moved = False
        self.zero_gate_triggered = False
        self.zero_gate_reason = ""
        self.latched_score = 0.0

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

        # Step the button and backlash logic
        btn_l_z = float(fixture_sliders.get("lock_btn_left", 0.0))
        btn_r_z = float(fixture_sliders.get("lock_btn_right", 0.0))
        logic_state = self.logic.step(setting_l, setting_r, btn_l_z, btn_r_z)

        # Zero-gate: fixture displacement
        if fix_shift > FIXTURE_SHIFT_LIMIT:
            self.zero_gate_triggered = True
            self.zero_gate_reason = f"Fixture shifted {fix_shift * 1000.0:.1f} mm > {FIXTURE_SHIFT_LIMIT * 1000.0:.1f} mm"

        # Zero-gate: logic zero-gate (lock attempt violations)
        if logic_state.zero_gate:
            self.zero_gate_triggered = True
            self.zero_gate_reason = logic_state.zero_gate_reason

        # Zero-gate: channel derailment
        lat_l = abs(pos_l[1] - g.TRACK_Y["left"])
        lat_r = abs(pos_r[1] - g.TRACK_Y["right"])
        if lat_l > KNOB_LATERAL_LIMIT or lat_r > KNOB_LATERAL_LIMIT:
            self.zero_gate_triggered = True
            self.zero_gate_reason = f"Knob left channel laterally (l={lat_l*1000:.1f}mm, r={lat_r*1000:.1f}mm)"

        if pos_l[2] > KNOB_MAX_Z or pos_r[2] > KNOB_MAX_Z or pos_l[2] < KNOB_MIN_Z or pos_r[2] < KNOB_MIN_Z:
            self.zero_gate_triggered = True
            self.zero_gate_reason = "Knob lifted out or dropped below table"

        # Movement tracking (>= 5 mm from start)
        dl = abs(setting_l - g.KNOB_START_SETTING_MM["left"])
        dr = abs(setting_r - g.KNOB_START_SETTING_MM["right"])
        if dl >= KNOB_MOVED_THRESHOLD_MM - 1e-4 or dr >= KNOB_MOVED_THRESHOLD_MM - 1e-4:
            self.knob_moved = True

        # Ladder evaluation
        current_score = 0.0
        success = False

        if not self.zero_gate_triggered:
            if self.knob_moved:
                current_score = max(current_score, 0.25)

            locked_l = logic_state.locked_in_band["left"]
            locked_r = logic_state.locked_in_band["right"]

            if locked_l or locked_r:
                current_score = max(current_score, 0.50)

            if locked_l and locked_r:
                current_score = max(current_score, 0.75)

            # Hold check for 1.00
            speed_l = float(np.linalg.norm(vel_l))
            speed_r = float(np.linalg.norm(vel_r))
            knobs_still = speed_l <= STILL_LIN_VEL and speed_r <= STILL_LIN_VEL

            at_1_00 = (
                locked_l
                and locked_r
                and grippers_open
                and knobs_still
            )

            if at_1_00:
                self.consecutive_success_steps += 1
            else:
                self.consecutive_success_steps = 0

            if self.consecutive_success_steps >= self.hold_steps:
                current_score = 1.00
                success = True

            self.latched_score = max(self.latched_score, current_score)
        else:
            self.latched_score = 0.0
            self.consecutive_success_steps = 0

        final_score = 1.00 if success else self.latched_score

        details = {
            "setting_left": setting_l,
            "setting_right": setting_r,
            "displayed_left": logic_state.displayed["left"],
            "displayed_right": logic_state.displayed["right"],
            "locked_left": logic_state.locked["left"],
            "locked_right": logic_state.locked["right"],
            "locked_in_band_left": logic_state.locked_in_band["left"],
            "locked_in_band_right": logic_state.locked_in_band["right"],
            "has_reversed_left": logic_state.has_reversed["left"],
            "has_reversed_right": logic_state.has_reversed["right"],
            "lock_attempts_left": logic_state.lock_attempts["left"],
            "lock_attempts_right": logic_state.lock_attempts["right"],
            "outside_band_locks_left": logic_state.outside_band_locks["left"],
            "outside_band_locks_right": logic_state.outside_band_locks["right"],
            "knob_moved": self.knob_moved,
            "zero_gate_triggered": self.zero_gate_triggered,
            "zero_gate_reason": self.zero_gate_reason,
            "consecutive_success_steps": self.consecutive_success_steps,
        }

        return ScoreResult(score=final_score, success=success, details=details)
