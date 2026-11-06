"""Scorer for the rolling firmware update task. Pure Python + NumPy.

Evaluates:
- 4-level partial credit ladder (0.25, 0.50, 0.75, 1.00).
- Zero-gates: OUTAGE latched, fixture displaced > 5 mm, fixture tilted > 5 deg.
- Success hold requirement: 1.0 s (20 control steps at 20 Hz).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

import geometry as g
import logic

FIXTURE_DISPLACEMENT_GATE_M = 0.005  # 5 mm
FIXTURE_TILT_GATE_RAD = np.radians(5.0)  # 5 degrees
FIXTURE_STILL_LIN_VEL_M_S = 0.005  # 5 mm/s
FIXTURE_STILL_ANG_VEL_RAD_S = 0.2  # 0.2 rad/s
SUCCESS_HOLD_S = 1.0
CONTROL_HZ = 20.0
HOLD_STEPS = int(round(SUCCESS_HOLD_S * CONTROL_HZ))  # 20 steps


@dataclass
class ScoreResult:
    score: float
    success: bool
    details: dict = field(default_factory=dict)


class Scorer:
    """Stateful scorer for rolling-firmware-update."""

    def __init__(self, hold_s: float = SUCCESS_HOLD_S, hz: float = CONTROL_HZ) -> None:
        self.hold_steps = int(round(hold_s * hz))
        self.logic = logic.RollingFirmwareLogic(hz=hz)
        self.reset()

    def reset(self) -> None:
        self.logic.reset()
        self.consecutive_success_steps = 0
        self.highest_partial_tier = 0.0
        self.zero_gate_triggered = False
        self.zero_gate_reason = ""

    def update(
        self,
        fixture_pos: tuple[float, float, float] | np.ndarray,
        fixture_quat: tuple[float, float, float, float] | np.ndarray,
        fixture_lin_vel: tuple[float, float, float] | np.ndarray,
        fixture_ang_vel: tuple[float, float, float] | np.ndarray,
        fixture_sliders: dict[str, float],
        grippers_open: bool,
    ) -> ScoreResult:
        """Step the scorer and return ScoreResult."""
        pos_fix = np.asarray(fixture_pos, dtype=float)
        fix_shift = float(np.linalg.norm(pos_fix - np.asarray(g.BASE_POS, dtype=float)))

        # Tilt angle from identity quaternion [x, y, z, w]
        q = np.asarray(fixture_quat, dtype=float)
        # z-axis tilt from w component: angle = 2 * arccos(clip(w, -1, 1))
        # More generally tilt of normal [0, 0, 1]:
        w = np.clip(q[3], -1.0, 1.0)
        # Total rotation angle
        rot_angle = 2.0 * np.arccos(abs(w))

        lin_speed = float(np.linalg.norm(fixture_lin_vel))
        ang_speed = float(np.linalg.norm(fixture_ang_vel))
        fixture_still = (lin_speed <= FIXTURE_STILL_LIN_VEL_M_S) and (ang_speed <= FIXTURE_STILL_ANG_VEL_RAD_S)

        # Sliders:
        # Door slider travel is along X: joint pos is in [-DOOR_TRAVEL, 0.0]
        # Deflection is -joint_pos
        door_a_q = float(fixture_sliders.get("door_a", 0.0))
        door_b_q = float(fixture_sliders.get("door_b", 0.0))
        deflection_a = -door_a_q
        deflection_b = -door_b_q

        # Button sliders travel along Z: joint pos is in [-BUTTON_TRAVEL, 0.0]
        btn_update_a_z = float(fixture_sliders.get("btn_update_a", 0.0))
        btn_update_b_z = float(fixture_sliders.get("btn_update_b", 0.0))
        btn_switch_z = float(fixture_sliders.get("btn_switch", 0.0))

        # Advance state machine
        logic_state = self.logic.step(
            door_a_deflection=deflection_a,
            door_b_deflection=deflection_b,
            btn_update_a_z=btn_update_a_z,
            btn_update_b_z=btn_update_b_z,
            btn_switch_z=btn_switch_z,
        )

        # Check zero-gates
        if self.logic.outage:
            self.zero_gate_triggered = True
            self.zero_gate_reason = "OUTAGE: active unit went offline (updating)"

        if fix_shift > FIXTURE_DISPLACEMENT_GATE_M:
            self.zero_gate_triggered = True
            self.zero_gate_reason = f"Fixture moved {fix_shift * 1000:.1f} mm > {FIXTURE_DISPLACEMENT_GATE_M * 1000:.1f} mm"

        if rot_angle > FIXTURE_TILT_GATE_RAD:
            self.zero_gate_triggered = True
            self.zero_gate_reason = f"Fixture tilted {np.degrees(rot_angle):.1f} deg > {np.degrees(FIXTURE_TILT_GATE_RAD):.1f} deg"

        if self.zero_gate_triggered:
            return ScoreResult(
                score=0.0,
                success=False,
                details={
                    "zero_gate": True,
                    "reason": self.zero_gate_reason,
                    "logic": logic_state,
                },
            )

        # Evaluate partial credit tiers
        # Level 0.25: a door deflected >= 18 mm or any counted press
        if (
            deflection_a >= g.DOOR_DEFLECTION_THRESHOLD
            or deflection_b >= g.DOOR_DEFLECTION_THRESHOLD
            or self.logic.raw_button_press_count >= 1
        ):
            self.highest_partial_tier = max(self.highest_partial_tier, 0.25)

        # Level 0.50 (order-agnostic): either unit updated to v1 with no outage
        if (self.logic.unit_a.version == 1 or self.logic.unit_b.version == 1) and not self.logic.outage:
            self.highest_partial_tier = max(self.highest_partial_tier, 0.50)

        # Level 0.75 (order-agnostic): a unit at v1 AND that unit carrying the load (active), with no outage
        unit_a_v1_active = (self.logic.unit_a.version == 1 and self.logic.unit_a.active)
        unit_b_v1_active = (self.logic.unit_b.version == 1 and self.logic.unit_b.active)
        if (unit_a_v1_active or unit_b_v1_active) and not self.logic.outage:
            self.highest_partial_tier = max(self.highest_partial_tier, 0.75)

        # Level 1.00 requirements (must hold simultaneously for hold_steps):
        # 1. Both units updated (v1)
        # 2. No outage (LOAD green)
        # 3. Both doors released (< 2 mm deflection)
        # 4. Both grippers open
        # 5. Everything still
        level_1_met = (
            self.logic.unit_a.version == 1
            and self.logic.unit_b.version == 1
            and not self.logic.outage
            and deflection_a < g.DOOR_RELEASED_THRESHOLD
            and deflection_b < g.DOOR_RELEASED_THRESHOLD
            and grippers_open
            and fixture_still
        )

        if level_1_met:
            self.consecutive_success_steps += 1
        else:
            self.consecutive_success_steps = 0

        success = self.consecutive_success_steps >= self.hold_steps
        current_score = 1.0 if success else self.highest_partial_tier

        return ScoreResult(
            score=current_score,
            success=success,
            details={
                "zero_gate": False,
                "highest_partial_tier": self.highest_partial_tier,
                "consecutive_success_steps": self.consecutive_success_steps,
                "level_1_met": level_1_met,
                "deflection_a_mm": deflection_a * 1000,
                "deflection_b_mm": deflection_b * 1000,
                "grippers_open": grippers_open,
                "fixture_still": fixture_still,
                "logic": logic_state,
            },
        )
