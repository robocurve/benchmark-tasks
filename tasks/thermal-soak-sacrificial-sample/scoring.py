"""Scorer for the thermal soak task. Pure Python.

Scoring ladder (judged on sample 2; sample 1 is expendable):
- 0.25: HEAT counted or a sample moved (> 10 mm)
- 0.50: sample 1 placed on plate, lifted, put on cool pad (any core)
- 0.75: sample 2 on cool pad with core in 45-80 deg C
- 1.00: sample 2 on cool pad with core 60-70 deg C at end (held 1.0 s / 20 steps,
        core settled: |dTc/dt| < 0.2 deg C/s), heater OFF, both grippers open.
Zero-gate: any sample core > 90 deg C at any time (FIRE), sample off table, or fixture moved > 5 mm.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import geometry as g
from logic import ThermalLogic, ThermalState


@dataclass
class ScoreResult:
    score: float
    success: bool
    details: dict[str, Any] = field(default_factory=dict)


def is_on_hot_plate(pos: tuple[float, float, float], lin_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> bool:
    """Check if a sample block is centered on the hot plate pad within 30 mm and at rest."""
    dx = pos[0] - g.PAD_POS[0]
    dy = pos[1] - g.PAD_POS[1]
    dist_xy = math.hypot(dx, dy)
    expected_z = g.PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0
    z_ok = abs(pos[2] - expected_z) <= 0.012
    v_norm = math.hypot(lin_vel[0], lin_vel[1], lin_vel[2])
    rest_ok = v_norm <= 0.02
    return dist_xy <= 0.030 and z_ok and rest_ok


def is_on_cool_pad(pos: tuple[float, float, float], lin_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> bool:
    """Check if a sample block is resting within the cool pad bounds."""
    dx = abs(pos[0] - g.COOL_PAD_POS[0])
    dy = abs(pos[1] - g.COOL_PAD_POS[1])
    expected_z = g.COOL_PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0
    z_ok = abs(pos[2] - expected_z) <= 0.012
    v_norm = math.hypot(lin_vel[0], lin_vel[1], lin_vel[2])
    rest_ok = v_norm <= 0.02
    return dx <= (g.COOL_PAD_SIZE[0] / 2.0 - 0.010) and dy <= (g.COOL_PAD_SIZE[1] / 2.0 - 0.010) and z_ok and rest_ok


def is_off_table(pos: tuple[float, float, float]) -> bool:
    """Check if an object has fallen off the table."""
    return pos[2] < (g.TOP - 0.05) or pos[0] < 0.05 or pos[0] > 0.60 or abs(pos[1]) > 0.50


class Scorer:
    """Evaluates the thermal soak task rules, hidden thermal ODEs, and scoring ladder."""

    def __init__(self) -> None:
        self.logic = ThermalLogic()
        self.initial_fixture_pos: tuple[float, float, float] | None = None
        self.consecutive_success_steps = 0
        self.latched_score = 0.0

    def reset(self) -> None:
        self.logic.reset()
        self.initial_fixture_pos = None
        self.consecutive_success_steps = 0
        self.latched_score = 0.0

    def update(
        self,
        fixture_pos: tuple[float, float, float],
        fixture_sliders: dict[str, float] | None,
        s1_data: tuple,
        s2_data: tuple,
        grippers_open: bool,
        dt: float = 0.05,
    ) -> ScoreResult:
        if self.initial_fixture_pos is None:
            self.initial_fixture_pos = tuple(float(x) for x in fixture_pos)

        s1_pos, _s1_quat, s1_vel, _s1_ang_vel = s1_data
        s2_pos, _s2_quat, s2_vel, _s2_ang_vel = s2_data

        # Read button slider displacements
        heat_z = 0.0
        off_z = 0.0
        if fixture_sliders:
            heat_z = fixture_sliders.get("heat_btn", 0.0)
            off_z = fixture_sliders.get("off_btn", 0.0)

        # Detect spatial conditions
        s1_on_plate = is_on_hot_plate(s1_pos, s1_vel)
        s1_on_cool = is_on_cool_pad(s1_pos, s1_vel)
        s2_on_plate = is_on_hot_plate(s2_pos, s2_vel)
        s2_on_cool = is_on_cool_pad(s2_pos, s2_vel)

        s1_moved = math.dist(s1_pos, g.START_POS_1) > 0.010
        s2_moved = math.dist(s2_pos, g.START_POS_2) > 0.010

        # Step thermal logic
        # Preliminary check if success conditions are currently held
        tentative_success = (self.consecutive_success_steps >= 20)
        thermal = self.logic.step(
            dt,
            heat_z,
            off_z,
            s1_on_plate,
            s1_on_cool,
            s2_on_plate,
            s2_on_cool,
            is_success=tentative_success,
        )

        # Zero-gate checks
        off_table = is_off_table(s1_pos) or is_off_table(s2_pos)
        fix_dist = math.dist(fixture_pos, self.initial_fixture_pos)
        fixture_displaced = fix_dist > 0.005
        fire = thermal.fire_tripped

        zero_gate = off_table or fixture_displaced or fire

        # Ladder evaluation
        score = 0.0
        success = False

        if not zero_gate:
            # 0.25: HEAT counted or a sample moved
            if thermal.heat_count > 0 or s1_moved or s2_moved:
                score = max(score, 0.25)

            # 0.50: sample 1 placed on plate, lifted, put on cool pad
            if thermal.s1_complete:
                score = max(score, 0.50)

            # 0.75: sample 2 on cool pad with core in 45-80 deg C
            if s2_on_cool and (45.0 <= thermal.tc2 <= 80.0):
                score = max(score, 0.75)

            # 1.00: sample 2 on cool pad with core 60-70 deg C at end (held 1 s,
            # core settled: |dTc/dt| < 0.2 deg C/s), heater OFF, both grippers open.
            core_in_target = (g.CORE_BAND_MIN <= thermal.tc2 <= g.CORE_BAND_MAX)
            core_settled = (abs(thermal.dtc2_dt) < g.CORE_SETTLED_RATE)
            heater_off = not thermal.heater_on

            full_conditions = (
                s2_on_cool
                and core_in_target
                and core_settled
                and heater_off
                and grippers_open
                and thermal.s1_complete
            )

            if full_conditions:
                self.consecutive_success_steps += 1
            else:
                self.consecutive_success_steps = 0

            if self.consecutive_success_steps >= int(g.SUCCESS_HOLD_S / dt):
                score = 1.00
                success = True
                thermal.status_lamp = "green"

            self.latched_score = max(self.latched_score, min(score, 0.75))
            score = max(score, self.latched_score)
        else:
            self.latched_score = 0.0
            self.consecutive_success_steps = 0
            score = 0.0
            success = False

        details = {
            "score": score,
            "success": success,
            "plate_temp": thermal.tp,
            "heater_on": thermal.heater_on,
            "heat_count": thermal.heat_count,
            "off_count": thermal.off_count,
            "s1_surf_temp": thermal.ts1,
            "s1_core_temp": thermal.tc1,
            "s2_surf_temp": thermal.ts2,
            "s2_core_temp": thermal.tc2,
            "s2_dtc_dt": thermal.dtc2_dt,
            "s1_on_plate": s1_on_plate,
            "s1_on_cool": s1_on_cool,
            "s2_on_plate": s2_on_plate,
            "s2_on_cool": s2_on_cool,
            "s1_complete": thermal.s1_complete,
            "fire_tripped": thermal.fire_tripped,
            "fixture_displaced": fixture_displaced,
            "off_table": off_table,
            "bar_count": thermal.bar_count,
            "status_lamp": thermal.status_lamp,
            "hold_steps": self.consecutive_success_steps,
        }

        return ScoreResult(score=score, success=success, details=details)
