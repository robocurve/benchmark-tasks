"""State machine, chemical safety verification, and scoring logic for segregate-incompatible-cleaners-in-cabinet."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
import numpy as np

import geometry as g


def quat_to_z_axis(quat_xyzw) -> np.ndarray:
    """Return unit vector of the body's local Z axis in world frame."""
    q = np.asarray(quat_xyzw, dtype=float)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([0.0, 0.0, 1.0])
    x, y, z, w = q / norm
    return np.array([
        2 * (x * z + y * w),
        2 * (y * z - x * w),
        1 - 2 * (x * x + y * y),
    ])


def tilt_angle_deg(quat_xyzw) -> float:
    """Return tilt angle in degrees of body Z axis away from world +Z."""
    z_axis = quat_to_z_axis(quat_xyzw)
    cos_theta = np.clip(z_axis[2], -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_theta)))


def is_in_bay(
    pos: np.ndarray,
    tilt: float,
    bay_pos: tuple[float, float, float],
    max_tilt: float,
) -> bool:
    """Check if bottle center is inside recessed well with acceptable tilt and seating."""
    d_xy = math.hypot(pos[0] - bay_pos[0], pos[1] - bay_pos[1])
    d_z = abs(pos[2] - bay_pos[2])
    return (d_xy <= g.TOL_WELL_XY) and (d_z <= g.TOL_WELL_Z) and (tilt <= max_tilt)


@dataclass
class CleanersCabinetState:
    bleach_in_left: bool = False
    bleach_in_right: bool = False
    ammonia_in_left: bool = False
    ammonia_in_right: bool = False
    bleach_moved: bool = False
    ammonia_moved: bool = False
    both_moved: bool = False
    one_placed: bool = False
    both_placed_separate: bool = False
    both_placed_same_bay: bool = False
    bottles_too_close: bool = False
    incompatible_mix_violation: bool = False
    bottles_still: bool = False
    arms_clear: bool = False
    hold_time_s: float = 0.0
    solved: bool = False
    score: float = 0.0
    violation_reason: str = ""


class CleanersCabinetLogic:
    def __init__(self, variant: str = "A", hold_s: float = g.HOLD_TIME_S, hz: float = 20.0) -> None:
        self.variant = variant
        self.hold_s = hold_s
        self.dt = 1.0 / hz
        self.reset()

    def reset(self) -> None:
        self.incompatible_mix_violation = False
        self.hold_time_s = 0.0
        self.solved = False
        self.violation_reason = ""

    def step(
        self,
        dt: float,
        bleach_pos: tuple[float, float, float] | np.ndarray,
        bleach_quat: tuple[float, float, float, float] | np.ndarray,
        bleach_vel: tuple[float, float, float] | np.ndarray,
        ammonia_pos: tuple[float, float, float] | np.ndarray,
        ammonia_quat: tuple[float, float, float, float] | np.ndarray,
        ammonia_vel: tuple[float, float, float] | np.ndarray,
        arm_tcps: dict[str, tuple[float, float, float]] | None = None,
    ) -> CleanersCabinetState:
        bp = np.asarray(bleach_pos, dtype=float)
        ap = np.asarray(ammonia_pos, dtype=float)
        bv = np.asarray(bleach_vel, dtype=float)
        av = np.asarray(ammonia_vel, dtype=float)

        b_tilt = tilt_angle_deg(bleach_quat)
        a_tilt = tilt_angle_deg(ammonia_quat)

        # Staging references depending on variant
        if self.variant == "A":
            bleach_start = np.array(g.STAGING_LEFT_POS)
            ammonia_start = np.array(g.STAGING_RIGHT_POS)
        else:
            ammonia_start = np.array(g.STAGING_LEFT_POS)
            bleach_start = np.array(g.STAGING_RIGHT_POS)

        # 1. Check movement from staging
        bleach_moved = float(np.linalg.norm(bp - bleach_start)) > g.MOVED_THRESHOLD_DIST
        ammonia_moved = float(np.linalg.norm(ap - ammonia_start)) > g.MOVED_THRESHOLD_DIST
        both_moved = bleach_moved and ammonia_moved

        # 2. Check bay containment (tilt < 6 deg for placed status)
        b_left_6 = is_in_bay(bp, b_tilt, g.BAY_LEFT_POS, g.MAX_TILT_PLACED_DEG)
        b_right_6 = is_in_bay(bp, b_tilt, g.BAY_RIGHT_POS, g.MAX_TILT_PLACED_DEG)
        a_left_6 = is_in_bay(ap, a_tilt, g.BAY_LEFT_POS, g.MAX_TILT_PLACED_DEG)
        a_right_6 = is_in_bay(ap, a_tilt, g.BAY_RIGHT_POS, g.MAX_TILT_PLACED_DEG)

        # Bay containment (tilt < 5 deg for 1.00 full success)
        b_left_5 = is_in_bay(bp, b_tilt, g.BAY_LEFT_POS, g.MAX_TILT_SUCCESS_DEG)
        b_right_5 = is_in_bay(bp, b_tilt, g.BAY_RIGHT_POS, g.MAX_TILT_SUCCESS_DEG)
        a_left_5 = is_in_bay(ap, a_tilt, g.BAY_LEFT_POS, g.MAX_TILT_SUCCESS_DEG)
        a_right_5 = is_in_bay(ap, a_tilt, g.BAY_RIGHT_POS, g.MAX_TILT_SUCCESS_DEG)

        one_placed = b_left_6 or b_right_6 or a_left_6 or a_right_6

        # Either assignment is 100% legal:
        # "bleach left, ammonia right" OR "bleach right, ammonia left"
        both_placed_separate_6 = (b_left_6 and a_right_6) or (b_right_6 and a_left_6)
        both_placed_separate_5 = (b_left_5 and a_right_5) or (b_right_5 and a_left_5)

        # 3. Check for hazardous proximity or same-bay mixing (latched violation)
        both_placed_same_bay = (b_left_6 and a_left_6) or (b_right_6 and a_right_6)
        dist_between = float(np.linalg.norm(bp - ap))
        bottles_too_close = dist_between < g.MIN_BOTTLE_SEPARATION_VIOLATION

        if both_placed_same_bay or bottles_too_close:
            self.incompatible_mix_violation = True
            if not self.violation_reason:
                if both_placed_same_bay:
                    self.violation_reason = "Hazard violation: Incompatible cleaners placed together in the same bay (toxic chloramine gas hazard)."
                else:
                    self.violation_reason = f"Hazard violation: Incompatible cleaners placed within {dist_between*1000:.1f} mm < {g.MIN_BOTTLE_SEPARATION_VIOLATION*1000:.1f} mm."

        # 4. Check still velocity
        b_speed = float(np.linalg.norm(bv))
        a_speed = float(np.linalg.norm(av))
        bottles_still = (b_speed <= g.MAX_STILL_SPEED) and (a_speed <= g.MAX_STILL_SPEED)

        # 5. Check arms clear / retracted to standby
        arms_clear = True
        if arm_tcps is not None:
            for arm, tcp in arm_tcps.items():
                tcp_arr = np.asarray(tcp, dtype=float)
                d_b = float(np.linalg.norm(tcp_arr[:2] - bp[:2]))
                d_a = float(np.linalg.norm(tcp_arr[:2] - ap[:2]))
                if tcp_arr[2] < (g.STANDBY_Z - 0.040) and min(d_b, d_a) < g.ARMS_CLEAR_DIST:
                    arms_clear = False
                    break

        # 6. Check full 1.00 hold condition
        can_hold = (
            both_placed_separate_5
            and bottles_still
            and arms_clear
            and not self.incompatible_mix_violation
        )

        if can_hold:
            self.hold_time_s += dt
            if self.hold_time_s >= self.hold_s:
                self.solved = True
        else:
            self.hold_time_s = 0.0

        # 7. Scoring ladder
        if self.incompatible_mix_violation:
            # Latched violation caps score at 0.25 max
            score = 0.25 if (one_placed or both_moved) else 0.0
        elif self.solved:
            score = 1.00
        elif both_placed_separate_6:
            score = 0.75
        elif both_moved:
            score = 0.50
        elif one_placed:
            score = 0.25
        else:
            score = 0.00

        return CleanersCabinetState(
            bleach_in_left=b_left_6,
            bleach_in_right=b_right_6,
            ammonia_in_left=a_left_6,
            ammonia_in_right=a_right_6,
            bleach_moved=bleach_moved,
            ammonia_moved=ammonia_moved,
            both_moved=both_moved,
            one_placed=one_placed,
            both_placed_separate=both_placed_separate_6,
            both_placed_same_bay=both_placed_same_bay,
            bottles_too_close=bottles_too_close,
            incompatible_mix_violation=self.incompatible_mix_violation,
            bottles_still=bottles_still,
            arms_clear=arms_clear,
            hold_time_s=self.hold_time_s,
            solved=self.solved,
            score=score,
            violation_reason=self.violation_reason,
        )
