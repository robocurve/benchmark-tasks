"""Pure scoring logic for receipt_printer_roll_reload_v2 (end state plus the run's latched facts)."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np

from geometry import (
    BAY_OPEN_DEG,
    COVER_CLOSED_TOL_DEG,
    FEED_TIP_MAX_ABS_Y,
    FEED_TIP_MAX_X,
    FEED_TIP_MAX_Z,
    LIFT_MIN_M,
    OPEN_HOLD_STEPS,
    ROLL_AXIS_LOCAL,
    ROLL_CENTRE_Z,
    ROLL_START_CENTRE_Z,
    ROLL_UP_LOCAL,
    SEAT_CENTRE_Z,
    SEAT_MAX_AXIS_DEG,
    SEAT_MAX_TILT_DEG,
    SEAT_TOL_X_M,
    SEAT_TOL_Y_M,
    SEAT_TOL_Z_M,
    SEAT_XY,
    STAND_HALF,
    STAND_MAX_TILT_DEG,
    STAND_XY,
    STAND_Z_BAND_M,
    TABLE_TOP,
    TAIL_TIP_LOCAL,
)


def rotation_from_quat(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))))))


def roll_centre(pos: Sequence[float], quat: Sequence[float]) -> np.ndarray:
    """The roll's axis midpoint in the env frame (z above the world origin, so table top is TABLE_TOP)."""
    return np.asarray(pos, dtype=float) + rotation_from_quat(quat) @ np.array([0.0, 0.0, ROLL_CENTRE_Z])


def tail_tip(pos: Sequence[float], quat: Sequence[float]) -> np.ndarray:
    """The paper's free end (centre of its tip edge) in the env frame."""
    return np.asarray(pos, dtype=float) + rotation_from_quat(quat) @ np.array(TAIL_TIP_LOCAL)


def tail_direction(quat: Sequence[float]) -> np.ndarray:
    return rotation_from_quat(quat) @ np.array([1.0, 0.0, 0.0])


def axis_off_y_deg(quat: Sequence[float]) -> float:
    """Angle between the roll's axis and the world y line (either way)."""
    axis = rotation_from_quat(quat) @ np.array(ROLL_AXIS_LOCAL)
    return min(_angle_deg(axis, np.array([0.0, 1.0, 0.0])), _angle_deg(axis, np.array([0.0, -1.0, 0.0])))


def tilt_deg(quat: Sequence[float]) -> float:
    """Angle between the roll's up direction (opposite its tail plane) and vertical."""
    return _angle_deg(rotation_from_quat(quat) @ np.array(ROLL_UP_LOCAL), np.array([0.0, 0.0, 1.0]))


def seated(pos: Sequence[float], quat: Sequence[float]) -> bool:
    """Roll centre in seat's tolerance box, axis along y, level. Says nothing about which way the free end runs."""
    c = roll_centre(pos, quat)
    return (
        abs(c[0] - SEAT_XY[0]) <= SEAT_TOL_X_M
        and abs(c[1] - SEAT_XY[1]) <= SEAT_TOL_Y_M
        and abs(c[2] - TABLE_TOP - SEAT_CENTRE_Z) <= SEAT_TOL_Z_M
        and axis_off_y_deg(quat) <= SEAT_MAX_AXIS_DEG
        and tilt_deg(quat) <= SEAT_MAX_TILT_DEG
    )


def feeds(pos: Sequence[float], quat: Sequence[float]) -> bool:
    """The free end lies on the chute at least 2 cm outside the front face and in line with the exit notch."""
    tip = tail_tip(pos, quat)
    return (
        float(tip[0]) <= FEED_TIP_MAX_X
        and abs(float(tip[1]) - SEAT_XY[1]) <= FEED_TIP_MAX_ABS_Y
        and float(tip[2]) - TABLE_TOP <= FEED_TIP_MAX_Z
    )


def tail_points_back(quat: Sequence[float]) -> bool:
    """The free end runs toward +x (toward the rear wall, away from the exit)."""
    return float(tail_direction(quat)[0]) > 0.0


def on_stand(pos: Sequence[float], quat: Sequence[float]) -> bool:
    c = roll_centre(pos, quat)
    dz = float(c[2]) - TABLE_TOP - ROLL_START_CENTRE_Z
    return (
        abs(float(c[0]) - STAND_XY[0]) <= STAND_HALF[0]
        and abs(float(c[1]) - STAND_XY[1]) <= STAND_HALF[1]
        and STAND_Z_BAND_M[0] <= dz <= STAND_Z_BAND_M[1]
        and tilt_deg(quat) <= STAND_MAX_TILT_DEG
    )


def lid_open_deg(joint_val: float) -> float:
    """Lid opening angle in degrees (positive joint angle in mechkit opens the front upward)."""
    return math.degrees(float(joint_val))


def cover_closed(joint_val: float) -> bool:
    """Cover closed within 3 degrees of rest (0 degrees)."""
    return abs(lid_open_deg(joint_val)) <= COVER_CLOSED_TOL_DEG + 1e-4


def bay_open(joint_val: float) -> bool:
    """Bay open when lid angle >= 60 degrees."""
    return lid_open_deg(joint_val) >= BAY_OPEN_DEG - 1e-4


def lamp_state(pos: Sequence[float], quat: Sequence[float], lid_joint_val: float) -> str:
    """The printer's status lamp (display only): green = roll seated with its free end through the exit and the cover
    closed; amber = roll in the seat but its free end does not feed; red otherwise (cover open or no roll seated)."""
    if seated(pos, quat):
        if feeds(pos, quat) and cover_closed(lid_joint_val):
            return "green"
        if not feeds(pos, quat):
            return "amber"
    return "red"


def evaluate_state(
    roll_pos: Sequence[float],
    roll_quat: Sequence[float],
    lid_joint_val: float,
    open_run: int,
    max_lift: float,
    fixture_moved: bool,
    released: bool,
    still: bool,
) -> tuple[float, bool, str]:
    """End state plus run's latched facts: open_run = longest run of control steps with lid open >= 60 deg,
    max_lift = highest roll lifted above start (m), fixture_moved = printer or stand shifted > 5 mm."""
    if fixture_moved:
        return 0.0, False, "fault: the printer or the stand moved"
    is_seated = seated(roll_pos, roll_quat)
    if is_seated and tail_points_back(roll_quat):
        return 0.0, False, "roll seated with its free end toward the rear wall"
    if is_seated and feeds(roll_pos, roll_quat):
        if cover_closed(lid_joint_val):
            if released and still:
                return 1.0, True, "roll seated, paper feeds out of the front, cover closed; released and still"
            return 0.75, False, "roll seated and feeding, cover closed; not released or not still"
        return 0.5, False, "roll seated and feeding, cover not closed"
    return 0.0, False, "roll not loaded"
