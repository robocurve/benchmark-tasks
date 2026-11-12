"""Pure scoring logic for pressure_pan_vent_then_lid (end state plus the run's latched facts)."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np

from geometry import (
    ABOVE_PAN_RADIUS_M,
    BYPASS_RISE_M,
    KEEPOUT_FINGER_MARGIN_M,
    KEEPOUT_RADIUS_M,
    KEEPOUT_Z_MAX_M,
    KEEPOUT_Z_MIN_M,
    LID_DISC,
    LID_KNOB,
    LID_LUG,
    LID_REST_Z_M,
    LIFT_MIN_M,
    LIFT_TILT_DEG,
    PAN_HANDLE,
    PAN_HEIGHT_M,
    PAN_RADIUS_M,
    RELEASE_MIN_DIST_M,
    SEAT_MAX_OFFCENTRE_M,
    SEAT_MAX_TILT_DEG,
    SEAT_MAX_Z_ERR_M,
    TABLE_TOP,
    TRIVET_TOP_M,
    TRIVET_XY,
    UNLOCK_HOLD_STEPS,
    VALVE_XY,
    VENT_BUTTON,
    VENT_RELEASED_M,
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


def tilt_deg(q: Sequence[float]) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rotation_from_quat(q)[2, 2])))))


def lid_rise(lid_pos: Sequence[float]) -> float:
    """Height of the lid's underside above where it rests on the pan's rim."""
    return float(lid_pos[2]) - TABLE_TOP - LID_REST_Z_M


def in_plume_zone(tcp: Sequence[float]) -> bool:
    """A gripper centre close enough to the valve's axis that a finger is in the keep-out cylinder (z is the world z)."""
    z_above = float(tcp[2]) - TABLE_TOP
    d = math.hypot(float(tcp[0]) - VALVE_XY[0], float(tcp[1]) - VALVE_XY[1])
    return d < KEEPOUT_RADIUS_M + KEEPOUT_FINGER_MARGIN_M and KEEPOUT_Z_MIN_M - 0.013 <= z_above <= KEEPOUT_Z_MAX_M


def _box_distance(point: np.ndarray, centre_world: np.ndarray, rot: np.ndarray, half: np.ndarray) -> float:
    local = (point - centre_world) @ rot
    d = np.maximum(np.abs(local) - half, 0.0)
    return float(np.linalg.norm(d))


def _cyl_distance(point: np.ndarray, base_centre: np.ndarray, radius: float, height: float) -> float:
    """Distance from a point to an upright cylinder standing on base_centre."""
    radial = max(math.hypot(point[0] - base_centre[0], point[1] - base_centre[1]) - radius, 0.0)
    vertical = max(point[2] - (base_centre[2] + height), base_centre[2] - point[2], 0.0)
    return math.hypot(radial, vertical)


def hands_clear(
    tcps: Sequence[Sequence[float]],
    lid_pos: Sequence[float],
    lid_quat: Sequence[float],
    pan_pos: Sequence[float],
    vent_press: float,
    min_dist: float = RELEASE_MIN_DIST_M,
) -> bool:
    """Every gripper centre at least min_dist from the lid (disc, knob, lug), the pan (body and handle) and the button."""
    rot = rotation_from_quat(lid_quat)
    lid = np.asarray(lid_pos, dtype=float)
    pan = np.asarray(pan_pos, dtype=float)
    lid_shapes = [
        (LID_DISC[0], (2 * LID_DISC[1], 2 * LID_DISC[1], LID_DISC[2])),
        (LID_KNOB[0], (2 * LID_KNOB[1], 2 * LID_KNOB[1], LID_KNOB[2])),
        LID_LUG,
    ]
    button_c = pan + np.array(VENT_BUTTON[0]) + np.array([float(vent_press), 0.0, 0.0])
    for tcp in tcps:
        p = np.asarray(tcp, dtype=float)
        for centre, size in lid_shapes:
            if _box_distance(p, lid + rot @ np.asarray(centre, dtype=float), rot, np.asarray(size, dtype=float) / 2) < min_dist:
                return False
        if _cyl_distance(p, pan, PAN_RADIUS_M, PAN_HEIGHT_M) < min_dist:
            return False
        if _box_distance(p, pan + np.array(PAN_HANDLE[0]), np.eye(3), np.array(PAN_HANDLE[1]) / 2) < min_dist:
            return False
        if _box_distance(p, button_c, np.eye(3), np.array(VENT_BUTTON[1]) / 2) < min_dist:
            return False
    return True


def seated_on_trivet(lid_pos: Sequence[float], lid_quat: Sequence[float]) -> bool:
    lx, ly, lz = (float(v) for v in lid_pos)
    off = math.hypot(lx - TRIVET_XY[0], ly - TRIVET_XY[1])
    z_err = lz - (TABLE_TOP + TRIVET_TOP_M)
    return abs(z_err) <= SEAT_MAX_Z_ERR_M and off <= SEAT_MAX_OFFCENTRE_M and tilt_deg(lid_quat) <= SEAT_MAX_TILT_DEG


def lifted_now(lid_pos: Sequence[float], lid_quat: Sequence[float]) -> bool:
    """The lid is 3 cm or more above the rim and within 15 degrees of level."""
    return lid_rise(lid_pos) >= LIFT_MIN_M and tilt_deg(lid_quat) <= LIFT_TILT_DEG


def tilt_fault_now(lid_pos: Sequence[float], lid_quat: Sequence[float], pan_xy: Sequence[float]) -> bool:
    """The lid tilts more than 15 degrees while it is above the pan."""
    above = math.hypot(float(lid_pos[0]) - float(pan_xy[0]), float(lid_pos[1]) - float(pan_xy[1])) <= ABOVE_PAN_RADIUS_M
    return above and lid_rise(lid_pos) > BYPASS_RISE_M and tilt_deg(lid_quat) > LIFT_TILT_DEG


def evaluate_state(
    lid_pos: Sequence[float],
    lid_quat: Sequence[float],
    vent_press: float,
    plume_fault: bool,
    pan_moved: bool,
    bypass: bool,
    tilt_fault: bool,
    lifted: bool,
    unlocked_steps: int,
    hands_ok: bool,
    still: bool,
) -> tuple[float, bool, str]:
    """End state plus the run's latched facts. vent_press is the vent's travel in metres (0 at rest)."""
    if plume_fault:
        return 0.0, False, "fault: a gripper entered the steam zone while the plume was shown"
    if pan_moved:
        return 0.0, False, "fault: the pan moved more than 5 mm"
    cap = 1.0
    if bypass:
        cap = min(cap, 0.25)
    if tilt_fault:
        cap = min(cap, 0.5)
    vent_back = float(vent_press) <= VENT_RELEASED_M
    if seated_on_trivet(lid_pos, lid_quat):
        if vent_back and hands_ok and still:
            base, why = 1.0, "lid flat on the trivet; vent released, hands clear and still"
        else:
            base, why = 0.75, "lid flat on the trivet; vent, hands or stillness not satisfied"
    elif lifted:
        base, why = 0.5, "lid was lifted clear (lock released) but does not rest on the trivet"
    elif unlocked_steps >= UNLOCK_HOLD_STEPS:
        base, why = 0.25, "vent held pressed 1 s but the lid was not lifted"
    else:
        base, why = 0.0, "lock not released and lid not moved"
    s = min(base, cap)
    if s < base:
        why += " (capped: bypass or tilt fault)"
    return s, s >= 1.0, why
