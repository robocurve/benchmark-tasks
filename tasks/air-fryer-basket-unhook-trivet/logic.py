"""Scoring logic and geometry tests for air_fryer_basket_release_button_v2."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np

from geometry import (
    BASKET_BODY_BOXES,
    BASKET_CLEARED_DRAWER_Z,
    BASKET_FLOOR_BACK_X,
    BASKET_FLOOR_HALF_X,
    BASKET_FLOOR_HALF_Y,
    BASKET_HANDLE_BOXES,
    BASKET_LIFT_MIN_M,
    BASKET_START_POS,
    BUTTON_BOXES,
    BUTTON_CLEARANCE_DISPLACEMENT_M,
    BUTTON_OUT_THRESHOLD_M,
    DRAWER_AT,
    DRAWER_BOXES,
    HOT_INFLATION_M,
    HOUSING_BOXES,
    HOUSING_POS,
    MAT_TOP_Z,
    MAT_X_MAX,
    MAT_X_MIN,
    MAT_Y_MAX,
    MAT_Y_MIN,
    MAT_Z_TOL,
    PAD_BOX_HALF,
    RELEASE_MIN_DIST_M,
    STILL_MOTION_M,
    STILL_WINDOW_STEPS,
    TABLE_TOP,
    TILT_MAX_DEG,
)


def rotation_from_quat(q: Sequence[float]) -> np.ndarray:
    """Convert (x, y, z, w) quaternion to 3x3 rotation matrix."""
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def tilt_deg(q: Sequence[float]) -> float:
    """Angle between the body +z axis and world +z in degrees."""
    rot = rotation_from_quat(q)
    cos_angle = max(-1.0, min(1.0, float(rot[2, 2])))
    return math.degrees(math.acos(cos_angle))


def _point_in_inflated_box(local_pt: np.ndarray, box_centre: Sequence[float], box_size: Sequence[float], inflation: float) -> bool:
    """Check if point is inside an axis-aligned box inflated by `inflation` + pad half-width."""
    for i in range(3):
        half = box_size[i] / 2.0 + inflation + PAD_BOX_HALF[i]
        if abs(local_pt[i] - box_centre[i]) > half:
            return False
    return True


def hot_contact(
    pad_positions: Sequence[Sequence[float]],
    basket_pos: Sequence[float],
    basket_quat: Sequence[float],
    drawer_slide: float = 0.0,
) -> bool:
    """Check if any of the fingertip pads overlap any of the 19 HOT collision boxes (inflated 2 mm)."""
    rot_basket = rotation_from_quat(basket_quat)
    b_origin = np.asarray(basket_pos, dtype=float)

    h_origin = np.asarray(HOUSING_POS, dtype=float)
    d_origin = h_origin + np.asarray(DRAWER_AT, dtype=float) + np.array([drawer_slide, 0.0, 0.0], dtype=float)

    for pad in pad_positions:
        p_world = np.asarray(pad, dtype=float)

        # 1. 5 Housing boxes (fixed in world)
        p_h = p_world - h_origin
        for centre, size, is_hot in HOUSING_BOXES:
            if is_hot and _point_in_inflated_box(p_h, centre, size, HOT_INFLATION_M):
                return True

        # 2. 9 Drawer boxes (shifted by drawer slide)
        p_d = p_world - d_origin
        for centre, size, is_hot in DRAWER_BOXES:
            if is_hot and _point_in_inflated_box(p_d, centre, size, HOT_INFLATION_M):
                return True

        # 3. 5 Basket body boxes (in basket frame)
        p_b = rot_basket.T @ (p_world - b_origin)
        for centre, size, is_hot in BASKET_BODY_BOXES:
            if is_hot and _point_in_inflated_box(p_b, centre, size, HOT_INFLATION_M):
                return True

    return False


def on_mat(basket_pos: Sequence[float], basket_quat: Sequence[float]) -> bool:
    """Check ON_MAT: 4 floor corners inside mat rectangle, underside within 5 mm of mat top, tilt < 5 deg."""
    if tilt_deg(basket_quat) > TILT_MAX_DEG:
        return False

    b_origin = np.asarray(basket_pos, dtype=float)
    rot = rotation_from_quat(basket_quat)

    # Underside of basket floor: floor bottom is at local z = -0.005 (5 mm feet stand-in)
    floor_underside_z = b_origin[2] - 0.005
    if abs(floor_underside_z - MAT_TOP_Z) > MAT_Z_TOL:
        return False

    # 4 corners of floor
    corners = [
        (-BASKET_FLOOR_HALF_X, -BASKET_FLOOR_HALF_Y, 0.0),
        (-BASKET_FLOOR_HALF_X, BASKET_FLOOR_HALF_Y, 0.0),
        (BASKET_FLOOR_BACK_X, -BASKET_FLOOR_HALF_Y, 0.0),
        (BASKET_FLOOR_BACK_X, BASKET_FLOOR_HALF_Y, 0.0),
    ]
    for c in corners:
        w_c = b_origin + rot @ np.array(c, dtype=float)
        if not (MAT_X_MIN <= w_c[0] <= MAT_X_MAX and MAT_Y_MIN <= w_c[1] <= MAT_Y_MAX):
            return False

    return True


def button_out(button_displacement: float) -> bool:
    """BUTTON_OUT: button >= 90% extended (displacement <= 1.4 mm of the 14 mm stroke)."""
    return button_displacement <= BUTTON_OUT_THRESHOLD_M


def catches_hold_basket(button_displacement: float) -> bool:
    """Catches are scored as hooking under the ledges while the button displacement is below 8 mm (they physically clear after 7 mm)."""
    return button_displacement < BUTTON_CLEARANCE_DISPLACEMENT_M


def released(
    tcp_positions: Sequence[Sequence[float]],
    basket_pos: Sequence[float],
    basket_quat: Sequence[float],
) -> bool:
    """RELEASED: every point passed in (both gripper centres and all finger pad box centres) is at least 3 cm from the basket body and handle."""
    rot = rotation_from_quat(basket_quat)
    b_origin = np.asarray(basket_pos, dtype=float)

    all_boxes = BASKET_BODY_BOXES + BASKET_HANDLE_BOXES
    for tcp in tcp_positions:
        p_b = rot.T @ (np.asarray(tcp, dtype=float) - b_origin)
        for centre, size, _ in all_boxes:
            d = np.maximum(np.abs(p_b - np.asarray(centre, dtype=float)) - np.asarray(size, dtype=float) / 2.0, 0.0)
            if np.linalg.norm(d) < RELEASE_MIN_DIST_M:
                return False
    return True


def resting_out_of_drawer(basket_pos: Sequence[float], basket_quat: Sequence[float]) -> bool:
    """End-state check for the 0.5 rung: the basket is out of the drawer (12 cm or more from its start in x/y), low on the
    bench (not on the housing), and upright (tilt < 20 degrees)."""
    dx = float(basket_pos[0]) - BASKET_START_POS[0]
    dy = float(basket_pos[1]) - BASKET_START_POS[1]
    if math.hypot(dx, dy) < 0.12:
        return False
    if float(basket_pos[2]) > TABLE_TOP + 0.05:
        return False
    return tilt_deg(basket_quat) < 20.0


def seated_in_drawer(basket_pos: Sequence[float], basket_quat: Sequence[float]) -> bool:
    """End-state check for the 0.25 rung: the basket is seated back in the drawer (within 10 mm of its start, tilt < 5 degrees)."""
    d = np.asarray(basket_pos, dtype=float) - np.asarray(BASKET_START_POS, dtype=float)
    return bool(float(np.max(np.abs(d))) < 0.010 and tilt_deg(basket_quat) < 5.0)


def still(hist: Sequence[Sequence[float]]) -> bool:
    """STILL: basket moved less than 1 mm in the last 1 s (20 steps)."""
    if len(hist) < STILL_WINDOW_STEPS:
        return False
    arr = np.asarray(hist[-STILL_WINDOW_STEPS:], dtype=float)
    ptp = np.ptp(arr, axis=0)
    return float(np.max(ptp)) <= STILL_MOTION_M


def evaluate_state(
    basket_pos: Sequence[float],
    basket_quat: Sequence[float],
    button_disp: float,
    hot_contact_ever: bool,
    cleared_drawer_ever: bool,
    unhook_lift_ever: bool,
    grippers_released: bool,
    basket_still: bool,
    bypass_ever: bool = False,
    dropped_ever: bool = False,
) -> tuple[float, str]:
    """Score the end state. Returns (score, reason)."""
    if hot_contact_ever:
        return 0.0, "hot contact occurred (permanent zero)"
    if bypass_ever:
        return 0.0, "latch bypassed: basket lifted without the release button pressed (permanent zero)"
    if dropped_ever:
        return 0.0, "the hot basket was dropped (fell more than 5 cm in 0.25 s: permanent zero)"

    mat_ok = on_mat(basket_pos, basket_quat)
    btn_ok = button_out(button_disp)

    if mat_ok:
        if btn_ok and grippers_released and basket_still:
            return 1.0, "basket placed on mat, released, still, no hot contact"
        return 0.75, "basket on mat without hot contact, but button not extended, grippers not released, or not still"

    if cleared_drawer_ever and resting_out_of_drawer(basket_pos, basket_quat):
        return 0.5, "basket cleared drawer, rests upright on the bench off the mat, not dropped, no hot contact"

    if unhook_lift_ever and seated_in_drawer(basket_pos, basket_quat):
        return 0.25, "button pressed and basket lifted, but it ended seated back in the drawer"

    return 0.0, "basket remained in drawer"
