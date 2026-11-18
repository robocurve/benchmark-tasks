"""Scoring logic and geometry tests for the air fryer fry-basket task."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np

from geometry import (
    BASKET_ABOVE_PAN_M,
    BASKET_BODY_BOXES,
    BASKET_FLOOR_BACK_X,
    BASKET_FLOOR_HALF_X,
    BASKET_FLOOR_HALF_Y,
    BASKET_HANDLE_BOXES,
    GUARD_BOXES,
    HOT_INFLATION_M,
    HOUSING_BOXES,
    HOUSING_POS,
    LEVER_AT,
    LEVER_BOXES,
    LEVER_OUT_RAD,
    MAT_TOP_Z,
    MAT_X_MAX,
    MAT_X_MIN,
    MAT_Y_MAX,
    MAT_Y_MIN,
    MAT_Z_TOL,
    PAD_BOX_HALF,
    PAN_BOXES,
    PAN_HOME_XY_M,
    PAN_HOME_Z_M,
    PAN_START_POS,
    RELEASE_MIN_DIST_M,
    STILL_MOTION_M,
    STILL_WINDOW_STEPS,
    TABLE_TOP,
    TCP_BELOW_LINK_M,
    TILT_MAX_DEG,
)

HOT_BOX_COUNT = sum(1 for b in HOUSING_BOXES + PAN_BOXES + BASKET_BODY_BOXES if b[2])


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


def hand_pose_boxes(body_pos: Sequence[float], quat_xyzw: Sequence[float], half_gap: float):
    """Tool point, eight pad-box centres and tilt of one gripper from its body position and its orientation.

    The orientation is (x, y, z, w), the order Isaac Lab 3 reports body_quat_w in. TCP = body position + rotation applied to
    (0, 0, TCP_BELOW_LINK_M). Pads sit on the gripper x axis (the closing axis) at half_gap + 6 mm either side of the finger
    line; four 12 mm boxes per finger at 0, 7 mm below, 3 cm above and 6 cm above the tool point. Returns (tcp, pads, tilt_deg).
    """
    rot = rotation_from_quat(quat_xyzw)
    tcp = np.asarray(body_pos, dtype=float) + rot @ np.array([0.0, 0.0, TCP_BELOW_LINK_M])
    pads = []
    for along in (0.0, 0.007, -0.030, -0.060):
        centre = tcp + rot @ np.array([0.0, 0.0, along])
        for side in (1.0, -1.0):
            pads.append(centre + rot[:, 0] * side * (half_gap + 0.006))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(rot[2, 2])))))
    return tcp, pads, tilt


# Finger link origins in the gripper frame, from the robot's URDF (yam.urdf joints joint7 and joint8: origin xyz, rpy
# (-pi, 0, pi/2), prismatic axis +y / -y, which the rpy turns into the gripper's +x / -x). joint7/joint8 read 0 closed
# and down to -0.04695 open.
TIP_LEFT_AT = (0.0445119, -0.0239931, 0.0566829)
TIP_RIGHT_AT = (-0.0445119, 0.0239931, 0.0566829)


def tip_link_positions(body_pos: Sequence[float], quat_xyzw: Sequence[float], q7: float, q8: float):
    """World positions of the tip_left and tip_right link origins predicted from the gripper body pose and the finger joint
    positions. Compared on the GPU with the tip_left/tip_right positions the simulator reports (an independent check of the
    quaternion order: those positions do not depend on how the gripper's quaternion is read)."""
    rot = rotation_from_quat(quat_xyzw)
    body = np.asarray(body_pos, dtype=float)
    left = body + rot @ (np.asarray(TIP_LEFT_AT) + np.array([q7, 0.0, 0.0]))
    right = body + rot @ (np.asarray(TIP_RIGHT_AT) + np.array([-q8, 0.0, 0.0]))
    return left, right


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
    pan_pos: Sequence[float] = PAN_START_POS,
    pan_quat: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
) -> bool:
    """True if any finger pad box overlaps any of the 16 HOT boxes (appliance body, outer basket, fry basket), each
    inflated 2 mm."""
    rot_basket = rotation_from_quat(basket_quat)
    rot_pan = rotation_from_quat(pan_quat)
    b_origin = np.asarray(basket_pos, dtype=float)
    p_origin = np.asarray(pan_pos, dtype=float)
    h_origin = np.asarray(HOUSING_POS, dtype=float)
    for pad in pad_positions:
        p_world = np.asarray(pad, dtype=float)
        frames = ((p_world - h_origin, HOUSING_BOXES), (rot_pan.T @ (p_world - p_origin), PAN_BOXES),
                  (rot_basket.T @ (p_world - b_origin), BASKET_BODY_BOXES))
        for local, boxes in frames:
            for centre, size, is_hot in boxes:
                if is_hot and _point_in_inflated_box(local, centre, size, HOT_INFLATION_M):
                    return True
    return False


def on_mat(basket_pos: Sequence[float], basket_quat: Sequence[float]) -> bool:
    """ON_MAT: all four floor corners inside the mat, underside within 5 mm of the mat top, tilt < 5 deg."""
    if tilt_deg(basket_quat) > TILT_MAX_DEG:
        return False
    b_origin = np.asarray(basket_pos, dtype=float)
    rot = rotation_from_quat(basket_quat)
    if abs(b_origin[2] - 0.005 - MAT_TOP_Z) > MAT_Z_TOL:   # floor underside 5 mm below the basket origin
        return False
    for c in ((-BASKET_FLOOR_HALF_X, -BASKET_FLOOR_HALF_Y, 0.0), (-BASKET_FLOOR_HALF_X, BASKET_FLOOR_HALF_Y, 0.0),
              (BASKET_FLOOR_BACK_X, -BASKET_FLOOR_HALF_Y, 0.0), (BASKET_FLOOR_BACK_X, BASKET_FLOOR_HALF_Y, 0.0)):
        w_c = b_origin + rot @ np.array(c, dtype=float)
        if not (MAT_X_MIN <= w_c[0] <= MAT_X_MAX and MAT_Y_MIN <= w_c[1] <= MAT_Y_MAX):
            return False
    return True


def pan_home(pan_pos: Sequence[float], pan_quat: Sequence[float]) -> bool:
    """PAN_HOME: the outer basket still rests on its trivet (within 1.5 cm of its start, 5 mm in height, tilt < 5 deg)."""
    d = np.asarray(pan_pos, dtype=float) - np.asarray(PAN_START_POS, dtype=float)
    return bool(math.hypot(d[0], d[1]) <= PAN_HOME_XY_M and abs(d[2]) <= PAN_HOME_Z_M and tilt_deg(pan_quat) < TILT_MAX_DEG)


def lever_out(lever_angle: float) -> bool:
    """LEVER_OUT: the release lever is back at rest (turned <= 0.02 rad of its 0.22 rad stroke)."""
    return lever_angle <= LEVER_OUT_RAD


def basket_rise_over_pan(basket_pos: Sequence[float], pan_pos: Sequence[float]) -> float:
    """How far the fry basket has risen out of the outer basket (0 when seated on its floor)."""
    return float(basket_pos[2]) - float(pan_pos[2]) - BASKET_ABOVE_PAN_M


def over_pan(basket_pos: Sequence[float], pan_pos: Sequence[float], margin: float = 0.03) -> bool:
    """The fry basket is still over the outer basket (centres within 3 cm in x and y)."""
    return abs(float(basket_pos[0]) - float(pan_pos[0])) <= margin and abs(float(basket_pos[1]) - float(pan_pos[1])) <= margin


def moving_part_boxes(lever_angle: float, guard_slide: float):
    """Lever and guard boxes in the basket frame at their current joint positions (centres turned with the lever; sizes
    unrotated, which over the lever's 13 degree stroke is within 1 mm)."""
    out = []
    c, s = math.cos(lever_angle), math.sin(lever_angle)
    for (x, y, z), size, _ in LEVER_BOXES:
        out.append(((LEVER_AT[0] + c * x + s * z, LEVER_AT[1] + y, LEVER_AT[2] - s * x + c * z), size))
    for (x, y, z), size, _ in GUARD_BOXES:
        out.append(((x + guard_slide, y, z), size))
    return out


def released(
    points: Sequence[Sequence[float]],
    basket_pos: Sequence[float],
    basket_quat: Sequence[float],
    lever_angle: float = 0.0,
    guard_slide: float = 0.0,
) -> bool:
    """RELEASED: every point passed in (both gripper centres and all finger pad box centres) is at least 3 cm from the fry
    basket, its handle, the release lever and the button guard."""
    rot = rotation_from_quat(basket_quat)
    b_origin = np.asarray(basket_pos, dtype=float)
    boxes = [(c, s) for c, s, _ in BASKET_BODY_BOXES + BASKET_HANDLE_BOXES] + moving_part_boxes(lever_angle, guard_slide)
    for pt in points:
        p_b = rot.T @ (np.asarray(pt, dtype=float) - b_origin)
        for centre, size in boxes:
            d = np.maximum(np.abs(p_b - np.asarray(centre, dtype=float)) - np.asarray(size, dtype=float) / 2.0, 0.0)
            if np.linalg.norm(d) < RELEASE_MIN_DIST_M:
                return False
    return True


def resting_off_pan(basket_pos: Sequence[float], basket_quat: Sequence[float], pan_pos: Sequence[float]) -> bool:
    """End state for the 0.5 rung: the fry basket is out of the outer basket (12 cm or more from it in x/y), low on the
    bench (origin within 5 cm of it) and upright (tilt < 20 deg)."""
    if math.hypot(float(basket_pos[0]) - float(pan_pos[0]), float(basket_pos[1]) - float(pan_pos[1])) < 0.12:
        return False
    if float(basket_pos[2]) > TABLE_TOP + 0.05:
        return False
    return tilt_deg(basket_quat) < 20.0


def seated_in_pan(basket_pos: Sequence[float], basket_quat: Sequence[float], pan_pos: Sequence[float]) -> bool:
    """End state for the 0.25 rung: the fry basket is seated back in the outer basket (within 1 cm, tilt < 5 deg)."""
    d = np.asarray(basket_pos, dtype=float) - np.asarray(pan_pos, dtype=float) - np.array([0.0, 0.0, BASKET_ABOVE_PAN_M])
    return bool(float(np.max(np.abs(d))) < 0.010 and tilt_deg(basket_quat) < 5.0)


def still(hist: Sequence[Sequence[float]]) -> bool:
    """STILL: basket moved less than 1 mm in the last 1 s (20 steps)."""
    if len(hist) < STILL_WINDOW_STEPS:
        return False
    arr = np.asarray(hist[-STILL_WINDOW_STEPS:], dtype=float)
    return float(np.max(np.ptp(arr, axis=0))) <= STILL_MOTION_M


def evaluate_state(
    basket_pos: Sequence[float],
    basket_quat: Sequence[float],
    pan_pos: Sequence[float],
    pan_quat: Sequence[float],
    lever_angle: float,
    facts: dict,
    grippers_released: bool,
    basket_still: bool,
) -> tuple[float, str]:
    """Score the end state from the latched facts. Returns (score, reason)."""
    if facts.get("hot_contact_ever"):
        return 0.0, "hot contact occurred (permanent zero)"
    if facts.get("released_in_air_ever"):
        return 0.0, "release pressed with the outer basket off its trivet (permanent zero)"
    if facts.get("unlatched_lift_ever"):
        return 0.0, "fry basket lifted out without the release lever pressed: latch defeated (permanent zero)"
    if facts.get("dropped_ever"):
        return 0.0, "a hot basket was dropped (fell more than 5 cm in 0.25 s: permanent zero)"

    home = pan_home(pan_pos, pan_quat)
    if home and on_mat(basket_pos, basket_quat):
        if lever_out(lever_angle) and grippers_released and basket_still:
            return 1.0, "fry basket on the mat, outer basket on its trivet, lever back out, hands clear, still"
        return 0.75, "fry basket on the mat and outer basket on its trivet, but lever not back out, hands not clear, or not still"
    if home and facts.get("cleared_pan_ever") and resting_off_pan(basket_pos, basket_quat, pan_pos):
        return 0.5, "fry basket out of the outer basket, upright on the bench off the mat"
    if facts.get("unhook_lift_ever") and seated_in_pan(basket_pos, basket_quat, pan_pos):
        return 0.25, "release pressed and fry basket lifted, but it ended seated back in the outer basket"
    return 0.0, "fry basket still in the outer basket"
