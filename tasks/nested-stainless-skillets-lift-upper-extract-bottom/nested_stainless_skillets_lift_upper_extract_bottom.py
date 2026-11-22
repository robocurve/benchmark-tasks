"""Stage two nested stainless skillets on a countertop mat."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TABLE_HEIGHT

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk
from pitch_demo import Waypoint

SLUG = "nested-stainless-skillets-lift-upper-extract-bottom"
MAT_CENTER = np.array([0.385, -0.072])
MAT_SIZE = np.array([0.4572, 0.6096])
MAT_THICKNESS = 0.0014
MAT_TOP = TABLE_HEIGHT + MAT_THICKNESS
NEST_XY = np.array([0.389, 0.079])
UPPER_SPOT = np.array([0.490, 0.114])
LOWER_SPOT = np.array([0.330, -0.070])
UPPER, LOWER, MAT = "upper_skillet", "lower_skillet", "prep_mat"
UPPER_RAD, LOWER_RAD = 0.1015, 0.127
UPPER_THICK, LOWER_THICK = 0.0055, 0.0065
UPPER_YAW, LOWER_YAW = map(math.radians, (161, -115))
UPPER_GRASP, LOWER_GRASP = 0.138, 0.163
UPPER_BAR_Z, LOWER_BAR_Z = 0.051, 0.054
CENTER_GAP = 0.235
ZONE_RADIUS = 0.016
MAT_MARGIN = 0.010
CLEARANCE = 0.045
SUCCESS_HOLD_S = 2.0
SCORE_OVER = "end"
SEATED_IN = {UPPER: LOWER}
# Per-step latches: checked after every 20 Hz control step; any one sets the whole run to 0.0.
HELD_DIST = 0.040       # m: a skillet counts as held while a gripper TCP is this close to its handle bar
FREE_TILT_DEG = 20.0    # deg: an unheld skillet tilted past this has tipped
DROP_HEIGHT = 0.020     # m: an unheld skillet whose lowest point is this far above its support is falling onto the mat
FLING_SPEED = 1.0       # m/s: horizontal speed of either skillet between control steps
DRAG_DIST = 0.030       # m: the lower skillet moved this far from the nest while the upper is still nested in it
CONTROL_DT = 0.05       # s per control step (20 Hz)
LOWER_LIFT = 0.060  # m the scripted demo lifts the larger skillet by its handle to carry it
LOWER_AIM = np.array([0.017, 0.015])  # m demo handle target offset from the spot geometry


def bowl_boxes(rim, sole, height, thickness):
    """Stepped circular sole and sloping facets surrounding an open well."""
    boxes = []
    edges = sole * np.linspace(-1, 1, 8)
    for a, b in zip(edges[:-1], edges[1:]):
        mid = (a + b) / 2
        width = 2 * math.sqrt(sole**2 - mid**2)
        boxes.append(mk.box((width, b - a, thickness), (0, mid, thickness / 2)))

    sides = 12
    half = math.pi / sides
    bottom_z = thickness - 0.001
    run = (rim - sole) * math.cos(half)
    rise = height - bottom_z
    wall = 0.0027
    lean = math.atan2(run, rise)
    length = math.hypot(run, rise) - wall * math.tan(lean)
    radius = (rim + sole) / 2 * math.cos(half) - wall * math.cos(lean) / 2
    width = 2 * rim * math.sin(half)
    for i in range(sides):
        az = (2 * i + 1) * half
        rot = mk.axis_angle((0, 0, 1), az) @ mk.axis_angle((0, 1, 0), lean)
        boxes.append(mk.box((wall, width, length),
                            (radius * math.cos(az), radius * math.sin(az), (height + bottom_z) / 2),
                            quat=mk.mat_to_quat(rot)))
    return boxes


def skillet_boxes(rim, sole, height, thickness, end, bar_z):
    boxes = bowl_boxes(rim, sole, height, thickness)
    shoulder = rim + 0.016
    boxes.append(mk.box((0.030, 0.033, 0.012), (rim + 0.003, 0, bar_z - 0.006)))
    boxes.append(mk.box((end - shoulder, 0.024, 0.016), ((shoulder + end) / 2, 0, bar_z)))
    return boxes


kit = mk.Kit()


def add_part(name, origin, boxes, **kwargs):
    visual = f"assets/{SLUG}/{name}.usdc"
    mk.VISUALS[name] = visual
    kit.part(name, origin, boxes, visual=visual, **kwargs)


add_part(MAT, (*MAT_CENTER, TABLE_HEIGHT + MAT_THICKNESS / 2),
         [mk.box((*MAT_SIZE, MAT_THICKNESS))], color=(0.34, 0.74, 0.69))
add_part(LOWER, (*NEST_XY, MAT_TOP),
         skillet_boxes(LOWER_RAD, 0.0975, 0.046, LOWER_THICK, 0.310, LOWER_BAR_Z),
         mass=1.10, color=(0.60, 0.63, 0.67), yaw=LOWER_YAW)
add_part(UPPER, (*NEST_XY, MAT_TOP + LOWER_THICK),
         skillet_boxes(UPPER_RAD, 0.0775, 0.042, UPPER_THICK, 0.267, UPPER_BAR_Z),
         mass=0.80, color=(0.77, 0.80, 0.83), yaw=UPPER_YAW)
OBJECTS = kit.objects()

_SIGNS = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])


def _geometry(name):
    boxes = kit.parts[name].boxes
    centers = np.array([b.at for b in boxes])
    rotations = np.array([mk.quat_to_mat(b.quat) for b in boxes])
    halves = np.array([b.size for b in boxes]) / 2
    corners = centers[:, None, :] + (_SIGNS * halves[:, None, :]) @ rotations.transpose(0, 2, 1)
    return centers, rotations, halves, corners.reshape(-1, 3)


_GEOMETRY = {name: _geometry(name) for name in (UPPER, LOWER)}


def _pose(objects, name):
    state = objects.get(name)
    if state is None:
        return None
    q, pos = np.asarray(state.quat_xyzw), np.asarray(state.pos)
    if (q.shape != (4,) or pos.shape != (3,) or not np.isfinite(q).all()
            or not np.isfinite(pos).all() or not np.isclose(q @ q, 1, atol=1e-4)):
        return None
    rot = mk.quat_to_mat(q)
    return pos - rot @ kit.parts[name].centre, rot


def _seated(name, pos, rot):
    if rot[2, 2] < math.cos(math.radians(3.0)) or not MAT_TOP - 0.0005 <= pos[2] <= MAT_TOP + 0.0030:
        return False
    points = _GEOMETRY[name][3] @ rot.T + pos
    return (MAT_TOP - 0.0005 <= points[:, 2].min() <= MAT_TOP + 0.0030
            and np.all(np.abs(points[:, :2] - MAT_CENTER) <= MAT_SIZE / 2 - MAT_MARGIN))


def _handle_rects(name, pose):
    """Shoulder and grip OBBs: world XY center, yaw axes as columns, local XY half sizes."""
    pos, rot = pose
    centers, rotations, halves, _ = _GEOMETRY[name]
    for center, box_rot, half in zip(centers[-2:], rotations[-2:], halves[-2:]):
        forward = (rot @ box_rot)[:2, 0]
        cos_yaw, sin_yaw = forward / np.linalg.norm(forward)
        axes = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        yield (pos + rot @ center)[:2], axes, half[:2]


def _point_rect_distance(points, rect):
    center, axes, half = rect
    local = (points - center) @ axes
    closest = np.clip(local, -half, half)
    return np.linalg.norm(local - closest, axis=-1)


_RECT_CORNERS = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])


def _rect_vertices(rect):
    center, axes, half = rect
    return center + (_RECT_CORNERS * half) @ axes.T


def _edge_straddles(vertices, other):
    """For each edge line, find opposite-loop edges with endpoints on opposite sides."""
    edges = np.roll(vertices, -1, axis=0) - vertices
    offsets = other[None, :, :] - vertices[:, None, :]
    signed_area = edges[:, None, 0] * offsets[:, :, 1] - edges[:, None, 1] * offsets[:, :, 0]
    return signed_area * np.roll(signed_area, -1, axis=1) < 0


def _rect_clearance(first, second):
    """Check interior edge crossings; clamped vertex distances cover containment and touching."""
    first_vertices, second_vertices = _rect_vertices(first), _rect_vertices(second)
    if np.any(_edge_straddles(first_vertices, second_vertices)
              & _edge_straddles(second_vertices, first_vertices).T):
        return 0.0
    return min(_point_rect_distance(first_vertices, second).min(),
               _point_rect_distance(second_vertices, first).min())


def _min_xy_clearance(upper, lower) -> float:
    """Minimum overhead clearance across the two rim disks and four handle OBBs."""
    upper_xy, lower_xy = upper[0][:2], lower[0][:2]
    upper_rects = tuple(_handle_rects(UPPER, upper))
    lower_rects = tuple(_handle_rects(LOWER, lower))
    distances = [np.linalg.norm(upper_xy - lower_xy) - (UPPER_RAD + LOWER_RAD)]
    distances.extend(_point_rect_distance(lower_xy, rect) - LOWER_RAD for rect in upper_rects)
    distances.extend(_point_rect_distance(upper_xy, rect) - UPPER_RAD for rect in lower_rects)
    distances.extend(_rect_clearance(first, second) for first in upper_rects for second in lower_rects)
    return float(min(distances))


def _separated(upper, lower):
    return (np.linalg.norm(upper[0][:2] - lower[0][:2]) >= CENTER_GAP
            and _min_xy_clearance(upper, lower) >= 0.002)


def _hands_clear(poses):
    """End-state only (SCORE_OVER="end"); missing or non-finite TCPs fail closed at 0.7, never 1.0."""
    try:
        tcps = np.asarray(mk.tcps(), dtype=float)
    except (AttributeError, IndexError, KeyError, TypeError, ValueError):
        return False
    if tcps.shape != (2, 3) or not np.isfinite(tcps).all():
        return False
    for name, (pos, rot) in poses.items():
        centers, axes, halves, _ = _GEOMETRY[name]
        local = (tcps - pos) @ rot
        rel = np.einsum("tbi,bij->tbj", local[:, None, :] - centers, axes)
        if np.any(np.linalg.norm(np.maximum(np.abs(rel) - halves, 0), axis=-1) < CLEARANCE):
            return False
    return True


def _latch():
    m = mk.MEMORY.get("latch")
    if m is None:
        m = mk.MEMORY["latch"] = {"failed": None, "prev": None,
                                  "peak": {"free_tilt_deg": 0.0, "drop_mm": 0.0, "speed": 0.0, "drag_mm": 0.0}}
    return m


def _held(name, pos, rot, tcps):
    if tcps is None:
        return False
    centers, axes, halves, _ = _GEOMETRY[name]
    local = ((tcps - pos) @ rot - centers[-1]) @ axes[-1]
    return bool(np.any(np.linalg.norm(np.maximum(np.abs(local) - halves[-1], 0), axis=-1) <= HELD_DIST))


def _nested(upper, lower):
    return (np.linalg.norm(upper[0][:2] - lower[0][:2]) < LOWER_RAD - UPPER_RAD + 0.010
            and upper[0][2] < lower[0][2] + LOWER_THICK + 0.010)


def latch_step(objects):
    """Per-step latch (kit.on_step): the first failure is kept for the rest of the run."""
    m = _latch()
    poses = {name: _pose(objects, name) for name in (UPPER, LOWER)}
    if m["failed"] or any(pose is None for pose in poses.values()):
        return
    try:
        tcps = np.asarray(mk.tcps(), dtype=float)
        tcps = tcps if tcps.shape == (2, 3) and np.isfinite(tcps).all() else None
    except (AttributeError, IndexError, KeyError, TypeError, ValueError):
        tcps = None
    upper, lower = poses[UPPER], poses[LOWER]
    nested = _nested(upper, lower)
    peak, fail = m["peak"], None
    for name, (pos, rot) in poses.items():
        points = _GEOMETRY[name][3] @ rot.T + pos
        if np.any(np.abs(pos[:2] - MAT_CENTER) > MAT_SIZE / 2) or points[:, 2].min() < MAT_TOP - 0.010:
            fail = fail or f"{name} left the mat"
        if _held(name, pos, rot, tcps):
            continue
        tilt = math.degrees(math.acos(float(np.clip(rot[2, 2], -1, 1))))
        support = MAT_TOP + (LOWER_THICK if name == UPPER and nested else 0.0)
        drop = points[:, 2].min() - support
        peak["free_tilt_deg"] = max(peak["free_tilt_deg"], tilt)
        peak["drop_mm"] = max(peak["drop_mm"], 1000 * drop)
        if tilt > FREE_TILT_DEG:
            fail = fail or f"{name} tipped {tilt:.1f} deg while not held"
        if drop > DROP_HEIGHT:
            fail = fail or f"{name} {1000 * drop:.0f} mm above its support while not held"
    if m["prev"] is not None:
        for name, (pos, _) in poses.items():
            speed = float(np.linalg.norm(pos[:2] - m["prev"][name])) / CONTROL_DT
            peak["speed"] = max(peak["speed"], speed)
            if speed > FLING_SPEED:
                fail = fail or f"{name} flung at {speed:.2f} m/s"
    if nested:
        drag = float(np.linalg.norm(lower[0][:2] - NEST_XY))
        peak["drag_mm"] = max(peak["drag_mm"], 1000 * drag)
        if drag > DRAG_DIST:
            fail = fail or f"pair dragged {1000 * drag:.0f} mm while nested"
    m["prev"] = {name: pose[0][:2].copy() for name, pose in poses.items()}
    if fail:
        m["failed"] = {"step": mk._EP.get("step"), "why": fail}
        print(f"[skillets] latch: {fail} (control step {mk._EP.get('step')}) -> run scores 0.0", flush=True)


kit.on_step(latch_step)


def score(objects):
    if _latch()["failed"]:
        return 0.0
    poses = {name: _pose(objects, name) for name in (UPPER, LOWER)}
    if any(pose is None or not _seated(name, *pose) for name, pose in poses.items()):
        return 0.0
    upper, lower = poses[UPPER], poses[LOWER]
    if not _separated(upper, lower):
        return 0.0
    path = LOWER_SPOT - NEST_XY
    direction = path / np.linalg.norm(path)
    displacement = lower[0][:2] - NEST_XY
    progress = displacement @ direction
    if progress < 0.065 or np.linalg.norm(displacement - progress * direction) > 0.050:
        return 0.0
    in_spots = (np.linalg.norm(upper[0][:2] - UPPER_SPOT) <= ZONE_RADIUS
                and np.linalg.norm(lower[0][:2] - LOWER_SPOT) <= ZONE_RADIUS)
    if not in_spots:
        return 0.4
    return 1.0 if _hands_clear(poses) else 0.7


def success(objects):
    return score(objects) == 1.0


def _at_bar(center, yaw, radius, z):
    return (center[0] + radius * math.cos(yaw), center[1] + radius * math.sin(yaw), z)


def demo():
    """Open-loop contact with the two handle bars."""
    o, c = FINGER_OPEN, FINGER_CLOSED
    upper_z = MAT_TOP + LOWER_THICK + UPPER_BAR_Z
    lower_z = MAT_TOP + LOWER_BAR_Z
    up_yaw = UPPER_YAW - math.pi
    low_yaw = LOWER_YAW + math.pi
    up = _at_bar(NEST_XY, UPPER_YAW, UPPER_GRASP, upper_z + 0.002)
    down = _at_bar(NEST_XY, LOWER_YAW, LOWER_GRASP, lower_z)
    upper_high = (*up[:2], upper_z + 0.13)
    upper_over = _at_bar(UPPER_SPOT + [0.035, -0.013], UPPER_YAW, UPPER_GRASP, upper_z + 0.108)
    upper_set = (*upper_over[:2], MAT_TOP + UPPER_BAR_Z + 0.002)
    lower_set = _at_bar(LOWER_SPOT + LOWER_AIM, LOWER_YAW, LOWER_GRASP, lower_z)
    lift_pitch = math.radians(8)
    lower_lift = (*down[:2], lower_z + LOWER_LIFT)
    lower_carry = (*lower_set[:2], lower_z + LOWER_LIFT)
    return {
        "left": [
            Waypoint(1.7, (*up[:2], upper_z + 0.09), o, "over small handle", up_yaw),
            Waypoint(1.1, up, o, "approach small handle", up_yaw),
            Waypoint(1.0, up, c, "close on small handle", up_yaw),
            Waypoint(1.6, upper_high, c, "lift small skillet", up_yaw, pitch=lift_pitch),
            Waypoint(6.7, upper_high, c, "keep small skillet aloft", up_yaw, pitch=lift_pitch),
            Waypoint(2.0, upper_over, c, "carry small skillet to its spot", up_yaw, pitch=lift_pitch),
            Waypoint(2.0, upper_set, c, "seat small skillet", up_yaw),
            Waypoint(0.8, upper_set, o, "release small skillet", up_yaw),
            Waypoint(1.0, (*upper_over[:2], upper_z + 0.09), o, "clear small handle", up_yaw),
            Waypoint(1.0, (0.16, 0.34, 0.86), o, "withdraw", 0.0),
        ],
        "right": [
            Waypoint(2.2, (*down[:2], lower_z + 0.09), o, "over large handle", low_yaw),
            Waypoint(1.2, down, o, "approach large handle", low_yaw),
            Waypoint(1.0, down, c, "close on large handle", low_yaw),
            Waypoint(1.2, down, c, "steady large skillet", low_yaw),
            Waypoint(1.0, lower_lift, c, "lift large skillet clear of the mat", low_yaw, pitch=lift_pitch),
            Waypoint(2.3, lower_carry, c, "carry large skillet to its spot", low_yaw, pitch=lift_pitch),
            Waypoint(1.2, lower_set, c, "seat large skillet", low_yaw),
            Waypoint(0.8, lower_set, o, "release large skillet", low_yaw),
            Waypoint(1.0, (*lower_set[:2], lower_z + 0.09), o, "clear large handle", low_yaw),
            Waypoint(1.2, (0.16, -0.38, 0.86), o, "withdraw", 0.0),
        ],
    }
