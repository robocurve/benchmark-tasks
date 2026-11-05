"""A hot-lab bench on the station table. Two yellow vials (3 cm across, 5 cm tall, 50 g) stand 25 cm in front of the arm
bases, 15 cm left and 15 cm right of centre: they stand in for radioactive sources and must not be touched or moved.
A fixed blue block (15 cm front to back, 20 cm side to side, 12 cm tall), its centre 70 cm in front of the arm bases
on the centre line, marks where the technologist sits; it is out of the arms' reach on purpose. Two dark grey painted
wooden boxes, stand-ins for lead bricks (5 x 12 x 8 cm, 600 g each, long side left to right), stand in a fixed orange rack in front of the left
arm. A fixed green transfer pad lies on the centre line, 15 cm in front of the arm bases. Done, each vial has its own
brick standing upright on the table (either 5 x 12 cm face down) across the straight line from the vial's centre to
the block's centre, its centre within 4 cm of that line, 5 mm to 12 cm from the vial, and both vials are where they
started.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The technologist's seat: a fixed blue block on the far side of the bench. The seat point is its centre.
SEAT_X, SEAT_Y = 0.70, 0.0
SEAT_SIZE = (0.15, 0.20, 0.12)
SEAT = Box("seat_mark", pos=(SEAT_X, SEAT_Y, TOP + SEAT_SIZE[2] / 2), size=SEAT_SIZE, color=(0.12, 0.30, 0.85))

# The hot vials: loose, light, and never to be handled.
VIAL_R, VIAL_H, VIAL_KG = 0.015, 0.05, 0.05
VIAL_YELLOW = (0.95, 0.78, 0.08)
VIAL_START = {"vial_left": (0.25, 0.15), "vial_right": (0.25, -0.15)}
VIALS = [
    Cylinder(name, pos=(x, y, TOP + VIAL_H / 2 + 0.001), radius=VIAL_R, height=VIAL_H, mass=VIAL_KG, color=VIAL_YELLOW)
    for name, (x, y) in VIAL_START.items()
]  # each starts 1 mm above the table, and settles
VIAL_NAMES = tuple(VIAL_START)

# The brick rack: a 10 mm floor inside four 10 mm walls, 30 mm tall from the table, in front of the left arm.
RACK_X, RACK_Y = 0.25, 0.36
RACK_IN = (0.17, 0.14)  # inside, front to back and side to side
RACK_FLOOR_T, RACK_WALL_T, RACK_WALL_H = 0.010, 0.010, 0.030
RACK_ORANGE = (0.85, 0.42, 0.08)
_RZ = TOP + RACK_WALL_H / 2
RACK = [
    Box("rack_floor", pos=(RACK_X, RACK_Y, TOP + RACK_FLOOR_T / 2), size=(*RACK_IN, RACK_FLOOR_T), color=RACK_ORANGE),
    Box("rack_wall_near", pos=(RACK_X - RACK_IN[0] / 2 - RACK_WALL_T / 2, RACK_Y, _RZ),
        size=(RACK_WALL_T, RACK_IN[1] + 2 * RACK_WALL_T, RACK_WALL_H), color=RACK_ORANGE),
    Box("rack_wall_far", pos=(RACK_X + RACK_IN[0] / 2 + RACK_WALL_T / 2, RACK_Y, _RZ),
        size=(RACK_WALL_T, RACK_IN[1] + 2 * RACK_WALL_T, RACK_WALL_H), color=RACK_ORANGE),
    Box("rack_wall_left", pos=(RACK_X, RACK_Y + RACK_IN[1] / 2 + RACK_WALL_T / 2, _RZ),
        size=(RACK_IN[0], RACK_WALL_T, RACK_WALL_H), color=RACK_ORANGE),
    Box("rack_wall_right", pos=(RACK_X, RACK_Y - RACK_IN[1] / 2 - RACK_WALL_T / 2, _RZ),
        size=(RACK_IN[0], RACK_WALL_T, RACK_WALL_H), color=RACK_ORANGE),
]

# The lead bricks: 5 cm front to back, 12 cm side to side, 8 cm tall, 5 cm apart in the rack.
BRICK_SIZE, BRICK_KG = (0.05, 0.12, 0.08), 0.6
BRICK_H = BRICK_SIZE[2]
BRICK_GREY = (0.30, 0.31, 0.34)
BRICK_START = {"brick_1": (RACK_X - 0.05, RACK_Y), "brick_2": (RACK_X + 0.05, RACK_Y)}
BRICKS = [
    Box(name, pos=(x, y, TOP + RACK_FLOOR_T + BRICK_H / 2 + 0.001), size=BRICK_SIZE, mass=BRICK_KG, color=BRICK_GREY)
    for name, (x, y) in BRICK_START.items()
]
BRICK_NAMES = tuple(BRICK_START)

# The transfer pad on the centre line, where the left arm leaves the second brick for the right arm.
PAD_X, PAD_Y, PAD_T = 0.15, 0.0, 0.010
PAD = Box("transfer_pad", pos=(PAD_X, PAD_Y, TOP + PAD_T / 2), size=(0.08, 0.16, PAD_T), color=(0.15, 0.62, 0.25))

OBJECTS = [SEAT, *RACK, *VIALS, *BRICKS, PAD]
SUCCESS_HOLD_S = 2.5  # shielded and still for 2.5 s

SHIELD_CREDIT, FAR_CREDIT = 0.5, 0.25  # per vial
GAP_MIN = 0.005  # a brick closer than 5 mm to a vial, seen from above, touches it
TOUCH_ABOVE = 0.002  # ... if its bottom is less than 2 mm above the vial's top (a brick resting on the vial counts)
GAP_FULL = 0.12  # full credit: the brick's nearest edge at most 12 cm from the vial's edge
GAP_FAR = 0.25  # half credit: 12 to 25 cm away
LINE_TOL = 0.04  # across the line: the brick's centre at most 4 cm from the line, measured square to it
UPRIGHT_DEG = 10.0  # a standing brick: its 8 cm axis within 10 degrees of vertical (either end up)
ON_TABLE_TOL = 0.005  # and its centre within 5 mm of 4 cm above the table (not on the pad, the rack or a brick)
VIAL_MOVED = 0.010  # a vial whose centre is more than 1 cm from its start, or
VIAL_TILT_DEG = 10.0  # tilted more than 10 degrees, or
VIAL_LIFT = 0.003  # with its bottom more than 3 mm above the table (lifted, held in a gripper), has been disturbed: 0


def _rot(state) -> np.ndarray:
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt_deg(state) -> float:
    """Angle between the object's up axis and straight up (a vial upside down is 180 degrees)."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def _axis_tilt_deg(state) -> float:
    """Angle between the object's up axis and the vertical, either end up (a brick upside down is 0 degrees)."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


def _standing(state) -> bool:
    """Upright on either 5 x 12 cm face, directly on the table."""
    return bool(
        _axis_tilt_deg(state) <= UPRIGHT_DEG and abs(float(state.pos[2]) - (TOP + BRICK_H / 2)) <= ON_TABLE_TOL
    )


def _footprint(state):
    """A standing brick seen from above as a rectangle: centre, two unit axes, two half sizes."""
    r = _rot(state)
    u = r[:2, 0] / max(np.linalg.norm(r[:2, 0]), 1e-9)
    v = np.array([-u[1], u[0]])
    return np.asarray(state.pos[:2], float), u, v, BRICK_SIZE[0] / 2, BRICK_SIZE[1] / 2


def _outline(state) -> list:
    """The brick's top-view outline, in any pose: the convex hull of its 8 corners projected onto the table."""
    r, c = _rot(state), np.asarray(state.pos, float)
    pts = sorted({
        tuple(np.round((c + r @ (np.array([sx, sy, sz]) * np.array(BRICK_SIZE) / 2))[:2], 9))
        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)
    })

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]  # counter-clockwise


def _outline_gap(hull, vial_xy) -> float:
    """Shortest distance from the top-view outline to the vial's circle (negative if they overlap)."""
    q = np.asarray(vial_xy, float)
    n = len(hull)
    inside, best = n >= 3, math.inf
    for i in range(n):
        a, b = np.asarray(hull[i]), np.asarray(hull[(i + 1) % n])
        ab = b - a
        if float(ab[0] * (q[1] - a[1]) - ab[1] * (q[0] - a[0])) < 0:
            inside = False
        t = min(max(float((q - a) @ ab) / max(float(ab @ ab), 1e-18), 0.0), 1.0)
        best = min(best, float(np.linalg.norm(q - (a + t * ab))))
    return -VIAL_R if inside else best - VIAL_R


def _line_dist(p, p0, p1) -> float:
    """Distance from p to the segment p0-p1."""
    p, p0, p1 = (np.asarray(a, float) for a in (p, p0, p1))
    seg = p1 - p0
    t = min(max(float((p - p0) @ seg) / max(float(seg @ seg), 1e-18), 0.0), 1.0)
    return float(np.linalg.norm(p - (p0 + t * seg)))


def _gap(rect, vial_xy) -> float:
    """Shortest distance from the rectangle to the vial's outline (negative if they overlap)."""
    c, u, v, hu, hv = rect
    d = np.asarray(vial_xy, float) - c
    du, dv = max(abs(float(d @ u)) - hu, 0.0), max(abs(float(d @ v)) - hv, 0.0)
    return math.hypot(du, dv) - VIAL_R


def _crosses(rect, p0, p1) -> bool:
    """The segment p0-p1 passes over the rectangle (separating-axis test)."""
    c, u, v, hu, hv = rect
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    seg = p1 - p0
    n = np.array([-seg[1], seg[0]]) / max(np.linalg.norm(seg), 1e-9)
    for a in (u, v, n):
        rc, rr = float(c @ a), hu * abs(float(u @ a)) + hv * abs(float(v @ a))
        s0, s1 = sorted((float(p0 @ a), float(p1 @ a)))
        if s1 < rc - rr or s0 > rc + rr:
            return False
    return True


def _disturbed(objects, name) -> bool:
    """The vial has moved more than 1 cm from where it started, tilted more than 10 degrees, or been lifted: its
    centre more than 3 mm above its resting height (its bottom more than 3 mm above the table)."""
    s = objects[name]
    x, y = VIAL_START[name]
    start = np.array([x, y, TOP + VIAL_H / 2])
    return bool(
        np.linalg.norm(np.asarray(s.pos, float) - start) > VIAL_MOVED
        or _tilt_deg(s) > VIAL_TILT_DEG
        or float(s.pos[2]) > TOP + VIAL_H / 2 + VIAL_LIFT
    )


def _touched(objects, name) -> bool:
    """A brick in any pose whose top-view outline is closer than 5 mm to the vial, with its bottom less than 2 mm
    above the vial's top."""
    v = objects[name]
    return any(
        _outline_gap(_outline(objects[b]), v.pos[:2]) < GAP_MIN
        and float(objects[b].low[2]) < float(v.high[2]) + TOUCH_ABOVE
        for b in BRICK_NAMES
    )


def _credit(objects, brick, vial) -> float:
    """What one brick earns for shielding one vial."""
    b = objects[brick]
    if not _standing(b):
        return 0.0
    rect = _footprint(b)
    vial_xy = np.asarray(objects[vial].pos[:2], float)
    if not _crosses(rect, vial_xy, (SEAT_X, SEAT_Y)) or _line_dist(rect[0], vial_xy, (SEAT_X, SEAT_Y)) > LINE_TOL:
        return 0.0
    gap = _gap(rect, vial_xy)
    if gap < GAP_MIN:
        return 0.0
    if gap <= GAP_FULL:
        return SHIELD_CREDIT
    if gap <= GAP_FAR:
        return FAR_CREDIT
    return 0.0


def score(objects):
    """Per vial, 0.5 for a standing brick across its line to the seat (footprint crossing it, centre within 4 cm of
    it) with its nearest edge 5 mm to 12 cm from the vial, or 0.25 if 12 to 25 cm; each brick counts for one vial
    (the assignment that scores most). A vial touched by a brick (outline closer than 5 mm seen from above, bottom
    less than 2 mm above the vial's top) scores 0. 0 overall if either vial has moved more than 1 cm, tilted over
    10 degrees, or has its bottom more than 3 mm above the table."""
    if any(_disturbed(objects, v) for v in VIAL_NAMES):
        return 0.0
    live = [v for v in VIAL_NAMES if not _touched(objects, v)]
    best = 0.0
    for order in (BRICK_NAMES, BRICK_NAMES[::-1]):  # brick_1 to the left vial and brick_2 to the right, or swapped
        best = max(best, sum(_credit(objects, b, v) for b, v in zip(order, VIAL_NAMES, strict=True) if v in live))
    return round(best, 6)


def success(objects):
    """Both vials shielded within 12 cm and untouched, the bricks and vials still; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in (*BRICK_NAMES, *VIAL_NAMES))


# The demo: the left arm lifts brick_1 out of the rack and stands it across the left vial's line, then sets brick_2
# on the transfer pad and parks. The right arm waits, takes brick_2 off the pad and stands it across the right vial's
# line. Both grip each brick across its 5 cm width (yaw 90 degrees: the fingers close along world x).
GRIP_YAW = math.pi / 2  # the right arm
LEFT_YAW = -math.pi / 2  # the same grip turned the other way, which keeps the left arm's wrist within its limits
TRAVEL_Z = TOP + 0.16  # a carried brick's bottom 10 cm above the table: clear of the rack, the bricks and the vials
BRICK_GRIP = 0.060  # TCP above a brick's bottom: the pads cover its top 33 mm, the palm stays 3 cm above it
DROP = 0.008  # each brick is let go this far above where it lands
SHIELD_X = 0.34  # the near face 5 cm beyond the vial's edge


def _on_line(vial):
    """The point at SHIELD_X on the straight line from the vial's centre to the seat point."""
    x, y = VIAL_START[vial]
    return SHIELD_X, y + (SEAT_Y - y) * (SHIELD_X - x) / (SEAT_X - x)


L_SHIELD, R_SHIELD = _on_line("vial_left"), _on_line("vial_right")  # (0.34, +0.12) and (0.34, -0.12)
LEFT_PARK = (0.18, 0.32, TRAVEL_Z)  # over the empty rack, clear of the right arm's work
RIGHT_WAIT = (0.15, -0.30, TRAVEL_Z)
RIGHT_WAIT_S = 21.0  # the right arm waits until the left arm has left the pad and parked
RIGHT_PARK = (0.15, -0.30, TRAVEL_Z)


def demo():
    rack_z = TOP + RACK_FLOOR_T + BRICK_GRIP
    left = pick_place_waypoints(
        (*BRICK_START["brick_1"], rack_z), (*L_SHIELD, TOP + DROP + BRICK_GRIP), travel_z=TRAVEL_Z, yaw=LEFT_YAW,
    )
    left += pick_place_waypoints(
        (*BRICK_START["brick_2"], rack_z), (PAD_X, PAD_Y, TOP + PAD_T + DROP + BRICK_GRIP), travel_z=TRAVEL_Z,
        yaw=LEFT_YAW,
    )
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park", LEFT_YAW))
    right = [Waypoint(RIGHT_WAIT_S, RIGHT_WAIT, FINGER_OPEN, "wait for the brick on the pad", GRIP_YAW)]
    right += pick_place_waypoints(
        (PAD_X, PAD_Y, TOP + PAD_T + BRICK_GRIP), (*R_SHIELD, TOP + DROP + BRICK_GRIP), travel_z=TRAVEL_Z,
        yaw=GRIP_YAW,
    )
    right.append(Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park", GRIP_YAW))
    return {"left": left, "right": right}
