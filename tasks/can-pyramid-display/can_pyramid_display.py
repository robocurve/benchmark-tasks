"""In a grocery shop: six cans of tomatoes wait on a delivery tray next to a green display mat. A pyramid display
(three cans, then two, then one) only stands if the bottom row is spaced so each upper can bridges two below it, and
every can is set down square and centred.
"""

import math
from itertools import combinations

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
FLOOR = TOP + 0.003
MAT_TOP = FLOOR + 0.002
D, H = 0.068, 0.06
MAT = (np.array([0.32, 0.12]), np.array([0.12, 0.26]))  # centre, size
TRAY = [(0.14, 0.29), (0.14, 0.37), (0.14, 0.45), (0.26, 0.29), (0.26, 0.37), (0.26, 0.45)]
PX, CY, PITCH = 0.32, 0.12, 0.070
PLAN = [((PX, CY - PITCH), 0), ((PX, CY), 0), ((PX, CY + PITCH), 0),
        ((PX, CY - PITCH / 2), 1), ((PX, CY + PITCH / 2), 1), ((PX, CY), 2)]
TOMATO, BEANS = (0.80, 0.15, 0.12), (0.25, 0.55, 0.25)

GAP_MIN, GAP_MAX = D, D + 0.015  # centre spacing of two cans that both carry the can above
CENTRED = 0.015  # an upper can's centre from the midpoint of its two supports
SEATED = 0.01  # an upper can's base from the tops of its supports
STRAIGHT = 0.01  # the bottom row's middle can centre from the line through the outer two
UPRIGHT, TIPPED = 10.0, 45.0

CANS = [Cylinder(f"can_{i}", pos=(*xy, FLOOR + H / 2), radius=D / 2, height=H, mass=0.3, color=TOMATO)
        for i, xy in enumerate(TRAY)]
SHELF = (0.66, 0.0)
DRESSING = [
    Box("shop_floor", pos=(0.45, 0.0, TOP + 0.0015), size=(0.90, 1.0, 0.003), color=(0.86, 0.84, 0.80)),
    Box("display_mat", pos=(*map(float, MAT[0]), FLOOR + 0.001), size=(*map(float, MAT[1]), 0.002),
        color=(0.20, 0.50, 0.30)),
    Box("delivery_tray", pos=(0.20, 0.37, FLOOR + 0.002), size=(0.22, 0.26, 0.004), color=(0.70, 0.55, 0.35)),
    Box("shelf_back", pos=(SHELF[0] + 0.05, 0.0, FLOOR + 0.15), size=(0.02, 0.80, 0.30), color=(0.95, 0.95, 0.93)),
    *[Box(f"shelf_{k}", pos=(SHELF[0], 0.0, FLOOR + 0.002 + 0.12 * k), size=(0.10, 0.80, 0.012),
          color=(0.75, 0.75, 0.78)) for k in (0, 1)],
    *[Cylinder(f"stock_{k}_{i}", pos=(SHELF[0], -0.30 + 0.085 * i, FLOOR + 0.008 + 0.12 * k + H / 2), radius=D / 2,
               height=H, color=BEANS if k else TOMATO) for k in (0, 1) for i in range(8)],
    Box("price_card", pos=(0.43, 0.12, FLOOR + 0.04), size=(0.008, 0.10, 0.08), color=(0.98, 0.85, 0.15)),
]
OBJECTS = [*DRESSING, *CANS]


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _standing(obj):
    return _tilt_deg(obj) < UPRIGHT and obj.resting()


def _on_mat(obj):
    centre, size = MAT
    return bool(np.all(np.abs(obj.pos[:2] - centre) <= size / 2)) and abs(obj.low[2] - MAT_TOP) < SEATED


def _pair_carries(a, b):
    return GAP_MIN - 0.002 <= np.linalg.norm(a.pos[:2] - b.pos[:2]) <= GAP_MAX


def _bridges(upper, a, b):
    """``upper`` stands centred across ``a`` and ``b``, its base on both their tops."""
    midpoint = (a.pos[:2] + b.pos[:2]) / 2
    return (_pair_carries(a, b) and np.linalg.norm(upper.pos[:2] - midpoint) <= CENTRED
            and abs(upper.low[2] - a.high[2]) < SEATED and abs(upper.low[2] - b.high[2]) < SEATED)


def _on_single(upper, cans):
    """``upper`` stands centred on one can, as in a tower."""
    return any(c is not upper and np.linalg.norm(upper.pos[:2] - c.pos[:2]) <= CENTRED
               and abs(upper.low[2] - c.high[2]) < SEATED for c in cans)


def _off_line(a, mid, b):
    """Distance of ``mid``'s centre from the line through ``a``'s and ``b``'s centres (horizontal)."""
    ab, am = b.pos[:2] - a.pos[:2], mid.pos[:2] - a.pos[:2]
    return abs(ab[0] * am[1] - ab[1] * am[0]) / max(float(np.linalg.norm(ab)), 1e-9)


def _bottom_row(cans):
    """Three cans standing on the mat in a straight line, each next to the next: (left, middle, right) or None."""
    on_mat = [c for c in cans if _standing(c) and _on_mat(c)]
    for trio in combinations(on_mat, 3):
        for mid in trio:
            a, b = (c for c in trio if c is not mid)
            if (_pair_carries(a, mid) and _pair_carries(mid, b) and np.linalg.norm(a.pos[:2] - b.pos[:2]) > 2 * D - 0.01
                    and _off_line(a, mid, b) <= STRAIGHT):
                return a, mid, b
    return None


def score(objects):
    """0 if any can ends tipped over (more than 45 degrees) or stands centred on a single can. Otherwise 0.25 for a
    straight bottom row of three cans on the mat (the middle centre within 1 cm of the line through the outer two);
    0.5 when two more cans each bridge a different pair of neighbours in it; 1.0 when a sixth bridges those two.
    Scored at the end of the trial."""
    cans = [objects[c.name] for c in CANS]
    if any(_tilt_deg(c) > TIPPED for c in cans) or any(_on_single(c, cans) for c in cans):
        return 0.0
    row = _bottom_row(cans)
    if row is None:
        return 0.0
    a, mid, b = row
    rest = [c for c in cans if all(c is not r for r in row) and _standing(c)]
    left = [c for c in rest if _bridges(c, a, mid)]
    right = [c for c in rest if _bridges(c, mid, b)]
    if not (left and right):
        return 0.25
    second = (left[0], right[0])
    if any(_bridges(c, *second) for c in rest if all(c is not s for s in second)):
        return 1.0
    return 0.5


def demo():
    """The left arm builds the pyramid can by can: the bottom row 7 cm apart, then the two cans bridging it, then the
    top one, gripping each with the fingers closing front to back so they never touch a neighbour."""
    left = []
    for (sx, sy), ((dx, dy), level) in zip(TRAY, PLAN):
        grip = FLOOR + 0.013 + H / 2
        place = MAT_TOP + level * H + 0.013 + H / 2 + 0.008
        left += pick_place_waypoints((sx, sy, grip), (dx, dy, place), travel_z=TOP + 0.175, yaw=math.pi / 2)
    left.append(Waypoint(2.0, (0.16, 0.32, TOP + 0.20), FINGER_OPEN, "move clear of the camera's view"))
    return {"left": left}
