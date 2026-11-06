"""On the station table, between the arms: an open navy lunch box (inside 24 cm front to back, 34 cm side to side,
walls 6 cm tall), its centre 23 cm in front of the arm bases. In front of the right arm, on a dark grey mat, lie a blue
gel ice pack (6 x 9 x 3 cm, 250 g) and a brown box of crackers (5 x 7 x 4 cm, 50 g). In front of the left arm, on a
second mat, stand a pink yogurt cup (5.5 cm across, 6 cm tall, 150 g) and a red apple (6.5 cm across, 150 g), and lies
a yellow wrapped sandwich (6 x 10 x 3.5 cm, 120 g). The box and the mats are fixed; the five food items are loose.
Packed, all five lie on the box floor, the ice pack flat, and the yogurt and the sandwich each within 5 mm of the ice
pack, side by side with it (food-safety guidance: perishables go directly against the cold source).
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The lunch box: a 5 mm floor and four 15 mm walls, 6 cm tall from the table, around a 24 x 34 cm inside.
BOX_X, BOX_Y = 0.23, 0.0
INSIDE = (0.24, 0.34)  # front to back (x), side to side (y)
WALL_T, WALL_H, FLOOR_T = 0.015, 0.06, 0.005
FLOOR = TOP + FLOOR_T
IN_X = (BOX_X - INSIDE[0] / 2, BOX_X + INSIDE[0] / 2)
IN_Y = (BOX_Y - INSIDE[1] / 2, BOX_Y + INSIDE[1] / 2)
OUT_X = (IN_X[0] - WALL_T, IN_X[1] + WALL_T)
OUT_Y = (IN_Y[0] - WALL_T, IN_Y[1] + WALL_T)
NAVY = (0.10, 0.14, 0.42)
_WZ = TOP + WALL_H / 2
LUNCH_BOX = [
    Box("box_floor", pos=(BOX_X, BOX_Y, TOP + FLOOR_T / 2), size=(*INSIDE, FLOOR_T), color=NAVY),
    Box("box_wall_near", pos=(IN_X[0] - WALL_T / 2, BOX_Y, _WZ), size=(WALL_T, INSIDE[1] + 2 * WALL_T, WALL_H),
        color=NAVY),
    Box("box_wall_far", pos=(IN_X[1] + WALL_T / 2, BOX_Y, _WZ), size=(WALL_T, INSIDE[1] + 2 * WALL_T, WALL_H),
        color=NAVY),
    Box("box_wall_left", pos=(BOX_X, IN_Y[1] + WALL_T / 2, _WZ), size=(INSIDE[0], WALL_T, WALL_H), color=NAVY),
    Box("box_wall_right", pos=(BOX_X, IN_Y[0] - WALL_T / 2, _WZ), size=(INSIDE[0], WALL_T, WALL_H), color=NAVY),
]

# Two staging mats, 28 cm front to back and 13 cm side to side, 2 mm thick, centred 33 cm left and right of centre.
MAT_SIZE = (0.28, 0.13, 0.002)
MAT_X, MAT_Y = 0.24, 0.33
MAT_TOP = TOP + MAT_SIZE[2]
MAT_GREY = (0.30, 0.30, 0.32)
MATS = [
    Box("left_mat", pos=(MAT_X, MAT_Y, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=MAT_GREY),
    Box("right_mat", pos=(MAT_X, -MAT_Y, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=MAT_GREY),
]

# The food. Box sizes are x (front to back), y (side to side), z (height) as they start; each starts 1 mm above its
# mat and settles.
PACK_SIZE, PACK_KG = (0.06, 0.09, 0.03), 0.25  # flat, long side running side to side
SANDWICH_SIZE, SANDWICH_KG = (0.06, 0.10, 0.035), 0.12  # flat, long side running side to side
CRACKERS_SIZE, CRACKERS_KG = (0.05, 0.07, 0.04), 0.05  # flat on its largest face, long side side to side
YOGURT_R, YOGURT_H, YOGURT_KG = 0.0275, 0.06, 0.15
APPLE_R, APPLE_KG = 0.0325, 0.15
PACK_BLUE = (0.10, 0.55, 0.95)
SANDWICH_YELLOW = (0.95, 0.80, 0.10)
CRACKERS_BROWN = (0.62, 0.36, 0.12)
YOGURT_PINK = (0.95, 0.30, 0.65)
APPLE_RED = (0.85, 0.08, 0.08)

PACK_START = (0.16, -MAT_Y)
CRACKERS_START = (0.30, -MAT_Y)
YOGURT_START = (0.14, MAT_Y)
APPLE_START = (0.23, MAT_Y)
SANDWICH_START = (0.34, MAT_Y)
PACK = Box("ice_pack", pos=(*PACK_START, MAT_TOP + PACK_SIZE[2] / 2 + 0.001), size=PACK_SIZE, mass=PACK_KG,
           color=PACK_BLUE)
CRACKERS = Box("crackers", pos=(*CRACKERS_START, MAT_TOP + CRACKERS_SIZE[2] / 2 + 0.001), size=CRACKERS_SIZE,
               mass=CRACKERS_KG, color=CRACKERS_BROWN)
SANDWICH = Box("sandwich", pos=(*SANDWICH_START, MAT_TOP + SANDWICH_SIZE[2] / 2 + 0.001), size=SANDWICH_SIZE,
               mass=SANDWICH_KG, color=SANDWICH_YELLOW)
YOGURT = Cylinder("yogurt", pos=(*YOGURT_START, MAT_TOP + YOGURT_H / 2 + 0.001), radius=YOGURT_R, height=YOGURT_H,
                  mass=YOGURT_KG, color=YOGURT_PINK)
APPLE = Sphere("apple", pos=(*APPLE_START, MAT_TOP + APPLE_R + 0.001), radius=APPLE_R, mass=APPLE_KG,
               color=APPLE_RED)

OBJECTS = [*LUNCH_BOX, *MATS, PACK, CRACKERS, SANDWICH, YOGURT, APPLE]
FOOD = ("ice_pack", "crackers", "sandwich", "yogurt", "apple")
PERISHABLES = ("sandwich", "yogurt")
SUCCESS_HOLD_S = 2.5  # packed and still for 2.5 s

ON_FLOOR_TOL = 0.003  # an item's lowest point within 3 mm of the box floor's top counts as on the floor
FLAT_DEG = 10.0  # ice pack, sandwich, crackers: thin axis at most 10 degrees from vertical (either face up)
UPRIGHT_DEG = 10.0  # yogurt: its axis at most 10 degrees from vertical, opening up
COLD_GAP = 0.005  # the perishable's side at most 5 mm from the ice pack's outline
GAP_EPS = 1e-6  # rounding allowance (1 micrometre) so a gap of exactly 5 mm counts as "at most 5 mm"
PACK_CREDIT, COLD_CREDIT, SHELF_CREDIT = 0.2, 0.3, 0.1


def _rot(state) -> np.ndarray:
    """Rotation matrix (columns: the object's own x, y, z axes in the world) from quat_xyzw."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt_deg(state) -> float:
    """Angle between the object's own z axis and straight up."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(_rot(state)[2, 2])))))


def _flat_deg(state) -> float:
    """Angle between the object's thin (z) axis and the vertical, whichever face is up."""
    return math.degrees(math.acos(min(1.0, abs(float(_rot(state)[2, 2])))))


def _in_box(state, radius=None) -> bool:
    """Centre inside the box's inner outline and lowest point on the box floor (within 3 mm). For a sphere (give its
    radius) the lowest point is its centre height minus the radius: a rolled sphere's world-aligned bounds are those
    of its turned bounding cube and read up to about 24 mm too low."""
    x, y, z = (float(v) for v in state.pos)
    bottom = float(state.low[2]) if radius is None else z - radius
    return bool(IN_X[0] <= x <= IN_X[1] and IN_Y[0] <= y <= IN_Y[1] and abs(bottom - FLOOR) <= ON_FLOOR_TOL)


def _over_box(state) -> bool:
    """Centre over the box's outer outline (walls included), at any height."""
    x, y, _ = (float(v) for v in state.pos)
    return bool(OUT_X[0] <= x <= OUT_X[1] and OUT_Y[0] <= y <= OUT_Y[1])


def _axis_at(state, z) -> np.ndarray:
    """Where the object's own z axis (through its centre) crosses the world height z, seen from above (x, y)."""
    r = _rot(state)
    p = np.asarray(state.pos, float)
    up = float(r[2, 2])
    if abs(up) < 0.1:  # lying on its side: the axis is nearly horizontal, so use the centre
        return p[:2]
    return p[:2] + r[:2, 2] * (z - p[2]) / up


def _outline(state, size, z=None) -> np.ndarray:
    """The four corners (x, y) of a box's own x-y rectangle, seen from above. With z, the rectangle is slid along
    the box's thickness axis to that height (its outline at height z); without, it is taken at the box's centre."""
    r = _rot(state)
    c = np.asarray(state.pos, float)[:2] if z is None else _axis_at(state, z)
    ax, ay = r[:2, 0] * size[0] / 2, r[:2, 1] * size[1] / 2
    return np.array([c + ax + ay, c - ax + ay, c - ax - ay, c + ax - ay])


def _cross(a, b) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def _point_poly(p, poly) -> float:
    """Distance from point p to a convex polygon (0 inside it)."""
    n = len(poly)
    signs = [_cross(poly[(i + 1) % n] - poly[i], p - poly[i]) for i in range(n)]
    if all(s >= 0 for s in signs) or all(s <= 0 for s in signs):
        return 0.0
    best = math.inf
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        ab = b - a
        t = max(0.0, min(1.0, float(np.dot(p - a, ab) / np.dot(ab, ab))))
        best = min(best, float(np.linalg.norm(p - (a + t * ab))))
    return best


def _segments_cross(a, b, c, d) -> bool:
    d1, d2 = _cross(b - a, c - a), _cross(b - a, d - a)
    d3, d4 = _cross(d - c, a - c), _cross(d - c, b - c)
    return d1 * d2 < 0 and d3 * d4 < 0


def _poly_gap(p, q) -> float:
    """Shortest distance between two convex polygons (0 if they touch or overlap)."""
    for i in range(4):
        for j in range(4):
            if _segments_cross(p[i], p[(i + 1) % 4], q[j], q[(j + 1) % 4]):
                return 0.0
    return min(min(_point_poly(v, q) for v in p), min(_point_poly(v, p) for v in q))


def _pack_ok(objects) -> bool:
    """The ice pack on the box floor, lying flat."""
    pack = objects["ice_pack"]
    return _in_box(pack) and _flat_deg(pack) <= FLAT_DEG


def _gap(objects, name) -> float:
    """Horizontal gap between a perishable's side and the ice pack's outline, measured at the height of the ice
    pack's centre (its mid-height): for the yogurt, from where the cup's axis crosses that height, minus the cup's
    radius; for the sandwich, between its outline at that height and the pack's."""
    pack_state = objects["ice_pack"]
    z = float(pack_state.pos[2])
    pack = _outline(pack_state, PACK_SIZE)
    item = objects[name]
    if name == "yogurt":
        return _point_poly(_axis_at(item, z), pack) - YOGURT_R
    return _poly_gap(_outline(item, SANDWICH_SIZE, z), pack)


def _cold(objects, name) -> bool:
    """The perishable on the box floor, the right way up, within 5 mm of the flat ice pack."""
    item = objects[name]
    pose_ok = _tilt_deg(item) <= UPRIGHT_DEG if name == "yogurt" else _flat_deg(item) <= FLAT_DEG
    return bool(_pack_ok(objects) and _in_box(item) and pose_ok and _gap(objects, name) <= COLD_GAP + GAP_EPS)


def score(objects):
    """Partial credit (success scores 1.0): 0.2 for the ice pack flat on the box floor; 0.3 each for the yogurt
    (upright) and the sandwich (flat) on the box floor within 5 mm of that ice pack; 0.1 each for the apple and the
    crackers (flat) on the box floor. 0 overall if a perishable is over the box without its cold credit."""
    cold = {n: _cold(objects, n) for n in PERISHABLES}
    if any(_over_box(objects[n]) and not cold[n] for n in PERISHABLES):
        return 0.0
    crackers = objects["crackers"]
    shelf = int(_in_box(objects["apple"], radius=APPLE_R)) + int(_in_box(crackers) and _flat_deg(crackers) <= FLAT_DEG)
    return round(PACK_CREDIT * _pack_ok(objects) + COLD_CREDIT * sum(cold.values()) + SHELF_CREDIT * shelf, 6)


def success(objects):
    """Everything packed, the perishables against the ice pack, all still; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in FOOD)


# The demo: the right arm lays the ice pack flat in the box, then the crackers in the box's near right corner (nearer
# the arm bases), and parks over its mat. The left arm then sets the sandwich against the pack's left end and the
# yogurt against its near side (each lowered 5 mm short, then slid while still held until it meets the pack), puts the apple in the near left corner, and parks. The pack, sandwich and crackers are gripped across their
# short side (fingers closing front to back), so no finger is ever on a face that must touch the pack; the yogurt and
# apple are gripped side to side, so the yogurt's far side, which meets the pack, is free.
TRAVEL_Z = TOP + 0.15  # carried items' bottoms clear the 6 cm walls and the packed items by 5 cm or more
PACK_GRIP, SANDWICH_GRIP, CRACKERS_GRIP = 0.02, 0.022, 0.022  # TCP above each item's bottom
YOGURT_GRIP, APPLE_GRIP = 0.03, 0.03
DROP = 0.006  # each item is let go this far above where it lands
PLACE_GAP = 0.002  # the nominal packed layout: each perishable this far from the pack
PRE_GAP = 0.005  # each perishable is lowered this far from the pack, then slid (still gripped) towards it ...
PUSH = 0.0015  # ... to this far past contact, so a few mm of pack or grip error still leaves them touching
SETTLE_S = 0.5  # fingers stay open this long before the arm lifts
ACROSS_X = math.pi / 2  # grip yaw that closes the fingers front to back

PACK_PLACE = (0.26, -0.02)  # the pack's near side at x 0.23, its left end at y 0.025
SANDWICH_PLACE = (PACK_PLACE[0], PACK_PLACE[1] + PACK_SIZE[1] / 2 + PLACE_GAP + SANDWICH_SIZE[1] / 2)
YOGURT_PLACE = (PACK_PLACE[0] - PACK_SIZE[0] / 2 - PLACE_GAP - YOGURT_R, 0.0)
SANDWICH_AWAY, YOGURT_AWAY = (0.0, 1.0), (-1.0, 0.0)  # unit direction from the pack to each perishable
SANDWICH_TOUCH = (SANDWICH_PLACE[0], SANDWICH_PLACE[1] - PLACE_GAP)  # in contact with the pack's left end
YOGURT_TOUCH = (YOGURT_PLACE[0] + PLACE_GAP, YOGURT_PLACE[1])  # in contact with the pack's near side
CRACKERS_PLACE = (0.18, -0.11)
APPLE_PLACE = (IN_X[0] + APPLE_R + 0.003, 0.105)  # 3 mm off the near wall, which stops it rolling nearer the bases
APPLE_DROP = 0.002  # the apple is let go only this far above the floor, so it lands without bouncing ...
APPLE_STILL_S = 0.8  # ... after the arm has held it still this long (the TCP lags sideways when lowered fast) ...
APPLE_SETTLE_S = 0.8  # ... and the fingers stay open this long before the arm lifts

RIGHT_PARK = (MAT_X, -MAT_Y, TRAVEL_Z)
LEFT_PARK = (MAT_X, MAT_Y, TRAVEL_Z)
LEFT_WAIT_S = 21.0  # the left arm waits above its mat until the right arm has parked


def _place_against_pack(pick, touch, away, z, yaw=0.0):
    """Pick, lower PRE_GAP short of the pack (``away`` is the unit (x, y) direction from the pack to the item), slide
    the held item PUSH past contact so it ends against the pack, let go, wait with the fingers open, then lift."""
    start = (touch[0] + away[0] * PRE_GAP, touch[1] + away[1] * PRE_GAP, z)
    end = (touch[0] - away[0] * PUSH, touch[1] - away[1] * PUSH, z)
    wps = pick_place_waypoints(pick, start, travel_z=TRAVEL_Z, yaw=yaw)[:-2]  # up to "lower"
    return wps + [
        Waypoint(1.0, end, FINGER_CLOSED, "slide against the ice pack", yaw),
        Waypoint(0.6, end, FINGER_OPEN, "release", yaw),
        Waypoint(SETTLE_S, end, FINGER_OPEN, "let it settle", yaw),
        Waypoint(1.0, (end[0], end[1], TRAVEL_Z), FINGER_OPEN, "retreat", yaw),
    ]


def demo():
    right = pick_place_waypoints(
        (*PACK_START, MAT_TOP + PACK_GRIP), (*PACK_PLACE, FLOOR + PACK_GRIP + DROP), travel_z=TRAVEL_Z, yaw=ACROSS_X,
    )
    right += pick_place_waypoints(
        (*CRACKERS_START, MAT_TOP + CRACKERS_GRIP), (*CRACKERS_PLACE, FLOOR + CRACKERS_GRIP + DROP),
        travel_z=TRAVEL_Z, yaw=ACROSS_X,
    )
    right.append(Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park", ACROSS_X))

    left = [Waypoint(LEFT_WAIT_S, LEFT_PARK, FINGER_OPEN, "wait for the ice pack")]
    left += _place_against_pack(
        (*SANDWICH_START, MAT_TOP + SANDWICH_GRIP), SANDWICH_TOUCH, SANDWICH_AWAY, FLOOR + SANDWICH_GRIP + DROP,
        yaw=-ACROSS_X,  # the same grip turned the other way, easier on the left arm's wrist
    )
    left += _place_against_pack(
        (*YOGURT_START, MAT_TOP + YOGURT_GRIP), YOGURT_TOUCH, YOGURT_AWAY, FLOOR + YOGURT_GRIP + DROP,
    )
    # The apple is a sphere and rolls off if let go while the arm still lags from the lowering or the pads drag on
    # it: hold it still just above the floor first, let go from 2 mm, wait with the fingers open, then lift slowly.
    apple_at = (*APPLE_PLACE, FLOOR + APPLE_GRIP + APPLE_DROP)
    left += pick_place_waypoints((*APPLE_START, MAT_TOP + APPLE_GRIP), apple_at, travel_z=TRAVEL_Z)[:-2]
    left += [
        Waypoint(APPLE_STILL_S, apple_at, FINGER_CLOSED, "hold still"),
        Waypoint(0.8, apple_at, FINGER_OPEN, "release"),
        Waypoint(APPLE_SETTLE_S, apple_at, FINGER_OPEN, "let it settle"),
        Waypoint(1.5, (*APPLE_PLACE, TRAVEL_Z), FINGER_OPEN, "retreat"),
    ]
    left.append(Waypoint(4.0, LEFT_PARK, FINGER_OPEN, "park"))
    return {"left": left, "right": right}
