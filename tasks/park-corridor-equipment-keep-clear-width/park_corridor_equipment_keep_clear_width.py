"""On the station table, a 1:10.9 model of a hospital corridor runs from side to side in front of the arms: two fixed
dark grey walls, 5 cm tall, their inner faces 12 and 40 cm in front of the arm bases (28 cm apart), each 44 cm long,
centred between the arms. On each wall's floor a red door zone covers the middle 20 cm, and a grey parking bay
(10 cm along the wall, 9 cm out from it) lies on each side of it, each closed at its outer end by a low black stop.
A yellow handover pad (9 x 9 cm) marks the corridor's centre. Beyond the left end of the corridor stand two green
shallow carts (4 cm deep, 7 cm long, 6 cm tall, 0.2 kg); beyond the right end stand two blue deep carts (8 cm deep,
7 cm long, 6 cm tall, 0.3 kg).
Parked, each half of the corridor holds one blue and one green cart facing each other, every cart flat against a
wall inside a grey bay, at least 14 cm clear between them, and nothing on the pad.
"""

import math

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The floor markings are 1 mm painted patches; a cart parked in a bay stands on one.
PAINT_T = 0.001
FLOOR = TOP + PAINT_T

# The corridor: x is the distance in front of the arm bases, y is to the robot's left.
NEAR_X, FAR_X = 0.12, 0.40  # the walls' inner faces
CORRIDOR_W = FAR_X - NEAR_X  # 280 mm
WALL_T, WALL_H, WALL_HALF_LEN = 0.02, 0.05, 0.22  # walls run y = -0.22 to +0.22
ZONE_D = 0.09  # door zones and bays reach 90 mm out from the wall face
DOOR_HALF = 0.10  # each door zone covers y = -0.10 to +0.10
BAY_Y = (0.10, 0.20)  # each bay covers |y| = 0.10 to 0.20 (left half y > 0, right half y < 0)
BAY_CY = sum(BAY_Y) / 2
STOP_SIZE = (0.02, 0.02, 0.025)  # out from the wall, along it, tall; at |y| = 0.20 to 0.22
PAD_X, PAD_Y, PAD_SIZE = (NEAR_X + FAR_X) / 2, 0.0, 0.09

WALL_GREY = (0.22, 0.23, 0.26)
BAY_GREY = (0.45, 0.45, 0.48)
DOOR_RED = (0.85, 0.12, 0.10)
PAD_YELLOW = (0.95, 0.78, 0.08)
STOP_BLACK = (0.06, 0.06, 0.07)

_WZ = TOP + WALL_H / 2
_PZ = TOP + PAINT_T / 2
_NEAR_ZONE_X = NEAR_X + ZONE_D / 2
_FAR_ZONE_X = FAR_X - ZONE_D / 2
FIXTURES = [
    Box("corridor_wall_near", pos=(NEAR_X - WALL_T / 2, 0.0, _WZ), size=(WALL_T, 2 * WALL_HALF_LEN, WALL_H),
        color=WALL_GREY),
    Box("corridor_wall_far", pos=(FAR_X + WALL_T / 2, 0.0, _WZ), size=(WALL_T, 2 * WALL_HALF_LEN, WALL_H),
        color=WALL_GREY),
    Box("door_zone_near", pos=(_NEAR_ZONE_X, 0.0, _PZ), size=(ZONE_D, 2 * DOOR_HALF, PAINT_T), color=DOOR_RED),
    Box("door_zone_far", pos=(_FAR_ZONE_X, 0.0, _PZ), size=(ZONE_D, 2 * DOOR_HALF, PAINT_T), color=DOOR_RED),
    Box("handover_pad", pos=(PAD_X, PAD_Y, _PZ), size=(PAD_SIZE, PAD_SIZE, PAINT_T), color=PAD_YELLOW),
]
for _side, _s in (("left", 1.0), ("right", -1.0)):
    for _wall, _zx, _sx in (("near", _NEAR_ZONE_X, NEAR_X + STOP_SIZE[0] / 2),
                            ("far", _FAR_ZONE_X, FAR_X - STOP_SIZE[0] / 2)):
        FIXTURES.append(Box(f"bay_{_side}_{_wall}", pos=(_zx, _s * BAY_CY, _PZ),
                            size=(ZONE_D, BAY_Y[1] - BAY_Y[0], PAINT_T), color=BAY_GREY))
        FIXTURES.append(Box(f"stop_{_side}_{_wall}", pos=(_sx, _s * (BAY_Y[1] + STOP_SIZE[1] / 2), TOP + STOP_SIZE[2] / 2),
                            size=STOP_SIZE, color=STOP_BLACK))

# The carts: depth across the corridor (x), 70 mm length along the wall (y), 60 mm tall.
DEEP_SIZE, SHALLOW_SIZE = (0.08, 0.07, 0.06), (0.04, 0.07, 0.06)
CART_H = 0.06
DEEP_KG, SHALLOW_KG = 0.30, 0.20
DEEP_BLUE = (0.12, 0.32, 0.88)
SHALLOW_GREEN = (0.10, 0.62, 0.18)
ROOM_X = 0.20  # the end rooms: open table beyond the corridor's ends, 20 cm in front of the arm bases
ROOM_Y_INNER, ROOM_Y_OUTER = 0.30, 0.42
CART_START = {
    "deep_cart_1": (ROOM_X, -ROOM_Y_INNER),
    "deep_cart_2": (ROOM_X, -ROOM_Y_OUTER),
    "shallow_cart_1": (ROOM_X, ROOM_Y_INNER),
    "shallow_cart_2": (ROOM_X, ROOM_Y_OUTER),
}
CART_SIZE = {name: DEEP_SIZE if name.startswith("deep") else SHALLOW_SIZE for name in CART_START}
CART_NAMES = tuple(CART_START)
CARTS = [
    Box(name, pos=(x, y, TOP + CART_H / 2 + 0.001), size=CART_SIZE[name],
        mass=DEEP_KG if name.startswith("deep") else SHALLOW_KG,
        color=DEEP_BLUE if name.startswith("deep") else SHALLOW_GREEN)
    for name, (x, y) in CART_START.items()
]  # each starts 1 mm above the table, and settles

OBJECTS = [*FIXTURES, *CARTS]
SUCCESS_HOLD_S = 2.5  # parked and still for 2.5 s

# Scoring thresholds.
CART_CREDIT = 0.25  # each parked cart
MIN_CLEAR = 0.14  # clear width between carts facing each other across a half of the corridor
WALL_TOL = 0.010  # a parked cart's nearest point at most 10 mm from its wall's face
BAY_TOL = 0.003  # a parked cart's footprint at most 3 mm past either end of its bay
SEAT_ABOVE = 0.001  # a parked cart's bottom at most 1 mm above the bay's 1 mm floor sheet (set down, not held)
SEAT_BELOW = 0.003  # and at most 3 mm below it (a cart standing on the bare table beside the sheet)
UPRIGHT_DEG = 10.0  # a parked cart tilted at most 10 degrees
YAW_TOL_DEG = 10.0  # a parked cart's 70 mm side within 10 degrees of the wall's direction
TIPPED_DEG = 30.0  # a cart tilted more than 30 degrees anywhere is knocked over


def _tilt_deg(state) -> float:
    """Angle between the cart's height axis and the vertical, either way up (the stand-in box has no top)."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


def _yaw_off_deg(state) -> float:
    """Angle between the cart's depth axis, seen from above, and the line across the corridor (0 to 90 degrees)."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    ax, ay = 1 - 2 * (y * y + z * z), 2 * (x * y + z * w)
    return math.degrees(math.atan2(abs(ay), abs(ax)))


def _upset(state) -> bool:
    """Knocked over (tilted past 30 degrees) or fallen off the table."""
    return bool(float(state.pos[2]) < TOP or _tilt_deg(state) > TIPPED_DEG)


def _in_corridor(state) -> bool:
    """The footprint reaches at least partly between the walls, anywhere along them."""
    lo, hi = state.low, state.high
    return bool(lo[0] < FAR_X and hi[0] > NEAR_X and lo[1] < WALL_HALF_LEN and hi[1] > -WALL_HALF_LEN)


def _parked(state):
    """(half, wall) of the bay the cart is parked in, or None. Parked: still, upright, its 70 mm side along the wall,
    standing on the bay's floor, its footprint inside one bay's length, and its nearest point within 10 mm of a wall."""
    if not state.resting() or _tilt_deg(state) > UPRIGHT_DEG or _yaw_off_deg(state) > YAW_TOL_DEG:
        return None
    lo, hi = state.low, state.high
    if not FLOOR - SEAT_BELOW <= float(lo[2]) <= FLOOR + SEAT_ABOVE:
        return None
    if float(lo[0]) < NEAR_X - BAY_TOL or float(hi[0]) > FAR_X + BAY_TOL:
        return None
    half = None
    for name, sign in (("left", 1.0), ("right", -1.0)):
        y0, y1 = sorted((sign * BAY_Y[0], sign * BAY_Y[1]))
        if float(lo[1]) >= y0 - BAY_TOL and float(hi[1]) <= y1 + BAY_TOL:
            half = name
    if half is None:
        return None
    if float(lo[0]) <= NEAR_X + WALL_TOL:
        return half, "near"
    if float(hi[0]) >= FAR_X - WALL_TOL:
        return half, "far"
    return None


def _footprint(state, size):
    """The cart's outline seen from above: the convex hull of its eight corners projected onto the table."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    row_x = (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w))
    row_y = (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w))
    cx, cy = float(state.pos[0]), float(state.pos[1])
    corners = [(sx * size[0] / 2, sy * size[1] / 2, sz * size[2] / 2)
               for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    pts = sorted({(round(cx + sum(a * b for a, b in zip(row_x, c)), 9),
                   round(cy + sum(a * b for a, b in zip(row_y, c)), 9)) for c in corners})

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    hull = []
    for seq in (pts, pts[::-1]):
        part = []
        for p in seq:
            while len(part) >= 2 and cross(part[-2], part[-1], p) <= 0:
                part.pop()
            part.append(p)
        hull += part[:-1]
    return hull


def _x_span_at(poly, y):
    """(nearest, farthest) distance in front of the arm bases of a convex outline along the line at this y."""
    xs = []
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if min(y1, y2) - 1e-12 <= y <= max(y1, y2) + 1e-12:
            if abs(y2 - y1) < 1e-12:
                xs += [x1, x2]
            else:
                t = min(1.0, max(0.0, (y - y1) / (y2 - y1)))
                xs.append(x1 + t * (x2 - x1))
    return min(xs), max(xs)


def _pair_gap(near_poly, far_poly):
    """The narrowest gap across the corridor between two outlines, measured perpendicular to the walls at every point
    along the corridor where both carts are present; None if they don't face each other at all."""
    lo = max(min(p[1] for p in near_poly), min(p[1] for p in far_poly))
    hi = min(max(p[1] for p in near_poly), max(p[1] for p in far_poly))
    if lo > hi:
        return None
    ys = [lo, hi] + [p[1] for p in near_poly + far_poly if lo <= p[1] <= hi]
    return min(_x_span_at(far_poly, y)[0] - _x_span_at(near_poly, y)[1] for y in ys)


def _clear_width(objects, parked, half):
    """Clear width across one half: the narrowest gap, perpendicular to the walls, between the near-wall cart and the
    far-wall cart wherever along the corridor both are present; None unless both bays of the half hold a parked cart."""
    near = [_footprint(objects[n], CART_SIZE[n]) for n, p in parked.items() if p == (half, "near")]
    far = [_footprint(objects[n], CART_SIZE[n]) for n, p in parked.items() if p == (half, "far")]
    gaps = [g for a in near for b in far if (g := _pair_gap(a, b)) is not None]
    return min(gaps) if gaps else None


def score(objects):
    """0.25 for each parked cart. 0 overall if any cart is knocked over or off the table, if any cart in the corridor
    is not parked (an obstruction: on the pad, in a door zone, off the wall, turned, stacked, moving, or held more
    than 1 mm above the floor), or if the carts facing each other across either half leave less than 140 mm clear."""
    carts = {n: objects[n] for n in CART_NAMES}
    if any(_upset(s) for s in carts.values()):
        return 0.0
    parked = {n: _parked(s) for n, s in carts.items()}
    if any(parked[n] is None and _in_corridor(s) for n, s in carts.items()):
        return 0.0
    for half in ("left", "right"):
        gap = _clear_width(objects, parked, half)
        if gap is not None and gap < MIN_CLEAR - 1e-9:
            return 0.0
    return round(CART_CREDIT * sum(p is not None for p in parked.values()), 6)


def success(objects):
    """All four carts parked (still) with at least 140 mm clear in both halves; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in CART_NAMES)


# The demo. Every cart is gripped straight down across its 70 mm length (fingers closing along y, yaw 0), so no
# finger ever comes between a cart and a wall. The left arm parks a green cart in the left near bay, takes the
# first blue cart off the pad (where the right arm left it) to the left far bay, then puts the other green cart on
# the pad; the right arm parks the second blue cart in the right far bay and finally takes the green cart off the pad
# to the right near bay. Each half ends with green near (4.4 cm out) and blue far (8.4 cm out): 15.2 cm clear.
TRAVEL_Z = TOP + 0.15  # a carried cart's bottom 10.5 cm up: clear of the 5 cm walls and 2.5 cm stops
PARK_Z = TOP + 0.16
GRIP_Z = 0.045  # TCP above a cart's bottom: the pad tips 3.2 cm up its ends, 7 mm above the stops' tops
DROP = 0.006  # each cart is let go 6 mm above where it lands
GRIP_FINGER = -(SHALLOW_SIZE[1] - 0.008) / 2  # close to 4 mm inside each end of the 70 mm cart, so it centres
PARK_GAP = 0.004  # carts are set down 4 mm off the wall so they don't scrape it
SHALLOW_NEAR_X = NEAR_X + SHALLOW_SIZE[0] / 2 + PARK_GAP
DEEP_FAR_X = FAR_X - DEEP_SIZE[0] / 2 - PARK_GAP
DEEP_NEAR_X = NEAR_X + DEEP_SIZE[0] / 2 + PARK_GAP  # (for the test layouts: a blue cart at the near wall)
SHALLOW_FAR_X = FAR_X - SHALLOW_SIZE[0] / 2 - PARK_GAP  # (a green cart at the far wall)
CART_Z = FLOOR + CART_H / 2  # a parked cart's centre height
ROOM_Z = TOP + CART_H / 2  # a cart's centre height in a room
LEFT_PARK = (0.22, 0.36, PARK_Z)  # above the emptied left room
RIGHT_PARK = (0.22, -0.36, PARK_Z)  # above the emptied right room
PAD_CLEAR_S = 0.5  # each arm waits this long after the other has left the pad
# The pad is ~0.40 m from both bases: the loaded placer lands short toward its own base and the picker lags toward its
# own, so their y errors add against the 12 mm finger clearance. The placer aims 3 mm past the pad centre (away from its
# base), the picker 3 mm the other way, and both settle above the pad and ease down.
PAD_SPLIT = 0.003
SETTLE_S = 0.6
HOVER_DZ = 0.06  # settle with the fingertips ~13 mm above a cart's top before easing down
# On the GPU a held cart lands 6-13 mm toward the arm's base (-x) after the fast lower: green carts caught the near wall's
# top and tipped, blue carts stopped 8-16 mm short of the far wall. So each bay cart is lowered 25 mm out from its wall
# (on the corridor side), settled, slid in toward the wall 8 mm above the floor, then set down. The aims leave 7 mm off
# the near wall (the arm lands short toward it) and 1 mm off the far wall (the arm lands short away from it).
SLIDE = 0.025
SLIDE_DZ = 0.008
NEAR_AIM_X = NEAR_X + SHALLOW_SIZE[0] / 2 + 0.007
FAR_AIM_X = FAR_X - DEEP_SIZE[0] / 2 - 0.001


def _carry(pick_xy, pick_floor, place_xy, settle_pick=False, settle_place=False, slide=0.0):
    """Top-down pick and place of a cart standing on ``pick_floor``, set down on a bay or the pad (FLOOR). With
    ``slide``, the cart is lowered ``slide`` m away in x, then slid to ``place_xy`` just above the floor."""
    (px, py), (qx, qy) = pick_xy, place_xy
    pz, qz = pick_floor + GRIP_Z, FLOOR + GRIP_Z + DROP
    wps = [Waypoint(1.5, (px, py, TRAVEL_Z), FINGER_OPEN, "to the cart")]
    if settle_pick:
        wps += [
            Waypoint(SETTLE_S, (px, py, TRAVEL_Z), FINGER_OPEN, "settle above"),
            Waypoint(1.0, (px, py, pz + HOVER_DZ), FINGER_OPEN, "hover"),
            Waypoint(0.5, (px, py, pz + HOVER_DZ), FINGER_OPEN, "settle at hover"),
            Waypoint(1.5, (px, py, pz), FINGER_OPEN, "ease down"),
        ]
    else:
        wps.append(Waypoint(1.5, (px, py, pz), FINGER_OPEN, "descend"))
    wps += [
        Waypoint(0.8, (px, py, pz), GRIP_FINGER, "close"),
        Waypoint(1.2, (px, py, TRAVEL_Z), GRIP_FINGER, "lift"),
        Waypoint(1.8, (qx + slide, qy, TRAVEL_Z), GRIP_FINGER, "carry"),
    ]
    if slide:
        wps += [
            Waypoint(1.2, (qx + slide, qy, qz + SLIDE_DZ), GRIP_FINGER, "lower beside"),
            Waypoint(0.4, (qx + slide, qy, qz + SLIDE_DZ), GRIP_FINGER, "settle beside"),
            Waypoint(0.8, (qx, qy, qz + SLIDE_DZ), GRIP_FINGER, "slide in"),
            Waypoint(0.5, (qx, qy, qz), GRIP_FINGER, "set down"),
        ]
    elif settle_place:
        wps += [
            Waypoint(1.0, (qx, qy, qz + HOVER_DZ), GRIP_FINGER, "hover"),
            Waypoint(0.4, (qx, qy, qz + HOVER_DZ), GRIP_FINGER, "settle at hover"),
            Waypoint(1.2, (qx, qy, qz), GRIP_FINGER, "ease down"),
        ]
    else:
        wps.append(Waypoint(1.2, (qx, qy, qz), GRIP_FINGER, "lower"))
    wps += [
        Waypoint(0.6, (qx, qy, qz), FINGER_OPEN, "release"),
        Waypoint(1.0, (qx, qy, TRAVEL_Z), FINGER_OPEN, "retreat"),
    ]
    return wps


def _duration(waypoints):
    return sum(w.duration for w in waypoints)


def demo():
    pad_left_side = (PAD_X, PAD_Y + PAD_SPLIT)  # where the right arm aims to place, and the right arm picks
    pad_right_side = (PAD_X, PAD_Y - PAD_SPLIT)  # where the left arm aims to place, and the left arm picks
    right = _carry(CART_START["deep_cart_1"], TOP, pad_left_side, settle_place=True)
    right_to_pad = _duration(right)  # the right arm has left the pad
    right += _carry(CART_START["deep_cart_2"], TOP, (FAR_AIM_X, -BAY_CY), slide=-SLIDE)

    left = _carry(CART_START["shallow_cart_2"], TOP, (NEAR_AIM_X, BAY_CY), slide=SLIDE)
    left.append(Waypoint(right_to_pad + PAD_CLEAR_S - _duration(left), left[-1].pos, FINGER_OPEN, "wait for the pad"))
    left += _carry(pad_right_side, FLOOR, (FAR_AIM_X, BAY_CY), settle_pick=True, slide=-SLIDE)
    left += _carry(CART_START["shallow_cart_1"], TOP, pad_right_side, settle_place=True)
    left_off_pad = _duration(left)
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park"))

    right.append(Waypoint(left_off_pad + PAD_CLEAR_S - _duration(right), right[-1].pos, FINGER_OPEN, "wait for the pad"))
    right += _carry(pad_left_side, FLOOR, (NEAR_AIM_X, -BAY_CY), settle_pick=True, slide=SLIDE)
    right.append(Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park"))
    return {"left": left, "right": right}
