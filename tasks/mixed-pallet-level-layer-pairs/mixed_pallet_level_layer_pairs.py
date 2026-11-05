"""On the station table, between the arms: a brown pallet deck (20 cm front to back, 18 cm side to side, 1.5 cm
tall), fixed, its centre 24 cm in front of the arm bases. In front of the left arm stand a short blue carton (6 x 6 cm,
4 cm tall, 150 g) and a tall green carton (6 x 6 x 6 cm, 200 g); in front of the right arm the same two kinds, the
green one nearer the arms. Between the left arm and the deck lie two orange top cartons (13 x 6 x 3 cm, 200 g),
long side pointing away from the arms. Built, all four base cartons stand flat on the deck and each orange carton
lies level across two base cartons of the same height. Each arm's own pair is one short and one tall carton, so the
equal-height pairs cross the midline.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The pallet deck: a fixed board raised 1.5 cm, so a carton pushed along the table can't slide onto it.
DECK_X, DECK_Y = 0.24, 0.0
DECK_SIZE = (0.20, 0.18, 0.015)  # 20 cm front to back, 18 cm side to side
DECK_Z = TOP + DECK_SIZE[2]  # the deck's top face
DECK_LO = (DECK_X - DECK_SIZE[0] / 2, DECK_Y - DECK_SIZE[1] / 2)
DECK_HI = (DECK_X + DECK_SIZE[0] / 2, DECK_Y + DECK_SIZE[1] / 2)
DECK_BROWN = (0.55, 0.33, 0.14)
DECK = Box("pallet_deck", pos=(DECK_X, DECK_Y, TOP + DECK_SIZE[2] / 2), size=DECK_SIZE, color=DECK_BROWN)

# Base cartons: 6 x 6 cm footprint, short (4 cm) blue or tall (6 cm) green.
BASE_W = 0.06
SHORT_H, TALL_H = 0.04, 0.06
SHORT_KG, TALL_KG = 0.15, 0.20
BLUE = (0.12, 0.30, 0.85)
GREEN = (0.10, 0.62, 0.22)
BASE_START = {  # (x, y) on the table; left side y > 0
    "blue_l": (0.15, 0.38),
    "green_l": (0.25, 0.38),
    "green_r": (0.15, -0.30),
    "blue_r": (0.25, -0.30),
}
BASE_H = {"blue_l": SHORT_H, "green_l": TALL_H, "green_r": TALL_H, "blue_r": SHORT_H}
BASE_NAMES = tuple(BASE_START)
BASES = [
    Box(name, pos=(x, y, TOP + BASE_H[name] / 2 + 0.001), size=(BASE_W, BASE_W, BASE_H[name]),
        mass=SHORT_KG if BASE_H[name] == SHORT_H else TALL_KG, color=BLUE if BASE_H[name] == SHORT_H else GREEN)
    for name, (x, y) in BASE_START.items()
]  # each starts 1 mm above the table, and settles

# Top cartons: 13 cm long, 6 cm wide, 3 cm tall; they start long side front to back (along x).
TOP_SIZE, TOP_KG = (0.13, 0.06, 0.03), 0.20
ORANGE = (0.95, 0.45, 0.08)
TOP_START = {"orange_1": (0.20, 0.165), "orange_2": (0.20, 0.27)}
TOP_NAMES = tuple(TOP_START)
TOPS = [
    Box(name, pos=(x, y, TOP + TOP_SIZE[2] / 2 + 0.001), size=TOP_SIZE, mass=TOP_KG, color=ORANGE)
    for name, (x, y) in TOP_START.items()
]

OBJECTS = [DECK, *BASES, *TOPS]
SUCCESS_HOLD_S = 2.5  # built and still for 2.5 s: long enough to see both upper cartons stay level

# Scoring thresholds (the same numbers are in task.json and the README).
BASE_CREDIT, TOP_CREDIT, TOP_HALF_CREDIT = 0.03, 0.44, 0.22
BASE_UPRIGHT_DEG = 5.0  # a base carton standing flat: a side of its height within 5 degrees of vertical
BASE_SEAT_TOL = 0.003  # its bottom within 3 mm of the deck's top face (not stacked, not off the deck)
LEVEL_FULL_DEG = 2.0  # an upper carton tilted at most 2 degrees (either big face down): full credit
LEVEL_HALF_DEG = 5.0  # 2 to 5 degrees: half credit; more than 5 degrees down on a base carton: score 0
PAIR_TOL = 0.005  # the two supporting cartons' top faces within 5 mm of each other
SEAT_TOL = 0.0005  # the upper carton's lowest point within 0.5 mm of the supports' top faces: resting on them, so
# a carton held even 1 mm above them doesn't count (the scorer can't see the gripper)
MIN_OVER = 0.040  # at least 40 mm of the upper carton's length over each support
MAX_GAP = 0.020  # at most 20 mm between the two supports, along the upper carton's length
MAX_OVERHANG = 0.015  # the upper carton at most 15 mm past the pair's outline on any side
OVER_MIN = 0.010  # "over" a carton: footprints overlapping by at least 10 mm each way
LAYER_TOL = 0.010  # "down on" a base carton: over it, lowest point at most 1 cm above its top face

# The body axes that may point up: a blue carton stands on either 6 x 6 cm face (its 4 cm axis vertical); a green
# carton is a cube with no up side, so any axis may; an orange carton lies on either 13 x 6 cm face.
UP_AXES = {name: (0, 1, 2) if BASE_H[name] == BASE_W else (2,) for name in BASE_NAMES}
UP_AXES.update({name: (2,) for name in TOP_NAMES})


def _tilt_deg(state, axes=(2,)) -> float:
    """Smallest angle between the vertical and any of the given body axes, either end up (0 when flipped over)."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    up = (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))  # body axes' vertical components
    return min(math.degrees(math.acos(min(1.0, abs(up[i])))) for i in axes)


def _overlap(state_a, state_b, axis: int) -> float:
    """Length of the overlap of two footprints along world axis 0 (x) or 1 (y); negative if apart."""
    return float(min(state_a.high[axis], state_b.high[axis]) - max(state_a.low[axis], state_b.low[axis]))


def _over(top, base) -> bool:
    """The upper carton's footprint covers the base carton's by at least OVER_MIN each way (overhead photo)."""
    return _overlap(top, base, 0) >= OVER_MIN and _overlap(top, base, 1) >= OVER_MIN


def _footprint_on_deck(state) -> bool:
    """The carton's footprint, seen from above, lies entirely within the deck's outline."""
    return bool(
        float(state.low[0]) >= DECK_LO[0] and float(state.high[0]) <= DECK_HI[0]
        and float(state.low[1]) >= DECK_LO[1] and float(state.high[1]) <= DECK_HI[1]
    )


def _base_ok(name, state) -> bool:
    """Standing flat (a blue on a 6 x 6 cm face, a green cube on any face) on the deck, not on another carton, its
    footprint entirely on the deck, and still."""
    return bool(
        state.resting()
        and _tilt_deg(state, UP_AXES[name]) <= BASE_UPRIGHT_DEG
        and abs(float(state.low[2]) - DECK_Z) <= BASE_SEAT_TOL
        and _footprint_on_deck(state)
    )


def _bridges(top, b1, b2) -> bool:
    """The upper carton lies across b1 and b2: equal heights, seated on both, 40 mm over each, a gap of at most
    20 mm between them and at most 15 mm of overhang past them. Its length is its longer footprint side."""
    ext = top.high[:2] - top.low[:2]
    along = 0 if ext[0] >= ext[1] else 1
    h1, h2 = float(b1.high[2]), float(b2.high[2])
    if abs(h1 - h2) > PAIR_TOL or abs(float(top.low[2]) - max(h1, h2)) > SEAT_TOL:
        return False
    if not (_over(top, b1) and _over(top, b2)):
        return False
    if min(_overlap(top, b1, along), _overlap(top, b2, along)) < MIN_OVER:
        return False
    gap = max(float(b1.low[along]), float(b2.low[along])) - min(float(b1.high[along]), float(b2.high[along]))
    if gap > MAX_GAP:
        return False
    lo = np.minimum(b1.low[:2], b2.low[:2]) - MAX_OVERHANG
    hi = np.maximum(b1.high[:2], b2.high[:2]) + MAX_OVERHANG
    return bool(np.all(top.low[:2] >= lo) and np.all(top.high[:2] <= hi))


def _down_on(top, base) -> bool:
    """The upper carton is over the base carton and down on it: its lowest point at most LAYER_TOL above the base
    carton's top face (a carton carried higher is not on the layer)."""
    return _over(top, base) and float(top.low[2]) <= float(base.high[2]) + LAYER_TOL


def _tipped_on_bases(objects) -> bool:
    """An upper carton down on a base carton that stands on the deck (its footprint within the deck's outline), and
    tilted more than 5 degrees: it is across cartons of unequal height (or on an edge) and would slide off in transit.
    Base cartons still on the table, or knocked off the deck, don't count here."""
    on_deck = [b for b in BASE_NAMES if _footprint_on_deck(objects[b])]
    return any(
        _tilt_deg(objects[t]) > LEVEL_HALF_DEG and any(_down_on(objects[t], objects[b]) for b in on_deck)
        for t in TOP_NAMES
    )


def _top_credit(top, good_bases) -> float:
    if not top.resting() or _tilt_deg(top) > LEVEL_HALF_DEG:
        return 0.0
    pairs = [(a, b) for i, a in enumerate(good_bases) for b in good_bases[i + 1:]]
    if not any(_bridges(top, a, b) for a, b in pairs):
        return 0.0
    return TOP_CREDIT if _tilt_deg(top) <= LEVEL_FULL_DEG else TOP_HALF_CREDIT


def score(objects):
    """Partial credit (success scores 1.0): 0.03 for each base carton standing flat on the deck; 0.44 for each orange
    carton level (at most 2 degrees, either big face down) across two such cartons of equal height, or 0.22 if
    tilted 2 to 5 degrees. 0 overall if an orange carton down on a base carton standing on the deck is tilted more
    than 5 degrees."""
    if _tipped_on_bases(objects):
        return 0.0
    good = [objects[b] for b in BASE_NAMES if _base_ok(b, objects[b])]
    total = BASE_CREDIT * len(good) + sum(_top_credit(objects[t], good) for t in TOP_NAMES)
    return round(total, 6)


def success(objects):
    """All four bases on the deck and both orange cartons level on equal pairs, everything still; must hold for
    SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in (*BASE_NAMES, *TOP_NAMES))


# The demo. The right arm first stands its green carton in the back row and its blue in the front row, right of the
# midline; then the left arm does the same left of the midline and lays one orange carton across the two blues and
# the other across the two greens, turning each a quarter turn so its long side runs side to side.
BLUE_ROW_X, GREEN_ROW_X = 0.19, 0.29  # 4 cm between the rows: the open fingers clear the other row by 1.3 cm
COL_Y = 0.035  # columns at y = +-3.5 cm: 1 cm between the left and right cartons
BASE_GOAL = {
    "blue_l": (BLUE_ROW_X, COL_Y), "green_l": (GREEN_ROW_X, COL_Y),
    "blue_r": (BLUE_ROW_X, -COL_Y), "green_r": (GREEN_ROW_X, -COL_Y),
}
TOP_GOAL = {"orange_1": (BLUE_ROW_X, 0.0, SHORT_H), "orange_2": (GREEN_ROW_X, 0.0, TALL_H)}  # x, y, support height
TRAVEL_Z = TOP + 0.15
BASE_GRIP = {SHORT_H: 0.025, TALL_H: 0.035}  # TCP above a base carton's bottom: pads cover its top 28 / 33 mm
TOP_GRIP = 0.027  # TCP above an orange carton's bottom: pads cover 16 mm, the tips 14 mm above what it stands on
DROP = 0.007  # each carton is let go this far above where it lands
QUARTER = math.pi / 2  # left arm turns -90 degrees, right arm +90: fingers close front to back over the deck
RIGHT_PARK = (0.20, -0.30, TRAVEL_Z)
LEFT_PARK = (0.20, 0.30, TRAVEL_Z)
LEFT_WAIT_S = 21.0  # the left arm waits above its blue carton until the right arm has parked


def _base_move(name, place_yaw):
    sx, sy = BASE_START[name]
    gx, gy = BASE_GOAL[name]
    grip = BASE_GRIP[BASE_H[name]]
    return pick_place_waypoints(
        (sx, sy, TOP + grip), (gx, gy, DECK_Z + grip + DROP), travel_z=TRAVEL_Z, place_yaw=place_yaw,
    )


def demo():
    right = _base_move("green_r", QUARTER) + _base_move("blue_r", QUARTER)
    right.append(Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park", QUARTER))
    left = [Waypoint(LEFT_WAIT_S, (*BASE_START["blue_l"], TRAVEL_Z), FINGER_OPEN, "wait for the right arm")]
    left += _base_move("blue_l", -QUARTER) + _base_move("green_l", -QUARTER)
    for name in TOP_NAMES:
        gx, gy, support_h = TOP_GOAL[name]
        left += pick_place_waypoints(
            (*TOP_START[name], TOP + TOP_GRIP), (gx, gy, DECK_Z + support_h + TOP_GRIP + DROP),
            travel_z=TRAVEL_Z, place_yaw=-QUARTER,
        )
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park", -QUARTER))
    return {"left": left, "right": right}
