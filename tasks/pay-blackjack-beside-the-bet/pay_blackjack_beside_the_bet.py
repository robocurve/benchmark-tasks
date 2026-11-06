"""On the station table, a blackjack settlement. Two green betting circles (15 cm across, 2 mm thick) lie 30 cm in
front of the arm bases: circle A 14 cm left of the midline, circle B 14 cm right of it. A fixed dark grey chip rack of
four square wells in a row (56 mm inside, walls 6 mm thick and 8 mm tall, no floor) stands 16 cm in front of the arm
bases on the right, its wells 20.0, 26.2, 32.4 and 38.6 cm right of the midline. A yellow handover square (7 cm, 2 mm
thick) lies on the midline 22 cm in front of the arm bases. Six identical red chips (40 mm across, 30 mm tall, 50 g):
two stacked at the centre of circle A (the player's 2-chip bet), one at the centre of circle B (a busted bet), and one
in each of the three wells nearest the midline (the house bank); the outermost well is empty. The circles, the
square and the rack are fixed; the chips are loose. Settled: B's chip in the rack, and a separate 3-chip stack (3 to 2
on a 2-chip bet) standing on circle A beside the untouched bet, never on top of it.
"""

import math

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Betting circles: 2 mm thick fixed discs, 15 cm across.
CIRCLE_R, CIRCLE_T = 0.075, 0.002
CIRCLE_TOP = TOP + CIRCLE_T
A_X, A_Y = 0.30, 0.14  # circle A (the blackjack), left arm only
B_X, B_Y = 0.30, -0.14  # circle B (the busted bet), right arm only
CIRCLE_GREEN = (0.10, 0.55, 0.25)
CIRCLES = [
    Cylinder("circle_a", pos=(A_X, A_Y, TOP + CIRCLE_T / 2), radius=CIRCLE_R, height=CIRCLE_T, color=CIRCLE_GREEN),
    Cylinder("circle_b", pos=(B_X, B_Y, TOP + CIRCLE_T / 2), radius=CIRCLE_R, height=CIRCLE_T, color=CIRCLE_GREEN),
]

# The handover square on the midline: both arms reach it.
HAND_X, HAND_Y = 0.22, 0.0
HAND_SIZE, HAND_T = 0.07, 0.002
HAND_TOP = TOP + HAND_T
HAND = Box("handover", pos=(HAND_X, HAND_Y, TOP + HAND_T / 2), size=(HAND_SIZE, HAND_SIZE, HAND_T),
           color=(0.95, 0.78, 0.08))

# The chip rack: four 56 mm square wells in a row along y, 6 mm walls 8 mm tall, standing on the table (no floor).
RACK_X = 0.16
WELL_IN, WALL_T, WALL_H = 0.056, 0.006, 0.008
PITCH = WELL_IN + WALL_T
WELL_Y = tuple(round(-0.200 - PITCH * i, 4) for i in range(4))  # -0.200, -0.262, -0.324, -0.386
RACK_GREY = (0.20, 0.21, 0.23)
_RACK_MID_Y = (WELL_Y[0] + WELL_Y[3]) / 2
_RACK_LEN = 4 * WELL_IN + 5 * WALL_T
_WZ = TOP + WALL_H / 2
RACK = [
    Box("rack_near", pos=(RACK_X - WELL_IN / 2 - WALL_T / 2, _RACK_MID_Y, _WZ), size=(WALL_T, _RACK_LEN, WALL_H),
        color=RACK_GREY),
    Box("rack_far", pos=(RACK_X + WELL_IN / 2 + WALL_T / 2, _RACK_MID_Y, _WZ), size=(WALL_T, _RACK_LEN, WALL_H),
        color=RACK_GREY),
    *(Box(f"rack_divider_{i}", pos=(RACK_X, WELL_Y[0] + PITCH / 2 - PITCH * i, _WZ), size=(WELL_IN, WALL_T, WALL_H),
          color=RACK_GREY) for i in range(5)),
]

# The chips: identical red pucks, a stylised casino chip (real 40 mm diameter, 30 mm thick so a gripper can hold it).
CHIP_R, CHIP_H, CHIP_KG = 0.020, 0.030, 0.05
CHIP_RED = (0.80, 0.10, 0.10)
# Each chip starts 0.3 mm above what it rests on (the bet's top chip 0.6 mm above its level), inside the 1 mm a seated
# chip may sit above its level, so a chip nobody moves already counts as seated before it settles.
GAP = 0.0003
CHIP_START = {
    "bet_low": (A_X, A_Y, CIRCLE_TOP + CHIP_H / 2 + GAP),
    "bet_high": (A_X, A_Y, CIRCLE_TOP + 1.5 * CHIP_H + 2 * GAP),
    "bust": (B_X, B_Y, CIRCLE_TOP + CHIP_H / 2 + GAP),
    "bank_1": (RACK_X, WELL_Y[0], TOP + CHIP_H / 2 + GAP),
    "bank_2": (RACK_X, WELL_Y[1], TOP + CHIP_H / 2 + GAP),
    "bank_3": (RACK_X, WELL_Y[2], TOP + CHIP_H / 2 + GAP),
}
CHIPS = [
    Cylinder(name, pos=pos, radius=CHIP_R, height=CHIP_H, mass=CHIP_KG, color=CHIP_RED)
    for name, pos in CHIP_START.items()
]
CHIP_NAMES = tuple(CHIP_START)

OBJECTS = [*CIRCLES, HAND, *RACK, *CHIPS]
SUCCESS_HOLD_S = 2.5  # settled and still for 2.5 s

# Scoring. Chips are identical, so they are scored by where they stand, not by which one is where.
PAYOUT_CREDIT, COLLECT_CREDIT, BET_CREDIT = 0.60, 0.25, 0.15
PAYOUT_CHIPS = 3  # 3 to 2 on a 2-chip bet
FLAT_DEG = 10.0  # a chip stands flat when its axis is within 10 degrees of vertical (either face up)
SEAT_DOWN, SEAT_UP = 0.004, 0.001  # a chip's bottom from 4 mm below to 1 mm above its level counts as seated on it
BET_FIND_R = 0.030  # the bet: a 2-chip stack whose lowest chip's axis is within 30 mm of A's centre
BESIDE_MIN, BESIDE_MAX = 0.040, 0.060  # the payout's lowest chip: axis 40-60 mm from the nearer bet chip's axis
ON_A_R = CIRCLE_R  # a chip stands on circle A when its axis is within 75 mm of A's centre
STACK_TOL = 0.015  # each chip in a stack: axis within 15 mm of the stack's lowest chip's axis (over half on its base)
CAP_R = 0.038  # a chip whose axis is within 38 mm of the bet's axis overlaps the bet (by 2 mm or more)
CAP_FROM = 2 * CHIP_H - SEAT_DOWN  # 56 mm: a capping chip's bottom is at least this far above circle A
BET_TOL = 0.005  # the bet untouched: both chips' axes within 5 mm of A's centre
NEAR_R = CIRCLE_R + CHIP_R  # 95 mm: a chip whose axis is this close to a circle's centre touches the circle
WELL_TOL = 0.010  # a chip in a well: axis within 10 mm of the well's centre


def _tilt_deg(state) -> float:
    """Angle between the chip's axis and the vertical, either face up."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


def _dist(state, xy) -> float:
    """Horizontal distance from the chip's axis to a point."""
    return math.hypot(float(state.pos[0]) - xy[0], float(state.pos[1]) - xy[1])


def _flat(state) -> bool:
    return _tilt_deg(state) <= FLAT_DEG and state.resting()


def _level(state, floor):
    """Which stack level the chip's bottom sits at above ``floor`` (0 on the floor, 1 on one chip, ...), or None."""
    h = float(state.low[2]) - floor
    k = round(h / CHIP_H)
    if k < 0 or not (-SEAT_DOWN <= h - k * CHIP_H <= SEAT_UP):
        return None
    return int(k)


def _seated_on_a(state) -> bool:
    """Standing flat on circle A at a stack level, axis within 75 mm of A's centre."""
    return _dist(state, (A_X, A_Y)) <= ON_A_R and _flat(state) and _level(state, CIRCLE_TOP) is not None


def _bases(objects, near, rmin, rmax):
    """Chips standing flat on circle A at the first level whose axis is rmin..rmax from ``near``."""
    return [
        c for c in CHIP_NAMES
        if rmin <= _dist(objects[c], near) <= rmax and _seated_on_a(objects[c]) and _level(objects[c], CIRCLE_TOP) == 0
    ]


def _bet(objects):
    """The bet: the stack on the first-level chip nearest A's centre, if its axis is within 30 mm of it ([] if none)."""
    bases = _bases(objects, (A_X, A_Y), 0.0, BET_FIND_R)
    if not bases:
        return []
    return _column(objects, min(bases, key=lambda c: _dist(objects[c], (A_X, A_Y))))


def _capped(objects, axis) -> bool:
    """A chip down on top of the bet, moving or not: its axis within 38 mm of the bet's, its bottom 56 mm or more
    above circle A, and within 1 mm of the top of another chip there."""
    spot = [c for c in CHIP_NAMES if _dist(objects[c], axis) <= CAP_R]
    for c in spot:
        bottom = float(objects[c].low[2])
        if bottom < CIRCLE_TOP + CAP_FROM:
            continue
        under = [float(objects[o].high[2]) for o in spot if o != c and float(objects[o].high[2]) <= bottom + SEAT_UP]
        if under and bottom <= max(under) + SEAT_UP:
            return True
    return False


def _column(objects, base):
    """The chips stacked level by level on ``base``: flat, still, axes within STACK_TOL of the base's axis."""
    bx, by = float(objects[base].pos[0]), float(objects[base].pos[1])
    levels = {}
    for c in CHIP_NAMES:
        s = objects[c]
        if _dist(s, (bx, by)) <= STACK_TOL and _flat(s):
            k = _level(s, CIRCLE_TOP)
            if k is not None:
                levels.setdefault(k, []).append(c)
    column, k = [], 0
    while len(levels.get(k, ())) == 1:
        column += levels[k]
        k += 1
    return column


def _beside(objects, bet, c) -> bool:
    """Chip ``c`` stands flat on circle A at the first level, its axis 40-60 mm from the nearer bet chip's axis."""
    near = min(_dist(objects[c], (float(objects[b].pos[0]), float(objects[b].pos[1]))) for b in bet)
    return (
        c not in bet and BESIDE_MIN <= near <= BESIDE_MAX
        and _seated_on_a(objects[c]) and _level(objects[c], CIRCLE_TOP) == 0
    )


def _payout(objects, bet):
    """The tallest stack on circle A whose lowest chip is beside the bet ([] if none)."""
    bases = [c for c in CHIP_NAMES if _beside(objects, bet, c)]
    return max((_column(objects, b) for b in bases), key=len, default=[])


def _in_well(state) -> bool:
    """Seated in a rack well: axis within 10 mm of a well's centre, flat, still, bottom on the table."""
    return (
        any(_dist(state, (RACK_X, wy)) <= WELL_TOL for wy in WELL_Y)
        and _flat(state)
        and _level(state, TOP) == 0
    )


def score(objects):
    """0 overall if the bet is broken or gone (no 2-chip stack whose lowest chip's axis is within 30 mm of A's centre)
    or capped (a chip down on top of it). Otherwise: 0.60 for the payout (a stack of exactly 3 chips whose lowest
    chip's axis is 40-60 mm from the nearer bet chip's axis, and no other chip within 95 mm of A's centre); 0.25 for
    B collected with no chip loose (no chip within 95 mm of B's centre; at least one chip seated in a rack well; every
    chip other than the bet seated in a rack well or in one single stack on circle A); 0.15 for the bet untouched (both
    its chips within 5 mm of A's centre), given only when at least one chip stands beside the bet."""
    bet = _bet(objects)
    if len(bet) != 2:
        return 0.0
    axis = tuple(sum(float(objects[c].pos[i]) for c in bet) / 2 for i in (0, 1))
    if _capped(objects, axis):
        return 0.0
    payout = _payout(objects, bet)
    strays = [c for c in CHIP_NAMES if _dist(objects[c], (A_X, A_Y)) <= NEAR_R and c not in bet and c not in payout]
    paid = len(payout) == PAYOUT_CHIPS and not strays
    racked = [c for c in CHIP_NAMES if c not in bet and _in_well(objects[c])]
    rest = {c for c in CHIP_NAMES if c not in bet and c not in racked}
    one_stack = not rest or any(
        set(_column(objects, b)) == rest for b in _bases(objects, (A_X, A_Y), 0.0, ON_A_R) if b in rest
    )
    collected = bool(racked) and all(_dist(objects[c], (B_X, B_Y)) > NEAR_R for c in CHIP_NAMES) and one_stack
    untouched = bool(payout) and all(_dist(objects[c], (A_X, A_Y)) <= BET_TOL for c in bet)
    return round(PAYOUT_CREDIT * paid + COLLECT_CREDIT * collected + BET_CREDIT * untouched, 6)


def success(objects):
    """Paid, collected and untouched, with every chip still; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[c].resting() for c in CHIP_NAMES)


# The demo. The right arm moves a bank chip from the rack to the yellow square, takes B's chip to the empty well, then
# feeds the other two bank chips to the square one at a time. The left arm takes each chip off the square and stacks
# the payout 5 cm right of A's centre (toward the midline), its fingers closing front to back so they stay beside
# the payout and away from the bet. Both arms grip with the fingers closing along world x (yaw +-90 degrees).
TRAVEL_Z = TOP + 0.15  # a carried chip's bottom 12 cm up, clear of both 6 cm stacks
GRIP_Z = 0.027  # TCP above a chip's bottom: pad tips 14 mm up its side, 6 mm above the rack walls
GRIP_FINGER = -(CHIP_R - 0.004)  # close each finger 4 mm inside the chip's side, so the pair centres it
DROP = 0.005  # each chip is let go this far above where it lands
EASE = 0.03  # slow the last 3 cm of each placement
LEFT_YAW, RIGHT_YAW = -math.pi / 2, math.pi / 2
PAY_X, PAY_Y = A_X, A_Y - 0.05  # the payout's axis: 50 mm from the bet's, a 10 mm gap
LEFT_WAIT = (0.18, 0.20, TRAVEL_Z)  # where the left arm waits for each chip, 18 cm from the square
LEFT_PARK = (0.24, 0.30, TRAVEL_Z)
RIGHT_PARK = (0.24, -0.30, TRAVEL_Z)
PICK_S = 5.0  # to the chip, descend, close, lift
CLEAR_S = 1.5  # the right arm's first move after the square, which takes it away
MARGIN_S = 0.5


def _move(pick, place, yaw, hold_s=0.0):
    """Top-down pick and place with a centring grip, an optional wait after the lift, and a slow last 3 cm."""
    (px, py, pz), (qx, qy, qz) = pick, place
    wps = [
        Waypoint(1.5, (px, py, TRAVEL_Z), FINGER_OPEN, "to the chip", yaw),
        Waypoint(1.5, (px, py, pz), FINGER_OPEN, "descend", yaw),
        Waypoint(0.8, (px, py, pz), GRIP_FINGER, "close", yaw),
        Waypoint(1.2, (px, py, TRAVEL_Z), GRIP_FINGER, "lift", yaw),
    ]
    if hold_s > 0:
        wps.append(Waypoint(hold_s, (px, py, TRAVEL_Z), GRIP_FINGER, "wait for the square to clear", yaw))
    return wps + [
        Waypoint(1.8, (qx, qy, TRAVEL_Z), GRIP_FINGER, "carry", yaw),
        Waypoint(1.2, (qx, qy, qz + EASE), GRIP_FINGER, "lower", yaw),
        Waypoint(0.5, (qx, qy, qz + EASE), GRIP_FINGER, "settle", yaw),
        Waypoint(0.8, (qx, qy, qz), GRIP_FINGER, "ease down", yaw),
        Waypoint(0.6, (qx, qy, qz), FINGER_OPEN, "release", yaw),
        Waypoint(1.0, (qx, qy, TRAVEL_Z), FINGER_OPEN, "retreat", yaw),
    ]


def _t(wps) -> float:
    return sum(w.duration for w in wps)


def demo():
    hand_pick = (HAND_X, HAND_Y, HAND_TOP + GRIP_Z)
    hand_place = (HAND_X, HAND_Y, HAND_TOP + GRIP_Z + DROP)

    def rack_pick(i):
        return (RACK_X, WELL_Y[i], TOP + GRIP_Z)

    def pay_place(k):
        return (PAY_X, PAY_Y, CIRCLE_TOP + k * CHIP_H + GRIP_Z + DROP)

    right = _move(rack_pick(0), hand_place, RIGHT_YAW)
    clear_1 = _t(right) + CLEAR_S
    right += _move((B_X, B_Y, CIRCLE_TOP + GRIP_Z), (RACK_X, WELL_Y[3], TOP + GRIP_Z + DROP), RIGHT_YAW)
    right += _move(rack_pick(1), hand_place, RIGHT_YAW)
    clear_2 = _t(right) + CLEAR_S
    # the left arm has stacked chip 2 and lifted off the payout (its stacking move takes as long as any _move)
    left_off_2 = clear_2 + _t(_move(hand_pick, pay_place(1), LEFT_YAW)) + MARGIN_S
    hold = max(0.0, left_off_2 - (_t(right) + PICK_S))
    right += _move(rack_pick(2), hand_place, RIGHT_YAW, hold_s=hold)
    right.append(Waypoint(CLEAR_S, RIGHT_PARK, FINGER_OPEN, "park", RIGHT_YAW))
    clear_3 = _t(right)
    right.append(Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "stay parked", RIGHT_YAW))

    left = []
    for k, clear in enumerate((clear_1, clear_2, clear_3)):
        left.append(Waypoint(1.5, LEFT_WAIT, FINGER_OPEN, "to the waiting spot", LEFT_YAW))
        left.append(Waypoint(max(0.2, clear - _t(left)), LEFT_WAIT, FINGER_OPEN, "wait for the next chip", LEFT_YAW))
        left += _move(hand_pick, pay_place(k), LEFT_YAW)
    left.append(Waypoint(2.5, LEFT_PARK, FINGER_OPEN, "park", LEFT_YAW))
    return {"left": left, "right": right}
