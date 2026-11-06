"""On the station table: two open dark grey totes, fixed, one centred 32 cm in front of each arm base (inside 15 cm
front to back, 20 cm side to side, walls 8 cm tall). Between the arms lies a brown inbound mat (25 cm front to back,
27 cm side to side, 5 mm thick), its centre 25 cm in front of the arm bases. On the mat stand five cardboard parcels,
upright: red (7 x 7 x 6 cm, stated 4 kg), blue and green (6 cm cubes, 3 kg each), yellow and orange (5 cm cubes,
2 kg each). The red, yellow and orange parcels are on the left half of the mat, the blue and green on the right
half. The simulated parcels weigh a tenth of their stated weight. Packed, every parcel rests flat on a tote's floor
(on any face) fully inside the tote, and neither tote holds more than 7 kg of stated weight: the only splits are the
red parcel with one 3 kg parcel, and the other 3 kg parcel with both 2 kg parcels.
"""

import math

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The totes: a 4 mm floor and four 5 mm walls, 8 cm tall, around a 15 x 20 cm inside (front to back x side to side).
TOTE_X = 0.32
LEFT_TOTE_Y, RIGHT_TOTE_Y = 0.305, -0.305
INSIDE = (0.15, 0.20)
WALL_T, WALL_H, FLOOR_T = 0.005, 0.08, 0.004
FLOOR = TOP + FLOOR_T
RIM = TOP + WALL_H
TOTE_GREY = (0.22, 0.23, 0.26)
_WZ = TOP + WALL_H / 2


def _tote(side, y):
    x0, x1 = TOTE_X - INSIDE[0] / 2, TOTE_X + INSIDE[0] / 2
    y0, y1 = y - INSIDE[1] / 2, y + INSIDE[1] / 2
    return [
        Box(f"{side}_tote_floor", pos=(TOTE_X, y, TOP + FLOOR_T / 2), size=(*INSIDE, FLOOR_T), color=TOTE_GREY),
        Box(f"{side}_tote_wall_near", pos=(x0 - WALL_T / 2, y, _WZ), size=(WALL_T, INSIDE[1] + 2 * WALL_T, WALL_H),
            color=TOTE_GREY),
        Box(f"{side}_tote_wall_far", pos=(x1 + WALL_T / 2, y, _WZ), size=(WALL_T, INSIDE[1] + 2 * WALL_T, WALL_H),
            color=TOTE_GREY),
        Box(f"{side}_tote_wall_left", pos=(TOTE_X, y1 + WALL_T / 2, _WZ), size=(INSIDE[0], WALL_T, WALL_H),
            color=TOTE_GREY),
        Box(f"{side}_tote_wall_right", pos=(TOTE_X, y0 - WALL_T / 2, _WZ), size=(INSIDE[0], WALL_T, WALL_H),
            color=TOTE_GREY),
    ]


TOTE_Y = {"left": LEFT_TOTE_Y, "right": RIGHT_TOTE_Y}
TOTES = [*_tote("left", LEFT_TOTE_Y), *_tote("right", RIGHT_TOTE_Y)]

# The inbound mat, fixed, between the arms.
MAT_POS, MAT_SIZE = (0.25, 0.0), (0.25, 0.27, 0.005)
MAT_TOP = TOP + MAT_SIZE[2]
MAT = Box("inbound_mat", pos=(*MAT_POS, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=(0.45, 0.28, 0.12))

# The parcels: size (m), simulated mass (kg), stated weight (kg), colour, start (x, y) on the mat.
RED_SIZE, MID_SIZE, SMALL_SIZE = (0.07, 0.07, 0.06), (0.06, 0.06, 0.06), (0.05, 0.05, 0.05)
PARCELS = {
    "parcel_red": (RED_SIZE, 0.40, 4, (0.80, 0.12, 0.10), (0.20, 0.035)),
    "parcel_blue": (MID_SIZE, 0.30, 3, (0.12, 0.30, 0.85), (0.31, -0.075)),
    "parcel_green": (MID_SIZE, 0.30, 3, (0.10, 0.60, 0.20), (0.20, -0.035)),
    "parcel_yellow": (SMALL_SIZE, 0.20, 2, (0.95, 0.80, 0.05), (0.31, 0.075)),
    "parcel_orange": (SMALL_SIZE, 0.20, 2, (0.95, 0.45, 0.05), (0.20, 0.105)),
}
PARCEL_NAMES = tuple(PARCELS)
STATED_KG = {name: p[2] for name, p in PARCELS.items()}
START = {name: p[4] for name, p in PARCELS.items()}
PARCEL_BOXES = [
    Box(name, pos=(*xy, MAT_TOP + size[2] / 2 + 0.001), size=size, mass=kg, color=color)
    for name, (size, kg, _, color, xy) in PARCELS.items()
]  # each starts 1 mm above the mat, and settles

OBJECTS = [*TOTES, MAT, *PARCEL_BOXES]
SUCCESS_HOLD_S = 2.5  # packed and still for 2.5 s

LIMIT_KG = 7  # the most stated weight one tote may hold
PARCEL_CREDIT = 0.2  # for each parcel packed while the split can still be completed
DEAD_END_CREDIT = 0.1  # for each parcel packed once the parcels left out can no longer be added under the limit
IN_TOL = 0.005  # a packed parcel's outline may reach this far past the tote's inside (it leans on a wall)
FLAT_DEG = 10.0  # a packed parcel is tilted at most 10 degrees from resting flat on one of its faces
FLOOR_TOL = 0.005  # a packed parcel's bottom is within 5 mm of the tote's floor (set down, not held above it)
FLOOR_DROP = 0.05  # a parcel whose bottom is more than 5 cm below the table top is on the floor


def _tilt_from_flat_deg(state) -> float:
    """Angle between the vertical and the nearest of the object's three axes: 0 when it rests flat on any face."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    cos_up = max(abs(2 * (x * z - w * y)), abs(2 * (y * z + w * x)), abs(1 - 2 * (x * x + y * y)))
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_up))))


def _over_tote(state, y) -> bool:
    """Centre over the tote's outer outline, walls included: its weight is on the tote, at any height or tilt."""
    px, py = float(state.pos[0]), float(state.pos[1])
    return bool(abs(px - TOTE_X) <= INSIDE[0] / 2 + WALL_T and abs(py - y) <= INSIDE[1] / 2 + WALL_T)


def _packed(state, y) -> bool:
    """Set down fully inside the tote: its outline within the inside (5 mm slack), bottom within 5 mm of the floor,
    top no higher than the rim, tilted at most 10 degrees from resting flat on one of its faces."""
    lo, hi = state.low, state.high
    return bool(
        float(lo[0]) >= TOTE_X - INSIDE[0] / 2 - IN_TOL and float(hi[0]) <= TOTE_X + INSIDE[0] / 2 + IN_TOL
        and float(lo[1]) >= y - INSIDE[1] / 2 - IN_TOL and float(hi[1]) <= y + INSIDE[1] / 2 + IN_TOL
        and abs(float(lo[2]) - FLOOR) <= FLOOR_TOL and float(hi[2]) <= RIM
        and _tilt_from_flat_deg(state) <= FLAT_DEG
    )


def _tote_kg(objects, y) -> float:
    return sum(STATED_KG[p] for p in PARCEL_NAMES if _over_tote(objects[p], y))


def _on_floor(state) -> bool:
    return float(state.low[2]) < TOP - FLOOR_DROP


def _completable(objects) -> bool:
    """The parcels over neither tote can still be added, each to one tote, without either tote going over 7 kg."""
    left, right = (_tote_kg(objects, y) for y in TOTE_Y.values())
    out = [STATED_KG[p] for p in PARCEL_NAMES if not any(_over_tote(objects[p], y) for y in TOTE_Y.values())]
    for mask in range(2 ** len(out)):
        to_left = sum(kg for i, kg in enumerate(out) if mask >> i & 1)
        if left + to_left <= LIMIT_KG and right + sum(out) - to_left <= LIMIT_KG:
            return True
    return False


def score(objects):
    """Partial credit (success scores 1.0): 0.2 for each parcel packed (set down flat, fully inside a tote) while the
    parcels left out can still be added without either tote going over 7 kg; 0.1 for each once they can't. 0 overall
    if a tote holds more than 7 kg of stated weight or a parcel is on the floor."""
    if any(_on_floor(objects[p]) for p in PARCEL_NAMES):
        return 0.0
    if any(_tote_kg(objects, y) > LIMIT_KG for y in TOTE_Y.values()):
        return 0.0
    packed = sum(any(_packed(objects[p], y) for y in TOTE_Y.values()) for p in PARCEL_NAMES)
    credit = PARCEL_CREDIT if _completable(objects) else DEAD_END_CREDIT
    return round(credit * packed, 6)


def success(objects):
    """All five parcels packed with neither tote over 7 kg; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[p].resting() for p in PARCEL_NAMES)


# The demo: the left arm packs the green, yellow and orange parcels (7 kg) into its tote and the right arm the red and
# blue parcels (7 kg) into its tote, each parcel set down in a row across the tote at its centre line. Every grip is
# turned 90 degrees, the fingers closing front to back, so the open fingers stay clear of the neighbouring parcels on
# the mat and of the tote walls.
TRAVEL_Z = TOP + 0.15  # a carried parcel's bottom clears the tote rims by 4 cm
GRIP = 0.03  # TCP above a parcel's bottom: the pads cover its side from 1.7 cm to 5 cm up
DROP = 0.008  # each parcel is let go this far above the tote floor
# The grip is turned a quarter turn toward each arm's own side (left -90, right +90 degrees): the same grip for a
# parallel gripper, and it keeps the wrist within its limits when an arm reaches across the mat.
YAW = {"left": -math.pi / 2, "right": math.pi / 2}
PLACE = {
    "parcel_yellow": (TOTE_X, LEFT_TOTE_Y - 0.065),
    "parcel_orange": (TOTE_X, LEFT_TOTE_Y - 0.005),
    "parcel_green": (TOTE_X, LEFT_TOTE_Y + 0.06),
    "parcel_blue": (TOTE_X, RIGHT_TOTE_Y + 0.055),
    "parcel_red": (TOTE_X, RIGHT_TOTE_Y - 0.03),
}
LEFT_PARK = (0.15, LEFT_TOTE_Y, TRAVEL_Z)
RIGHT_PARK = (0.15, RIGHT_TOTE_Y, TRAVEL_Z)
RIGHT_WAIT_S = 9.6  # the right arm waits above its tote while the left arm packs the orange parcel
LEFT_WAIT_S = 6.8  # the left arm waits above its tote until the right arm has carried the red parcel away


def _move(name, side):
    x, y = START[name]
    return pick_place_waypoints(
        (x, y, MAT_TOP + GRIP), (*PLACE[name], FLOOR + GRIP + DROP), travel_z=TRAVEL_Z, yaw=YAW[side],
    )


def demo():
    ly, ry = YAW["left"], YAW["right"]
    left = _move("parcel_yellow", "left") + _move("parcel_orange", "left")
    left.append(Waypoint(LEFT_WAIT_S, (TOTE_X, LEFT_TOTE_Y, TRAVEL_Z), FINGER_OPEN, "wait for the red parcel", ly))
    left += _move("parcel_green", "left")
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park", ly))
    right = _move("parcel_blue", "right")
    right.append(Waypoint(RIGHT_WAIT_S, (TOTE_X, RIGHT_TOTE_Y, TRAVEL_Z), FINGER_OPEN, "wait", ry))
    right += _move("parcel_red", "right")
    right.append(Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park", ry))
    return {"left": left, "right": right}
