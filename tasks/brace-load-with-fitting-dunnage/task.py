"""On the station table, in front of the left arm: a load bay made of four fixed low walls, holding a loose
wooden crate pushed against the bay's far wall. Between the crate and the bay's near wall (the wall nearest the
robot) is an empty gap 62 mm wide, so the crate can slide 62 mm front to back. To the robot's left of the bay,
four loose wooden dunnage blocks stand in a row; they differ only in thickness (44, 54, 60 and 66 mm). Fill the
gap with the block that fits it closely, so the crate can slide no more than 3 mm.
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
# Load bay: inside 262.5 mm front to back (x) by 162 mm side to side (y); walls 10 mm thick, 40 mm tall.
BAY_CX, BAY_CY = 0.30, 0.11
CRATE = (0.200, 0.160, 0.050)  # x, y, z
CRATE_MASS = 0.8
GAP = 0.062  # between the crate's near face and the near wall, at the start
FAR_GAP = 0.0005  # the crate starts this far off the far wall (so the two do not overlap at spawn)
SIDE_PLAY = 0.001  # per side, between the crate and each side wall
INNER_X = CRATE[0] + GAP + FAR_GAP  # 0.2625
INNER_Y = CRATE[1] + 2 * SIDE_PLAY  # 0.162
WALL_T, WALL_H = 0.010, 0.040
NEAR_IN = BAY_CX - INNER_X / 2  # inside face of the near wall
FAR_IN = BAY_CX + INNER_X / 2  # inside face of the far wall
SIDE_IN = INNER_Y / 2  # inside faces of the side walls at BAY_CY +- this

CRATE_X = FAR_IN - FAR_GAP - CRATE[0] / 2
GAP_CX = NEAR_IN + GAP / 2  # centre of the empty gap

# Dunnage blocks: thickness (x) by 70 mm (y) by 75 mm tall, pine, standing on their 70 mm x thickness face.
BLOCK_W, BLOCK_H = 0.070, 0.075
THICK = {"block_a": 0.054, "block_b": 0.066, "block_c": 0.060, "block_d": 0.044}  # in the row, nearest first
FITS = "block_c"
ROW_Y = 0.30
ROW_X = {"block_a": 0.16, "block_b": 0.24, "block_c": 0.32, "block_d": 0.40}
PINE = 450.0  # kg/m^3

WALL = (0.80, 0.80, 0.82)
WOOD = (0.78, 0.62, 0.40)
CRATE_COL = (0.55, 0.36, 0.20)

_side_len = INNER_X + 2 * WALL_T
BAY = [
    Box("bay_wall_near", pos=(NEAR_IN - WALL_T / 2, BAY_CY, TOP + WALL_H / 2), size=(WALL_T, INNER_Y, WALL_H), color=WALL),
    Box("bay_wall_far", pos=(FAR_IN + WALL_T / 2, BAY_CY, TOP + WALL_H / 2), size=(WALL_T, INNER_Y, WALL_H), color=WALL),
    Box("bay_wall_left", pos=(BAY_CX, BAY_CY + SIDE_IN + WALL_T / 2, TOP + WALL_H / 2), size=(_side_len, WALL_T, WALL_H),
        color=WALL),
    Box("bay_wall_right", pos=(BAY_CX, BAY_CY - SIDE_IN - WALL_T / 2, TOP + WALL_H / 2), size=(_side_len, WALL_T, WALL_H),
        color=WALL),
]
CRATE_BOX = Box("crate", pos=(CRATE_X, BAY_CY, TOP + CRATE[2] / 2), size=CRATE, mass=CRATE_MASS, color=CRATE_COL)
BLOCKS = {
    n: Box(n, pos=(ROW_X[n], ROW_Y, TOP + BLOCK_H / 2), size=(t, BLOCK_W, BLOCK_H),
           mass=round(PINE * t * BLOCK_W * BLOCK_H, 4), color=WOOD)
    for n, t in THICK.items()
}
OBJECTS = [*BAY, CRATE_BOX, *BLOCKS.values()]

FLOOR_TOL = 0.0015  # a block or the crate stands on the floor: its bottom within 1.5 mm of the table top
TILT_MAX = np.radians(3.0)  # standing square: within 3 degrees of upright ...
YAW_MAX = np.radians(5.0)  # ... and its faces within 5 degrees of the bay's walls
FULL_PLAY = 0.003
HALF_PLAY = 0.010


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _square(state) -> bool:
    """Its own x axis (the thickness) within 5 degrees of the bay's length, and some body axis within 3 degrees of
    vertical (so it stands or lies flat, not leaning)."""
    r = _rot(state.quat_xyzw)
    return bool(abs(r[0, 0]) >= np.cos(YAW_MAX) and np.max(np.abs(r[2])) >= np.cos(TILT_MAX))


def _in_bay(state) -> bool:
    """Standing on the bay floor, between its walls (1 mm allowed for contact)."""
    return bool(
        abs(state.low[2] - TOP) <= FLOOR_TOL
        and state.low[0] > NEAR_IN - 0.001 and state.high[0] < FAR_IN + 0.001
        and state.low[1] > BAY_CY - SIDE_IN - 0.001 and state.high[1] < BAY_CY + SIDE_IN + 0.001
    )


def play(objects) -> float:
    """How far the crate can slide front to back (m): the bay's inside length minus the crate's length minus the
    thickness of every block standing square on the bay floor in line with it. Lengths are the objects' own
    dimensions (the crate's 200 mm, each block's thickness), not their bounding boxes, so a turned block is not
    credited with more than its thickness. A block that leans, is turned more than 5 degrees, sits on the crate or a
    wall, or is still moving does not count. A ruler or feeler gauge measures the same thing: the sum of the open
    gaps along the bay."""
    crate = objects["crate"]
    used = CRATE[0]
    for n in BLOCKS:
        b = objects[n]
        if _in_bay(b) and _square(b) and b.resting() and (b.high[0] <= crate.low[0] + 0.001 or b.low[0] >= crate.high[0] - 0.001):
            used += THICK[n]
    return float(INNER_X - used)


def _crate_ok(objects) -> bool:
    c = objects["crate"]
    return bool(_in_bay(c) and _square(c) and c.resting())


def success(objects):
    """The crate can slide at most 3 mm, with everything at rest."""
    return bool(_crate_ok(objects) and play(objects) <= FULL_PLAY)


def distance(objects):
    b = objects[FITS].pos
    return float(np.linalg.norm(b - np.array((GAP_CX, BAY_CY, TOP + BLOCK_H / 2))))


def score(objects):
    """1.0 when the crate can slide at most 3 mm; 0.5 when it can slide at most 10 mm; otherwise 0."""
    if not _crate_ok(objects):
        return 0.0
    p = play(objects)
    if p <= FULL_PLAY:
        return 1.0
    return 0.5 if p <= HALF_PLAY else 0.0


# Demo: the left arm grips the 60 mm block across its 70 mm width (fingers closing along y) near its top, so the
# finger pads stay above the crate (50 mm) and the walls (40 mm) while the block goes down into the gap.
TIP_PAST_TCP = 0.013
GRIP_DEPTH = 0.018  # finger tips this far below the block's top
GRIP_DZ = BLOCK_H / 2 - GRIP_DEPTH + TIP_PAST_TCP  # TCP height above the block's centre
TRAVEL_Z = TOP + 0.15
# Compensation measured with the trace wrapper: gripping turns the block about 4.9 degrees about the vertical
# and, held over the gap, it settles about 4.2 mm short (-x) of the TCP target. The demo turns the grip back by
# that much and aims long by that much so the block goes into the gap square and centred.
SAG_X, SAG_Y, SAG_Z = -0.0042, 0.0, 0.0
YAW_FIX = -0.086
RELEASE_LIFT = 0.003  # block bottom this far above the floor when the fingers open


def demo():
    b = BLOCKS[FITS]
    bx, by, bz = b.pos
    grip = (bx, by, bz + GRIP_DZ)
    tx, ty = GAP_CX - SAG_X, BAY_CY - SAG_Y
    place_z = TOP + BLOCK_H / 2 + RELEASE_LIFT + GRIP_DZ + SAG_Z
    w = [
        Waypoint(1.5, (bx, by, TRAVEL_Z), FINGER_OPEN, "over the 60 mm block"),
        Waypoint(1.5, grip, FINGER_OPEN, "descend"),
        Waypoint(0.8, grip, FINGER_CLOSED, "grip"),
        Waypoint(1.2, (bx, by, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (tx, ty, TRAVEL_Z), FINGER_CLOSED, "carry over the gap", YAW_FIX),
        Waypoint(1.0, (tx, ty, place_z + 0.03), FINGER_CLOSED, "lower to the gap", YAW_FIX),
        Waypoint(1.0, (tx, ty, place_z + 0.03), FINGER_CLOSED, "settle", YAW_FIX),
        Waypoint(2.0, (tx, ty, place_z), FINGER_CLOSED, "lower into the gap", YAW_FIX),
        Waypoint(0.6, (tx, ty, place_z), FINGER_OPEN, "release", YAW_FIX),
        Waypoint(1.2, (tx, ty, TRAVEL_Z), FINGER_OPEN, "up", YAW_FIX),
    ]
    return {"left": w}
