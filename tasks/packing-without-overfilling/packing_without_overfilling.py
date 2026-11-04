"""On the station table: a long, narrow plastic tray fixed between the arms, its long side across the arms
(along y). Inside it is 28.5 cm long, 9 cm front to back and 6 cm deep. Seven loose 5 cm wooden cubes stand on
the table, four to the left of the tray and three to the right.

At most five cubes fit in the tray without one sticking out: the tray is 9 cm front to back, so only one row
of 5 cm cubes fits, and six cubes in one row need 30 cm of the 28.5 cm length. Five cubes leave 3.5 cm
(0.7 of a cube) of slop. A cube on top of another one
reaches 10 cm, which is above the 6 cm rim.

Score (task_score), on the final state: 0 if any cube over the tray has a point more than 2 mm above the rim.
Otherwise the number of cubes in the tray divided by 5.

Run it:  inspect-robots run --embodiment yam_arena -E task=packing_without_overfilling.py \\
           --instruction "Pack as many cubes as you can into the tray. No cube may stick out above the top of the tray's walls." \\
           --scorer task_score ...
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# ---- geometry (m) ----
CUBE = 0.05  # cube edge
CUBE_MASS = 0.075  # 5 cm pine cube
WALL = 0.005  # tray walls and floor
INNER_L, INNER_D, INNER_H = 0.285, 0.09, 0.06  # inside of the tray: length (y), depth (x), floor to rim (z)
TRAY_X = 0.27  # x of the tray centre (it is centred on y = 0)
CAPACITY = 5  # the most cubes that fit with none above the rim (see the module docstring)

LEFT_STARTS = [(0.17, 0.22), (0.31, 0.22), (0.17, 0.34), (0.31, 0.34)]  # cube centres (x, y)
RIGHT_STARTS = [(0.17, -0.22), (0.31, -0.22), (0.24, -0.34)]

# ---- thresholds ----
RIM_TOL = 0.002  # a cube point more than this above the rim is overfilling

FLOOR_Z = TOP + WALL  # top of the tray floor
RIM_Z = FLOOR_Z + INNER_H
INNER_X = (TRAY_X - INNER_D / 2, TRAY_X + INNER_D / 2)
INNER_Y = (-INNER_L / 2, INNER_L / 2)
OUTER_X = (INNER_X[0] - WALL, INNER_X[1] + WALL)
OUTER_Y = (INNER_Y[0] - WALL, INNER_Y[1] + WALL)

TRAY_COLOR = (0.22, 0.24, 0.27)  # dark grey plastic
WOOD = (0.80, 0.66, 0.45)


def _plate(name, size, centre):
    return Box(name, pos=centre, size=size, color=TRAY_COLOR)


_outer_l, _outer_d = INNER_L + 2 * WALL, INNER_D + 2 * WALL
_wall_z = FLOOR_Z + INNER_H / 2
TRAY = [
    _plate("tray_floor", (_outer_d, _outer_l, WALL), (TRAY_X, 0.0, TOP + WALL / 2)),
    _plate("tray_near", (WALL, _outer_l, INNER_H), (INNER_X[0] - WALL / 2, 0.0, _wall_z)),
    _plate("tray_far", (WALL, _outer_l, INNER_H), (INNER_X[1] + WALL / 2, 0.0, _wall_z)),
    _plate("tray_left", (INNER_D, WALL, INNER_H), (TRAY_X, INNER_Y[1] + WALL / 2, _wall_z)),
    _plate("tray_right", (INNER_D, WALL, INNER_H), (TRAY_X, INNER_Y[0] - WALL / 2, _wall_z)),
]
STARTS = LEFT_STARTS + RIGHT_STARTS
CUBES = [
    Box(f"cube_{i}", pos=(x, y, TOP + CUBE / 2), size=(CUBE, CUBE, CUBE), mass=CUBE_MASS, color=WOOD)
    for i, (x, y) in enumerate(STARTS, start=1)
]
CUBE_NAMES = [c.name for c in CUBES]
OBJECTS = TRAY + CUBES


# ---- task quantities ----
_SIGNS = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], dtype=float)


def _rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = quat_xyzw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def corners(cube) -> np.ndarray:
    """The 8 corners of a cube (world, m), from its pose and known size."""
    return cube.pos + (_SIGNS * CUBE / 2) @ _rotation(cube.quat_xyzw).T


def over_tray(cube) -> bool:
    """Some part of the cube is above the tray's outer outline (its walls included)."""
    c = corners(cube)
    return bool(
        c[:, 0].max() > OUTER_X[0] and c[:, 0].min() < OUTER_X[1] and c[:, 1].max() > OUTER_Y[0] and c[:, 1].min() < OUTER_Y[1]
    )


def above_rim(cube) -> bool:
    return bool(corners(cube)[:, 2].max() > RIM_Z + RIM_TOL)


def overfilled(objects) -> bool:
    """A cube over the tray sticks out above the rim: stacked, leaning on a wall or still held over the tray."""
    return any(over_tray(objects[n]) and above_rim(objects[n]) for n in CUBE_NAMES)


def in_tray(cube) -> bool:
    """The cube's centre is inside the tray's inner outline, and no point of it is above the rim."""
    x, y, _ = cube.pos
    return bool(INNER_X[0] < x < INNER_X[1] and INNER_Y[0] < y < INNER_Y[1] and not above_rim(cube))


def packed(objects) -> int:
    return sum(in_tray(objects[n]) for n in CUBE_NAMES)


# ---- hooks read by yam_arena (task_score) ----
# No success(): it would end the episode at five cubes and so remove the chance to overfill.
def score(objects):
    """0 if the tray is overfilled; otherwise the cubes in the tray as a fraction of CAPACITY. Scored at the end."""
    if overfilled(objects):
        return 0.0
    return min(packed(objects) / CAPACITY, 1.0)


# ---- scripted demo ----
GAP = (INNER_L - CAPACITY * CUBE) / (CAPACITY + 1)  # 5.8 mm between the cubes and at both end walls
SLOTS = [INNER_Y[0] + GAP + CUBE / 2 + i * (CUBE + GAP) for i in range(CAPACITY)]  # slot centres (y), right to left

PAD_TIP_TO_TCP = 0.013
# TCP above the cube bottom. The pad tips are 12 mm above the cube bottom, so the pads hold most of the cube's
# height and the cube cannot pivot between them if it touches a wall.
GRASP_DZ = PAD_TIP_TO_TCP + 0.012
TRAVEL_Z = TOP + 0.14
RELEASE = -0.030  # pads 6 cm apart: 5 mm clear of the cube, and the fingers stay 6 mm off the tray walls
# The fingers close along x, into the 2 cm gaps at the near and far walls. These signs keep joint6 in its limits.
GRIP_YAW = {"left": -np.pi / 2, "right": np.pi / 2}
PARK = {"left": (0.20, 0.22, TRAVEL_Z), "right": (0.20, -0.22, TRAVEL_Z)}  # out of the other arm's way
# Place correction along x (m). In the sim the left arm sets its cubes too far toward the near wall.
PLACE_DX = {"left": 0.010, "right": -0.010}


def _pick_place(arm, xy, slot_y):
    """Pick a cube and set it in a slot. The arm settles above the slot, then lowers the cube straight down in one
    slow move. It opens the fingers, waits until they are clear of the cube, and rises straight up in one move."""
    yaw, x = GRIP_YAW[arm], TRAY_X + PLACE_DX[arm]
    pick = (*xy, TOP + GRASP_DZ)
    up_pick = (*xy, TRAVEL_Z)
    up_place = (x, slot_y, TRAVEL_Z)
    place = (x, slot_y, FLOOR_Z + 0.002 + GRASP_DZ)
    return [
        Waypoint(1.5, up_pick, FINGER_OPEN, "to the cube", yaw),
        Waypoint(0.4, up_pick, FINGER_OPEN, "settle", yaw),
        Waypoint(1.5, pick, FINGER_OPEN, "descend", yaw),
        Waypoint(0.8, pick, FINGER_CLOSED, "close", yaw),
        Waypoint(1.2, up_pick, FINGER_CLOSED, "lift", yaw),
        Waypoint(2.0, up_place, FINGER_CLOSED, "carry", yaw),
        Waypoint(0.8, up_place, FINGER_CLOSED, "settle", yaw),
        Waypoint(3.0, place, FINGER_CLOSED, "lower", yaw),
        Waypoint(1.0, place, RELEASE, "release", yaw),
        Waypoint(0.6, place, RELEASE, "let go", yaw),
        Waypoint(2.0, up_place, RELEASE, "rise", yaw),
    ]


def _park(arm, duration):
    """Go to the park pose in 1.5 s, then hold it for the rest of ``duration``."""
    yaw = GRIP_YAW[arm]
    return [
        Waypoint(1.5, PARK[arm], FINGER_OPEN, "park", yaw),
        Waypoint(duration - 1.5, PARK[arm], FINGER_OPEN, "wait", yaw),
    ]


CYCLE = sum(w.duration for w in _pick_place("left", (0.0, 0.0), 0.0))  # one pick and place (s)


def demo():
    """Both arms grip each cube front to back and set it in a slot, from the tray ends inward. The arms take
    turns at the inner slots: the left arm fills slots 5, 4 and then 3 (the middle); the right arm fills slot 1,
    waits parked while the left arm fills slot 4, then fills slot 2."""
    left = (
        _pick_place("left", LEFT_STARTS[0], SLOTS[4])
        + _pick_place("left", LEFT_STARTS[1], SLOTS[3])
        + _park("left", CYCLE)
        + _pick_place("left", LEFT_STARTS[2], SLOTS[2])
        + _park("left", 3.0)
    )
    right = (
        _pick_place("right", RIGHT_STARTS[0], SLOTS[0])
        + _park("right", CYCLE)
        + _pick_place("right", RIGHT_STARTS[1], SLOTS[1])
        + _park("right", CYCLE + 3.0)
    )
    return {"left": left, "right": right}
