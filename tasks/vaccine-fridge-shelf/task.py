"""An open-topped vaccine-fridge mock facing the arms: two side walls and a back wall (16 cm tall, fixed,
white), with one fixed shelf across the whole inside 7 cm above the table. Its door stands open at 90
degrees on the robot's right, along the fridge's right side, with a door bin (a ledge with a 3 cm lip) on its
inner face. Two vaccine cartons (6 x 4 x 3 cm, pale blue) lie on the table in front of the fridge.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Put the vaccine boxes in the fridge."

WHITE = (0.92, 0.92, 0.92)
T = 0.01  # wall thickness
WALL_H = 0.16
FRONT_X, BACK_X = 0.22, 0.42  # open front; inner face of the back wall
SIDE_Y = 0.13  # inner faces of the side walls
SHELF_TOP = TOP + 0.07
WALL_GAP = 0.05  # CDC: vaccines 2-3 in. (5-7.5 cm) from the walls; at the back the cooling plate can freeze them

# Door bin on the open door (door along y = -0.14, from the fridge's front corner toward the arms).
BIN_X0, BIN_X1 = 0.08, 0.20
BIN_Y0, BIN_Y1 = -0.135, -0.08
BIN_FLOOR = TOP + 0.06
LIP_H = 0.03

FRIDGE = [
    Box("fridge_left", pos=((FRONT_X + BACK_X) / 2, SIDE_Y + T / 2, TOP + WALL_H / 2),
        size=(BACK_X - FRONT_X, T, WALL_H), color=WHITE),
    Box("fridge_right", pos=((FRONT_X + BACK_X) / 2, -SIDE_Y - T / 2, TOP + WALL_H / 2),
        size=(BACK_X - FRONT_X, T, WALL_H), color=WHITE),
    Box("fridge_back", pos=(BACK_X + T / 2, 0.0, TOP + WALL_H / 2), size=(T, 2 * SIDE_Y + 2 * T, WALL_H),
        color=WHITE),
    Box("fridge_shelf", pos=((FRONT_X + BACK_X) / 2, 0.0, SHELF_TOP - T / 2), size=(BACK_X - FRONT_X, 2 * SIDE_Y, T),
        color=(0.80, 0.85, 0.90)),
    Box("fridge_door", pos=(0.13, -0.14 - T / 2, TOP + WALL_H / 2), size=(0.18, T, WALL_H), color=WHITE),
    Box("door_bin_floor", pos=((BIN_X0 + BIN_X1) / 2, (BIN_Y0 + BIN_Y1) / 2, BIN_FLOOR - T / 2),
        size=(BIN_X1 - BIN_X0, BIN_Y1 - BIN_Y0, T), color=(0.80, 0.85, 0.90)),
    Box("door_bin_lip", pos=((BIN_X0 + BIN_X1) / 2, BIN_Y1 + 0.002, BIN_FLOOR + LIP_H / 2),
        size=(BIN_X1 - BIN_X0, 0.004, LIP_H), color=(0.80, 0.85, 0.90)),
]
CARTON = (0.06, 0.04, 0.03)
BLUE = (0.65, 0.80, 0.95)
VAX_A = Box("vaccine_a", pos=(0.15, 0.08, TOP + 0.015), size=CARTON, mass=0.06, color=BLUE)
VAX_B = Box("vaccine_b", pos=(0.15, -0.02, TOP + 0.015), size=CARTON, mass=0.06, color=BLUE)
OBJECTS = [*FRIDGE, VAX_A, VAX_B]
CARTONS = ("vaccine_a", "vaccine_b")


def _flat(s, max_tilt_deg=15.0):
    qx, qy = s.quat_xyzw[0], s.quat_xyzw[1]
    return 1 - 2 * (qx * qx + qy * qy) > np.cos(np.radians(max_tilt_deg))


def _on_shelf(s):
    """Lying flat and still on the shelf: its bottom on the shelf top, its footprint inside the fridge."""
    return bool(
        _flat(s) and s.resting() and abs(s.low[2] - SHELF_TOP) < 0.006
        and s.low[0] > FRONT_X and s.high[0] < BACK_X
        and s.low[1] > -SIDE_Y and s.high[1] < SIDE_Y
    )


def _clear_of_walls(s):
    """Nearest point at least 5 cm from the back wall and from both side walls."""
    return bool(s.high[0] <= BACK_X - WALL_GAP and s.low[1] >= -SIDE_Y + WALL_GAP and s.high[1] <= SIDE_Y - WALL_GAP)


def _good(s):
    """On the shelf, at least 5 cm from every wall."""
    return _on_shelf(s) and _clear_of_walls(s)


def _against_wall(s):
    return _on_shelf(s) and not _clear_of_walls(s)


def _in_door_bin(s):
    """Center over the door bin and below the top of its lip: in it, or resting on its rim."""
    return bool(BIN_X0 < s.pos[0] < BIN_X1 and BIN_Y0 - 0.01 < s.pos[1] < BIN_Y1 + 0.01
                and s.low[2] < BIN_FLOOR + LIP_H + 0.005)


def success(objects):
    """1.0: both cartons flat on the shelf, each at least 5 cm from every wall."""
    return all(_good(objects[c]) for c in CARTONS)


def score(objects):
    """0 if either carton is in the door bin or on the shelf within 5 cm of a wall. Otherwise 0.5 per carton
    flat on the shelf at least 5 cm from every wall (anything else, such as on the table, stacked, or on a
    wall, earns nothing for that carton)."""
    if any(_in_door_bin(objects[c]) or _against_wall(objects[c]) for c in CARTONS):
        return 0.0
    return 0.5 * sum(_good(objects[c]) for c in CARTONS)


def demo():
    """Left arm only (the open door on the right is in the right arm's way): carton A onto the shelf's left
    half, then carton B onto its right half, both 7 cm in from the shelf's front edge, well clear of the
    back wall."""
    travel = TOP + 0.14
    def grip(o):
        return (o.pos[0], o.pos[1], TOP + 0.03 - 0.015 + 0.013)  # pad tips 1.5 cm below the top
    def place(y):
        return (0.29, y, SHELF_TOP + 0.005 + 0.03 - 0.015 + 0.013)
    left = pick_place_waypoints(grip(VAX_A), place(0.05), travel_z=travel)
    left += pick_place_waypoints(grip(VAX_B), place(-0.05), travel_z=travel)
    left.append(Waypoint(1.5, (0.15, 0.25, TOP + 0.25), FINGER_OPEN, "park"))
    return {"left": left}
