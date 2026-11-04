"""On the station table, between the arms: an open dark grey grocery tote (inside 16 cm front to back, 10 cm side to
side, walls 6 cm tall), its centre 24 cm in front of the arm bases. In front of the left arm stand two red food cans
(7 cm across, 8 cm tall, 400 g each), 20 and 32 cm in front of the arm bases. In front of the right arm lies a blue
carton of six eggs (14 cm long front to back, 8 cm wide, 6 cm tall, 400 g), 25 cm in front of the arm bases. The tote
is fixed; the cans and the carton are loose. Packed, both cans stand in the tote and the egg carton lies flat on top
of them, inside the tote's outline, with nothing resting on it.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The tote: a 4 mm floor and four 5 mm walls, 6 cm tall, around a 16 x 10 cm inside.
TOTE_X, TOTE_Y = 0.24, 0.0
INSIDE = (0.16, 0.10)
WALL_T, WALL_H, FLOOR_T = 0.005, 0.06, 0.004
FLOOR = TOP + FLOOR_T
IN_X = (TOTE_X - INSIDE[0] / 2, TOTE_X + INSIDE[0] / 2)
IN_Y = (TOTE_Y - INSIDE[1] / 2, TOTE_Y + INSIDE[1] / 2)
TOTE_GREY = (0.20, 0.21, 0.23)
_WZ = TOP + WALL_H / 2
TOTE = [
    Box("tote_floor", pos=(TOTE_X, TOTE_Y, TOP + FLOOR_T / 2), size=(*INSIDE, FLOOR_T), color=TOTE_GREY),
    Box("tote_wall_near", pos=(IN_X[0] - WALL_T / 2, TOTE_Y, _WZ), size=(WALL_T, INSIDE[1] + 2 * WALL_T, WALL_H),
        color=TOTE_GREY),
    Box("tote_wall_far", pos=(IN_X[1] + WALL_T / 2, TOTE_Y, _WZ), size=(WALL_T, INSIDE[1] + 2 * WALL_T, WALL_H),
        color=TOTE_GREY),
    Box("tote_wall_left", pos=(TOTE_X, IN_Y[1] + WALL_T / 2, _WZ), size=(INSIDE[0], WALL_T, WALL_H), color=TOTE_GREY),
    Box("tote_wall_right", pos=(TOTE_X, IN_Y[0] - WALL_T / 2, _WZ), size=(INSIDE[0], WALL_T, WALL_H), color=TOTE_GREY),
]

CAN_R, CAN_H, CAN_KG = 0.035, 0.08, 0.4
CAN_RED = (0.80, 0.14, 0.10)
CAN_START = {"can_1": (0.20, 0.30), "can_2": (0.32, 0.30)}
CANS = [
    Cylinder(name, pos=(x, y, TOP + CAN_H / 2 + 0.001), radius=CAN_R, height=CAN_H, mass=CAN_KG, color=CAN_RED)
    for name, (x, y) in CAN_START.items()
]  # each starts 1 mm above the table, and settles
CAN_NAMES = tuple(CAN_START)

EGGS_SIZE, EGGS_KG = (0.14, 0.08, 0.06), 0.4  # long side front to back
EGGS_BLUE = (0.15, 0.35, 0.85)  # saturated: pale objects vanish from the top camera
EGGS_START = (0.25, -0.30)
EGGS = Box("eggs", pos=(*EGGS_START, TOP + EGGS_SIZE[2] / 2 + 0.001), size=EGGS_SIZE, mass=EGGS_KG, color=EGGS_BLUE)

OBJECTS = [*TOTE, *CANS, EGGS]
SUCCESS_HOLD_S = 2.5  # packed and still for 2.5 s: long enough to see the stack is stable

IN_TOL = 0.005  # an object's centre may sit this far past the tote's inside (it can lean on a wall)
CAN_CREDIT, EGGS_ON_TOP_CREDIT, EGGS_IN_CREDIT = 0.2, 0.6, 0.1
EGGS_FLAT_DEG = 10.0  # on top of the cans: tilted at most 10 degrees
EGGS_UPRIGHT_DEG = 30.0  # in the tote but not on top: tilted at most 30 degrees
ON_TOP_SLACK = 0.01  # the carton's bottom at most 1 cm below the higher can's top


def _tilt_deg(state) -> float:
    """Angle between the object's up axis and the vertical."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def _in_tote(state) -> bool:
    """Centre inside the tote's outline (or leaning just past it), and bottom no lower than the tote's floor."""
    x, y, _ = (float(v) for v in state.pos)
    return bool(
        IN_X[0] - IN_TOL <= x <= IN_X[1] + IN_TOL
        and IN_Y[0] - IN_TOL <= y <= IN_Y[1] + IN_TOL
        and float(state.low[2]) >= FLOOR - 0.005
    )


def _overlap_xy(a, b) -> bool:
    return bool(np.all(a.low[:2] < b.high[:2]) and np.all(b.low[:2] < a.high[:2]))


def _can_on_eggs(objects) -> bool:
    """A can resting on the carton (its bottom at or above the carton's top, over it): the eggs are crushed."""
    eggs = objects["eggs"]
    return any(
        float(objects[c].low[2]) >= float(eggs.high[2]) - 0.01 and _overlap_xy(objects[c], eggs) for c in CAN_NAMES
    )


def _eggs_on_top(objects) -> bool:
    """The carton in the tote's outline, flat, above both cans, and still."""
    eggs = objects["eggs"]
    cans_top = max(float(objects[c].high[2]) for c in CAN_NAMES)
    return bool(
        _in_tote(eggs)
        and _tilt_deg(eggs) <= EGGS_FLAT_DEG
        and float(eggs.low[2]) >= cans_top - ON_TOP_SLACK
        and all(_in_tote(objects[c]) for c in CAN_NAMES)
        and eggs.resting()
    )


def score(objects):
    """Partial credit (success scores 1.0): 0.2 for each can in the tote; 0.6 for the egg carton flat on top of both
    cans in the tote, or 0.1 if it is in the tote upright (tilted at most 30 degrees) but not on top. 0 overall if a
    can rests on the carton."""
    if _can_on_eggs(objects):
        return 0.0
    cans = sum(_in_tote(objects[c]) for c in CAN_NAMES)
    eggs = objects["eggs"]
    if _eggs_on_top(objects):
        eggs_credit = EGGS_ON_TOP_CREDIT
    elif _in_tote(eggs) and _tilt_deg(eggs) <= EGGS_UPRIGHT_DEG:
        eggs_credit = EGGS_IN_CREDIT
    else:
        eggs_credit = 0.0
    return round(CAN_CREDIT * cans + eggs_credit, 6)


def success(objects):
    """Both cans in the tote and the carton flat on top of them; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[c].resting() for c in CAN_NAMES)


# The demo: the left arm stands both cans in the tote, one behind the other; then the right arm lays the carton on
# top of them, its long side front to back. Both grip straight down with the fingers closing along world y.
TRAVEL_Z = TOP + 0.15  # a carried can's bottom clears the walls by 4 cm; the carton's clears the cans' tops by 2 cm
CAN_GRIP = 0.045  # TCP above a can's bottom: the pad tips 3.2 cm up its side
EGGS_GRIP = 0.035  # TCP above the carton's bottom
DROP = 0.008  # each object is let go this far above where it lands
CAN_PLACE_X = (TOTE_X - 0.0375, TOTE_X + 0.0375)  # centres 7.5 cm apart: 5 mm between the cans, 7.5 mm to each wall
LEFT_PARK = (0.20, 0.30, TRAVEL_Z)  # where the left arm waits once the cans are in, clear of the right arm
# Let go on the cans, the open fingertips still reach 7 mm down the carton's sides at TRAVEL_Z, so the right arm
# lifts to CLEAR_Z (tips 1.3 cm above the carton) before it moves sideways; moving off at TRAVEL_Z dragged the
# carton off the cans (official stack, 2026-11-03).
CLEAR_Z = TOP + 0.17
RIGHT_PARK = (*EGGS_START, CLEAR_Z)  # where the right arm ends, so the camera sees the packed tote
RIGHT_WAIT_S = 22.0  # the right arm waits above the carton until the left arm has parked


def demo():
    left = []
    for name, place_x in zip(CAN_NAMES, CAN_PLACE_X, strict=True):
        x, y = CAN_START[name]
        left += pick_place_waypoints(
            (x, y, TOP + CAN_GRIP), (place_x, TOTE_Y, FLOOR + CAN_GRIP + DROP), travel_z=TRAVEL_Z,
        )
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park"))
    eggs_place_z = FLOOR + CAN_H + DROP + EGGS_GRIP
    right = [Waypoint(RIGHT_WAIT_S, (*EGGS_START, TRAVEL_Z), FINGER_OPEN, "wait for the cans")]
    right += pick_place_waypoints(
        (*EGGS_START, TOP + EGGS_GRIP), (TOTE_X, TOTE_Y, eggs_place_z), travel_z=TRAVEL_Z,
    )
    right.append(Waypoint(0.8, (TOTE_X, TOTE_Y, CLEAR_Z), FINGER_OPEN, "lift clear of the carton"))
    right.append(Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park"))
    return {"left": left, "right": right}
