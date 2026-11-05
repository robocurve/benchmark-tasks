"""On the station table, between the arms: a grey shelf lane, fixed, running away from the arms (inside 8 cm side to
side and 23 cm front to back, from 13 to 36 cm in front of the arm bases), with 4 cm side walls and back wall on a
5 mm floor and a 1.5 cm stop lip across its near end. In the lane stand two bins, each 10 cm long, 7 cm wide and
8 cm tall: the empty red bin at the front against the stop (80 g), and the full blue bin behind it with a 1 cm gap
(500 g). On the robot's left stands the fixed orange reorder shelf, a block 15 x 15 cm and 7 cm tall, its centre
25 cm in front of the arm bases and 26 cm left of centre. Done, the red bin stands on the orange shelf and the blue
bin stands at the front of the lane against the stop; the red bin is no longer in the lane.
"""

import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The lane: a 5 mm floor plate, two 1 cm side walls and a back wall 4 cm tall, and a 1.5 cm stop lip at the near end.
LANE_Y = 0.0
X_STOP, X_BACK = 0.13, 0.36  # the stop lip's inner face and the back wall's inner face
LANE_W = 0.08
IN_X = (X_STOP, X_BACK)
IN_Y = (LANE_Y - LANE_W / 2, LANE_Y + LANE_W / 2)
WALL_T, WALL_H, LIP_H, FLOOR_T = 0.01, 0.04, 0.015, 0.005
FLOOR = TOP + FLOOR_T
LANE_GREY = (0.30, 0.31, 0.34)
_LEN = X_BACK - X_STOP + 2 * WALL_T
_MID_X = (X_STOP + X_BACK) / 2
LANE = [
    Box("lane_floor", pos=(_MID_X, LANE_Y, TOP + FLOOR_T / 2), size=(_LEN, LANE_W + 2 * WALL_T, FLOOR_T),
        color=LANE_GREY),
    Box("lane_wall_left", pos=(_MID_X, IN_Y[1] + WALL_T / 2, FLOOR + WALL_H / 2), size=(_LEN, WALL_T, WALL_H),
        color=LANE_GREY),
    Box("lane_wall_right", pos=(_MID_X, IN_Y[0] - WALL_T / 2, FLOOR + WALL_H / 2), size=(_LEN, WALL_T, WALL_H),
        color=LANE_GREY),
    Box("lane_back", pos=(X_BACK + WALL_T / 2, LANE_Y, FLOOR + WALL_H / 2), size=(WALL_T, LANE_W, WALL_H),
        color=LANE_GREY),
    Box("lane_stop", pos=(X_STOP - WALL_T / 2, LANE_Y, FLOOR + LIP_H / 2), size=(WALL_T, LANE_W, LIP_H),
        color=LANE_GREY),
]

# The orange reorder shelf: a fixed block on the robot's left, only the left arm reaches it.
SHELF_X, SHELF_Y = 0.25, 0.26
SHELF_SIZE = (0.15, 0.15, 0.07)
SHELF_TOP = TOP + SHELF_SIZE[2]
SHELF_X_RANGE = (SHELF_X - SHELF_SIZE[0] / 2, SHELF_X + SHELF_SIZE[0] / 2)
SHELF_Y_RANGE = (SHELF_Y - SHELF_SIZE[1] / 2, SHELF_Y + SHELF_SIZE[1] / 2)
SHELF_ORANGE = (0.95, 0.45, 0.05)
SHELF = Box("reorder_shelf", pos=(SHELF_X, SHELF_Y, TOP + SHELF_SIZE[2] / 2), size=SHELF_SIZE, color=SHELF_ORANGE)

# The bins: long side front to back, 7 cm across the lane (5 mm to each wall).
BIN_L, BIN_W, BIN_H = 0.10, 0.07, 0.08
BIN_SIZE = (BIN_L, BIN_W, BIN_H)
RED_KG, BLUE_KG = 0.08, 0.50
BIN_RED = (0.80, 0.12, 0.10)
BIN_BLUE = (0.15, 0.35, 0.85)
BIN_GAP = 0.010
RED_START_X = X_STOP + BIN_L / 2 + 0.001  # 1 mm off the stop lip
BLUE_START_X = RED_START_X + BIN_L + BIN_GAP
RED = Box("red_bin", pos=(RED_START_X, LANE_Y, FLOOR + BIN_H / 2 + 0.001), size=BIN_SIZE, mass=RED_KG,
          color=BIN_RED)
BLUE = Box("blue_bin", pos=(BLUE_START_X, LANE_Y, FLOOR + BIN_H / 2 + 0.001), size=BIN_SIZE, mass=BLUE_KG,
           color=BIN_BLUE)  # each starts 1 mm above the lane floor, and settles

OBJECTS = [*LANE, SHELF, RED, BLUE]
SUCCESS_HOLD_S = 2.5  # both bins placed and still for 2.5 s

RED_CREDIT, BLUE_CREDIT = 0.5, 0.5
UPRIGHT_DEG = 10.0  # a placed bin is tilted at most 10 degrees
SEATED_TOL = 0.002  # a placed bin's bottom within 2 mm of the surface it stands on (set down, not held above it)
FRONT_TOL = 0.010  # the blue bin's front face at most 10 mm behind the stop lip's inner face
LIP_TOL = 0.001  # no part of the blue bin more than 1 mm in front of the stop lip's inner face (not up on the lip)
LANE_TOL = 0.005  # the blue bin's outline may reach 5 mm past the lane's inside (wall and part tolerances)
FLOOR_DROP = 0.01  # a bin whose bottom is more than 1 cm below the table top has fallen off the table
# The lane's whole outline seen from above, walls and stop lip included.
LANE_OUT_X = (X_STOP - WALL_T, X_BACK + WALL_T)
LANE_OUT_Y = (IN_Y[0] - WALL_T, IN_Y[1] + WALL_T)


def _tilt_deg(state) -> float:
    """Angle between the object's up axis and the vertical."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def _centre_in(state, xr, yr) -> bool:
    """The object's centre over the rectangle xr x yr (any height)."""
    x, y = float(state.pos[0]), float(state.pos[1])
    return bool(xr[0] <= x <= xr[1] and yr[0] <= y <= yr[1])


def _outline_in(state, xr, yr, tol) -> bool:
    """The object's whole outline seen from above inside the rectangle, give or take tol."""
    lo, hi = state.low, state.high
    return bool(xr[0] - tol <= float(lo[0]) and float(hi[0]) <= xr[1] + tol
                and yr[0] - tol <= float(lo[1]) and float(hi[1]) <= yr[1] + tol)


def _outline_overlaps(state, xr, yr) -> bool:
    """Any part of the object's outline seen from above over the rectangle xr x yr (any height)."""
    lo, hi = state.low, state.high
    return bool(float(lo[0]) < xr[1] and float(hi[0]) > xr[0] and float(lo[1]) < yr[1] and float(hi[1]) > yr[0])


def _red_on_shelf(objects) -> bool:
    """The red bin standing on the orange shelf: all of its base over the shelf top, bottom on it, upright."""
    red = objects["red_bin"]
    return bool(
        _outline_in(red, SHELF_X_RANGE, SHELF_Y_RANGE, 0.0)
        and abs(float(red.low[2]) - SHELF_TOP) <= SEATED_TOL
        and _tilt_deg(red) <= UPRIGHT_DEG
    )


def _blue_at_front(objects) -> bool:
    """The blue bin standing in the lane against the stop: inside the lane, bottom on the lane floor, upright, and
    its front face at most FRONT_TOL behind the stop and no part of it over the stop lip."""
    blue = objects["blue_bin"]
    return bool(
        _outline_in(blue, IN_X, IN_Y, LANE_TOL)
        and abs(float(blue.low[2]) - FLOOR) <= SEATED_TOL
        and _tilt_deg(blue) <= UPRIGHT_DEG
        and X_STOP - LIP_TOL <= float(blue.low[0]) <= X_STOP + FRONT_TOL
    )


def _signal_broken(objects) -> bool:
    """The replenishment signal is broken: any part of the empty red bin is still (or again) over the lane, walls
    and stop lip included, the full blue bin is on the reorder shelf, or either bin has fallen off the table."""
    red, blue = objects["red_bin"], objects["blue_bin"]
    return bool(
        _outline_overlaps(red, LANE_OUT_X, LANE_OUT_Y)
        or _centre_in(blue, SHELF_X_RANGE, SHELF_Y_RANGE)
        or min(float(red.low[2]), float(blue.low[2])) < TOP - FLOOR_DROP
    )


def score(objects):
    """Partial credit (success scores 1.0): 0.5 for the red bin standing on the orange shelf; 0.5 for the blue bin
    standing at the front of the lane against the stop. 0 overall if any part of the red bin is over the lane (walls
    and stop lip included), the blue bin's centre is over the shelf, or either bin has fallen off the table."""
    if _signal_broken(objects):
        return 0.0
    return round(RED_CREDIT * _red_on_shelf(objects) + BLUE_CREDIT * _blue_at_front(objects), 6)


def success(objects):
    """The red bin on the shelf and the blue bin at the stop, both still; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and objects["red_bin"].resting() and objects["blue_bin"].resting()


# The demo: the left arm lifts the red bin out of the lane by its sides and sets it on the orange shelf, then parks
# to its left. The right arm waits off to the right until then, grips the blue bin by its sides above the walls,
# lifts it 2.5 cm, carries it along the lane to the stop and sets it down. Fingers close along world y throughout.
GRIP_Z = 0.062  # TCP above a bin's bottom: the pads grip its top 31 mm, the fingertips 9 mm above the lane walls
DROP = 0.006  # each bin is let go this far above where it lands
TRAVEL_Z = TOP + 0.16  # the carried red bin's bottom clears the lane walls by 5 cm and the shelf by 2.8 cm
CLEAR_Z = TOP + 0.175  # fingertips 1.2 cm above the red bin's top on the shelf
LEFT_PARK = (0.15, 0.42, CLEAR_Z)  # left of the shelf, so the camera sees the shelf and the lane
LANE_LIFT = 0.025  # the blue bin is carried with its bottom 2.5 cm above the lane floor, over the 1.5 cm stop lip
BLUE_PLACE_X = X_STOP + BIN_L / 2 + 0.003  # it lands 3 mm behind the stop
HOVER_Z = TOP + 0.12  # the right arm's approach height: fingertips 2.2 cm above the blue bin's top
RIGHT_WAIT = (0.30, -0.18, TOP + 0.15)  # off the lane, away from the left arm
RIGHT_WAIT_S = 12.0  # the left arm's whole plan takes 11.9 s
RIGHT_PARK = (0.20, -0.30, TOP + 0.15)


def demo():
    left = pick_place_waypoints(
        (RED_START_X, LANE_Y, FLOOR + GRIP_Z), (SHELF_X, SHELF_Y, SHELF_TOP + DROP + GRIP_Z), travel_z=TRAVEL_Z,
    )
    left.append(Waypoint(0.8, (SHELF_X, SHELF_Y, CLEAR_Z), FINGER_OPEN, "lift clear of the red bin"))
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park"))
    grip = (BLUE_START_X, LANE_Y, FLOOR + GRIP_Z)
    carry_z = FLOOR + GRIP_Z + LANE_LIFT
    place = (BLUE_PLACE_X, LANE_Y, FLOOR + GRIP_Z + DROP)
    right = [
        Waypoint(RIGHT_WAIT_S, RIGHT_WAIT, FINGER_OPEN, "wait for the lane to clear"),
        Waypoint(1.5, (BLUE_START_X, LANE_Y, HOVER_Z), FINGER_OPEN, "to the blue bin"),
        Waypoint(1.5, grip, FINGER_OPEN, "descend"),
        Waypoint(0.8, grip, FINGER_CLOSED, "close"),
        Waypoint(1.0, (BLUE_START_X, LANE_Y, carry_z), FINGER_CLOSED, "lift off the lane floor"),
        Waypoint(2.0, (BLUE_PLACE_X, LANE_Y, carry_z), FINGER_CLOSED, "carry to the stop"),
        Waypoint(1.0, place, FINGER_CLOSED, "lower"),
        Waypoint(0.6, place, FINGER_OPEN, "release"),
        Waypoint(1.0, (BLUE_PLACE_X, LANE_Y, HOVER_Z), FINGER_OPEN, "lift clear of the blue bin"),
        Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park"),
    ]
    return {"left": left, "right": right}
