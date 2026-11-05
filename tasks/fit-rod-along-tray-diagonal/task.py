"""On the station table: an open-topped grey tray between the arms, fixed to the table, its inside 200 mm front
to back and 150 mm across, with walls 40 mm tall above its floor. An orange square rod, 220 mm long and 20 mm
thick, lies on the table to the left of the tray, its long side front to back.
"""

import math

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
TRAY_X, TRAY_Y = 0.30, 0.0  # centre of the tray
INNER_X, INNER_Y = 0.200, 0.150  # inside length (front to back) and width
WALL_T, WALL_H = 0.010, 0.040  # wall thickness; wall height above the tray floor
FLOOR_T = 0.004  # floor plate under the whole tray
FLOOR_Z = TOP + FLOOR_T  # top of the tray floor
OUTER_X, OUTER_Y = INNER_X + 2 * WALL_T, INNER_Y + 2 * WALL_T
X0, X1 = TRAY_X - INNER_X / 2, TRAY_X + INNER_X / 2  # inside faces of the front and back walls
Y0, Y1 = TRAY_Y - INNER_Y / 2, TRAY_Y + INNER_Y / 2  # inside faces of the right and left walls

ROD = (0.220, 0.020, 0.020)  # square tube, 220 mm long, 20 x 20 mm section
ROD_MASS = 0.066  # 1.5 mm aluminium wall
ROD_XY = (0.30, 0.19)  # lying along x beside the tray, 95 mm clear of its left wall

GREY, FLOOR_GREY, ORANGE = (0.45, 0.47, 0.50), (0.62, 0.64, 0.66), (0.95, 0.45, 0.10)
WALL_Z = FLOOR_Z + WALL_H / 2

TRAY = [
    Box("tray_floor", pos=(TRAY_X, TRAY_Y, TOP + FLOOR_T / 2), size=(OUTER_X, OUTER_Y, FLOOR_T), color=FLOOR_GREY),
    Box("tray_wall_front", pos=(X0 - WALL_T / 2, TRAY_Y, WALL_Z), size=(WALL_T, OUTER_Y, WALL_H), color=GREY),
    Box("tray_wall_back", pos=(X1 + WALL_T / 2, TRAY_Y, WALL_Z), size=(WALL_T, OUTER_Y, WALL_H), color=GREY),
    Box("tray_wall_right", pos=(TRAY_X, Y0 - WALL_T / 2, WALL_Z), size=(INNER_X, WALL_T, WALL_H), color=GREY),
    Box("tray_wall_left", pos=(TRAY_X, Y1 + WALL_T / 2, WALL_Z), size=(INNER_X, WALL_T, WALL_H), color=GREY),
]
ROD_BOX = Box("rod", pos=(*ROD_XY, TOP + ROD[2] / 2), size=ROD, mass=ROD_MASS, color=ORANGE)

OBJECTS = [*TRAY, ROD_BOX]

INSIDE_TOL = 0.001  # the rod's outline may reach 1 mm past an inside wall face (contact)
ON_FLOOR = 0.001  # lowest point at most 1 mm above the floor
FLAT_TOP = 0.025  # lying flat: highest point at most 25 mm above the floor (the rod is 20 mm thick)
PROP_TOP = 0.100  # propped: highest point at most 100 mm above the floor (not stood on end)


def _wholly_inside(rod) -> bool:
    """The rod's outline seen from above is within the inside faces of the four walls."""
    return bool(
        rod.low[0] >= X0 - INSIDE_TOL and rod.high[0] <= X1 + INSIDE_TOL
        and rod.low[1] >= Y0 - INSIDE_TOL and rod.high[1] <= Y1 + INSIDE_TOL
    )


def _centre_over_floor(rod) -> bool:
    return bool(X0 < rod.pos[0] < X1 and Y0 < rod.pos[1] < Y1)


def success(objects):
    """Wholly inside the tray, lying flat on its floor, at rest."""
    rod = objects["rod"]
    return bool(
        _wholly_inside(rod)
        and rod.low[2] <= FLOOR_Z + ON_FLOOR
        and rod.high[2] <= FLOOR_Z + FLAT_TOP
        and rod.resting()
    )


def distance(objects):
    rod = objects["rod"]
    return float(np.linalg.norm(rod.pos - np.array((TRAY_X, TRAY_Y, FLOOR_Z + ROD[2] / 2))))


def score(objects):
    """1.0 on success. 0.3 if the rod is propped in the tray: its lowest point on the tray floor, its midpoint over
    the floor, part of it over a wall (it rests on a wall top), its highest point more than 25 mm and at most
    100 mm above the floor, at rest. 0 otherwise (on the table, across the wall tops, stood on end, outside)."""
    if success(objects):
        return 1.0
    rod = objects["rod"]
    if (
        rod.resting()
        and rod.low[2] <= FLOOR_Z + ON_FLOOR
        and _centre_over_floor(rod)
        and not _wholly_inside(rod)
        and FLOOR_Z + FLAT_TOP < rod.high[2] <= FLOOR_Z + PROP_TOP
    ):
        return 0.3
    return 0.0


# Demo: the left arm grips the rod at its middle from above, lifts it, turns it 34 degrees while carrying it over
# the tray, lowers it to 7 mm above the floor and lets go.
PLACE_YAW = math.radians(34.0)
TIP_PAST_TCP = 0.013
TIP_ABOVE_BOTTOM = 0.004  # finger tips 4 mm above the rod's underside
GRIP_Z = TOP + TIP_ABOVE_BOTTOM + TIP_PAST_TCP
TRAVEL_Z = TOP + 0.14
DROP = 0.007  # rod underside above the floor at release: it drops onto the floor, so nothing holds it
# The left arm's measured tracking error while carrying a light object near x 0.30 m: about 2.4 mm to the robot's
# left and 4 mm low. The demo aims off by that much.
SAG_Y, SAG_Z = 0.0024, 0.004
PLACE_Z = FLOOR_Z + DROP + TIP_ABOVE_BOTTOM + TIP_PAST_TCP + SAG_Z


def demo():
    px, py = ROD_XY
    qx, qy = TRAY_X, TRAY_Y - SAG_Y
    a = PLACE_YAW
    w = [
        Waypoint(1.5, (px, py, TRAVEL_Z), FINGER_OPEN, "over the rod"),
        Waypoint(1.5, (px, py, GRIP_Z), FINGER_OPEN, "descend"),
        Waypoint(0.8, (px, py, GRIP_Z), FINGER_CLOSED, "grip"),
        Waypoint(1.5, (px, py, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(2.5, (qx, qy, TRAVEL_Z), FINGER_CLOSED, "carry and turn", a),
        Waypoint(1.5, (qx, qy, PLACE_Z + 0.03), FINGER_CLOSED, "lower", a),
        Waypoint(1.5, (qx, qy, PLACE_Z), FINGER_CLOSED, "set down", a),
        Waypoint(0.6, (qx, qy, PLACE_Z), FINGER_OPEN, "release", a),
        Waypoint(1.2, (qx, qy, TRAVEL_Z), FINGER_OPEN, "up", a),
    ]
    return {"left": w}
