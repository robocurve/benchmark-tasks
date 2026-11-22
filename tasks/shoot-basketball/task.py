"""On the station table, on the robot's right side: a mini orange basketball (5.4 cm diameter) resting near
the front of the table. Further back on the right side sits a miniature basketball goal consisting of a
white backboard and a red square hoop/basket (13 cm inner opening) mounted on a low stand. The ball is
loose; nothing else is on the table.

Task: Using the right arm, pick up the basketball and throw it into the basket goal so that it lands and
comes to rest inside the basket.
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # z of table upper surface

# World frame: x away from the arm bases, y to the robot's left, z up (m).
ORANGE = (0.90, 0.40, 0.08)
RED = (0.82, 0.16, 0.16)
WHITE = (0.92, 0.92, 0.92)
DARK_GRAY = (0.25, 0.25, 0.28)

# Mini basketball sized to fit comfortably inside the parallel-jaw gripper
BALL_R = 0.027  # 54 mm diameter
BALL_MASS = 0.06
BALL_START_XY = (0.18, -0.18)

BALL = Sphere(
    "basketball",
    pos=(*BALL_START_XY, TOP + BALL_R),
    radius=BALL_R,
    mass=BALL_MASS,
    color=ORANGE,
)

# Basket goal positioned further back (+x) on the right side (-y)
GOAL_XY = (0.48, -0.18)
BASKET_INNER = 0.130  # 13 cm x 13 cm inner opening
WALL_T = 0.010  # 1 cm wall thickness
WALL_H = 0.080  # 8 cm basket wall height
FLOOR_T = 0.010  # 1 cm basket floor thickness
STAND_H = 0.050  # 5 cm stand raising the basket off the table

BASKET_FLOOR_Z = TOP + STAND_H + FLOOR_T  # inner floor surface where the ball comes to rest
BASKET_RIM_Z = BASKET_FLOOR_Z + WALL_H
BACKBOARD_H = 0.160
BACKBOARD_W = 0.180
BACKBOARD_T = 0.012

_gx, _gy = GOAL_XY
_half = BASKET_INNER / 2 + WALL_T / 2
_outer = BASKET_INNER + 2 * WALL_T

GOAL_PARTS = [
    # Support pedestal
    Box(
        "goal_stand",
        pos=(_gx, _gy, TOP + STAND_H / 2),
        size=(0.08, 0.08, STAND_H),
        color=DARK_GRAY,
    ),
    # Basket floor (catches the ball)
    Box(
        "basket_floor",
        pos=(_gx, _gy, TOP + STAND_H + FLOOR_T / 2),
        size=(_outer, _outer, FLOOR_T),
        color=RED,
    ),
    # Four basket walls forming the hoop/catch bin
    Box(
        "basket_wall_front",
        pos=(_gx - _half, _gy, BASKET_FLOOR_Z + WALL_H / 2),
        size=(WALL_T, _outer, WALL_H),
        color=RED,
    ),
    Box(
        "basket_wall_back",
        pos=(_gx + _half, _gy, BASKET_FLOOR_Z + WALL_H / 2),
        size=(WALL_T, _outer, WALL_H),
        color=RED,
    ),
    Box(
        "basket_wall_left",
        pos=(_gx, _gy + _half, BASKET_FLOOR_Z + WALL_H / 2),
        size=(BASKET_INNER, WALL_T, WALL_H),
        color=RED,
    ),
    Box(
        "basket_wall_right",
        pos=(_gx, _gy - _half, BASKET_FLOOR_Z + WALL_H / 2),
        size=(BASKET_INNER, WALL_T, WALL_H),
        color=RED,
    ),
    # Backboard mounted behind the back wall
    Box(
        "backboard",
        pos=(_gx + _half + WALL_T / 2 + BACKBOARD_T / 2, _gy, BASKET_FLOOR_Z + BACKBOARD_H / 2),
        size=(BACKBOARD_T, BACKBOARD_W, BACKBOARD_H),
        color=WHITE,
    ),
]

OBJECTS = [BALL, *GOAL_PARTS]

ON_TOL = 0.008  # resting on basket floor within 8 mm
EPS = 1e-6


def _in_basket(st) -> bool:
    """Return True if the basketball is inside the basket goal (even if still rolling)."""
    bx, by, bz = float(st.pos[0]), float(st.pos[1]), float(st.pos[2])
    half_in = BASKET_INNER / 2 - BALL_R * 0.5
    in_xy = abs(bx - _gx) <= half_in + EPS and abs(by - _gy) <= half_in + EPS
    in_z = (BASKET_FLOOR_Z - ON_TOL) <= (bz - BALL_R) <= (BASKET_RIM_Z - BALL_R)
    return in_xy and in_z



def success(objects) -> bool:
    """Basketball is at rest inside the basket goal."""
    return _in_basket(objects["basketball"])


def distance(objects) -> float:
    """3D distance from the basketball center to the target resting center inside the basket."""
    ball_pos = np.asarray(objects["basketball"].pos, float)
    target_pos = np.array([_gx, _gy, BASKET_FLOOR_Z + BALL_R], float)
    return float(np.linalg.norm(ball_pos - target_pos))


def score(objects) -> float:
    """1.0: basketball is at rest inside the basket goal.
    0.0: basketball did not land inside the basket.
    """
    return 1.0 if _in_basket(objects["basketball"]) else 0.0


# ---- scripted demo ------------------------------------------------------------------------------------------
# Single-arm (right) pick-and-throw: grasp the basketball, pull back into a wind-up pose, swing forward
# and upward toward the basket goal, and release mid-swing so the ball arcs into the basket.
TIP = 0.013  # finger tips below the TCP
GRASP_Z = TOP + BALL_R + TIP
APPROACH_Z = TOP + BALL_R * 2 + 0.10
RIGHT_YAW = 0.0

WINDUP_POS = (0.12, _gy, TOP + 0.14)
RELEASE_POS = (0.28, _gy, BASKET_RIM_Z + 0.08)
FOLLOW_THROUGH_POS = (0.35, _gy, BASKET_RIM_Z + 0.14)


def demo():
    sx, sy = BALL_START_XY

    def wp(t, x, y, z, finger, label):
        return Waypoint(t, (float(x), float(y), float(z)), finger, label, RIGHT_YAW)

    return {
        "right": [
            wp(1.2, sx, sy, APPROACH_Z, FINGER_OPEN, "move above basketball"),
            wp(1.0, sx, sy, GRASP_Z, FINGER_OPEN, "lower onto basketball"),
            wp(0.5, sx, sy, GRASP_Z, FINGER_CLOSED, "grasp basketball"),
            wp(0.9, *WINDUP_POS, FINGER_CLOSED, "lift and wind up"),
            # Fast forward/upward swing; open gripper at the release point
            wp(0.28, *RELEASE_POS, FINGER_CLOSED, "swing toward basket"),
            wp(0.08, *RELEASE_POS, FINGER_OPEN, "release basketball"),
            wp(0.45, *FOLLOW_THROUGH_POS, FINGER_OPEN, "follow through"),
        ]
    }