"""On the station table, near the front-right: a 5.8 cm diameter dark blue bowling ball. Further
back across the table, beyond the robot arm's reach, stand three white bowling pins arranged in a
two-row triangular rack (1-2 formation). Nothing else is on the table.

Task: Using the right arm, pick up the bowling ball and roll it forward down the table to knock
down at least a majority of the pins.
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # z of table upper surface

# World frame: x away from the arm bases, y to the robot's left, z up (m).
BALL_BLUE = (0.14, 0.28, 0.62)
PIN_WHITE = (0.94, 0.94, 0.94)
PIN_STRIPE = (0.82, 0.16, 0.16)

# 5.8 cm diameter bowling ball positioned in the right arm's (-y) lane
BALL_R = 0.029  # 58 mm diameter (fits cleanly inside open parallel-jaw gripper for a firm grasp)
BALL_MASS = 0.14  # 140 g: light enough not to slip in the jaws, 14x heavier than each 10 g pin
BALL_START_XY = (0.24, -0.10)

BALL = Sphere(
    "bowling_ball",
    pos=(*BALL_START_XY, TOP + BALL_R),
    radius=BALL_R,
    mass=BALL_MASS,
    color=BALL_BLUE,
)

# Three tall, ultra-light pins arranged in a 1-2 triangle far down the table (beyond arm reach)
PIN_W = 0.020  # 2.0 cm square footprint (slender base so pins topple easily on impact)
PIN_H = 0.140  # 14.0 cm tall (high center of mass for easy tipping and chain reactions)
PIN_MASS = 0.010  # 10 g per pin
HEAD_PIN_XY = (0.74, -0.104)  # aligned with the ball's roll path
ROW_DX = 0.045  # spacing between rows along +x
COL_DY = 0.044  # lateral spacing between pins in the second row

_hx, _hy = HEAD_PIN_XY
PIN_LAYOUT = [
    ("pin_1", (_hx, _hy)),
    ("pin_2", (_hx + ROW_DX, _hy - COL_DY / 2)),
    ("pin_3", (_hx + ROW_DX, _hy + COL_DY / 2)),
]

PINS = [
    Box(
        name,
        pos=(px, py, TOP + PIN_H / 2),
        size=(PIN_W, PIN_W, PIN_H),
        mass=PIN_MASS,
        color=PIN_WHITE,
    )
    for name, (px, py) in PIN_LAYOUT
]

OBJECTS = [BALL, *PINS]

# A pin standing upright has center z = TOP + PIN_H / 2; lying on its side has z ≈ TOP + PIN_W / 2.
TOPPLED_Z_MAX = TOP + PIN_H * 0.35
MIN_ROLL_X = BALL_START_XY[0] + 0.15  # ball must actually be rolled forward down the lane
MIN_KNOCKED = len(PIN_LAYOUT) // 2 + 1  # majority of targets (2 of 3)


def _ball_rolled(objects) -> bool:
    """Return True if the bowling ball was rolled forward away from the start area."""
    return float(objects["bowling_ball"].pos[0]) >= MIN_ROLL_X


def _is_knocked_down(st) -> bool:
    """Return True if the pin has tipped over onto the table (or been knocked off the back edge)."""
    return float(st.pos[2]) <= TOPPLED_Z_MAX


def _knocked_count(objects) -> int:
    if not _ball_rolled(objects):
        return 0
    return sum(1 for name, _ in PIN_LAYOUT if _is_knocked_down(objects[name]))


def success(objects) -> bool:
    """At least a majority of the pins (2 of 3) were knocked down by rolling the bowling ball."""
    return _knocked_count(objects) >= MIN_KNOCKED


def distance(objects) -> float:
    """3D distance from the bowling ball center to the head pin's initial position."""
    ball_pos = np.asarray(objects["bowling_ball"].pos, float)
    target_pos = np.array([_hx, _hy, TOP + BALL_R], float)
    return float(np.linalg.norm(ball_pos - target_pos))


def score(objects) -> float:
    """Normalized score reaching 1.0 once at least a majority of pins (2 of 3) are down."""
    return min(1.0, float(_knocked_count(objects)) / MIN_KNOCKED)


# ---- scripted demo (right arm only, straight-line grasp-and-roll) ------------------------------------------
TIP = 0.013  # finger tips below the TCP
GRASP_Z = TOP + BALL_R + TIP
ROLL_Z = GRASP_Z + 0.002  # 2 mm above table so the ball stays in contact with the table
Y_TRIM = -0.088  # right (-y) aim offset dead-center through the 3-pin rack


def demo():
    sx, sy = BALL_START_XY

    def wp(t, x, y, z, finger, label):
        return Waypoint(t, (float(x), float(y), float(z)), finger, label)

    right = [
        wp(0.80, sx, sy, GRASP_Z + 0.08, FINGER_OPEN, "move above bowling ball"),
        wp(0.60, sx, sy, GRASP_Z, FINGER_OPEN, "lower onto bowling ball"),
        wp(0.35, sx, sy, GRASP_Z, FINGER_CLOSED, "grasp bowling ball firmly"),
        wp(0.55, sx - 0.14, sy, ROLL_Z, FINGER_CLOSED, "pull back for long rolling runway"),
        wp(0.25, sx - 0.14, sy, ROLL_Z, FINGER_CLOSED, "stabilize before forward roll"),
        # Keep z completely flat at ROLL_Z through the entire forward push so the arm never lifts the ball
        wp(0.08, sx - 0.04, sy + Y_TRIM * 0.35, ROLL_Z, FINGER_OPEN, "accelerate forward and open jaws early"),
        wp(0.07, sx + 0.05, sy + Y_TRIM * 0.70, ROLL_Z, FINGER_OPEN, "push ball horizontally at peak speed"),
        wp(0.07, sx + 0.15, sy + Y_TRIM, ROLL_Z, FINGER_OPEN, "complete flat horizontal follow-through"),
        wp(0.15, sx + 0.15, sy + Y_TRIM, ROLL_Z, FINGER_OPEN, "pause flat after ball exits jaws"),
        wp(0.25, sx + 0.15, sy + Y_TRIM, ROLL_Z + 0.09, FINGER_OPEN, "lift clear after release"),
    ]
    return {"right": right}