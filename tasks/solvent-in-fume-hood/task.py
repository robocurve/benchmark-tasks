"""A fume-hood mock, open-topped, facing the arms: a 2 cm tall sill across the sash opening (x 12-13 cm),
side walls at y +-17 cm running back to a back wall at x 41.5 cm, 15 cm tall, all fixed and grey. An amber
250 mL solvent bottle (5 cm across, 10 cm tall) stands on the table outside the hood, to its right.
"""

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Put the solvent bottle in the fume hood."  # read by our run script, not the framework

GREY = (0.75, 0.75, 0.78)
SILL_X0, SILL_X1 = 0.12, 0.13  # sash opening's sill: the 15 cm rule is measured from its rear edge
BACK_X = 0.415  # inner face of the back wall
SIDE_Y = 0.16  # inner faces of the side walls (walls 1 cm thick, centred at +-0.165)
SAFE_DEPTH = 0.15

OBJECTS = [
    Box("hood_sill", pos=(0.125, 0.0, TOP + 0.01), size=(0.01, 0.34, 0.02), color=GREY),
    Box("hood_left", pos=(0.27, 0.165, TOP + 0.075), size=(0.30, 0.01, 0.15), color=GREY),
    Box("hood_right", pos=(0.27, -0.165, TOP + 0.075), size=(0.30, 0.01, 0.15), color=GREY),
    Box("hood_back", pos=(0.42, 0.0, TOP + 0.075), size=(0.01, 0.34, 0.15), color=GREY),
    Cylinder("solvent_bottle", pos=(0.16, -0.24, TOP + 0.05), radius=0.025, height=0.10, mass=0.25,
             color=(0.55, 0.30, 0.05)),
]


def _upright(s, max_tilt_deg=20.0):
    qx, qy = s.quat_xyzw[0], s.quat_xyzw[1]
    return 1 - 2 * (qx * qx + qy * qy) > np.cos(np.radians(max_tilt_deg))


def _in_hood(s):
    """Standing on the table between the walls, wholly behind the sill."""
    return bool(
        s.low[2] < TOP + 0.01  # on the table, not on the sill or a wall top
        and s.low[0] > SILL_X1 and s.high[0] < BACK_X
        and s.low[1] > -SIDE_Y and s.high[1] < SIDE_Y
    )


def _ok(objects):
    s = objects["solvent_bottle"]
    return _upright(s) and s.resting() and _in_hood(s)


def success(objects):
    """Nearest point of the bottle at least 15 cm behind the sill's rear edge."""
    s = objects["solvent_bottle"]
    return bool(_ok(objects) and s.low[0] >= SILL_X1 + SAFE_DEPTH)


def score(objects):
    """0.25 inside the hood but closer than 15 cm; 0 anywhere else (tipped, moving, outside, on a wall)."""
    return 0.25 if _ok(objects) else 0.0


def demo():
    """Right arm: grip the bottle 3 cm below its top, lift it 4 cm clear of the table, bring it in front of
    the sash, slide it through the opening to 20 cm behind the sill, set it down, let go, rise."""
    grip_z = TOP + 0.10 - 0.03 + 0.013  # pad tips 3 cm below the cap
    lift_z = grip_z + 0.045
    up_z = TOP + 0.24
    bx, by = 0.16, -0.24
    gx, gy = 0.33, -0.07
    return {
        "right": [
            Waypoint(1.5, (bx, by, up_z), FINGER_OPEN, "above bottle"),
            Waypoint(1.5, (bx, by, grip_z), FINGER_OPEN, "descend"),
            Waypoint(0.8, (bx, by, grip_z), FINGER_CLOSED, "close"),
            Waypoint(1.0, (bx, by, lift_z), FINGER_CLOSED, "lift"),
            Waypoint(1.5, (0.08, by, lift_z), FINGER_CLOSED, "back out"),
            Waypoint(1.5, (0.08, gy, lift_z), FINGER_CLOSED, "in front of sash"),
            Waypoint(2.0, (gx, gy, lift_z), FINGER_CLOSED, "through the sash"),
            Waypoint(1.0, (gx, gy, grip_z + 0.005), FINGER_CLOSED, "lower"),
            Waypoint(0.6, (gx, gy, grip_z + 0.005), FINGER_OPEN, "release"),
            Waypoint(1.5, (gx, gy, up_z), FINGER_OPEN, "rise"),
        ]
    }
