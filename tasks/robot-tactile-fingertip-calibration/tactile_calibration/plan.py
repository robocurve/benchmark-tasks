"""Reference motions with separate housing and probe grasps."""

import math

import numpy as np

from . import config as C
from .motion import Program, rotation


def grasp_rotation(lateral):
    forward = np.array([0.64, lateral, -0.18])
    forward /= np.linalg.norm(forward)
    close = np.array([forward[1], -forward[0], 0.0])
    close /= np.linalg.norm(close)
    return np.column_stack((close, np.cross(forward, close), forward))


LEFT = grasp_rotation(-0.77)
RIGHT = grasp_rotation(0.70)
SENSOR_R = rotation("Z", math.pi / 4)


def grasp(root, height, matrix, grip, back=0):
    return root + [0, 0, height] - matrix[:, 2] * back, matrix, grip


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves

    left = lambda root, grip=0.04, back=0: grasp(root, C.SENSOR_GRIP, LEFT, grip, back)
    right = lambda root, grip=0.04, back=0: grasp(root, C.TOOL_GRIP, RIGHT, grip, back)
    p.add(2.4, "approach staged tools", left=left(C.SENSOR_START, 1, 0.070), right=right(C.TOOL_START, 0.95, 0.075))
    p.add(1.3, "straddle both grips", left=left(C.SENSOR_START, 1), right=right(C.TOOL_START, 0.95))
    p.add(0.95, "close on housing and probe", left=left(C.SENSOR_START), right=right(C.TOOL_START))
    p.add(1.25, "raise both tools", left=left(C.SENSOR_START+[0,0,0.040]), right=right(C.TOOL_START+[0,0,0.062]))
    p.add(1.55, "steady housing for calibration", left=left(C.SENSOR_WORK), right=right(C.TOOL_START+[0,0,0.123]))
    for i, region in enumerate(C.REGIONS):
        center = C.SENSOR_WORK + SENSOR_R @ np.r_[region, C.PAD_Z + C.BALL_R - C.BALL_Z]
        high = center + [0, 0, 0.017]
        low = center + [0, 0, 0.008 if mode == "hover" else -0.0040]
        p.add(1.35, f"aim {i}", right=right(high))
        p.add(1.25, f"indent {i}", right=right(low))
        p.add(2.30, f"capture {i}", right=right(low))
        p.add(0.9, f"lift {i}", right=right(high))
    p.add(2.4, "draw probe away", right=right(C.TOOL_START+[0,0,0.1]))
    return p.moves
