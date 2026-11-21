"""Guide the panel into the four cups, then rotate the physical latch."""

import numpy as np

from . import config as C
from .motion import Program, rotation

FRONT = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
DOWN = rotation("Y", np.radians(38.)) @ FRONT


def left_pose(x, z=.095):
    return np.array([x, .185, C.FLOOR_Z + z]), DOWN, .06


def handle_pose(deg, retraction=0., lift=0., grip=.07):
    rad = np.radians(deg)
    tip = np.array([-.029 - retraction, -C.LEVER_LENGTH*np.cos(rad), -C.LEVER_LENGTH*np.sin(rad)+lift])
    p = C.LEVER_PIVOT + tip + [C.GATE_X+.006, .006, C.FLOOR_Z]
    return p, DOWN, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    program = Program()
    program.add(1.5, "Approach the loose lower gate rail",
                left=left_pose(C.GATE_START_X-.078, .155))
    program.add(1.6, "Guide the gate toward all four wall cups",
                left=left_pose(C.GATE_X+.025))
    program.add(1.6, "Steady the panel while approaching the locking handle",
                right=handle_pose(0., .065, .045, .76))
    if mode != "loose":
        program.add(1.8, "Line up the open fingertips around the grip",
                    right=handle_pose(0., 0., 0., .76))
        program.add(.9, "Grasp the locking handle", right=handle_pose(0.))
        for angle in np.linspace(0., C.LEVER_STOP+5., 19)[1:]:
            program.add(.32, "Rotate the pressure handle past center", right=handle_pose(angle))
        program.add(.45, "Release the closed handle", right=handle_pose(104., 0., 0., .79))
    program.add(1.6, "Withdraw both grippers from the locked gate",
                left=left_pose(C.GATE_X-.130, .150), right=handle_pose(100., .135, .02, .79))
    return program.moves
