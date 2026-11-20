"""Grasp the ends, peel one sheet, then roll the swatch in a loaded pass."""

import numpy as np

from . import config as C
from .motion import Program, rotation

TOP = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])
HOLD = rotation("Z", np.deg2rad(-90.)) @ TOP
ROLL = rotation("Y", np.deg2rad(-15.)) @ HOLD
PINCH = rotation("Y", np.deg2rad(-20.)) @ rotation("Z", np.deg2rad(90.)) @ TOP
SEAM_APPROACH = 3.75
SEAM_SAMPLE_END = 5.15


def waypoints(mode="demo", seam=None):
    p = Program()
    if mode == "hold":
        return p.moves
    handle = C.ROLLER + [0., C.HANDLE_Y, .002]
    edge = C.TAB if seam is None else seam
    tab = edge + [0., -.003, .002]
    brace = C.CLOTH + [.008, -.105, .005]
    rolling_start = np.array([.331, C.CLOTH[1], C.TABLE + C.CLOTH_SIZE[2] + C.RADIUS - .0065])
    rolling_end = rolling_start + [.173, 0., 0.]
    p.add(2.0, "approach the lint roller",
          left=(handle + [0., 0., .052], HOLD, .65))
    p.add(1.1, "surround the ordinary handle",
          left=(handle, HOLD, .65))
    p.add(.65, "grasp the handle", left=(handle, HOLD, .02))
    p.add(1.5, "counterhold the roll and approach the factory seam",
          left=(handle, HOLD, .02), right=(tab + [0., 0., .035], PINCH, .36))
    if mode != "dirty-roll":
        p.add(1.25, "approach the outer sheet edge", right=(tab, PINCH, .30))
        p.add(.4, "pinch the sheet edge", right=(tab, PINCH, 0.))
        p.add(1.2, "lift the initial adhesive seam",
              right=(tab + [-.025, -.010, .045], PINCH, 0.))
        pull = tab + [.025, -.174, .150]
        p.add(2.5, "unwind one complete sheet against adhesion",
              right=(pull, PINCH, 0.))
        p.add(.8, "finish the perforation release")
        discard = np.array([.298, -.224, .845])
        p.add(1.2, "carry off the spent sheet",
              right=(discard, PINCH, 0.))
        p.add(.6, "release the spent sheet",
              right=(discard, PINCH, .6))
    if mode == "peel-only":
        p.add(.8, "hold the exposed fresh roll")
        return p.moves
    p.add(.65, "lift the roller from the rest", left=(handle + [0., 0., .048], HOLD, .02))
    p.add(1.65, "move the clean layer over the fabric",
          left=(rolling_start + [0., C.HANDLE_Y, .066], ROLL, .02),
          right=(brace + [0., 0., .043], TOP, .11))
    p.add(1.15, "brace the cloth hem",
          right=(brace, TOP, .11))
    p.add(1.2, "load the roller on the cloth",
          left=(rolling_start + [0., C.HANDLE_Y, .001], ROLL, .02))
    p.add(3.8, "roll across the lint patch without sliding",
          left=(rolling_end + [0., C.HANDLE_Y, .001], ROLL, .02))
    p.add(.5, "hold the complete cleaned patch in view")
    p.add(1., "lift the roller away",
          left=(rolling_end + [0., C.HANDLE_Y, .075], ROLL, .02),
          right=(brace + [0., 0., .07], TOP, .4))
    return p.moves
