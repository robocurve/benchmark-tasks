"""Dimensions and reset frames for office materials in the calibrated YAM station."""

import numpy as np

from .motion import rotation

KEY = "page_protector"
INSTRUCTION = "Put the loose printed page all the way inside the clear sheet protector. Leave the page flat inside and take both grippers away."
DESCRIPTION = INSTRUCTION
PHYSICS_HZ = 480
MAX_STEPS = 760
TABLE = .7475

PAPER_LENGTH = .2794
PAPER_WIDTH = .2159
PAPER_MASS = .00724
PAPER_PROXY = .00054
PAPER_THICKNESS = .000153
PAPER_PANELS = ("Rear", "Middle", "Leading")
PANEL_LENGTH = PAPER_LENGTH / len(PAPER_PANELS)
PAPER_BEND_NM_RAD = .0105
PAPER_BEND_DAMPING_NMS_RAD = .00032

MOUTH = np.array([.481, .005, .888])
SLEEVE_PITCH = 24.
SLEEVE_ROTATION = rotation("Y", np.deg2rad(SLEEVE_PITCH))
SLEEVE_DEPTH = .289
INNER_HALF_WIDTH = .111
UPPER_OVERHANG = .009
LAYERS_GAP = .00145
FILM_PROXY = .00055

STOCK_REAR = np.array([.207, -.240, .969])
STOCK_PITCH = 24.
STOCK_ROTATION = rotation("Y", np.deg2rad(STOCK_PITCH))
STOCK_OVERHANG = .022
HAND_OFFSET_Y = -.027


def sleeve_point(x, y, z):
    return MOUTH + SLEEVE_ROTATION @ np.array([x, y, z])
