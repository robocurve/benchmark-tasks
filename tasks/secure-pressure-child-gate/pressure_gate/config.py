"""Metre dimensions for the fixed dry stairwell workcell."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "pressure_gate"
INSTRUCTION = "Block off the stairs for the child."
DESCRIPTION = "A loose pressure-mounted safety gate is at the foot of a stairway."
PHYSICS_HZ = 240
MAX_STEPS = 410
CHECK_DT = 0.05
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)

FLOOR_Z = 0.460
GATE_HEIGHT = 0.6604
GATE_X = 0.374
GATE_START_X = 0.355
WIDTH = 0.700
CUP_BOTTOM = 0.345
CUP_HEIGHTS = (0.089, 0.582)
PAD_RADIUS = 0.016
PAD_LENGTH = 0.017
PAD_CENTER = 0.331
LEVER_PIVOT = np.array([-.037, -.012, .324])
LEVER_LENGTH = 0.114
LEVER_STOP = 104.
CAM_OFFSET = .018
CAM_RADIUS = .013
