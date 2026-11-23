"""Full-size domestic ironing board and its two indexed height positions."""

import math

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "ironing_height"
INSTRUCTION = "Lower the ironing board to its marked working height and leave it locked."
DESCRIPTION = "An unloaded domestic ironing board has crossed steel legs and a spring lever underneath the deck."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
PHYSICS_HZ = 240
MAX_STEPS = 380
CHECK_DT = .05

CENTER_X = .445
BASE_Z = .026
FIXED_Y = .325
UPPER_Y = -.335
UPPER_Z = .837
LOWER_Y = -.425
BAR_LENGTH = math.hypot(FIXED_Y - UPPER_Y, UPPER_Z - BASE_Z)
LOWER_Z = BASE_Z + math.sqrt(BAR_LENGTH**2 - (FIXED_Y - LOWER_Y)**2)
LIFT_Z = UPPER_Z + .009

HANDLE_PIVOT = np.array([.338, -.382, UPPER_Z - .047])
HANDLE_Y = -.222
HANDLE_END = np.array([-.073, HANDLE_Y - HANDLE_PIVOT[1], 0.])
LEVER_STOP = 37.
CATCH_SIZE = np.array([.025, .020, .021])
SHOULDER_CENTER = np.array([0., 0., -.064])
SHOULDER_SIZE = np.array([.032, .022, .018])
LOWER_CATCH = np.array([.10, LOWER_Y - HANDLE_PIVOT[1] - .0205, -.017])
