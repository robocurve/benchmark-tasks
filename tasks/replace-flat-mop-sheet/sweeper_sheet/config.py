"""A 400 mm strip-mop frame on a raised service tile."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "sweeper_sheet"
INSTRUCTION = (
    "Fit the dry cleaning pad onto the floor mop and close both end clamps. "
    "Lift the mop by its handle, then slide it across the tiled area with the pad still fitted."
)
DESCRIPTION = INSTRUCTION
MAX_STEPS = 760
PHYSICS_HZ = 240
BASES = station.BASES
STOWED = {side: READY for side in BASES}
TABLE = station.TABLE_TOP
PLATFORM = TABLE + .043
X = .360
FABRIC_Z = PLATFORM + .0027
HEAD_Z = PLATFORM + .0050
HALF_WIDTH = .201
HEAD_DEPTH = .111
HEAD_THICK = .014
WRAP_Y = .202
WRAP_LENGTH = .021
STRIP_LENGTH = .039
CLAMP_Y = .145
CLAMP_LENGTH = .043
CLAMP_Z = .0216
HANDLE_Z = HEAD_Z + .075
POLE_LENGTH = 1.22
POLE_REST_ANGLE = 70.
WIPE_MIN_X = .116
WIPE_MAX_X = .226
WIPE_DISTANCE = .060
END_POINTS = {"Left": 1, "Right": -1}
LIFT_HEIGHT = .031


def end_point(side, y, z, x=0.):
    return np.array([X+x, side*y, z])
