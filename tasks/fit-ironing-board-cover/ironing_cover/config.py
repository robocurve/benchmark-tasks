"""Full-size tabletop board and fitted pad dimensions in metres."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "ironing_cover"
INSTRUCTION = "Fit the padded cover over the ironing board. Tuck the elastic edge under both ends and leave the ironing surface flat."
DESCRIPTION = "A 73 by 32 cm tabletop ironing board supports a loose, unfitted padded cotton cover."
MAX_STEPS = 600
PHYSICS_HZ = 240
BASES = station.BASES
STOWED = {side: READY for side in BASES}
TABLE = station.TABLE_TOP
BOARD_X = .372
BOARD_TOP = TABLE + .105
HALF_LENGTH = .365
BOARD_THICK = .018
TOP = .006
START_Z = BOARD_TOP + TOP / 2
NEUTRAL_LENGTH = .308
TRAVEL = .122
DROP = .044
SPRING = 105.
DAMPING = 2.5
