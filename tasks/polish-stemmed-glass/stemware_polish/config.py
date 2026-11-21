"""Dimensions and reset geometry in metres for a standard stemmed glass."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "stemware_polish"
INSTRUCTION = (
    "Polish the water marks off the outside of the wine glass with the microfiber "
    "cloth. Support the glass near its bowl while polishing, then set the glass "
    "upright and leave the cloth on the worktop."
)
DESCRIPTION = "Polish the curved outer bowl of a loose glass with a folded cloth."
MAX_STEPS = 760
PHYSICS_HZ = 240
BASES = station.BASES
STOWED = {name: READY for name in BASES}
TABLE = .7475
GLASS_START = np.array([.443, .146, TABLE + .001])
GLASS_WORK = np.array([.450, .112, TABLE + .064])
GRASP = np.array([0., 0., .062])
STEM_RADIUS = .006
CLOTH_START = np.array([.403, -.245, TABLE + .001])
CLOTH_GRASP = np.array([-.029, 0., .0205])
PAD_FACE = np.array([.055, 0., .012])
PAD_WIDTH = .047
PAD_HEIGHT = .019
FLEX_K = 2700.
LOAD_BAND = (.65, 12.)
CLOTH_YAW = 75.
MARK_DEGREES = (-18, 16, 0, -16, 19)
MARK_HEIGHTS = (.130, .130, .148, .166, .166)
MARK_Z = .148
MARK_RADIUS = .0042
PROFILE_Z = (.085, .091, .103, .119, .137, .155, .173, .191, .205)
PROFILE_R = (.008, .017, .029, .0385, .044, .045, .0435, .039, .035)
WALL = .002
