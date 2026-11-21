"""Physical geometry in metres on the YAM tabletop."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "tlc_plate"
INSTRUCTION = (
    "Finish developing the TLC plate. When the solvent front is 9 to 14 mm "
    "below the top edge, lift the plate out by its top margin, pencil-mark the "
    "wet front, and stand it coating-out in the clean drying rest."
)
DESCRIPTION = "A silica plate developing food dye in a covered glass tank."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
TOP = station.TABLE_TOP
PHYSICS_HZ = 240
MAX_STEPS = 900
CHECK_DT = .05

TANK = np.array([.289, -.064, TOP + .004])
TANK_SIZE = (.092, .120, .075)
WALL = .0035
SOLVENT_DEPTH = .0045
LID_SIZE = (.101, .128, .005)
LID_SKIRT = .029
LID_Z = TANK[2] + TANK_SIZE[2] + LID_SKIRT + LID_SIZE[2] / 2
LID = np.array([*TANK[:2], LID_Z])
COVER_PARK = np.array([.389, .232, TOP + LID_SKIRT + LID_SIZE[2] / 2])

PLATE_WIDTH = .050
PLATE_LENGTH = .100
PLATE_THICK = .002
PLATE_ANGLE = np.deg2rad(12.)
# Local Y runs from the dipped bottom edge to the extraction margin.
PLATE_R = np.array([
    [0., np.sin(PLATE_ANGLE), -np.cos(PLATE_ANGLE)],
    [-1., 0., 0.],
    [0., np.cos(PLATE_ANGLE), np.sin(PLATE_ANGLE)],
])
PLATE_CENTER = np.array([
    TANK[0] + .022,
    TANK[1],
    TANK[2] + WALL + PLATE_LENGTH * np.cos(PLATE_ANGLE),
]) - PLATE_R @ [0., PLATE_LENGTH / 2, 0.]
BASELINE = .012
INITIAL_FRONT = .0825
FRONT_MIN = .086
FRONT_MAX = .091
# Square-root wetting law begins at the specified pre-developed distance.
CAPILLARY_K = .000038
MAX_MARKS = 64
MARK_DEADLINE = 15.
MARK_TOLERANCE = .0025
PLACEMENT = np.array([.275, .095, TOP + .014 + PLATE_LENGTH * np.cos(PLATE_ANGLE) / 2])
WORK = np.array([.263, .042, .891])
PENCIL_START = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])

PENCIL = np.array([.216, .212, TOP + .024])
PENCIL_TIP = np.array([0., -.064, 0.])
LID_GRIP_OFFSET = np.array([0., 0., .022])
PLATE_GRIP_OFFSET = np.array([.010, .045, 0.])
