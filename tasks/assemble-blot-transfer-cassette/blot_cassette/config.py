"""Full-size inert cassette and roller dimensions in SI units."""

import numpy as np

KEY = "blot_cassette"
INSTRUCTION = "Brace the wet gel and membrane, roll the trapped air out to an open edge, and close the cassette."
DESCRIPTION = "A full-size blot-transfer cassette holds an inert wet gel and membrane with three visible air pockets. The cover is open and a rubber roller rests beside it."
MAX_STEPS = 540
PHYSICS_HZ = 240
BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}
TABLE = .7475
ORIGIN = np.array([.365, .025, .826])
SHEET_LENGTH = .100
SHEET_WIDTH = .075
SHEET_Z = .017
ROLLER_R = .014
ROLLER_WIDTH = .106
ROLLER_START = np.array([ORIGIN[0], -.116, ORIGIN[2] + SHEET_Z + ROLLER_R])
GRIP_HEIGHT = .081
SHEAR_LIMIT = .012
SHEAR_STIFFNESS = 80.
CUSHION_STIFFNESS = 1800.
LID_HINGE = ORIGIN + [.066, 0., .027]
LID_OPEN_DEG = 96.
LID_GRIP = np.array([-.116, .018, .016])
GEL_DAMAGE_DEPTH = .0041
GEL_DAMAGE_SHIFT = .0045
ROLLER_NIP = .004
GAPS = ((-.029, -.022, .007), (.012, -.002, .008), (.028, .021, .007))
