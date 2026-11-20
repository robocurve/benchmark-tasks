"""Fixed, full-size two-channel bench pump geometry in SI units."""

import numpy as np

KEY = "peristaltic_pump"
DESCRIPTION = "Load two flexible tubes into a cam-closing peristaltic head."
INSTRUCTION = "Seat both tubes in the open pump head, one in each track. Keep them straight at both retainers, close the lever, and let go for the water check."
MAX_STEPS = 860
PHYSICS_HZ = 160
TABLE = 0.7475
HEAD = np.array([0.358, 0.004, TABLE])
PIVOT = np.array([0.048, 0.0, 0.099])
LEVER = np.array([-0.020, -0.110, -0.020])
OPEN = np.deg2rad(70.0)
BED_Z = 0.077
TUBE_R = 0.004
TUBE_L = 0.032
LINKS = 11
TUBE_GRIP = 2
LANES = (0.009, -0.009)
STAGING = (0.335, 0.307)
PORT_Y = -0.174
STAGE_Z = 0.093
HANDS_Y = (0.088, -0.078)
RECEIVERS = ((0.383, 0.170), (0.348, 0.170))
TUBE_NAMES = ("upper_tube", "lower_tube")
COLORS = ((0.90, 0.48, 0.17), (0.22, 0.62, 0.82))
