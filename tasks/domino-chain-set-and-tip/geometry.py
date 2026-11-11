"""Geometry and station physical layout for domino-chain-set-and-tip. Pure Python.

World frame: x forward, y left, z up, metres.
Arm bases: Left (0, +0.305, 0.75), Right (0, -0.305, 0.75).
Table top z = 0.7475 m (TOP).

Contents:
- Four white target marks (20 x 40 x 2 mm, non-colliding):
  Mark 0 at (0.260, 0.000, TOP + 0.001)
  Mark 1 at (0.310, 0.000, TOP + 0.001)
  Mark 2 at (0.360, 0.000, TOP + 0.001)
  Mark 3 at (0.410, 0.000, TOP + 0.001)
  Row along x at y = 0.0 with 50 mm pitch, 30 mm gap.
- Four dominoes (20 x 40 x 70 mm tall when standing, mass 0.02 kg, friction 0.45):
  Domino 0 (orange): starts near left at (0.280, +0.200)
  Domino 1 (blue):   starts near left at (0.380, +0.200)
  Domino 2 (yellow): starts near right at (0.280, -0.200)
  Domino 3 (green):  starts near right at (0.380, -0.200)
"""

from __future__ import annotations

import os

TOP = 0.7475

# Colours (sRGB)
MARK_WHITE = (0.95, 0.95, 0.95)
COLOR_ORANGE = (0.90, 0.45, 0.10)
COLOR_BLUE = (0.15, 0.35, 0.85)
COLOR_YELLOW = (0.95, 0.80, 0.15)
COLOR_GREEN = (0.15, 0.75, 0.25)

DOMINO_COLORS = [
    COLOR_ORANGE,
    COLOR_BLUE,
    COLOR_YELLOW,
    COLOR_GREEN,
]

# Domino dimensions (tunable thickness knob)
DOMINO_THICKNESS = float(os.environ.get("DOMINO_THICKNESS", "0.020"))
DOMINO_WIDTH = 0.040
DOMINO_HEIGHT = 0.070
DOMINO_MASS = 0.020
DOMINO_FRICTION = float(os.environ.get("DOMINO_FRICTION", "0.45"))

DOMINO_SIZE = (DOMINO_THICKNESS, DOMINO_WIDTH, DOMINO_HEIGHT)
DOMINO_SIZE_FLAT = (DOMINO_HEIGHT, DOMINO_WIDTH, DOMINO_THICKNESS)

# Elevations
STANDING_Z = TOP + DOMINO_HEIGHT / 2          # 0.7825 m
FLAT_Z = TOP + DOMINO_THICKNESS / 2           # 0.7575 m

# Target marks (20 x 40 x 2 mm in a row along x at y = 0.0)
MARK_SIZE = (0.020, 0.040, 0.002)
MARK_PITCH = 0.050                            # 50 mm pitch
MARK_GAP = 0.030                              # 30 mm gap (0.050 - 0.020)

MARK_0_POS = (0.260, 0.000, TOP + 0.001)
MARK_1_POS = (0.310, 0.000, TOP + 0.001)
MARK_2_POS = (0.360, 0.000, TOP + 0.001)
MARK_3_POS = (0.410, 0.000, TOP + 0.001)

MARK_POSITIONS = [
    MARK_0_POS,
    MARK_1_POS,
    MARK_2_POS,
    MARK_3_POS,
]

# Start positions: two near left y = +0.20, two near right y = -0.20
DOMINO_0_START_POS = (0.280, +0.200, STANDING_Z)
DOMINO_1_START_POS = (0.380, +0.200, STANDING_Z)
DOMINO_2_START_POS = (0.280, -0.200, STANDING_Z)
DOMINO_3_START_POS = (0.380, -0.200, STANDING_Z)

DOMINO_START_POSITIONS = [
    DOMINO_0_START_POS,
    DOMINO_1_START_POS,
    DOMINO_2_START_POS,
    DOMINO_3_START_POS,
]

# Tolerances & thresholds per task spec
MAX_STANDING_TILT_DEG = 6.0       # tilt < 6 deg from vertical
MARK_XY_TOL = 0.008               # on mark within 8 mm
REST_TIME_S = 0.3                 # resting 0.3 s
STILL_LIN_VEL = 0.005             # 5 mm/s linear velocity limit for resting
PREMATURE_TILT_DEG = 45.0         # tilt > 45 deg before all 4 standing = premature
TIPPED_TILT_DEG = 20.0            # first domino tipped: tilt > 20 deg
FALLEN_TILT_DEG = 60.0            # fallen = tilt > 60 deg toward +x
MAX_TIP_TO_FALL_TIME_S = 5.0      # all four fallen within 5 s of tip
ARMS_CLEAR_DIST = 0.060           # TCP > 60 mm from dominoes / mark strip
TIP_PUSH_HEIGHT = 0.060           # tip first domino at 60 mm height above table

# Kinematics & grasping
PAD_TO_TCP = 0.013
HOVER_Z = 0.8875                  # transit clearance height
GRASP_Z = TOP + 0.045             # grasp height on upright domino
PLACE_Z = STANDING_Z + 0.005      # release height slightly above mark
PUSH_Z = TOP + TIP_PUSH_HEIGHT    # 60 mm height push target (0.8075 m)

FINGER_OPEN_VAL = -0.04695
FINGER_GRASP_DOMINO = float(os.environ.get("DEMO_GRASP_FINGER", "-0.010"))  # jaw gap ~2|v|: 20 mm squeeze on the 40 mm domino (was -0.020 = zero squeeze, nothing lifted)
FINGER_CLOSED_VAL = 0.0           # closed fingers for tipping push

LEFT_BASE = (0.0, +0.305)
RIGHT_BASE = (0.0, -0.305)
MAX_ARM_REACH = 0.50
