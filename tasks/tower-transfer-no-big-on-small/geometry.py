"""Geometry and physical station layout for tower-transfer-no-big-on-small. Pure Python.

World frame: x forward, y left, z up, metres.
Arm bases: Left (0, +0.305, 0.75), Right (0, -0.305, 0.75).
Table top z = 0.7475 m (TOP).

Contents:
- Three target marks (flat 100 x 100 x 2 mm, non-colliding):
  LEFT at (0.240, +0.090, TOP + 0.001)
  MIDDLE at (0.320, 0.000, TOP + 0.001)
  RIGHT at (0.240, -0.090, TOP + 0.001)
- Three free blocks:
  Large (blue): 80 x 80 x 25 mm, mass 0.06 kg, friction 1.2
  Medium (orange): 60 x 60 x 25 mm, mass 0.05 kg, friction 1.2
  Small (yellow): 45 x 45 x 25 mm, mass 0.04 kg, friction 1.2
  Initial configuration: Tower on LEFT mark (Large at bottom, Med in middle, Small on top).
"""

from __future__ import annotations

TOP = 0.7475

# Colours (sRGB)
MARK_WHITE = (0.95, 0.95, 0.95)
BLOCK_LARGE_BLUE = (0.15, 0.35, 0.85)
BLOCK_MED_ORANGE = (0.90, 0.45, 0.10)
BLOCK_SMALL_YELLOW = (0.95, 0.80, 0.15)

# Target marks (100 x 100 x 2 mm)
MARK_SIZE = (0.100, 0.100, 0.002)
LEFT_MARK_POS = (0.240, +0.090, TOP + 0.001)
MID_MARK_POS = (0.320, 0.000, TOP + 0.001)
RIGHT_MARK_POS = (0.240, -0.090, TOP + 0.001)

# Block dimensions
BLOCK_HEIGHT = 0.025

LARGE_SIZE = (0.080, 0.080, BLOCK_HEIGHT)
LARGE_MASS = 0.060
LARGE_FRICTION = 1.2
LARGE_START_POS = (0.240, +0.090, TOP + BLOCK_HEIGHT / 2)

MEDIUM_SIZE = (0.060, 0.060, BLOCK_HEIGHT)
MEDIUM_MASS = 0.050
MEDIUM_FRICTION = 1.2
MEDIUM_START_POS = (0.240, +0.090, TOP + BLOCK_HEIGHT + BLOCK_HEIGHT / 2)

SMALL_SIZE = (0.045, 0.045, BLOCK_HEIGHT)
SMALL_MASS = 0.040
SMALL_FRICTION = 1.2
SMALL_START_POS = (0.240, +0.090, TOP + 2 * BLOCK_HEIGHT + BLOCK_HEIGHT / 2)

# Stack target heights (centre of blocks at each level)
LEVEL_1_TARGET_Z = TOP + BLOCK_HEIGHT / 2                     # 0.7600 m
LEVEL_2_TARGET_Z = TOP + BLOCK_HEIGHT + BLOCK_HEIGHT / 2      # 0.7850 m
LEVEL_3_TARGET_Z = TOP + 2 * BLOCK_HEIGHT + BLOCK_HEIGHT / 2  # 0.8100 m

# Tolerances
MARK_XY_TOL = 0.020           # 20 mm from mark centre for detection
FINAL_XY_TOL = 0.012          # 12 mm from RIGHT mark centre for 1.00
MAX_TILT_DEG = 5.0            # tilt < 5 degrees
STILL_LIN_VEL = 0.005         # 5 mm/s lin vel
REST_TIME_S = 0.3             # resting 0.3 s to count resting
HOLD_TIME_S = 2.0             # 2.0 s stable hold for 1.00
ARMS_CLEAR_DIST = 0.060       # TCP > 60 mm from stack centre

# Kinematics & grasping
PAD_TO_TCP = 0.013
HOVER_Z = 0.8500              # transit height (TOP + 0.1025 m)
FINGER_OPEN_VAL = -0.04695
FINGER_GRASP_LARGE = -0.038
FINGER_GRASP_MED = -0.028
FINGER_GRASP_SMALL = -0.020

LEFT_BASE = (0.0, +0.305)
RIGHT_BASE = (0.0, -0.305)
MAX_ARM_REACH = 0.50
