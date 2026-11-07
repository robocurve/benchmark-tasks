"""Geometry and physical station layout for stack-crates-largest-first. Pure Python.

World frame: x forward, y left, z up, metres.
Arm bases: Left (0, +0.305, 0.75), Right (0, -0.305, 0.75).
Table top z = 0.7475 m (TOP).

Contents:
- Pallet marker: 110 x 110 x 2 mm white square at (0.340, 0.000, TOP + 0.001).
- Large crate (blue): 80 x 80 x 40 mm, mass 0.06 kg, friction 1.2, start at (0.280, +0.280, TOP + 0.020).
- Medium crate (orange): 60 x 60 x 40 mm, mass 0.05 kg, friction 1.2, start at (0.380, -0.280, TOP + 0.020).
- Small crate (yellow): 45 x 45 x 40 mm, mass 0.04 kg, friction 1.2, start at (0.300, -0.220, TOP + 0.020).
"""

from __future__ import annotations

TOP = 0.7475

# Colours (sRGB)
PALLET_WHITE = (0.95, 0.95, 0.95)
CRATE_LARGE_BLUE = (0.15, 0.35, 0.85)
CRATE_MEDIUM_ORANGE = (0.90, 0.45, 0.10)
CRATE_SMALL_YELLOW = (0.95, 0.80, 0.15)

# Pallet target marker
PALLET_POS = (0.260, 0.000, TOP + 0.001)  # adjusted from 0.340 for bimanual reach margin
PALLET_SIZE = (0.110, 0.110, 0.002)

# Crate dimensions
CRATE_HEIGHT = 0.040

LARGE_SIZE = (0.080, 0.080, CRATE_HEIGHT)
LARGE_MASS = 0.060
LARGE_FRICTION = 1.2
LARGE_START_POS = (0.280, +0.280, TOP + CRATE_HEIGHT / 2)  # predicted, to be measured

MEDIUM_SIZE = (0.060, 0.060, CRATE_HEIGHT)
MEDIUM_MASS = 0.050
MEDIUM_FRICTION = 1.2
MEDIUM_START_POS = (0.380, -0.280, TOP + CRATE_HEIGHT / 2)  # predicted, to be measured

SMALL_SIZE = (0.045, 0.045, CRATE_HEIGHT)
SMALL_MASS = 0.040
SMALL_FRICTION = 1.2
SMALL_START_POS = (0.300, -0.220, TOP + CRATE_HEIGHT / 2)  # predicted, to be measured

# Stack target heights (centre of crates at each level)
LEVEL_1_TARGET_Z = TOP + CRATE_HEIGHT / 2                    # 0.7675 m
LEVEL_2_TARGET_Z = TOP + CRATE_HEIGHT + CRATE_HEIGHT / 2     # 0.8075 m
LEVEL_3_TARGET_Z = TOP + 2 * CRATE_HEIGHT + CRATE_HEIGHT / 2 # 0.8475 m

# Placement and stability tolerances
LEVEL_1_MAX_OFFSET = 0.015   # 15 mm from pallet centre
LEVEL_2_MAX_OFFSET = 0.012   # 12 mm from support centre
LEVEL_3_MAX_OFFSET = 0.012   # 12 mm from support centre
FINAL_ALL_MAX_OFFSET = 0.012 # 12 mm for final perfect stack 1.00

MAX_TILT_DEG = 5.0           # tilt < 5 degrees
MIN_SUPPORT_OVERLAP_DZ = 0.035  # must be sitting above lower crate floor

STILL_LIN_VEL = 0.005        # 5 mm/s
STILL_ANG_VEL = 0.200        # 0.2 rad/s
ARMS_CLEAR_DIST = 0.060      # TCP > 60 mm from stack centre

# Kinematics & grasping
PAD_TO_TCP = 0.013
HOVER_Z = TOP + 0.140        # safe transit height (0.8875 m)
GRASP_Z_OFFSET = 0.010       # grip 10 mm below crate top

LEFT_BASE = (0.0, +0.305)
RIGHT_BASE = (0.0, -0.305)
MAX_ARM_REACH = 0.50

