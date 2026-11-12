"""Geometry, spatial layout, and physical constants for segregate-incompatible-cleaners-in-cabinet.

Single source of truth across all modules, tests, and documentation.
"""

from __future__ import annotations

# World table reference height in yam_arena
TABLE_Z = 0.7475

# Real-size 32 oz commercial HDPE spray bottle dimensions
BOTTLE_DIAMETER = 0.084       # 84 mm
BOTTLE_RADIUS = 0.042         # 42 mm
BOTTLE_HEIGHT = 0.210         # 210 mm
BOTTLE_MASS = 0.42            # 0.42 kg

# Storage cabinet recessed cylindrical wells
CABINET_X = 0.250             # Center X of bays
BAY_LEFT_Y = 0.130            # Left Bay (+Y)
BAY_RIGHT_Y = -0.130          # Right Bay (-Y)
WELL_DIAMETER = 0.096         # 96 mm
WELL_RADIUS = 0.048           # 48 mm
WELL_DEPTH = 0.030            # 30 mm
RADIAL_CLEARANCE = 0.006      # 6.0 mm (strictly >= 4.0 mm: 48 mm - 42 mm = 6 mm)
BAY_DISTANCE = 0.260          # 260 mm between bay centers

# Seated target positions (geometric center of bottle when sitting flat in well)
BOTTLE_CENTER_Z = TABLE_Z + BOTTLE_HEIGHT / 2.0  # 0.8525 m
BAY_LEFT_POS = (CABINET_X, BAY_LEFT_Y, BOTTLE_CENTER_Z)
BAY_RIGHT_POS = (CABINET_X, BAY_RIGHT_Y, BOTTLE_CENTER_Z)

# Staging positions on table surface
STAGING_X = 0.220
STAGING_LEFT_Y = 0.290  # beyond the cabinet slab (y +-0.190) and its collars so the bottles do not start inside them
STAGING_RIGHT_Y = -0.290
STAGING_LEFT_POS = (STAGING_X, STAGING_LEFT_Y, BOTTLE_CENTER_Z)
STAGING_RIGHT_POS = (STAGING_X, STAGING_RIGHT_Y, BOTTLE_CENTER_Z)

# Standby and trajectory waypoints
HOVER_Z = 0.940         # carry height: bottle base 0.791 m clears the well collar top (0.7825 m)
GRASP_Z = 0.880
WELL_SEAT_TCP_Z = 0.8865     # TCP z when the bottle base is 1 mm above the well floor (floor top 0.7535 m)
STANDBY_Z = 0.900
HOME_LEFT = (0.180, STAGING_LEFT_Y, STANDBY_Z)
HOME_RIGHT = (0.180, STAGING_RIGHT_Y, STANDBY_Z)

# Gripper values (jaw gap = 2 * abs(v))
FINGER_OPEN = -0.04695        # 94 mm opening
FINGER_GRASP = -0.0180        # firm squeeze on contoured bottle neck (38 mm neck)
FINGER_CLOSED = 0.0000

# Scoring tolerances and thresholds
TOL_WELL_XY = 0.012           # 12 mm horizontal well centering tolerance
TOL_WELL_Z = 0.010            # 10 mm vertical seating tolerance
MAX_TILT_PLACED_DEG = 6.0     # tilt < 6.0 deg for placed/seated credit (0.25, 0.75)
MAX_TILT_SUCCESS_DEG = 5.0    # tilt < 5.0 deg for full success (1.00)
MIN_BOTTLE_SEPARATION_VIOLATION = 0.120  # distance < 0.120 m latches violation
MOVED_THRESHOLD_DIST = 0.020  # 20 mm displacement indicates bottle picked/moved
MAX_STILL_SPEED = 0.020       # 20 mm/s maximum linear velocity for still check
HOLD_TIME_S = 2.0             # 2.0 s stable hold required for 1.00 score
ARMS_CLEAR_DIST = 0.050       # 50 mm clearance from bottles to consider arms retracted
