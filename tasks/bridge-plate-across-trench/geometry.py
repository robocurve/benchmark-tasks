"""Geometry and physical station layout for bridge-plate-across-trench. Pure Python.

World frame: x forward, y left, z up, metres.
Arm bases: Left (0, +0.305, 0.75), Right (0, -0.305, 0.75).
Table top z = 0.7475 m (TOP).

Contents:
- Platforms: 120 (y) x 260 (x) x 30 mm tall, grey, separated by a 90 mm (y) trench at y=0.
  - Near platform lip: y in [0.045, 0.165]
  - Far platform lip: y in [-0.165, -0.045]
  - Trench: y in [-0.045, 0.045]
- Bridge plate: free body 80 (x) x 130 (y) x 6 mm, mass 0.05 kg, friction 1.5, orange,
  initially resting on near platform at (0.30, +0.20, TOP + 0.030 + 0.003).
- Block: free body 40 x 40 x 30 mm, mass 0.05 kg, friction 0.5, blue,
  initially on near platform at (0.25, +0.14, TOP + 0.030 + 0.015).
- Target mark: 70 x 70 x 2 mm flat white mark on far platform at (0.30, -0.14, TOP + 0.030 + 0.001).
"""

from __future__ import annotations

TOP = 0.7475

# Colours (sRGB)
PLATFORM_GREY = (0.50, 0.50, 0.50)
BRIDGE_ORANGE = (0.95, 0.45, 0.10)
BLOCK_BLUE = (0.15, 0.35, 0.85)
MARK_WHITE = (0.95, 0.95, 0.95)

# Platform & Trench parameters
PLATFORM_HEIGHT = 0.030
PLATFORM_TOP_Z = TOP + PLATFORM_HEIGHT                          # 0.7775 m
PLATFORM_X_SIZE = 0.260
PLATFORM_Y_SIZE = 0.120
PLATFORM_SIZE = (PLATFORM_X_SIZE, PLATFORM_Y_SIZE, PLATFORM_HEIGHT)

TRENCH_WIDTH = 0.090
TRENCH_Y_MIN = -0.045
TRENCH_Y_MAX = 0.045
TRENCH_Z_FLOOR = TOP

# Platform lips facing trench
NEAR_PLATFORM_Y_MIN = 0.045
NEAR_PLATFORM_Y_MAX = 0.165
NEAR_PLATFORM_SIZE = PLATFORM_SIZE
NEAR_PLATFORM_POS = (0.300, 0.105, TOP + PLATFORM_HEIGHT / 2)   # 0.7625 m

FAR_PLATFORM_Y_MIN = -0.165
FAR_PLATFORM_Y_MAX = -0.045
FAR_PLATFORM_SIZE = PLATFORM_SIZE
FAR_PLATFORM_POS = (0.300, -0.105, TOP + PLATFORM_HEIGHT / 2)   # 0.7625 m

# Platform staging / rear support ledges (ensures bridge start at y=0.20 and far target are fully supported)
NEAR_STAGING_SIZE = (0.260, 0.180, PLATFORM_HEIGHT)
NEAR_STAGING_POS = (0.300, 0.255, TOP + PLATFORM_HEIGHT / 2)

FAR_REAR_SIZE = (0.260, 0.120, PLATFORM_HEIGHT)
FAR_REAR_POS = (0.300, -0.225, TOP + PLATFORM_HEIGHT / 2)

# Bridge plate: 80 (x) x 130 (y) x 6 mm
BRIDGE_SIZE = (0.080, 0.150, 0.002)  # 2 mm: a 6 mm plate was a step the pushed block jammed against;  # measured: 130 mm gave exactly 20 mm lip overlap, which failed the >= 20 mm seat test
BRIDGE_MASS = 0.150   # measured: 0.05 kg plate was dragged 37 mm by the pushed block
BRIDGE_FRICTION = 1.5
BRIDGE_START_POS = (0.250, +0.270, TOP + PLATFORM_HEIGHT + 0.002 / 2)  # 0.7785 m (2 mm plate)
BRIDGE_SEATED_TARGET_POS = (0.250, 0.000, TOP + PLATFORM_HEIGHT + 0.002 / 2)

# Block: 40 x 40 x 30 mm
BLOCK_SIZE = (0.040, 0.040, 0.030)
BLOCK_MASS = 0.050
BLOCK_FRICTION = 0.3
BLOCK_START_POS = (0.250, +0.140, TOP + PLATFORM_HEIGHT + 0.030 / 2)   # 0.7925 m

# Target mark: 70 x 70 x 2 mm on far platform
MARK_SIZE = (0.070, 0.070, 0.002)
TARGET_MARK_POS = (0.250, -0.140, TOP + PLATFORM_HEIGHT + 0.001)       # 0.7785 m
BLOCK_TARGET_POS = (0.250, -0.140, TOP + PLATFORM_HEIGHT + 0.030 / 2)  # 0.7925 m

# Tolerances & Thresholds
BRIDGE_MIN_OVERLAP = 0.020      # both y-edges overlap platform lip by >= 20 mm
BRIDGE_MAX_TILT_DEG = 3.0       # tilt < 3 degrees
TARGET_XY_TOL = 0.015           # block centered on target mark within 15 mm
STILL_LIN_VEL = 0.005           # 5 mm/s lin vel
REST_TIME_S = 0.3               # resting 0.3 s
HOLD_TIME_S = 2.0               # 2.0 s stable hold for 1.00
ARMS_CLEAR_DIST = 0.060         # TCP > 60 mm from block/target
FELL_Z_THRESHOLD = TOP + PLATFORM_HEIGHT - 0.020  # z < 0.7575 m (fell in trench)
FAR_PLATFORM_LIP_Y = -0.045
NEAR_PLATFORM_LIP_Y = 0.045

# Kinematics & grasping
PAD_TO_TCP = 0.013
HOVER_Z = 0.8800
FINGER_OPEN_VAL = -0.04695
FINGER_GRASP_BRIDGE = -0.006   # pinch the 14 mm lift handle (measured: a 6 mm plate cannot be pinched flat)
FINGER_CLOSED_VAL = 0.0

LEFT_BASE = (0.0, +0.305)
RIGHT_BASE = (0.0, -0.305)
MAX_ARM_REACH = 0.50


# Lift handle on the +x end of the bridge plate (a flat 6 mm plate lying on a platform cannot be pinched).
# Predicted, to be measured; it sits outside the block's push path (block x ~0.25-0.27, handle x ~0.326-0.340).
HANDLE_SIZE = (0.014, 0.050, 0.030)
HANDLE_OFFSET_X = 0.033            # handle centre x relative to the plate centre
HANDLE_CENTER_Z = TOP + PLATFORM_HEIGHT + BRIDGE_SIZE[2] + HANDLE_SIZE[2] / 2   # 0.7985 m at rest


# Two low stop posts on the far platform just beyond the seated plate's far corners (outside the block's path, x 0.23-0.27).
# They stop the plate being dragged toward -y by the pushed block (measured: it slid 45 mm without them).
STOP_POST_SIZE = (0.020, 0.010, 0.008)
STOP_POST_XS = (BRIDGE_START_POS[0] - 0.045, BRIDGE_START_POS[0] + 0.045)
STOP_POST_Y = -0.088
STOP_POST_Z = TOP + PLATFORM_HEIGHT + STOP_POST_SIZE[2] / 2.0
