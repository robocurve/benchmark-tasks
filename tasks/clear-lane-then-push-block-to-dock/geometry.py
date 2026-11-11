"""Geometry and physical constants for clear-lane-then-push-block-to-dock."""

from __future__ import annotations

import os
import numpy as np

# Table surface height
TABLE_Z = 0.7475

# Lane parameters: two fixed rails (20 mm tall, 100 mm apart inner clear width, 240 mm long along x)
# Shifted along x by -60 mm from draft to keep push stroke within 0.475 m bimanual reach envelope
LANE_X_MIN = 0.140
LANE_X_MAX = 0.380
LANE_LEN_X = LANE_X_MAX - LANE_X_MIN  # 0.240 m
LANE_WIDTH = 0.100                    # inner width between rails
RAIL_THICKNESS = 0.010
RAIL_HEIGHT = 0.020
RAIL_SIZE = (LANE_LEN_X, RAIL_THICKNESS, RAIL_HEIGHT)

# Rails centered at x = 0.260 m, y = +/- 0.055 m (inner surfaces at +/- 0.050 m)
RAIL_LEFT_POS = (0.260, +0.055, TABLE_Z + RAIL_HEIGHT / 2.0)
RAIL_RIGHT_POS = (0.260, -0.055, TABLE_Z + RAIL_HEIGHT / 2.0)

# Dock: stop plate front face at x = 0.375 m (so the cargo, 60 mm long, comes to rest with its centre at 0.345, within 12 mm of
# the white target mark centre at x = 0.340 m). The plate is 98 mm wide so it fits between the rails (cl_1 and cl_2 showed that a
# plate at x = 0.395 stopped the cargo at 0.360, 20 mm past the mark, which can never reach the full-score tolerance).
STOP_PLATE_SIZE = (0.010, 0.098, 0.040)
STOP_PLATE_POS = (0.380, 0.000, TABLE_Z + 0.020)

MARK_SIZE = (0.080, 0.080, 0.002)
MARK_POS = (0.340, 0.000, TABLE_Z + 0.001)

# Fragile red post: 20 x 20 x 80 mm free body at (0.340, +0.090)
POST_SIZE = (0.020, 0.020, 0.080)
POST_MASS = 0.02
POST_START_POS = (0.340, +0.090, TABLE_Z + POST_SIZE[2] / 2.0)
POST_COLOR = (0.85, 0.15, 0.15)

# Loose blocks: 40 x 40 x 40 mm, mass 0.04 kg, friction 0.8
BLOCK_SIZE = (0.040, 0.040, 0.040)
BLOCK_MASS = 0.04
BLOCK_FRICTION = 0.8
BLUE_START_POS = (0.220, 0.000, TABLE_Z + BLOCK_SIZE[2] / 2.0)
YELLOW_START_POS = (0.280, 0.000, TABLE_Z + BLOCK_SIZE[2] / 2.0)
BLUE_COLOR = (0.15, 0.35, 0.85)
YELLOW_COLOR = (0.90, 0.80, 0.15)

# Staging positions outside lane (|y| > 0.075 m)
BLUE_STAGING_POS = (0.220, +0.160, TABLE_Z + BLOCK_SIZE[2] / 2.0)
YELLOW_STAGING_POS = (0.280, -0.160, TABLE_Z + BLOCK_SIZE[2] / 2.0)

# Cargo block: 60 x 60 x 40 mm, mass 0.06 kg, friction 0.4, orange
CARGO_SIZE = (0.060, 0.060, 0.040)
CARGO_MASS = 0.06
CARGO_FRICTION = 0.4
CARGO_START_POS = (0.150, 0.000, TABLE_Z + CARGO_SIZE[2] / 2.0)
CARGO_COLOR = (0.95, 0.45, 0.05)

# Push parameters
PUSH_Z = TABLE_Z + 0.020  # mid-height of cargo block
PUSH_OVERSHOOT = float(os.environ.get("DEMO_PUSH_OVERSHOOT", "0.025"))  # 25 mm overshoot past mark center
PUSH_END_X = MARK_POS[0] + PUSH_OVERSHOOT  # 0.365 m

# Evaluation thresholds
LANE_CLEAR_Y = 0.075
CARGO_STAGE2_X = 0.230  # moved 80 mm from 0.150
DOCK_STAGE3_TOL = 0.025
DOCK_STAGE4_XY_TOL = 0.012
DOCK_STAGE4_YAW_DEG = 10.0
POST_MAX_TILT_DEG = 10.0
POST_MAX_DIST = 0.010
HOLD_TIME_S = 2.0

# Demo parameters
DEMO_TIME_SCALE = float(os.environ.get("DEMO_TIME_SCALE", "1.8"))
DEMO_HOVER_Z = float(os.environ.get("DEMO_HOVER_Z", "0.860"))
DEMO_PUSH_HOLD_S = float(os.environ.get("DEMO_PUSH_HOLD_S", "1.2"))
DEMO_FINAL_HOLD_S = float(os.environ.get("DEMO_FINAL_HOLD_S", "2.6"))

# Finger positions
FINGER_OPEN = -0.04695
FINGER_CLOSED = 0.00000
FINGER_GRASP_BLOCK = -0.0180  # grasps 40 mm block
