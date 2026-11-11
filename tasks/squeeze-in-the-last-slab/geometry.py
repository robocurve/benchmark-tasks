"""Geometry and physical constants for squeeze-in-the-last-slab."""

from __future__ import annotations

import os
import numpy as np

# Table surface height
TABLE_Z = 0.7475

# Standing slab dimensions: 12 mm (x) x 60 mm (y) x 80 mm (z)
SLAB_THICKNESS = float(os.environ.get("SLAB_THICKNESS", "0.012"))
SLAB_WIDTH = 0.060
SLAB_HEIGHT = 0.080
SLAB_DIMS_STANDING = (SLAB_THICKNESS, SLAB_WIDTH, SLAB_HEIGHT)
SLAB_DIMS_FLAT = (SLAB_HEIGHT, SLAB_WIDTH, SLAB_THICKNESS)  # 80 x 60 x 12 mm

SLAB_MASS = 0.03
SLAB_FRICTION = 1.0

# Standing rest height
SLAB_STANDING_Z = TABLE_Z + SLAB_HEIGHT / 2.0  # 0.7875 m
SLAB_FLAT_Z = TABLE_Z + SLAB_THICKNESS / 2.0   # 0.7535 m

# Slabs row along x at y = 0.040 (generous bimanual reach margin).
# Slab 1 at 0.190, slab 2 at 0.235, slab 3 at 0.300, slab 4 at 0.350, empty slot centred at 0.325:
# the neighbours of the slot sit 50 mm apart centre to centre, so the clear width is 38 mm for a
# 12 mm slab (13 mm each side).
SLAB_Y = 0.040
SLAB_1_POS = (0.190, SLAB_Y, SLAB_STANDING_Z)
SLAB_2_POS = (0.235, SLAB_Y, SLAB_STANDING_Z)
SLAB_3_POS = (0.300, SLAB_Y, SLAB_STANDING_Z)
SLAB_4_POS = (0.350, SLAB_Y, SLAB_STANDING_Z)

SLOT_TARGET_POS = (0.325, SLAB_Y, SLAB_STANDING_Z)

# Fifth slab (blue) start pose: standing on its 12 mm edge on the table, left arm side. The arms grip
# straight down (yaw only), so they cannot turn a flat slab on end: a first GPU run grasped it flat
# and nothing stood it up (score 0.0), so the fifth slab starts already standing.
SLAB_5_START = (0.190, +0.220, SLAB_STANDING_Z)

# White strip under standing slabs
STRIP_POS = (0.280, SLAB_Y, TABLE_Z + 0.001)
STRIP_SIZE = (0.260, 0.080, 0.002)

# Tolerances and zero-gate limits
KNOCK_TILT_DEG = 10.0
KNOCK_DIST_XY = 0.008

GOAL_TILT_DEG = 4.0
GOAL_XY_TOL = 0.008
NEIGHBOUR_STABLE_TILT = 5.0
NEIGHBOUR_STABLE_DIST = 0.004

HOLD_TIME_S = 2.0

# Demo parameters
DEMO_TIME_SCALE = float(os.environ.get("DEMO_TIME_SCALE", "1.8"))
DEMO_HOVER_Z = float(os.environ.get("DEMO_HOVER_Z", "0.885"))
DEMO_INSERT_DZ = float(os.environ.get("DEMO_INSERT_DZ", "0.003"))
DEMO_FINAL_HOLD_S = float(os.environ.get("DEMO_FINAL_HOLD_S", "2.6"))

# Finger values
FINGER_OPEN = -0.04695
# jaw gap ~ 2|v|: the 60 mm slab needs a squeeze (-0.030 = zero squeeze, nothing lifts)
FINGER_GRASP_WIDTH = float(os.environ.get("DEMO_GRASP_FINGER", "-0.021"))
DEMO_KNOCK_DX = float(os.environ.get("DEMO_KNOCK_DX", "0.015"))  # knock_neighbour: off-centre by this much

# Colors (RGB)
COLOR_SLAB_GREY = (0.70, 0.70, 0.70)
COLOR_SLAB_BLUE = (0.15, 0.45, 0.85)
COLOR_STRIP = (0.95, 0.95, 0.95)
