"""Geometry and physical station layout for hook-block-from-guarded-zone. Pure Python.

World frame: x forward, y left, z up, metres.
Arm bases: Left (0, +0.305, 0.75), Right (0, -0.305, 0.75).
Table top z = 0.7475 m (TOP).

Contents:
- Yellow line marker: 8 mm wide (X), 0.40 m long (Y), at x = 0.40.
- Guarded zone: x > 0.40. Boundary fault threshold: x > 0.41 (0.40 + 0.01 m margin).
- Red block: 40 x 40 x 40 mm, mass 0.05 kg, friction 0.4, at (0.58, 0.0). Beyond hand reach (> 0.50 m).
- Hooked rod: shaft 300 x 20 x 20 mm + 20 x 60 x 20 mm cross-piece (T-hook at far end), mass 0.06 kg, friction 1.5, green, at (0.30, +0.25).
- Green tray: 130 x 130 x 20 mm receptacle at (0.28, -0.30).
"""

from __future__ import annotations

import math
from typing import Sequence

TOP = 0.7475

# Colours (sRGB)
YELLOW_LINE_COL = (0.95, 0.85, 0.10)
RED_BLOCK_COL = (0.85, 0.15, 0.15)
GREEN_ROD_COL = (0.15, 0.75, 0.25)
GREEN_TRAY_COL = (0.20, 0.65, 0.30)
DARK_GREY = (0.25, 0.25, 0.25)

# Yellow line marker (flat 8 mm wide, 0.40 m long along y, 2 mm thick)
LINE_SIZE = (0.008, 0.400, 0.002)
LINE_POS = (0.400, 0.000, TOP + 0.001)

# Guarded zone boundaries
GUARDED_ZONE_X = 0.400
FAULT_MARGIN = 0.010
FAULT_BOUNDARY_X = GUARDED_ZONE_X + FAULT_MARGIN  # 0.410 m

# Block dimensions and properties
BLOCK_SIZE = (0.040, 0.040, 0.040)
BLOCK_HEIGHT = 0.040
BLOCK_MASS = 0.050
BLOCK_FRICTION = 0.40
BLOCK_INIT_POS = (0.580, 0.000, TOP + BLOCK_HEIGHT / 2)  # (0.580, 0.000, 0.7675)

# Hooked rod dimensions and properties
ROD_SHAFT_SIZE = (0.300, 0.020, 0.020)
ROD_HOOK_SIZE = (0.020, 0.060, 0.020)
ROD_THICKNESS = 0.020
ROD_HOOK_Y_OFFSET = -0.030
ROD_MASS = 0.060
ROD_FRICTION = 1.5
ROD_INIT_POS = (0.300, +0.250, TOP + ROD_THICKNESS / 2)  # (0.300, 0.250, 0.7575)

# Grasp offset: 60 mm from handle end along shaft (+X)
# In rod local frame, shaft extends from x = -0.150 to +0.150. Handle end is at -0.150.
# 60 mm from handle end is at local x = -0.150 + 0.060 = -0.090 m.
# In world frame at start: x = 0.300 - 0.090 = 0.210 m.
ROD_HANDLE_LOCAL_X = -0.150
ROD_GRASP_OFFSET = 0.030       # was 0.060: the T must clear the block's back face with the TCP at x <= 0.36
ROD_GRASP_LOCAL_X = ROD_HANDLE_LOCAL_X + ROD_GRASP_OFFSET  # -0.090 m
ROD_GRASP_START_POS = (ROD_INIT_POS[0] + ROD_GRASP_LOCAL_X, ROD_INIT_POS[1], ROD_INIT_POS[2])  # (0.210, 0.250, 0.7575)

# Staging park position for hooked rod outside guarded zone
ROD_PARK_POS = (0.220, +0.250, TOP + ROD_THICKNESS / 2)

# Green Tray dimensions (130 x 130 mm outer footprint)
TRAY_POS = (0.280, -0.300, TOP)
TRAY_SIZE = (0.130, 0.130, 0.020)
TRAY_WALL_THICK = 0.010
TRAY_FLOOR_THICK = 0.005
TRAY_INNER_HALF_X = (TRAY_SIZE[0] - 2 * TRAY_WALL_THICK) / 2  # 0.055 m
TRAY_INNER_HALF_Y = (TRAY_SIZE[1] - 2 * TRAY_WALL_THICK) / 2  # 0.055 m
TRAY_SEAT_Z = TOP + TRAY_FLOOR_THICK + BLOCK_HEIGHT / 2         # 0.7725 m

# Scoring thresholds
BLOCK_MOVED_X = 0.520          # 60 mm moved from 0.580 (x <= 0.520)
BLOCK_OUTSIDE_X = 0.380        # block outside guarded zone (x < 0.380 resting)
STILL_LIN_VEL = 0.005          # 5 mm/s maximum velocity for resting
REST_TIME_S = 0.3              # resting 0.3 s to register resting
HOLD_TIME_S = 2.0              # 2.0 s stable hold for 1.00
ARMS_CLEAR_DIST = 0.080        # TCP > 80 mm from tray centre
ROD_OUTSIDE_X = 0.380          # rod resting outside guarded zone

# Arm bases and kinematics
LEFT_BASE = (0.0, +0.305, 0.750)
RIGHT_BASE = (0.0, -0.305, 0.750)
MAX_ARM_REACH = 0.50           # maximum nominal arm reach

PAD_TO_TCP = 0.013
HOVER_Z = 0.8500               # transit hover height
FINGER_OPEN_VAL = -0.04695
FINGER_GRASP_ROD = -0.005      # 20 mm shaft: pads must close to ~10 mm (the first GPU run used -0.018 = 36 mm gap and never gripped). Env DEMO_ROD_FINGER
FINGER_GRASP_BLOCK = -0.012    # 40 mm block: 24 mm gap = 16 mm squeeze (the V7 pinch). Env DEMO_BLOCK_FINGER


def in_guarded_zone(point: Sequence[float]) -> bool:
    """Check if point is in the guarded zone (x > 0.40 m)."""
    return point[0] > GUARDED_ZONE_X


def is_boundary_violation(point: Sequence[float]) -> bool:
    """Check if point violates the boundary limit with margin (x > 0.41 m)."""
    return point[0] > FAULT_BOUNDARY_X


def in_tray(point: Sequence[float]) -> bool:
    """Check if block centre rests within the green tray inner footprint and height."""
    x, y, z = point[0], point[1], point[2]
    return (
        abs(x - TRAY_POS[0]) <= TRAY_INNER_HALF_X
        and abs(y - TRAY_POS[1]) <= TRAY_INNER_HALF_Y
        and (TOP <= z <= TOP + 0.080)
    )


def is_rod_outside(point: Sequence[float]) -> bool:
    """Check if hooked rod centre rests safely outside the guarded zone."""
    return point[0] < ROD_OUTSIDE_X


def _box(centre, size, col):
    return (tuple(centre), tuple(size), tuple(col))


def tray_world_boxes() -> list[tuple]:
    """Boxes forming the green tray receptacle."""
    hw_x = TRAY_SIZE[0] / 2
    hw_y = TRAY_SIZE[1] / 2
    wall_h = TRAY_SIZE[2] - TRAY_FLOOR_THICK
    wall_z = TOP + TRAY_FLOOR_THICK + wall_h / 2

    return [
        # Floor plate
        _box((TRAY_POS[0], TRAY_POS[1], TOP + TRAY_FLOOR_THICK / 2),
             (TRAY_SIZE[0], TRAY_SIZE[1], TRAY_FLOOR_THICK), GREEN_TRAY_COL),
        # Left wall (+Y)
        _box((TRAY_POS[0], TRAY_POS[1] + hw_y - TRAY_WALL_THICK / 2, wall_z),
             (TRAY_SIZE[0], TRAY_WALL_THICK, wall_h), GREEN_TRAY_COL),
        # Right wall (-Y)
        _box((TRAY_POS[0], TRAY_POS[1] - hw_y + TRAY_WALL_THICK / 2, wall_z),
             (TRAY_SIZE[0], TRAY_WALL_THICK, wall_h), GREEN_TRAY_COL),
        # Front wall (+X)
        _box((TRAY_POS[0] + hw_x - TRAY_WALL_THICK / 2, TRAY_POS[1], wall_z),
             (TRAY_WALL_THICK, TRAY_SIZE[1] - 2 * TRAY_WALL_THICK, wall_h), GREEN_TRAY_COL),
        # Rear wall (-X)
        _box((TRAY_POS[0] - hw_x + TRAY_WALL_THICK / 2, TRAY_POS[1], wall_z),
             (TRAY_WALL_THICK, TRAY_SIZE[1] - 2 * TRAY_WALL_THICK, wall_h), GREEN_TRAY_COL),
    ]


def rod_local_boxes() -> list[tuple]:
    """Local boxes forming the hooked rod: shaft (300x20x20 mm) + cross-piece (20x60x20 mm) at far end."""
    return [
        # Main shaft: length 300 mm along X, centered at 0
        _box((0.000, 0.000, 0.000), ROD_SHAFT_SIZE, GREEN_ROD_COL),
        # Cross-piece (T-hook): 20 mm along X, 60 mm along Y, centered at x = +0.140 m
        # Offset in -Y by 30 mm so the hook is an L: the shaft passes BESIDE the block (shaft y = block y + 0.045)
        # and the cross-piece reaches behind it.
        _box((0.140, ROD_HOOK_Y_OFFSET, 0.000), ROD_HOOK_SIZE, GREEN_ROD_COL),
    ]
