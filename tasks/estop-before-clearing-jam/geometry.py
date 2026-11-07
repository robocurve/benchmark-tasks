"""Geometry and spatial layout for estop-before-clearing-jam benchmark task.

Pure Python, no simulator dependencies.
World frame: x forward from arm bases, y left, z up (metres).
Station table top at z = 0.7475 m (TOP).
Left arm base at (0.0, +0.305, 0.75), right arm base at (0.0, -0.305, 0.75).

Scene components:
- Lane fixture: 300 (x) x 150 (y) mm walled channel at (0.340, +0.100), walls 10 mm thick, 20 mm tall.
- Danger zone: the lane volume x in [0.190, 0.490], y in [0.025, 0.175], z up to 0.120 m above table top.
- Pusher: 30 (x) x 120 (y) x 25 (z) mm bar sweeping along x over 90 mm, period 4.0 s, driven by code.
- Stuck block: orange 40 x 40 x 40 mm rigid cube lying in lane at (0.400, +0.100).
- Pushbuttons console: rigid base plate at y = -0.200 carrying 3 prismatic button sliders:
  - E-STOP: red, 60 mm cap at (0.280, -0.200)
  - RESET: white, 40 mm cap at (0.340, -0.200)
  - START: green, 40 mm cap at (0.400, -0.200)
- Indicator lamps: 25 mm diameter visual markers on the console plate:
  - STOP lamp (red) at (0.280, -0.155)
  - FAULT lamp (amber) at (0.340, -0.155)
  - RUN lamp (green) at (0.400, -0.155)
- Green tray: 130 x 130 x 20 mm receptacle at (0.300, +0.300).
- Parked pad: 100 x 100 x 5 mm staging pad at (0.420, +0.300).
"""

from __future__ import annotations

import math
from typing import Mapping, Sequence

TOP = 0.7475  # table top elevation in metres

# Arm bases and reach
LEFT_BASE = (0.0, +0.305, 0.75)
RIGHT_BASE = (0.0, -0.305, 0.75)
MIN_ARM_REACH = 0.10
MAX_ARM_REACH = 0.50

# Colours (sRGB 0..1)
DARK = (0.12, 0.12, 0.13)
DARK_GREY = (0.22, 0.22, 0.24)
STEEL_GREY = (0.35, 0.36, 0.38)
GREY = (0.55, 0.56, 0.58)
LIGHT_GREY = (0.75, 0.76, 0.78)
WHITE = (0.92, 0.92, 0.92)

RED_BTN = (0.88, 0.15, 0.15)
WHITE_BTN = (0.92, 0.92, 0.92)
GREEN_BTN = (0.15, 0.82, 0.22)
ORANGE_BLOCK = (1.00, 0.48, 0.00)
GREEN_TRAY_COL = (0.12, 0.68, 0.24)

LAMP_OFF = (0.18, 0.18, 0.20)
LAMP_AMBER = (0.98, 0.60, 0.05)
LAMP_GREEN = (0.10, 0.95, 0.20)
LAMP_RED = (0.95, 0.12, 0.12)

# ==============================================================================
# 1. Lane Fixture & Danger Zone
# ==============================================================================
LANE_POS = (0.340, 0.100)
LANE_SIZE_X = 0.300   # 300 mm along x
LANE_SIZE_Y = 0.150   # 150 mm along y
LANE_WALL_THICK = 0.010
LANE_WALL_HEIGHT = 0.020
LANE_FLOOR_THICK = 0.005

LANE_FLOOR_POS = (LANE_POS[0], LANE_POS[1], TOP + LANE_FLOOR_THICK / 2)
LANE_FLOOR_SIZE = (LANE_SIZE_X, LANE_SIZE_Y, LANE_FLOOR_THICK)
LANE_FLOOR_TOP_Z = TOP + LANE_FLOOR_THICK
LANE_WALL_TOP_Z = LANE_FLOOR_TOP_Z + LANE_WALL_HEIGHT

# Danger Zone boundaries
LANE_X_MIN = round(LANE_POS[0] - LANE_SIZE_X / 2, 4)   # 0.190 m
LANE_X_MAX = round(LANE_POS[0] + LANE_SIZE_X / 2, 4)   # 0.490 m
LANE_Y_MIN = round(LANE_POS[1] - LANE_SIZE_Y / 2, 4)   # 0.025 m
LANE_Y_MAX = round(LANE_POS[1] + LANE_SIZE_Y / 2, 4)   # 0.175 m
DANGER_Z_MAX = TOP + 0.120                   # 0.8675 m (120 mm above table)

def in_danger_zone(point: Sequence[float]) -> bool:
    """Check if point (x, y, z) lies inside the lane danger zone volume."""
    x, y, z = point[0], point[1], point[2]
    return (
        (LANE_X_MIN <= x <= LANE_X_MAX)
        and (LANE_Y_MIN <= y <= LANE_Y_MAX)
        and (TOP - 0.005 <= z <= DANGER_Z_MAX)
    )

# ==============================================================================
# 2. Pusher Bar (Code-driven non-colliding visual marker)
# ==============================================================================
PUSHER_SIZE = (0.030, 0.120, 0.025)  # 30 x 120 x 25 mm
PUSHER_STROKE = 0.090                 # 90 mm sweep stroke
PUSHER_PERIOD = 4.0                   # 4.0 s period
PUSHER_X_MIN = 0.240                  # sweeps from x = 0.240 to 0.330
PUSHER_X_MAX = PUSHER_X_MIN + PUSHER_STROKE
PUSHER_Z = LANE_FLOOR_TOP_Z + PUSHER_SIZE[2] / 2
PUSHER_INIT_POS = (PUSHER_X_MIN, LANE_POS[1], PUSHER_Z)

def pusher_x_at_time(t: float) -> float:
    """Position function of the code-driven pusher along X."""
    # Smooth sinusoidal sweep: 0.5 * stroke * (1 - cos(2*pi*t/T))
    phase = 2.0 * math.pi * (t % PUSHER_PERIOD) / PUSHER_PERIOD
    return PUSHER_X_MIN + 0.5 * PUSHER_STROKE * (1.0 - math.cos(phase))

# ==============================================================================
# 3. Stuck Block
# ==============================================================================
BLOCK_SIZE = (0.040, 0.040, 0.040)
BLOCK_INIT_POS = (0.400, 0.100, LANE_FLOOR_TOP_Z + BLOCK_SIZE[2] / 2)
BLOCK_MASS = 0.045                    # 45 g (<= 0.05 kg per lesson 3)
BLOCK_FRICTION = 1.5

# ==============================================================================
# 4. Green Tray & Parked Pad
# ==============================================================================
TRAY_POS = (0.300, 0.300, TOP + 0.010)
TRAY_SIZE = (0.130, 0.130, 0.020)
TRAY_WALL_THICK = 0.010
TRAY_FLOOR_THICK = 0.005
TRAY_INNER_HALF_X = (TRAY_SIZE[0] - 2 * TRAY_WALL_THICK) / 2  # 0.055 m
TRAY_INNER_HALF_Y = (TRAY_SIZE[1] - 2 * TRAY_WALL_THICK) / 2  # 0.055 m

PARK_PAD_POS = (0.420, 0.300, TOP + 0.0025)
PARK_PAD_SIZE = (0.100, 0.100, 0.005)

def in_tray(point: Sequence[float]) -> bool:
    """Check if point (x, y, z) rests inside the green tray."""
    x, y, z = point[0], point[1], point[2]
    return (
        abs(x - TRAY_POS[0]) <= TRAY_INNER_HALF_X
        and abs(y - TRAY_POS[1]) <= TRAY_INNER_HALF_Y
        and (TOP <= z <= TOP + 0.080)
    )

def in_parked_pad(point: Sequence[float]) -> bool:
    """Check if point (x, y, z) rests on the alternative parked pad."""
    x, y, z = point[0], point[1], point[2]
    return (
        abs(x - PARK_PAD_POS[0]) <= PARK_PAD_SIZE[0] / 2
        and abs(y - PARK_PAD_POS[1]) <= PARK_PAD_SIZE[1] / 2
        and (TOP <= z <= TOP + 0.080)
    )

# ==============================================================================
# 5. Pushbuttons Console & Sliders
# ==============================================================================
CONSOLE_BASE_POS = (0.340, -0.200, TOP + 0.010)
CONSOLE_BASE_SIZE = (0.240, 0.120, 0.020)
CONSOLE_TOP_Z = TOP + CONSOLE_BASE_SIZE[2]  # TOP + 0.020 = 0.7675 m

BUTTON_TRAVEL = 0.008                 # 8 mm travel to stop
BUTTON_RATE = 100.0                   # 100 N/m soft spring
BUTTON_DAMPING = 5.0                  # 5 N s/m damping
BUTTON_MASS = 0.008                   # 8 g mass

# Button cap specifications
ESTOP_BTN_POS_XY = (0.280, -0.200)
RESET_BTN_POS_XY = (0.340, -0.200)
START_BTN_POS_XY = (0.400, -0.200)

ESTOP_CAP_SIZE = (0.060, 0.060, 0.012)
RESET_CAP_SIZE = (0.040, 0.040, 0.012)
START_CAP_SIZE = (0.040, 0.040, 0.012)

# Button rest elevation: cap bottom rests 8 mm above fixture top
BUTTON_REST_BOTTOM_Z = CONSOLE_TOP_Z + BUTTON_TRAVEL
BUTTON_REST_Z = BUTTON_REST_BOTTOM_Z + 0.012 / 2       # TOP + 0.034 m
BUTTON_REST_TOP_Z = BUTTON_REST_BOTTOM_Z + 0.012      # TOP + 0.040 m

BUTTON_PRESS_THRESHOLD = -0.0064      # 80 % travel (-6.4 mm)
BUTTON_RELEASE_THRESHOLD = -0.0035    # re-arm threshold (-3.5 mm)

BUTTON_PRESS_DEPTH_CMD = 0.014
PAD_TO_TCP = 0.013
BUTTON_PRESS_TCP_Z = BUTTON_REST_TOP_Z - BUTTON_PRESS_DEPTH_CMD + PAD_TO_TCP  # TOP + 0.039 m

ESTOP_REST_POS = (ESTOP_BTN_POS_XY[0], ESTOP_BTN_POS_XY[1], BUTTON_REST_Z)
RESET_REST_POS = (RESET_BTN_POS_XY[0], RESET_BTN_POS_XY[1], BUTTON_REST_Z)
START_REST_POS = (START_BTN_POS_XY[0], START_BTN_POS_XY[1], BUTTON_REST_Z)

# ==============================================================================
# 6. Indicator Lamps (25 mm diameter visual markers on console)
# ==============================================================================
LAMP_DIAMETER = 0.025
LAMP_RADIUS = LAMP_DIAMETER / 2
LAMP_HEIGHT = 0.004
LAMP_Y = -0.155                       # inboard of buttons on console plate

STOP_LAMP_POS = (ESTOP_BTN_POS_XY[0], LAMP_Y, CONSOLE_TOP_Z + LAMP_HEIGHT / 2)
FAULT_LAMP_POS = (RESET_BTN_POS_XY[0], LAMP_Y, CONSOLE_TOP_Z + LAMP_HEIGHT / 2)
RUN_LAMP_POS = (START_BTN_POS_XY[0], LAMP_Y, CONSOLE_TOP_Z + LAMP_HEIGHT / 2)

LAMP_LIT_Z = CONSOLE_TOP_Z + LAMP_HEIGHT + 0.001
STOWED_Z = TOP - 0.100

STOP_LAMP_LIT_POS = (STOP_LAMP_POS[0], STOP_LAMP_POS[1], LAMP_LIT_Z)
FAULT_LAMP_LIT_POS = (FAULT_LAMP_POS[0], FAULT_LAMP_POS[1], LAMP_LIT_Z)
RUN_LAMP_LIT_POS = (RUN_LAMP_POS[0], RUN_LAMP_POS[1], LAMP_LIT_Z)

# ==============================================================================
# 7. Arm Waypoint Elevations and Home Poses
# ==============================================================================
HOVER_Z = TOP + 0.100                 # 0.8475 m (clear above danger zone with kinematic headroom)
LIFT_CLEAR_Z = TOP + 0.100            # 0.8475 m (lift block clear over 20 mm lane walls)

HOME_L = (0.200, +0.280, HOVER_Z)
HOME_R = (0.200, -0.280, HOVER_Z)

# Block grasping elevation (center of 40 mm block is TOP + 0.025, TCP is 13 mm higher)
GRASP_BLOCK_TCP_Z = BLOCK_INIT_POS[2] + PAD_TO_TCP   # TOP + 0.038 m
TRAY_PLACE_TCP_Z = TOP + TRAY_FLOOR_THICK + BLOCK_SIZE[2] / 2 + PAD_TO_TCP  # TOP + 0.038 m
PAD_PLACE_TCP_Z = TOP + PARK_PAD_SIZE[2] + BLOCK_SIZE[2] / 2 + PAD_TO_TCP   # TOP + 0.0355 m


def _box(centre, size, colour):
    return (tuple(float(v) for v in centre), tuple(float(v) for v in size), tuple(float(v) for v in colour))


def _cyl(centre, radius, height, colour, axis="Z"):
    return (tuple(float(v) for v in centre), float(radius), float(height), tuple(float(v) for v in colour), str(axis).upper())


def lane_world_boxes() -> list[tuple]:
    """Boxes forming the static lane fixture body in world coordinates."""
    boxes = [
        # Floor plate
        _box(LANE_FLOOR_POS, LANE_FLOOR_SIZE, STEEL_GREY),
        # Left wall (outer edge y = 0.175)
        _box((LANE_POS[0], LANE_Y_MAX - LANE_WALL_THICK / 2, LANE_FLOOR_TOP_Z + LANE_WALL_HEIGHT / 2),
             (LANE_SIZE_X, LANE_WALL_THICK, LANE_WALL_HEIGHT), GREY),
        # Right wall (outer edge y = 0.025)
        _box((LANE_POS[0], LANE_Y_MIN + LANE_WALL_THICK / 2, LANE_FLOOR_TOP_Z + LANE_WALL_HEIGHT / 2),
             (LANE_SIZE_X, LANE_WALL_THICK, LANE_WALL_HEIGHT), GREY),
        # Far end wall (x = 0.490)
        _box((LANE_X_MAX - LANE_WALL_THICK / 2, LANE_POS[1], LANE_FLOOR_TOP_Z + LANE_WALL_HEIGHT / 2),
             (LANE_WALL_THICK, LANE_SIZE_Y - 2 * LANE_WALL_THICK, LANE_WALL_HEIGHT), GREY),
        # Near end wall (x = 0.190)
        _box((LANE_X_MIN + LANE_WALL_THICK / 2, LANE_POS[1], LANE_FLOOR_TOP_Z + LANE_WALL_HEIGHT / 2),
             (LANE_WALL_THICK, LANE_SIZE_Y - 2 * LANE_WALL_THICK, LANE_WALL_HEIGHT), GREY),
    ]
    return boxes


def console_world_boxes() -> list[tuple]:
    """Boxes forming the static button console fixture body."""
    return [
        _box(CONSOLE_BASE_POS, CONSOLE_BASE_SIZE, DARK_GREY),
    ]


def console_world_cylinders() -> list[tuple]:
    """Cylinders forming base housings for the 3 lamps on the console."""
    return [
        _cyl(STOP_LAMP_POS, LAMP_RADIUS, LAMP_HEIGHT, LAMP_OFF, "Z"),
        _cyl(FAULT_LAMP_POS, LAMP_RADIUS, LAMP_HEIGHT, LAMP_OFF, "Z"),
        _cyl(RUN_LAMP_POS, LAMP_RADIUS, LAMP_HEIGHT, LAMP_OFF, "Z"),
    ]


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
        # Left wall
        _box((TRAY_POS[0], TRAY_POS[1] + hw_y - TRAY_WALL_THICK / 2, wall_z),
             (TRAY_SIZE[0], TRAY_WALL_THICK, wall_h), GREEN_TRAY_COL),
        # Right wall
        _box((TRAY_POS[0], TRAY_POS[1] - hw_y + TRAY_WALL_THICK / 2, wall_z),
             (TRAY_SIZE[0], TRAY_WALL_THICK, wall_h), GREEN_TRAY_COL),
        # Front wall
        _box((TRAY_POS[0] + hw_x - TRAY_WALL_THICK / 2, TRAY_POS[1], wall_z),
             (TRAY_WALL_THICK, TRAY_SIZE[1] - 2 * TRAY_WALL_THICK, wall_h), GREEN_TRAY_COL),
        # Rear wall
        _box((TRAY_POS[0] - hw_x + TRAY_WALL_THICK / 2, TRAY_POS[1], wall_z),
             (TRAY_WALL_THICK, TRAY_SIZE[1] - 2 * TRAY_WALL_THICK, wall_h), GREEN_TRAY_COL),
    ]
