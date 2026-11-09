"""Geometry and physical station layout for compress-and-lock-spring-cartridge. Pure Python.

World frame: x forward from the arm bases, y left, z up, metres.
Arm bases: Left (0, +0.305, 0.75), Right (0, -0.305, 0.75).
Table top z = 0.7475 m (TOP).

Station contents:
- Heavy fixture base plate: 240 x 460 x 20 mm, mass 8.0 kg, friction 1.0,
  centered at (0.340, +0.040, TOP + 0.010 = 0.7575).
- Plunger slider at (0.340, +0.180):
  * Axis: +X (pushed toward +x, deeper into cartridge).
  * Travel: 45 mm (0.045 m).
  * Return spring rate: 50 N/m (2.25 N max at 45 mm, <= 2.5 N per lesson 10).
  * Knob: 45 wide (y) x 30 deep (x) x 35 tall (z) mm, orange.
  * Stop box & notch: moves with plunger; blocks lock bar until compressed >= 38 mm.
- Lock bar slider at (0.340, -0.100):
  * Axis: +Y (slides toward plunger in +y direction).
  * Travel: 70 mm (0.070 m).
  * Friction hold anywhere (spring rate 0.01 N/m, damping 15 N s/m).
  * Handle: 45 mm wide (x) x 30 deep (y) x 35 tall (z) mm, steel blue.
  * Tongue: extends toward plunger; stops at 15 mm if plunger uncompressed; passes through notch when compressed >= 38 mm.
- Status lamp: 25 mm green disc marker atop cartridge housing, lit green when locked and plunger compressed.
"""

from __future__ import annotations

import math
import numpy as np

TOP = 0.7475  # table top elevation in metres

# Colours (sRGB)
DARK_GREY = (0.22, 0.22, 0.24)
BASE_GREY = (0.35, 0.35, 0.38)
CARTRIDGE_GREY = (0.45, 0.45, 0.48)
PLUNGER_ORANGE = (0.90, 0.45, 0.10)
LOCK_BAR_BLUE = (0.25, 0.50, 0.85)
LAMP_GREEN = (0.10, 0.95, 0.20)
LAMP_OFF = (0.20, 0.20, 0.20)
STEEL = (0.70, 0.72, 0.75)

# Fixture Base Plate
BASE_SIZE = (0.300, 0.460, 0.020)  # 300 x 460 x 20 mm (covers x: 0.220 to 0.520)
BASE_POS = (0.370, +0.040, TOP + BASE_SIZE[2] / 2)  # (0.370, +0.040, 0.7575)
FIXTURE_TOP_Z = TOP + BASE_SIZE[2]                  # 0.7675 m
FIXTURE_MASS = 8.0                                  # kg
FIXTURE_FRICTION = 1.0

# Plunger Slider
PLUNGER_KNOB_SIZE = (0.030, 0.045, 0.035)  # 30 deep (x) x 45 wide (y) x 35 tall (z)
PLUNGER_TRAVEL = 0.045                     # 45 mm travel along +X
PLUNGER_SPRING_K = 50.0                    # 50 N/m -> 2.25 N max at full stroke
PLUNGER_DAMPING = 5.0                      # N s/m
PLUNGER_MASS = 0.050                       # 50 g
PLUNGER_AXIS = "+X"

PLUNGER_REST_X = 0.340
PLUNGER_REST_Y = +0.180
PLUNGER_REST_Z = FIXTURE_TOP_Z + 0.005 + PLUNGER_KNOB_SIZE[2] / 2  # 0.790 m
PLUNGER_REST_POS = (PLUNGER_REST_X, PLUNGER_REST_Y, PLUNGER_REST_Z)
PLUNGER_LOCAL_REST = (
    PLUNGER_REST_POS[0] - BASE_POS[0],
    PLUNGER_REST_POS[1] - BASE_POS[1],
    PLUNGER_REST_POS[2] - BASE_POS[2],
)

PLUNGER_SHAFT_SIZE = (0.125, 0.025, 0.025)
PLUNGER_STOP_BOX_SIZE = (0.020, 0.020, 0.025)

# Lock Bar Slider
LOCK_BAR_HANDLE_SIZE = (0.045, 0.030, 0.035)  # 45 wide (x) x 30 deep (y) x 35 tall (z)
LOCK_BAR_TRAVEL = 0.070                       # 70 mm travel along +Y
LOCK_BAR_RATE = 0.01                          # friction hold anywhere (near-zero spring)
LOCK_BAR_DAMPING = 15.0                       # viscous damping for smooth friction feel
LOCK_BAR_MASS = 0.050                         # 50 g
LOCK_BAR_AXIS = "+Y"

LOCK_BAR_REST_X = 0.340
LOCK_BAR_REST_Y = -0.100
LOCK_BAR_REST_Z = FIXTURE_TOP_Z + 0.005 + LOCK_BAR_HANDLE_SIZE[2] / 2  # 0.790 m
LOCK_BAR_REST_POS = (LOCK_BAR_REST_X, LOCK_BAR_REST_Y, LOCK_BAR_REST_Z)
LOCK_BAR_LOCAL_REST = (
    LOCK_BAR_REST_POS[0] - BASE_POS[0],
    LOCK_BAR_REST_POS[1] - BASE_POS[1],
    LOCK_BAR_REST_POS[2] - BASE_POS[2],
)

LOCK_BAR_TONGUE_SIZE = (0.020, 0.200, 0.020)

# Cartridge Housing on Fixture Body (moved back to x_min = 0.450 m to leave >= 75 mm clear stroke & hand clearance)
CARTRIDGE_HOUSING_SIZE = (0.070, 0.070, 0.045)
CARTRIDGE_HOUSING_POS = (0.485, +0.180, FIXTURE_TOP_Z + CARTRIDGE_HOUSING_SIZE[2] / 2)
CARTRIDGE_TOP_Z = FIXTURE_TOP_Z + CARTRIDGE_HOUSING_SIZE[2]

# Status Lamp (25 mm green marker atop cartridge housing)
LAMP_SIZE = (0.025, 0.025, 0.004)  # 25 mm diameter disc
LAMP_POS = (0.485, +0.180, CARTRIDGE_TOP_Z + LAMP_SIZE[2] / 2)
LAMP_ON_Z = LAMP_POS[2]
LAMP_OFF_Z = LAMP_POS[2] - 0.050

# Thresholds & Rules (from section 5.2 spec)
PLUNGER_COMPRESS_THRESHOLD = 0.038    # 38 mm required to unlock & score 0.25
BAR_LOCKED_THRESHOLD = 0.055          # 55 mm bar across required for LOCKED
BAR_STOP_EARLY_LIMIT = 0.015          # 15 mm maximum bar travel when plunger < 38 mm
PLUNGER_RETAINED_HOLD_Z = PLUNGER_REST_Z
PLUNGER_RETAINED_MIN_DISP = 0.038     # plunger stays >= 38 mm when held by locked bar

# Arm reach & geometry
LEFT_BASE = (0.0, +0.305, 0.75)
RIGHT_BASE = (0.0, -0.305, 0.75)
MAX_ARM_REACH = 0.50
MIN_ARM_REACH = 0.10

HOVER_Z = TOP + 0.120                 # 0.8675 m
PAD_TO_TCP = 0.013                    # distance from fingertip to tool center point

# Demostration Waypoints & Tuning Parameters
DEMO_TIME_SCALE = 1.8                 # demo time stretch factor
PLUNGER_PUSH_OVERSHOOT = 0.018        # 18 mm command overshoot to compensate for arm PD compliance

# Clearance margins
MIN_ARM_ARM_CLEARANCE = 0.030         # 30 mm minimum arm-arm clearance
CLEAR_APPROACH_DISTANCE = 0.025       # 25 mm clear approach before handle contact


def fixture_world_boxes() -> list[tuple]:
    """Shapes of the rigid base fixture in world coordinates (body frame origin at BASE_POS)."""
    return [
        # Base plate
        (BASE_POS, BASE_SIZE, BASE_GREY),
        # Cartridge sleeve / guide housing at plunger
        (CARTRIDGE_HOUSING_POS, CARTRIDGE_HOUSING_SIZE, CARTRIDGE_GREY),
        # Lock bar guide block along y
        ((0.340, +0.020, FIXTURE_TOP_Z + 0.015), (0.050, 0.100, 0.030), CARTRIDGE_GREY),
    ]


def plunger_slider_world_boxes() -> list[tuple]:
    """Shapes of the plunger slider at rest in world coordinates."""
    knob_c = PLUNGER_REST_POS
    shaft_c = (PLUNGER_REST_X + PLUNGER_KNOB_SIZE[0] / 2 + PLUNGER_SHAFT_SIZE[0] / 2, PLUNGER_REST_Y, PLUNGER_REST_Z)
    stop_box_c = (PLUNGER_REST_X + 0.0025, PLUNGER_REST_Y - 0.020, PLUNGER_REST_Z)
    return [
        (knob_c, PLUNGER_KNOB_SIZE, PLUNGER_ORANGE),
        (shaft_c, PLUNGER_SHAFT_SIZE, STEEL),
        (stop_box_c, PLUNGER_STOP_BOX_SIZE, STEEL),
    ]


def lock_bar_slider_world_boxes() -> list[tuple]:
    """Shapes of the lock bar slider at rest in world coordinates."""
    handle_c = LOCK_BAR_REST_POS
    tongue_c = (LOCK_BAR_REST_X, LOCK_BAR_REST_Y + LOCK_BAR_HANDLE_SIZE[1] / 2 + LOCK_BAR_TONGUE_SIZE[1] / 2, LOCK_BAR_REST_Z)
    return [
        (handle_c, LOCK_BAR_HANDLE_SIZE, LOCK_BAR_BLUE),
        (tongue_c, LOCK_BAR_TONGUE_SIZE, LOCK_BAR_BLUE),
    ]
