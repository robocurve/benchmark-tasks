"""Geometry and station layout for lockout-tagout-before-service. Pure Python.

World frame: x forward, y left, z up, metres.
Arm bases: Left (0, +0.305, 0.75), Right (0, -0.305, 0.75).
Table top z = 0.7475 m (TOP).

Contents:
- Safety Console Base Plate: 240 x 540 x 20 mm dark grey plate at (0.300, +0.050).
- Breaker Panel (left side, y = +0.180):
  Lever slider: 40 mm throw along X. Starts ON (x = 0.280), pushed OFF (x = 0.320).
  18 mm lock pin hole at (0.280, +0.180) (uncovered only when lever is OFF).
- Red Lock Pin:
  Cylinder 16 mm diameter x 50 mm tall. Starts on pin stand at (0.220, +0.280).
  Seated when bottom within 4 mm of hole bottom at (0.280, +0.180).
- TEST Station (center, y = 0.000):
  White pushbutton (40 x 40 x 12 mm, 8 mm travel to stop, 100 N/m spring).
  30 mm diameter lamps at x = 0.360:
    LIVE lamp (amber): lit while breaker is ON.
    DEAD lamp (green): lights for 3.0 s on TEST press if breaker is OFF.
- Service Enclosure (right side, y = -0.080):
  Sliding cover with 45 mm handle, 90 mm travel along -X with 18 N/m return spring.
  Blue SERVICE button (40 x 40 x 12 mm, 8 mm travel) underneath at (0.300, -0.080),
  reachable only when cover is out >= 70 mm.
"""

from __future__ import annotations

TOP = 0.7475

# Colours (sRGB)
DARK_GREY = (0.22, 0.22, 0.24)
PANEL_GREY = (0.35, 0.35, 0.38)
LEVER_ORANGE = (0.90, 0.45, 0.10)
PIN_RED = (0.85, 0.15, 0.15)
BUTTON_WHITE = (0.90, 0.90, 0.90)
BUTTON_BLUE = (0.15, 0.35, 0.85)
LAMP_AMBER = (0.95, 0.65, 0.10)
LAMP_GREEN = (0.15, 0.85, 0.25)
LAMP_RED = (0.85, 0.15, 0.15)
COVER_STEEL = (0.55, 0.55, 0.58)

# Base fixture
BASE_SIZE = (0.240, 0.540, 0.020)
BASE_POS = (0.300, +0.060, TOP + BASE_SIZE[2] / 2)
BASE_TOP_Z = TOP + BASE_SIZE[2]

# Breaker lever slider
LEVER_START_X = 0.280       # ON position (forward toward robot)
LEVER_OFF_X = 0.320         # OFF position (pushed away)
LEVER_THROW = 0.040         # 40 mm throw
LEVER_Y = +0.180
LEVER_HANDLE_SIZE = (0.025, 0.025, 0.030)
LEVER_Z = BASE_TOP_Z + LEVER_HANDLE_SIZE[2] / 2 + 0.001

# Lock pin and hole
PIN_RADIUS = 0.008          # 16 mm diameter
PIN_HEIGHT = 0.050          # 50 mm tall
PIN_SIZE = (0.016, 0.016, 0.050)
PIN_STAND_POS = (0.220, +0.280, BASE_TOP_Z + PIN_HEIGHT / 2)
HOLE_POS = (0.280, +0.180, BASE_TOP_Z)
PIN_SEATED_Z = BASE_TOP_Z + PIN_HEIGHT / 2 - 0.015  # inserted 15 mm into hole
HOLE_SIZE = 0.028           # square through-hole in the base plate, 6 mm clearance per side around the pin (arm placement error)
PIN_GRASP_DZ = 0.012        # grip the pin above its centre so the pads stay clear of the plate while it is inserted

# TEST button and lamps
TEST_BTN_SIZE = (0.040, 0.040, 0.012)
TEST_BTN_TRAVEL = 0.008
TEST_BTN_POS = (0.300, +0.035, BASE_TOP_Z + TEST_BTN_TRAVEL + TEST_BTN_SIZE[2] / 2)  # cap bottom one stroke above the plate

LAMP_DIAMETER = 0.030
LAMP_SIZE = (LAMP_DIAMETER, LAMP_DIAMETER, 0.004)
LAMP_LIVE_POS = (0.360, +0.020, BASE_TOP_Z + LAMP_SIZE[2] / 2)
LAMP_DEAD_POS = (0.360, +0.050, BASE_TOP_Z + LAMP_SIZE[2] / 2)
LAMP_ON_Z = BASE_TOP_Z + LAMP_SIZE[2] / 2
LAMP_OFF_Z = BASE_TOP_Z - 0.050

# Service sliding cover and SERVICE button
COVER_REST_X = 0.300
COVER_Y = -0.040
COVER_SIZE = (0.120, 0.080, 0.015)
COVER_Z = BASE_TOP_Z + 0.024 + COVER_SIZE[2] / 2   # cover floats 24 mm above the plate, clear of the button cap
COVER_TRAVEL = 0.090        # 90 mm travel toward robot (-X)
COVER_SPRING_K = 18.0       # 18 N/m
COVER_HANDLE_SIZE = (0.020, 0.045, 0.030)
COVER_HANDLE_X = COVER_REST_X - COVER_SIZE[0] / 2 - COVER_HANDLE_SIZE[0] / 2   # on the front edge, sticks out toward the robot
COVER_HANDLE_Z = BASE_TOP_Z + 0.024 + COVER_HANDLE_SIZE[2] / 2

SERVICE_BTN_SIZE = (0.040, 0.040, 0.012)
SERVICE_BTN_TRAVEL = 0.008
SERVICE_BTN_POS = (0.300, -0.040, BASE_TOP_Z + SERVICE_BTN_TRAVEL + SERVICE_BTN_SIZE[2] / 2)

# Thresholds
BREAKER_OFF_THRESHOLD = 0.032       # throw >= 32 mm counts as OFF
PIN_SEATED_THRESHOLD_M = 0.004      # bottom within 4 mm of hole
COVER_OPEN_MIN = 0.070              # >= 70 mm open to reach service button
COVER_FAULT_OPEN = 0.020            # opening >= 20 mm before prerequisites = FAULT

# Navigation and Grasp Heights
HOVER_Z = TOP + 0.110               # ~0.8575 m
GRASP_PIN_Z = PIN_STAND_POS[2]
PRESS_Z = BASE_TOP_Z + TEST_BTN_TRAVEL + TEST_BTN_SIZE[2] - 0.014 + 0.013   # fingertip 14 mm below the rest cap top (PAD_TO_TCP = 0.013)

PAD_TO_TCP = 0.013
MAX_ARM_REACH = 0.50
LEFT_BASE = (0.0, +0.305)
RIGHT_BASE = (0.0, -0.305)


def base_world_boxes() -> list[tuple]:
    """Base plate as four boxes around the square lock-pin hole (world frame), so the pin can really drop in."""
    x0, x1 = BASE_POS[0] - BASE_SIZE[0] / 2, BASE_POS[0] + BASE_SIZE[0] / 2
    y0, y1 = BASE_POS[1] - BASE_SIZE[1] / 2, BASE_POS[1] + BASE_SIZE[1] / 2
    hx0, hx1 = HOLE_POS[0] - HOLE_SIZE / 2, HOLE_POS[0] + HOLE_SIZE / 2
    hy0, hy1 = HOLE_POS[1] - HOLE_SIZE / 2, HOLE_POS[1] + HOLE_SIZE / 2
    z = BASE_POS[2]
    t = BASE_SIZE[2]

    def box(xa, xb, ya, yb):
        return (((xa + xb) / 2, (ya + yb) / 2, z), (xb - xa, yb - ya, t), DARK_GREY)

    return [box(x0, hx0, y0, y1), box(hx1, x1, y0, y1), box(hx0, hx1, y0, hy0), box(hx0, hx1, hy1, y1)]
