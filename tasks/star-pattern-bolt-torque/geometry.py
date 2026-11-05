"""Every dimension of the star-pattern cold-plate bolt torque task in one place. Pure Python.

World frame (the station's): x forward from the arm bases, y left, z up, metres.
The left arm base is at (0, +0.305, 0.75), the right at (0, -0.305, 0.75).
Table top z = 0.7475 m (TOP).

The station contains:
- A heavy base fixture (200 x 260 x 15 mm, dark grey, mass 6.0 kg, friction 1.0) centered at (0.280, 0.0).
- A cold plate top (140 x 200 x 10 mm, aluminium, mass ~2.0 kg as part of the rigid fixture body).
- Four bolt heads modelled as spring buttons (36 x 36 x 12 mm white caps, 8 mm travel with hard end stop,
  100 N/m spring rate, 4 g mass):
  * L_near: (0.225, +0.105)
  * L_far:  (0.335, +0.105)
  * R_near: (0.225, -0.105)
  * R_far:  (0.335, -0.105)
  Corners spaced 110 mm in x and 210 mm in y.
  Diagonal pairs: (L_near, R_far) and (L_far, R_near).
- Beside each bolt, a column of three cyan lamp discs (diameter 28 mm, height 3 mm) indicates stages 1 to 3
  (lit from bottom / smaller x to top / larger x).
- A large status lamp (diameter 50 mm, height 15 mm) centered at (0.280, 0.0) above the plate:
  off / red (WARP) / green (TORQUED).
"""

from __future__ import annotations

import math
from typing import Mapping, Sequence

TOP = 0.7475  # table top height in metres

# Colours (sRGB 0..1)
DARK = (0.12, 0.12, 0.13)
DARK_GREY = (0.22, 0.22, 0.24)
GREY = (0.55, 0.56, 0.58)
LIGHT_GREY = (0.75, 0.76, 0.78)
STEEL = (0.70, 0.72, 0.75)
ALUMINIUM = (0.74, 0.76, 0.78)
WHITE = (0.92, 0.92, 0.92)
CYAN = (0.10, 0.85, 0.90)
LAMP_DARK_CYAN = (0.05, 0.22, 0.25)
LAMP_BRIGHT_CYAN = (0.10, 0.92, 0.98)
LAMP_OFF = (0.18, 0.18, 0.20)
LAMP_BRIGHT_RED = (0.90, 0.10, 0.10)
LAMP_BRIGHT_GREEN = (0.10, 0.95, 0.20)

# Fixture dimensions
FIXTURE_MASS = 6.0       # kg (spec: >= 5 kg)
FIXTURE_FRICTION = 1.0   # high friction so fixture does not slide
BASE_SIZE = (0.200, 0.260, 0.015)       # 200 mm along x, 260 mm along y, 15 mm thick
BASE_POS = (0.280, 0.0, TOP + BASE_SIZE[2] / 2)  # center (0.280, 0.0, TOP + 0.0075)

COLD_PLATE_SIZE = (0.140, 0.200, 0.010) # 140 mm along x, 200 mm along y, 10 mm thick
COLD_PLATE_POS = (0.280, 0.0, TOP + BASE_SIZE[2] + COLD_PLATE_SIZE[2] / 2)  # (0.280, 0.0, TOP + 0.020)
COLD_PLATE_TOP_Z = TOP + BASE_SIZE[2] + COLD_PLATE_SIZE[2]  # TOP + 0.025 m

# Bolt corner coordinates in world frame (x, y)
# Spacing: 110 mm in x (x_near=0.225, x_far=0.335), 210 mm in y (y_left=+0.105, y_right=-0.105)
BOLT_X_NEAR = 0.225
BOLT_X_FAR = 0.335
BOLT_Y_LEFT = 0.105
BOLT_Y_RIGHT = -0.105

CORNERS = ("L_near", "L_far", "R_near", "R_far")

BOLT_POS_XY: dict[str, tuple[float, float]] = {
    "L_near": (BOLT_X_NEAR, BOLT_Y_LEFT),
    "L_far":  (BOLT_X_FAR,  BOLT_Y_LEFT),
    "R_near": (BOLT_X_NEAR, BOLT_Y_RIGHT),
    "R_far":  (BOLT_X_FAR,  BOLT_Y_RIGHT),
}

DIAGONAL_PARTNER: dict[str, str] = {
    "L_near": "R_far",
    "R_far":  "L_near",
    "L_far":  "R_near",
    "R_near": "L_far",
}

# Arm assignment: left arm operates left bolts only, right arm operates right bolts only
BOLT_ARM: dict[str, str] = {
    "L_near": "left",
    "L_far":  "left",
    "R_near": "right",
    "R_far":  "right",
}

# Button cap and slider specifications
# Arm PD vertical compliance is ~1.8-3.1 mm/N. Soft 100 N/m spring bottoms onto 8 mm hard stop at 0.8 N.
BUTTON_CAP_SIZE = (0.036, 0.036, 0.012)
BUTTON_TRAVEL = 0.008        # 8 mm travel to end stop
BUTTON_RATE = 100.0          # N/m
BUTTON_DAMPING = 4.0         # N s/m
BUTTON_MASS = 0.004          # 4 g (prevents resting sag on 100 N/m spring: m*g/k = 0.39 mm)

# Cap elevation: at full 8 mm travel, cap bottom rests flush on cold plate top (COLD_PLATE_TOP_Z)
# At rest: cap bottom is 8 mm above cold plate top
BUTTON_REST_BOTTOM_Z = COLD_PLATE_TOP_Z + BUTTON_TRAVEL
BUTTON_REST_Z = BUTTON_REST_BOTTOM_Z + BUTTON_CAP_SIZE[2] / 2  # center of cap at rest: TOP + 0.039 m
BUTTON_REST_TOP_Z = BUTTON_REST_BOTTOM_Z + BUTTON_CAP_SIZE[2] # top of cap at rest: TOP + 0.045 m

# Press thresholds for hidden state logic
BUTTON_PRESS_THRESHOLD = -0.0064   # 80 % of 8 mm travel (-6.4 mm)
BUTTON_RELEASE_THRESHOLD = -0.0035 # re-arm threshold (-3.5 mm)

# Commanded press depth: fingertip commanded 14 mm below rest cap top (6 mm past the 8 mm stop)
# Lowest fingertip point is PAD_TIP_PAST_TCP (13 mm) below TCP
BUTTON_PRESS_DEPTH_CMD = 0.014
PAD_TIP_PAST_TCP = 0.013
BUTTON_PRESS_TCP_Z = BUTTON_REST_TOP_Z - BUTTON_PRESS_DEPTH_CMD + PAD_TIP_PAST_TCP  # TOP + 0.044 m

# Bolt button slider rest positions (world frame)
BOLT_REST_POS: dict[str, tuple[float, float, float]] = {
    c: (BOLT_POS_XY[c][0], BOLT_POS_XY[c][1], BUTTON_REST_Z) for c in CORNERS
}

# Cyan stage indicator lamps
# Beside each bolt, a column of 3 lamp discs (diameter 28 mm, height 3 mm)
# Inboard in y at y = ±0.055 m (50 mm from bolt center, 21 mm edge clearance to closed fist gripper)
STAGE_LAMP_R = 0.014  # radius 14 mm (diameter 28 mm)
STAGE_LAMP_H = 0.003  # height 3 mm
STAGE_LAMP_PITCH_X = 0.032  # 32 mm pitch along x
STAGE_LAMP_Y_LEFT = 0.055
STAGE_LAMP_Y_RIGHT = -0.055

# World x for stages 1, 2, 3:
# Near column: stage 1 at x = 0.193, stage 2 at x = 0.225, stage 3 at x = 0.257
# Far column:  stage 1 at x = 0.303, stage 2 at x = 0.335, stage 3 at x = 0.367
def _stage_lamp_x(corner: str, stage: int) -> float:
    base_x = BOLT_POS_XY[corner][0]
    return base_x + (stage - 2) * STAGE_LAMP_PITCH_X

def _stage_lamp_y(corner: str) -> float:
    return STAGE_LAMP_Y_LEFT if corner.startswith("L") else STAGE_LAMP_Y_RIGHT

STAGE_LAMP_Z = COLD_PLATE_TOP_Z + STAGE_LAMP_H / 2

STAGE_LAMP_POS: dict[tuple[str, int], tuple[float, float, float]] = {
    (c, s): (_stage_lamp_x(c, s), _stage_lamp_y(c), STAGE_LAMP_Z)
    for c in CORNERS
    for s in (1, 2, 3)
}

# Lit stage lamp marker discs (kinematic non-colliding discs teleported over base lamps)
STAGE_LAMP_LIT_R = STAGE_LAMP_R + 0.001  # 15 mm radius
STAGE_LAMP_LIT_H = STAGE_LAMP_H + 0.001  # 4 mm height
STAGE_LAMP_LIT_Z = STAGE_LAMP_Z + 0.0005

# Status lamp (diameter 50 mm, height 15 mm) centered at (0.280, 0.0)
STATUS_LAMP_R = 0.025  # radius 25 mm (diameter 50 mm >= 50 mm)
STATUS_LAMP_H = 0.015  # height 15 mm
STATUS_LAMP_POS = (0.280, 0.0, COLD_PLATE_TOP_Z + STATUS_LAMP_H / 2)  # center (0.280, 0.0, TOP + 0.0325)

STATUS_LAMP_LIT_R = STATUS_LAMP_R + 0.001  # 26 mm radius
STATUS_LAMP_LIT_H = STATUS_LAMP_H + 0.002  # 17 mm height
STATUS_LAMP_LIT_POS = (STATUS_LAMP_POS[0], STATUS_LAMP_POS[1], STATUS_LAMP_POS[2] + 0.001)

# Visual indicator stowing position (under table)
STOWED_Z = TOP - 0.100

# Arm park / hover heights
HOVER_Z = TOP + 0.120
HOME_L = (0.180, 0.220, HOVER_Z)
HOME_R = (0.180, -0.220, HOVER_Z)


def _box(centre, size, colour):
    return (tuple(float(v) for v in centre), tuple(float(v) for v in size), tuple(float(v) for v in colour))


def _cyl(centre, radius, height, colour, axis="Z"):
    return (tuple(float(v) for v in centre), float(radius), float(height), tuple(float(v) for v in colour), str(axis).upper())


def fixture_boxes() -> list[tuple]:
    """Boxes forming the static fixture body (base plate + cold plate)."""
    return [
        # Heavy base plate (dark grey)
        _box(BASE_POS, BASE_SIZE, DARK_GREY),
        # Cold plate top (aluminium)
        _box(COLD_PLATE_POS, COLD_PLATE_SIZE, ALUMINIUM),
    ]


def fixture_cylinders() -> list[tuple]:
    """Cylinders forming base lamps and status lamp housing."""
    cyls = []
    # Status lamp base cylinder (dark when off)
    cyls.append(_cyl(STATUS_LAMP_POS, STATUS_LAMP_R, STATUS_LAMP_H, LAMP_OFF, "Z"))
    # 12 base stage lamp discs (dark when off)
    for pos in STAGE_LAMP_POS.values():
        cyls.append(_cyl(pos, STAGE_LAMP_R, STAGE_LAMP_H, LAMP_OFF, "Z"))
    return cyls
