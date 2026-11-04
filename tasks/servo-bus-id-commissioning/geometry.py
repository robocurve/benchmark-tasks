"""Every dimension of the servo bus ID commissioning task in one place. Pure Python.

World frame (the station's): x forward from the arm bases, y left, z up, metres.
The left arm base is at (0, +0.305), the right at (0, -0.305).
Table top z = 0.7475 m (TOP).

The station contains:
- A fixed bus strip with three keyed sockets in a row (pitch 70 mm), mouths facing -x.
  Each socket has a spring plunger detent on its +y wall (the groove of the seated nose is on the same side).
- A fixed controller box with ASSIGN (white) and RESET (yellow) pushbuttons,
  a bus status lamp (red/green, 30 mm dome), and a 3x3 readout board (20 mm lamps).
- Three servo modules (red, green, blue), each built with the keyed plug geometry:
  housing 45 x 18 x 28 mm, steel nose 28 x 16 x 18 mm with key rib and detent groove.
  All three servos start plugged into the bus strip in order: blue (socket 0), red (socket 1), green (socket 2).
"""

from __future__ import annotations

import math
import numpy as np

TOP = 0.7475  # table top height

# Colours (sRGB 0..1)
DARK = (0.12, 0.12, 0.13)
DARK_GREY = (0.22, 0.22, 0.24)
GREY = (0.55, 0.56, 0.58)
LIGHT_GREY = (0.75, 0.76, 0.78)
STEEL = (0.78, 0.79, 0.80)
ALUMINIUM = (0.74, 0.76, 0.78)
RED = (0.85, 0.15, 0.12)
GREEN = (0.12, 0.75, 0.18)
BLUE = (0.15, 0.35, 0.85)
YELLOW = (0.95, 0.85, 0.10)
WHITE = (0.92, 0.92, 0.92)
LAMP_DARK_RED = (0.35, 0.05, 0.05)
LAMP_BRIGHT_RED = (0.90, 0.10, 0.10)
LAMP_BRIGHT_GREEN = (0.10, 0.95, 0.20)
LAMP_OFF = (0.18, 0.18, 0.20)

CLEAR = 0.001  # 1.0 mm clearance per side between nose and socket cavity

# -- Servo module (plug) dimensions (identical to reference task) -----------------------------------
NOSE_W = 0.028   # x in plug frame
NOSE_H = 0.016   # z in plug frame
NOSE_L = 0.018   # y in plug frame (protrusion from housing)
HOUSING = (0.045, 0.018, 0.028)  # x (width), y (depth), z (height)
KEY_W = 0.006
KEY_H = 0.003
KEY_X = 0.001
GROOVE = (0.0075, 0.0125)
GROOVE_DEPTH = 0.0015
SERVO_MASS = 0.080  # 80 g
SERVO_FRICTION = 0.8

# Plug orientation when plugged in: yaw = -pi/2 so nose points along world +x
PLUG_YAW_SEATED = -math.pi / 2

# -- Socket dimensions (fixture frame) -------------------------------------------------------------
FLANGE_W = 0.056
FLANGE_H = 0.042
FLANGE_T = 0.006
SHELL_HX = 0.019  # half-width of shell box
SHELL_HZ = 0.015  # half-height of shell box
SHELL_L = 0.024   # cavity depth

CAV_HX = NOSE_W / 2 + CLEAR  # 0.015
CAV_HZ = NOSE_H / 2 + CLEAR  # 0.009
SLOT_X0 = KEY_X - KEY_W / 2 - CLEAR
SLOT_X1 = KEY_X + KEY_W / 2 + CLEAR
SLOT_Z1 = NOSE_H / 2 + KEY_H + CLEAR

# Plunger specifications
PLUNGER_S = 0.010
PLUNGER_R = 0.002
PLUNGER_H = 0.006
PLUNGER_PROUD = CLEAR + 0.0005
PLUNGER_TRAVEL = 0.0015
PLUNGER_RATE = 12000.0  # N/m
PLUNGER_DAMPING = 40.0
PLUNGER_MASS = 0.020    # 20 g

# -- Fixed fixture layout (world frame) ------------------------------------------------------------
FIXTURE_MASS = 8.0  # heavy base fixture
FIXTURE_FRICTION = 1.0

# Three sockets along y: pitch 70 mm
SOCKET_PITCH = 0.070
SOCKET_Y = (-0.050, -0.120, -0.190)  # socket 0 (blue), 1 (red), 2 (green)
MOUTH_X = 0.240  # world x coordinate of socket mouth planes
SOCKET_Z = TOP + 0.025  # height of socket axes (25 mm above table)

# Servo starting poses when fully seated in sockets
START_POS = {
    "blue": (MOUTH_X, SOCKET_Y[0], SOCKET_Z),
    "red": (MOUTH_X, SOCKET_Y[1], SOCKET_Z),
    "green": (MOUTH_X, SOCKET_Y[2], SOCKET_Z),
}

# Staging positions on table when unplugged (pulled back along -x by 70 mm)
STAGE_X = 0.170
STAGE_Z = TOP + HOUSING[2] / 2
STAGE_POS = {
    "blue": (STAGE_X, SOCKET_Y[0], STAGE_Z),
    "red": (STAGE_X, SOCKET_Y[1], STAGE_Z),
    "green": (STAGE_X, SOCKET_Y[2], STAGE_Z),
}

# Controller box
CTRL_POS = (0.260, 0.100, TOP + 0.020)  # centre of controller box
CTRL_SIZE = (0.140, 0.080, 0.040)       # 140 mm along x, 80 mm along y, 40 mm tall

# Buttons on top of controller box (top of box is TOP + 0.040, caps raised 8 mm so full travel bottoms flush with box)
# The cap sits on a SOFT spring (100 N/m): the arm's joints are soft PD controllers (about 1.8 mm/N at the press pose
# for the left arm, 3.1 mm/N for the right arm; see tests/test_button_press.py), so a stiff cap spring would barely
# move. At 100 N/m the 8 mm end stop is reached with 0.8 N, and the press is commanded deeper than the stop so the
# STOP, not the arm compliance, sets the travel.
BUTTON_TRAVEL = 0.008        # hard stop (joint limit): cap bottom is then flush with the box top
BUTTON_RATE = 100.0          # N/m (was 625 N/m)
BUTTON_DAMPING = 4.0
BUTTON_MASS = 0.004          # 4 g (was 20 g): a heavier cap would sag m*g/k = 2 mm on the soft spring and look pressed
BUTTON_CAP_SIZE = (0.036, 0.036, 0.012)
BUTTON_REST_Z = TOP + 0.040 + 0.008 + BUTTON_CAP_SIZE[2] / 2
BUTTON_REST_TOP_Z = TOP + 0.060
BUTTON_PRESS_THRESHOLD = 0.0064   # logic: 80 % of travel
BUTTON_PRESS_DEPTH_CMD = 0.014    # commanded fingertip depth below the REST cap top (6 mm beyond the 8 mm stop)
PAD_TIP_PAST_TCP = 0.013          # lowest fingertip point below the TCP (yam_arena constants)
BUTTON_PRESS_TCP_Z = BUTTON_REST_TOP_Z - BUTTON_PRESS_DEPTH_CMD + PAD_TIP_PAST_TCP
ASSIGN_BTN_POS = (0.215, 0.085, BUTTON_REST_Z)
RESET_BTN_POS = (0.285, 0.085, BUTTON_REST_Z)

# Status lamp (30 mm diameter, 15 mm height, placed on controller box)
STATUS_LAMP_R = 0.015
STATUS_LAMP_H = 0.015
STATUS_LAMP_POS = (0.250, 0.120, TOP + 0.040 + STATUS_LAMP_H / 2)
STATUS_LAMP_LIT_R = STATUS_LAMP_R + 0.001
STATUS_LAMP_LIT_H = STATUS_LAMP_H + 0.002
STATUS_LAMP_LIT_POS = (STATUS_LAMP_POS[0], STATUS_LAMP_POS[1], STATUS_LAMP_POS[2] + 0.001)

# Readout board (120 x 80 x 6 mm) located beside controller box at y=0.210
READOUT_SIZE = (0.120, 0.080, 0.006)
READOUT_POS = (0.260, 0.210, TOP + READOUT_SIZE[2] / 2)
READOUT_PITCH = 0.030
READOUT_LAMP_R = 0.010
READOUT_LAMP_H = 0.004

# Readout grid lamp centres in world frame:
# Rows: red (x=0.230), green (x=0.260), blue (x=0.290)
# Cols: 1 (y=0.240), 2 (y=0.210), 3 (y=0.180) - 1, 2, 3 left to right as seen looking forward (+x)
READOUT_ROWS = {"red": 0.230, "green": 0.260, "blue": 0.290}
READOUT_COLS = {1: 0.240, 2: 0.210, 3: 0.180}
READOUT_LAMP_Z = TOP + READOUT_SIZE[2] + READOUT_LAMP_H / 2
READOUT_LAMP_LIT_R = READOUT_LAMP_R + 0.001
READOUT_LAMP_LIT_H = READOUT_LAMP_H + 0.002
READOUT_LAMP_LIT_Z = READOUT_LAMP_Z + 0.001

READOUT_MARK_X = READOUT_POS[0] + READOUT_SIZE[0] / 2 - 0.007  # x of the column dots (far edge of the board)

# Visual indicator stowing position (under table)
STOWED_Z = TOP - 0.100


def _box(centre, size, colour):
    return (tuple(float(v) for v in centre), tuple(float(v) for v in size), tuple(float(v) for v in colour))


def _cyl(centre, radius, height, colour, axis="Z"):
    return (tuple(float(v) for v in centre), float(radius), float(height), tuple(float(v) for v in colour), str(axis).upper())


def servo_boxes(colour):
    """The servo module shapes in the plug frame (origin at nose root, nose along +y)."""
    g0, g1 = GROOVE
    hx = NOSE_W / 2
    # Housing body: dark ends, thick coloured middle band
    band_x = 0.024
    side_x = (HOUSING[0] - band_x) / 2
    return [
        # Coloured central band of housing
        _box((0.0, -HOUSING[1] / 2, 0.0), (band_x, HOUSING[1], HOUSING[2]), colour),
        # Dark housing flanks
        _box((-band_x / 2 - side_x / 2, -HOUSING[1] / 2, 0.0), (side_x, HOUSING[1], HOUSING[2]), DARK),
        _box((band_x / 2 + side_x / 2, -HOUSING[1] / 2, 0.0), (side_x, HOUSING[1], HOUSING[2]), DARK),
        # Steel nose pieces
        _box((0.0, g0 / 2, 0.0), (NOSE_W, g0, NOSE_H), STEEL),
        _box((GROOVE_DEPTH / 2, (g0 + g1) / 2, 0.0), (NOSE_W - GROOVE_DEPTH, g1 - g0, NOSE_H), STEEL),
        _box((0.0, (g1 + NOSE_L) / 2, 0.0), (NOSE_W, NOSE_L - g1, NOSE_H), STEEL),
        # Key rib
        _box((KEY_X, NOSE_L / 2, NOSE_H / 2 + KEY_H / 2), (KEY_W, NOSE_L, KEY_H), colour),
    ]


def fixture_boxes():
    """All boxes for the static bus strip, sockets, controller box and readout base."""
    boxes = []
    # 1. Bus strip main beam (behind sockets)
    strip_w = 0.050
    strip_l = 0.220
    strip_h = 0.050
    strip_cx = MOUTH_X + SHELL_L + FLANGE_T + strip_w / 2
    strip_cy = (SOCKET_Y[0] + SOCKET_Y[2]) / 2  # -0.120
    boxes.append(_box((strip_cx, strip_cy, TOP + strip_h / 2), (strip_w, strip_l, strip_h), DARK_GREY))

    # 2. Sockets (one per socket slot)
    for sy in SOCKET_Y:
        flange_x = MOUTH_X + SHELL_L + FLANGE_T / 2
        boxes.append(_box((flange_x, sy, SOCKET_Z), (FLANGE_T, FLANGE_W, FLANGE_H), ALUMINIUM))

        # Shell walls around cavity
        # Cavity opens along -x from MOUTH_X to MOUTH_X + SHELL_L
        shell_cx = MOUTH_X + SHELL_L / 2
        # Right wall (+y in world, which is -x in socket frame)
        boxes.append(_box((shell_cx, sy + CAV_HX + (SHELL_HX - CAV_HX) / 2, SOCKET_Z), (SHELL_L, SHELL_HX - CAV_HX, 2 * SHELL_HZ), DARK))
        # Left wall (-y in world)
        boxes.append(_box((shell_cx, sy - CAV_HX - (SHELL_HX - CAV_HX) / 2, SOCKET_Z), (SHELL_L, SHELL_HX - CAV_HX, 2 * SHELL_HZ), DARK))
        # Bottom wall (-z)
        boxes.append(_box((shell_cx, sy, SOCKET_Z - CAV_HZ - (SHELL_HZ - CAV_HZ) / 2), (SHELL_L, 2 * CAV_HX, SHELL_HZ - CAV_HZ), DARK))
        # Top wall (+z) with key slot
        # Key slot sits at key_x in plug frame (world y = sy - KEY_X); key slot width is 8 mm
        slot_w = KEY_W + 2 * CLEAR
        slot_cy = sy - KEY_X
        slot_y0 = slot_cy - slot_w / 2
        slot_y1 = slot_cy + slot_w / 2
        wall_y_left = sy - CAV_HX
        wall_y_right = sy + CAV_HX
        # Top wall left
        boxes.append(_box((shell_cx, (wall_y_left + slot_y0) / 2, SOCKET_Z + CAV_HZ + (SHELL_HZ - CAV_HZ) / 2), (SHELL_L, slot_y0 - wall_y_left, SHELL_HZ - CAV_HZ), DARK))
        # Top wall right
        boxes.append(_box((shell_cx, (slot_y1 + wall_y_right) / 2, SOCKET_Z + CAV_HZ + (SHELL_HZ - CAV_HZ) / 2), (SHELL_L, wall_y_right - slot_y1, SHELL_HZ - CAV_HZ), DARK))
        # Cap over key slot
        cap_h = SHELL_HZ - (NOSE_H / 2 + KEY_H + CLEAR)
        boxes.append(_box((shell_cx, slot_cy, SOCKET_Z + SHELL_HZ - cap_h / 2), (SHELL_L, slot_w, cap_h), DARK))

    # 3. Controller box
    boxes.append(_box(CTRL_POS, CTRL_SIZE, DARK_GREY))

    # 4. Readout board base plate
    boxes.append(_box(READOUT_POS, READOUT_SIZE, GREY))
    # Column marks along the far edge of the board: 1, 2 or 3 white dots (4 x 4 x 1 mm) at the column's y
    for col, cy in READOUT_COLS.items():
        for k in range(col):
            dy = (k - (col - 1) / 2) * 0.007
            boxes.append(_box((READOUT_MARK_X, cy + dy, TOP + READOUT_SIZE[2] + 0.0005), (0.004, 0.004, 0.001), WHITE))

    # 5. Base tie connecting controller box and strip on the table
    tie_size = (0.040, abs(SOCKET_Y[0] - (CTRL_POS[1] - CTRL_SIZE[1] / 2)), 0.010)
    tie_pos = (CTRL_POS[0], (SOCKET_Y[0] + (CTRL_POS[1] - CTRL_SIZE[1] / 2)) / 2, TOP + tie_size[2] / 2)
    boxes.append(_box(tie_pos, tie_size, DARK_GREY))

    # 6. Base tie connecting controller box and readout board on the table
    readout_tie_size = (0.040, abs(READOUT_POS[1] - READOUT_SIZE[1] / 2 - (CTRL_POS[1] + CTRL_SIZE[1] / 2)), 0.006)
    readout_tie_pos = (CTRL_POS[0], (READOUT_POS[1] - READOUT_SIZE[1] / 2 + (CTRL_POS[1] + CTRL_SIZE[1] / 2)) / 2, TOP + readout_tie_size[2] / 2)
    boxes.append(_box(readout_tie_pos, readout_tie_size, DARK_GREY))

    return boxes


def fixture_cylinders():
    """Cylinders for the fixed fixture: base domes for lamps."""
    cyls = []
    # Status lamp base (bright red dome)
    cyls.append(_cyl(STATUS_LAMP_POS, STATUS_LAMP_R, STATUS_LAMP_H, LAMP_BRIGHT_RED, "Z"))
    # Readout lamps (9 dark off lamps)
    for row_x in READOUT_ROWS.values():
        for col_y in READOUT_COLS.values():
            cyls.append(_cyl((row_x, col_y, READOUT_LAMP_Z), READOUT_LAMP_R, READOUT_LAMP_H, LAMP_OFF, "Z"))
    return cyls
