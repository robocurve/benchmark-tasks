"""Every dimension and physical constant of the buffer-socket-module-permutation task.

World frame (station frame): x forward from arm bases, y left, z up, metres.
Left arm base: (0.0, +0.305)
Right arm base: (0.0, -0.305)
Table top height: z = 0.7475 m (TOP).

Station hardware:
- Rigid 8 kg bus strip with four keyed sockets in a row (pitch 70 mm), mouths at x = 0.240 m,
  at y = -0.260, -0.190, -0.120, -0.050 (sockets 1..4).
  Sockets 1-3 each have a coloured lamp disc (red, green, blue) showing required module band colour.
  Socket 4 has no lamp disc (free buffer socket).
- Three keyed plug modules (45 x 18 x 28 mm housing, coloured bands red, green, blue; 1 mm play per side).
  Initial seating:
    - Socket 1 holds green module
    - Socket 2 holds red module
    - Socket 3 holds blue module
    - Socket 4 is empty (buffer)
- Controller box reachable by left arm at (x = 0.280, y = +0.150) with:
  - White COMMIT button (36x36x12 mm cap, 8 mm travel to stop, 100 N/m spring).
  - 50 mm diameter STATUS lamp beside COMMIT button:
    * Green when all 3 coloured sockets hold matching module AND no other module displaced > 5 mm.
    * Red otherwise (or while any module is being held/unseated).
"""

from __future__ import annotations

import math

TOP = 0.7475  # table top height

# Colours (sRGB 0..1)
DARK = (0.12, 0.12, 0.13)
DARK_GREY = (0.22, 0.22, 0.24)
GREY = (0.55, 0.56, 0.58)
LIGHT_GREY = (0.75, 0.76, 0.78)
STEEL = (0.78, 0.79, 0.80)
ALUMINIUM = (0.74, 0.76, 0.78)
WHITE = (0.92, 0.92, 0.92)

# Module band colours
BAND_RED = (0.90, 0.10, 0.10)
BAND_GREEN = (0.10, 0.85, 0.20)
BAND_BLUE = (0.15, 0.35, 0.90)

# Status lamp colours
STATUS_LAMP_OFF = (0.18, 0.18, 0.20)
STATUS_LAMP_RED = (0.90, 0.10, 0.10)
STATUS_LAMP_GREEN = (0.10, 0.95, 0.20)

CLEAR = 0.001  # 1.0 mm clearance per side between nose and socket cavity

# -- Module (plug) dimensions (identical to reference task) ----------------------------------------
NOSE_W = 0.028   # x in plug frame
NOSE_H = 0.016   # z in plug frame
NOSE_L = 0.018   # y in plug frame (protrusion from housing)
HOUSING = (0.045, 0.018, 0.028)  # x (width), y (depth), z (height)
KEY_W = 0.006
KEY_H = 0.003
KEY_X = 0.001
GROOVE = (0.0075, 0.0125)
GROOVE_DEPTH = 0.0015
MODULE_MASS = 0.080  # 80 g
MODULE_FRICTION = 0.8

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

# Plunger specifications (proven spring detents)
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

# Four sockets along y: pitch 70 mm
SOCKET_PITCH = 0.070
SOCKET_Y = (-0.260, -0.190, -0.120, -0.050)  # sockets 1, 2, 3, 4
MOUTH_X = 0.240  # world x coordinate of socket mouth planes
SOCKET_Z = TOP + 0.025  # height of socket axes (25 mm above table)

# Required colour per coloured socket (sockets 1..3)
SOCKET_TARGET_COLOURS = {
    0: "red",
    1: "green",
    2: "blue",
}
SOCKET_TARGET_RGB = {
    0: BAND_RED,
    1: BAND_GREEN,
    2: BAND_BLUE,
}

# Module names and starting sockets:
# Socket 1 (index 0) holds green module
# Socket 2 (index 1) holds red module
# Socket 3 (index 2) holds blue module
# Socket 4 (index 3) is empty (buffer)
MODULE_NAMES = ("module_red", "module_green", "module_blue")
MODULE_COLOURS = {
    "module_red": BAND_RED,
    "module_green": BAND_GREEN,
    "module_blue": BAND_BLUE,
}
START_SOCKET = {
    "module_green": 0,
    "module_red": 1,
    "module_blue": 2,
}
TARGET_SOCKET = {
    "module_red": 0,    # Target: Socket 1 (red lamp)
    "module_green": 1,  # Target: Socket 2 (green lamp)
    "module_blue": 2,   # Target: Socket 3 (blue lamp)
}

START_POS = {
    "module_green": (MOUTH_X, SOCKET_Y[0], SOCKET_Z),
    "module_red": (MOUTH_X, SOCKET_Y[1], SOCKET_Z),
    "module_blue": (MOUTH_X, SOCKET_Y[2], SOCKET_Z),
}

STAGE_X = 0.170  # x when pulled straight back out of socket (70 mm travel)

# Socket indicator lamp discs (on top of socket shells 1, 2, 3)
SOCKET_LAMP_R = 0.010  # 20 mm diameter
SOCKET_LAMP_H = 0.002  # 2 mm thick
SOCKET_LAMP_POS = {
    i: (MOUTH_X + SHELL_L / 2, SOCKET_Y[i], SOCKET_Z + SHELL_HZ + SOCKET_LAMP_H / 2)
    for i in (0, 1, 2)
}

# Controller box (reachable by left arm)
CTRL_POS = (0.280, 0.150, TOP + 0.020)  # centre of controller box
CTRL_SIZE = (0.140, 0.120, 0.040)       # 140 mm along x, 120 mm along y, 40 mm tall

# COMMIT pushbutton on controller box
BUTTON_TRAVEL = 0.008        # hard stop: 8 mm travel
BUTTON_RATE = 100.0          # N/m
BUTTON_DAMPING = 4.0
BUTTON_MASS = 0.004          # 4 g
BUTTON_CAP_SIZE = (0.036, 0.036, 0.012)
BUTTON_REST_Z = TOP + 0.040 + 0.008 + BUTTON_CAP_SIZE[2] / 2
BUTTON_REST_TOP_Z = TOP + 0.060
BUTTON_PRESS_THRESHOLD = 0.0064   # logic: 80% of travel (displacement <= -6.4 mm)
BUTTON_RELEASE_THRESHOLD = 0.0035 # re-arm >= -3.5 mm displacement
BUTTON_PRESS_DEPTH_CMD = 0.014    # commanded fingertip depth below rest cap top
PAD_TIP_PAST_TCP = 0.013          # lowest fingertip point below TCP
BUTTON_PRESS_TCP_Z = BUTTON_REST_TOP_Z - BUTTON_PRESS_DEPTH_CMD + PAD_TIP_PAST_TCP
COMMIT_BTN_POS = (0.245, 0.150, BUTTON_REST_Z)

# STATUS lamp (50 mm diameter, 15 mm height, placed beside COMMIT button on controller box)
STATUS_LAMP_R = 0.025  # 50 mm diameter
STATUS_LAMP_H = 0.015
STATUS_LAMP_POS = (0.315, 0.150, TOP + 0.040 + STATUS_LAMP_H / 2)
STATUS_LAMP_LIT_R = STATUS_LAMP_R + 0.001
STATUS_LAMP_LIT_H = STATUS_LAMP_H + 0.002
STATUS_LAMP_LIT_POS = (STATUS_LAMP_POS[0], STATUS_LAMP_POS[1], STATUS_LAMP_POS[2] + 0.001)

# Visual indicator stowing position (under table)
STOWED_Z = TOP - 0.100


def _box(centre, size, colour):
    return (tuple(float(v) for v in centre), tuple(float(v) for v in size), tuple(float(v) for v in colour))


def _cyl(centre, radius, height, colour, axis="Z"):
    return (tuple(float(v) for v in centre), float(radius), float(height), tuple(float(v) for v in colour), str(axis).upper())


def servo_boxes(colour):
    """The servo module shapes in the plug frame (origin at nose root, nose along +y)."""
    g0, g1 = GROOVE
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
    """All boxes for the static bus strip, sockets, controller box, tie bar, and socket lamp discs."""
    boxes = []
    # 1. Bus strip main beam (behind sockets)
    strip_w = 0.050
    strip_l = 0.290  # covers y from -0.010 to -0.300
    strip_h = 0.050
    strip_cx = MOUTH_X + SHELL_L + FLANGE_T + strip_w / 2
    strip_cy = (SOCKET_Y[0] + SOCKET_Y[3]) / 2  # -0.155
    boxes.append(_box((strip_cx, strip_cy, TOP + strip_h / 2), (strip_w, strip_l, strip_h), DARK_GREY))

    # 2. Sockets (four socket slots)
    for i, sy in enumerate(SOCKET_Y):
        flange_x = MOUTH_X + SHELL_L + FLANGE_T / 2
        boxes.append(_box((flange_x, sy, SOCKET_Z), (FLANGE_T, FLANGE_W, FLANGE_H), ALUMINIUM))

        # Shell walls around cavity
        shell_cx = MOUTH_X + SHELL_L / 2
        # Right wall (+y in world)
        boxes.append(_box((shell_cx, sy + CAV_HX + (SHELL_HX - CAV_HX) / 2, SOCKET_Z), (SHELL_L, SHELL_HX - CAV_HX, 2 * SHELL_HZ), DARK))
        # Left wall (-y in world)
        boxes.append(_box((shell_cx, sy - CAV_HX - (SHELL_HX - CAV_HX) / 2, SOCKET_Z), (SHELL_L, SHELL_HX - CAV_HX, 2 * SHELL_HZ), DARK))
        # Bottom wall (-z)
        boxes.append(_box((shell_cx, sy, SOCKET_Z - CAV_HZ - (SHELL_HZ - CAV_HZ) / 2), (SHELL_L, 2 * CAV_HX, SHELL_HZ - CAV_HZ), DARK))

        # Top wall (+z) with key slot
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

    # 4. Base tie connecting bus strip and controller box on table
    tie_y_min = SOCKET_Y[3] + 0.030  # ~ -0.020
    tie_y_max = CTRL_POS[1] - CTRL_SIZE[1] / 2  # 0.090
    tie_size = (0.040, tie_y_max - tie_y_min, 0.010)
    tie_pos = (CTRL_POS[0], (tie_y_min + tie_y_max) / 2, TOP + tie_size[2] / 2)
    boxes.append(_box(tie_pos, tie_size, DARK_GREY))

    return boxes


def fixture_cylinders():
    """Cylinders for the fixed fixture: status lamp base dome, and coloured socket lamp discs."""
    cyls = []
    # Status lamp base (dark off dome, 50 mm diameter)
    cyls.append(_cyl(STATUS_LAMP_POS, STATUS_LAMP_R, STATUS_LAMP_H, STATUS_LAMP_OFF, "Z"))

    # Coloured lamp discs for sockets 1, 2, 3
    for idx, pos in SOCKET_LAMP_POS.items():
        col = SOCKET_TARGET_RGB[idx]
        cyls.append(_cyl(pos, SOCKET_LAMP_R, SOCKET_LAMP_H, col, "Z"))

    return cyls
