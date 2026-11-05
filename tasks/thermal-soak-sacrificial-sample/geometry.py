"""Every dimension and physical parameter of the thermal soak task in one place. Pure Python.

World frame (the station's): x forward from the arm bases, y left, z up, metres.
The left arm base is at (0, +0.305, 0.75), the right at (0, -0.305, 0.75).
Table top z = 0.7475 m (TOP).

The station contains:
- A fixed hot plate fixture (160 x 160 x 30 mm, dark grey, fixed 8.0 kg) with a 100 x 100 mm sample pad.
- Two buttons on the fixture front (70 mm apart): HEAT (red cap) and OFF (white cap), 8 mm travel to stop,
  100 N/m spring, 4 g mass, press counted <= -6.4 mm, re-armed >= -3.5 mm.
- A SURFACE-temperature bar of 9 stacked lamp discs (28 mm diameter, lamp k lit if surface T >= 20 + 10*k deg C;
  discs above 70 deg C are red, <= 70 deg C are cyan).
- A large status lamp (52 mm diameter): off (dark) / amber (sample on plate and heating) / green (success conditions met).
- A light blue cool pad on the right (200 x 100 x 5 mm) with space for two 60 mm blocks at 120 mm pitch.
- A staging area on the left with two sample blocks (60 x 60 x 40 mm, 0.3 kg each, blue = sample 1 expendable, orange = sample 2).
"""

from __future__ import annotations

import math
import numpy as np

TOP = 0.7475  # table top height in metres

# Colours (sRGB 0..1)
DARK = (0.12, 0.12, 0.13)
DARK_GREY = (0.22, 0.22, 0.24)
GREY = (0.55, 0.56, 0.58)
LIGHT_GREY = (0.75, 0.76, 0.78)
STEEL = (0.70, 0.72, 0.75)
ALUMINIUM = (0.74, 0.76, 0.78)
BLUE = (0.15, 0.35, 0.85)         # Sample 1 (expendable)
ORANGE = (0.95, 0.50, 0.10)       # Sample 2 (target)
LIGHT_BLUE = (0.55, 0.78, 0.92)   # Cool pad
HEAT_RED = (0.85, 0.15, 0.12)     # HEAT button cap
WHITE = (0.92, 0.92, 0.92)        # OFF button cap
STATUS_AMBER = (0.95, 0.65, 0.10) # Status lamp heating / core < 60 C
STATUS_GREEN = (0.12, 0.85, 0.18) # Status lamp target core 60-70 C
STATUS_RED = (0.95, 0.15, 0.12)   # Status lamp core > 70 C (overheated)
LAMP_CYAN = (0.15, 0.85, 0.85)    # Bar discs <= 70 deg C
LAMP_RED = (0.95, 0.15, 0.12)     # Bar discs > 70 deg C
LAMP_OFF = (0.18, 0.18, 0.20)     # Dark unlit lamp dome
PAD_COLOR = (0.16, 0.16, 0.17)    # Heating pad surface
STAGE_COLOR = (0.30, 0.30, 0.32)  # Staging mat surface

# Clearances and gripper dimensions
PAD_TO_TCP = 0.013  # distance from pad bottom to TCP z
FINGER_OPEN_M = 0.094  # 94 mm jaw opening fully open
SAMPLE_WIDTH = 0.060   # 60 mm sample block width
GRASP_CLEARANCE_PER_SIDE = (FINGER_OPEN_M - SAMPLE_WIDTH) / 2.0  # 17 mm per side

# -- Fixture layout (world frame) -----------------------------------------------------------------
FIXTURE_POS = (0.280, 0.000, TOP + 0.015)
FIXTURE_SIZE = (0.160, 0.160, 0.030)
FIXTURE_TOP_Z = TOP + FIXTURE_SIZE[2]  # TOP + 0.030 m
FIXTURE_MASS = 8.0  # kg (fixed >= 5 kg)
FIXTURE_FRICTION = 1.0

# Heating pad on top of fixture
PAD_SIZE = (0.100, 0.100, 0.002)
PAD_POS = (0.280, 0.000, FIXTURE_TOP_Z + PAD_SIZE[2] / 2.0)
PAD_TOP_Z = FIXTURE_TOP_Z + PAD_SIZE[2]  # TOP + 0.032 m

# Buttons on fixture front (70 mm pitch along y)
BUTTON_CAP_SIZE = (0.036, 0.036, 0.012)
BUTTON_TRAVEL = 0.008       # 8 mm travel to stop
BUTTON_RATE = 100.0         # 100 N/m
BUTTON_DAMPING = 4.0        # 4 N s/m
BUTTON_MASS = 0.004         # 4 g
BUTTON_AXIS = "Z"           # down along -z
BUTTON_TRIGGER_Z = -0.0064  # <= -6.4 mm counts as pressed
BUTTON_REARM_Z = -0.0035    # >= -3.5 mm re-arms

BUTTON_REST_Z = FIXTURE_TOP_Z + 0.014  # resting 8 mm proud above fixture top (hard stop)
BUTTON_REST_TOP_Z = BUTTON_REST_Z + BUTTON_CAP_SIZE[2] / 2.0  # TOP + 0.050 m
BUTTON_PRESS_DEPTH_CMD = 0.014  # 14 mm below rest top (6 mm past the 8 mm stop; tip stays 6 mm above fixture top)
BUTTON_PRESS_TCP_Z = BUTTON_REST_TOP_Z - BUTTON_PRESS_DEPTH_CMD + PAD_TO_TCP

# Heat button (red, front-left) and Off button (white, front-right)
HEAT_BTN_POS = (0.215, +0.035, BUTTON_REST_Z)
OFF_BTN_POS = (0.215, -0.035, BUTTON_REST_Z)

# Cool pad on right (200 x 100 x 5 mm)
COOL_PAD_SIZE = (0.200, 0.100, 0.005)
COOL_PAD_POS = (0.280, -0.200, TOP + COOL_PAD_SIZE[2] / 2.0)
COOL_PAD_TOP_Z = TOP + COOL_PAD_SIZE[2]  # TOP + 0.005 m

# Designated placement spots on cool pad (120 mm apart)
COOL_SPOT_1 = (0.220, -0.200, COOL_PAD_TOP_Z)  # Sample 1 (near)
COOL_SPOT_2 = (0.340, -0.200, COOL_PAD_TOP_Z)  # Sample 2 (far)

# Staging area on left (200 x 100 x 2 mm)
STAGE_MAT_SIZE = (0.200, 0.100, 0.002)
STAGE_MAT_POS = (0.280, +0.200, TOP + STAGE_MAT_SIZE[2] / 2.0)
STAGE_MAT_TOP_Z = TOP + STAGE_MAT_SIZE[2]  # TOP + 0.002 m

# Sample starting poses (120 mm apart)
SAMPLE_SIZE = (0.060, 0.060, 0.040)  # 60 x 60 x 40 mm
SAMPLE_MASS = 0.300  # 300 g
SAMPLE_FRICTION = 0.8
SAMPLE_START_Z = STAGE_MAT_TOP_Z + SAMPLE_SIZE[2] / 2.0

START_POS_1 = (0.220, +0.200, SAMPLE_START_Z)  # Sample 1 (blue)
START_POS_2 = (0.340, +0.200, SAMPLE_START_Z)  # Sample 2 (orange)

# Status lamp (52 mm diameter dome, 12 mm height)
STATUS_LAMP_R = 0.026
STATUS_LAMP_H = 0.012
STATUS_LAMP_POS = (0.400, 0.000, FIXTURE_TOP_Z + STATUS_LAMP_H / 2.0)
STATUS_LAMP_LIT_R = STATUS_LAMP_R + 0.001
STATUS_LAMP_LIT_H = STATUS_LAMP_H + 0.002
STATUS_LAMP_LIT_POS = (STATUS_LAMP_POS[0], STATUS_LAMP_POS[1], STATUS_LAMP_POS[2] + 0.001)

# Surface-temperature bar (9 stacked lamp discs, >= 28 mm diameter)
# Oriented along x at y = -0.095 (pitch 25 mm from x = 0.170 to x = 0.370)
BAR_LAMP_R = 0.014   # 28 mm diameter
BAR_LAMP_H = 0.004   # 4 mm height
BAR_LAMP_Y = -0.095
BAR_LAMP_Z = FIXTURE_TOP_Z + BAR_LAMP_H / 2.0
BAR_LAMP_LIT_R = BAR_LAMP_R + 0.001
BAR_LAMP_LIT_H = BAR_LAMP_H + 0.002
BAR_LAMP_LIT_Z = BAR_LAMP_Z + 0.001

BAR_DISC_X = tuple(0.170 + (k - 1) * 0.025 for k in range(1, 10))  # 9 discs k=1..9
BAR_DISC_TEMP = tuple(20.0 + 10.0 * k for k in range(1, 10))      # 30, 40, ..., 110 deg C

# Stowed position for non-active visual indicator markers (under table)
STOWED_Z = TOP - 0.100

# -- Thermal state machine constants ---------------------------------------------------------------
PLATE_TEMP_MIN = 20.0        # deg C (ambient)
PLATE_TEMP_SETPOINT = 110.0  # deg C (thermostat setpoint)
PLATE_TEMP_MAX = 110.0       # deg C (thermostat holds setpoint)
PLATE_HEAT_RATE = 10.0       # deg C / s (heats to 110 C in 9.0 s <= 10 s, then holds)
PLATE_COOL_RATE = 1.5        # deg C / s
TAU_SURF_HEAT = 10.0         # s (surface relaxing toward thermostat Tp=110 C)
TAU_CORE = 18.0              # s (core relaxing toward Ts)
TAU_COOL = 50.0              # s (cooling toward 20 C)
CORE_BAND_MIN = 60.0         # deg C
CORE_BAND_MAX = 70.0         # deg C
CORE_FIRE_LIMIT = 90.0       # deg C (zero-gate)
CORE_SETTLED_RATE = 0.2      # deg C / s (|dTc/dt| < 0.2 for 1.0)
SUCCESS_HOLD_S = 1.0         # 1.0 s success hold (20 steps at 20 Hz)
DEMO_NOMINAL_DWELL = 14.0    # s on plate for 60-70 C peak core (robust across +/-1.5 s)


def _box(center, size, color):
    return (tuple(float(x) for x in center), tuple(float(s) for s in size), tuple(float(c) for c in color))


def _cyl(center, radius, height, color, axis="Z"):
    return (tuple(float(x) for x in center), float(radius), float(height), tuple(float(c) for c in color), str(axis))


def fixture_boxes() -> list[tuple]:
    """Boxes composing the rigid fixture: base, heating pad, indicator housings, cool pad, staging mat."""
    boxes = [
        # Main fixture block
        _box(FIXTURE_POS, FIXTURE_SIZE, DARK_GREY),
        # Heating pad
        _box(PAD_POS, PAD_SIZE, PAD_COLOR),
        # Temperature bar base strip
        _box(((BAR_DISC_X[0] + BAR_DISC_X[-1]) / 2.0, BAR_LAMP_Y, FIXTURE_TOP_Z + 0.001), (0.220, 0.036, 0.002), DARK),
        # Cool pad (rigidly mounted to station)
        _box(COOL_PAD_POS, COOL_PAD_SIZE, LIGHT_BLUE),
        # Staging mat (rigidly mounted to station)
        _box(STAGE_MAT_POS, STAGE_MAT_SIZE, STAGE_COLOR),
    ]
    return boxes


def fixture_cylinders() -> list[tuple]:
    """Cylinders composing unlit indicator bases on the fixture."""
    cyls = [
        # Status lamp dark dome
        _cyl(STATUS_LAMP_POS, STATUS_LAMP_R, STATUS_LAMP_H, LAMP_OFF, "Z"),
    ]
    # 9 dark off discs for temperature bar
    for x in BAR_DISC_X:
        cyls.append(_cyl((x, BAR_LAMP_Y, BAR_LAMP_Z), BAR_LAMP_R, BAR_LAMP_H, LAMP_OFF, "Z"))
    return cyls


def fixture_named_boxes_world() -> list[tuple[str, tuple, tuple, tuple]]:
    """Named boxes in the world frame for collision checking."""
    fb = fixture_boxes()
    names = [
        "fixture_base",
        "sample_pad",
        "bar_strip",
        "cool_pad",
        "staging_mat",
    ]
    return [(names[i], b[0], b[1], b[2]) for i, b in enumerate(fb)]


def sample_boxes(color: tuple) -> list[tuple]:
    """Single box collider for a sample block in its local frame."""
    return [((0.0, 0.0, 0.0), SAMPLE_SIZE, color)]
