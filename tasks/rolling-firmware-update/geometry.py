"""Every dimension of the rolling firmware update task in one place. Pure Python.

World frame (the station's): x forward from the arm bases, y left, z up, metres.
The left arm base is at (0, +0.305), the right at (0, -0.305).
Table top z = 0.7475 m (TOP).

The station contains:
- A rigid free-standing base fixture (240 x 280 x 15 mm, mass 8.0 kg, friction 1.0, dark grey).
- Two redundant unit panels (Unit A on left y > 0, Unit B on right y < 0):
  * Panel body (120 x 90 x 55 mm, grey, mounted on the base plate).
  * SERVICE DOOR: spring slider along X, 25 mm travel with hard stop, 2 N spring (80 N/m), orange handle.
  * UPDATE button: recessed, yellow cap (38 x 38 x 12 mm), 8 mm travel to hard stop, 100 N/m spring, 4 g mass.
  * ACTIVE lamp: 40 mm diameter disc, bright blue when active.
  * VERSION lamp: 40 mm diameter disc (grey = v0, amber = updating, white = v1).
  * 4-lamp progress bar: four 28 mm diameter discs, cyan when lit.
- One shared central panel (between Unit A and Unit B):
  * Panel body (180 x 60 x 40 mm, dark grey).
  * SWITCH button: white cap (38 x 38 x 12 mm), 8 mm travel to hard stop, 100 N/m spring, 4 g mass.
  * LOAD lamp: 60 mm diameter disc (green = served, red = outage).
"""

from __future__ import annotations

import math
import numpy as np

TOP = 0.7475  # table top height in metres

# sRGB colours (0..1)
DARK = (0.12, 0.12, 0.13)
DARK_GREY = (0.22, 0.22, 0.24)
GREY = (0.55, 0.56, 0.58)
LIGHT_GREY = (0.75, 0.76, 0.78)
STEEL = (0.70, 0.72, 0.75)
ALUMINIUM = (0.74, 0.76, 0.78)
ORANGE = (0.95, 0.50, 0.10)
YELLOW = (0.95, 0.80, 0.10)
BLUE = (0.10, 0.40, 0.95)
CYAN = (0.10, 0.85, 0.85)
AMBER = (0.95, 0.60, 0.10)
RED = (0.85, 0.15, 0.12)
BRIGHT_RED = (0.95, 0.05, 0.05)
GREEN = (0.12, 0.88, 0.18)
WHITE = (0.92, 0.92, 0.92)

# Clearances and tolerances
CLEAR_MIN = 0.001  # minimum 1.0 mm clearance per side
PAD_TO_TCP = 0.013  # 13 mm from finger pad bottom to TCP z
GRIP_PAST_CONTACT = 0.005  # 5 mm clamp past contact

# Arm compliance and pushing model (proven in Task G)
FINGER_HALF_DEPTH_MM = (
    (0.5, 6.5), (1.0, 7.0), (2.0, 7.75), (3.0, 8.0), (5.0, 8.75),
    (7.5, 9.75), (10.0, 10.75), (12.5, 11.75), (15.0, 12.75),
    (17.5, 13.75), (20.0, 14.75), (22.5, 15.75), (25.0, 16.5), (30.0, 18.5)
)


def finger_half_depth(h_above_pad_bottom: float) -> float:
    """Closed-finger half-depth in x (metres) at height ``h`` (metres) above pad bottom."""
    hs = [h for h, _ in FINGER_HALF_DEPTH_MM]
    ws = [w for _, w in FINGER_HALF_DEPTH_MM]
    return float(np.interp(h_above_pad_bottom * 1000.0, hs, ws)) / 1000.0


# --- Base Fixture Geometry ---
BASE_LX = 0.220  # length in x (220 mm)
BASE_LY = 0.440  # width in y (440 mm)
BASE_LZ = 0.015  # thickness in z (15 mm)
BASE_CX = 0.280  # center x
BASE_CY = 0.000  # center y
BASE_CZ = TOP + BASE_LZ / 2  # 0.7550 m
BASE_POS = (BASE_CX, BASE_CY, BASE_CZ)
BASE_SIZE = (BASE_LX, BASE_LY, BASE_LZ)
BASE_TOP_Z = TOP + BASE_LZ  # 0.7625 m
BASE_MASS = 8.0  # kg
BASE_FRICTION = 1.0

# --- Unit Panels A and B ---
# Unit panels sit side-by-side on the base fixture:
# Unit A is on the left (y > 0), Unit B on the right (y < 0).
PANEL_LX = 0.120  # 120 mm along x
PANEL_LY = 0.090  # 90 mm along y
PANEL_LZ = 0.055  # 55 mm tall above base plate
PANEL_CY = {"A": +0.160, "B": -0.160}  # centered at y = +/- 160 mm
PANEL_CX = 0.280
PANEL_CZ = BASE_TOP_Z + PANEL_LZ / 2  # 0.7900 m
PANEL_TOP_Z = BASE_TOP_Z + PANEL_LZ  # 0.8175 m

# --- Shared Panel (Centre) ---
# Sits along midline between Unit A and Unit B
SHARED_LX = 0.180  # 180 mm along x
SHARED_LY = 0.050  # 50 mm along y (y in [-0.025, +0.025])
SHARED_LZ = 0.040  # 40 mm tall above base plate
SHARED_CX = 0.280
SHARED_CY = 0.000
SHARED_CZ = BASE_TOP_Z + SHARED_LZ / 2
SHARED_TOP_Z = BASE_TOP_Z + SHARED_LZ  # 0.8025 m

# --- Pushbuttons (UPDATE A, UPDATE B, SWITCH) ---
# Proven specs: 38x38x12 mm cap, 8 mm travel with end stop, 100 N/m spring, 4 g mass.
BUTTON_CAP_LX = 0.038
BUTTON_CAP_LY = 0.038
BUTTON_CAP_LZ = 0.012
BUTTON_CAP_SIZE = (BUTTON_CAP_LX, BUTTON_CAP_LY, BUTTON_CAP_LZ)
BUTTON_CAP_MASS = 0.004  # 4 g
BUTTON_TRAVEL = 0.008  # 8 mm travel
BUTTON_SPRING_RATE = 100.0  # 100 N/m
BUTTON_DAMPING = 4.0  # N s/m
BUTTON_PRESS_THRESHOLD = -0.0064  # press counted at <= -6.4 mm
BUTTON_REARM_THRESHOLD = -0.0035  # re-armed at >= -3.5 mm
BUTTON_COMMAND_OVERSHOOT = 0.006  # commanded 6 mm past stop (14 mm below rest top)

# Recessed UPDATE button pocket:
# Pocket opening 42x48 mm, wall 3 mm (outer 48x54 mm), inside Unit panel top.
POCKET_LX = 0.048
POCKET_LY = 0.054
POCKET_WALL = 0.003
POCKET_INNER_LX = POCKET_LX - 2 * POCKET_WALL  # 0.042 m (1.0 mm per side to 38 mm cap)
POCKET_INNER_LY = POCKET_LY - 2 * POCKET_WALL
POCKET_DEPTH = 0.022  # 22 mm deep pocket cavity

# UPDATE button positions (Unit A and Unit B):
# Recessed into panel top so the service door can slide over it.
UPDATE_BTN_CX = 0.290
UPDATE_BTN_CY = {"A": +0.160, "B": -0.160}
# Button cap rest top sits 2 mm below panel top (recessed):
UPDATE_BTN_CAP_TOP_REST_Z = PANEL_TOP_Z - 0.002
UPDATE_BTN_CAP_REST_CZ = UPDATE_BTN_CAP_TOP_REST_Z - BUTTON_CAP_LZ / 2

# SWITCH button (on shared panel at midline):
SWITCH_BTN_CX = 0.205
SWITCH_BTN_CY = 0.000
# Sits with cap top 8 mm proud of shared panel top at rest:
SWITCH_BTN_CAP_REST_CZ = SHARED_TOP_Z + BUTTON_TRAVEL + BUTTON_CAP_LZ / 2 - 0.004
SWITCH_BTN_CAP_TOP_REST_Z = SWITCH_BTN_CAP_REST_CZ + BUTTON_CAP_LZ / 2

# --- Service Doors ---
# Spring slider along X: 25 mm travel, 2 N spring (80 N/m), 25 g mass.
# Deflection >= 18 mm required for update.
# Handle on native arm side; cover plate at y = UPDATE_BTN_CY covering pocket when closed.
DOOR_TRAVEL = 0.025  # 25 mm
DOOR_SPRING_RATE = 80.0  # 80 N/m (2.0 N at 25 mm); GPU run: the arm is only ~165 N/m stiff, a 320 N/m door stalled at 11 mm
DOOR_DAMPING = 10.0  # N s/m
DOOR_MASS = 0.025  # 25 g
DOOR_DEFLECTION_THRESHOLD = 0.018  # 18 mm required to update (clears pad footprint by >= 3.3 mm)
DOOR_RELEASED_THRESHOLD = 0.002  # < 2 mm for level 1.0 success

# Door Handle:
# Orange box, 24 mm in X, 20 mm in Y, 20 mm in Z.
DOOR_HANDLE_LX = 0.024
DOOR_HANDLE_LY = 0.020
DOOR_HANDLE_LZ = 0.020
DOOR_HANDLE_SIZE = (DOOR_HANDLE_LX, DOOR_HANDLE_LY, DOOR_HANDLE_LZ)
# Rest positions of the orange handles (native arm side to maintain arm-arm clearance):
DOOR_HANDLE_REST_CX = 0.255
DOOR_HANDLE_REST_CY = {"A": -0.025, "B": +0.025}
DOOR_HANDLE_REST_CZ = PANEL_TOP_Z + DOOR_HANDLE_LZ / 2 + 0.004  # stands proud of panel

# Door Cover Plate:
# Slides over recessed pocket at (UPDATE_BTN_CX, UPDATE_BTN_CY).
# Size 32 x 44 x 4 mm, sits at rest covering the pocket.
DOOR_COVER_LX = 0.032
DOOR_COVER_LY = 0.044
DOOR_COVER_LZ = 0.004
DOOR_COVER_SIZE = (DOOR_COVER_LX, DOOR_COVER_LY, DOOR_COVER_LZ)
DOOR_COVER_REST_CX = 0.275
DOOR_COVER_REST_CY = {"A": +0.160, "B": -0.160}
DOOR_COVER_REST_CZ = PANEL_TOP_Z + DOOR_COVER_LZ / 2

# Door Slider Origin / Anchor (relative to BASE_POS):
DOOR_LOCAL_REST = {
    "A": (DOOR_HANDLE_REST_CX - BASE_CX, DOOR_HANDLE_REST_CY["A"] - BASE_CY, DOOR_HANDLE_REST_CZ - BASE_CZ),
    "B": (DOOR_HANDLE_REST_CX - BASE_CX, DOOR_HANDLE_REST_CY["B"] - BASE_CY, DOOR_HANDLE_REST_CZ - BASE_CZ),
}

# UPDATE button local rest (relative to BASE_POS):
UPDATE_BTN_LOCAL_REST = {
    "A": (UPDATE_BTN_CX - BASE_CX, UPDATE_BTN_CY["A"] - BASE_CY, UPDATE_BTN_CAP_REST_CZ - BASE_CZ),
    "B": (UPDATE_BTN_CX - BASE_CX, UPDATE_BTN_CY["B"] - BASE_CY, UPDATE_BTN_CAP_REST_CZ - BASE_CZ),
}

# SWITCH button local rest:
SWITCH_BTN_LOCAL_REST = (
    SWITCH_BTN_CX - BASE_CX,
    SWITCH_BTN_CY - BASE_CY,
    SWITCH_BTN_CAP_REST_CZ - BASE_CZ,
)

# --- Display Indicators & Lamps ---
# Stowed height for off markers (under the table)
STOWED_Z = TOP - 0.100

# ACTIVE Lamps: 40 mm diameter disc, 8 mm tall (blue)
ACTIVE_LAMP_R = 0.020  # 40 mm diameter
ACTIVE_LAMP_H = 0.008
ACTIVE_LAMP_CX = 0.230
ACTIVE_LAMP_CY = {"A": +0.160, "B": -0.160}
ACTIVE_LAMP_CZ = PANEL_TOP_Z + ACTIVE_LAMP_H / 2
ACTIVE_LAMP_POS = {
    "A": (ACTIVE_LAMP_CX, ACTIVE_LAMP_CY["A"], ACTIVE_LAMP_CZ),
    "B": (ACTIVE_LAMP_CX, ACTIVE_LAMP_CY["B"], ACTIVE_LAMP_CZ),
}

# VERSION Lamps: 40 mm diameter disc, 8 mm tall
VERSION_LAMP_R = 0.020
VERSION_LAMP_H = 0.008
VERSION_LAMP_CX = 0.350
VERSION_LAMP_CY = {"A": +0.160, "B": -0.160}
VERSION_LAMP_CZ = PANEL_TOP_Z + VERSION_LAMP_H / 2
VERSION_LAMP_POS = {
    "A": (VERSION_LAMP_CX, VERSION_LAMP_CY["A"], VERSION_LAMP_CZ),
    "B": (VERSION_LAMP_CX, VERSION_LAMP_CY["B"], VERSION_LAMP_CZ),
}

# 4-Lamp Progress Bar (Unit A and Unit B):
# 4 discs, 28 mm diameter (14 mm radius), 6 mm tall, along outer edge (y = +/- 0.200)
PROGRESS_LAMP_R = 0.014  # 28 mm diameter
PROGRESS_LAMP_H = 0.006
PROGRESS_LAMP_CY = {"A": +0.200, "B": -0.200}
PROGRESS_LAMP_XS = (0.230, 0.265, 0.300, 0.335)
PROGRESS_LAMP_CZ = PANEL_TOP_Z + PROGRESS_LAMP_H / 2


def progress_lamp_pos(unit: str, index: int) -> tuple[float, float, float]:
    """World position of progress lamp index (1..4) for 'A' or 'B'."""
    assert 1 <= index <= 4, f"index must be 1..4, got {index}"
    return (PROGRESS_LAMP_XS[index - 1], PROGRESS_LAMP_CY[unit], PROGRESS_LAMP_CZ)


# LOAD Lamp (Shared Panel):
# 60 mm diameter disc (30 mm radius), 8 mm tall (green = served, red = outage)
LOAD_LAMP_R = 0.030  # 60 mm diameter
LOAD_LAMP_H = 0.008
LOAD_LAMP_CX = 0.320
LOAD_LAMP_CY = 0.000
LOAD_LAMP_CZ = SHARED_TOP_Z + LOAD_LAMP_H / 2
LOAD_LAMP_POS = (LOAD_LAMP_CX, LOAD_LAMP_CY, LOAD_LAMP_CZ)


# --- Helper to author fixture shapes ---

def _box(centre, size, colour):
    return (tuple(float(v) for v in centre), tuple(float(v) for v in size), tuple(float(v) for v in colour))


def _cyl(centre, radius, height, colour, axis="Z"):
    return (tuple(float(v) for v in centre), float(radius), float(height), tuple(float(v) for v in colour), axis)


def fixture_named_boxes_world() -> list[tuple[str, tuple, tuple, tuple]]:
    """Return all fixed visual/collision boxes comprising the rigid station fixture in WORLD frame."""
    boxes = []

    def add(name, centre, size, colour):
        boxes.append((name, centre, size, colour))

    # 1. Base plate
    add("base plate", BASE_POS, BASE_SIZE, DARK_GREY)

    # 2. Shared central panel body
    add("shared panel body", (SHARED_CX, SHARED_CY, SHARED_CZ), (SHARED_LX, SHARED_LY, SHARED_LZ), DARK)

    # 3. SWITCH button sleeve ring on shared panel (hollow ring: cap slides inside it)
    # GPU run 2: finger pads landed on the old 12 mm tall ring (cap only reached -4.5 mm). Keep the ring low (4 mm).
    sw_sleeve_h = 0.004
    sw_sleeve_cz = SHARED_TOP_Z + sw_sleeve_h / 2
    sw_pw = 0.002  # 2 mm wall
    sw_lx = POCKET_LX
    sw_ly = 0.050
    ox = sw_lx / 2 - sw_pw / 2
    oy = sw_ly / 2 - sw_pw / 2
    inner_lx = sw_lx - 2 * sw_pw
    for sx, nm in ((-1, "near"), (1, "far")):
        add(f"switch sleeve ({nm} wall)", (SWITCH_BTN_CX + sx * ox, SWITCH_BTN_CY, sw_sleeve_cz), (sw_pw, sw_ly, sw_sleeve_h), DARK_GREY)
    for sy, nm in ((-1, "-y"), (1, "+y")):
        add(f"switch sleeve ({nm} wall)", (SWITCH_BTN_CX, SWITCH_BTN_CY + sy * oy, sw_sleeve_cz), (inner_lx, sw_pw, sw_sleeve_h), DARK_GREY)

    # 4. Unit A and Unit B panel bodies (hollowed around recessed UPDATE button pocket)
    floor_lz = PANEL_LZ - POCKET_DEPTH  # 33 mm floor
    floor_cz = BASE_TOP_Z + floor_lz / 2  # 0.7790 m
    cav_cz = PANEL_TOP_Z - POCKET_DEPTH / 2  # 0.8065 m
    cav_wall_y = (PANEL_LY - POCKET_LY) / 2  # 21 mm wall on east/west sides

    for u in ("A", "B"):
        cy = PANEL_CY[u]
        bx = UPDATE_BTN_CX

        # Floor plate under entire panel
        add(f"unit {u} panel floor", (PANEL_CX, cy, floor_cz), (PANEL_LX, PANEL_LY, floor_lz), GREY)

        # South block (-x from pocket cavity): x in [0.220, 0.266]
        add(f"unit {u} panel south", (0.243, cy, cav_cz), (0.046, PANEL_LY, POCKET_DEPTH), GREY)

        # North block (+x from pocket cavity): x in [0.314, 0.340]
        add(f"unit {u} panel north", (0.327, cy, cav_cz), (0.026, PANEL_LY, POCKET_DEPTH), GREY)

        # West block (-y from pocket cavity): y in [cy - 0.045, cy - 0.024]
        add(f"unit {u} panel west", (bx, cy - 0.0345, cav_cz), (POCKET_LX, cav_wall_y, POCKET_DEPTH), GREY)

        # East block (+y from pocket cavity): y in [cy + 0.024, cy + 0.045]
        add(f"unit {u} panel east", (bx, cy + 0.0345, cav_cz), (POCKET_LX, cav_wall_y, POCKET_DEPTH), GREY)

    return boxes


def fixture_boxes() -> tuple:
    """Return visual/collision boxes for the fixture Part in LOCAL part frame (relative to BASE_POS)."""
    world = fixture_named_boxes_world()
    bx, by, bz = BASE_POS
    local = []
    for _name, centre, size, colour in world:
        cx, cy, cz = centre
        local_pos = (cx - bx, cy - by, cz - bz)
        local.append(_box(local_pos, size, colour))
    return tuple(local)


def fixture_cylinders() -> tuple:
    """Return visual unlit lamp bases on the fixture in LOCAL part frame."""
    bx, by, bz = BASE_POS
    cyls = []

    # ACTIVE lamp unlit bases (blueish grey)
    for u in ("A", "B"):
        cx, cy, cz = ACTIVE_LAMP_POS[u]
        cyls.append(_cyl((cx - bx, cy - by, cz - bz - 0.002), ACTIVE_LAMP_R, 0.004, DARK, "Z"))

    # VERSION lamp unlit bases (grey)
    for u in ("A", "B"):
        cx, cy, cz = VERSION_LAMP_POS[u]
        cyls.append(_cyl((cx - bx, cy - by, cz - bz - 0.002), VERSION_LAMP_R, 0.004, GREY, "Z"))

    # 4 Progress lamp unlit bases (dark)
    for u in ("A", "B"):
        for i in range(1, 5):
            cx, cy, cz = progress_lamp_pos(u, i)
            cyls.append(_cyl((cx - bx, cy - by, cz - bz - 0.002), PROGRESS_LAMP_R, 0.004, DARK, "Z"))

    # LOAD lamp unlit base (dark)
    cx, cy, cz = LOAD_LAMP_POS
    cyls.append(_cyl((cx - bx, cy - by, cz - bz - 0.002), LOAD_LAMP_R, 0.004, DARK, "Z"))

    return tuple(cyls)


def door_slider_boxes(unit: str) -> tuple:
    """Return boxes for the door slider in LOCAL part frame (relative to BASE_POS)."""
    bx, by, bz = BASE_POS
    hx = DOOR_HANDLE_REST_CX - bx
    hy = DOOR_HANDLE_REST_CY[unit] - by
    hz = DOOR_HANDLE_REST_CZ - bz
    b_handle = _box((hx, hy, hz), DOOR_HANDLE_SIZE, ORANGE)

    # Cover plate at relative position in part frame:
    cx = DOOR_COVER_REST_CX - bx
    cy = DOOR_COVER_REST_CY[unit] - by
    cz = DOOR_COVER_REST_CZ - bz
    b_cover = _box((cx, cy, cz), DOOR_COVER_SIZE, STEEL)

    # Connecting tie bar: connects handle to cover plate
    bar_lx = abs(cx - hx) + DOOR_COVER_LX / 2
    bar_ly = abs(cy - hy)
    bar_lz = 0.004
    bar_cx = (hx + cx) / 2
    bar_cy = (hy + cy) / 2
    bar_cz = cz
    b_bar = _box((bar_cx, bar_cy, bar_cz), (bar_lx, bar_ly, bar_lz), STEEL)

    return (b_handle, b_cover, b_bar)
