"""Every dimension of the backlash dial setpoint task in one place. Pure Python.

World frame (the station's): x forward from the arm bases, y left, z up, metres.
The left arm base is at (0, +0.305), the right at (0, -0.305).
Table top z = 0.7475 m (TOP).

The station contains:
- A fixed base plate (320 x 440 x 10 mm, mass 5.0 kg, friction 1.0, dark grey).
- Two parallel tracks (channels): 140 mm long x 36 mm wide, 20 mm tall walls (6 mm thick), 300 mm apart (y = +/-0.150 m).
- 20 mm wide alternating light / blue position bands along the channel floors (0-20, 20-40, 60-80, 80-100 mm).
- Two loose knobs (30 x 30 x 14 mm, orange on left, teal on right, mass 80 g, friction 0.5), sliding 100 mm along rails.
- Two spring-loaded LOCK buttons beside the tracks (cap 36 x 36 x 12 mm, travel 8 mm to hard stop, 100 N/m spring, yellow).
- Two 10-lamp indicator bars above the rails at x = 0.380 m, showing displayed value (rail position minus hidden backlash).
- White target marker lamps indicating the target band (setpoint +- 2.5 mm displayed).
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
ORANGE = (0.95, 0.50, 0.10)
TEAL = (0.10, 0.65, 0.65)
YELLOW = (0.95, 0.85, 0.10)
RED = (0.85, 0.15, 0.12)
BRIGHT_RED = (0.95, 0.10, 0.10)
GREEN = (0.12, 0.85, 0.18)
BRIGHT_GREEN = (0.15, 0.95, 0.20)
CYAN = (0.15, 0.80, 0.90)
BRIGHT_CYAN = (0.20, 0.90, 1.00)
WHITE = (0.92, 0.92, 0.92)
BRIGHT_WHITE = (0.98, 0.98, 0.98)

# Band marker colours along track floors (every 20 mm)
BAND_LIGHT = (0.90, 0.90, 0.90)
BAND_BLUE = (0.20, 0.35, 0.80)
BAND_COLORS = (
    BAND_BLUE,   # 0-20 mm
    BAND_LIGHT,  # 20-40 mm
    BAND_BLUE,   # 40-60 mm
    BAND_LIGHT,  # 60-80 mm
    BAND_BLUE,   # 80-100 mm
    GREY,        # 100-130 mm (far stop end zone)
)

# Clearances and tolerances
CLEAR_MIN = 0.001  # minimum 1.0 mm clearance per side
PAD_TO_TCP = 0.013  # 13 mm distance from pad bottom to TCP z (measured 12.84 mm from collision hulls)

# Closed-finger half-depth along x versus height above pad bottom (measured from collision hulls)
FINGER_HALF_DEPTH_MM = (
    (0.5, 6.5), (1.0, 7.0), (2.0, 7.75), (3.0, 8.0), (5.0, 8.75), (7.5, 9.75), (10.0, 10.75),
    (12.5, 11.75), (15.0, 12.75), (17.5, 13.75), (20.0, 14.75), (22.5, 15.75), (25.0, 16.5),
    (30.0, 18.5)
)


def finger_half_depth(h_above_pad_bottom: float) -> float:
    """Closed-finger half-depth in x (metres) at height ``h`` (metres) above the pad bottom (linear interpolation)."""
    hs = [h for h, _ in FINGER_HALF_DEPTH_MM]
    ws = [w for _, w in FINGER_HALF_DEPTH_MM]
    return float(np.interp(h_above_pad_bottom * 1000.0, hs, ws)) / 1000.0


PUSH_PAD_HEIGHT = 0.0025  # pad bottom height above channel floor while pushing
PUSH_APPROACH_CLEAR = 0.003  # gap between finger and knob face when descending beside knob
PUSH_OVERSHOOT = 0.008  # 8 mm commanded push overshoot to overcome arm compliance

# Arm compliance at push pose (J K^-1 J^T)
ARM_X_COMPLIANCE = 5.9  # mm per N along push direction
ARM_ZX_COUPLING = 0.95  # vertical deflection per N of horizontal force
ARM_Z_COMPLIANCE_PUSH = 2.0  # vertical at push pose
FINGER_FACE_SLOPE = 0.4  # dx/dh of wedge face


def push_force_model(mu: float | None = None, mass: float | None = None) -> dict:
    """Quasi-static push of a knob by slanted finger face."""
    mu = KNOB_FRICTION if mu is None else mu
    mass = KNOB_MASS if mass is None else mass
    a = math.atan(FINGER_FACE_SLOPE)
    w = mass * 9.81
    n = mu * w / (math.cos(a) - mu * math.sin(a))
    f_h = n * math.cos(a)
    f_up = n * math.sin(a)
    lift = ARM_Z_COMPLIANCE_PUSH * f_up + ARM_ZX_COUPLING * f_h
    lag = ARM_X_COMPLIANCE * f_h
    return dict(normal=n, horizontal=f_h, finger_lift_mm=lift, stall_lag_mm=lag)


def push_contact_half_depth() -> float:
    """Half-depth of finger at knob's top edge, which is where a closed-finger push touches."""
    return finger_half_depth(KNOB_H - PUSH_PAD_HEIGHT)


# -- Base Plate -------------------------------------------------------------------------------------
BASE_L = 0.320  # length along x (320 mm: x in [0.120, 0.440])
BASE_W = 0.440  # width along y (440 mm: y in [-0.220, +0.220])
BASE_H = 0.010  # thickness along z (10 mm)
BASE_CX = 0.280  # center x
BASE_CY = 0.000  # center y
BASE_CZ = TOP + BASE_H / 2  # 0.7525 m
BASE_POS = (BASE_CX, BASE_CY, BASE_CZ)
BASE_SIZE = (BASE_L, BASE_W, BASE_H)
BASE_MASS = 5.0  # kg
BASE_FRICTION = 1.0
BASE_TOP_Z = TOP + BASE_H  # 0.7575 m

# -- Tracks (Channels) -----------------------------------------------------------------------------
# Two parallel tracks, 300 mm apart in y: left track at y = +0.150, right track at y = -0.150
TRACK_SPACING_Y = 0.300
TRACK_Y = {"left": +0.150, "right": -0.150}

TRACK_L = 0.140  # 140 mm total outer length along x
TRACK_CHANNEL_W = 0.036  # 36 mm inner channel width
TRACK_WALL_T = 0.006  # 6 mm side wall thickness
TRACK_WALL_H = 0.020  # 20 mm wall height above base plate
TRACK_END_WALL_T = 0.005  # 5 mm end stop wall thickness
TRACK_FLOOR_T = 0.002  # 2 mm channel floor thickness

TRACK_OUTER_W = TRACK_CHANNEL_W + 2 * TRACK_WALL_T  # 48 mm
TRACK_INNER_L = TRACK_L - 2 * TRACK_END_WALL_T  # 130 mm (sliding space)

# Track bounds along x: [0.200, 0.340], center x = 0.270
TRACK_START_X = 0.200
TRACK_END_X = TRACK_START_X + TRACK_L  # 0.340
TRACK_CX = (TRACK_START_X + TRACK_END_X) / 2  # 0.270
TRACK_CZ = BASE_TOP_Z + TRACK_WALL_H / 2  # 0.7675 m

# Inner channel stops along x:
CHANNEL_NEAR_STOP_X = TRACK_START_X + TRACK_END_WALL_T  # 0.205 m (inner face of near end stop)
CHANNEL_FAR_STOP_X = TRACK_END_X - TRACK_END_WALL_T  # 0.335 m (inner face of far end stop)

# Channel floor top z (knobs rest on floor):
KNOB_BOTTOM_Z = BASE_TOP_Z + TRACK_FLOOR_T  # 0.7595 m

# -- Knobs (Loose Sliders) --------------------------------------------------------------------------
KNOB_L = 0.030  # 30 mm along x
KNOB_W = 0.030  # 30 mm along y
KNOB_H = 0.014  # 14 mm along z
KNOB_SIZE = (KNOB_L, KNOB_W, KNOB_H)
KNOB_MASS = 0.080  # 80 g
KNOB_FRICTION = 0.5  # Coulomb friction 0.5 for tipping stability
KNOB_TRAVEL = 0.100  # 100 mm travel along track

# Lateral clearance per side in 36 mm channel: (36 - 30)/2 = 3.0 mm
KNOB_LATERAL_CLEARANCE = (TRACK_CHANNEL_W - KNOB_W) / 2  # 0.003 m

# Knob center z rests on channel floor:
KNOB_CZ = KNOB_BOTTOM_Z + KNOB_H / 2  # 0.7665 m

# Knob setting s in [0, 100] mm:
# Setting 0 mm: knob rear face against near stop -> center x = 0.205 + 0.015 = 0.220 m
# Setting 100 mm: knob front face against far stop -> center x = 0.335 - 0.015 = 0.320 m
KNOB_ZERO_CX = CHANNEL_NEAR_STOP_X + KNOB_L / 2  # 0.220 m


def knob_cx_from_setting_mm(s_mm: float) -> float:
    """Return world x coordinate (m) for a slider setting in [0, 100] mm."""
    return KNOB_ZERO_CX + float(s_mm) / 1000.0


def knob_setting_mm_from_cx(cx: float) -> float:
    """Return slider setting in mm from world x coordinate (m)."""
    return (float(cx) - KNOB_ZERO_CX) * 1000.0


# Start settings: Left starts below target (30 mm), Right starts above target (70 mm)
KNOB_START_SETTING_MM = {"left": 30.0, "right": 70.0}
START_POS = {
    "left": (knob_cx_from_setting_mm(KNOB_START_SETTING_MM["left"]), TRACK_Y["left"], KNOB_CZ),
    "right": (knob_cx_from_setting_mm(KNOB_START_SETTING_MM["right"]), TRACK_Y["right"], KNOB_CZ),
}

# Target setpoint settings (Left = 65 mm, Right = 40 mm)
SETPOINT_MM = {"left": 65.0, "right": 40.0}
TARGET_BAND_TOL_MM = 2.5  # setpoint +- 2.5 mm displayed

# Backlash hysteresis parameter (6 mm deadband and following lag)
BACKLASH_DEADBAND_MM = 6.0

# -- LOCK Buttons ----------------------------------------------------------------------------------
# Pushbutton cap 36 x 36 x 12 mm, travel 8 mm along Z, spring 100 N/m, damping 4 N s/m, mass 20 g
BUTTON_CAP_L = 0.036
BUTTON_CAP_W = 0.036
BUTTON_CAP_H = 0.012
BUTTON_CAP_SIZE = (BUTTON_CAP_L, BUTTON_CAP_W, BUTTON_CAP_H)
BUTTON_MASS = 0.020
BUTTON_TRAVEL = 0.008  # 8 mm travel
BUTTON_RATE = 100.0  # 100 N/m spring rate (0.8 N at 8 mm)
BUTTON_DAMPING = 4.0  # 4 N s/m
BUTTON_AXIS = "Z"

# Button sleeve / pocket ring: 44 x 44 x 16 mm (walls 3 mm, inner 38 x 38 mm -> 1 mm clearance per side to cap)
BUTTON_BASE_L = 0.044
BUTTON_BASE_W = 0.044
BUTTON_BASE_H = 0.016
BUTTON_BASE_SIZE = (BUTTON_BASE_L, BUTTON_BASE_W, BUTTON_BASE_H)
POCKET_WALL_T = 0.003

# Button positions beside tracks: Left at y = +0.075, Right at y = -0.075, x = 0.270
BUTTON_POS_XY = {
    "left": (0.270, +0.075),
    "right": (0.270, -0.075),
}
BUTTON_BASE_CZ = BASE_TOP_Z + BUTTON_BASE_H / 2  # 0.7655 m
BUTTON_BASE_POS = {
    side: (BUTTON_POS_XY[side][0], BUTTON_POS_XY[side][1], BUTTON_BASE_CZ)
    for side in ("left", "right")
}

# Cap rest position (top at BASE_TOP_Z + 0.024 m = 0.7815 m):
BUTTON_CAP_REST_CZ = BASE_TOP_Z + BUTTON_BASE_H + BUTTON_CAP_H / 2 - 0.004  # 0.7755 m
BUTTON_CAP_TOP_REST_Z = BUTTON_CAP_REST_CZ + BUTTON_CAP_H / 2  # 0.7815 m
LOCK_BTN_POS = {
    side: (BUTTON_POS_XY[side][0], BUTTON_POS_XY[side][1], BUTTON_CAP_REST_CZ)
    for side in ("left", "right")
}

# Slider rest position in fixture frame (relative to BASE_POS):
LOCK_BTN_LOCAL_REST = {
    side: (
        BUTTON_POS_XY[side][0] - BASE_CX,
        BUTTON_POS_XY[side][1] - BASE_CY,
        BUTTON_CAP_REST_CZ - BASE_CZ,
    )
    for side in ("left", "right")
}

# Button press parameters
BUTTON_TRIGGER_TRAVEL = 0.0064  # <= -6.4 mm (80% travel)
BUTTON_RELEASE_TRAVEL = 0.0035  # >= -3.5 mm re-arm
BUTTON_PRESS_OVERSHOOT = 0.005  # 5 mm commanded past end stop
BUTTON_PRESS_DEPTH = BUTTON_TRAVEL + BUTTON_PRESS_OVERSHOOT  # 13 mm below resting cap top
ARM_Z_STIFFNESS = 319.6  # N/m


def button_equilibrium_travel(depth_cmd: float = BUTTON_PRESS_DEPTH, k_arm: float = ARM_Z_STIFFNESS,
                              k_spring: float = BUTTON_RATE, travel: float = BUTTON_TRAVEL) -> float:
    """Static cap travel (m) when TCP is commanded ``depth_cmd`` below resting cap top."""
    free = depth_cmd * k_arm / (k_arm + k_spring)
    return min(free, travel)


# -- 10-Lamp Bars and Indicator Housings ------------------------------------------------------------
# Meter housings above rails at x = 0.380 m, centered in y at TRACK_Y[side]
METER_L = 0.040  # 40 mm along x
METER_W = 0.120  # 120 mm along y
METER_H = 0.020  # 20 mm along z
METER_CX = 0.380
METER_CZ = BASE_TOP_Z + METER_H / 2  # 0.7675 m
METER_TOP_Z = BASE_TOP_Z + METER_H  # 0.7775 m

METER_POS = {
    side: (METER_CX, TRACK_Y[side], METER_CZ)
    for side in ("left", "right")
}
METER_SIZE = (METER_L, METER_W, METER_H)

# 10 lamps per bar along y on top of meter housing:
# Left bar: spans y in [+0.100, +0.200] m (0 to 100 mm displayed)
# Right bar: spans y in [-0.200, -0.100] m (0 to 100 mm displayed)
LAMP_PAD_LX = 0.020  # 20 mm along x
LAMP_PAD_LY = 0.008  # 8 mm along y
LAMP_PAD_LZ = 0.003  # 3 mm along z
LAMP_PAD_SIZE = (LAMP_PAD_LX, LAMP_PAD_LY, LAMP_PAD_LZ)
LAMP_CZ = METER_TOP_Z + LAMP_PAD_LZ / 2  # 0.7790 m

STOWED_Z = TOP - 0.100  # stowed under table when off (0.6475 m)


def bar_origin_y(side: str) -> float:
    """Return y coordinate of 0 mm mark for side's lamp bar."""
    return TRACK_Y[side] - 0.050 if side == "left" else TRACK_Y[side] - 0.050


def lamp_pos_world(side: str, k: int) -> tuple[float, float, float]:
    """World position of lamp k (1..10) on side's bar. k corresponds to 10*k mm displayed."""
    # Lamp k center at (k - 0.5) * 10 mm from bar origin
    y0 = bar_origin_y(side)
    y = y0 + (k - 0.5) * 0.010
    return (METER_CX, y, LAMP_CZ)


def target_lamp_pos_world(side: str) -> tuple[float, float, float]:
    """World position of white target marker lamp on side's bar."""
    y0 = bar_origin_y(side)
    y = y0 + SETPOINT_MM[side] / 1000.0
    return (METER_CX, y, LAMP_CZ + 0.001)


TARGET_LAMP_SIZE = (0.024, 0.008, 0.004)

# Locked indicator lamp (green dome on meter housing):
LOCKED_LAMP_R = 0.012  # diameter 24 mm
LOCKED_LAMP_H = 0.004
LOCKED_LAMP_POS = {
    side: (METER_CX + 0.012, TRACK_Y[side], METER_TOP_Z + LOCKED_LAMP_H / 2)
    for side in ("left", "right")
}


def _box(centre, size, color):
    return (centre, size, color)


def fixture_named_boxes_world() -> tuple:
    """(name, centre, size, colour) of every fixed box of fixture in WORLD frame."""
    boxes = []

    def add(name, centre, size, color):
        boxes.append((name, centre, size, color))

    add("base plate", BASE_POS, BASE_SIZE, DARK_GREY)

    for side in ("left", "right"):
        ty = TRACK_Y[side]
        # Track floor bands (every 20 mm)
        band_starts = [0.0, 0.020, 0.040, 0.060, 0.080, 0.100]
        band_lens = [0.020, 0.020, 0.020, 0.020, 0.020, 0.030]
        for b_start, b_len, b_color in zip(band_starts, band_lens, BAND_COLORS, strict=True):
            bx = CHANNEL_NEAR_STOP_X + b_start + b_len / 2
            add(
                f"{side} track floor band {int(b_start * 1000)}-{int((b_start + b_len) * 1000)} mm",
                (bx, ty, BASE_TOP_Z + TRACK_FLOOR_T / 2),
                (b_len, TRACK_CHANNEL_W, TRACK_FLOOR_T),
                b_color,
            )
        # Side walls
        wall_py = ty + TRACK_CHANNEL_W / 2 + TRACK_WALL_T / 2
        add(f"{side} track wall (+y)", (TRACK_CX, wall_py, TRACK_CZ), (TRACK_L, TRACK_WALL_T, TRACK_WALL_H), DARK_GREY)
        wall_my = ty - TRACK_CHANNEL_W / 2 - TRACK_WALL_T / 2
        add(f"{side} track wall (-y)", (TRACK_CX, wall_my, TRACK_CZ), (TRACK_L, TRACK_WALL_T, TRACK_WALL_H), DARK_GREY)
        # End stop walls
        near_cx = TRACK_START_X + TRACK_END_WALL_T / 2
        add(f"{side} track near end stop", (near_cx, ty, TRACK_CZ), (TRACK_END_WALL_T, TRACK_CHANNEL_W, TRACK_WALL_H), DARK_GREY)
        far_cx = TRACK_END_X - TRACK_END_WALL_T / 2
        add(f"{side} track far end stop", (far_cx, ty, TRACK_CZ), (TRACK_END_WALL_T, TRACK_CHANNEL_W, TRACK_WALL_H), DARK_GREY)

        # LOCK button pocket ring: 44 x 44 x 16 mm, 3 mm walls
        bx, by = BUTTON_POS_XY[side]
        ring_z = BASE_TOP_Z + BUTTON_BASE_H / 2
        ox = BUTTON_BASE_L / 2 - POCKET_WALL_T / 2
        oy = BUTTON_BASE_W / 2 - POCKET_WALL_T / 2
        inner_l = BUTTON_BASE_L - 2 * POCKET_WALL_T
        for sx, nm in ((-1, "near"), (1, "far")):
            add(f"{side} LOCK pocket ring ({nm} wall)", (bx + sx * ox, by, ring_z), (POCKET_WALL_T, BUTTON_BASE_W, BUTTON_BASE_H), DARK_GREY)
        for sy, nm in ((-1, "-y"), (1, "+y")):
            add(f"{side} LOCK pocket ring ({nm} wall)", (bx, by + sy * oy, ring_z), (inner_l, POCKET_WALL_T, BUTTON_BASE_H), DARK_GREY)

        # Meter housing above rail
        add(f"{side} meter housing", METER_POS[side], METER_SIZE, DARK_GREY)

        # Fixed target band tick mark on housing (dark tick at setpoint position)
        add(f"{side} target band tick", target_lamp_pos_world(side), (0.024, 0.005, 0.001), DARK)

    return tuple(boxes)


def fixture_boxes_world() -> tuple:
    """Return all fixed visual/collision boxes comprising the rigid station fixture in WORLD frame."""
    return tuple(_box(c, sz, col) for _n, c, sz, col in fixture_named_boxes_world())


def fixture_boxes() -> tuple:
    """Return visual/collision boxes for fixture Part in LOCAL part frame (relative to BASE_POS)."""
    world = fixture_boxes_world()
    bx, by, bz = BASE_POS
    local = []
    for centre, size, color in world:
        cx, cy, cz = centre
        local_pos = (cx - bx, cy - by, cz - bz)
        local.append(_box(local_pos, size, color))
    return tuple(local)


def knob_boxes(color: tuple[float, float, float]) -> tuple:
    """Return visual/collision boxes for a loose knob."""
    return (
        ((0.0, 0.0, 0.0), KNOB_SIZE, color),
    )


def lamp_boxes(color: tuple[float, float, float] = CYAN) -> tuple:
    """Return visual boxes for an indicator lamp pad."""
    return (
        ((0.0, 0.0, 0.0), LAMP_PAD_SIZE, color),
    )


def target_marker_boxes() -> tuple:
    """Return visual boxes for the white target marker lamp."""
    return (
        ((0.0, 0.0, 0.0), TARGET_LAMP_SIZE, BRIGHT_WHITE),
    )
