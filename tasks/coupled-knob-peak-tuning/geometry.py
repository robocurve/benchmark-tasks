"""Every dimension of the coupled knob peak tuning task in one place. Pure Python.

World frame (the station's): x forward from the arm bases, y left, z up, metres.
The left arm base is at (0, +0.305), the right at (0, -0.305).
Table top z = 0.7475 m (TOP).

The station contains:
- A fixed base plate (300 x 200 x 10 mm, mass 5.0 kg, friction 1.0, dark grey).
- Two parallel tracks (channels): 140 mm long x 36 mm wide, 20 mm tall walls (6 mm thick), 120 mm apart.
- 20 mm wide alternating light / blue position bands along the channel floors (band edges at 0, 20, 40, ... 100 mm).
- Two knobs (30 x 30 x 14 mm, orange and teal, mass 80 g, friction 0.5), sliding 100 mm along tracks.
- A fixed meter housing (200 x 40 x 30 mm, dark grey).
- A scale bar (160 x 24 x 4 mm, white) with a green target band 36 mm deep over its last 8 mm (152 to 160 mm).
- A bright red needle (34 x 12 x 8 mm) moving along the scale bar via write_pose.
- A spring-loaded READ button in a pocket ring (cap 40 x 40 x 12 mm, travel 8 mm, spring 100 N/m, damping 4 N s/m, 20 g).
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
RED = (0.85, 0.15, 0.12)
BRIGHT_RED = (0.95, 0.10, 0.10)
GREEN = (0.12, 0.85, 0.18)
WHITE = (0.92, 0.92, 0.92)

# Band marker colours along track floors (every 20 mm)
BAND_LIGHT = (0.90, 0.90, 0.90)
BAND_BLUE = (0.20, 0.35, 0.80)
BAND_COLORS = (
    BAND_BLUE,   # 0-20 mm
    BAND_LIGHT,  # 20-40 mm (the start setting 30 mm sits here)
    BAND_BLUE,   # 40-60 mm
    BAND_LIGHT,  # 60-80 mm
    BAND_BLUE,   # 80-100 mm (a knob rear face at 80 mm sits exactly on the light/blue edge)
    GREY,        # 100-130 mm (far end zone)
)

# Clearances and tolerances
CLEAR_MIN = 0.001  # minimum 1.0 mm clearance per side
PAD_TO_TCP = 0.013  # 13 mm distance from pad bottom to TCP z (measured 12.84 mm from the collision hulls)

# Closed-finger half-depth along x (the pushing direction) versus height above the pad bottom, measured as the cross
# section of the union of the real collision hulls (tip_left, tip_right, gripper) at the push pose, 0.25 mm grid
# (scratch script prof.py, see FIX_LOG.md). The fingers are a WEDGE: wider higher up, so a pushed block is first touched
# at its TOP edge, not at the pad bottom.
FINGER_HALF_DEPTH_MM = ((0.5, 6.5), (1.0, 7.0), (2.0, 7.75), (3.0, 8.0), (5.0, 8.75), (7.5, 9.75), (10.0, 10.75),
                        (12.5, 11.75), (15.0, 12.75), (17.5, 13.75), (20.0, 14.75), (22.5, 15.75), (25.0, 16.5),
                        (30.0, 18.5))


def finger_half_depth(h_above_pad_bottom: float) -> float:
    """Closed-finger half-depth in x (metres) at height ``h`` (metres) above the pad bottom (linear interpolation)."""
    hs = [h for h, _ in FINGER_HALF_DEPTH_MM]
    ws = [w for _, w in FINGER_HALF_DEPTH_MM]
    return float(np.interp(h_above_pad_bottom * 1000.0, hs, ws)) / 1000.0


PUSH_PAD_HEIGHT = 0.0025  # pad bottom height above the channel floor while pushing
PUSH_APPROACH_CLEAR = 0.003  # gap between the finger and the knob face when descending beside a knob
PUSH_OVERSHOOT = 0.00145  # the command goes this far beyond the nominal contact (half the 2.9 mm stall lag of the arm against friction)


# Arm compliance at the push pose (J K^-1 J^T, scratch comp2.py; left/right arm differ by < 0.4 mm/N), mm per N:
ARM_X_COMPLIANCE = 5.9  # along the push direction
ARM_ZX_COUPLING = 0.95  # vertical deflection per N of horizontal force
ARM_Z_COMPLIANCE_PUSH = 2.0  # vertical, at the push pose (3.13 at the press pose, see ARM_Z_STIFFNESS)
FINGER_FACE_SLOPE = 0.4  # dx/dh of the wedge face (7.5 mm + 0.4 h half-depth), so the face leans 21.8 degrees


def push_force_model(mu: float | None = None, mass: float | None = None) -> dict:
    """Quasi-static push of a knob by the slanted finger face. Friction acts on the weight plus the face's downward push.

    Solves N cos(a) = mu (m g + N sin(a)) for the contact normal N (a = face angle). Returns forces in N and lengths in mm.
    """
    mu = KNOB_FRICTION if mu is None else mu
    mass = KNOB_MASS if mass is None else mass
    a = math.atan(FINGER_FACE_SLOPE)
    w = mass * 9.81
    n = mu * w / (math.cos(a) - mu * math.sin(a))
    f_h = n * math.cos(a)  # horizontal force the arm must supply
    f_up = n * math.sin(a)  # upward reaction on the finger
    lift = ARM_Z_COMPLIANCE_PUSH * f_up + ARM_ZX_COUPLING * f_h  # mm, finger rises (worst case, both add)
    lag = ARM_X_COMPLIANCE * f_h  # mm, the finger stalls this far short of its command
    return dict(normal=n, horizontal=f_h, finger_lift_mm=lift, stall_lag_mm=lag)


def push_contact_half_depth() -> float:
    """Half-depth of the finger at the knob's top edge, which is where a closed-finger push touches the knob."""
    return finger_half_depth(KNOB_H - PUSH_PAD_HEIGHT)

# -- Base Plate -------------------------------------------------------------------------------------
BASE_L = 0.300  # length along x (300 mm)
BASE_W = 0.200  # width along y (200 mm)
BASE_H = 0.010  # thickness along z (10 mm)
BASE_CX = 0.300  # center x
BASE_CY = 0.000  # center y
BASE_CZ = TOP + BASE_H / 2  # center z (0.7525 m)
BASE_POS = (BASE_CX, BASE_CY, BASE_CZ)
BASE_SIZE = (BASE_L, BASE_W, BASE_H)
BASE_MASS = 5.0  # kg
BASE_FRICTION = 1.0

# Base bounds: x in [0.150, 0.450], y in [-0.100, 0.100], z in [TOP, TOP + 0.010]
BASE_TOP_Z = TOP + BASE_H  # 0.7575 m

# -- Tracks (Channels) -----------------------------------------------------------------------------
# Two parallel tracks, 120 mm apart in y: left track at y = +0.060, right track at y = -0.060
TRACK_SPACING_Y = 0.120
TRACK_Y = {"left": +0.060, "right": -0.060}

TRACK_L = 0.140  # 140 mm total outer length along x
TRACK_CHANNEL_W = 0.036  # 36 mm inner channel width
TRACK_WALL_T = 0.006  # 6 mm side wall thickness
TRACK_WALL_H = 0.020  # 20 mm wall height above base plate
TRACK_END_WALL_T = 0.005  # 5 mm end stop wall thickness
TRACK_FLOOR_T = 0.002  # 2 mm channel floor thickness

TRACK_OUTER_W = TRACK_CHANNEL_W + 2 * TRACK_WALL_T  # 48 mm
TRACK_INNER_L = TRACK_L - 2 * TRACK_END_WALL_T  # 130 mm (sliding space)

# Track bounds along x: [0.170, 0.310], center x = 0.240
TRACK_START_X = 0.170
TRACK_END_X = TRACK_START_X + TRACK_L  # 0.310
TRACK_CX = (TRACK_START_X + TRACK_END_X) / 2  # 0.240
TRACK_CZ = BASE_TOP_Z + TRACK_WALL_H / 2  # 0.7675 m

# Inner channel stops along x:
CHANNEL_NEAR_STOP_X = TRACK_START_X + TRACK_END_WALL_T  # 0.175 m (inner face of near end stop)
CHANNEL_FAR_STOP_X = TRACK_END_X - TRACK_END_WALL_T  # 0.305 m (inner face of far end stop)

# Channel floor top z (knobs rest on this floor):
KNOB_BOTTOM_Z = BASE_TOP_Z + TRACK_FLOOR_T  # 0.7595 m

# -- Knobs (Loose Sliders) --------------------------------------------------------------------------
KNOB_L = 0.030  # 30 mm along x
KNOB_W = 0.030  # 30 mm along y
KNOB_H = 0.014  # 14 mm along z (low, so the wedge-shaped closed fingers push it at 14 mm height, see FINGER_HALF_DEPTH_MM)
KNOB_SIZE = (KNOB_L, KNOB_W, KNOB_H)
KNOB_MASS = 0.080  # 80 g
KNOB_FRICTION = 0.5  # Coulomb friction 0.5 for tipping stability
KNOB_TRAVEL = 0.100  # 100 mm travel along track

# Lateral clearance per side in 36 mm channel: (36 - 30)/2 = 3.0 mm
KNOB_LATERAL_CLEARANCE = (TRACK_CHANNEL_W - KNOB_W) / 2  # 0.003 m

# Knob z rests cleanly on top of 2 mm channel floor (0.7595 m):
KNOB_CZ = KNOB_BOTTOM_Z + KNOB_H / 2  # 0.7720 m (eliminates 2 mm floor penetration at spawn)
# Top of knob is at KNOB_BOTTOM_Z + 0.014 = 0.7735 m (4 mm below the 20 mm wall tops)

# Knob setting s in [0, 100] mm:
# Distance from inner face of near stop (0.175 m) to knob's rear face:
# Setting 0 mm: knob rear face against near stop -> rear face x = 0.175, center x = 0.175 + 0.015 = 0.190 m
# Setting 100 mm: knob front face against far stop -> front face x = 0.305, center x = 0.305 - 0.015 = 0.290 m
KNOB_ZERO_CX = CHANNEL_NEAR_STOP_X + KNOB_L / 2  # 0.190 m

def knob_cx_from_setting_mm(s_mm: float) -> float:
    """Return world x coordinate (m) for a slider setting in [0, 100] mm."""
    return KNOB_ZERO_CX + s_mm / 1000.0

def knob_setting_mm_from_cx(cx: float) -> float:
    """Return slider setting in mm from world x coordinate (m)."""
    return (cx - KNOB_ZERO_CX) * 1000.0

# Start positions at s = 30 mm (0.030 m gap from near stop, 14.6 mm clear of 15.4 mm closed fingers):
KNOB_START_SETTING_MM = 30.0
KNOB_START_CX = knob_cx_from_setting_mm(KNOB_START_SETTING_MM)  # 0.215 m
START_POS = {
    "left": (KNOB_START_CX, TRACK_Y["left"], KNOB_CZ),
    "right": (KNOB_START_CX, TRACK_Y["right"], KNOB_CZ),
}

# Target peak settings (x0=80 mm, y0=50 mm):
PEAK_SETTING_MM = {"left": 80.0, "right": 50.0}
PEAK_POS = {
    "left": (knob_cx_from_setting_mm(PEAK_SETTING_MM["left"]), TRACK_Y["left"], KNOB_CZ),
    "right": (knob_cx_from_setting_mm(PEAK_SETTING_MM["right"]), TRACK_Y["right"], KNOB_CZ),
}

# -- Meter Housing and Scale Bar -------------------------------------------------------------------
METER_L = 0.040  # 40 mm along x
METER_W = 0.200  # 200 mm along y
METER_H = 0.030  # 30 mm along z
METER_CX = 0.400  # center x
METER_CY = 0.000  # center y
METER_CZ = BASE_TOP_Z + METER_H / 2  # 0.7725 m
METER_POS = (METER_CX, METER_CY, METER_CZ)
METER_SIZE = (METER_L, METER_W, METER_H)
METER_TOP_Z = BASE_TOP_Z + METER_H  # 0.7875 m

# Scale Bar on top face of meter housing:
SCALE_L = 0.024  # 24 mm along x (across the bar; was 12 mm, doubled for the 320x176 camera)
SCALE_GREEN_L = 0.036  # the green band is 36 mm deep in x so it stands out on both sides of the bar
SCALE_W = 0.160  # 160 mm total length along y
SCALE_H = 0.004  # 4 mm tall along z
SCALE_CX = METER_CX  # 0.400 m
SCALE_CZ = METER_TOP_Z + SCALE_H / 2  # 0.7895 m
SCALE_TOP_Z = METER_TOP_Z + SCALE_H  # 0.7915 m

# The scale bar extends in y from -0.080 m (0 mm mark) to +0.080 m (160 mm mark)
SCALE_ORIGIN_Y = -SCALE_W / 2  # -0.080 m
SCALE_GREEN_START_MM = 152.0  # green target band over last 8 mm (152 to 160 mm)
SCALE_GREEN_LEN_MM = 8.0

# White section of scale (0 to 152 mm):
SCALE_WHITE_LEN = 0.152  # m
SCALE_WHITE_CY = SCALE_ORIGIN_Y + SCALE_WHITE_LEN / 2  # -0.004 m
SCALE_WHITE_SIZE = (SCALE_L, SCALE_WHITE_LEN, SCALE_H)
SCALE_WHITE_POS = (SCALE_CX, SCALE_WHITE_CY, SCALE_CZ)

# Green target band section of scale (152 to 160 mm, spans y in [+0.072, +0.080] m):
SCALE_GREEN_LEN = 0.008  # m
SCALE_GREEN_CY = SCALE_ORIGIN_Y + SCALE_WHITE_LEN + SCALE_GREEN_LEN / 2  # +0.076 m
SCALE_GREEN_SIZE = (SCALE_GREEN_L, SCALE_GREEN_LEN, SCALE_H)
SCALE_GREEN_POS = (SCALE_CX, SCALE_GREEN_CY, SCALE_CZ)

# -- Needle -----------------------------------------------------------------------------------------
NEEDLE_LX = 0.034  # 34 mm along x (crosses the 24 mm scale bar)
NEEDLE_LY = 0.012  # 12 mm along y (widened indicator width for 320x176 camera visibility)
NEEDLE_LZ = 0.008  # 8 mm along z (height)
NEEDLE_SIZE = (NEEDLE_LX, NEEDLE_LY, NEEDLE_LZ)
NEEDLE_MASS = 0.010  # 10 g
NEEDLE_CX = SCALE_CX  # 0.400 m
NEEDLE_CZ = SCALE_TOP_Z + NEEDLE_LZ / 2  # 0.7955 m

# Needle displacement: needle center moves r * 160 mm along scale bar from zero mark.
# At r = 0.0, needle center sits at SCALE_ORIGIN_Y = -0.080 m (0 mm mark)
# At r = 0.95, needle center sits at -0.080 + 0.95 * 0.160 = +0.072 m (152 mm: EXACT left edge of green band!)
# At r = 1.0, needle center sits at -0.080 + 0.160 = +0.080 m (160 mm mark: right edge of green band)
NEEDLE_ZERO_CY = SCALE_ORIGIN_Y  # -0.080 m
NEEDLE_MAX_TRAVEL = 0.160  # 160 mm travel for r in [0, 1]

def needle_pos_from_r(r: float) -> tuple[float, float, float]:
    """Return world pose (x, y, z) for the needle given reading r in [0, 1]."""
    r_clamped = max(0.0, min(1.0, float(r)))
    ny = NEEDLE_ZERO_CY + r_clamped * NEEDLE_MAX_TRAVEL
    return (NEEDLE_CX, ny, NEEDLE_CZ)

STOWED_Z = -0.500  # below-table stowing position when hidden

# -- READ Button -----------------------------------------------------------------------------------
# Pushbutton cap 40 x 40 x 12 mm, travel 8 mm along Z, spring rate 625 N/m, damping 10 N s/m, mass 20 g
BUTTON_CAP_L = 0.040  # 40 mm along x
BUTTON_CAP_W = 0.040  # 40 mm along y
BUTTON_CAP_H = 0.012  # 12 mm along z
BUTTON_CAP_SIZE = (BUTTON_CAP_L, BUTTON_CAP_W, BUTTON_CAP_H)
BUTTON_MASS = 0.020  # 20 g
BUTTON_TRAVEL = 0.008  # 8 mm travel
BUTTON_RATE = 100.0  # 100 N/m spring rate (0.8 N at 8 mm); was 625 N/m, softer than the arm (319 N/m) so the press registers
BUTTON_DAMPING = 4.0  # 4 N s/m damping (critical damping for 20 g at 100 N/m is 2.8)
BUTTON_AXIS = "Z"

# Button sleeve / pedestal on base plate:
BUTTON_BASE_L = 0.048
BUTTON_BASE_W = 0.048
BUTTON_BASE_H = 0.016
BUTTON_BASE_CX = 0.335  # centered between tracks (0.310) and meter housing (0.380)
BUTTON_BASE_CY = 0.000  # centered in y between the two tracks (y = +/- 0.060)
BUTTON_BASE_CZ = BASE_TOP_Z + BUTTON_BASE_H / 2  # 0.7655 m
BUTTON_BASE_POS = (BUTTON_BASE_CX, BUTTON_BASE_CY, BUTTON_BASE_CZ)
BUTTON_BASE_SIZE = (BUTTON_BASE_L, BUTTON_BASE_W, BUTTON_BASE_H)
BUTTON_BASE_TOP_Z = BASE_TOP_Z + BUTTON_BASE_H  # 0.7735 m

# Cap rest position (sitting in sleeve, top at BASE_TOP_Z + 0.024 m = 0.7815 m):
BUTTON_CAP_REST_CZ = BASE_TOP_Z + BUTTON_BASE_H + BUTTON_CAP_H / 2 - 0.004  # 0.7755 m
BUTTON_CAP_TOP_REST_Z = BUTTON_CAP_REST_CZ + BUTTON_CAP_H / 2  # 0.7815 m
READ_BTN_POS = (BUTTON_BASE_CX, BUTTON_BASE_CY, BUTTON_CAP_REST_CZ)

# Slider rest position in part frame (relative to BASE_POS):
READ_BTN_LOCAL_REST = (
    BUTTON_BASE_CX - BASE_CX,
    BUTTON_BASE_CY - BASE_CY,
    BUTTON_CAP_REST_CZ - BASE_CZ,
)

# Press parameters
BUTTON_TRIGGER_TRAVEL = 0.0064  # logic.BUTTON_PRESSED_THRESHOLD, repeated here for the compliance model and its test
BUTTON_PRESS_OVERSHOOT = 0.005  # the pad is commanded this far below the END STOP (cap top at full travel)
BUTTON_PRESS_DEPTH = BUTTON_TRAVEL + BUTTON_PRESS_OVERSHOOT  # 13 mm commanded below the resting cap top
# The cap travel is set by the END STOP (the slider's 8 mm limit; the cap top then sits level with the pocket ring top),
# NOT by the arm: at the press pose the arm is only ARM_Z_STIFFNESS stiff, softer than the old 625 N/m spring.
ARM_Z_STIFFNESS = 319.6  # N/m, vertical TCP stiffness at the press pose: J K^-1 J^T with the joint gains of
# yam_arena/constants.py (shoulder 80 N m/rad, wrist 10 N m/rad); 3.128 mm/N, same for both arms (scratch comp.py)
POCKET_WALL_T = 0.003  # the pocket ring around the cap: 48 mm outside, 42 mm inside (1 mm gap per side to the cap)


def button_equilibrium_travel(depth_cmd: float = BUTTON_PRESS_DEPTH, k_arm: float = ARM_Z_STIFFNESS,
                              k_spring: float = BUTTON_RATE, travel: float = BUTTON_TRAVEL) -> float:
    """Static cap travel (m) when the TCP is commanded ``depth_cmd`` below the resting cap top.

    Series spring model: arm (k_arm) pushes the cap against its return spring; the end stop caps the travel at ``travel``.
    """
    free = depth_cmd * k_arm / (k_arm + k_spring)
    return min(free, travel)


def _box(centre, size, color):
    return (centre, size, color)


def fixture_named_boxes_world() -> tuple:
    """(name, centre, size, colour) of every fixed box of the fixture in WORLD frame (names are used by the checks)."""
    boxes = []

    def add(name, centre, size, color):
        boxes.append((name, centre, size, color))

    add("base plate", BASE_POS, BASE_SIZE, DARK_GREY)

    for side in ("left", "right"):
        ty = TRACK_Y[side]
        band_starts = [0.0, 0.020, 0.040, 0.060, 0.080, 0.100]  # bands start at the near stop inner face
        band_lens = [0.020, 0.020, 0.020, 0.020, 0.020, 0.030]  # total 130 mm
        for b_start, b_len, b_color in zip(band_starts, band_lens, BAND_COLORS, strict=True):
            bx = CHANNEL_NEAR_STOP_X + b_start + b_len / 2
            add(f"{side} track floor band {int(b_start * 1000)}-{int((b_start + b_len) * 1000)} mm", (bx, ty, BASE_TOP_Z + TRACK_FLOOR_T / 2),
                (b_len, TRACK_CHANNEL_W, TRACK_FLOOR_T), b_color)
        wall_py = ty + TRACK_CHANNEL_W / 2 + TRACK_WALL_T / 2
        add(f"{side} track wall (+y)", (TRACK_CX, wall_py, TRACK_CZ), (TRACK_L, TRACK_WALL_T, TRACK_WALL_H), DARK_GREY)
        wall_my = ty - TRACK_CHANNEL_W / 2 - TRACK_WALL_T / 2
        add(f"{side} track wall (-y)", (TRACK_CX, wall_my, TRACK_CZ), (TRACK_L, TRACK_WALL_T, TRACK_WALL_H), DARK_GREY)
        near_cx = TRACK_START_X + TRACK_END_WALL_T / 2
        add(f"{side} track near end stop", (near_cx, ty, TRACK_CZ), (TRACK_END_WALL_T, TRACK_CHANNEL_W, TRACK_WALL_H), DARK_GREY)
        far_cx = TRACK_END_X - TRACK_END_WALL_T / 2
        add(f"{side} track far end stop", (far_cx, ty, TRACK_CZ), (TRACK_END_WALL_T, TRACK_CHANNEL_W, TRACK_WALL_H), DARK_GREY)

    # READ pocket ring: the cap slides inside it; nothing solid under the cap, so the cap never overlaps another body
    ring_z = BASE_TOP_Z + BUTTON_BASE_H / 2
    ox = BUTTON_BASE_L / 2 - POCKET_WALL_T / 2
    oy = BUTTON_BASE_W / 2 - POCKET_WALL_T / 2
    inner_l = BUTTON_BASE_L - 2 * POCKET_WALL_T
    for sx, nm in ((-1, "near"), (1, "far")):
        add(f"READ pocket ring ({nm} wall)", (BUTTON_BASE_CX + sx * ox, BUTTON_BASE_CY, ring_z), (POCKET_WALL_T, BUTTON_BASE_W, BUTTON_BASE_H), DARK_GREY)
    for sy, nm in ((-1, "-y"), (1, "+y")):
        add(f"READ pocket ring ({nm} wall)", (BUTTON_BASE_CX, BUTTON_BASE_CY + sy * oy, ring_z), (inner_l, POCKET_WALL_T, BUTTON_BASE_H), DARK_GREY)

    add("meter housing", METER_POS, METER_SIZE, DARK_GREY)
    add("scale bar (white)", SCALE_WHITE_POS, SCALE_WHITE_SIZE, WHITE)
    add("scale bar (green band)", SCALE_GREEN_POS, SCALE_GREEN_SIZE, GREEN)
    return tuple(boxes)


def fixture_boxes_world() -> tuple:
    """Return all fixed visual/collision boxes comprising the rigid station fixture in WORLD frame."""
    return tuple(_box(c, sz, col) for _n, c, sz, col in fixture_named_boxes_world())


def fixture_boxes() -> tuple:
    """Return visual/collision boxes for the fixture Part in LOCAL part frame (relative to BASE_POS)."""
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


def needle_boxes() -> tuple:
    """Return visual/collision boxes for the bright red meter needle."""
    return (
        ((0.0, 0.0, 0.0), NEEDLE_SIZE, BRIGHT_RED),
    )
