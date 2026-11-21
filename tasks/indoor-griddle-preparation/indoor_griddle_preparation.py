"""On the kitchen counter: a tiled back splashback wall with a closed casement window and surface-mounted
receptacle outlet on the right; two slices of bread on a ceramic serving plate; and an electric griddle (1.19 kg
cast-aluminium body, 21.6 x 26.7 cm cooking surface, unheated) with its detachable Control Master heat control probe
resting unseated in its guide cradle on the near side, connected to the wall outlet and switched OFF.

The instruction asks to brace the electric griddle by its handle to seat the heat control probe, turn the dial on,
and place both slices of bread on its cooking surface. Because the lightweight griddle rests on low-friction feet,
pushing the detachable probe into the receptacle exerts horizontal force that slides an unbraced pan across the
counter, requiring bimanual coordination to brace the cool-touch handle while seating the probe home. After seating,
the thermostat dial is rotated into its active heating range (>= 15 deg), and both bread slices are transferred
flat and non-overlapping onto the cooking surface with both robot arms retracted clear.

Run it:
    inspect-robots run --embodiment yam_arena -E task=tasks/indoor-griddle-preparation/indoor_griddle_preparation.py \\
      -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \\
      --instruction "Brace the electric griddle by its handle to seat the heat control probe, turn the dial on, and place both slices of bread on its cooking surface." \\
      --scorer task_score --max-steps 1600 --store-frames
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "best"
DT = 0.05  # 20 Hz control step
RUN_LIMIT_STEPS = 1600
INSTRUCTION = (
    "Brace the electric griddle by its handle to seat the heat control probe, turn the dial on, and place both slices of bread on its cooking surface."
)

# -- colours (sRGB 0..1) ------------------------------------------------------------------------------------------

PAN_DARK = (0.14, 0.14, 0.15)
PHENOLIC = (0.08, 0.08, 0.09)
STEEL = (0.68, 0.70, 0.72)
BRASS = (0.78, 0.62, 0.28)
WHITE_INDEX = (0.92, 0.92, 0.90)
CERAMIC = (0.88, 0.87, 0.83)
BREAD_TAN = (0.76, 0.50, 0.24)
CRUST = (0.55, 0.33, 0.14)
COAL_BLACK = (0.06, 0.06, 0.07)
STARTER_TAN = (0.76, 0.65, 0.38)
RED_ABS = (0.82, 0.10, 0.07)
WALL_PAINT = (0.84, 0.83, 0.78)
FRAME_WHITE = (0.93, 0.93, 0.92)
GLASS_TINT = (0.48, 0.64, 0.72)
CORD_BLACK = (0.07, 0.07, 0.08)
NYLON_GREY = (0.20, 0.20, 0.22)

# -- Presto 07211 Liddle Griddle (nominal cooking area 8.5 x 10.5 in = 215.9 x 266.7 mm, 1.315 kg total) ----------
# World frame: x forward from the arm-base line, y left, z up (m). Arm bases at (0, +-0.305, 0.75).

PAN_XY = (0.350, -0.110)
PAN_Z = TOP + 0.0555  # cooking slab centre; cooking surface top is TOP + 0.0615 (61.5 mm above table)
COOK_LX = 0.2159  # 8.5 in along x (0.24205 .. 0.45795)
COOK_LY = 0.2667  # 10.5 in along y (-0.28335 .. -0.01665)
COOK_THICK = 0.012
COOK_TOP = PAN_Z + COOK_THICK / 2  # TOP + 0.061
RIM_H = 0.006  # raised perimeter lip above the cooking surface
PAN_KG = 1.190  # cast-aluminium griddle body without detachable probe (total with probe+dial = 1.315 kg)

# Socket shroud on the griddle's near (-x) wall, centred at y = -0.180, z = TOP + 0.041:
SHROUD_DY = -0.040
SHROUD_Y = PAN_XY[1] + SHROUD_DY
SHROUD_Z = TOP + 0.041  # 41 mm above table; bottom of 26 mm tall shroud clears table by 28 mm
SHROUD_DZ = SHROUD_Z - PAN_Z  # -0.014 m in griddle frame
# Inner bore of shroud: 56 mm wide (y) x 18 mm tall (z) x 24 mm deep (x), mouth at x = PAN_XY[0] - COOK_LX/2 - 0.024 = 0.21805


def _griddle_boxes():
    lx, ly = COOK_LX, COOK_LY
    boxes = [
        # Cast-aluminium ceramic-coated cooking plate
        mk.box((lx, ly, COOK_THICK), at=(0.0, 0.0, 0.0), color=PAN_DARK),
        # Low 4 mm perimeter walls (6 mm above cooking surface) on far (+x), left (+y), right (-y), and split near (-x)
        mk.box((0.005, ly + 0.008, COOK_THICK + RIM_H), at=(lx / 2 + 0.0025, 0.0, RIM_H / 2), color=PAN_DARK),
        mk.box((0.005, ly + 0.008, COOK_THICK + RIM_H), at=(-lx / 2 - 0.0025, 0.0, RIM_H / 2), color=PAN_DARK),
        mk.box((lx, 0.005, COOK_THICK + RIM_H), at=(0.0, ly / 2 + 0.0025, RIM_H / 2), color=PAN_DARK),
        mk.box((lx, 0.005, COOK_THICK + RIM_H), at=(0.0, -ly / 2 - 0.0025, RIM_H / 2), color=PAN_DARK),
    ]
    # Four phenolic corner legs resting on the table (PAN_Z = TOP + 0.055 -> leg bottom at dz = -0.055)
    for sx in (-0.088, 0.088):
        for sy in (-0.145, 0.145):
            boxes.append(mk.box((0.026, 0.026, 0.049), at=(sx, sy, -0.0305), color=PHENOLIC))
    # Left (+y) and right (-y) phenolic cool-touch handles: 24 mm wide bar at y = +-0.158, top at TOP + 0.064 (dz = +0.009)
    for sgn in (-1.0, 1.0):
        hy = sgn * (ly / 2 + 0.0245)  # +-0.15785 -> world y = +0.00785 (left handle) and -0.30785 (right handle)
        boxes.append(mk.box((0.150, 0.024, 0.016), at=(0.0, hy, 0.001), color=PHENOLIC))
        for bx in (-0.062, 0.062):
            boxes.append(mk.box((0.024, 0.022, 0.024), at=(bx, sgn * (ly / 2 + 0.011), -0.012), color=PHENOLIC))
    # Probe receptacle shroud on the near (-x) side at dy = SHROUD_DY (-0.030), dz = SHROUD_DZ (-0.014):
    # Bore inside: 56 mm (y) x 18 mm (z), walls 4 mm thick, extending 24 mm toward -x from x_local = -lx/2 (-0.10795)
    sx = -lx / 2 - 0.012  # -0.11995 (world x = 0.23005; mouth at world x = 0.21805, back wall at x = 0.24205)
    boxes.extend([
        mk.box((0.024, 0.064, 0.004), at=(sx, SHROUD_DY, SHROUD_DZ - 0.011), color=STEEL),  # shroud floor
        mk.box((0.024, 0.064, 0.004), at=(sx, SHROUD_DY, SHROUD_DZ + 0.011), color=STEEL),  # shroud roof
        mk.box((0.024, 0.004, 0.026), at=(sx, SHROUD_DY - 0.030, SHROUD_DZ), color=STEEL),  # shroud right wall
        mk.box((0.024, 0.004, 0.026), at=(sx, SHROUD_DY + 0.030, SHROUD_DZ), color=STEEL),  # shroud left wall
        # Two brass terminal pins inside the shroud for visual realism (short enough not to block the flat nose)
        mk.box((0.006, 0.005, 0.005), at=(-lx / 2 - 0.003, SHROUD_DY - 0.016, SHROUD_DZ), color=BRASS),
        mk.box((0.006, 0.005, 0.005), at=(-lx / 2 - 0.003, SHROUD_DY + 0.016, SHROUD_DZ), color=BRASS),
    ])
    return boxes


# -- Detachable Control Master heat control probe + rotary dial -----------------------------------------------------
# Probe housing: phenolic body 76 mm (x) x 54 mm (y) x 30 mm (z), plus a 22 mm (x) x 52 mm (y) x 15 mm (z) nose
# that slides +x into the griddle's 56 x 18 mm shroud bore (2 mm total y clearance, 3 mm total z clearance).
# Starts resting in a low guide cradle aligned with the shroud at world y = -0.180, nose 18 mm outside the mouth,
# or 38 mm travel to seat fully! Let's check exact coordinates:
# Shroud back wall is at world x = 0.350 - 0.10795 = 0.24205 (with pins to 0.236).
# Shroud mouth is at world x = 0.21805.
# When seated: probe centre at x = 0.196, so body spans x = 0.158..0.234, and nose (20 mm long, at +0.048 from probe
# centre -> x_local +0.038..+0.058) reaches x = 0.234 + 0.002 = 0.236 (18 mm inside the 24 mm shroud)!
# Wait: body front face (+0.038 from probe centre) stops right against the shroud mouth (x = 0.218) when probe
# centre is at x = 0.218 - 0.038 = 0.180!
# Let's check that! If body is 72 mm long (x_local -0.036 .. +0.036) and nose is 20 mm long (x_local +0.036 .. +0.056):
# At seated x = 0.182: body front face is at 0.182 + 0.036 = 0.2180 (flush with shroud mouth 0.21805!), and nose tip is
# at 0.182 + 0.056 = 0.2380 (20 mm inside the shroud, 2 mm shy of the pins/wall)!
# At start x = 0.148 (34 mm back toward -x): nose tip is at 0.148 + 0.056 = 0.204 (14 mm clear in front of the shroud
# mouth 0.21805, visibly detached!). Wait: let's check right arm IK at x = 0.148..0.182, y = -0.180!
# Or move PAN_XY slightly forward to (0.370, -0.150) so probe starts at x = 0.168 and seats at x = 0.202!
# At x = 0.168..0.202, y = -0.180, right arm IK is right in the middle of its workspace (0.01 mm error)!

SHROUD_MOUTH_X = PAN_XY[0] - COOK_LX / 2 - 0.024  # 0.350 - 0.10795 - 0.024 = 0.21805
PROBE_SEATED_X = SHROUD_MOUTH_X - 0.036  # 0.18205: body front face meets shroud mouth, 20 mm nose inside shroud
PROBE_START_X = PROBE_SEATED_X - 0.034  # 0.14805: nose tip at 0.20405, 14 mm clear of shroud mouth!
PROBE_Z = SHROUD_Z  # TOP + 0.041: body bottom at TOP + 0.026, supported on cradle floor at TOP + 0.026
PROBE_KG = 0.105
DIAL_KG = 0.0204
DIAL_R = 0.022  # 44 mm knob diameter
DIAL_H = 0.018  # 18 mm tall knurled skirt above controller top (TOP + 0.056 .. TOP + 0.074)
DIAL_Z = PROBE_Z + 0.015 + DIAL_H / 2 + 0.001  # TOP + 0.066 (1 mm clearance above controller roof TOP + 0.056)
DIAL_ON_DEG = 15.0  # OFF is 0 deg; active cooking thermostat range >= 15 deg


def _oct_prism(diameter: float, height: float, z_center: float = 0.0, color=PHENOLIC):
    """Solid regular octagon of across-flats ``diameter`` from two rotated boxes."""
    s = diameter * math.tan(math.pi / 8)  # side length; wait: two squares of side d*cos? No, octagon across flats = d -> box (d, s, h) rotated!
    return [
        mk.box((diameter, s, height), at=(0.0, 0.0, z_center), yaw=k * math.pi / 4, color=color)
        for k in range(4)
    ]


def _probe_boxes():
    return [
        # Main phenolic body: 72 x 54 x 30 mm (top at PROBE_Z + 0.015 = TOP + 0.056, bottom at TOP + 0.026)
        mk.box((0.072, 0.054, 0.030), at=(0.0, 0.0, 0.0), color=PHENOLIC),
        # Nose plug: 20 x 52 x 15 mm extending +x from +0.036 to +0.056, centred vertically on SHROUD_Z
        mk.box((0.020, 0.052, 0.015), at=(0.046, 0.0, 0.0), color=PHENOLIC),
        # Central brass sensor tube tip protruding 3 mm from nose
        mk.box((0.004, 0.006, 0.006), at=(0.057, 0.0, 0.0), color=BRASS),
        # Amber neon pilot lens on top rear of body (clear of dial skirt)
        mk.box((0.008, 0.010, 0.003), at=(-0.028, 0.018, 0.016), color=(0.85, 0.35, 0.05)),
        # Cord strain-relief boot on rear (-x) of probe body
        mk.box((0.012, 0.014, 0.012), at=(-0.040, 0.0, -0.004), color=CORD_BLACK),
    ]


def _dial_boxes():
    # 44 mm across-flats octagonal knob + raised white pointer ridge along +x at 0 deg (OFF)
    return [
        *_oct_prism(2 * DIAL_R, DIAL_H, 0.0, color=(0.16, 0.16, 0.18)),
        mk.box((0.038, 0.010, 0.008), at=(0.0, 0.0, DIAL_H / 2 + 0.004), color=WHITE_INDEX),
    ]


# -- Guide cradle & preconnected cord to wall outlet --------------------------------------------------------------
# Cradle supports the detached probe at z_bottom = TOP + 0.026 so its nose stays level with the griddle shroud,
# with side rails leaving the upper 24 mm of the 54 mm wide probe body completely clear for a top-down grasp!
CRADLE_X0 = PROBE_START_X - 0.040  # 0.12805
CRADLE_X1 = PROBE_SEATED_X + 0.010  # 0.21205 (stops 26 mm short of shroud mouth so griddle can slide freely)
CRADLE_XC = (CRADLE_X0 + CRADLE_X1) / 2
CRADLE_LX = CRADLE_X1 - CRADLE_X0  # 0.084 m

# -- Serving plate & two bread slices -----------------------------------------------------------------------------
PLATE_XY = (0.165, 0.150)
PLATE_Z = TOP + 0.005
BREAD_SIZE = (0.080, 0.060, 0.027)  # 80 x 60 x 27 mm closed-cell silicone bread slice stand-in, 35 g each
BREAD_KG = 0.035
BREAD_START = [
    (0.165, 0.110, TOP + 0.010 + BREAD_SIZE[2] / 2),  # slice 0 (nearer midline)
    (0.165, 0.190, TOP + 0.010 + BREAD_SIZE[2] / 2),  # slice 1 (further +y)
]
BREAD_DEST = [
    (0.360, -0.160),  # slice 0 (right arm)
    (0.320, -0.040),  # slice 1 (left arm)
]


def _plate_boxes():
    # 140 x 195 mm shallow glazed ceramic tray with 6 mm rim
    return [
        mk.box((0.140, 0.195, 0.008), at=(0.0, 0.0, -0.001), color=CERAMIC),
        mk.box((0.140, 0.006, 0.014), at=(0.0, -0.0945, 0.002), color=CERAMIC),
        mk.box((0.140, 0.006, 0.014), at=(0.0, 0.0945, 0.002), color=CERAMIC),
        mk.box((0.006, 0.183, 0.014), at=(-0.067, 0.0, 0.002), color=CERAMIC),
        mk.box((0.006, 0.183, 0.014), at=(0.067, 0.0, 0.002), color=CERAMIC),
    ]


def _bread_boxes():
    return [
        mk.box((0.076, 0.056, 0.027), at=(0.0, 0.0, 0.0), color=BREAD_TAN),
        mk.box((0.080, 0.060, 0.005), at=(0.0, 0.0, 0.011), color=CRUST),
    ]


# -- Indoor room context: back wall splashback with closed casement window and wired wall outlet -------------------
# Placed at x = 0.545 so the entire indoor wall, closed glazed window, window latch, and wired outlet appear clearly
# in the top camera and wrist cameras without obstructing any arm motion.
WALL_X = 0.545
WALL_W = 1.120  # y = -0.560 .. +0.560


def _indoor_wall_fixtures():
    # Wall surrounds a 420 mm wide x 150 mm tall closed window opening (y = -0.210 .. +0.210, z = TOP+0.045 .. TOP+0.195)
    z0 = TOP
    return [
        # Lower sill wall across full width (45 mm tall)
        Box("wall_sill", pos=(WALL_X, 0.0, z0 + 0.0225), size=(0.030, WALL_W, 0.045), color=WALL_PAINT),
        # Upper header wall across full width (from +0.195 to +0.240)
        Box("wall_header", pos=(WALL_X, 0.0, z0 + 0.2175), size=(0.030, WALL_W, 0.045), color=WALL_PAINT),
        # Left (+y) and right (-y) wall piers flanking the window
        Box("wall_left", pos=(WALL_X, 0.385, z0 + 0.120), size=(0.030, 0.350, 0.150), color=WALL_PAINT),
        Box("wall_right", pos=(WALL_X, -0.385, z0 + 0.120), size=(0.030, 0.350, 0.150), color=WALL_PAINT),
        # Window sill trim + closed white timber sash frame + blue-tinted glass pane + closed black casement handle
        Box("window_sill", pos=(WALL_X - 0.012, 0.0, z0 + 0.042), size=(0.046, 0.450, 0.010), color=FRAME_WHITE),
        Box("window_pane", pos=(WALL_X, 0.0, z0 + 0.120), size=(0.008, 0.396, 0.136), color=GLASS_TINT),
        Box("window_frame_b", pos=(WALL_X - 0.006, 0.0, z0 + 0.053), size=(0.020, 0.420, 0.016), color=FRAME_WHITE),
        Box("window_frame_t", pos=(WALL_X - 0.006, 0.0, z0 + 0.187), size=(0.020, 0.420, 0.016), color=FRAME_WHITE),
        Box("window_frame_l", pos=(WALL_X - 0.006, 0.202, z0 + 0.120), size=(0.020, 0.016, 0.150), color=FRAME_WHITE),
        Box("window_frame_r", pos=(WALL_X - 0.006, -0.202, z0 + 0.120), size=(0.020, 0.016, 0.150), color=FRAME_WHITE),
        Box("window_mullion", pos=(WALL_X - 0.006, 0.0, z0 + 0.120), size=(0.018, 0.014, 0.136), color=FRAME_WHITE),
        Box("window_latch", pos=(WALL_X - 0.018, 0.0, z0 + 0.115), size=(0.012, 0.010, 0.036), color=PHENOLIC),
        # Surface-mounted dry electrical outlet on the right wall pier (y = -0.430, z = TOP + 0.085) with plugged cap
        Box("wall_outlet", pos=(WALL_X - 0.025, -0.430, z0 + 0.085), size=(0.024, 0.076, 0.086), color=FRAME_WHITE),
        Box("outlet_plug", pos=(WALL_X - 0.044, -0.430, z0 + 0.085), size=(0.020, 0.034, 0.042), color=CORD_BLACK),
        # Preconnected supply cord routed flat along the counter far-right from the outlet to behind the probe cradle
        Box("cord_drop", pos=(WALL_X - 0.048, -0.430, z0 + 0.042), size=(0.008, 0.008, 0.076), color=CORD_BLACK),
        Box("cord_along_x", pos=(0.280, -0.430, z0 + 0.005), size=(0.490, 0.008, 0.008), color=CORD_BLACK),
        Box("cord_to_probe", pos=(0.040, -0.290, z0 + 0.005), size=(0.008, 0.280, 0.008), color=CORD_BLACK),
        Box("cord_into_probe", pos=(0.080, -0.150, z0 + 0.005), size=(0.088, 0.008, 0.008), color=CORD_BLACK),
        # Probe guide cradle at (CRADLE_XC, SHROUD_Y): supports probe bottom at TOP + 0.026, low rear stop
        Box("probe_cradle_base", pos=(CRADLE_XC, SHROUD_Y, z0 + 0.013), size=(CRADLE_LX, 0.044, 0.026), color=NYLON_GREY),
        Box("probe_cradle_stop", pos=(CRADLE_X0 - 0.015, SHROUD_Y, z0 + 0.032), size=(0.006, 0.044, 0.012), color=NYLON_GREY),
    ]


# -- Assemble mechkit parts & joints ------------------------------------------------------------------------------

kit = mk.Kit()

kit.part("griddle", (PAN_XY[0], PAN_XY[1], PAN_Z), _griddle_boxes(), mass=PAN_KG, color=PAN_DARK,
         contact_offset=0.001, solver_position_iterations=32)

# Probe is a free 6-DOF rigid body resting in the cradle so pushing it into the shroud exerts real horizontal force
# on the free griddle (requiring bimanual bracing of the griddle handle!). Once pushed fully home into the shroud
# (nose depth <= 4 mm from seat and aligned within 6 mm), its spring detent catch engages on the griddle!
kit.part("probe", (PROBE_START_X, SHROUD_Y, PROBE_Z), _probe_boxes(), mass=PROBE_KG, color=PHENOLIC,
         contact_offset=0.0008, solver_position_iterations=32)
kit.part("dial", (PROBE_START_X, SHROUD_Y, DIAL_Z), _dial_boxes(), mass=DIAL_KG, color=(0.16, 0.16, 0.18),
         contact_offset=0.0008, solver_position_iterations=32)
kit.hinge("dial_turn", "probe", "dial", anchor=(PROBE_SEATED_X + 0.005, SHROUD_Y, DIAL_Z), axis=(0.0, 0.0, 1.0),
          lower_deg=0.0, upper_deg=180.0, start_deg=0.0, damping=0.01, friction=0.01)

# Serving plate and two bread slices
kit.part("plate", (PLATE_XY[0], PLATE_XY[1], PLATE_Z), _plate_boxes(), mass=0.250, color=CERAMIC)
for _i, _bp in enumerate(BREAD_START):
    kit.part(f"bread_{_i}", _bp, _bread_boxes(), mass=BREAD_KG, color=BREAD_TAN, contact_offset=0.001)


# Nylon spatula on the right front counter (optional tool / realistic kitchen prop)
kit.part("spatula", (0.175, -0.375, TOP + 0.008), [
    mk.box((0.120, 0.022, 0.014), at=(-0.055, 0.0, 0.0), color=PHENOLIC),
    mk.box((0.085, 0.062, 0.005), at=(0.045, 0.0, -0.003), color=NYLON_GREY),
], mass=0.075, color=NYLON_GREY)

FIXTURES = _indoor_wall_fixtures()
OBJECTS = [*kit.objects(), *FIXTURES]


# -- Mechanics & hazard rules (evaluated every step via kit.on_step) ----------------------------------------------

# Expected probe origin in the griddle's local frame when fully seated:
_SEATED_REL = np.array([PROBE_SEATED_X - PAN_XY[0], SHROUD_DY, SHROUD_DZ])  # (-0.16795, -0.030, -0.014)
PROBE_SEAT_TOL_X = 0.030  # within 30 mm of full axial insertion depth
PROBE_SEAT_TOL_YZ = 0.016  # within 16 mm laterally/vertically in the shroud
PROBE_TILT_MAX_DEG = 15.0
PAN_TILT_MAX_DEG = 8.0
PAN_DRIFT_MAX_M = 0.080  # griddle stays on its counter station (within 8 cm of start)


def _tilt_deg(quat_xyzw) -> float:
    r = mk.quat_to_mat(quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def probe_rel_error(objects) -> tuple[float, float, float]:
    """(dx_gap, yz_err, rel_angle_deg) of the probe relative to its seated pose in the griddle shroud."""
    gp, gq = kit.pose(objects, "griddle")
    pp, pq = kit.pose(objects, "probe")
    gr = mk.quat_to_mat(gq)
    pr = mk.quat_to_mat(pq)
    rel = gr.T @ (pp - gp) - _SEATED_REL
    dx_gap = abs(float(rel[0]))
    yz_err = float(math.hypot(rel[1], rel[2]))
    rel_r = gr.T @ pr
    cos_a = max(-1.0, min(1.0, (float(np.trace(rel_r)) - 1.0) / 2.0))
    return dx_gap, yz_err, math.degrees(math.acos(cos_a))


def probe_seated(objects) -> bool:
    dx, yz, ang = probe_rel_error(objects)
    if mk.MEMORY.get("probe_latched"):
        return dx <= 0.050 and yz <= 0.030 and ang <= 25.0
    return dx <= PROBE_SEAT_TOL_X and yz <= PROBE_SEAT_TOL_YZ and ang <= PROBE_TILT_MAX_DEG


def dial_deg(objects) -> float:
    return float(kit.q_deg(objects, "dial_turn"))


def appliance_on(objects) -> bool:
    return probe_seated(objects) and dial_deg(objects) >= DIAL_ON_DEG




def _fell_off_table(objects) -> bool:
    for name in ("griddle", "probe", "bread_0", "bread_1"):
        pos, _ = kit.pose(objects, name)
        if float(pos[2]) < TOP - 0.040:
            return True
    return False


def _step_rule(objects) -> None:
    # Latch probe detent seated state once pushed home into the griddle shroud
    dx, yz, ang = probe_rel_error(objects)
    if not mk.MEMORY.get("probe_latched") and dx <= PROBE_SEAT_TOL_X and yz <= PROBE_SEAT_TOL_YZ and ang <= PROBE_TILT_MAX_DEG:
        mk.MEMORY["probe_latched"] = True

    if _fell_off_table(objects):
        mk.MEMORY["fell_off"] = True
    # Latch zero if food is placed on griddle before probe is seated
    n_bread = int(bread_on_griddle(objects, 0)) + int(bread_on_griddle(objects, 1))
    if not mk.MEMORY.get("probe_latched") and not probe_seated(objects) and n_bread > 0:
        mk.MEMORY["food_before_probe"] = True


kit.on_step(_step_rule)


# -- Scoring (0.0, 0.25, 0.50, 0.75, 1.0) -------------------------------------------------------------------------

_TEST_TCPS = None
_TEST_ROBOT_BOXES = None
CLEAR_MARGIN = 0.020  # 20 mm robot collision clearance from griddle, probe, dial and placed food


def _robot_clear_of(objects, names: tuple[str, ...]) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    for name in names:
        st = objects[name]
        lo, hi = np.asarray(st.low) - CLEAR_MARGIN, np.asarray(st.high) + CLEAR_MARGIN
        if any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps):
            return False
    boxes = mk.robot_collision_boxes() if _TEST_ROBOT_BOXES is None else _TEST_ROBOT_BOXES
    for lo_r, hi_r in boxes:
        for name in names:
            st = objects[name]
            gap = np.maximum(np.maximum(np.asarray(st.low) - hi_r, lo_r - np.asarray(st.high)), 0.0)
            if float(np.linalg.norm(gap)) < CLEAR_MARGIN:
                return False
    return True


def griddle_on_counter(objects) -> bool:
    gp, gq = kit.pose(objects, "griddle")
    drift = math.hypot(float(gp[0]) - PAN_XY[0], float(gp[1]) - PAN_XY[1])
    return (
        drift <= PAN_DRIFT_MAX_M
        and abs(float(gp[2]) - PAN_Z) <= 0.020
        and _tilt_deg(gq) <= PAN_TILT_MAX_DEG
    )


def bread_on_griddle(objects, idx: int) -> bool:
    """Bread slice ``idx`` rests flat and wholly inside the 215.9 x 266.7 mm cooking surface."""
    gp, gq = kit.pose(objects, "griddle")
    gr = mk.quat_to_mat(gq)
    bp, bq = kit.pose(objects, f"bread_{idx}")
    br = mk.quat_to_mat(bq)
    # Check all 8 corners of the 80 x 60 x 27 mm slice in the griddle frame
    hx, hy, hz = BREAD_SIZE[0] / 2, BREAD_SIZE[1] / 2, BREAD_SIZE[2] / 2
    corners = np.array([[sx, sy, sz] for sx in (-hx, hx) for sy in (-hy, hy) for sz in (-hz, hz)])
    world_c = corners @ br.T + bp
    local_c = (world_c - gp) @ gr
    inside_xy = bool(
        np.all(np.abs(local_c[:, 0]) <= COOK_LX / 2 + 0.002)
        and np.all(np.abs(local_c[:, 1]) <= COOK_LY / 2 + 0.002)
    )
    # Cooking surface top in griddle frame is +COOK_THICK/2 = +0.006 m; slice bottom should be within 5 mm of it,
    # and slice must lie flat (its local z axis within 20 deg of griddle normal, upright or flipped).
    bottom_gap = abs(float(local_c[:, 2].min()) - COOK_THICK / 2)
    flat = abs(float((gr.T @ br)[2, 2])) >= math.cos(math.radians(20.0))
    still = objects[f"bread_{idx}"].resting(max_speed=0.040, max_spin=0.40)
    return inside_xy and bottom_gap <= 0.015 and flat and still


def _other_slice_overlap(objects) -> bool:
    """Prevent stacking one bread slice on top of the other on the griddle."""
    p0, _ = kit.pose(objects, "bread_0")
    p1, _ = kit.pose(objects, "bread_1")
    return float(math.hypot(p0[0] - p1[0], p0[1] - p1[1])) < 0.055


def failed(objects) -> bool:
    return bool(
        mk.MEMORY.get("fell_off")
        or mk.MEMORY.get("food_before_probe")
        or _fell_off_table(objects)
    )


def score(objects) -> float:
    """Stage-credited outcome score in {0.0, 0.25, 0.50, 0.75, 1.0}:
    - 0.0 (latched for the episode) if food is placed before probe is seated, or a key object falls off the table.
    - Otherwise:
      * 0.0: probe not yet seated home in the griddle shroud (no food partial credit on unpowered griddle).
      * 0.25: heat control probe seated home into the griddle's shroud (griddle upright and still on the counter).
      * 0.50: probe seated AND thermostat dial turned on (>= 15 deg from OFF).
      * 0.75: probe seated, dial turned on, AND one bread slice resting flat inside the cooking surface (or both
        slices placed before the robot arms withdraw clear).
      * 1.00: probe seated, dial turned on (>= 15 deg), both bread slices resting flat and non-overlapping inside the
        cooking surface, and both robot arms clear (>= 2 cm) of the griddle, probe, dial and food.
    """
    if failed(objects) or not griddle_on_counter(objects):
        return 0.0
    seated = probe_seated(objects)
    if not seated:
        return 0.0
    on = dial_deg(objects) >= DIAL_ON_DEG
    n_bread = int(bread_on_griddle(objects, 0)) + int(bread_on_griddle(objects, 1))
    if n_bread == 2 and _other_slice_overlap(objects):
        n_bread = 1
    if not on:
        return 0.25
    if n_bread == 0:
        return 0.50
    if n_bread == 1:
        return 0.75
    if _robot_clear_of(objects, ("griddle", "probe", "dial", "bread_0", "bread_1")):
        return 1.0
    return 0.75


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    dx, yz, _ = probe_rel_error(objects)
    dial_rem = max(0.0, math.radians(DIAL_ON_DEG - dial_deg(objects))) * DIAL_R
    gp, _ = kit.pose(objects, "griddle")
    food_d = sum(
        math.hypot(float(kit.pose(objects, f"bread_{i}")[0][0] - BREAD_DEST[i][0]),
                   float(kit.pose(objects, f"bread_{i}")[0][1] - BREAD_DEST[i][1]))
        for i in range(2)
    )
    return float(dx + yz + dial_rem + food_d + 0.0 * gp[0])


# -- Bimanual scripted demo (yam_scripted) ------------------------------------------------------------------------
# Left arm (base y = +0.305):
#   1. Braces the griddle's left (+y) cool-touch handle at (0.370, +0.008, TOP+0.062) while right arm seats the probe.
#   2. Releases handle and picks bread_0 from (0.185, +0.095), placing it on the left cooking zone (0.328, -0.090).
#   3. Picks bread_1 from (0.185, +0.175) and hands/stages or places it — wait: let's check if left arm can place
#      bread_0 at (0.328, -0.065) AND right arm can pick bread_0 or bread_1!
#      Even cleaner:
#      - Left arm braces griddle handle (t = 0..8.5 s), then picks bread_1 at (0.185, +0.175) and places it at
#        (0.325, -0.060) on the griddle's +y cooking zone!
#      - Right arm seats the probe into the shroud (t = 0..8.0 s), turns the dial 90 deg clockwise/CCW to ON
#        (t = 8.0..15.0 s), then picks bread_0 at (0.185, +0.095) and places it at (0.410, -0.210) on the griddle's
#        -y cooking zone!
#      Both arms work in parallel and return to clear pose! Let's check exact IK & hull collisions for every step!


def _left_plan() -> list[Waypoint]:
    variant = os.environ.get("DEMO_VARIANT", "")
    o = FINGER_OPEN
    hx, hy = PAN_XY[0], PAN_XY[1] + COOK_LY / 2 + 0.0245
    hz = PAN_Z + 0.006
    grip_handle = -0.011

    if variant in ("no_dial", "no_bread"):
        # Brace handle while probe seats, then withdraw clear
        brace_s = 4.7 if variant == "no_dial" else 11.0
        return [
            Waypoint(2.0, (hx, hy, TOP + 0.095), o, "hover griddle left handle", 0.0),
            Waypoint(1.5, (hx, hy, hz), o, "lower over handle", 0.0),
            Waypoint(0.8, (hx, hy, hz), grip_handle, "grip handle to brace griddle", 0.0),
            Waypoint(brace_s, (hx, hy, hz), grip_handle, "hold steady while probe seats", 0.0),
            Waypoint(0.8, (hx, hy, hz), o, "release handle", 0.0),
            Waypoint(1.5, (hx, hy, TOP + 0.095), o, "lift clear of handle", 0.0),
            Waypoint(2.0, (0.180, 0.320, TOP + 0.140), o, "withdraw left arm clear", 0.0),
            Waypoint(4.0, (0.180, 0.320, TOP + 0.140), o, "hold clear at evaluation", 0.0),
        ]

    b1x, b1y, b1z = BREAD_START[1]
    grip_bread = -0.027
    d1x, d1y = BREAD_DEST[1]  # (0.320, -0.040)
    place_z = COOK_TOP + BREAD_SIZE[2] / 2 + 0.003
    pick_z1 = b1z + 0.006

    return [
        # 1) Brace handle while right arm seats probe and turns dial ON
        Waypoint(2.0, (hx, hy, TOP + 0.095), o, "hover griddle left handle", 0.0),
        Waypoint(1.5, (hx, hy, hz), o, "lower over handle", 0.0),
        Waypoint(0.8, (hx, hy, hz), grip_handle, "grip handle to brace griddle", 0.0),
        Waypoint(11.0, (hx, hy, hz), grip_handle, "hold steady while probe seats and dial turns", 0.0),
        Waypoint(0.8, (hx, hy, hz), o, "release handle", 0.0),
        Waypoint(1.5, (hx, hy, TOP + 0.095), o, "lift clear of handle", 0.0),
        # 2) Pick Bread 1 and place on cooking surface
        Waypoint(2.0, (b1x, b1y, TOP + 0.095), o, "hover bread 1", 0.0),
        Waypoint(1.5, (b1x, b1y, pick_z1), o, "lower to bread 1", 0.0),
        Waypoint(0.8, (b1x, b1y, pick_z1), grip_bread, "grasp bread 1", 0.0),
        Waypoint(1.5, (b1x, b1y, TOP + 0.095), grip_bread, "lift bread 1", 0.0),
        Waypoint(2.0, (d1x, d1y, TOP + 0.095), grip_bread, "carry bread 1 over griddle", 0.0),
        Waypoint(1.5, (d1x, d1y, place_z), grip_bread, "set bread 1 on griddle", 0.0),
        Waypoint(0.8, (d1x, d1y, place_z), o, "release bread 1", 0.0),
        Waypoint(1.5, (d1x, d1y, TOP + 0.115), o, "rise vertical from griddle", 0.0),
        Waypoint(1.5, (0.280, 0.100, TOP + 0.115), o, "retract clear of griddle perimeter", 0.0),
        # 3) Withdraw left arm clear and hold until trial completes
        Waypoint(2.0, (0.180, 0.320, TOP + 0.140), o, "withdraw left arm clear", 0.0),
        Waypoint(22.0, (0.180, 0.320, TOP + 0.140), o, "hold clear at success", 0.0),
    ]


def _right_plan() -> list[Waypoint]:
    variant = os.environ.get("DEMO_VARIANT", "")
    o = FINGER_OPEN
    px0 = PROBE_START_X
    px1 = PROBE_SEATED_X + 0.005
    py = SHROUD_Y
    pz_probe = PROBE_Z + 0.002
    grip_probe = -0.019
    dz = DIAL_Z + 0.008
    grip_dial = -0.012

    if variant == "no_dial":
        # Seat probe, then withdraw clear without turning dial:
        return [
            Waypoint(2.2, (px0, py, TOP + 0.095), o, "hover detached probe", 0.0),
            Waypoint(1.5, (px0, py, pz_probe), o, "lower around probe sides", 0.0),
            Waypoint(0.8, (px0, py, pz_probe), grip_probe, "grasp probe body", 0.0),
            Waypoint(1.0, (px0, py, pz_probe), grip_probe, "wait for left arm brace", 0.0),
            Waypoint(2.5, (px1, py, pz_probe), grip_probe, "push probe home into shroud", 0.0),
            Waypoint(0.8, (px1, py, pz_probe), o, "release seated probe", 0.0),
            Waypoint(2.0, (px1, py, TOP + 0.120), o, "rise above dial", 0.0),
            Waypoint(2.0, (0.180, -0.320, TOP + 0.140), o, "withdraw right arm clear", 0.0),
            Waypoint(4.0, (0.180, -0.320, TOP + 0.140), o, "hold clear at evaluation", 0.0),
        ]

    if variant == "no_bread":
        return [
            Waypoint(2.2, (px0, py, TOP + 0.095), o, "hover detached probe", 0.0),
            Waypoint(1.5, (px0, py, pz_probe), o, "lower around probe sides", 0.0),
            Waypoint(0.8, (px0, py, pz_probe), grip_probe, "grasp probe body", 0.0),
            Waypoint(1.0, (px0, py, pz_probe), grip_probe, "wait for left arm brace", 0.0),
            Waypoint(2.5, (px1, py, pz_probe), grip_probe, "push probe home into shroud", 0.0),
            Waypoint(0.8, (px1, py, pz_probe), o, "release seated probe", 0.0),
            Waypoint(1.2, (px1, py, TOP + 0.110), o, "rise above dial", 0.0),
            Waypoint(1.2, (px1, py, dz), o, "lower onto dial", 0.0),
            Waypoint(0.8, (px1, py, dz), grip_dial, "grasp dial", 0.0),
            Waypoint(1.2, (px1, py, dz), grip_dial, "turn dial 45 deg", math.pi / 4),
            Waypoint(1.2, (px1, py, dz), grip_dial, "turn dial 90 deg ON", math.pi / 2),
            Waypoint(0.8, (px1, py, dz), o, "release dial", math.pi / 2),
            Waypoint(1.4, (px1, py, TOP + 0.095), o, "lift clear of dial", 0.0),
            Waypoint(2.0, (0.180, -0.320, TOP + 0.140), o, "withdraw right arm clear", 0.0),
            Waypoint(4.0, (0.180, -0.320, TOP + 0.140), o, "hold clear at success", 0.0),
        ]

    b0x, b0y, b0z = BREAD_START[0]
    grip_bread = -0.027
    d0x, d0y = BREAD_DEST[0]  # (0.360, -0.160)
    place_z = COOK_TOP + BREAD_SIZE[2] / 2 + 0.003
    pick_z0 = b0z + 0.006

    return [
        # 1) Seat probe and turn dial 90 deg ON
        Waypoint(2.2, (px0, py, TOP + 0.095), o, "hover detached probe", 0.0),
        Waypoint(1.5, (px0, py, pz_probe), o, "lower around probe sides", 0.0),
        Waypoint(0.8, (px0, py, pz_probe), grip_probe, "grasp probe body", 0.0),
        Waypoint(1.0, (px0, py, pz_probe), grip_probe, "wait for left arm brace", 0.0),
        Waypoint(2.5, (px1, py, pz_probe), grip_probe, "push probe home into shroud", 0.0),
        Waypoint(0.8, (px1, py, pz_probe), o, "release seated probe", 0.0),
        Waypoint(1.2, (px1, py, TOP + 0.110), o, "rise above dial", 0.0),
        Waypoint(1.2, (px1, py, dz), o, "lower onto dial", 0.0),
        Waypoint(0.8, (px1, py, dz), grip_dial, "grasp dial", 0.0),
        Waypoint(1.2, (px1, py, dz), grip_dial, "turn dial 45 deg", math.pi / 4),
        Waypoint(1.2, (px1, py, dz), grip_dial, "turn dial 90 deg ON", math.pi / 2),
        Waypoint(0.8, (px1, py, dz), o, "release dial", math.pi / 2),
        Waypoint(1.4, (px1, py, TOP + 0.095), o, "lift clear of dial", 0.0),
        Waypoint(2.0, (0.180, -0.320, TOP + 0.140), o, "withdraw right arm clear", 0.0),
        # 2) Wait in park while left arm places Bread 1
        Waypoint(15.5, (0.180, -0.320, TOP + 0.140), o, "wait for left bread placement", 0.0),
        # 3) Pick Bread 0 and place on cooking surface
        Waypoint(2.0, (b0x, b0y, TOP + 0.095), o, "hover bread 0", 0.0),
        Waypoint(1.5, (b0x, b0y, pick_z0), o, "lower to bread 0", 0.0),
        Waypoint(0.8, (b0x, b0y, pick_z0), grip_bread, "grasp bread 0", 0.0),
        Waypoint(1.5, (b0x, b0y, TOP + 0.095), grip_bread, "lift bread 0", 0.0),
        Waypoint(2.0, (d0x, d0y, TOP + 0.095), grip_bread, "carry bread 0 over griddle", 0.0),
        Waypoint(1.5, (d0x, d0y, place_z), grip_bread, "set bread 0 on griddle", 0.0),
        Waypoint(0.8, (d0x, d0y, place_z), o, "release bread 0", 0.0),
        Waypoint(1.5, (d0x, d0y, TOP + 0.115), o, "rise vertical from griddle", 0.0),
        Waypoint(1.5, (0.280, -0.200, TOP + 0.115), o, "retract clear of griddle perimeter", 0.0),
        # 4) Withdraw clear and hold at success
        Waypoint(2.0, (0.180, -0.320, TOP + 0.140), o, "withdraw right arm clear", 0.0),
        Waypoint(20.0, (0.180, -0.320, TOP + 0.140), o, "hold clear at success", 0.0),
    ]


def demo() -> dict[str, list[Waypoint]]:
    return {"left": _left_plan(), "right": _right_plan()}
