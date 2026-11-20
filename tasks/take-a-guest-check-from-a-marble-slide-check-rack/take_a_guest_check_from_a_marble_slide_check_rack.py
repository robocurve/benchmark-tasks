"""On the station table: a commercial anodized aluminum marble slide check rack (San Jamar CK6512A, 305 x 51 mm)
mounted 195 mm above the table on the front apron of a stainless-steel kitchen pass shelf stand, holding two 89 x 171 mm
heavyweight board guest checks side by side: a green target check on the left and a tan neighbour check 31 mm to its
right. Four 16 mm aqua glass marbles roll freely on the rack's internal 16-degree inclined ramp, two per check, forming a
self-energizing gravity ball-wedge clutch that locks on a straight-down pull and releases when the check is lifted slightly
and pivoted forward before withdrawing down-and-forward. On the table in front of the left arm sits a flat, clipless
black melamine rectangular check tray (Winco WDM018-301, 127 x 197 x 11 mm).

Requires -E object_jitter=0: the starting positions of the checks and tray are fixed.
"""

from __future__ import annotations

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from belimits import Rect, bottom, inside, is_airborne, tilt_deg, top
import closedloop
from closedloop import grip_rot, placeholder_demo
import mechkit
from mechkit import Mech, ball, box, cyl, extrude, on_reset, released
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0
SCORE_OVER = "end"

# -- colours (sRGB 0..1) -----------------------------------------------------------------------------------------------

STEEL = (0.68, 0.70, 0.72)
ALUM = (0.76, 0.78, 0.80)
DARK_BASE = (0.26, 0.28, 0.30)
CAP_BLACK = (0.14, 0.14, 0.16)
TRAY_BLACK = (0.14, 0.15, 0.17)
AQUA = (0.45, 0.78, 0.82)
GREEN_CHECK = (0.20, 0.68, 0.36)
TAN_CHECK = (0.94, 0.88, 0.72)
INK_DARK = (0.12, 0.24, 0.16)
INK_TAN = (0.34, 0.30, 0.24)

# -- rack & kitchen pass stand (San Jamar CK6512A mounted on pass shelf apron) -----------------------------------------

RACK_MOUTH_LOCAL_Z = 0.195  # rack bottom 195 mm above table top
RACK_MOUTH_Z = TOP + RACK_MOUTH_LOCAL_Z
RACK_H = 0.051  # 51 mm (2.0 in) tall
RACK_BODY_L = 0.295  # 295 mm aluminum extrusion + two 5 mm ABS end caps = 305 mm (12.0 in)
RACK_BACK_X = 0.335  # vertical rear wall inner face
RACK_FRONT_X = 0.309  # outer front face (26 mm channel depth for 2 mm sim checks and >= 3 mm walls)
RACK_LIP_X = 0.3210  # 14.0 mm bottom slot (x 0.3210..0.3350); 16 mm marbles cannot fall through
RACK_LIP_H = 0.002  # 2.0 mm lip wall thickness
RAMP_DEG = 16.0
RAMP_TAN = math.tan(math.radians(RAMP_DEG))
RAMP_COS = math.cos(math.radians(RAMP_DEG))
RAMP_SIN = math.sin(math.radians(RAMP_DEG))
RAMP_TOP_X = 0.312
RAMP_TOP_Z = RACK_MOUTH_LOCAL_Z + RACK_LIP_H + (RACK_LIP_X - RAMP_TOP_X) / RAMP_TAN
CEIL_Z = RACK_MOUTH_LOCAL_Z + 0.046  # 5 mm thick top roof (0.241..0.246)

# Cross-section in (u=z, v=x) for extrude(axis="y"): 3 convex hulls, min thickness 3.0 mm, zero overlap at x=0.335
_RACK_OUTLINE = [
    (RACK_MOUTH_LOCAL_Z, RACK_FRONT_X),
    (RACK_MOUTH_LOCAL_Z, RACK_LIP_X),
    (RACK_MOUTH_LOCAL_Z + RACK_LIP_H, RACK_LIP_X),
    (RAMP_TOP_Z, RAMP_TOP_X),
    (CEIL_Z, RAMP_TOP_X),
    (CEIL_Z, RACK_BACK_X),
    (RACK_MOUTH_LOCAL_Z + RACK_H, RACK_BACK_X),
    (RACK_MOUTH_LOCAL_Z + RACK_H, RACK_FRONT_X),
]

RACK_EXT = extrude((0.0, 0.0, 0.0), _RACK_OUTLINE, RACK_BODY_L, ALUM, axis="y")
APRON = box((0.340, 0.0, RACK_MOUTH_LOCAL_Z + RACK_H / 2), (0.010, 0.305, RACK_H), STEEL)
SHELF_TOP = box((0.3975, 0.0, RACK_MOUTH_LOCAL_Z + 0.0435), (0.105, 0.320, 0.015), STEEL)
LEG_L = cyl((0.430, +0.140, 0.1215), 0.011, 0.219, STEEL, axis="z")
LEG_R = cyl((0.430, -0.140, 0.1215), 0.011, 0.219, STEEL, axis="z")
STAND_BASE = box((0.425, 0.0, 0.006), (0.070, 0.320, 0.012), DARK_BASE)
CAP_L = box(((RACK_FRONT_X + RACK_BACK_X) / 2, +0.150, RACK_MOUTH_LOCAL_Z + RACK_H / 2), (0.026, 0.005, RACK_H), CAP_BLACK)
CAP_R = box(((RACK_FRONT_X + RACK_BACK_X) / 2, -0.150, RACK_MOUTH_LOCAL_Z + RACK_H / 2), (0.026, 0.005, RACK_H), CAP_BLACK)

# Recessed invisible lateral keeper ribs (x 0.311..0.322, clear of the continuous check slot and tilted check sweep)
KEEPER_YS = (+0.110, +0.070, +0.030, -0.010, -0.050, -0.090)
KEEPER_RIBS = tuple(
    dataclasses.replace(
        box((0.3165, ry, RACK_MOUTH_LOCAL_Z + 0.024), (0.011, 0.003, 0.044), ALUM),
        visible=False,
    )
    for ry in KEEPER_YS
)
# Look-only filler marbles in the unused stations along the rail
FILLER_MARBLES = tuple(
    dataclasses.replace(
        ball((0.327, fy, RACK_MOUTH_LOCAL_Z + 0.010), 0.008, AQUA, sides=24, rows=6),
        collide=False,
    )
    for fy in (+0.130, +0.010, -0.110, -0.130)
)

PASS_RACK = Mech(
    "pass_rack",
    pos=(0.0, 0.0, TOP),
    mass=None,
    friction=0.24,
    shapes=(RACK_EXT, APRON, SHELF_TOP, LEG_L, LEG_R, STAND_BASE, CAP_L, CAP_R, *KEEPER_RIBS, *FILLER_MARBLES),
)

# -- checks (NCCO G3632 green / T3624-SP tan, 89 x 171 mm; 2 mm rigid plates in sim) ----------------------------------

CHECK_T = 0.002
CHECK_W = 0.089  # 3.50 in
CHECK_H = 0.171  # 6.75 in
CHECK_MASS = 0.012

MARBLE_R = 0.008  # 16 mm (5/8 in) glass marble
MARBLE_L = 0.012  # smooth rolling cylinder width
MARBLE_MASS = 0.0055

CHECK_X = RACK_BACK_X - CHECK_T / 2  # 0.3340 (back face at 0.3350, front face at 0.3330)
MARBLE_X = RACK_BACK_X - CHECK_T - MARBLE_R  # 0.3250
# Exact tangent contact height on the 16-deg ramp when pinching a 2 mm check
_MARBLE_XC = MARBLE_X - MARBLE_R * RAMP_COS
_MARBLE_ZC = RACK_MOUTH_LOCAL_Z + RACK_LIP_H + (RACK_LIP_X - _MARBLE_XC) / RAMP_TAN
MARBLE_REST_LOCAL_Z = _MARBLE_ZC + MARBLE_R * RAMP_SIN  # ~0.2121
MARBLE_SPAWN_Z = TOP + MARBLE_REST_LOCAL_Z + 0.0002  # 0.2 mm clearance above exact tangent

CHECK_TOP_LOCAL_Z = RACK_MOUTH_LOCAL_Z + 0.026  # 0.221 (26 mm inside rack, ~9 mm above marble centre)
CHECK_START_Z = TOP + CHECK_TOP_LOCAL_Z - CHECK_H / 2  # TOP + 0.1355 (bottom at TOP + 0.050)
GREEN_Y = +0.070
TAN_Y = -0.050  # 31 mm gap between edges (+0.0255 and -0.0055)


def _check_shapes(body_color, ink_color):
    plate = box((0.0, 0.0, 0.0), (CHECK_T, CHECK_W, CHECK_H), body_color)
    # Look-only printed order lines on the front (-x) face, recessed flush so bounding box is unchanged
    lines = []
    # Header bar
    lines.append(
        dataclasses.replace(
            box((-CHECK_T / 2 + 0.0001, 0.0, 0.048), (0.0002, 0.072, 0.006), ink_color),
            collide=False,
        )
    )
    for k in range(6):
        lz = 0.028 - k * 0.016
        lines.append(
            dataclasses.replace(
                box((-CHECK_T / 2 + 0.0001, 0.0, lz), (0.0002, 0.068, 0.0022), ink_color),
                collide=False,
            )
        )
    return (plate, *lines)


CHECK_GREEN = Mech(
    "green_check",
    pos=(CHECK_X, GREEN_Y, CHECK_START_Z),
    mass=CHECK_MASS,
    friction=0.36,
    shapes=_check_shapes(GREEN_CHECK, INK_DARK),
)
CHECK_TAN = Mech(
    "tan_check",
    pos=(CHECK_X, TAN_Y, CHECK_START_Z),
    mass=CHECK_MASS,
    friction=0.36,
    shapes=_check_shapes(TAN_CHECK, INK_TAN),
)

# Four active rolling marbles: native smooth cylinder collider + look-only 16 mm glass sphere visual
_MARBLE_COL = dataclasses.replace(cyl((0.0, 0.0, 0.0), MARBLE_R, MARBLE_L, AQUA, axis="y", sides=0), visible=False)
_MARBLE_VIS = dataclasses.replace(ball((0.0, 0.0, 0.0), MARBLE_R, AQUA, sides=24, rows=6), collide=False)

MARBLE_SPECS = (
    ("marble_g1", +0.090),
    ("marble_g2", +0.050),
    ("marble_t1", -0.030),
    ("marble_t2", -0.070),
)
MARBLES = tuple(
    Mech(name, pos=(MARBLE_X, my, MARBLE_SPAWN_Z), mass=MARBLE_MASS, friction=0.24, shapes=(_MARBLE_COL, _MARBLE_VIS))
    for name, my in MARBLE_SPECS
)

# -- flat clipless check tray (Winco WDM018-301 black melamine, 197 x 127 x 14 mm) -------------------------------------

TRAY_XY = (0.180, +0.125)
TRAY_X_LEN = 0.197  # 7.75 in along x
TRAY_Y_LEN = 0.127  # 5.00 in along y
TRAY_FLOOR_H = 0.008  # 8 mm thick melamine floor (top at TOP + 8 mm)
TRAY_RIM_H = 0.006  # 6 mm raised rim (top at TOP + 14 mm)
TRAY_WALL = 0.004  # 4 mm rim walls -> inner floor 189 x 119 mm
TRAY_FLOOR_Z = TOP + TRAY_FLOOR_H

TRAY_OUTER = Rect(
    (TRAY_XY[0] - TRAY_X_LEN / 2, TRAY_XY[0] + TRAY_X_LEN / 2),
    (TRAY_XY[1] - TRAY_Y_LEN / 2, TRAY_XY[1] + TRAY_Y_LEN / 2),
)
TRAY_INNER = Rect(
    (TRAY_OUTER.x0 + TRAY_WALL, TRAY_OUTER.x1 - TRAY_WALL),
    (TRAY_OUTER.y0 + TRAY_WALL, TRAY_OUTER.y1 - TRAY_WALL),
)

_TRAY_FLOOR = box((0.0, 0.0, TRAY_FLOOR_H / 2), (TRAY_X_LEN, TRAY_Y_LEN, TRAY_FLOOR_H), TRAY_BLACK)
_TRAY_RIM_X0 = box((-TRAY_X_LEN / 2 + TRAY_WALL / 2, 0.0, TRAY_FLOOR_H + TRAY_RIM_H / 2), (TRAY_WALL, TRAY_Y_LEN, TRAY_RIM_H), TRAY_BLACK)
_TRAY_RIM_X1 = box((+TRAY_X_LEN / 2 - TRAY_WALL / 2, 0.0, TRAY_FLOOR_H + TRAY_RIM_H / 2), (TRAY_WALL, TRAY_Y_LEN, TRAY_RIM_H), TRAY_BLACK)
_TRAY_RIM_Y0 = box((0.0, -TRAY_Y_LEN / 2 + TRAY_WALL / 2, TRAY_FLOOR_H + TRAY_RIM_H / 2), (TRAY_X_LEN - 2 * TRAY_WALL, TRAY_WALL, TRAY_RIM_H), TRAY_BLACK)
_TRAY_RIM_Y1 = box((0.0, +TRAY_Y_LEN / 2 - TRAY_WALL / 2, TRAY_FLOOR_H + TRAY_RIM_H / 2), (TRAY_X_LEN - 2 * TRAY_WALL, TRAY_WALL, TRAY_RIM_H), TRAY_BLACK)

CHECK_TRAY = Mech(
    "check_tray",
    pos=(*TRAY_XY, TOP),
    mass=None,
    friction=0.50,
    shapes=(_TRAY_FLOOR, _TRAY_RIM_X0, _TRAY_RIM_X1, _TRAY_RIM_Y0, _TRAY_RIM_Y1),
)

OBJECTS = [PASS_RACK, CHECK_TRAY, CHECK_GREEN, CHECK_TAN, *MARBLES]

# -- scorer ------------------------------------------------------------------------------------------------------------

SCORER_HZ = 20
SETTLE_STEPS = int(1.0 * SCORER_HZ)  # ignore first 1.0 s settle
D_STEPS = int(0.5 * SCORER_HZ)  # 0.5 s sustained tan drop
V_STEPS = int(0.5 * SCORER_HZ)  # 0.5 s sustained tan disturb
F_STEPS = int(0.2 * SCORER_HZ)  # 0.2 s sustained fault
E_STEPS = int(0.5 * SCORER_HZ)  # 0.5 s sustained clean extraction
END_STEPS = int(1.0 * SCORER_HZ)  # last 1.0 s sustained release + stillness

TAN_DROP_TOP_Z = RACK_MOUTH_Z  # TOP + 195 mm
TAN_SHIFT_LIMIT = 0.025  # 25 mm
TAN_TILT_LIMIT_DEG = 15.0
GREEN_FAULT_Z = TOP - 0.020
MARBLE_FAULT_Z = TOP + 0.170
EXTRACT_TOP_Z = TOP + 0.190  # 5 mm below rack mouth
HOME_FLOOR_TOL = 0.003  # within 3 mm of TOP + 8 mm (rim is at TOP + 14 mm)
HOME_TILT_DEG = 10.0
TABLE_TILT_DEG = 15.0
TABLE_MAX_BOT_Z = TOP + 0.018  # <= 10 mm above tray floor / table top (not airborne)
STILL_LIN = 0.010  # 10 mm/s
STILL_ANG = 0.50  # rad/s
RELEASE_CLEAR = 0.030  # 30 mm from all finger pad points

# Offline replays (mjr.py and check.py) have no Isaac env for mechkit.released().
# In Isaac Sim mechkit._ENV["env"] is always live, so released() is strictly enforced there.
OFFLINE_FINGERS_CLEAR = True


class _LatchState:
    def __init__(self) -> None:
        self.clear()

    def clear(self) -> None:
        self.seen = None
        self.steps = 0
        self.d_streak = 0
        self.v_streak = 0
        self.f_streak = 0
        self.e_streak = 0
        self.rel_still_streak = 0
        self.dropped_tan = False  # D
        self.disturbed_tan = False  # V
        self.fault = False  # F
        self.extracted = False  # E
        self.prev_green_pos = None


_LATCH = _LatchState()


@on_reset
def _reset_latches(_env=None) -> None:
    _LATCH.clear()


def flat_tilt_deg(state) -> float:
    """Angle (deg) between the check's face normal (local x axis) and world vertical (0 deg = lying flat)."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    # Third row, first column of rotation matrix = dot(world_z, R @ local_x)
    r20 = 2.0 * (x * z - y * w)
    return float(math.degrees(math.acos(min(1.0, abs(r20)))))


def _fingers_clear(green_state) -> bool:
    if mechkit._ENV["env"] is None:
        return bool(OFFLINE_FINGERS_CLEAR)
    return bool(released(green_state, clearance=RELEASE_CLEAR))


def _is_still(state) -> bool:
    return bool(
        float(np.linalg.norm(state.lin_vel)) < STILL_LIN
        and float(np.linalg.norm(state.ang_vel)) < STILL_ANG
    )


def _is_home(green_state) -> bool:
    b = bottom(green_state, CHECK_GREEN)
    return bool(
        flat_tilt_deg(green_state) <= HOME_TILT_DEG
        and abs(b - TRAY_FLOOR_Z) <= HOME_FLOOR_TOL
        and not is_airborne(green_state, CHECK_GREEN, (TOP, TRAY_FLOOR_Z), clearance=0.010)
        and inside(green_state, CHECK_GREEN, TRAY_INNER)
    )


def _is_flat_on_table_or_tray(green_state) -> bool:
    b = bottom(green_state, CHECK_GREEN)
    return bool(
        flat_tilt_deg(green_state) <= TABLE_TILT_DEG
        and TOP - 0.005 <= b <= TABLE_MAX_BOT_Z
        and not is_airborne(green_state, CHECK_GREEN, (TOP, TRAY_FLOOR_Z), clearance=0.010)
    )


def _update_latches(objects) -> None:
    if _LATCH.seen is objects:
        return
    green = objects["green_check"]
    tan = objects["tan_check"]
    g_pos = np.asarray(green.pos, dtype=float)
    t_pos = np.asarray(tan.pos, dtype=float)

    # Detect reset jump if called without on_reset (fallback)
    g_start = np.array([CHECK_X, GREEN_Y, CHECK_START_Z], dtype=float)
    if (
        _LATCH.prev_green_pos is not None
        and float(np.linalg.norm(_LATCH.prev_green_pos - g_start)) > 0.03
        and float(np.linalg.norm(g_pos - g_start)) <= 0.005
        and _is_still(green)
    ):
        _LATCH.clear()

    _LATCH.seen = objects
    _LATCH.prev_green_pos = g_pos.copy()
    _LATCH.steps += 1

    # End-of-trial sustained Release + Stillness window (tracks continuously)
    if _fingers_clear(green) and _is_still(green):
        _LATCH.rel_still_streak += 1
    else:
        _LATCH.rel_still_streak = 0

    # Ignore first 1.0 s settle window for D, V, F, E
    if _LATCH.steps <= SETTLE_STEPS:
        return

    # D: tan dropped (top < TOP + 195 mm for >= 0.5 s)
    tan_dropped_now = top(tan, CHECK_TAN) < TAN_DROP_TOP_Z
    _LATCH.d_streak = _LATCH.d_streak + 1 if tan_dropped_now else 0
    if _LATCH.d_streak >= D_STEPS:
        _LATCH.dropped_tan = True

    # V: tan disturbed (shifted >= 25 mm or tilted >= 15 deg for >= 0.5 s without D)
    t_start = np.array([CHECK_X, TAN_Y, CHECK_START_Z], dtype=float)
    tan_shift = float(np.linalg.norm(t_pos - t_start))
    tan_disturbed_now = (not tan_dropped_now) and (
        tan_shift >= TAN_SHIFT_LIMIT or tilt_deg(tan) >= TAN_TILT_LIMIT_DEG
    )
    _LATCH.v_streak = _LATCH.v_streak + 1 if tan_disturbed_now else 0
    if _LATCH.v_streak >= V_STEPS:
        _LATCH.disturbed_tan = True

    # F: green off table or any marble fell out of rack (>= 0.2 s)
    green_off = bottom(green, CHECK_GREEN) < GREEN_FAULT_Z
    marble_out = any(float(objects[n].pos[2]) < MARBLE_FAULT_Z for n, _ in MARBLE_SPECS)
    _LATCH.f_streak = _LATCH.f_streak + 1 if (green_off or marble_out) else 0
    if _LATCH.f_streak >= F_STEPS:
        _LATCH.fault = True

    # E: green extracted (top <= TOP + 190 mm for >= 0.5 s with no D/V up to then)
    green_extracted_now = top(green, CHECK_GREEN) <= EXTRACT_TOP_Z and not (_LATCH.dropped_tan or _LATCH.disturbed_tan)
    _LATCH.e_streak = _LATCH.e_streak + 1 if green_extracted_now else 0
    if _LATCH.e_streak >= E_STEPS:
        _LATCH.extracted = True


def score(objects) -> float:
    """0.0: not extracted (jammed/idle), tan dropped (D), or green off table / marble dropped (F).
    1.0: green flat inside tray inner floor (H), no V/D/F, released and still for last 1.0 s.
    0.5: clean extraction (E), green flat on table/tray (<= 15 deg, bottom <= TOP + 15 mm), no V/D/F, released & still.
    0.25: clean extraction (E) with no V/D/F otherwise; or H with V (tan nudged, not dropped), no D/F, released & still.
    """
    _update_latches(objects)
    if _LATCH.dropped_tan or _LATCH.fault:
        return 0.0

    green = objects["green_check"]
    rel_still = _LATCH.rel_still_streak >= END_STEPS
    home = _is_home(green)

    if home and rel_still:
        if _LATCH.extracted and not _LATCH.disturbed_tan:
            return 1.0
        if _LATCH.disturbed_tan:
            return 0.25

    if _LATCH.extracted and not _LATCH.disturbed_tan:
        if _is_flat_on_table_or_tray(green) and rel_still:
            return 0.5
        return 0.25

    return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


# -- arm keep-out (checked by sweep.py) --------------------------------------------------------------------------------
# Plan-view keep-outs extending down to the table: the adjacent tan check (padded 20+ mm) and the rear pass shelf legs/base.
# (The overhead rack housing above z = TOP + 185 mm is enforced as a 3D box in the task's BE envelope.)
ARM_KEEP_OUT = [
    {"name": "tan check guard", "x": (0.305, 0.365), "y": (-0.115, +0.015), "z_top": TOP + 0.250},
    {"name": "pass stand rear support", "x": (0.385, 0.465), "y": (-0.175, +0.175), "z_top": TOP + 0.265},
]

# -- scripted demo (left arm only, closedloop default) -----------------------------------------------------------------

demo = placeholder_demo

FINGER_APPROACH = -0.014  # 28 mm opening (13 mm clear per face, keeps back finger clear of rear stand)
FINGER_RELEASE = -0.018  # 36 mm clean symmetric release opening
GRASP_PITCH_RAD = math.radians(82.0)  # 8 deg below horizontal keeps all wrist joints >= 11 deg from stops
PIVOT_DEG = 19.0
_R_GRASP_BASE = grip_rot(yaw=-math.pi / 2, pitch=GRASP_PITCH_RAD, roll=0.0)


def _pivot_rot(beta_rad: float) -> np.ndarray:
    """Gripper orientation during side-edge grasp and pure +y forward-bottom pivot (+beta about world +y)."""
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), float(beta_rad)) @ _R_GRASP_BASE


def demo_script(ctx):
    """Left arm grasps the green check's left outer edge, lifts straight up to roll both left marbles up the 16-deg
    ramp, pivots the lower edge forward (+19 deg about world +y) around the check's top-back corner, draws smoothly
    down-and-forward out of the rack without disturbing the tan check, rotates the check flat over the black tray,
    lowers and releases it inside the inner floor, then parks at READY. Right arm holds READY throughout."""
    L = ctx.left

    # 0) Let the checks settle into the marbles (and pass the 1.0 s settle window)
    yield from ctx.wait(1.2)
    g0 = ctx.obj("green_check")
    cx = float(g0.pos[0])
    cy = float(g0.pos[1])
    cz = float(g0.pos[2])
    z_top = float(g0.high[2])
    grasp_y = cy + 0.020  # +0.090: 24.5 mm onto check from left edge, 75 mm clear of tan guard

    # 1) Approach from left (+y) outside the green check with fingers pointing -y (pads closing along x)
    r_mid = grip_rot(yaw=-math.pi / 2, pitch=math.radians(60.0), roll=0.0)
    r_grasp = _pivot_rot(0.0)
    mid_pos = (0.265, 0.165, TOP + 0.145)
    hover_pos = (cx, 0.155, cz)
    grasp_pos = (cx, grasp_y, cz)
    yield from L.move(mid_pos, duration=3.0, finger=FINGER_APPROACH, rot=r_mid)
    yield from L.move(hover_pos, duration=2.0, finger=FINGER_APPROACH, rot=r_grasp)
    yield from L.trim_to_measured(hover_pos, max_duration=0.8)
    yield from L.move(grasp_pos, duration=1.8, finger=FINGER_APPROACH, rot=r_grasp)
    yield from L.trim_to_measured(grasp_pos, max_duration=0.8)
    yield from L.grip(FINGER_CLOSED, duration=0.8)
    yield from L.hold(0.3)

    # Record measured offset so subsequent contact moves cancel wrist sag cleanly
    sag = L.pos - L.measured_pos

    # 2) Unseat gravity ball-wedge: lift straight up +8 mm at x = cx (rolls marbles up the 16-deg ramp)
    lift_dz = 0.008
    lift_target = np.array([cx, grasp_y, cz + lift_dz], dtype=float)
    yield from L.move(lift_target + sag, duration=1.0, rot=r_grasp)
    yield from L.hold(0.2)

    # 3) Pivot lower edge forward (-x) by +19 deg about world +y, pivoting strictly around top-back corner
    beta_rad = math.radians(PIVOT_DEG)
    piv_x = RACK_BACK_X
    piv_z0 = z_top + lift_dz
    d_piv = np.array([cx - piv_x, 0.0, cz - z_top], dtype=float)  # (-0.001, 0, -0.0855)
    piv_steps = 36  # 1.8 s at 20 Hz
    for i in range(1, piv_steps + 1):
        s = i / piv_steps
        s_smooth = s * s * (3.0 - 2.0 * s)
        ang = beta_rad * s_smooth
        # Ease top-back corner down 7 mm during pivot (to z_top + 1 mm) so front lip clearance >= 2.5 mm
        piv_pt = np.array([piv_x, grasp_y, piv_z0 - 0.007 * s_smooth], dtype=float)
        rot_y = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), ang)
        tcp_des = piv_pt + rot_y @ d_piv
        L._set_cartesian(tcp_des + sag, _pivot_rot(ang), FINGER_CLOSED)
        yield

    # 4) Angled withdrawal down-and-forward along (-sin 19, 0, -cos 19) by 40 mm at ~0.02 m/s
    r_piv = _pivot_rot(beta_rad)
    piv_end_pt = np.array([piv_x, grasp_y, piv_z0 - 0.007], dtype=float)
    rot_y_end = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), beta_rad)
    tcp_piv_end = piv_end_pt + rot_y_end @ d_piv
    u_withdraw = np.array([-math.sin(beta_rad), 0.0, -math.cos(beta_rad)], dtype=float)
    tcp_out = tcp_piv_end + 0.040 * u_withdraw
    yield from L.move(tcp_out + sag, duration=2.2, rot=r_piv)
    yield from L.hold(0.2)

    # Clear forward away from the rack before rotating horizontal
    clear_pos = np.array([0.235, 0.145, TOP + 0.115], dtype=float)
    yield from L.move(clear_pos, duration=2.2, rot=r_piv)

    # 5) Rotate flat (keep fingers pointing -y, pitched 22 deg down so wrist/carriage stay well clear of the 14 mm rim;
    #    check 171 mm length aligns along x and 89 mm width along y inside the 189 x 119 mm inner floor)
    place_pitch = math.radians(68.0)  # 22 deg below horizontal
    down_ang = math.pi / 2 - place_pitch  # 22 deg
    r_place = grip_rot(yaw=-math.pi / 2, pitch=place_pitch, roll=-math.pi / 2)
    # TCP gripped 20 mm toward -gz (+y) from check centre -> shift TCP +20*cos(22) mm in y so check lands centred on tray
    place_x = TRAY_XY[0]
    place_y = TRAY_XY[1] + 0.020 * math.cos(down_ang)
    pre_place = (place_x, place_y, TOP + 0.105)

    yield from L.move(pre_place, duration=2.6, rot=r_place)
    yield from L.trim_to_measured(pre_place, max_duration=0.8)

    # Read live held check pose over the tray to cancel any slip from wedge extraction
    g_held = ctx.obj("green_check")
    dx_slip = TRAY_XY[0] - float(g_held.pos[0])
    dy_slip = TRAY_XY[1] - float(g_held.pos[1])
    if abs(dx_slip) < 0.045 and abs(dy_slip) < 0.045:
        place_x = float(L.measured_pos[0]) + dx_slip
        place_y = float(L.measured_pos[1]) + dy_slip - 0.002
        yield from L.move((place_x, place_y, TOP + 0.105), duration=1.0, rot=r_place)

    release_pos = (place_x, place_y, TOP + 0.036)
    yield from L.move(release_pos, duration=2.4, rot=r_place)
    yield from L.trim_to_measured(release_pos, max_duration=1.0)

    # 6) Clean symmetric release inside tray inner floor, rise clear, open wide, and park at READY
    yield from L.grip(FINGER_RELEASE, duration=0.45)
    yield from L.hold(0.5)
    retreat_up = (place_x, place_y, TOP + 0.135)
    yield from L.move(retreat_up, duration=2.0, finger=FINGER_OPEN, rot=r_place)
    yield from L.home(duration=4.5)

    # Hold parked clear with grippers open for the full 8 s success hold
    yield from ctx.wait(9.0)
