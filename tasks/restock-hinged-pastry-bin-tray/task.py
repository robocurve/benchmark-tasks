"""A countertop rear-hinged clear-acrylic bakery display bin (`20.0 x 20.0 x 7.5 cm` closed, with an unpowered
gravity-closing top lid pivoting on rear hinges across `0..85 deg` — dropping shut under gravity whenever released)
stands center-table containing an empty stainless pastry tray (`9.8 x 7.2 x 2.4 cm`). Beside the bin sit a slate
counter return mat and a stocked pastry tray carrying three golden croissants/buns, randomized left or right (`BIN_SIDE=L|R`).

Because the clear acrylic top cover swings shut under gravity the moment it is released—blocking tray access and
slamming if dropped unchecked—one arm must lift and hold the lid's front handle bar open (`>= 50 deg`) while the
other removes the empty tray onto the counter mat, places the stocked pastry tray flat inside the bin, clears out,
and allows the holding arm to lower the cover gently shut without slamming.
"""

from __future__ import annotations

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import binkit as bk  # noqa: E402
import guardkit as gk  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, ObjectState  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20
RUN_LIMIT_STEPS = 2400
INSTRUCTION = (
    "Replace the empty inner tray inside the hinged bakery bin with the stocked pastry tray, "
    "leaving the empty tray flat on the counter mat and the bin lid closed without letting it slam."
)

# -- fixed arm assignment, default L (set BIN_SIDE=R for the mirrored layout) -------------------------------------
SIDES = ("L", "R")


def _draw_side() -> str:
    env = os.environ.get("BIN_SIDE", "").strip().upper()
    return env if env in SIDES else "L"


BIN_SIDE = _draw_side()
SIDE_SIGN = 1.0 if BIN_SIDE == "L" else -1.0
print(f"pastry-bin: BIN_SIDE={BIN_SIDE}", file=sys.stderr)

# -- countertop rear-hinged pastry bin geometry (1:1 single-tray bakery display box, m) --------------------------------
BIN_X, BIN_Y = 0.304, 0.0  # spans x in [0.204, 0.404]
BIN_POS = (BIN_X, BIN_Y, TOP)
BIN_D = 0.200
BIN_IN_W = 0.184  # wide 18.4 cm tub easily clears 14.2 cm open YAM fingers around 7.2 cm tray
WALL_T = 0.008
BIN_OUT_W = BIN_IN_W + 2 * WALL_T  # 0.200 m (y in [-0.100, +0.100])
FLOOR_T = 0.008
WALL_H = 0.054  # lower tub rim top at TOP + 0.054; rear hinge pillars & lid dome top at TOP + 0.075
Z_BIN_FLOOR = TOP + FLOOR_T  # 0.758 m

# Hinge sits on rear pillars (+X edge) turning around +Y so positive angle lifts lid forward-up (-X -> +Z)
HINGE_LOCAL = (BIN_D / 2.0 - 0.006, 0.0, 0.064)  # (+0.094, 0.0, 0.064) -> world x = 0.398
HINGE_WORLD = (BIN_X + HINGE_LOCAL[0], BIN_Y, TOP + HINGE_LOCAL[2])
LID_MASS = 0.28
LID_UPPER_DEG = 85.0  # < 90 deg guarantees gravity always pulls lid closed
LID_DAMPING = 0.004

# In lid frame (origin at HINGE_LOCAL when angle = 0 closed):
LID_T = 0.007
LID_W = 0.196
LID_LEN = 0.194  # spans x_lid in [-0.194, 0.0] -> closed front edge at x_world = 0.204
SKIRT_H = 0.014  # low-profile front/side lip stays clear of the working arm's wrist behind it when open 82 deg
HANDLE_X = -0.212  # brushed steel front grab bar 18 mm forward of lid front edge
HANDLE_Z = 0.006
HANDLE_W = 0.228
HANDLE_THICK = 0.012
HANDLE_GRIP_Y = 0.096  # outer grip wing keeps holding arm well outside central tray-swap corridor

# -- stainless trays & counter mat (1:1 dimensions, m) -----------------------------------------------------------------
TRAY_X, TRAY_Y, TRAY_RIM_H = 0.098, 0.072, 0.024
TRAY_WALL = 0.003
TRAY_BASE_T = 0.004
TRAY_BIN_X = 0.267  # forward inside cavity [0.212, 0.396] so wrist/link5 clears raised rear-hinged lid cleanly
EMPTY_MASS = 0.22
STOCKED_MASS = 0.36

MAT_POS = (0.360, -SIDE_SIGN * 0.235, TOP)
MAT_SIZE = (0.130, 0.102, 0.004)
MAT_TOP_Z = TOP + MAT_SIZE[2]
STOCK_START = (0.215, -SIDE_SIGN * 0.235, TOP)

# -- scoring & safety thresholds ---------------------------------------------------------------------------------------
SLAM_SPEED_RAD_S = 1.80  # controlled lowering peaks near ~0.7 rad/s; unheld drop slams > 2.5 rad/s
SLAM_ANGLE_MAX = 0.75  # rad (~43 deg) before/at closed stop
LID_CLOSED_MAX_RAD = math.radians(4.5)
LID_STILL_VEL = 0.12
STOCK_TILT_MAX_DEG = 18.0
EMPTY_TILT_MAX_DEG = 22.0
TRAY_END_TILT_DEG = 6.0
TRAY_MAT_TILT_DEG = 10.0
STILL_SPEED = 0.005
STILL_SPIN = math.radians(5.0)
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
GRIPPERS = ("left_gripper", "right_gripper")

# -- visual materials --------------------------------------------------------------------------------------------------
ACRYLIC = bk.BinLook((0.86, 0.94, 0.98), roughness=0.12, metallic=0.0, opacity=0.34)
STEEL_BIN = bk.BinLook((0.74, 0.76, 0.79), roughness=0.30, metallic=0.80, opacity=1.0)
HINGE_DARK = bk.BinLook((0.18, 0.19, 0.21), roughness=0.55, metallic=0.60, opacity=1.0)

STEEL_TRAY = kit.Look((0.76, 0.78, 0.81), roughness=0.28, metallic=0.82)
PARCHMENT = kit.Look((0.96, 0.93, 0.84), roughness=0.75)
PASTRY_CRUST = kit.Look((0.78, 0.50, 0.20), roughness=0.65)
PASTRY_GLAZE = kit.Look((0.60, 0.34, 0.12), roughness=0.45)
SLATE_MAT = kit.Look((0.16, 0.22, 0.28), roughness=0.80)

# -- bin housing & articulated lid -------------------------------------------------------------------------------------
_yw = BIN_IN_W / 2.0 + WALL_T / 2.0
_front_wall_h = 0.042  # slightly lower front tub wall for clear pastry visibility when closed
_bin_boxes = (
    ((0.0, 0.0, FLOOR_T / 2.0), (BIN_D, BIN_OUT_W, FLOOR_T), STEEL_BIN),
    ((-BIN_D / 2.0 + WALL_T / 2.0, 0.0, _front_wall_h / 2.0 + FLOOR_T),
     (WALL_T, BIN_IN_W, _front_wall_h), ACRYLIC),
    ((BIN_D / 2.0 - WALL_T / 2.0, 0.0, 0.060 / 2.0), (WALL_T, BIN_OUT_W, 0.060), ACRYLIC),
    ((0.0, _yw, WALL_H / 2.0), (BIN_D, WALL_T, WALL_H), ACRYLIC),
    ((0.0, -_yw, WALL_H / 2.0), (BIN_D, WALL_T, WALL_H), ACRYLIC),
    ((HINGE_LOCAL[0], _yw - 0.004, 0.058), (0.014, 0.014, 0.016), HINGE_DARK),
    ((HINGE_LOCAL[0], -_yw + 0.004, 0.058), (0.014, 0.014, 0.016), HINGE_DARK),
)

_lid_boxes = (
    ((-LID_LEN / 2.0, 0.0, LID_T / 2.0 + 0.008), (LID_LEN, LID_W, LID_T), ACRYLIC),
    ((-LID_LEN + 0.004, 0.0, 0.002), (0.007, LID_W, SKIRT_H), ACRYLIC),
    ((-LID_LEN / 2.0, LID_W / 2.0 - 0.0035, 0.004), (LID_LEN, 0.007, 0.012), ACRYLIC),
    ((-LID_LEN / 2.0, -LID_W / 2.0 + 0.0035, 0.004), (LID_LEN, 0.007, 0.012), ACRYLIC),
    ((-0.010, 0.0, 0.012), (0.018, LID_W - 0.010, 0.006), STEEL_BIN),
    ((-LID_LEN - 0.008, 0.036, HANDLE_Z), (0.018, 0.006, 0.008), STEEL_BIN),
    ((-LID_LEN - 0.008, -0.036, HANDLE_Z), (0.018, 0.006, 0.008), STEEL_BIN),
    ((HANDLE_X, 0.0, HANDLE_Z), (HANDLE_THICK, HANDLE_W, HANDLE_THICK), STEEL_BIN),
)

PASTRY_BIN = bk.hinged_bin(
    "pastry_bin",
    pos=BIN_POS,
    boxes=_bin_boxes,
    door_boxes=_lid_boxes,
    hinge=HINGE_LOCAL,
    door_mass=LID_MASS,
    upper_deg=LID_UPPER_DEG,
    damping=LID_DAMPING,
    friction=0.55,
)


def _tray_shell_meshes() -> list:
    hx, hy = TRAY_X / 2.0, TRAY_Y / 2.0
    ix, iy = TRAY_X - 2 * TRAY_WALL, TRAY_Y - 2 * TRAY_WALL
    return [
        kit.box((0.0, 0.0, TRAY_BASE_T / 2.0), (TRAY_X, TRAY_Y, TRAY_BASE_T), STEEL_TRAY),
        kit.box((0.0, 0.0, TRAY_BASE_T + 0.0005), (ix - 0.004, iy - 0.004, 0.001), PARCHMENT),
        kit.box((-hx + TRAY_WALL / 2.0, 0.0, TRAY_RIM_H / 2.0), (TRAY_WALL, TRAY_Y, TRAY_RIM_H), STEEL_TRAY),
        kit.box((hx - TRAY_WALL / 2.0, 0.0, TRAY_RIM_H / 2.0), (TRAY_WALL, TRAY_Y, TRAY_RIM_H), STEEL_TRAY),
        kit.box((0.0, hy - TRAY_WALL / 2.0, TRAY_RIM_H / 2.0), (ix, TRAY_WALL, TRAY_RIM_H), STEEL_TRAY),
        kit.box((0.0, -hy + TRAY_WALL / 2.0, TRAY_RIM_H / 2.0), (ix, TRAY_WALL, TRAY_RIM_H), STEEL_TRAY),
    ]


EMPTY_TRAY = kit.Prop(
    "empty_tray",
    pos=(TRAY_BIN_X, BIN_Y, Z_BIN_FLOOR),
    mass=EMPTY_MASS,
    size=(TRAY_X, TRAY_Y, TRAY_RIM_H),
    meshes=tuple(_tray_shell_meshes()),
    colliders=(kit.Collider((0.0, 0.0, TRAY_RIM_H / 2.0), size=(TRAY_X, TRAY_Y, TRAY_RIM_H)),),
)

_pastry_meshes = _tray_shell_meshes() + [
    kit.cylinder((-0.020, -0.014, 0.018), 0.0115, 0.026, PASTRY_CRUST, axis="z"),
    kit.cylinder((-0.020, -0.014, 0.033), 0.009, 0.007, PASTRY_GLAZE, axis="z"),
    kit.cylinder((-0.020, 0.014, 0.018), 0.0115, 0.026, PASTRY_CRUST, axis="z"),
    kit.cylinder((-0.020, 0.014, 0.033), 0.009, 0.007, PASTRY_GLAZE, axis="z"),
    kit.cylinder((0.017, 0.0, 0.018), 0.0135, 0.028, PASTRY_CRUST, axis="z"),
    kit.cylinder((0.017, 0.0, 0.034), 0.0105, 0.007, PASTRY_GLAZE, axis="z"),
]

STOCKED_TRAY = kit.Prop(
    "stocked_tray",
    pos=STOCK_START,
    mass=STOCKED_MASS,
    size=(TRAY_X, TRAY_Y, 0.038),
    meshes=tuple(_pastry_meshes),
    colliders=(
        kit.Collider((0.0, 0.0, TRAY_RIM_H / 2.0), size=(TRAY_X, TRAY_Y, TRAY_RIM_H)),
        kit.Collider((0.0, 0.0, 0.032), size=(0.060, 0.044, 0.014)),
    ),
)

TRAY_MAT = kit.Prop(
    "tray_mat",
    pos=MAT_POS,
    mass=None,
    size=MAT_SIZE,
    meshes=(kit.box((0.0, 0.0, MAT_SIZE[2] / 2.0), MAT_SIZE, SLATE_MAT),),
    colliders=(kit.Collider((0.0, 0.0, MAT_SIZE[2] / 2.0), size=MAT_SIZE),),
)

OBJECTS = [PASTRY_BIN, EMPTY_TRAY, STOCKED_TRAY, TRAY_MAT]

TRAY_PARTS = (gk.Block((0.0, 0.0, TRAY_RIM_H / 2.0), (TRAY_X, TRAY_Y, TRAY_RIM_H)),)
LID_PARTS = (
    gk.Block((-LID_LEN / 2.0, 0.0, LID_T / 2.0 + 0.008), (LID_LEN, LID_W, LID_T)),
    gk.Block((HANDLE_X, 0.0, HANDLE_Z), (HANDLE_THICK, HANDLE_W, HANDLE_THICK)),
)


# -- measurements & scoring --------------------------------------------------------------------------------------------
def _tilt_deg(obj) -> float:
    x, y, _, _ = (float(v) for v in obj.quat_xyzw)
    return float(math.degrees(math.acos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0))))


def _lid_obj(b) -> ObjectState:
    if hasattr(b, "door_as_object"):
        return b.door_as_object()
    ang = float(getattr(b, "angle", 0.0))
    dpos, dquat = bk.compute_door_frame(b.pos, b.quat_xyzw, HINGE_LOCAL, ang)
    return ObjectState(pos=dpos, quat_xyzw=dquat, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=b.low, high=b.high)


def _bin_lid_slam(b) -> bool:
    ang = float(getattr(b, "angle", 0.0))
    vel = float(getattr(b, "door_vel", 0.0))
    return bool(ang <= SLAM_ANGLE_MAX and vel < -SLAM_SPEED_RAD_S)


def _tray_dropped(obj) -> bool:
    return float(np.asarray(obj.low)[2]) < TOP - 0.015


def _hazard_cond(objects) -> bool:
    b = objects["pastry_bin"]
    em = objects["empty_tray"]
    st = objects["stocked_tray"]
    return bool(
        _bin_lid_slam(b)
        or _tilt_deg(st) > STOCK_TILT_MAX_DEG
        or _tilt_deg(em) > EMPTY_TILT_MAX_DEG
        or _tray_dropped(st)
        or _tray_dropped(em)
    )


def _empty_outside_bin(em) -> bool:
    x, y = float(em.pos[0]), float(em.pos[1])
    return bool(abs(x - BIN_X) > BIN_D / 2.0 + 0.025 or abs(y) > BIN_OUT_W / 2.0 + 0.025)


def _empty_on_mat(em, max_tilt: float) -> bool:
    dx, dy = abs(float(em.pos[0]) - MAT_POS[0]), abs(float(em.pos[1]) - MAT_POS[1])
    inside = dx <= MAT_SIZE[0] / 2.0 - 0.012 and dy <= MAT_SIZE[1] / 2.0 - 0.010
    z_ok = abs(float(np.asarray(em.low)[2]) - MAT_TOP_Z) <= 0.008
    return bool(inside and z_ok and _tilt_deg(em) <= max_tilt)


def _stocked_in_bin(st, max_tilt: float) -> bool:
    dx, dy = abs(float(st.pos[0]) - TRAY_BIN_X), abs(float(st.pos[1]) - BIN_Y)
    inside = dx <= 0.028 and dy <= 0.032
    z_ok = abs(float(np.asarray(st.low)[2]) - Z_BIN_FLOOR) <= 0.008
    return bool(inside and z_ok and _tilt_deg(st) <= max_tilt)


def _lid_closed(b) -> bool:
    ang = float(getattr(b, "angle", 0.0))
    vel = abs(float(getattr(b, "door_vel", 0.0)))
    return bool(ang <= LID_CLOSED_MAX_RAD and vel <= LID_STILL_VEL)


def _touched_any(objects) -> bool:
    probe = dict(objects)
    probe["_lid"] = _lid_obj(objects["pastry_bin"])
    return bool(
        gk.touching(probe, "empty_tray", TRAY_PARTS, grippers=GRIPPERS)
        or gk.touching(probe, "stocked_tray", TRAY_PARTS, grippers=GRIPPERS)
        or gk.touching(probe, "_lid", LID_PARTS, grippers=GRIPPERS)
    )


def _still_all(objects) -> bool:
    return all(objects[n].resting(max_speed=STILL_SPEED, max_spin=STILL_SPIN) for n in ("empty_tray", "stocked_tray"))


HAZARD = gk.Latch(steps=1)


class _Tracker:
    def __init__(self):
        self.ep = kit.episode()
        self.key = None
        self.objs = None
        self.hold_steps = 0


_T = _Tracker()


def _step(objects) -> None:
    global _T
    if _T.ep != kit.episode():
        _T = _Tracker()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else None
    if (key is not None and key == _T.key) or (key is None and objects is _T.objs):
        return
    _T.key, _T.objs = key, objects
    HAZARD.feed(objects, _hazard_cond(objects))
    b = objects["pastry_bin"]
    em = objects["empty_tray"]
    st = objects["stocked_tray"]
    goal_ok = _empty_on_mat(em, TRAY_END_TILT_DEG) and _stocked_in_bin(st, TRAY_END_TILT_DEG) and _lid_closed(b)
    settled = goal_ok and _still_all(objects) and not _touched_any(objects)
    _T.hold_steps = _T.hold_steps + 1 if settled else 0
    if os.environ.get("BIN_TRACE") and n is not None and (_T.hold_steps or n % 10 == 0 or HAZARD.on):
        print(
            f"TRACE step={n} haz={HAZARD.on} ang={math.degrees(float(getattr(b, 'angle', 0.0))):.1f}deg "
            f"vel={float(getattr(b, 'door_vel', 0.0)):.2f} "
            f"em=({em.pos[0]:.3f},{em.pos[1]:.3f},{float(em.low[2])-TOP:.4f}) "
            f"st=({st.pos[0]:.3f},{st.pos[1]:.3f},{float(st.low[2])-TOP:.4f})",
            file=sys.stderr,
            flush=True,
        )


def success(objects) -> bool:
    _step(objects)
    return not HAZARD.on and _T.hold_steps > 0


def score(objects) -> float:
    _step(objects)
    if HAZARD.on:
        return 0.0
    if _T.hold_steps >= HOLD_STEPS:
        return 1.0
    em = objects["empty_tray"]
    st = objects["stocked_tray"]
    if _empty_on_mat(em, TRAY_MAT_TILT_DEG) and _stocked_in_bin(st, TRAY_MAT_TILT_DEG):
        return 0.75
    if _empty_on_mat(em, TRAY_MAT_TILT_DEG) and _tilt_deg(st) <= STOCK_TILT_MAX_DEG:
        return 0.50
    if _empty_outside_bin(em) and _tilt_deg(st) <= STOCK_TILT_MAX_DEG:
        return 0.25
    return 0.0


# -- symmetric bimanual scripted demo ----------------------------------------------------------------------------------
def _handle_world(q_rad: float, y_side: float) -> tuple[float, float, float]:
    """World TCP on the rear-hinged lid's front handle bar at opening angle ``q_rad``."""
    x = HINGE_WORLD[0] + HANDLE_X * math.cos(q_rad) + HANDLE_Z * math.sin(q_rad)
    z = HINGE_WORLD[2] - HANDLE_X * math.sin(q_rad) + HANDLE_Z * math.cos(q_rad)
    return (x, y_side, z)


def demo() -> dict[str, list]:
    """Door-side arm lifts the clear acrylic lid to 81 deg along its rear hinge arc and holds it against gravity while
    the opposite arm uses clean top-down grasps to move the empty tray onto the counter mat and place the stocked tray
    inside the open bin; once clear, the holding arm lowers the lid gently shut without slamming.

    Set ``DEMO_NO_HOLD=1`` to drop the lid immediately after lifting halfway, or ``DEMO_DROP_DOOR=1`` to release it from
    full open before lowering (negative-control hazard checks).
    """
    s = SIDE_SIGN
    hold_side = "left" if BIN_SIDE == "L" else "right"
    work_side = "right" if BIN_SIDE == "L" else "left"
    no_hold = os.environ.get("DEMO_NO_HOLD", "0") == "1"
    drop_door = os.environ.get("DEMO_DROP_DOOR", "0") == "1"

    yh = s * HANDLE_GRIP_Y
    ys = -s * 0.235

    yaw_h = -s * math.pi / 2.0
    pitch_0 = -0.45
    pitch_mid = -0.90
    pitch_hi = -1.25
    az_hi = -s * 0.35  # swings holding wrist outward away from central tray corridor when held open

    q_mid = math.radians(42.0)
    q_open = math.radians(82.0)

    def hpose(q_rad: float):
        return _handle_world(q_rad, yh)

    p0 = hpose(0.0)
    pmid = hpose(q_mid)
    phi = hpose(q_open)
    p_app = (p0[0] - 0.045, yh, p0[2] + 0.045)

    # Top-down work arm picks empty_tray -> tray_mat, then stocked_tray -> pastry_bin (~15.7s total work)
    z_bin_g = Z_BIN_FLOOR + 0.019
    z_mat_g = MAT_TOP_Z + 0.019
    z_stk_g = TOP + 0.019
    z_carry = TOP + 0.140  # clears 4.4 cm lower bin tub & closed pastry height cleanly while staying well under lid

    work = [
        Waypoint(4.3, (0.20, ys, z_carry), FINGER_OPEN, "wait_lid"),
        Waypoint(1.0, (TRAY_BIN_X, 0.0, z_carry), FINGER_OPEN, "over_empty"),
        Waypoint(0.9, (TRAY_BIN_X, 0.0, z_bin_g), FINGER_OPEN, "down_empty"),
        Waypoint(0.4, (TRAY_BIN_X, 0.0, z_bin_g), FINGER_CLOSED, "grip_empty"),
        Waypoint(0.8, (TRAY_BIN_X, 0.0, z_carry), FINGER_CLOSED, "lift_empty"),
        Waypoint(1.1, (MAT_POS[0], MAT_POS[1], z_carry), FINGER_CLOSED, "carry_empty"),
        Waypoint(0.8, (MAT_POS[0], MAT_POS[1], z_mat_g), FINGER_CLOSED, "down_mat"),
        Waypoint(0.4, (MAT_POS[0], MAT_POS[1], z_mat_g), FINGER_OPEN, "release_empty"),
        Waypoint(0.7, (MAT_POS[0], MAT_POS[1], z_carry), FINGER_OPEN, "up_mat"),
        Waypoint(0.9, (STOCK_START[0], STOCK_START[1], z_carry), FINGER_OPEN, "over_stock"),
        Waypoint(0.8, (STOCK_START[0], STOCK_START[1], z_stk_g), FINGER_OPEN, "down_stock"),
        Waypoint(0.4, (STOCK_START[0], STOCK_START[1], z_stk_g), FINGER_CLOSED, "grip_stock"),
        Waypoint(0.8, (STOCK_START[0], STOCK_START[1], z_carry), FINGER_CLOSED, "lift_stock"),
        Waypoint(1.1, (TRAY_BIN_X, 0.0, z_carry), FINGER_CLOSED, "carry_stock"),
        Waypoint(0.9, (TRAY_BIN_X, 0.0, z_bin_g + 0.002), FINGER_CLOSED, "down_bin"),
        Waypoint(0.4, (TRAY_BIN_X, 0.0, z_bin_g + 0.002), FINGER_OPEN, "release_stock"),
        Waypoint(0.8, (TRAY_BIN_X, 0.0, z_carry), FINGER_OPEN, "up_bin"),
        Waypoint(1.0, (0.18, ys, z_carry), FINGER_OPEN, "clear_away"),
        Waypoint(6.5, (0.18, ys, z_carry), FINGER_OPEN, "wait_close"),
    ]
    # work clears away by t = 17.5s

    if no_hold:
        hold = [
            kit.Tilted(1.0, p_app, FINGER_OPEN, "approach_bar", yaw=yaw_h, pitch=pitch_0),
            kit.Tilted(0.7, p0, FINGER_OPEN, "at_bar", yaw=yaw_h, pitch=pitch_0),
            kit.Tilted(0.4, p0, FINGER_CLOSED, "grip_bar", yaw=yaw_h, pitch=pitch_0),
            kit.Tilted(0.9, pmid, FINGER_CLOSED, "crack_open", yaw=yaw_h, pitch=pitch_mid),
            kit.Tilted(0.3, pmid, FINGER_OPEN, "drop_early", yaw=yaw_h, pitch=pitch_mid),
            Waypoint(1.0, (0.14, s * 0.24, TOP + 0.20), FINGER_OPEN, "retreat"),
            Waypoint(14.0, (0.14, s * 0.24, TOP + 0.20), FINGER_OPEN, "park"),
        ]
    else:
        hold = [
            kit.Tilted(1.0, p_app, FINGER_OPEN, "approach_bar", yaw=yaw_h, pitch=pitch_0),
            kit.Tilted(0.7, p0, FINGER_OPEN, "at_bar", yaw=yaw_h, pitch=pitch_0),
            kit.Tilted(0.4, p0, FINGER_CLOSED, "grip_bar", yaw=yaw_h, pitch=pitch_0),
            kit.Tilted(1.0, pmid, FINGER_CLOSED, "swing_mid", yaw=yaw_h, pitch=pitch_mid, azim=0.5 * az_hi),
            kit.Tilted(1.0, phi, FINGER_CLOSED, "swing_open", yaw=yaw_h, pitch=pitch_hi, azim=az_hi),
            kit.Tilted(13.8, phi, FINGER_CLOSED, "hold_open", yaw=yaw_h, pitch=pitch_hi, azim=az_hi),  # holds until t=17.9s
        ]
        if drop_door:
            hold += [
                kit.Tilted(0.3, phi, FINGER_OPEN, "release_high", yaw=yaw_h, pitch=pitch_hi, azim=az_hi),
                Waypoint(2.0, (0.14, s * 0.24, TOP + 0.20), FINGER_OPEN, "clear"),
            ]
        else:
            p_hi2 = hpose(math.radians(55.0))
            p_lo2 = hpose(math.radians(28.0))
            p_seat = (p0[0], yh, p0[2] - 0.008)  # gently seats lid flush at 0 deg on rim before releasing
            hold += [
                kit.Tilted(0.9, p_hi2, FINGER_CLOSED, "lower_hi", yaw=yaw_h, pitch=-1.00, azim=0.6 * az_hi),
                kit.Tilted(0.9, p_lo2, FINGER_CLOSED, "lower_mid", yaw=yaw_h, pitch=-0.72, azim=0.25 * az_hi),
                kit.Tilted(1.0, p_seat, FINGER_CLOSED, "lower_gentle", yaw=yaw_h, pitch=pitch_0),
                kit.Tilted(0.5, p_seat, FINGER_OPEN, "release_bar", yaw=yaw_h, pitch=pitch_0),
                kit.Tilted(0.8, (p_seat[0] - 0.055, yh, p_seat[2]), FINGER_OPEN, "pull_back", yaw=yaw_h, pitch=pitch_0),
                Waypoint(1.8, (0.14, s * 0.24, TOP + 0.20), FINGER_OPEN, "park"),
            ]

    return {hold_side: hold, work_side: work}
