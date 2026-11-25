"""On the station table, an ESD bench mat. On it, a black four-inch waffle pack (Entegris H44-999-62C02 class, nine
pockets) with one lidless exposed-die FCBGA chip (AMD/Xilinx SBG484 class: 19 x 19 x 2.5 mm with a 3.5 mm 45-degree
pin-1 corner chamfer, bare 14 x 14 mm silicon die on top) lying in its centre pocket, 0.8 mm below the pocket rim, with
no edge to grip. Beside it a keyed open-top BGA test-socket nest (Enplas OTB / Ironwood 19 mm class, ivory PEEK,
19.5 x 19.5 mm locating pocket with pin-1 corner key and 45-degree funnel), and on the far side a slate bench cradle
holding a Virtual Industries PEN-VAC V8901-A-ESD vacuum pen (12.7 mm barrel, 45 degree bent probe, 9.53 mm silicone cup,
side push-button) horizontally.

The robot must use the pen as a person does: one hand holds the pen pitched 45 degrees so the cup is flat above the die,
the other hand squeezes the side button BEFORE touchdown (this expels the bellows), the cup makes actual contact and
presses onto the die (>= 0.6 N for >= 0.20 s) before the button is released to draw the vacuum. The chip is lifted clear
of the pocket walls, transferred under suction over the socket and lowered to hover 4 mm above its floor with pin-1
corners aligned, the button is squeezed again to break the vacuum (the chip drops flat and keyed into the nest) and the
pen is put back on its cradle. A gripper pad touching the chip, a drop, or a pen off the table scores 0.

Requires -E object_jitter=0 (fixed layout).

Run it:  inspect-robots run --embodiment yam_arena -E task=transfer_exposed_die_fcbga_with_bent_vacuum_pen.py \\
           -E object_jitter=0 --instruction "..." --scorer task_score ...
"""

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import belimits  # noqa: E402
import mechkit  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402
from mechkit import (  # noqa: E402
    Joint, Mech, box, cone, cyl, extrude, on_reset, on_step, sweep, touch_state, watch_touch,
)
from yam_arena import ik  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # Isaac ends the episode once success() holds this long; the arms are parked by then
SCORE_OVER = "end"
CONTROL_DT = 0.05  # s per control step (20 Hz)
S2 = math.sqrt(0.5)

# Colours (sRGB)
MAT = (0.48, 0.58, 0.68)  # ESD mat
CHARCOAL = (0.14, 0.15, 0.17)  # waffle pack
EMERALD = (0.12, 0.52, 0.24)  # chip substrate
SILICON = (0.84, 0.89, 0.95)  # bare die
GOLD = (0.90, 0.72, 0.15)
IVORY = (0.88, 0.85, 0.76)  # PEEK socket
DARK = (0.10, 0.10, 0.12)
SLATE = (0.22, 0.28, 0.36)  # cradle
SILVER = (0.80, 0.81, 0.84)  # pen barrel
STEEL = (0.55, 0.56, 0.60)
BLACK = (0.04, 0.04, 0.05)


def _ghost(shape):
    return dataclasses.replace(shape, collide=False)


# ---------------------------------------------------------------------------------------------------------------------
# Real sizes. Everything sits on a 4 mm ESD mat (top at Z_MAT).
MAT_T = 0.004
Z_MAT = TOP + MAT_T
MAT_XY, MAT_SIZE = (0.270, -0.045), (0.220, 0.360)
MAT_BOX = Box("mat", pos=(MAT_XY[0], MAT_XY[1], TOP + MAT_T / 2), size=(MAT_SIZE[0], MAT_SIZE[1], MAT_T), color=MAT)

# Entegris H44-999-62C02 (4 in STAT-PRO 3 x 3 waffle pack): 101.6 x 101.6 x 6.5 mm, 25.32 mm square pockets, 30.8 mm pitch,
# 3.2 mm floor, 3.3 mm tall walls (5.48 mm between pockets, 7.34 mm round the border).
WAFFLE_XY = (0.305, 0.045)
PACK = 0.1016
POCKET, PACK_PITCH = 0.02532, 0.0308
FLOOR_T, RIB_H = 0.0032, 0.0033
Z_SRC = Z_MAT + FLOOR_T  # pocket floor
RIM = Z_MAT + FLOOR_T + RIB_H  # pocket rim, TOP + 0.0105
_ribs = []
for _c, _w in ((PACK_PITCH / 2, 0.00548), (-PACK_PITCH / 2, 0.00548), (PACK_PITCH + POCKET / 2 + 0.00734 / 2, 0.00734),
               (-(PACK_PITCH + POCKET / 2 + 0.00734 / 2), 0.00734)):
    _ribs.append(box((_c, 0, FLOOR_T + RIB_H / 2), (_w, PACK, RIB_H), CHARCOAL))
    _ribs.append(box((0, _c, FLOOR_T + RIB_H / 2), (PACK, _w, RIB_H), CHARCOAL))
WAFFLE = Mech("waffle_pack", pos=(WAFFLE_XY[0], WAFFLE_XY[1], Z_MAT), mass=None, friction=0.5,
              shapes=[box((0, 0, FLOOR_T / 2), (PACK, PACK, FLOOR_T), CHARCOAL), *_ribs])
POCKET_CENTRES = [(WAFFLE_XY[0] + i * PACK_PITCH, WAFFLE_XY[1] + j * PACK_PITCH) for i in (-1, 0, 1) for j in (-1, 0, 1)]

# AMD/Xilinx SBG484 lidless FCBGA: 19 x 19 mm substrate 1.75 mm thick with a 3.5 mm 45-degree pin-1 chamfer at (-x, -y)
# (balls under it are not modelled), bare silicon die 14 x 14 x 0.75 mm on top. Frame: centre of the underside.
CHIP_A, SUB_T, DIE_A, DIE_T = 0.019, 0.00175, 0.014, 0.00075
CHIP_HALF, CHAMFER = CHIP_A / 2, 0.0035
CHIP_H = SUB_T + DIE_T  # 2.5 mm
CHIP_MASS = 0.005
CHIP_START = (WAFFLE_XY[0], WAFFLE_XY[1], Z_SRC)
DIE_TOP_START = Z_SRC + CHIP_H  # TOP + 0.0097
_chip_poly = [
    (-CHIP_HALF + CHAMFER, -CHIP_HALF), (CHIP_HALF, -CHIP_HALF), (CHIP_HALF, CHIP_HALF),
    (-CHIP_HALF, CHIP_HALF), (-CHIP_HALF, -CHIP_HALF + CHAMFER),
]
CHIP = Mech(
    "chip", pos=CHIP_START, mass=CHIP_MASS, friction=0.7, iterations=32,
    shapes=[
        extrude((0, 0, SUB_T / 2), _chip_poly, SUB_T, EMERALD, axis="z"),
        box((0, 0, SUB_T + DIE_T / 2), (DIE_A, DIE_A, DIE_T), SILICON),
        _ghost(box((-0.0052, -0.0052, CHIP_H + 0.00005), (0.0032, 0.0032, 0.0001), GOLD)),  # pin-1 mark (look only)
    ],
)

# Open-top 19 mm BGA test-socket nest (Enplas OTB-484 / Ironwood SG-BGA 19 mm class): 38 x 38 x 10 mm PEEK body,
# 19.5 x 19.5 mm locating cavity (0.25 mm/side clearance), 2.5 mm vertical walls with a 3.65 mm 45-deg pin-1 corner
# key at (-x, -y), then a 45-degree lead-in funnel to a 26.5 x 26.5 mm mouth.
SOCKET_XY = (0.305, -0.065)
SOCK = 0.038
Z_DST = Z_MAT + 0.004  # nest floor, TOP + 0.0080
LOC_Z = 0.0065  # top of vertical locating walls (2.5 mm above floor)
SOCK_TOP = Z_MAT + 0.010
OPEN_HALF, MOUTH_HALF = 0.00975, 0.01325
KEY_LEG = 0.00365  # pin-1 corner key leg (x + y = -0.01585: 0.25 mm clearance to chip chamfer x + y = -0.01550)
_wall_x = [(0.004, OPEN_HALF), (0.004, SOCK / 2), (0.010, SOCK / 2), (0.010, MOUTH_HALF), (LOC_Z, OPEN_HALF)]  # (z, x)
_wall_y = [(OPEN_HALF, 0.004), (SOCK / 2, 0.004), (SOCK / 2, 0.010), (MOUTH_HALF, 0.010), (OPEN_HALF, LOC_Z)]  # (y, z)
_key_poly = [(-OPEN_HALF, -OPEN_HALF), (-OPEN_HALF + KEY_LEG, -OPEN_HALF), (-OPEN_HALF, -OPEN_HALF + KEY_LEG)]
SOCKET = Mech(
    "test_socket", pos=(SOCKET_XY[0], SOCKET_XY[1], Z_MAT), mass=None, friction=0.5,
    shapes=[
        box((0, 0, 0.002), (SOCK, SOCK, 0.004), IVORY),
        extrude((0, 0, 0), _wall_x, SOCK, IVORY, axis="y"),
        extrude((0, 0, 0), [(z, -x) for z, x in _wall_x], SOCK, IVORY, axis="y"),
        extrude((0, 0, 0), _wall_y, SOCK, IVORY, axis="x"),
        extrude((0, 0, 0), [(-y, z) for y, z in _wall_y], SOCK, IVORY, axis="x"),
        extrude((0, 0, (0.004 + LOC_Z) / 2), _key_poly, LOC_Z - 0.004, IVORY, axis="z"),
        _ghost(box((0, 0, 0.00405), (2 * OPEN_HALF - 0.0002, 2 * OPEN_HALF - 0.0002, 0.0001), DARK)),  # dark floor
        _ghost(box((-0.0158, -0.0158, 0.01005), (0.0045, 0.0045, 0.0001), GOLD)),  # pin-1 mark at (-x, -y)
    ],
)

# Virtual Industries PEN-VAC V8901-A-ESD: aluminium barrel 12.7 mm OD x 146 mm, 36 g, 45 degree bent V3950-B probe
# (18 ga) and V9038-ESD 9.53 mm conductive silicone cup, side push-button. Frame: barrel centre, nose along +x, button up.
PEN_R, PEN_L = 0.00635, 0.146
PROBE_R = 0.0015  # thickened collider (real 18 ga tube is 1.27 mm OD)
CUP_R, CUP_T = 0.004765, 0.005
_p0 = (0.096, 0.0, 0.0)
_p1 = (0.096 + 0.018 * S2, 0.0, -0.018 * S2)
CUP_DIR = np.array([S2, 0.0, -S2])  # cup axis, pointing out of the cup face
CUP_C = np.array(_p1) + CUP_DIR * (CUP_T / 2)
CUP_FACE = np.array(_p1) + CUP_DIR * CUP_T  # centre of the cup face in the pen frame
BTN_AT = (0.033, 0.0, 0.009)
BTN_TRAVEL, BTN_SPRING = 0.0035, 714.0  # 2.5 N at full travel
BTN_SIZE = (0.014, 0.010, 0.0045)
BTN_TOP = BTN_AT[2] + BTN_SIZE[2] / 2  # 11.25 mm above the axis at rest
PEN_BASE_MASS, BTN_MASS = 0.036, 0.004
PEN_REST = (0.245, -0.175, TOP + 0.045)  # barrel axis on the cradle
PEN = Mech(
    "vac_pen", pos=PEN_REST, mass=PEN_BASE_MASS, friction=0.9, iterations=32,
    shapes=[
        cyl((0, 0, 0), PEN_R, PEN_L, SILVER, axis="x", sides=12),
        cone((PEN_L / 2, 0, 0), PEN_R, 0.002, 0.015, SILVER, axis="x"),
        sweep((0, 0, 0), [(0.088, 0, 0), _p0, _p1], PROBE_R, STEEL, ends="round"),
        cyl(tuple(CUP_C), CUP_R, CUP_T, BLACK, axis="z", tilt=mechkit._quat_z_to(tuple(CUP_DIR)), sides=12),
    ],
    joints=[
        Joint("button", "slide", axis="z", at=BTN_AT, low=-BTN_TRAVEL, high=0.0, spring=BTN_SPRING, damping=0.5,
              mass=BTN_MASS, armature=0.025, friction=0.9, shapes=[box((0, 0, 0), BTN_SIZE, BLACK)]),
    ],
)

# Twin V-saddle bench cradle (printed): saddles 10 mm thick, 90 mm apart inside, holding the barrel axis 41 mm above the
# mat, and a rear stop 4 mm behind the barrel's end.
CRADLE_XY = (PEN_REST[0], PEN_REST[1])
SADDLE_U = 0.050  # saddle centres at +-50 mm along the barrel
AXIS_H = PEN_REST[2] - Z_MAT  # 41 mm
V_BOT = AXIS_H - PEN_R * math.sqrt(2.0) - 0.0003
_saddle = [(-0.020, 0.0), (0.020, 0.0), (0.020, V_BOT + 0.020), (0.0, V_BOT), (-0.020, V_BOT + 0.020)]  # (y, z)
CRADLE = Mech(
    "pen_cradle", pos=(CRADLE_XY[0], CRADLE_XY[1], Z_MAT), mass=None, friction=0.8,
    shapes=[
        box((0, 0, 0.0015), (0.110, 0.040, 0.003), SLATE),
        extrude((-SADDLE_U, 0, 0), _saddle, 0.010, SLATE, axis="x"),
        extrude((SADDLE_U, 0, 0), _saddle, 0.010, SLATE, axis="x"),
        box((-PEN_L / 2 - 0.006, 0, 0.024), (0.004, 0.030, 0.048), SLATE),  # rear stop, x 0.168
    ],
)

OBJECTS = [MAT_BOX, WAFFLE, CHIP, SOCKET, CRADLE, PEN]

_plain_author = mechkit._author


def _author_with_cup_contact(prim_path: str, cfg, translation=None, orientation=None, **kw):
    prim = _plain_author(prim_path, cfg, translation=translation, orientation=orientation, **kw)
    if getattr(cfg, "mech_name", "") in ("vac_pen", "chip") and prim.IsValid():
        from pxr import PhysxSchema

        base_prim = prim.GetStage().GetPrimAtPath(f"{prim_path}/base")
        if base_prim.IsValid():
            PhysxSchema.PhysxContactReportAPI.Apply(base_prim).CreateThresholdAttr(0.0)
    return prim


mechkit._author = _author_with_cup_contact

# Contact watch (scorer side): a finger pad touching the chip.
watch_touch("chip")

# ---------------------------------------------------------------------------------------------------------------------
# Keep-out for the robot: no part of either arm may come within 4 cm of the chip's pocket below 3.5 cm above the table
# (only the pen's cup and the chip go there; a pad on the chip scores 0).
_KO = POCKET / 2 + 0.04
ARM_KEEP_OUT = [
    {"name": "chip pocket and 4 cm round it", "x": (WAFFLE_XY[0] - _KO, WAFFLE_XY[0] + _KO),
     "y": (WAFFLE_XY[1] - _KO, WAFFLE_XY[1] + _KO), "z_top": TOP + 0.035},
]

# ---------------------------------------------------------------------------------------------------------------------
# Frames and states


def _rot(q) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(q, dtype=float))


def pen_frame(pen_s):
    """(origin, 3x3 rotation) of the pen frame from its state."""
    org = np.asarray(pen_s.origin if getattr(pen_s, "origin", None) is not None else pen_s.pos, float)
    return org, _rot(pen_s.quat_xyzw)


def chip_frame(chip_s):
    org = np.asarray(chip_s.origin if getattr(chip_s, "origin", None) is not None else chip_s.pos, float)
    return org, _rot(chip_s.quat_xyzw)


def cup_face(pen_s) -> np.ndarray:
    org, R = pen_frame(pen_s)
    return org + R @ CUP_FACE


def cup_tilt_deg(pen_s) -> float:
    """Angle between the cup's axis and straight down: 0 is a flat cup."""
    _, R = pen_frame(pen_s)
    d = R @ CUP_DIR
    return float(math.degrees(math.acos(max(-1.0, min(1.0, -d[2])))))


def die_top(chip_s) -> np.ndarray:
    org, R = chip_frame(chip_s)
    return org + R @ np.array([0.0, 0.0, CHIP_H])


def chip_tilt_deg(chip_s) -> float:
    return float(belimits.tilt_deg(chip_s))


def chip_yaw_deg(chip_s) -> float:
    """Chip yaw angle (deg) in [-180, 180] relative to the aligned pin-1 corner (-x, -y)."""
    _, R = chip_frame(chip_s)
    return float(math.degrees(math.atan2(R[1, 0], R[0, 0])))


CUP_SHAPE = PEN.shapes[3]
DIE_SHAPE = CHIP.shapes[1]


def cup_die_gap(pen_s, chip_s) -> float:
    """Signed separation (m) between the silicone cup collider and the bare die collider."""
    p_org, p_R = pen_frame(pen_s)
    c_org, c_R = chip_frame(chip_s)
    return float(mechkit.gap(CUP_SHAPE, (p_R, p_org), DIE_SHAPE, (c_R, c_org)))


def pen_pitch_deg(pen_s) -> float:
    """Barrel's angle from horizontal."""
    _, R = pen_frame(pen_s)
    return float(math.degrees(math.asin(max(-1.0, min(1.0, (R @ np.array([1.0, 0, 0]))[2])))))


def button_depth(pen_s) -> float:
    return -float(pen_s.joints.get("button", 0.0))


def _in_pocket(xy, tol=0.0) -> bool:
    return any(abs(xy[0] - c[0]) <= POCKET / 2 + tol and abs(xy[1] - c[1]) <= POCKET / 2 + tol for c in POCKET_CENTRES)


def _in_mouth(xy) -> bool:
    return abs(xy[0] - SOCKET_XY[0]) <= MOUTH_HALF and abs(xy[1] - SOCKET_XY[1]) <= MOUTH_HALF


# ---------------------------------------------------------------------------------------------------------------------
# Scoring (numbers match task.json and README.md)
PRIME_DEPTH = 0.0025  # button squeezed at least 2.5 mm: bellows expelled / vacuum broken
RELEASE_DEPTH = 0.0006  # button back within 0.6 mm of rest: vacuum drawn
HOVER_MIN = 0.005  # priming only counts with the cup at least 5 mm above the die (before touchdown)
SEAL_TILT = 4.0  # deg
SEAL_GAP = 0.0003  # m: max collider gap between cup and die (no air gap)
SEAL_DZ_LO, SEAL_DZ_HI = -0.0015, 0.0003  # m: cup face centre relative to die top (allows compliant press, no gap)
SEAL_DXY = 0.0020  # m: 9.53 mm cup stays wholly inside the 14 x 14 mm die
SEAL_FORCE = 0.6  # N: minimum cup-to-die normal contact force for seal-forming press
SEAL_PRESS_STEPS = 4  # 0.20 s of sustained cup-to-die contact and press while primed
LIFT_Z = RIM + 0.010  # chip underside 10 mm above the pocket rim: TOP + 0.0205
DROP_MAX = 0.015
HARD_VZ = -0.55
BAD_STEPS = 2  # 0.1 s
FINAL_STEPS = 40  # 2.0 s
STILL = 0.005  # m/s
SOCK_XY_TOL, SOCK_Z_TOL, SOCK_TILT, SOCK_YAW_TOL = 0.0010, 0.0010, 3.0, 8.0
PEN_XY_TOL, PEN_TILT = 0.015, 10.0
RELEASE_CLEAR = 0.030
PEN_FLOOR = TOP - 0.05

FORCE_RELEASED: list = [None]  # offline test hook for check.py; None in live runs
FORCE_CUP_FORCE: list = [None]  # offline/diagnostic hook for cup-to-die force (N); None in live Isaac runs
_ST: dict = {}


def _reset_state(env=None) -> None:
    _ST.update(last=None, step=0, ctrl_step=0, primed=False, pressed=False, sealed=False, lifted=False, transferred=False,
               zero=False, why="", rel_pos=None, rel_R=None, bad_touch=0, bad_vz=0, bad_mat=0, rel_chip=0, rel_pen=0,
               still_chip=0, still_pen=0, seals=0, broke=0, press_steps=0, press_peak=0.0, cup_force_step=0.0)


_reset_state()


@on_reset
def _on_reset(env=None) -> None:
    _reset_state(env)
    _install_weld(env)


def _fail(why: str) -> None:
    if not _ST["zero"]:
        _ST["zero"], _ST["why"] = True, why


def _released(part, objects=None, clearance=RELEASE_CLEAR) -> bool:
    if FORCE_RELEASED[0] is not None:
        return bool(FORCE_RELEASED[0])
    if mechkit._ENV.get("env") is None:
        return True
    try:
        return bool(mechkit.released(part, clearance=clearance))
    except Exception:
        return False


def _touching() -> bool:
    try:
        t = touch_state("chip")
        return bool(t.touching)
    except Exception:
        return False


def _consume_cup_force():
    """Peak cup-to-die normal force (N) over this step, or None if live PhysX contact view is unavailable."""
    if FORCE_CUP_FORCE[0] is not None:
        return float(FORCE_CUP_FORCE[0])
    if _ST.get("cup_view_ok"):
        f = float(_ST.get("cup_force_step", 0.0))
        _ST["cup_force_step"] = 0.0
        return f
    if mechkit._ENV.get("env") is None:
        return 0.0
    return None


def _update(objects) -> None:
    """Advance every latch once per control step (score() and success() get the same dict each step)."""
    ctrl = mechkit.step_count()
    if ctrl > 0:
        if ctrl == _ST["ctrl_step"]:
            return
        _ST["ctrl_step"] = ctrl
    elif objects is _ST["last"]:
        return
    _ST["last"] = objects
    _ST["step"] += 1
    pen_s, chip_s = objects["vac_pen"], objects["chip"]
    if not all(np.all(np.isfinite(np.asarray(v, float))) for v in (pen_s.pos, pen_s.quat_xyzw, chip_s.pos,
                                                                   chip_s.quat_xyzw)):
        _fail("nonfinite state")
        return
    depth = button_depth(pen_s)
    cup = cup_face(pen_s)
    die = die_top(chip_s)
    dz = float(cup[2] - die[2])
    dxy = float(math.hypot(cup[0] - die[0], cup[1] - die[1]))
    c_gap = cup_die_gap(pen_s, chip_s)
    tilt = cup_tilt_deg(pen_s)
    in_contact = (c_gap <= SEAL_GAP and SEAL_DZ_LO <= dz <= SEAL_DZ_HI and dxy <= SEAL_DXY and tilt <= SEAL_TILT)
    cup_f = _consume_cup_force()
    press_ok = in_contact and (cup_f is None or cup_f >= SEAL_FORCE)

    # --- the button: crossing into 'squeezed' primes (cup above the die) or breaks a seal; rising to 'released' seals
    if depth >= PRIME_DEPTH and not _ST["pressed"]:
        _ST["pressed"] = True
        _ST["press_steps"] = 0
        _ST["press_peak"] = 0.0
        if _ST["sealed"]:
            _ST["sealed"] = False
            _ST["broke"] += 1
            _on_release(chip_s)
        else:
            _ST["primed"] = bool(dz >= HOVER_MIN or dxy > 0.010)
    elif depth <= RELEASE_DEPTH and _ST["pressed"]:
        _ST["pressed"] = False
        if _ST["primed"] and not _ST["sealed"]:
            if (in_contact and _ST["press_steps"] >= SEAL_PRESS_STEPS
                    and (cup_f is None or cup_f >= SEAL_FORCE or _ST["press_peak"] >= SEAL_FORCE)):
                _seal(pen_s, chip_s, cup, die)
        _ST["primed"] = False
        _ST["press_steps"] = 0
        _ST["press_peak"] = 0.0
    elif depth <= RELEASE_DEPTH and not _ST["pressed"]:
        _ST["primed"] = False  # released in mid-air: the bellows is spent
        _ST["press_steps"] = 0
        _ST["press_peak"] = 0.0
    elif _ST["primed"] and not _ST["sealed"]:
        if press_ok:
            _ST["press_steps"] += 1
            if cup_f is not None:
                _ST["press_peak"] = max(_ST["press_peak"], cup_f)
        elif c_gap > 0.0006 or dz > 0.0006:
            _ST["press_steps"] = 0
            _ST["press_peak"] = 0.0

    # --- violations
    t_bad = _touching()
    _ST["bad_touch"] = _ST["bad_touch"] + 1 if t_bad else 0
    if _ST["bad_touch"] >= BAD_STEPS:
        _fail("a gripper pad touched the chip")
    vz = float(np.asarray(chip_s.lin_vel, float)[2])
    _ST["bad_vz"] = _ST["bad_vz"] + 1 if (vz < HARD_VZ and not _ST["sealed"]) else 0
    if _ST["bad_vz"] >= BAD_STEPS:
        _fail("chip fell hard")
    xy = np.asarray(chip_s.pos, float)[:2]
    on_mat = (float(chip_s.low[2]) < TOP + 0.006 and not _in_pocket(xy, 0.001) and not _in_mouth(xy)
              and not _ST["sealed"])
    _ST["bad_mat"] = _ST["bad_mat"] + 1 if on_mat else 0
    if _ST["bad_mat"] >= BAD_STEPS:
        _fail("chip dropped onto the table or the pack rim")
    if float(pen_s.low[2]) < PEN_FLOOR:
        _fail("pen fell off the table")
    if _ST["sealed"]:
        lo, hi = np.asarray(chip_s.low, float), np.asarray(chip_s.high, float)
        over_pack = bool(abs(chip_s.pos[0] - WAFFLE_XY[0]) < PACK / 2 and abs(chip_s.pos[1] - WAFFLE_XY[1]) < PACK / 2)
        if over_pack and float(chip_s.low[2]) < RIM - 0.001:  # still below the rim: it must stay inside its pocket
            if not any(lo[0] >= c[0] - POCKET / 2 - 0.0005 and hi[0] <= c[0] + POCKET / 2 + 0.0005
                       and lo[1] >= c[1] - POCKET / 2 - 0.0005 and hi[1] <= c[1] + POCKET / 2 + 0.0005
                       for c in POCKET_CENTRES):
                _fail("chip dragged through a pocket wall")
        if float(chip_s.low[2]) >= LIFT_Z:
            _ST["lifted"] = True

    # --- settle counters for the end-of-run tiers
    _ST["rel_chip"] = _ST["rel_chip"] + 1 if _released(chip_s) else 0
    _ST["rel_pen"] = _ST["rel_pen"] + 1 if _released(pen_s) else 0
    cs = float(np.linalg.norm(np.asarray(chip_s.lin_vel, float)))
    ps = float(np.linalg.norm(np.asarray(pen_s.lin_vel, float)))
    _ST["still_chip"] = _ST["still_chip"] + 1 if cs < STILL else 0
    _ST["still_pen"] = _ST["still_pen"] + 1 if ps < STILL else 0


def _seal(pen_s, chip_s, cup, die) -> None:
    """The cup is pressed flat and centred on the die: vacuum is drawn. Chip offset is recorded flush to the cup face."""
    org, R = pen_frame(pen_s)
    corg, CR = chip_frame(chip_s)
    shift = cup - die  # the die top meets the cup face
    _ST["sealed"] = True
    _ST["seals"] += 1
    _ST["rel_pos"] = R.T @ (corg + shift - org)
    _ST["rel_R"] = R.T @ CR


def _on_release(chip_s) -> None:
    """The vacuum is broken: the chip must be over a pocket or the nest and no more than 15 mm above its floor."""
    xy = np.asarray(chip_s.pos, float)[:2]
    z = float(chip_s.low[2])
    print(f"VACUUM released: chip at dx {1000 * (xy[0] - SOCKET_XY[0]):+.2f} dy {1000 * (xy[1] - SOCKET_XY[1]):+.2f} mm "
          f"from the nest, underside {1000 * (z - Z_DST):+.2f} mm, yaw {chip_yaw_deg(chip_s):+.2f} deg, "
          f"tilt {chip_tilt_deg(chip_s):.2f} deg", flush=True)
    if _in_mouth(xy):
        floor = Z_DST
        if _ST["lifted"]:
            _ST["transferred"] = True
    elif _in_pocket(xy):
        floor = Z_SRC
    else:
        _fail("chip released away from the pack and the nest")
        return
    if z - floor > DROP_MAX:
        _fail("chip released more than 15 mm above its floor")


def _on_socket_floor(chip_s) -> bool:
    org, _ = chip_frame(chip_s)
    return bool(math.hypot(org[0] - SOCKET_XY[0], org[1] - SOCKET_XY[1]) <= SOCK_XY_TOL
                and -0.001 <= float(chip_s.low[2]) - Z_DST <= SOCK_Z_TOL
                and chip_tilt_deg(chip_s) <= SOCK_TILT
                and abs(chip_yaw_deg(chip_s)) <= SOCK_YAW_TOL)


def _pen_home(pen_s) -> bool:
    org, _ = pen_frame(pen_s)
    return bool(float(np.linalg.norm(org - np.array(PEN_REST))) <= PEN_XY_TOL and abs(pen_pitch_deg(pen_s)) <= PEN_TILT)


def _tier(objects) -> float:
    pen_s, chip_s = objects["vac_pen"], objects["chip"]
    if _ST["sealed"] or not _ST["lifted"]:
        return 0.0  # a chip still on the pen, or never lifted by suction, is not placed
    if _ST["rel_chip"] < FINAL_STEPS or _ST["still_chip"] < FINAL_STEPS:
        return 0.0
    xy = np.asarray(chip_s.pos, float)[:2]
    if _ST["transferred"] and _on_socket_floor(chip_s):
        if _pen_home(pen_s) and _ST["rel_pen"] >= FINAL_STEPS and _ST["still_pen"] >= FINAL_STEPS:
            return 1.0
        return 0.60
    resting = float(chip_s.low[2]) <= max(Z_DST, Z_SRC) + 0.012
    if resting and (_in_mouth(xy) or _in_pocket(xy)):
        return 0.25
    return 0.0


def score(objects) -> float:
    """0 for the whole run if a pad touches the chip, the chip drops onto the table or rim, falls hard (under -0.55 m/s),
    is released more than 15 mm above a floor or away from the pack and nest, is dragged through a pocket wall, or the pen
    falls off the table. Otherwise, with the chip released (vacuum off), still and both grippers 30 mm clear for the last 2 s:
    1.0 suction lift and transfer completed, chip seated flat on the keyed nest floor (within 1 mm xy/z, 3 deg tilt) with
    pin-1 aligned (within 8 deg yaw), and the pen back on its cradle (within 15 mm, 10 deg of level) and still;
    0.60 suction lift and transfer completed and chip seated flat on the keyed nest floor with pin-1 aligned;
    0.25 the chip was lifted at least 10 mm above the pocket rim by suction and is resting in a pocket or in the nest;
    else 0."""
    _update(objects)
    if _ST["zero"]:
        return 0.0
    return float(_tier(objects))


def success(objects) -> bool:
    """Full score and both grippers 10 cm clear of the pen and the chip."""
    if score(objects) < 1.0:
        return False
    if FORCE_RELEASED[0] is not None or mechkit._ENV.get("env") is None:
        return True
    try:
        return bool(mechkit.released(objects["vac_pen"], clearance=0.10) and mechkit.released(objects["chip"], clearance=0.10))
    except Exception:
        return False


# ---------------------------------------------------------------------------------------------------------------------
# Suction. The vacuum is a scorer-side hold: while sealed, the chip is written onto the pen at its recorded offset before
# every physics step (240 Hz); it carries no collision with the pocket walls (the score() rule above judges a drag).


def _rows(asset, rows):
    import torch

    return torch.tensor(rows, dtype=torch.float32, device=asset.device)


def _weld_physics(environment) -> None:
    if not _ST.get("sealed") or _ST.get("rel_pos") is None:
        return
    base = environment.unwrapped
    scene = base.scene
    pen, chip = scene["vac_pen"], scene["chip"]
    pose = mechkit._to_numpy(pen.data.root_link_pose_w)[0]
    R = mechkit._quat_xyzw_matrix(pose[3:])
    pos = pose[:3] + R @ _ST["rel_pos"]
    q = ik.matrix_to_quat_xyzw(R @ _ST["rel_R"])
    write_pose = getattr(chip, "write_root_pose_to_sim_index", None) or chip.write_root_pose_to_sim
    write_vel = getattr(chip, "write_root_velocity_to_sim_index", None) or chip.write_root_velocity_to_sim
    write_pose(root_pose=_rows(chip, [[*[float(v) for v in pos], *[float(v) for v in q]]]))
    write_vel(root_velocity=_rows(chip, [[0.0] * 6]))


def _bind_cup_view(environment) -> None:
    try:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        mechkit._enable_contact_processing()
        stage = get_current_stage()
        prims = [p for p in stage.Traverse() if "/env_0/" in p.GetPath().pathString]
        rigid = [p.GetPath().pathString for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        pen_paths = mechkit._match_bodies(rigid, "vac_pen", ("base",))
        chip_paths = mechkit._match_bodies(rigid, "chip", ("base",))
        if not pen_paths or not chip_paths:
            raise RuntimeError(f"missing cup/chip rigid bodies: pen={pen_paths}, chip={chip_paths}")
        sv = mechkit._physics_sim_view(environment)
        view = sv.create_rigid_contact_view(pen_paths, filter_patterns=[chip_paths] * len(pen_paths),
                                            max_contact_data_count=64 * len(pen_paths) * len(chip_paths))
        _ST["cup_view"] = view
        _ST["cup_view_ok"] = True
        _ST["cup_view_failed"] = False
    except Exception as exc:  # noqa: BLE001
        _ST["cup_view"] = None
        _ST["cup_view_ok"] = False
        _ST["cup_view_failed"] = True
        print(f"cup-die contact view unavailable ({exc!r}); using geometric contact dwell", flush=True)


def _poll_cup_force(environment) -> None:
    view = _ST.get("cup_view")
    if view is None or _ST.get("sealed"):
        return
    try:
        fm = view.get_contact_force_matrix(dt=mechkit._physics_dt(environment))
        f = float(np.linalg.norm(mechkit._any_numpy(fm).reshape(-1, 3).sum(axis=0)))
        if math.isfinite(f):
            _ST["cup_force_step"] = max(float(_ST.get("cup_force_step", 0.0)), f)
    except Exception:  # noqa: BLE001 - first step before sim.step() may not have contact buffers yet
        pass


def _install_weld(environment) -> None:
    if environment is None:
        return
    scene = environment.unwrapped.scene
    if getattr(scene, "_vacuum_weld", False):
        return
    _bind_cup_view(environment)
    write = scene.write_data_to_sim

    def write_data_to_sim(*a, **k):
        _poll_cup_force(environment)
        try:
            _weld_physics(environment)
        except Exception as exc:  # noqa: BLE001 - never stop the run on a hold error
            if not _ST.get("weld_warned"):
                _ST["weld_warned"] = True
                print(f"vacuum hold failed: {exc!r}", flush=True)
        return write(*a, **k)

    scene.write_data_to_sim, scene._vacuum_weld = write_data_to_sim, True


@on_step
def _step_hook(env=None) -> None:
    """Read the live pen and chip once per control step and advance the latches, so a seal is made at 20 Hz while the
    physics hook above holds the chip."""
    if env is None:
        return
    try:
        _install_weld(env)
        _poll_cup_force(env)
        states = {"vac_pen": mechkit._mech_state(env, PEN), "chip": mechkit._mech_state(env, CHIP)}
    except Exception:
        return
    _update(states)


# ---------------------------------------------------------------------------------------------------------------------
# Scripted demo (closedloop). Right arm holds the pen; left arm squeezes its button. The demo reads the fixed layout
# (object_jitter=0), both arms' joint encoders and gripper openings, the live poses of vac_pen and chip, and the live
# travel of the pen's button joint.
demo = placeholder_demo

PHI = -math.radians(35.0)  # right grip: approach +y tipped 35 deg down, pads closing across the barrel
LTHETA = math.radians(20.0)  # left grip: approach -y tipped 20 deg toward the nose
U_GRIP = -0.022  # right pads on the barrel 22 mm behind the barrel centre
U_BTN = BTN_AT[0]
G_OPEN = 0.035  # pad gap while approaching the barrel
SPEED = 0.045  # m/s average (smoothstep peak 1.5x): under the declared 0.12 m/s


def ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def right_rot0() -> np.ndarray:
    """Right gripper's rotation with the pen level: approach (0, cos phi, sin phi), pads closing across the barrel."""
    z = np.array([0.0, math.cos(PHI), math.sin(PHI)])
    c = -np.cross(np.array([1.0, 0, 0]), z)
    return np.stack([c, np.cross(z, c), z], 1)


def left_rot0() -> np.ndarray:
    """Left gripper's rotation in the pen frame: pads closing along the button's axis (-z), approach from +y."""
    z = math.cos(LTHETA) * np.array([0.0, -1.0, 0.0]) + math.sin(LTHETA) * np.array([1.0, 0, 0])
    c = np.array([0.0, 0.0, -1.0])
    return np.stack([c, np.cross(z, c), z], 1)


def pen_origin_for_cup(face_xyz, pitch: float) -> np.ndarray:
    """Pen origin that puts the cup face centre at face_xyz with the barrel pitched nose down by ``pitch``."""
    return np.asarray(face_xyz, float) - ry(pitch) @ CUP_FACE


def right_tcp(P, pitch: float) -> np.ndarray:
    return np.asarray(P, float) + ry(pitch) @ np.array([U_GRIP, 0.0, 0.0])


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _pen_rot(pitch) -> np.ndarray:
    """The pen's rotation from a nose-down pitch (float) or a full 3x3 rotation."""
    return ry(pitch) if np.isscalar(pitch) else np.asarray(pitch, float)


def left_tcp(P, pitch, gap: float) -> np.ndarray:
    """Left tool point for a pad gap ``gap``: the pad on the barrel's underside stays 0.5 mm clear of it as the gap closes.
    ``pitch`` is the barrel's nose-down pitch or the pen's full rotation (pitch then a small yaw about vertical)."""
    return np.asarray(P, float) + _pen_rot(pitch) @ np.array([U_BTN, 0.0, -PEN_R - 0.0005 + gap / 2])


TRAVEL = TOP + 0.20
PITCH = math.radians(45.0)
FACE_DIE = np.array([WAFFLE_XY[0], WAFFLE_XY[1], DIE_TOP_START])  # cup face on the die
HOVER_UP = 0.025  # cup 25 mm above the die while the button is primed
FACE_SOCK = np.array([SOCKET_XY[0], SOCKET_XY[1], Z_DST + 0.004 + CHIP_H])  # chip 4 mm above the nest floor
PRESS_G = 0.0132  # commanded pad gap while pressing: 14 mm of barrel and pressed button, 0.8 mm tighter
PRESS_SPEED = 0.006  # m/s of pad closing


def _mv(arm, p, rot, finger=None, speed=SPEED, minimum=0.6):
    d = float(np.linalg.norm(np.asarray(p, float) - arm.pos))
    yield from arm.move(p, max(minimum, d / speed), finger, rot=rot)


def _btn_reading(ctx) -> float:
    """Button travel from the pen's joint reading (0 offline, so dry_run runs the squeeze to its deepest command)."""
    try:
        return button_depth(ctx.obj("vac_pen"))
    except Exception:
        return 0.0


def _left_squeeze(ctx, L, R, P_fn, pitch, lrot, want_pressed: bool, g_from: float, g_to: float, stop_at: float = 0.0031):
    """Close (or open) the left pads about the button, the lower pad held on the barrel's underside, with the right
    hand's measured pose giving the pen. Closing stops when the button reads ``stop_at`` (3.1 mm) or at g_to."""
    secs = abs(g_from - g_to) / (2 * PRESS_SPEED)
    n = max(2, round(secs / CONTROL_DT))
    g = g_from
    for i in range(1, n + 1):
        P = P_fn()
        g = g_from + (g_to - g_from) * i / n
        L._set_cartesian(left_tcp(P, pitch, g), lrot, -g / 2)
        yield
        if want_pressed and _btn_reading(ctx) >= stop_at:
            break
    return g


def _live_cup(ctx):
    """The pen's cup-face centre from its live pose (a simulator state read; None offline)."""
    try:
        return cup_face(ctx.obj("vac_pen"))
    except Exception:
        return None


def _live_chip_bottom_centre(ctx):
    try:
        c = ctx.obj("chip")
        o, _ = chip_frame(c)
        return np.array([o[0], o[1], float(c.low[2])])
    except Exception:
        return None


def _live_chip_attitude(ctx):
    """The chip's live attitude from its pose: (yaw in deg, 0 when its pin-1 chamfer faces the nest's pin-1 key; the x and
    y components of its top-face normal). None offline."""
    try:
        c = ctx.obj("chip")
        _, CR = chip_frame(c)
        return chip_yaw_deg(c), float(CR[0, 2]), float(CR[1, 2])
    except Exception:
        return None


def _rotvec(w) -> np.ndarray:
    """Rotation matrix of a rotation vector (rad)."""
    w = np.asarray(w, float)
    a = float(np.linalg.norm(w))
    if a < 1e-12:
        return np.eye(3)
    k = w / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


class _Place:
    """Right-hand placement servo over the nest. The commanded tool pose is the approach pose turned by ``D`` about
    ``pivot`` and shifted by ``corr``; each step walks ``corr`` toward the live chip centre target (xy and height) and
    turns ``D`` so the chip lies flat (top normal vertical) and square to the key (yaw 0). Both are capped (20 mm, 6 deg),
    so a bad reading cannot throw the arm."""

    def __init__(self, R, anchor, rot, pivot, target):
        self.R, self.anchor, self.rot0 = R, np.asarray(anchor, float), np.asarray(rot, float)
        self.pivot, self.target = np.asarray(pivot, float), np.asarray(target, float)
        self.corr, self.D, self.good = np.zeros(3), np.eye(3), 0

    def turn_deg(self) -> float:
        return float(math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(self.D) - 1) / 2)))))

    def rot(self) -> np.ndarray:
        return self.D @ self.rot0

    def pos(self) -> np.ndarray:
        return self.pivot + self.D @ (self.anchor - self.pivot) + self.corr

    def pen_origin(self, P_nominal) -> np.ndarray:
        """The pen origin the current command puts the pen at, from the nominal (unturned, uncorrected) origin."""
        return self.pivot + self.D @ (np.asarray(P_nominal, float) - self.pivot) + self.corr

    def tick(self, ctx, gain=0.12, ang_gain=0.30, tol=0.00025, yaw_tol=0.3, tilt_tol=0.5) -> None:
        cur, att = _live_chip_bottom_centre(ctx), _live_chip_attitude(ctx)
        if cur is not None and att is not None and not self.R.offline:
            yaw, nx, ny = att
            err = self.target - cur
            self.corr = np.clip(self.corr + gain * err, -0.02, 0.02)
            D = _rotvec(ang_gain * np.array([ny, -nx, -math.radians(yaw)])) @ self.D
            a = math.acos(max(-1.0, min(1.0, (np.trace(D) - 1) / 2)))
            if a <= math.radians(6.0):
                self.D = D
            ok = (float(np.hypot(err[0], err[1])) < tol and abs(err[2]) < 0.0005 and abs(yaw) < yaw_tol
                  and math.degrees(math.asin(min(1.0, math.hypot(nx, ny)))) < tilt_tol)
            self.good = self.good + 1 if ok else 0
        self.R._set_cartesian(self.pos(), self.rot(), self.R.finger)

    def settle(self, ctx, secs, need=6):
        self.good = 0
        for _ in range(max(1, round(secs / CONTROL_DT))):
            self.tick(ctx)
            yield
            if self.good >= need:
                break

    def follow(self, ctx, flag):
        """Keep servoing until ``flag[0]`` is set (the left hand's squeeze loads the pen by a few mm)."""
        while not flag[0]:
            self.tick(ctx)
            yield


def _servo(ctx, arms, anchors, rots, reading_fn, target, secs, gain=0.12, cap=0.02, tol=0.0008, use_z=False):
    """Walk every arm in ``arms`` together (same offset) until the live reading sits on ``target``: xy always, z when
    ``use_z``. The offset is capped at ``cap`` per axis, so a bad reading can never throw the arms. Returns the offset."""
    corr = np.zeros(3)
    n = max(1, round(secs / CONTROL_DT))
    ok_steps = 0
    for _ in range(n):
        cur = reading_fn(ctx)
        if cur is not None and not arms[0].offline:
            err = np.asarray(target, float) - cur
            if not use_z:
                err[2] = 0.0
            corr = np.clip(corr + gain * err, -cap, cap)
            ok_steps = ok_steps + 1 if float(np.linalg.norm(err)) < tol else 0
        for a, anc, rot in zip(arms, anchors, rots):
            a._set_cartesian(anc + corr, rot, a.finger)
        yield
        if ok_steps >= 10:
            break
    return corr


def _right_follow(ctx, R, rot, anchor, off, flag, target_xy, gain=0.10, cap=0.02):
    """Hold the right hand so the pen's cup stays over ``target_xy`` (live pose, xy) until ``flag[0]`` is set: the left
    hand's squeeze and release load the right arm and spring it back by a few mm, which this walks out. ``off`` is
    updated in place."""
    while not flag[0]:
        cur = _live_cup(ctx)
        if cur is not None and not R.offline:
            err = np.asarray(target_xy, float) - cur[:2]
            off[:2] = np.clip(off[:2] + gain * err, -cap, cap)
        R._set_cartesian(anchor + off, rot, R.finger)
        yield


def demo_script(ctx):
    L, R = ctx.left, ctx.right
    rrot0 = ry(0.0) @ right_rot0()
    lrot_for = lambda pitch: ry(pitch) @ left_rot0()  # noqa: E731
    approach0 = rrot0[:, 2]

    # 1) Right hand takes the pen from the cradle: in along its approach with the pads 35 mm apart, close on the barrel.
    tcp_c = right_tcp(PEN_REST, 0.0)
    pre = tcp_c - 0.09 * approach0
    yield from _mv(R, pre + np.array([0, 0, 0.04]), rrot0, -G_OPEN / 2, speed=0.05)
    yield from _mv(R, pre, rrot0, -G_OPEN / 2)
    yield from R.trim_to_measured(pre)
    yield from _mv(R, tcp_c, rrot0, speed=0.03)
    yield from R.trim_to_measured(tcp_c)
    yield from R.grip(FINGER_CLOSED, 0.8)
    yield from R.hold(0.3)
    if not R.holding(0.0075):
        yield from R.grip(-G_OPEN / 2, 0.5)
        yield from _mv(R, tcp_c + np.array([0, 0, 0.003]), rrot0, speed=0.02)
        yield from R.grip(FINGER_CLOSED, 0.8)
        yield from R.hold(0.3)
    yield from _mv(R, tcp_c + np.array([0, 0, 0.060]), rrot0, speed=0.03)

    # 2) Lift clear, roll the pen 45 degrees nose down and carry it over the die: cup 25 mm above the die.
    P_hover = pen_origin_for_cup(FACE_DIE + np.array([0, 0, HOVER_UP]), PITCH)
    rrot45 = ry(PITCH) @ right_rot0()
    P_mid = np.array([PEN_REST[0] + 0.02, PEN_REST[1] + 0.06, PEN_REST[2] + 0.12])
    yield from _mv(R, right_tcp(P_mid, 0.0), rrot0)
    yield from _mv(R, right_tcp(P_hover, PITCH), rrot45, speed=0.05)
    yield from R.trim_to_measured(right_tcp(P_hover, PITCH))
    # centre the cup over the die from the pen's live pose (xy only)
    anc = R.pos.copy()
    corr = yield from _servo(ctx, [R], [anc], [rrot45], _live_cup, FACE_DIE + np.array([0, 0, HOVER_UP]), 5.0)
    yield from ctx.wait(0.3)

    # 3) Left hand in from +y, pads 35 mm apart round the barrel at the button, then squeezes: button down 3.5 mm.
    def P_live():
        return np.asarray(R.measured_pos, float) - ry(PITCH) @ np.array([U_GRIP, 0.0, 0.0])

    lrot45 = lrot_for(PITCH)
    P_h = P_hover + corr
    l_away = left_tcp(P_h, PITCH, G_OPEN) + np.array([0.0, 0.12, 0.03])
    yield from _mv(L, l_away, lrot45, -G_OPEN / 2, speed=0.05)
    yield from _mv(L, left_tcp(P_h, PITCH, G_OPEN), lrot45, speed=0.03)
    yield from L.trim_to_measured(left_tcp(P_h, PITCH, G_OPEN))
    yield from _left_squeeze(ctx, L, R, P_live, PITCH, lrot45, True, G_OPEN, PRESS_G)
    yield from L.hold(0.4)

    # 4) Touchdown: both hands lower together at 1.2 cm/s, steering the cup onto the die centre, and press 2.0 mm past contact.
    drop = HOVER_UP + 0.0020
    n = round(drop / 0.012 / CONTROL_DT)
    base_r, base_l = R.pos.copy(), L.pos.copy()
    off = np.zeros(3)
    for i in range(1, n + 1):
        s_ = ik.smoothstep(i / n)
        cur = _live_cup(ctx)
        if cur is not None and not R.offline:
            err = FACE_DIE - cur
            off[:2] = np.clip(off[:2] + 0.10 * err[:2], -0.015, 0.015)
        dz = np.array([0.0, 0.0, -drop * s_])
        R._set_cartesian(base_r + dz + off, rrot45, R.finger)
        L._set_cartesian(base_l + dz + off, lrot45, L.finger)
        yield
    yield from ctx.wait(0.5)

    # 5) Left opens, the button rebounds and the vacuum is drawn; the left hand backs away to +y while the right hand
    #    keeps the cup over the die.
    anchor_r = R.pos - off
    flag = [False]

    def left_open():
        yield from _left_squeeze(ctx, L, R, P_live, PITCH, lrot45, False, PRESS_G, G_OPEN)
        yield from L.hold(0.3)
        yield from _mv(L, L.pos + np.array([0.0, 0.14, 0.03]), lrot45, speed=0.05)
        flag[0] = True

    yield from ctx.both(left_open(), _right_follow(ctx, R, rrot45, anchor_r, off, flag, FACE_DIE[:2]))

    # 6) Lift 28 mm (clear of the pocket walls), carry 110 mm along -y to the nest, lower to hover 4 mm above its floor.
    start = R.pos.copy()
    corr = np.zeros(3)
    nlift = round(2.8 / CONTROL_DT)
    for i in range(1, round(1.5 / CONTROL_DT) + nlift + 1):  # 1.5 s to re-centre the chip, then 28 mm up at about 1 cm/s
        k = max(0, i - round(1.5 / CONTROL_DT))
        cur = _live_chip_bottom_centre(ctx)
        if cur is not None and not R.offline:
            err = np.array([WAFFLE_XY[0], WAFFLE_XY[1], 0.0]) - cur
            err[2] = 0.0
            corr = np.clip(corr + 0.10 * err, -0.02, 0.02)
        R._set_cartesian(start + corr + np.array([0.0, 0.0, 0.028 * ik.smoothstep(k / nlift)]), rrot45, R.finger)
        yield
    P_over = pen_origin_for_cup(np.array([SOCKET_XY[0], SOCKET_XY[1], FACE_SOCK[2] + 0.030]), PITCH)
    yield from _mv(R, right_tcp(P_over, PITCH), rrot45, speed=0.05)
    P_sock = pen_origin_for_cup(FACE_SOCK, PITCH)
    yield from _mv(R, right_tcp(P_sock, PITCH), rrot45, speed=0.02)
    yield from R.trim_to_measured(right_tcp(P_sock, PITCH))
    # Square and centre the chip over the keyed nest from its live pose: turn the pen about the nest centre until the
    # chip lies flat (within 0.5 deg) and its pin-1 chamfer lines up with the key (yaw within 0.3 deg: the 19.5 mm pocket
    # admits the 19 mm chip only within about 1.5 deg), centre it within 0.25 mm and set its underside 4 mm above the floor.
    tgt = np.array([SOCKET_XY[0], SOCKET_XY[1], Z_DST + 0.004])
    place = _Place(R, R.pos.copy(), rrot45, tgt, tgt)
    yield from place.settle(ctx, 8.0)
    if not R.offline:
        print(f"PLACE servo: pen turned {place.turn_deg():.2f} deg, shifted {np.round(1000 * place.corr, 2)} mm, "
              f"settled {place.good >= 10}", flush=True)
    yield from ctx.wait(0.2)

    # 7) Left hand squeezes the button again: the vacuum breaks and the chip drops 4 mm into the nest. The squeeze stops
    #    at 1.8 mm of travel (still sealed) while the right hand walks out the load and re-squares the chip, then goes on
    #    to 3.1 mm, so the chip is centred and square at the moment it lets go.
    def rp():
        return place.D @ ry(PITCH)

    def P_live():
        return np.asarray(R.measured_pos, float) - rp() @ np.array([U_GRIP, 0.0, 0.0])

    P_s = place.pen_origin(P_sock)
    lrot_p = place.D @ lrot45
    l_away = left_tcp(P_s, rp(), G_OPEN) + np.array([0.0, 0.14, 0.03])
    yield from _mv(L, l_away, lrot_p, -G_OPEN / 2, speed=0.05)
    yield from _mv(L, left_tcp(P_s, rp(), G_OPEN), lrot_p, speed=0.03)
    yield from L.trim_to_measured(left_tcp(P_s, rp(), G_OPEN))
    flag = [False]
    g_now = [G_OPEN]

    def left_press():
        g_now[0] = yield from _left_squeeze(ctx, L, R, P_live, rp(), lrot_p, True, G_OPEN, PRESS_G, stop_at=0.0018)
        yield from L.hold(0.3)
        flag[0] = True

    yield from ctx.both(left_press(), place.follow(ctx, flag))
    yield from place.settle(ctx, 3.0)
    flag = [False]

    def left_break():
        g_from = min(g_now[0], G_OPEN)
        g_now[0] = yield from _left_squeeze(ctx, L, R, P_live, rp(), lrot_p, True, g_from, PRESS_G)
        yield from L.hold(0.6)
        flag[0] = True

    yield from ctx.both(left_break(), place.follow(ctx, flag))
    # lift the pen clear while the button is still held, so the cup cannot re-seal
    P_clear = place.pen_origin(P_sock) + np.array([0.0, 0.0, 0.030])
    rot_hold = place.rot()
    yield from ctx.both(
        _mv(L, L.pos + np.array([0.0, 0.0, 0.030]), lrot_p, speed=0.03),
        _mv(R, R.pos + np.array([0.0, 0.0, 0.030]), rot_hold, speed=0.03),
    )
    yield from _left_squeeze(ctx, L, R, P_live, rp(), lrot_p, False, g_now[0], G_OPEN)
    yield from _mv(L, L.pos + np.array([0.0, 0.14, 0.03]), lrot_p, speed=0.05)
    yield from L.home(7.0)

    # 8) Pen back: lift, return over the cradle, unroll to level and set it on the saddles.
    P_home_hi = np.array([PEN_REST[0], PEN_REST[1], PEN_REST[2] + 0.060])
    yield from _mv(R, right_tcp(P_clear + np.array([0, 0, 0.03]), PITCH), rrot45, speed=0.04)
    yield from _mv(R, right_tcp(P_mid, 0.0), rrot0, speed=0.05)
    yield from _mv(R, right_tcp(P_home_hi, 0.0), rrot0, speed=0.05)
    yield from _mv(R, right_tcp(np.array([PEN_REST[0], PEN_REST[1], PEN_REST[2] + 0.003]), 0.0), rrot0, speed=0.015)
    yield from R.grip(-G_OPEN / 2, 1.5)
    yield from R.hold(0.4)
    yield from _mv(R, tcp_c - 0.09 * approach0, rrot0, speed=0.04)
    yield from R.home(7.0)
    yield from ctx.wait(2.5)
