"""Close an inline skate cuff with its ratchet buckle: compress the sprung cuff ears, feed the notched ladder strap,
pump the ratchet lever at least two teeth under spring tension, fold flat, and release.

Real objects: an adult inline-skate cuff shell (260 g body + 120 g hinged overlap wing + 32 g ratchet lever + 42 g
notched ladder strap on a weighted boot plinth, resting unbolted on a dark rubber bench pad). The cuff shell is moulded
open (6 N / 32 deg opening spring at the front-left squeeze ear), so pushing one ear without pinching/counter-holding
slides the unbolted cuff across the pad. Squeezing the opposing front-left ears shut aligns the buckle mouth on the
right side; feeding the 42 g notched ladder strap past the entry catch and pumping the hinged ratchet lever >= 2 strokes
draws the cuff snugly latched (<= 1.5 deg open) against the opening spring.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import mechkit  # noqa: E402, F401  (must precede labkit so labkit wraps mechkit's import finder)
import labkit  # noqa: E402
from labkit import Joint, on_reset, on_step, part  # noqa: E402
import pitch_demo  # noqa: E402
from pitch_demo import Waypoint  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

# -- geometry (part frame: cuff axis at bench pad top; +X forward, +Y left) --------------------------------------------
XC, YC = 0.270, 0.000
MAT_H = 0.004
PAD_Z = TOP + MAT_H
R_SH = 0.054
R_W = 0.058
T_SH = 0.0045
Z0, Z1 = 0.012, 0.126
ZM = (Z0 + Z1) / 2
H_FORM = 0.114
WING_ARC = (-104.0, 78.0)  # front/right wrap wing from -Y buckle side across +X to +Y hinge
HINGE = (0.0, 0.056, ZM)  # wing hinge on +Y left side
BUCKLE_PHI = -90.0  # ratchet buckle receiver on -Y right side
B_R0, B_R1 = 0.060, 0.074
B_Z = (0.082, 0.118)
LEVER_AXIS = (0.0, -0.077, 0.098)  # lever hinge along buckle tangent (+angle lifts outward toward -Y)
LEVER_TOP = 0.160
KNOB = (0.024, 0.018, 0.020)  # 24 x 18 x 20 mm lever grip handle
STRAP_PHI, STRAP_R = -135.0, 0.062  # strap rivet on -Y rear shell
STRAP_Z = (0.097, 0.117)
STRAP_L = 0.068
TAB = (0.016, 0.016, 0.044)  # 16 mm thick strap pinch tab (z = 0.112..0.156)
TAB_AT = 0.016  # 16 mm from rivet -> keeps gripper >= 38 mm clear of buckle/lever at seat
WTAB_POS = (0.038, 0.048, 0.136)  # moving squeeze ear on wing (z = 0.116..0.156)
ANCHOR_POS = (0.038, 0.031, 0.136)  # fixed shell anchor ear on cuff (+38, +31 mm; 17 mm center spacing when shut)
EAR_SIZE = (0.016, 0.009, 0.040)


def _pol(phi_deg: float, r: float, z: float = 0.0) -> tuple[float, float, float]:
    a = math.radians(phi_deg)
    return (r * math.cos(a), r * math.sin(a), z)


RIVET = np.array(_pol(STRAP_PHI, STRAP_R))
_BM = np.array(_pol(BUCKLE_PHI, (B_R0 + B_R1) / 2))
STRAP_DIR = (_BM - RIVET) / np.linalg.norm(_BM - RIVET)
STRAP_YAW = math.atan2(STRAP_DIR[1], STRAP_DIR[0])

# -- mechanics + scoring thresholds (every scoring threshold appears literally here) ----------------------------------
OPEN_TARGET = math.radians(32.0)  # +32 deg open spring target (~6 N at front squeeze ear)
WING_MAX = math.radians(38.0)
K_WING = 0.55  # N*m/rad opening spring (~0.31 N*m -> ~6 N at ear)
STRAP_START = -math.radians(70.0)  # swung -70 deg outward toward -Y
STRAP_IN = math.radians(3.0)  # <= 3 deg into buckle mouth past entry catch
STRAP_OUT = math.radians(7.0)  # pulled >= 7 deg out to re-arm
ALIGN = math.radians(3.5)  # wing within 3.5 deg of shut for channel entry
LEVER_UP = math.radians(28.0)  # pumped >= 28 deg outward
LEVER_DOWN = math.radians(8.0)  # folded flat <= 8 deg
LEVER_MAX = math.radians(52.0)
B_ARM = 0.124
SLACK = 0.014  # 14 mm slack at first notch (~6.5 deg at 0 pumps, ~3.7 deg at 1 pump, <= 0.92 deg at >= 2 pumps)
PITCH = 0.006  # 6 mm notch pitch per ratchet stroke
NOTCHES_SNUG = 2  # >= 2 ratchet teeth pulled
MAX_NOTCHES = 4
K_LOCK = 25.0  # N*m/rad pawl retention stiffness against opening spring
K_STRAP_HOLD = 2.5  # N*m/rad engaged strap retention in buckle channel
PARTIAL_OPEN = math.radians(8.0)  # <= 8 deg held closed for partial (engaged, < 2 notches) tier
SNUG = math.radians(1.5)  # cuff closed within 1.5 deg
CUFF_DISP_TOL = 0.025  # 25 mm max displacement of unbolted cuff on bench pad
CUFF_TILT_DEG = 8.0  # 8 deg max cuff base tilt
CLEAR = 0.05  # >= 5 cm (50 mm) gripper clearance from cuff envelope
HOLD_S = 3.0  # 3.0 s hands-off retention hold
HOLD_STEPS = 60  # 3 s hands-off retention hold at 20 Hz
CONTROL_HZ = 20
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
S_FULL, S_CAUGHT = 1.0, 0.5

# -- distinct PBR colors ----------------------------------------------------------------------------------------------
MAT_DARK = (0.18, 0.20, 0.23)
MAT_TRIM = (0.31, 0.35, 0.40)
SHELL_BLUE = (0.11, 0.24, 0.54)
SHELL_ACCENT = (0.20, 0.40, 0.78)
SHELL_DARK = (0.14, 0.15, 0.18)
BOOT_SOLE = (0.24, 0.25, 0.28)
FOAM_LINER = (0.82, 0.77, 0.68)
PAD_GREY = (0.56, 0.59, 0.63)
STEEL = (0.78, 0.80, 0.83)
STRAP_RED = (0.82, 0.15, 0.12)
PAWL_ORANGE = (0.88, 0.42, 0.10)


def _arc(phis, r, color, z=(Z0, Z1)):
    out = []
    for phi in phis:
        c = _pol(phi, r, (z[0] + z[1]) / 2)
        out.append((c, (T_SH, 2 * r * math.sin(math.radians(11.0)), z[1] - z[0]), color, math.radians(phi)))
    return out


def hold_angle(notches: int) -> float:
    return max(0.0, SLACK - PITCH * notches) / B_ARM


_wing_phis = [WING_ARC[0] + 20.0 * k for k in range(9)] + [WING_ARC[1] - 10.0]
WING = Joint(
    "wing", kind="hinge", axis="Z", anchor=HINGE, lower=0.0, upper=WING_MAX, stiffness=K_WING,
    damping=0.02, target=OPEN_TARGET, initial=OPEN_TARGET, mass=0.120,
    boxes=(
        *_arc(_wing_phis, R_W, SHELL_BLUE),
        *_arc(_wing_phis[::2], R_W + 0.002, SHELL_ACCENT, z=(Z1 - 0.010, Z1 + 0.002)),
        (WTAB_POS, EAR_SIZE, SHELL_DARK, math.radians(45.0)),
        (_pol(BUCKLE_PHI, (B_R0 + B_R1) / 2, sum(B_Z) / 2), (0.030, B_R1 - B_R0, B_Z[1] - B_Z[0]), STEEL),
        (_pol(BUCKLE_PHI, B_R1 + 0.002, B_Z[1] - 0.004), (0.032, 0.006, 0.008), PAWL_ORANGE),  # entry catch shroud
    ),
)

_lz = (LEVER_AXIS[2] + LEVER_TOP - KNOB[2]) / 2
LEVER = Joint(
    "lever", kind="hinge", axis="X", parent="wing", anchor=LEVER_AXIS, lower=0.0, upper=LEVER_MAX,
    stiffness=0.04, damping=0.004, target=0.0, initial=0.0, mass=0.032,
    boxes=(
        ((0.0, LEVER_AXIS[1], _lz), (0.022, 0.0045, LEVER_TOP - KNOB[2] - LEVER_AXIS[2]), STEEL),
        ((0.0, LEVER_AXIS[1] - 0.003, LEVER_TOP - KNOB[2] / 2), KNOB, SHELL_DARK),
    ),
)


def _along(d0, d1, z, thick, color):
    c = RIVET + STRAP_DIR * (d0 + d1) / 2
    return ((float(c[0]), float(c[1]), (z[0] + z[1]) / 2), (d1 - d0, thick, z[1] - z[0]), color, STRAP_YAW)


_tab_c = RIVET + STRAP_DIR * TAB_AT
STRAP = Joint(
    "strap", kind="hinge", axis="Z", anchor=(float(RIVET[0]), float(RIVET[1]), sum(STRAP_Z) / 2),
    lower=-math.radians(82.0), upper=0.0, stiffness=0.0, damping=0.004, friction=0.002, target=0.0,
    initial=STRAP_START, mass=0.042,
    boxes=(
        _along(-0.006, STRAP_L, STRAP_Z, 0.0035, STRAP_RED),
        *[_along(d, d + 0.0022, (STRAP_Z[0] + 0.003, STRAP_Z[1] - 0.003), 0.005, SHELL_DARK)
          for d in (0.042, 0.048, 0.054, 0.060)],
        ((float(_tab_c[0]), float(_tab_c[1]), 0.112 + TAB[2] / 2), TAB, STRAP_RED, STRAP_YAW),
    ),
)

BENCH_PAD = part(
    "bench_pad", (XC, YC, TOP), fixed=True, friction=0.90, color=MAT_DARK,
    boxes=(
        ((0.0, 0.0, MAT_H / 2), (0.220, 0.240, MAT_H), MAT_DARK),
        ((0.0, 0.115, MAT_H + 0.0015), (0.210, 0.008, 0.003), MAT_TRIM),
        ((0.0, -0.115, MAT_H + 0.0015), (0.210, 0.008, 0.003), MAT_TRIM),
    ),
)

CUFF = part(
    "cuff", (XC, YC, PAD_Z + 0.0005), mass=0.260, friction=0.90, color=SHELL_BLUE,
    joints=(WING, LEVER, STRAP),
    boxes=(
        ((0.0, 0.0, 0.004), (0.136, 0.136, 0.008), BOOT_SOLE),
        ((-0.025, 0.0, 0.013), (0.068, 0.090, 0.010), SHELL_DARK),
        *_arc([85.0 + 20.0 * k for k in range(9)], R_SH, SHELL_BLUE),
        ((-0.058, 0.0, 0.069), (0.006, 0.026, 0.104), SHELL_ACCENT),
        (ANCHOR_POS, EAR_SIZE, SHELL_DARK, math.radians(45.0)),
        ((float(RIVET[0]), float(RIVET[1]), sum(STRAP_Z) / 2), (0.010, 0.010, 0.010), STEEL),
    ),
    cylinders=(
        ((0.0, 0.0, 0.008 + (H_FORM - 0.008) / 2), 0.047, H_FORM - 0.008, FOAM_LINER, "Z"),
        ((0.0, 0.0, 0.117), 0.049, 0.014, PAD_GREY, "Z"),
    ),
)
OBJECTS = [BENCH_PAD, CUFF]

# -- trial watch (ratchet engagement, strokes, cuff displacement & hands-off hold) --------------------------------------
_EP: dict = {}
_IDS: dict = {}


def _fresh() -> dict:
    return {"engaged": False, "armed": True, "notches": 0, "up": False,
            "clear_steps": 0, "open_while_clear": 0.0, "step": 0, "tcps": None}


_EP.update(_fresh())


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    return x.detach().cpu().numpy().astype(np.float64) if hasattr(x, "detach") else np.asarray(x, dtype=np.float64)


def _tensor(a, like):
    try:
        import torch
        return torch.tensor(np.asarray(a), dtype=torch.float32, device=like.device)
    except ImportError:
        return np.asarray(a)


def _arm_tcps(env):
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS
    out = []
    for side in ("left", "right"):
        rob = env.unwrapped.scene[f"{side}_robot"]
        if side not in _IDS:
            _IDS[side] = rob.find_joints(ARM_JOINTS, preserve_order=True)[0]
        p, _ = ik.tcp_pose(_np(rob.data.joint_pos)[0][_IDS[side]], station.BASES[side])
        out.append(np.asarray(p, float))
    return out


def cuff_seated(cuff_state) -> bool:
    if cuff_state is None:
        return False
    p = np.asarray(cuff_state.pos, float)
    disp = math.hypot(p[0] - XC, p[1] - YC)
    x, y, z, w = (float(v) for v in cuff_state.quat_xyzw)
    r22 = 1.0 - 2.0 * (x * x + y * y)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, r22))))
    return disp <= CUFF_DISP_TOL and tilt <= CUFF_TILT_DEG


def cuff_gap(p) -> float:
    p = np.asarray(p, float)
    dr = max(0.0, math.hypot(p[0] - XC, p[1] - YC) - 0.080)
    dz = max(0.0, p[2] - (PAD_Z + 0.156))
    return math.hypot(dr, dz)


def hands_clear(tcps) -> bool:
    return tcps is not None and len(tcps) == 2 and all(cuff_gap(t) >= CLEAR for t in tcps)


def watch(q: dict, tcps, cuff_state=None) -> None:
    """Advance one control step of ratchet engagement, stroke counting, and hands-off hold."""
    e = _EP
    wing, strap, lever = q.get("wing", OPEN_TARGET), q.get("strap", STRAP_START), q.get("lever", 0.0)
    e["step"] += 1
    if e["engaged"] and strap < -STRAP_OUT:
        e["engaged"] = False
        e["notches"] = 0
        e["armed"] = True
    if not e["engaged"]:
        if strap < -STRAP_OUT:
            e["armed"] = True
        elif strap >= -STRAP_IN and e["armed"] and abs(wing) <= ALIGN and cuff_seated(cuff_state):
            e["armed"] = False
            e["engaged"] = True
    if lever >= LEVER_UP:
        e["up"] = True
    elif lever <= LEVER_DOWN and e["up"]:
        e["up"] = False
        if e["engaged"]:
            e["notches"] = min(MAX_NOTCHES, e["notches"] + 1)
    if hands_clear(tcps):
        e["open_while_clear"] = max(e["open_while_clear"], wing) if e["clear_steps"] else wing
        e["clear_steps"] += 1
    else:
        e["clear_steps"], e["open_while_clear"] = 0, 0.0
    e["tcps"] = tcps


def _drives(cu, q) -> None:
    if not (hasattr(cu, "joint_names") and hasattr(cu, "write_joint_stiffness_to_sim")
            and hasattr(cu, "set_joint_position_target")):
        return
    names = list(cu.joint_names)
    k = {j.joint_name: j.stiffness for j in CUFF.joints}
    tgt = {j.joint_name: j.target for j in CUFF.joints}
    if _EP["engaged"]:
        h = hold_angle(_EP["notches"])
        tgt[WING.joint_name] = h
        k[WING.joint_name] = K_LOCK if q["wing"] > h else K_WING
        tgt[STRAP.joint_name], k[STRAP.joint_name] = 0.0, K_STRAP_HOLD
    cu.write_joint_stiffness_to_sim(_tensor([[k[n] for n in names]], cu))
    cu.set_joint_position_target(_tensor([[tgt[n] for n in names]], cu))


@on_reset
def _reset(_env) -> None:
    _EP.update(_fresh())


@on_step
def _rules(env) -> None:
    try:
        tcps = _arm_tcps(env)
    except Exception:
        tcps = None
    cu = env.unwrapped.scene["cuff"]
    st = labkit._part_state(env, CUFF)
    q = st.joints
    watch(q, tcps, st)
    _drives(cu, q)
    if _EP["step"] % 20 == 0:
        print("STATE " + json.dumps({
            "step": _EP["step"], **{n: round(math.degrees(v), 1) for n, v in q.items()},
            "engaged": _EP["engaged"], "notches": _EP["notches"],
            "clear_steps": _EP["clear_steps"], "seated": cuff_seated(st),
        }), flush=True)


# -- scoring ----------------------------------------------------------------------------------------------------------
def _joints(objects) -> dict:
    j = getattr(objects.get("cuff"), "joints", None)
    if j:
        return j
    env = labkit.env()
    return labkit._part_state(env, CUFF).joints if env is not None else {}


def components(objects) -> dict:
    cuff_st = objects.get("cuff")
    q = _joints(objects)
    tcps = _EP.get("tcps")
    strap_q = float(q.get("strap", STRAP_START))
    return {
        "seated": cuff_seated(cuff_st),
        "strap_in": strap_q >= -STRAP_OUT,
        "lever_flat": float(q.get("lever", 0.0)) <= LEVER_DOWN,
        "wing_open": max(float(q.get("wing", OPEN_TARGET)), float(_EP["open_while_clear"])),
        "clear": hands_clear(tcps),
        "hold_steps": int(_EP["clear_steps"]),
    }


def score(objects) -> float:
    """Judged at the end of the trial using observable end-state geometry and 3 s hold."""
    k = components(objects)
    if not (k["seated"] and k["strap_in"] and k["wing_open"] <= PARTIAL_OPEN and k["clear"]
            and k["hold_steps"] >= HOLD_STEPS):
        return 0.0
    if k["lever_flat"] and k["wing_open"] <= SNUG:
        return S_FULL
    return S_CAUGHT


def success(objects) -> bool:
    return score(objects) >= S_FULL - 1e-9


def distance(objects) -> float:
    k = components(objects)
    q = _joints(objects)
    strap_gap = max(0.0, -STRAP_OUT - float(q.get("strap", STRAP_START)))
    return float(max(0.0, k["wing_open"]) + strap_gap + max(0.0, float(q.get("lever", 0.0))))


# -- scripted demo (fixed scene constants only) -----------------------------------------------------------------------
def _world(p) -> np.ndarray:
    return np.array([XC + p[0], YC + p[1], PAD_Z + p[2]])


def _rotz(p, about, a: float):
    c, s = math.cos(a), math.sin(a)
    d = np.asarray(p[:2]) - np.asarray(about[:2])
    return (about[0] + c * d[0] - s * d[1], about[1] + s * d[0] + c * d[1])


def _yaw_closing(psi: float, near: float = 0.0) -> float:
    y = psi + math.pi / 2
    while y > near + math.pi / 2:
        y -= math.pi
    while y < near - math.pi / 2:
        y += math.pi
    return y


def wing_tab(a: float):
    xy = _rotz(WTAB_POS, HINGE, a)
    return xy, math.radians(45.0) + a


def strap_tab(a: float):
    xy = _rotz(_tab_c, RIVET, a)
    return xy, STRAP_YAW + a


def knob(lev: float):
    """Lever knob centre (part frame y, z) at outward lift angle lev >= 0."""
    dy0 = (LEVER_AXIS[1] - 0.003) - LEVER_AXIS[1]
    dz0 = (LEVER_TOP - KNOB[2] / 2) - LEVER_AXIS[2]
    c, s = math.cos(lev), math.sin(lev)
    return LEVER_AXIS[1] + c * dy0 - s * dz0, LEVER_AXIS[2] + s * dy0 + c * dz0


PAD_Z_TAB = 0.141
OVER = math.radians(3.0)
PUMP = math.radians(42.0)
LEAN_RAD = math.radians(15.0)  # outward wrist splay keeps upper forearms >= 50 mm apart


def _splay(yaw: float, side_sign: float):
    """Apply outward forearm splay (+1 left/+Y, -1 right/-Y) about world X."""
    return axis_angle_matrix([1.0, 0.0, 0.0], -side_sign * LEAN_RAD) @ pitch_demo.rotation(yaw)


def demo():
    """Left hand pinches sprung cuff ears shut on bench pad; right hand feeds ladder strap into buckle, pumps lever 3x,
    folds lever flat; both release >= 50 mm clear for >= 3 s."""
    W, o, c = Waypoint, FINGER_OPEN, FINGER_CLOSED
    T = PAD_TIP_PAST_TCP

    def at(xy, z, dz=0.0):
        return tuple(_world((xy[0], xy[1], z + T + dz)))

    park_l, park_r = (0.16, 0.32, TOP + 0.18), (0.16, -0.32, TOP + 0.18)
    # Left hand pinches across fixed ANCHOR_POS (y=+0.031) and moving WTAB_POS (y=+0.048 when shut, ~+0.068 open)
    pinch_x = ANCHOR_POS[0]
    pinch_y_open = 0.5 * (ANCHOR_POS[1] + wing_tab(OPEN_TARGET)[0][1])
    pinch_y_shut = 0.5 * (ANCHOR_POS[1] + WTAB_POS[1])
    rl = _splay(0.0, 1.0)
    left = [
        W(1.5, at((pinch_x, pinch_y_open), PAD_Z_TAB, 0.018), o, "over cuff ears", rot=rl),
        W(0.9, at((pinch_x, pinch_y_open), PAD_Z_TAB), o, "straddle both ears", rot=rl),
        W(1.1, at((pinch_x, pinch_y_shut), PAD_Z_TAB), c, "pinch sprung ears shut", rot=rl),
    ]
    t_shut = sum(w.duration for w in left)

    # Right hand picks 42 g notched ladder strap on -Y, swings into buckle mouth, then pumps ratchet lever 3 strokes
    xy0, psi0 = strap_tab(STRAP_START)
    ys = _yaw_closing(psi0, 0.0)
    right = [
        W(1.5, at(xy0, PAD_Z_TAB, 0.018), o, "over strap tab", rot=_splay(ys, -1.0)),
        W(0.9, at(xy0, PAD_Z_TAB), o, "round strap tab", rot=_splay(ys, -1.0)),
        W(1.0, at(xy0, PAD_Z_TAB), c, "pinch strap tab", rot=_splay(ys, -1.0)),
        W(max(0.5, t_shut + 0.4 - 3.4), at(xy0, PAD_Z_TAB), c, "wait for cuff shut", rot=_splay(ys, -1.0)),
    ]
    for k in range(1, 7):
        a = STRAP_START + (OVER - STRAP_START) * k / 6
        xy, psi = strap_tab(a)
        ys = _yaw_closing(psi, ys)
        right.append(W(0.5, at(xy, PAD_Z_TAB), c, "feed strap into buckle", rot=_splay(ys, -1.0)))
    right += [
        W(0.5, at(xy, PAD_Z_TAB), c, "seat into entry catch", rot=_splay(ys, -1.0)),
        W(0.8, at(xy, PAD_Z_TAB), o, "release strap tab", rot=_splay(ys, -1.0)),
        W(0.7, at(xy, PAD_Z_TAB, 0.018), o, "lift clear of strap", rot=_splay(ys, -1.0)),
    ]
    ky = _yaw_closing(0.0, 1.0)  # fingers close along X across 24 mm lever knob
    r_knob = pitch_demo.rotation(ky)
    y0, z0 = knob(0.0)
    knob_p0 = at((0.0, y0), z0 - KNOB[2] / 2 + 0.002)
    hinge_w = _world(LEVER_AXIS)
    right += [
        W(0.9, at((0.0, y0), z0 - KNOB[2] / 2 + 0.002, 0.018), o, "over lever knob", yaw=ky),
        W(0.8, knob_p0, o, "straddle lever knob", yaw=ky),
        W(1.0, knob_p0, c, "pinch lever knob", yaw=ky),
    ]
    for _ in range(3):
        right += pitch_demo.hinge_turn(knob_p0, r_knob, hinge_w, (1.0, 0.0, 0.0), PUMP,
                                       steps=3, duration=1.1, finger=c, label="pump lever out")
        p_out, r_out = np.asarray(right[-1].pos, float), right[-1].yaw.rotation
        right += pitch_demo.hinge_turn(p_out, r_out, hinge_w, (1.0, 0.0, 0.0), -PUMP - 0.03,
                                       steps=3, duration=1.1, finger=c, label="fold lever flat")
    right += [W(0.7, knob_p0, o, "release lever", yaw=ky), W(0.7, at((0.0, y0), z0, 0.018), o, "lift clear", yaw=ky),
              W(1.2, park_r, o, "park clear", yaw=0.0)]
    t_done = sum(w.duration for w in right) - 1.9
    left += [W(max(0.5, t_done - sum(w.duration for w in left)), at((pinch_x, pinch_y_shut), PAD_Z_TAB), c,
               "hold cuff shut", rot=rl),
             W(0.7, at((pinch_x, pinch_y_shut), PAD_Z_TAB), o, "release ears", rot=rl),
             W(0.7, at((pinch_x, pinch_y_shut), PAD_Z_TAB, 0.018), o, "lift clear", rot=rl),
             W(1.2, park_l, o, "park clear", yaw=0.0)]
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    end = max(tl, tr) + HOLD_STEPS / CONTROL_HZ + SUCCESS_HOLD_S + 1.5
    left.append(W(end - tl, park_l, o, "hold clear")); right.append(W(end - tr, park_r, o, "hold clear"))
    return {"left": left, "right": right}
