"""Center a dual-handle stainless apple wedger over a seated apple and press it level all the way through to the cutting board.

Real product class: a dual-handle stainless apple corer and divider (160 mm handle-to-handle grip span, 108 mm outer
ring diameter, 23 mm OD / 18.4 mm ID hollow central coring tube, 8 radial divider blades, dual wing flanges with raised
cushioned grips, 0.210 kg) paired with a 0.180 kg medium apple specimen (72 mm diameter, 58 mm tall, 18 N cut-through
resistance) seated in the 6 mm recessed cradle of a 170 x 200 x 18 mm wooden cutting board.

Bimanual physics: pressing downward on only one side handle (69 mm lever arm from the core axis) against the 18 N central
cutting resistance generates a ~1.24 N*m overturning roll moment, cocking the wedger > 5 deg so its 40 mm-tall coring
tube and 8 radial divider blades bind against the apple segments before cut-through. Grasping both side handles
simultaneously balances the roll moment (< 5 deg tilt) and drives the blade ring cleanly down onto the cutting board deck.
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
import pitch_demo  # noqa: E402
from labkit import Joint, on_reset, on_step, part  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

# -- distinct PBR colours ---------------------------------------------------------------------------------------------
BOARD_WOOD = (0.76, 0.59, 0.40)
BOARD_TRIM = (0.48, 0.33, 0.21)
RUBBER_PAD = (0.14, 0.15, 0.17)
MAT_DARK = (0.19, 0.21, 0.24)
MAT_GUIDE = (0.31, 0.35, 0.40)
APPLE_SKIN = (0.78, 0.13, 0.14)
APPLE_FLESH = (0.96, 0.91, 0.76)
STEM_BROWN = (0.36, 0.22, 0.12)
LEAF_GREEN = (0.24, 0.56, 0.22)
STEEL = (0.76, 0.79, 0.82)
HANDLE_BLK = (0.13, 0.14, 0.16)
GRIP_ELAST = (0.24, 0.27, 0.30)

# -- 1:1 manufacturer dimensions (metres, kg, N) ----------------------------------------------------------------------
BOARD_X, BOARD_Y = 0.345, 0.000
BOARD_L, BOARD_W, BOARD_H = 0.170, 0.200, 0.018  # 170 x 200 x 18 mm wooden cutting board
WELL_DEPTH = 0.006  # 6 mm recessed cradle pocket
WELL_HALF = 0.0385  # 77 x 77 mm recessed cradle opening (2.5 mm radial clearance around 72 mm apple)
WELL_Z = TOP + (BOARD_H - WELL_DEPTH)
DECK_Z = TOP + BOARD_H

START_X, START_Y = 0.200, 0.000  # countertop staging mat (145 mm behind the cutting board centre)
MAT_H = 0.003

FRUIT_MASS_BODY = 0.090  # 90 g segmented body + 90 g core resistance anvil = 180 g specimen (all links >= 25 g)
FRUIT_MASS_CORE = 0.090
FRUIT_OD = 0.072  # 72 mm diameter medium apple
FRUIT_H = 0.058  # 58 mm total height
CUT_STROKE = 0.048  # 48 mm downward cut-through stroke to flush with cutting board deck
CUT_FORCE_N = 18.0  # 18 N inelastic flesh/core cutting resistance
CUT_YIELD_M = 0.0045  # 4.5 mm elastic deflection at 18 N yield -> stiffness 4000 N/m, zero sag at rest
CUT_K = CUT_FORCE_N / CUT_YIELD_M

WEDGER_MASS = 0.210  # 210 g stainless steel + ergonomic dual-wing apple corer/wedger
WEDGER_SPAN = 0.160  # 160 mm handle-to-handle grip span (+-80 mm in Y)
RING_OD, RING_ID = 0.108, 0.098  # 108 mm outer ring OD, 98 mm ID
RING_H = 0.036  # 36 mm stainless blade/ring height (upper collar tops out at 42 mm)
CORER_R_OUT = 0.0115  # 23.0 mm OD hollow stainless coring tube (4.7 mm radial clearance; binds if cocked > 5 deg)
CORER_R_IN = 0.0092
BLADE_T = 0.0018  # 1.8 mm radial stainless blade thickness (8 spokes at 45 deg intervals)
HANDLE_Y = 0.069  # side grip spine centres at +-69 mm (wing spans +-84 mm)
HANDLE_Z = 0.057  # raised 20 mm-tall grip boss centre above horizontal wing shelf (top at z = 0.047 m)
HANDLE_W_Y = 0.020  # 20 mm pinch thickness along Y


def _build_board():
    """170 x 200 x 18 mm wooden cutting board with 6 mm recessed walnut-lined cradle and rubber feet."""
    foot_h = 0.002
    base_h = BOARD_H - WELL_DEPTH - foot_h  # 10 mm wood slab above 2 mm feet
    base_zc = foot_h + base_h / 2
    zf = BOARD_H - WELL_DEPTH / 2
    sx = BOARD_L / 2 - WELL_HALF
    sy = BOARD_W / 2 - WELL_HALF
    cx, cy = WELL_HALF + sx / 2, WELL_HALF + sy / 2
    liner_t = 0.004
    boxes = [
        ((0.0, 0.0, base_zc), (BOARD_L, BOARD_W, base_h), BOARD_WOOD),
        ((-cx, 0.0, zf), (sx, BOARD_W, WELL_DEPTH), BOARD_WOOD),
        ((cx, 0.0, zf), (sx, BOARD_W, WELL_DEPTH), BOARD_WOOD),
        ((0.0, cy, zf), (2 * WELL_HALF, sy, WELL_DEPTH), BOARD_WOOD),
        ((0.0, -cy, zf), (2 * WELL_HALF, sy, WELL_DEPTH), BOARD_WOOD),
        # Dark walnut inner cradle trim rails (flush with deck, framing 77 x 77 mm well)
        ((-(WELL_HALF + liner_t / 2), 0.0, zf + 0.0001), (liner_t, 2 * WELL_HALF + 2 * liner_t, WELL_DEPTH), BOARD_TRIM),
        (((WELL_HALF + liner_t / 2), 0.0, zf + 0.0001), (liner_t, 2 * WELL_HALF + 2 * liner_t, WELL_DEPTH), BOARD_TRIM),
        ((0.0, WELL_HALF + liner_t / 2, zf + 0.0001), (2 * WELL_HALF, liner_t, WELL_DEPTH), BOARD_TRIM),
        ((0.0, -(WELL_HALF + liner_t / 2), zf + 0.0001), (2 * WELL_HALF, liner_t, WELL_DEPTH), BOARD_TRIM),
        # Rear walnut grip accent inlay well clear of wedger outer ring
        ((-0.074, 0.0, BOARD_H - 0.001), (0.012, 0.110, 0.002), BOARD_TRIM),
    ]
    for fx in (-0.068, 0.068):
        for fy in (-0.082, 0.082):
            boxes.append(((fx, fy, foot_h / 2), (0.022, 0.022, foot_h), RUBBER_PAD))
    return part("board", (BOARD_X, BOARD_Y, TOP), fixed=True, friction=0.85, color=BOARD_WOOD, boxes=boxes)


def _build_fruit():
    """180 g 8-wedge segmented apple with cream flesh interior, red skin, stem/leaf crown, and 18 N core slide."""
    boxes = []
    for k in range(8):
        th = (k + 0.5) * math.pi / 4.0  # 22.5 deg midpoint between radial blades
        c, s = math.cos(th), math.sin(th)
        # Inner cream flesh wedge (r = 16.0..29.0 mm; >= 2.8 mm blade clearance & >= 4.7 mm corer clearance)
        boxes.append(((0.0225 * c, 0.0225 * s, 0.0270), (0.0130, 0.0052, 0.0435), APPLE_FLESH, th))
        # Outer curved red apple skin cheek (r = 25.5..36.0 mm -> 72 mm OD, clears 77 mm cradle cleanly)
        boxes.append(((0.03075 * c, 0.03075 * s, 0.0280), (0.0105, 0.0110, 0.0450), APPLE_SKIN, th))
        # Rounded upper crown shoulder bevel (tops out at 52.0 mm, flush with core anvil for clean blade entry)
        boxes.append(((0.0265 * c, 0.0265 * s, 0.0495), (0.0100, 0.0080, 0.0050), APPLE_SKIN, th))
    cyls = [
        ((0.0, 0.0, 0.0026), 0.0335, 0.0052, APPLE_SKIN, "Z"),  # base seated in 77 mm cradle
    ]
    core_top = WELL_DEPTH + CUT_STROKE - 0.0005  # 0.0535 m: clears well floor by 1 mm at -48 mm
    core_h = 0.0045
    cut_joint = Joint(
        "cut", kind="slide", axis="Z", anchor=(0.0, 0.0, core_top - core_h / 2),
        lower=-CUT_STROKE, upper=0.0, initial=0.0, target=0.0,
        stiffness=CUT_K, damping=22.0, friction=0.0, mass=FRUIT_MASS_CORE,
        cylinders=(
            ((0.0, 0.0, core_top - core_h / 2), 0.0150, core_h, APPLE_FLESH, "Z"),
            ((0.0, 0.0, core_top + 0.0025), 0.0020, 0.0050, STEM_BROWN, "Z"),
        ),
        boxes=(((0.0028, 0.0, core_top + 0.0038), (0.0065, 0.0032, 0.0018), LEAF_GREEN, 0.35),),
    )
    return part("fruit", (BOARD_X, BOARD_Y, WELL_Z + 0.0004), mass=FRUIT_MASS_BODY, friction=0.90,
                color=APPLE_SKIN, boxes=boxes, cylinders=cyls, joints=(cut_joint,))


def _build_wedger():
    """210 g dual-wing stainless apple wedger: 12-gon ring & collar, 8 radial blades, hollow corer, flanged grips."""
    boxes = []
    # 12-sided stainless blade ring (OD 108 mm, ID 98 mm) + upper black ergonomic collar rim
    r_mid = 0.5 * (RING_OD + RING_ID) / 2.0
    wall_t = (RING_OD - RING_ID) / 2.0
    chord12 = 2.0 * r_mid * math.tan(math.pi / 12.0) + 0.0015
    for k in range(12):
        a = k * math.pi / 6.0
        ca, sa = math.cos(a), math.sin(a)
        boxes.append(((r_mid * ca, r_mid * sa, RING_H / 2), (wall_t, chord12, RING_H), STEEL, a))
        boxes.append((((r_mid + 0.001) * ca, (r_mid + 0.001) * sa, 0.0375), (0.0080, chord12 + 0.001, 0.0090),
                      HANDLE_BLK, a))
    # 8-sided hollow stainless central coring tube (OD 23.0 mm, ID 18.4 mm -> stem fits inside)
    rc_mid = 0.5 * (CORER_R_OUT + CORER_R_IN)
    wc = CORER_R_OUT - CORER_R_IN
    cc8 = 2.0 * rc_mid * math.tan(math.pi / 8.0) + 0.0008
    for k in range(8):
        a = k * math.pi / 4.0
        boxes.append(((rc_mid * math.cos(a), rc_mid * math.sin(a), 0.0200), (wc, cc8, 0.0400), STEEL, a))
    # 8 radial stainless divider blades from coring tube outer wall to blade ring inner wall
    r_blade_mid = 0.5 * (CORER_R_OUT + RING_ID / 2.0)
    blade_len = (RING_ID / 2.0) - CORER_R_OUT + 0.001
    for k in range(8):
        a = k * math.pi / 4.0
        boxes.append(((r_blade_mid * math.cos(a), r_blade_mid * math.sin(a), 0.0185),
                      (blade_len, BLADE_T, 0.0350), STEEL, a))
    # Dual wing handle flanges (z = 0.040..0.047) + raised 20 mm cushioned grip spines (z = 0.047..0.067) + gussets
    for sy in (-1.0, 1.0):
        boxes.append(((0.0, sy * 0.056, 0.0360), (0.028, 0.012, 0.010), HANDLE_BLK))
        boxes.append(((0.0, sy * 0.068, 0.0435), (0.054, 0.032, 0.007), HANDLE_BLK))
        boxes.append(((0.0, sy * HANDLE_Y, HANDLE_Z), (0.040, HANDLE_W_Y, 0.020), GRIP_ELAST))
    return part("wedger", (START_X, START_Y, TOP + MAT_H + 0.0005), mass=WEDGER_MASS, friction=0.95,
                color=STEEL, boxes=boxes)


BOARD = _build_board()
MAT = part("mat", (START_X, START_Y, TOP), fixed=True, friction=0.75, color=MAT_DARK,
           boxes=[
               ((0.0, 0.0, MAT_H / 2), (0.130, 0.210, MAT_H), MAT_DARK),
               ((0.0, 0.098, MAT_H + 0.001), (0.116, 0.006, 0.002), MAT_GUIDE),
               ((0.0, -0.098, MAT_H + 0.001), (0.116, 0.006, 0.002), MAT_GUIDE),
           ])
FRUIT = _build_fruit()
WEDGER = _build_wedger()
OBJECTS = (BOARD, MAT, FRUIT, WEDGER)

# -- scoring thresholds (every number in task.json scoring appears here as a literal) ----------------------------------
CRADLE_TOL = 0.012  # 12 mm: apple stays seated in the cutting board cradle
CENTER_TOL = 0.010  # 10 mm: wedger concentric with the seated apple
ENGAGE_CUT = 0.010  # 10 mm: initial cut engagement for partial tier
CUT_MIN = 0.042  # 42 mm of the 48 mm cut stroke required for full cut-through
DECK_TOL = 0.004  # 4 mm: wedger bottom ring flush on the cutting board deck
LEVEL_DEG = 5.0  # 5 deg: resting flatness limit on deck (apple tilt limit 8 deg in fruit_seated)
CLEAR_DIST = 0.120  # 120 mm: both grippers retracted clear of wedger and apple
HOLD_S = 3.0  # 3 s post-release stability hold (60 steps at 20 Hz)
HOLD_STEPS = 60
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.5
PASS_SCORE = 1.0 - 1e-9
STATE_EVERY = 40

_trial: dict = {}
_ids: dict = {}


def _fresh_trial() -> dict:
    return {
        "step": -1,
        "tcps": None,
        "cut_target": 0.0,
        "hold_steps": 0,
    }


_trial.update(_fresh_trial())


@on_reset
def _reset(_env) -> None:
    _trial.clear()
    _trial.update(_fresh_trial())
    _ids.clear()


def _rot(ps) -> np.ndarray:
    x, y, z, w = (float(v) for v in ps.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def tilt_deg(ps) -> float:
    r = _rot(ps)
    return math.degrees(math.acos(float(np.clip(r[2, 2], -1.0, 1.0))))


def cut_depth(fruit) -> float:
    return max(0.0, -float(fruit.joints.get("cut", 0.0)))


def fruit_seated(fruit) -> bool:
    p = np.asarray(fruit.pos, float)
    return (math.hypot(p[0] - BOARD_X, p[1] - BOARD_Y) <= CRADLE_TOL
            and abs(p[2] - WELL_Z) <= 0.005
            and tilt_deg(fruit) <= 8.0)


def wedger_centered(wedger, fruit) -> bool:
    pw, pf = np.asarray(wedger.pos, float), np.asarray(fruit.pos, float)
    return math.hypot(pw[0] - pf[0], pw[1] - pf[1]) <= CENTER_TOL


def wedger_on_deck(wedger) -> bool:
    z_gap = float(wedger.pos[2]) - DECK_Z
    return -0.002 <= z_gap <= DECK_TOL and tilt_deg(wedger) <= LEVEL_DEG


def hands_clear(wedger, fruit) -> bool:
    t = _trial["tcps"]
    if t is None or len(t) < 2:
        return False
    pw, pf = np.asarray(wedger.pos, float), np.asarray(fruit.pos, float)
    return all(float(np.linalg.norm(np.asarray(x, float) - pw)) > CLEAR_DIST
               and float(np.linalg.norm(np.asarray(x, float) - pf)) > CLEAR_DIST for x in t)


def _arm_tcps(env):
    try:
        base = env.unwrapped
        out = []
        for side in ("left", "right"):
            asset = base.scene[f"{side}_robot"]
            key = f"arm_{side}"
            if key not in _ids:
                _ids[key] = asset.find_joints(ARM_JOINTS, preserve_order=True)[0]
            q = labkit._np(asset.data.joint_pos)[0, _ids[key]]
            tcp_p, _ = ik.tcp_pose(q, station.BASES[side])
            out.append(tcp_p)
        return (out[0], out[1])
    except Exception:
        return None


def _advance_cut_yield(env, fruit) -> None:
    """Irreversible 18 N cutting law: advances continuous plastic target when push exceeds 4.5 mm (18 N) yield."""
    import torch

    q = float(fruit.joints.get("cut", 0.0))
    cur = float(_trial["cut_target"])
    if q < cur - CUT_YIELD_M:
        nxt = max(-CUT_STROKE, q + CUT_YIELD_M / 3.0)
        if nxt < cur - 1e-5:
            _trial["cut_target"] = nxt
            asset = env.unwrapped.scene["fruit"]
            if "fruit_cut" not in _ids:
                _ids["fruit_cut"] = [list(asset.joint_names).index("cut_joint")]
            asset.set_joint_position_target(torch.tensor([[nxt]], device=asset.device), joint_ids=_ids["fruit_cut"])


def _update_trial(wedger, fruit, tcps) -> None:
    """Advance the 3 s post-release stability hold counter by one 20 Hz step."""
    _trial["tcps"] = tcps
    done_now = (
        fruit_seated(fruit)
        and wedger_centered(wedger, fruit)
        and cut_depth(fruit) >= CUT_MIN
        and wedger_on_deck(wedger)
        and hands_clear(wedger, fruit)
    )
    _trial["hold_steps"] = int(_trial["hold_steps"]) + 1 if done_now else 0


@on_step
def _watch(env) -> None:
    wedger = labkit._part_state(env, labkit._SPECS["wedger"])
    fruit = labkit._part_state(env, labkit._SPECS["fruit"])
    _trial["step"] += 1
    _advance_cut_yield(env, fruit)
    tcps = _arm_tcps(env)
    _update_trial(wedger, fruit, tcps)
    if _trial["step"] % STATE_EVERY == STATE_EVERY - 1:
        print("STATE " + json.dumps({
            "step": _trial["step"] + 1,
            "cut_mm": round(cut_depth(fruit) * 1000, 1),
            "deck_gap_mm": round((float(wedger.pos[2]) - DECK_Z) * 1000, 1),
            "tilt_deg": round(tilt_deg(wedger), 2),
            "hold_steps": int(_trial["hold_steps"]),
        }), flush=True)


def components(objects: dict) -> dict:
    wedger, fruit = objects.get("wedger"), objects.get("fruit")
    if wedger is None or fruit is None:
        return {
            "fruit_seated": False, "wedger_centered": False, "cut_depth": 0.0,
            "wedger_on_deck": False, "hands_clear": False, "hold_steps": 0,
        }
    return {
        "fruit_seated": fruit_seated(fruit),
        "wedger_centered": wedger_centered(wedger, fruit),
        "cut_depth": cut_depth(fruit),
        "wedger_on_deck": wedger_on_deck(wedger),
        "hands_clear": hands_clear(wedger, fruit),
        "hold_steps": int(_trial["hold_steps"]),
    }


def score(objects: dict) -> float:
    k = components(objects)
    if not (k["fruit_seated"] and k["wedger_centered"]) or k["cut_depth"] < ENGAGE_CUT:
        return 0.0
    if k["cut_depth"] < CUT_MIN or not k["wedger_on_deck"]:
        return 0.25
    if not k["hands_clear"]:
        return 0.50
    if k["hold_steps"] < HOLD_STEPS:
        return 0.75
    return 1.0


def success(objects: dict) -> bool:
    return score(objects) >= PASS_SCORE


def distance(objects: dict) -> float:
    wedger, fruit = objects.get("wedger"), objects.get("fruit")
    if wedger is None or fruit is None:
        return 1.0
    xy = math.hypot(float(wedger.pos[0] - fruit.pos[0]), float(wedger.pos[1] - fruit.pos[1]))
    rem_cut = max(0.0, CUT_MIN - cut_depth(fruit))
    return float(xy + rem_cut)


# -- scripted demo (scene constants only) -----------------------------------------------------------------------------
LEAN_RAD = math.radians(18.0)  # splay wrists outward +-18 deg so housings clear and top camera sees blades
AIM_DX = 0.0060  # +6.0 mm forward aim compensates wrist pitch compliance for vertical cut over 0.345 m cradle
PRESS_OVERDRIVE = 0.026  # 26 mm below wing-shelf grip Z so compliant arm+wrist delivers > 18 N cleanly to deck


def demo():
    """Open-loop bimanual plan using scene constants only."""
    W = pitch_demo.Waypoint
    o, c = FINGER_OPEN, FINGER_CLOSED
    rl = axis_angle_matrix([1.0, 0.0, 0.0], -LEAN_RAD) @ pitch_demo.rotation(0.0)
    rr = axis_angle_matrix([1.0, 0.0, 0.0], LEAN_RAD) @ pitch_demo.rotation(0.0)
    r_park = pitch_demo.rotation(0.0)
    y_tcp = HANDLE_Y
    bx = BOARD_X + AIM_DX
    z_pick = TOP + MAT_H + HANDLE_Z + 0.004  # pads at z_pick - 12.4 mm sit 1.1 mm above 47 mm wing shelf
    z_carry = DECK_Z + FRUIT_H + HANDLE_Z + 0.034
    z_crown = DECK_Z + CUT_STROKE + HANDLE_Z + 0.008
    z_bottom = DECK_Z + HANDLE_Z + 0.004 - PRESS_OVERDRIVE

    seq = [
        (0.8, (START_X, y_tcp + 0.015, z_pick + 0.070), o, rl, (START_X, -y_tcp - 0.015, z_pick + 0.070), o, rr, "over grips"),
        (0.8, (START_X, y_tcp, z_pick), o, rl, (START_X, -y_tcp, z_pick), o, rr, "straddle both 20 mm grip bosses"),
        (1.0, (START_X, y_tcp, z_pick), c, rl, (START_X, -y_tcp, z_pick), c, rr, "pinch left and right handles"),
        (0.8, (START_X, y_tcp, z_carry), c, rl, (START_X, -y_tcp, z_carry), c, rr, "lift wedger clear"),
        (1.1, (bx, y_tcp, z_carry), c, rl, (bx, -y_tcp, z_carry), c, rr, "center over seated apple"),
        (0.7, (bx, y_tcp, z_crown), c, rl, (bx, -y_tcp, z_crown), c, rr, "register level on crown"),
        (2.6, (bx, y_tcp, z_bottom), c, rl, (bx, -y_tcp, z_bottom), c, rr, "drive level cut to deck"),
        (0.6, (bx, y_tcp, z_bottom), o, rl, (bx, -y_tcp, z_bottom), o, rr, "release handles in place"),
        (0.8, (bx, y_tcp + 0.025, z_carry), o, rl, (bx, -y_tcp - 0.025, z_carry), o, rr, "retract up and out"),
        (0.8, (0.18, 0.30, TOP + 0.16), o, r_park, (0.18, -0.30, TOP + 0.16), o, r_park, "park arms clear"),
        (2.4, (0.18, 0.30, TOP + 0.16), o, r_park, (0.18, -0.30, TOP + 0.16), o, r_park, "hold 3 s post-release"),
    ]
    left = [W(d, lp, lf, lab, rot=lr) for d, lp, lf, lr, _, _, _, lab in seq]
    right = [W(d, rp, rf, lab, rot=rr_) for d, _, _, _, rp, rf, rr_, lab in seq]
    return {"left": left, "right": right}
