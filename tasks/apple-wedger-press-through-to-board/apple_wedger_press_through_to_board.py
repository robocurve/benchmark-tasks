"""Center a dual-handle stainless apple wedger over an apple on a cutting board and press it level through to the board.

Real product class: the OXO Good Grips 32681 apple slicer, corer and divider -- a dual-handle stainless 8-blade
corer/wedger, 4.5 in (114 mm) blade span, 80 g (manufacturer listing), two cushioned side grips, an 8-sided hollow
central coring tube and eight radial blades -- used on a 203 x 305 x 19 mm (8 x 12 x 0.75 in) solid maple cutting
board, with a 66 mm Granny Smith apple standing on the board.

Bimanual physics: pressing down on one side grip only, 50 mm from the corer axis, against the cut resistance of the
eight seams rolls the wedger up; the blades cock and bind in the flesh. A level two-sided press drives all eight
blades down together and the ring bottoms out on the board.

The apple is eight wedges, each a free body glued to the world on its own glue line, plus the core plug. The blades and
the coring tube pass through the fruit without collision and no cutting force is modelled (the accepted #699
convention); a seam's glue is released by a rule on the wedger pose -- centred within 5 mm, level within 5 deg, ring
through to the board -- so the cut appears exactly where a blade is, and nowhere else. The scorer reads body poses, the glue-released set and the gripper clearance; the glue set is written only by
the geometry-gated cut rule above (no joint target is ever commanded), so what the scorer reads follows from where
the blades physically are.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import mechkit as mk  # noqa: E402, F401  (patches the task scene before labkit does)
import labkit  # noqa: E402, F401
import pitch_demo  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

# -- distinct PBR colours ---------------------------------------------------------------------------------------------
BOARD_WOOD = (0.78, 0.62, 0.40)
BOARD_TRIM = (0.55, 0.40, 0.26)
RUBBER_PAD = (0.14, 0.15, 0.17)
MAT_DARK = (0.19, 0.21, 0.24)
MAT_GUIDE = (0.31, 0.35, 0.40)
APPLE_SKIN = (0.72, 0.16, 0.18)
APPLE_FLESH = (0.96, 0.92, 0.78)
STEM_BROWN = (0.36, 0.22, 0.12)
LEAF_GREEN = (0.24, 0.56, 0.22)
STEEL = (0.76, 0.79, 0.82)
HANDLE_BLK = (0.13, 0.14, 0.16)
GRIP_ELAST = (0.24, 0.27, 0.30)

# -- 1:1 dimensions (metres, kg) --------------------------------------------------------------------------------------
BOARD_X, BOARD_Y = 0.360, 0.000  # board near edge at x 0.2585: 16 mm clear of the staged ring (edge 0.242)
BOARD_L, BOARD_W, BOARD_H = 0.203, 0.305, 0.019  # 8 x 12 x 0.75 in solid maple board
DECK_Z = TOP + BOARD_H

START_X, START_Y = 0.185, 0.000  # staging mat centre, 175 mm short of the board centre
MAT_H = 0.003

# Apple: 66 mm Granny Smith, 54 mm tall, standing on the board deck (no cradle pocket: a plain board).
APPLE_OD = 0.066  # 66 mm: a 64-67 mm Granny Smith (pre-pack size band)
APPLE_R = APPLE_OD / 2.0
APPLE_H = 0.054
CORE_HALF = 0.0050  # core plug: a 10 mm square (7.1 mm to a corner): clear of the 9.2 mm corer bore by 2.1 mm
WEDGE_R0 = 0.0140  # wedge inner faces at 14 mm: the 23 mm OD coring tube (12.4 mm to a corner) passes inside them
SEAM_GAP = 0.0060  # 6 mm pre-scored kerf between the modelled wedges (the stand-in spec); in PhysX the blades pass through anyway
FRUIT_MASS = 0.150  # 150 g apple; split between the eight wedges and the core plug

# Wedger: OXO 32681. Blade span 4.5 in = 114 mm (manufacturer); 80 g (manufacturer); the rest measured on our unit.
WEDGER_MASS = 0.080
WEDGER_SPAN = 0.114
RING_OD, RING_ID = 0.114, 0.104
RING_H = 0.060  # blade/ring height (listing: 2.1 in = 53 mm; +7 mm so the collar clears the 54 mm specimen)
CORER_R_OUT = 0.0115  # 23 mm OD hollow coring tube
CORER_R_IN = 0.0092
BLADE_T = float(os.environ.get("AW_BLADE_T", "0.0012"))  # 1.2 mm stainless blades (operator knob)
HANDLE_Y = 0.049  # cushioned side grips centred at +-49 mm (outer edges at the 114 mm span)
HANDLE_Z = float(os.environ.get("AW_HANDLE_Z", "0.062"))  # grip centre above the blade bottoms (operator knob)
HANDLE_W_Y = float(os.environ.get("AW_HANDLE_W", "0.026"))  # cushioned grip width along Y (operator knob)
N_BLADES = 8


# -- the board and the mat (labkit fixed parts) -----------------------------------------------------------------------
BOARD = labkit.part("board", (BOARD_X, BOARD_Y, TOP), fixed=True, friction=0.85, color=BOARD_WOOD,
                    boxes=[((0.0, 0.0, BOARD_H / 2 - 0.001), (BOARD_L, BOARD_W, BOARD_H - 0.002), BOARD_WOOD),
                           ((0.0, 0.0, BOARD_H - 0.0008), (BOARD_L - 0.004, BOARD_W - 0.004, 0.0016), BOARD_TRIM),
                           ((0.074, 0.0, BOARD_H - 0.0004), (0.012, 0.110, 0.0010), BOARD_TRIM)])

MAT = labkit.part("mat", (START_X, START_Y, TOP), fixed=True, friction=0.75, color=MAT_DARK,
                  boxes=[((0.0, 0.0, MAT_H / 2), (0.130, 0.210, MAT_H), MAT_DARK),
                         ((0.0, 0.098, MAT_H + 0.001), (0.116, 0.006, 0.002), MAT_GUIDE),
                         ((0.0, -0.098, MAT_H + 0.001), (0.116, 0.006, 0.002), MAT_GUIDE)])


# -- the apple: eight wedges and the core plug, each glued to the world on its own glue line ---------------------------
def _wedge_name(k: int) -> str:
    return f"wedge_{k}"


def _seam_name(k: int) -> str:
    return f"seam_{k}"


def _wedge_boxes(th: float) -> tuple:
    """One wedge: three radial bands of cream flesh inside a red skin cheek, filling the 45 deg sector less the kerf."""
    c, s = math.cos(th), math.sin(th)
    out = []
    bands = ((WEDGE_R0, 0.0175), (0.0175, 0.021), (0.021, 0.025), (0.025, 0.029), (0.029, APPLE_R))
    for r0, r1 in bands:
        rm = 0.5 * (r0 + r1)
        half_t = r0 * math.sin(math.pi / N_BLADES) - SEAM_GAP / 2.0  # fits inside the sector at its inner edge
        skin = r1 >= APPLE_R
        out.append(((r1 - r0, 2.0 * half_t, APPLE_H), (rm * c, rm * s, APPLE_H / 2.0),
                    APPLE_SKIN if skin else APPLE_FLESH, th))
    return tuple(out)


WEDGE_MASS = (FRUIT_MASS * 0.90) / N_BLADES
CORE_MASS = FRUIT_MASS * 0.10

kit = mk.Kit()
SEATS: dict[str, np.ndarray] = {}
for _k in range(N_BLADES):
    _th = (_k + 0.5) * 2.0 * math.pi / N_BLADES
    _name = _wedge_name(_k)
    _pos = (BOARD_X, BOARD_Y, DECK_Z + 0.0004)
    kit.part(_name, _pos, _wedge_boxes(_th), mass=WEDGE_MASS, color=APPLE_SKIN)
    SEATS[_name] = np.array(_pos, float)

kit.part("core_plug", (BOARD_X, BOARD_Y, DECK_Z + 0.0004),
         [mk.box((2.0 * CORE_HALF, 2.0 * CORE_HALF, APPLE_H), at=(0.0, 0.0, APPLE_H / 2.0), color=APPLE_FLESH),
          mk.box((0.0020, 0.0020, 0.0060), at=(0.0, 0.0, APPLE_H + 0.0025), color=STEM_BROWN),
          mk.box((0.0065, 0.0032, 0.0018), at=(0.0028, 0.0, APPLE_H + 0.0038), color=LEAF_GREEN, yaw=0.35)],
         mass=CORE_MASS, color=APPLE_FLESH)
SEATS["core_plug"] = np.array([BOARD_X, BOARD_Y, DECK_Z + 0.0004], float)

# Glue: every seam and the core are held by a bond. Released only by the cut rule below (sim: joint switched off).
for _k in range(N_BLADES):
    kit.bond(_seam_name(_k), None, _wedge_name(_k))
kit.bond("seam_core", None, "core_plug")


# -- the wedger -------------------------------------------------------------------------------------------------------
def _build_wedger() -> str:
    boxes = []
    r_mid = 0.5 * (RING_OD + RING_ID) / 2.0
    wall_t = (RING_OD - RING_ID) / 2.0
    chord12 = 2.0 * r_mid * math.tan(math.pi / 12.0) + 0.0015
    for k in range(12):  # 12-sided stainless blade ring with a dark collar rim on top
        a = k * math.pi / 6.0
        ca, sa = math.cos(a), math.sin(a)
        boxes.append(mk.box((wall_t, chord12, RING_H), at=(r_mid * ca, r_mid * sa, RING_H / 2.0), color=STEEL, yaw=a))
        boxes.append(mk.box((0.0080, chord12 + 0.001, 0.0090),
                            at=((r_mid + 0.001) * ca, (r_mid + 0.001) * sa, RING_H + 0.0045),
                            color=HANDLE_BLK, yaw=a))
    rc_mid = 0.5 * (CORER_R_OUT + CORER_R_IN)
    wc = CORER_R_OUT - CORER_R_IN
    cc8 = 2.0 * rc_mid * math.tan(math.pi / N_BLADES) + 0.0008
    for k in range(N_BLADES):  # 8-sided hollow stainless coring tube, one flat per seam plane
        a = k * 2.0 * math.pi / N_BLADES
        boxes.append(mk.box((wc, cc8, RING_H - 0.014),
                            at=(rc_mid * math.cos(a), rc_mid * math.sin(a), (RING_H - 0.014) / 2.0 + 0.012),
                            color=STEEL, yaw=a, collide=False))  # passes through the fruit (#699 convention)
    r_blade_mid = 0.5 * (CORER_R_OUT + RING_ID / 2.0)
    blade_len = (RING_ID / 2.0) - CORER_R_OUT + 0.001
    for k in range(N_BLADES):  # 8 radial stainless blades on the seam planes, running the full depth of the ring
        a = k * 2.0 * math.pi / N_BLADES
        boxes.append(mk.box((blade_len, BLADE_T, RING_H - 0.002),
                            at=(r_blade_mid * math.cos(a), r_blade_mid * math.sin(a), (RING_H - 0.002) / 2.0 + 0.001),
                            color=STEEL, yaw=a, collide=False))  # passes through the fruit (#699 convention)
    for sy in (-1.0, 1.0):  # cushioned side grips on the collar
        boxes.append(mk.box((0.034, 0.012, 0.010), at=(0.0, sy * 0.046, RING_H + 0.001), color=HANDLE_BLK))
        boxes.append(mk.box((0.040, HANDLE_W_Y, 0.020), at=(0.0, sy * HANDLE_Y, HANDLE_Z), color=GRIP_ELAST))
    return kit.part("wedger", (START_X, START_Y, TOP + MAT_H + 0.0005), boxes, mass=WEDGER_MASS, color=STEEL)


_build_wedger()

OBJECTS = [BOARD, MAT, *kit.objects()]

# -- scoring thresholds (every number in task.json scoring appears here as a literal) --------------------------------
CRADLE_TOL = 0.012  # 12 mm: each wedge stays within this of where it stood on the board
CENTER_TOL = 0.010  # 10 mm: wedger axle concentric with the apple
DECK_TOL = 0.0015  # 1.5 mm: the ring bottom sits on the board deck
LEVEL_DEG = 5.0  # 5 deg: wedger level within this on the deck
CLEAR_DIST = 0.120  # 120 mm: both grippers clear of wedger and apple
HOLD_S = 3.0  # 3 s of a full end state
HOLD_STEPS = 60
PART_CUT = 6  # 6 of 8 seams: the half-credit cut
# cut rule
ON_LINE = 0.0025  # a blade within 2.5 mm of a seam plane is cutting that seam
ANG_TOL = math.radians(6.0)  # ... and within 6 deg of it
THROUGH = APPLE_H - 0.002  # a seam gives only when its blade is through the flesh: the ring bottoming on the board
SUCCESS_HOLD_S = 3.0
SCORE_OVER = "end"
PASS_SCORE = 1.0 - 1e-9
STATE_EVERY = 20

_trial: dict = {}


def _fresh_trial() -> dict:
    return {"step": -1, "tcps": None, "hold_steps": 0, "cut": [False] * N_BLADES, "core": False}


_trial.update(_fresh_trial())


def _reset_trial() -> None:
    _trial.clear()
    _trial.update(_fresh_trial())


@labkit.on_reset
def _reset(_env) -> None:
    _reset_trial()


# -- geometry helpers --------------------------------------------------------------------------------------------------
def _rot(quat) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _tilt_deg(quat) -> float:
    r = _rot(quat)
    return math.degrees(math.acos(float(np.clip(r[2, 2], -1.0, 1.0))))


def _wedger_pose(objects):
    if "wedger" not in objects:
        return None, None
    pos, quat = kit.pose(objects, "wedger")
    return np.asarray(pos, float), quat


def _ring_gap(objects) -> float:
    """Ring bottom height above the board deck (negative: pressed into it)."""
    pos, _ = _wedger_pose(objects)
    return float("inf") if pos is None else float(pos[2]) - DECK_Z


def _separated(objects) -> int:
    """Seams cut through: the glue line a blade released. A wedge merely nudged by a finger is not a cut wedge."""
    return sum(1 for k in range(N_BLADES) if _seam_name(k) in kit._released)


def _wedges_seated(objects) -> bool:
    if not all(_wedge_name(k) in objects for k in range(N_BLADES)):
        return False
    for k in range(N_BLADES):
        pos, _ = kit.pose(objects, _wedge_name(k))  # the part origin, not the bounding-box centre
        p = np.asarray(pos, float)
        if math.hypot(float(p[0]) - SEATS[_wedge_name(k)][0], float(p[1]) - SEATS[_wedge_name(k)][1]) > CRADLE_TOL:
            return False
        if float(p[2]) < DECK_Z - 0.004 or float(p[2]) > DECK_Z + 0.040:
            return False
    return True


def _tcps() -> list:
    """Both grippers' TCPs from mechkit (works in Isaac and in the offline MuJoCo stand-in); [] offline."""
    return [np.asarray(p, float) for p in mk.tcps()]


def _hands_clear(objects) -> bool:
    t = _trial["tcps"]
    if t is None or len(t) < 2:
        return False
    pos, _ = _wedger_pose(objects)
    if pos is None:
        return False
    apple = np.array([BOARD_X, BOARD_Y, DECK_Z + APPLE_H / 2.0])
    return all(float(np.linalg.norm(np.asarray(x, float) - pos)) > CLEAR_DIST
               and float(np.linalg.norm(np.asarray(x, float) - apple)) > CLEAR_DIST for x in t)


# -- the cut rule: a seam gives where a blade is, in the flesh, level and centred ---------------------------------------
def _blade_dirs(quat) -> list[np.ndarray]:
    r = _rot(quat)
    out = []
    for j in range(N_BLADES):
        a = j * 2.0 * math.pi / N_BLADES
        out.append(np.array([math.cos(a), math.sin(a), 0.0]) @ r.T)
    return out


def _blade_depth(objects) -> float:
    """How far the blade bottoms are below the apple crown (positive = in the flesh); -inf when not over it."""
    pos, _ = _wedger_pose(objects)
    if pos is None:
        return float("-inf")
    if math.hypot(float(pos[0]) - BOARD_X, float(pos[1]) - BOARD_Y) > APPLE_R:
        return float("-inf")
    return (DECK_Z + APPLE_H) - (float(pos[2]) + 0.001)  # blade bottoms sit 1 mm above the ring bottom


def _cut_rule(objects) -> None:
    pos, quat = _wedger_pose(objects)
    if pos is None:
        return
    if _blade_depth(objects) < THROUGH:  # the blades are not through the flesh yet (a press that stops short cuts nothing)
        return
    if _tilt_deg(quat) > LEVEL_DEG:
        return
    centre = np.array([BOARD_X, BOARD_Y, 0.0])
    offset = np.array([pos[0], pos[1], 0.0]) - centre
    if math.hypot(offset[0], offset[1]) > CENTER_TOL * 0.5:  # every blade within 5 mm of its seam
        return
    dirs = _blade_dirs(quat)
    for k in range(N_BLADES):
        if _trial["cut"][k]:
            continue
        a = k * 2.0 * math.pi / N_BLADES
        u = np.array([math.cos(a), math.sin(a), 0.0])
        for v in dirs:
            lateral = abs(float(u[0] * offset[1] - u[1] * offset[0]))
            ang = math.acos(max(-1.0, min(1.0, abs(float(np.dot(u, v))))))
            if lateral <= ON_LINE and ang <= ANG_TOL:
                kit.release(_seam_name(k))
                _trial["cut"][k] = True
                break
    if not _trial["core"] and _blade_depth(objects) >= THROUGH:  # corer through to the board
        kit.release("seam_core")
        _trial["core"] = True


def _watch(objects) -> None:
    """Every control step (registered with kit.on_step, so it receives the built state dict)."""
    _trial["step"] += 1
    _trial["tcps"] = _tcps()
    _cut_rule(objects)
    done = (_separated(objects) == N_BLADES and _wedges_seated(objects) and _ring_ok(objects)
            and _hands_clear(objects))
    _trial["hold_steps"] = int(_trial["hold_steps"]) + 1 if done else 0
    if _trial["step"] % STATE_EVERY == STATE_EVERY - 1:
        pos, quat = _wedger_pose(objects)
        r = _rot(quat) if quat is not None else None
        print("STATE " + json.dumps({
            "step": _trial["step"] + 1,
            "cut": _separated(objects),
            "deck_gap_mm": round((float(pos[2]) - DECK_Z) * 1000, 1) if pos is not None else None,
            "dx_mm": round((float(pos[0]) - BOARD_X) * 1000, 1) if pos is not None else None,
            "dy_mm": round((float(pos[1]) - BOARD_Y) * 1000, 1) if pos is not None else None,
            "yaw_deg": round(math.degrees(math.atan2(float(r[1, 0]), float(r[0, 0]))), 2) if r is not None else None,
            "tilt_deg": round(_tilt_deg(quat), 2) if quat is not None else None,
            "hold_steps": int(_trial["hold_steps"]),
        }), flush=True)


kit.on_step(_watch)


def _ring_ok(objects) -> bool:
    pos, quat = _wedger_pose(objects)
    if pos is None:
        return False
    gap = _ring_gap(objects)
    if not (-0.002 <= gap <= DECK_TOL):
        return False
    if _tilt_deg(quat) > LEVEL_DEG:
        return False
    return math.hypot(float(pos[0]) - BOARD_X, float(pos[1]) - BOARD_Y) <= CENTER_TOL


# -- scorer: observable end state only ---------------------------------------------------------------------------------
def components(objects: dict) -> dict:
    pos, quat = _wedger_pose(objects)
    return {
        "separated": _separated(objects),
        "wedges_seated": _wedges_seated(objects),
        "deck_gap": _ring_gap(objects),
        "tilt_deg": _tilt_deg(quat) if quat is not None else 180.0,
        "centered": bool(pos is not None
                         and math.hypot(float(pos[0]) - BOARD_X, float(pos[1]) - BOARD_Y) <= CENTER_TOL),
        "hands_clear": _hands_clear(objects),
        "hold_steps": int(_trial["hold_steps"]),
    }


def score(objects: dict) -> float:
    k = components(objects)
    level_over_apple = k["tilt_deg"] <= LEVEL_DEG and k["centered"]
    flush = level_over_apple and -0.002 <= k["deck_gap"] <= DECK_TOL
    if not k["wedges_seated"]:
        return 0.0
    if flush and k["separated"] == N_BLADES:
        if not k["hands_clear"]:
            return 0.50
        return 1.0 if k["hold_steps"] >= HOLD_STEPS else 0.75
    if flush and k["separated"] >= PART_CUT:
        return 0.50
    return 0.0


def success(objects: dict) -> bool:
    return score(objects) >= PASS_SCORE


def distance(objects: dict) -> float:
    pos, _ = _wedger_pose(objects)
    if pos is None:
        return 1.0
    xy = math.hypot(float(pos[0]) - BOARD_X, float(pos[1]) - BOARD_Y)
    return float(xy + 0.05 * (N_BLADES - _separated(objects)))


# -- scripted demo (scene constants only) ------------------------------------------------------------------------------
LEAN_RAD = math.radians(float(os.environ.get("AW_LEAN_DEG", "14.0")))  # outward wrist splay (operator knob): 14 deg keeps the
# link-4 housings from pushing each other during the straddle and pinch (9 deg: 19.7 N; 14 deg: 3.9 N)
AIM_DX = float(os.environ.get("AW_AIM_DX", "0.0055"))  # forward aim against wrist compliance (operator knob)
PRESS_OVERDRIVE = 0.020
REGISTER_ABOVE = float(os.environ.get("AW_REGISTER_ABOVE", "0.004"))  # blade bottoms stop 4 mm above the crown after the
# 0.7 s register move (2-4 mm per step); the 4 s drive then meets the fruit at 0.9 mm per step. Hitting the crown at register
# speed swings the ring about the pinch axis in PhysX (rigid contacts) and the blades jam against the kerf walls.
PINCH_SQUEEZE = float(os.environ.get("AW_PINCH_SQUEEZE", "0.0025"))  # finger target 2.5 mm past grip contact (operator
# knob). The YAM finger carriages are offset from each other across the closing axis (URDF joint7/joint8 origins), so a
# full-force pinch (target 0: 13 mm of error at 5000 N/m, capped 50 N) is a couple that yaws the wedger in the pads and
# deflects the 10 Nm/rad wrists (4.6 deg at the crown in Isaac); a 2.5 mm squeeze (~12 N per pad) holds the 80 g wedger
# and keeps that couple small (0.4 deg).


def demo():
    """Open-loop bimanual plan using scene constants only."""
    W = pitch_demo.Waypoint
    o = FINGER_OPEN
    c = FINGER_CLOSED - (HANDLE_W_Y / 2.0 - PINCH_SQUEEZE)  # pads meet the 26 mm grip at -13 mm; squeeze 2.5 mm past it
    o_rel = FINGER_CLOSED - (HANDLE_W_Y / 2.0 + 0.012)  # release: open 12 mm past the grip faces, not the full 94 mm --
    # fully open, the two inboard pads meet at the midline between the +-49 mm grips
    from yam_arena.urdf import axis_angle_matrix

    rl = axis_angle_matrix([1.0, 0.0, 0.0], -LEAN_RAD) @ pitch_demo.rotation(0.0)
    rr = axis_angle_matrix([1.0, 0.0, 0.0], LEAN_RAD) @ pitch_demo.rotation(0.0)
    r_park = pitch_demo.rotation(0.0)
    y_tcp = HANDLE_Y
    bx = BOARD_X + AIM_DX
    z_pick = TOP + MAT_H + HANDLE_Z + 0.0124  # pad centres on the 20 mm grips
    z_carry = DECK_Z + APPLE_H + HANDLE_Z + 0.0124 + 0.030
    z_crown = DECK_Z + APPLE_H + HANDLE_Z + 0.0124 + REGISTER_ABOVE  # register ABOVE the crown; the slow drive crosses it
    z_bottom = DECK_Z + HANDLE_Z + 0.0124 - PRESS_OVERDRIVE

    seq = [
        (0.8, (START_X, y_tcp + 0.015, z_pick + 0.070), o, rl, (START_X, -y_tcp - 0.015, z_pick + 0.070), o, rr, "over grips"),
        (0.8, (START_X, y_tcp, z_pick), o, rl, (START_X, -y_tcp, z_pick), o, rr, "straddle both grips"),
        (1.0, (START_X, y_tcp, z_pick), c, rl, (START_X, -y_tcp, z_pick), c, rr, "pinch left and right handles"),
        (0.8, (START_X, y_tcp, z_carry), c, rl, (START_X, -y_tcp, z_carry), c, rr, "lift wedger clear"),
        (1.1, (bx, y_tcp, z_carry), c, rl, (bx, -y_tcp, z_carry), c, rr, "center over the apple"),
        (0.7, (bx, y_tcp, z_crown), c, rl, (bx, -y_tcp, z_crown), c, rr, "register level on the crown"),
        (4.0, (bx, y_tcp, z_bottom), c, rl, (bx, -y_tcp, z_bottom), c, rr, "drive level through to the board"),
        (0.6, (bx, y_tcp, z_bottom), o_rel, rl, (bx, -y_tcp, z_bottom), o_rel, rr, "release handles in place"),
        (0.8, (bx, y_tcp + 0.025, z_carry), o_rel, rl, (bx, -y_tcp - 0.025, z_carry), o_rel, rr, "retract up and out"),
        (0.8, (0.18, 0.30, TOP + 0.16), o, r_park, (0.18, -0.30, TOP + 0.16), o, r_park, "park arms clear"),
        (3.0, (0.18, 0.30, TOP + 0.16), o, r_park, (0.18, -0.30, TOP + 0.16), o, r_park, "hold 3 s post-release"),
    ]
    left = [W(d, lp, lf, lab, rot=lr_) for d, lp, lf, lr_, _, _, _, lab in seq]
    right = [W(d, rp, rf, lab, rot=rr_) for d, _, _, _, rp, rf, rr_, lab in seq]
    return {"left": left, "right": right}
