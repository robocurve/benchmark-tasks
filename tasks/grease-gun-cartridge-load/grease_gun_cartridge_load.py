"""Reload a grease gun barrel: catch the spring follower rod's neck in the end-cap keyhole slot and push a new 14 oz
grease cartridge into the barrel, open end first, until its end is flush with the mouth.

Bench layout in station coordinates (TOP = station.TABLE_TOP, +X forward, +Y left, metres):
- mat (fixed): ribbed rubber bench mat 560 x 640 x 2 mm centred at (0.270, 0.000, TOP).
- gun (fixed): grease-gun barrel 56 mm OD, 52 mm bore, 250 mm long, clamped in a rubber bench holder with its axis along
  Y at x = 0.320, 38 mm above the table; open mouth at y = +0.125 (toward the left arm), end cap at y = -0.128. The cap
  plate has a keyhole: a 16 mm hole on the axis joined to a 6.5 mm slot running 16.5 mm straight down. The holder
  continues past the mouth as a loading V (y +0.135..+0.330) that puts a laid cartridge on the bore axis.
- plunger (fixed base, two joints, its own part so it collides with the cap and the cartridge):
  ``follower`` (slide Y, -0.245..0): follower disc in the bore on the compression spring, 58.3 N/m from rest (14 N at
  the 240 mm catch). ``rod`` (slide Z on the follower, -0.0145..+0.003): the 10 mm rod, its 5 mm neck right behind the
  follower and the 100 mm T-handle (y -0.159 at rest); the rod drops in the cap plane only as far as the keyhole lets it.
- cartridge (loose, 0.45 kg): 14 oz grease cartridge, 50 mm x 240 mm tube, open end toward -Y, on a fixed two-block
  rest at (0.185, 0.215), axis 34 mm above the table.
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
import labkit  # noqa: E402
from labkit import Joint, PartState, on_reset, on_step, part  # noqa: E402
from yam_arena import demo as _demo
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN

# -- geometry & scoring thresholds (single source of truth for score(), success(), the hook and demo()) ----

TOP = station.TABLE_TOP
CONTROL_DT = 0.05  # 20 Hz control step

MAT_POS = (0.270, 0.000, TOP)
MAT_SIZE = (0.560, 0.640, 0.002)
MAT_H = MAT_SIZE[2]

# Barrel (gun part frame: origin on the barrel axis at mid-length, axis along +Y toward the mouth)
GUN_X = 0.320
AXIS_H = 0.036  # barrel axis above the mat
GUN_SPAWN_POS = (GUN_X, 0.0, TOP + MAT_H + AXIS_H)
BARREL_MASS = 0.40
BARREL_LEN = 0.250
MOUTH_Y = BARREL_LEN / 2.0  # 0.125
CAP_IN_Y = -BARREL_LEN / 2.0  # -0.125: inner face of the cap plate
CAP_T = 0.003
CAP_OUT_Y = CAP_IN_Y - CAP_T  # -0.128
BORE_R = 0.026  # 52 mm bore
BARREL_T = 0.002
BARREL_R = BORE_R + BARREL_T  # 56 mm OD
COLLAR_R = 0.031  # 62 mm end-cap collar
COLLAR_Y0, COLLAR_Y1 = -0.128, -0.110
HOLE_HALF = 0.008  # 16 mm keyhole hole (square collision opening, 16 x 16 mm)
SLOT_HALF = 0.00325  # 6.5 mm slot
SLOT_BOTTOM = -0.0165  # slot end, below the axis

# Follower, rod, neck, T-handle (gun frame at joint zero)
SPRING_K = 14.0 / 0.240  # 58.3 N/m from rest: 14 N at the 240 mm catch
STROKE_MAX = 0.245
ROD_R = 0.005
NECK_R = 0.0025
NECK_Y0, NECK_Y1 = 0.109, 0.118  # neck spans the cap plate for strokes 0.237 .. 0.243
FOLLOWER_Y0, FOLLOWER_Y1 = 0.118, 0.124
FOLLOWER_R = 0.0235
TBAR_R = 0.006
TBAR_LEN = 0.100
TBAR_Y = CAP_OUT_Y - 0.025 - TBAR_R  # -0.159: T-handle 25 mm off the cap plate at rest (finger room)
ROD_DROP_MIN = -0.0145
ROD_REST_Z = -0.003  # the rod lies on the bottom of the 16 mm hole at reset

# Cartridge (part frame: origin on its axis at mid-length; open end toward -Y, tab-seal end at +Y)
CART_MASS = 0.45
CART_R = 0.025
CART_HALF = 0.120  # 240 mm long: pushed onto the caught follower (face 113 mm behind the axis centre) it stands 2 mm proud
CART_AXIS_H = 0.032
CART_SPAWN_POS = (0.185, 0.215, TOP + MAT_H + CART_AXIS_H)

# Scoring thresholds
LATCH_STROKE_M = 0.230  # follower drawn back >= 230 mm (T-handle >= 23 cm out from the cap)
NECK_IN_SLOT_M = -0.010  # rod dropped >= 10 mm into the slot: its shoulder bears on the slot edges
SEAT_GAP_M = 0.005  # cartridge rim within 5 mm of the barrel mouth face
CART_AXIS_DEG = 5.0
CART_RADIAL_M = 0.004
HAND_CLEAR_M = 0.100
HAND_OFF_ROD_M = 0.060  # no gripper TCP within 6 cm of the T-handle: the cap, not a hand, holds the caught rod
CART_STILL_MPS = 0.020  # cartridge centre slower than 2 cm/s = still (it may roll slightly about its own axis in the bore)
HOLD_S = 2.0  # final state held this long inside score()

S_ROD = 0.35
S_CART = 0.45
S_FINAL = 0.20
SCORE_LEVELS = (0.0, 0.35, 0.80, 1.0)  # every total score() can return

SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
PASS_SCORE = 1.0 - 1e-9

# -- colours ---------------------------------------------------------------------------------------------------------

RUBBER = (0.10, 0.10, 0.11)
STEEL = (0.62, 0.64, 0.67)
RED = (0.70, 0.10, 0.08)
CART_BLUE = (0.10, 0.22, 0.55)
TIN = (0.80, 0.80, 0.78)

VIS = pathlib.Path(_HERE) / "assets" / "visual"
SHOW_SHAPES = not (VIS / "gun.usda").exists()  # collision shapes are drawn only until the visual meshes exist

# -- objects ---------------------------------------------------------------------------------------------------------


def _vis(name: str, body: str = "") -> list:
    path = VIS / f"{name}.usda"
    return [(body, str(path))] if path.exists() else []


MAT = part(
    "mat",
    pos=MAT_POS,
    fixed=True,
    friction=0.9,
    boxes=[((0.0, 0.0, MAT_H / 2.0), MAT_SIZE, RUBBER)],
    visuals=_vis("mat") + _vis("bench_props"),
    hide_shapes=not SHOW_SHAPES,
)


def _ring(r_in: float, t: float, y0: float, y1: float, color, n: int = 16) -> list:
    """A tube along Y (inscribed radius r_in, wall t) as n flat slabs turned about Y."""
    w = 2.0 * (r_in + t) * math.tan(math.pi / n) + 0.0004
    rc = r_in + t / 2.0
    out = []
    for k in range(n):
        phi = 2.0 * math.pi * k / n
        out.append(((rc * math.sin(phi), 0.5 * (y0 + y1), rc * math.cos(phi)), (w, y1 - y0, t), color, (0.0, phi)))
    return out


def _keyhole_plate() -> list:
    """Cap plate y -0.128..-0.125 with a 16 mm (square) hole on the axis and a 6.5 mm slot down to z -0.0165."""
    yc = 0.5 * (CAP_OUT_Y + CAP_IN_Y)
    R = COLLAR_R
    h, s = HOLE_HALF, SLOT_HALF
    boxes = [
        ((0.0, yc, 0.5 * (h + R)), (2 * R, CAP_T, R - h), STEEL),  # above the hole
        ((0.0, yc, 0.5 * (SLOT_BOTTOM - R)), (2 * s + 0.0002, CAP_T, R + SLOT_BOTTOM), STEEL),  # below the slot end
    ]
    for sx in (-1.0, 1.0):
        boxes.append(((sx * 0.5 * (h + R), yc, 0.0), (R - h, CAP_T, 2 * h), STEEL))  # beside the hole
        boxes.append(((sx * 0.5 * (s + R), yc, 0.5 * (-h - R)), (R - s, CAP_T, R - h), STEEL))  # beside the slot
    return boxes


FOLLOWER = Joint(
    name="follower",
    kind="slide",
    axis="Y",
    lower=-STROKE_MAX,
    upper=0.0,
    target=0.0,
    stiffness=SPRING_K,
    damping=0.5,
    initial=0.0,
    mass=0.03,
    cylinders=[((0.0, 0.5 * (FOLLOWER_Y0 + FOLLOWER_Y1), 0.0), FOLLOWER_R, FOLLOWER_Y1 - FOLLOWER_Y0, TIN, "Y")],
)

ROD = Joint(
    name="rod",
    kind="slide",
    axis="Z",
    parent="follower",
    lower=ROD_DROP_MIN,
    upper=0.003,
    initial=ROD_REST_Z,
    target=ROD_REST_Z,
    stiffness=0.0,
    damping=0.2,
    mass=0.06,
    cylinders=[
        ((0.0, 0.5 * (CAP_OUT_Y - 0.0005 + NECK_Y0), 0.0), ROD_R, NECK_Y0 - CAP_OUT_Y + 0.0005, STEEL, "Y"),
        ((0.0, 0.5 * (NECK_Y0 + NECK_Y1), 0.0), NECK_R, NECK_Y1 - NECK_Y0, STEEL, "Y"),
        ((0.0, TBAR_Y, 0.0), TBAR_R, TBAR_LEN, RED, "X"),
    ],
)

# V-cradle with a clamp band: faces tangent to the barrel 45 deg either side of the bottom.
_V_T = 0.010
_V_W = 0.026
_V_C = BARREL_R * math.sin(math.pi / 4) + 0.5 * _V_T * math.sin(math.pi / 4)
CRADLE_REAR_Y = (-0.1085, -0.0785)
CRADLE_FRONT_Y = (0.060, 0.090)
TROUGH_Y = ((0.135, 0.165), (0.300, 0.330))  # loading V in front of the mouth: a cartridge laid in it is on the bore axis
TROUGH_DZ = BORE_R - CART_R  # the laid cartridge's underside is level with the bore floor (no step either way)
_T_C = CART_R * math.sin(math.pi / 4) + 0.5 * _V_T * math.sin(math.pi / 4)


def _vblock(y0: float, y1: float) -> list:
    yc, L = 0.5 * (y0 + y1), y1 - y0
    z = -_V_C
    return [
        ((+_V_C, yc, z), (_V_W, L, _V_T), RUBBER, (0.0, -math.pi / 4)),
        ((-_V_C, yc, z), (_V_W, L, _V_T), RUBBER, (0.0, +math.pi / 4)),
        ((0.0, yc, 0.004 - AXIS_H), (0.070, L, 0.008), RUBBER),
    ]


def _trough(y0: float, y1: float) -> list:
    yc, L = 0.5 * (y0 + y1), y1 - y0
    return [
        ((+_T_C, yc, -_T_C - TROUGH_DZ), (_V_W, L, _V_T), RUBBER, (0.0, -math.pi / 4)),
        ((-_T_C, yc, -_T_C - TROUGH_DZ), (_V_W, L, _V_T), RUBBER, (0.0, +math.pi / 4)),
        ((0.0, yc, 0.004 - AXIS_H), (0.070, L, 0.008), RUBBER),
    ]


# The barrel with its keyhole end cap, clamped in the bench holder (fixed). Frame: origin on the barrel axis.
GUN = part(
    "gun",
    pos=GUN_SPAWN_POS,
    fixed=True,
    friction=0.45,
    visuals=_vis("gun"),
    hide_shapes=not SHOW_SHAPES,
    boxes=_ring(BORE_R, BARREL_T, CAP_IN_Y, MOUTH_Y, STEEL)
    + _ring(BARREL_R, COLLAR_R - BARREL_R, COLLAR_Y0, COLLAR_Y1, STEEL)
    + _keyhole_plate()
    + _vblock(*CRADLE_REAR_Y)
    + _vblock(*CRADLE_FRONT_Y)
    + _trough(*TROUGH_Y[0])
    + _trough(*TROUGH_Y[1]),
)

# Follower and rod on the spring, in the same barrel frame. Its base is the spring seat at the cap; the follower slides
# along the bore on the spring, the rod rides on the follower and can drop in the cap plane only as far as the keyhole
# lets it. A separate part so that the rod and follower really collide with the cap and with the cartridge.
PLUNGER = part(
    "plunger",
    pos=GUN_SPAWN_POS,
    fixed=True,
    friction=0.30,
    joints=(FOLLOWER, ROD),
    visuals=_vis("follower", "follower") + _vis("rod", "rod"),
    hide_shapes=not SHOW_SHAPES,
    boxes=[((0.0, CAP_IN_Y + 0.004, 0.019), (0.006, 0.006, 0.004), TIN)],  # spring seat tab inside the cap
)

CARTRIDGE = part(
    "cartridge",
    pos=CART_SPAWN_POS,
    mass=CART_MASS,
    friction=0.35,
    visuals=_vis("cartridge"),
    hide_shapes=not SHOW_SHAPES,
    cylinders=[((0.0, 0.0, 0.0), CART_R, 2 * CART_HALF, CART_BLUE, "Y")],
)

# Cartridge rest: two low V blocks under the cartridge body ends (the grip zone in the middle stays clear).
_CV_C = CART_R * math.sin(math.pi / 4) + 0.5 * 0.008 * math.sin(math.pi / 4)


def _cblock(yc: float) -> list:
    z = CART_AXIS_H - _CV_C
    return [
        ((+_CV_C, yc, z), (0.018, 0.030, 0.008), RUBBER, (0.0, -math.pi / 4)),
        ((-_CV_C, yc, z), (0.018, 0.030, 0.008), RUBBER, (0.0, +math.pi / 4)),
    ]


REST = part(
    "rest",
    pos=(CART_SPAWN_POS[0], CART_SPAWN_POS[1], TOP + MAT_H),
    fixed=True,
    friction=0.9,
    visuals=_vis("rest"),
    hide_shapes=not SHOW_SHAPES,
    boxes=_cblock(-0.080) + _cblock(+0.080),
)

SCENE = "station"
OBJECTS = (MAT, GUN, PLUNGER, CARTRIDGE, REST)

# -- trial memory (filled by score() and the observe-only hook; cleared on reset) ---------------------------

_ids: dict = {}
_trial: dict = {}


def _reset_trial() -> None:
    _ids.clear()
    _trial.clear()
    _trial.update(tcps=None, step=0, hold_s=0.0, failed=False)


_reset_trial()


def _set_test_trial(*, tcps=None) -> None:
    """Test helper only; allows offline unit tests to supply arm TCP poses."""
    _trial["tcps"] = tcps


def _angle_deg(a, b) -> float:
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    c = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def _metrics(objects: dict[str, PartState]) -> dict:
    gun = objects["gun"]
    plunger = objects["plunger"]
    cart = objects["cartridge"]
    stroke = -float(plunger.joints["follower"])
    drop = float(plunger.joints["rod"])
    latched = stroke >= LATCH_STROKE_M and drop <= NECK_IN_SLOT_M

    g_axis = gun.rotation()[:, 1]
    c_axis = cart.rotation()[:, 1]
    end_local = gun.to_local(cart.to_world((0.0, CART_HALF, 0.0)))  # tab-end face centre in the barrel frame
    seat_gap = float(end_local[1] - MOUTH_Y)
    radial = float(math.hypot(end_local[0], end_local[2]))
    axis_err = _angle_deg(g_axis, c_axis)  # open end first: the cartridge +Y (tab end) points out of the mouth
    seated = abs(seat_gap) <= SEAT_GAP_M and axis_err <= CART_AXIS_DEG and radial <= CART_RADIAL_M

    dropped = float(cart.low[2]) < TOP - 0.05
    return {
        "stroke": stroke,
        "drop": drop,
        "latched": latched,
        "seat_gap": seat_gap,
        "radial": radial,
        "axis_err": axis_err,
        "seated": seated,
        "dropped": dropped,
        "resting": bool(float(np.linalg.norm(cart.lin_vel)) < CART_STILL_MPS),
    }


def _point_segment_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-12:
        return float(np.linalg.norm(p - a))
    t = max(0.0, min(1.0, float(np.dot(p - a, ab) / denom)))
    return float(np.linalg.norm(p - (a + t * ab)))


def _hands_clear(objects: dict[str, PartState], tcps) -> bool:
    """Both TCPs >= 10 cm (in plan) from the gun: the line from the drawn-out T-handle to the barrel mouth."""
    if tcps is None or len(tcps) < 2:
        return False
    gun = objects["gun"]
    a = gun.to_world((0.0, TBAR_Y - STROKE_MAX, 0.0))[:2]
    b = gun.to_world((0.0, MOUTH_Y + 0.02, 0.0))[:2]
    return all(_point_segment_dist(np.asarray(tp, float)[:2], a, b) >= HAND_CLEAR_M for tp in tcps)


def _hand_off_rod(objects: dict[str, PartState], tcps) -> bool:
    """No gripper TCP within HAND_OFF_ROD_M of the T-handle centre (unknown TCPs fail closed)."""
    if tcps is None or len(tcps) < 2:
        return False
    j = objects["plunger"].joints
    t = objects["gun"].to_world((0.0, TBAR_Y + float(j["follower"]), float(j["rod"])))
    return all(float(np.linalg.norm(np.asarray(tp, float) - t)) >= HAND_OFF_ROD_M for tp in tcps)


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
            out.append(ik.tcp_pose(q, station.BASES[side])[0])
        return (out[0], out[1])
    except Exception:
        return None


@on_reset
def _on_reset(env) -> None:
    _reset_trial()


SUBACTIONS = (
    ("rod drawn back", "plunger.follower"),
    ("neck in cap slot", "plunger.rod"),
    ("cartridge seated", "cartridge"),
    ("arms clear", "tcps"),
)


@on_step
def _watch(env) -> None:
    """Observe-only: record the arm TCPs for the clearance check and log the sub-action end states during the final
    hold. Moves nothing and writes no joint target."""
    _trial["step"] += 1
    _trial["tcps"] = _arm_tcps(env)
    if _trial["hold_s"] < HOLD_S:
        return
    gun = labkit._part_state(env, labkit._SPECS["gun"])
    plunger = labkit._part_state(env, labkit._SPECS["plunger"])
    cart = labkit._part_state(env, labkit._SPECS["cartridge"])
    tcps = _trial["tcps"]
    ends = (
        round(float(plunger.joints.get("follower", 0.0)), 6),
        round(float(plunger.joints.get("rod", 0.0)), 6),
        [round(float(v), 6) for v in gun.to_local(cart.pos)],
        [[round(float(v), 4) for v in t] for t in tcps] if tcps else None,
    )
    for (name, key), end in zip(SUBACTIONS, ends):
        rec = {"name": name, "joint_or_body": key, "end_value": end, "step": _trial["step"], "hold_s": _trial["hold_s"]}
        print("SUBACTION_END " + json.dumps(rec), flush=True)


# -- scoring -----------------------------------------------------------------------------------------------


def _evaluate(objects: dict[str, PartState], update: bool) -> float:
    try:
        m = _metrics(objects)
    except (KeyError, TypeError, AttributeError):  # a tracked part is missing: fail closed
        m = None
    if update and m is not None and m["dropped"]:
        _trial["failed"] = True  # remembered: a cartridge that fell off the bench fails the run even if put back
    if m is None or m["dropped"] or _trial["failed"]:
        if update:
            _trial["hold_s"] = 0.0
        return 0.0
    total = 0.0
    if m["latched"] and _hand_off_rod(objects, _trial["tcps"]):
        total += S_ROD
        if m["seated"]:  # the cartridge only counts behind a caught rod (the follower must be out of the bore)
            total += S_CART
    final_ready = (m["latched"] and m["seated"] and m["resting"] and _hand_off_rod(objects, _trial["tcps"])
                   and _hands_clear(objects, _trial["tcps"]))
    if update:
        _trial["hold_s"] = _trial["hold_s"] + CONTROL_DT if final_ready else 0.0
    if final_ready and _trial["hold_s"] >= HOLD_S - 1e-9:
        total += S_FINAL
    return round(total, 6)


def score(objects: dict[str, PartState]) -> float:
    """0.35 follower drawn back >= 230 mm with its rod neck dropped >= 10 mm into the cap slot and no gripper within
    6 cm of the T-handle (the cap holds it); +0.45 (only with the rod caught) cartridge in the bore, its end within 5 mm
    of the mouth face, axis within 5 deg and 4 mm of the barrel axis; +0.20 all of that with the cartridge centre
    moving < 2 cm/s and both grippers >= 10 cm from the gun, held 2 s. Evaluated on the current state every control
    step, except one remembered failure: once the cartridge has fallen off the bench the run scores 0 to the end. A
    part missing: 0."""
    return _evaluate(objects, update=True)


def success(objects: dict[str, PartState]) -> bool:
    """Pass iff the full score holds (same thresholds as score(); does not advance the hold timer)."""
    return _evaluate(objects, update=False) >= PASS_SCORE


def distance(objects: dict[str, PartState]) -> float:
    """Metres from the loaded goal: stroke still to draw plus the cartridge rim's distance from the mouth."""
    m = _metrics(objects)
    return float(max(0.0, LATCH_STROKE_M - m["stroke"]) + max(0.0, m["drop"] - NECK_IN_SLOT_M) + abs(m["seat_gap"]))


# -- scripted demo (fixed bench constants only; never reads _trial or the simulator) -------------------------

TIP_BELOW_TCP = 0.013
PULL_OVER_M = 0.070
FINGER_BAR = -0.014  # fingers opened 28 mm around the 12 mm T-handle bar (clear of the cap behind it)
YAW_X = math.pi / 2  # fingers close along world x


def demo() -> dict[str, list[_demo.Waypoint]]:
    """Left hand grips the barrel in its cradle; right hand pinches the T-handle beside the rod, draws it 243 mm back
    to the follower stop and lowers it so the rod neck drops into the cap slot, then lets go (the shoulder catches on
    the slot edges). Left hand picks the cartridge from its rest, slides it open end first into the bore, lets go,
    then pushes its rim end home onto the mouth with closed fingers. Both arms park clear."""
    W = _demo.Waypoint
    z_ax = TOP + MAT_H + AXIS_H
    gx = GUN_X

    # right hand: T-handle pinched 25 mm beside the rod, fingers closing along y around the 12 mm bar
    tx = gx + 0.032
    ty0 = TBAR_Y
    z_t = z_ax + ROD_REST_Z + TIP_BELOW_TCP - 0.008
    ty1 = TBAR_Y - 0.243 - PULL_OVER_M  # commanded past the follower stop: the arm's servo sags ~50 mm at 14 N
    ty_ease = TBAR_Y - 0.243 + 0.010  # ease the hand forward: the spring seats the rod shoulder on the slot edges
    z_drop = z_t - 0.014
    z_hover = z_ax + 0.090

    # cartridge
    cx, cy, cz = CART_SPAWN_POS
    z_cpick = cz + TIP_BELOW_TCP - 0.006
    y_lay = MOUTH_Y + 0.006 + CART_HALF  # laid in the loading V, open end 6 mm short of the mouth
    z_lay = z_ax - TROUGH_DZ + TIP_BELOW_TCP - 0.006 + 0.003  # released 3 mm above its seat in the V
    z_carry = z_ax + 0.070
    y_end = y_lay + CART_HALF  # tab end of the laid cartridge
    y_home = MOUTH_Y - 0.001  # closed fingers pressed 3 mm past where the cartridge bears on the caught follower
    z_push = z_ax + TIP_BELOW_TCP - 0.004
    right_pull = [
        W(1.2, (tx, ty0, z_hover), FINGER_BAR, "over the T-handle"),
        W(0.8, (tx, ty0, z_t), FINGER_BAR, "fingers either side of the bar"),
        W(0.5, (tx, ty0, z_t), FINGER_CLOSED, "pinch the T-handle"),
        W(3.0, (tx, ty1, z_t), FINGER_CLOSED, "draw the rod back against the spring"),
    ]
    t_pulled = sum(w.duration for w in right_pull)
    left = [
        W(1.0, (cx, cy, z_hover), FINGER_OPEN, "over the cartridge", YAW_X),
        W(0.8, (cx, cy, z_cpick), FINGER_OPEN, "fingers either side of the cartridge", YAW_X),
        W(0.5, (cx, cy, z_cpick), FINGER_CLOSED, "grip the cartridge at mid-length", YAW_X),
        W(0.8, (cx, cy, z_carry), FINGER_CLOSED, "lift it off the rest", YAW_X),
        W(t_pulled + 0.4 - 3.1, (gx, y_lay, z_carry), FINGER_CLOSED, "carry over the loading V", YAW_X),
        W(0.8, (gx, y_lay, z_lay), FINGER_CLOSED, "lay it in the V in line with the bore", YAW_X),
        W(0.5, (gx, y_lay, z_lay), FINGER_OPEN, "let go", YAW_X),
        W(0.6, (gx, y_lay, z_carry), FINGER_OPEN, "lift", YAW_X),
        W(0.6, (gx, y_end + 0.030, z_carry), FINGER_CLOSED, "behind the tab end"),
        W(0.6, (gx, y_end + 0.030, z_push), FINGER_CLOSED, "closed fingers down behind it"),
        W(3.0, (gx, y_home, z_push), FINGER_CLOSED, "push it open end first into the bore up to the follower"),
        W(0.5, (gx, y_home + 0.020, z_push), FINGER_CLOSED, "back off"),
        W(0.6, (gx, y_home + 0.020, z_hover), FINGER_OPEN, "lift clear"),
        W(1.0, (0.140, 0.320, TOP + 0.150), FINGER_OPEN, "park"),
    ]
    t_loaded = sum(w.duration for w in left[:-1])  # cartridge pushed home and the left hand lifted clear
    right = right_pull + [
        W(t_loaded - t_pulled, (tx, ty1, z_t), FINGER_CLOSED, "hold the follower back while the cartridge goes in"),
        W(0.6, (tx, ty1, z_drop), FINGER_CLOSED, "lower: neck drops into the cap slot"),
        W(1.0, (tx, ty_ease, z_drop), FINGER_CLOSED, "ease forward: shoulder seats on the slot edges"),
        W(0.5, (tx, ty_ease, z_drop), FINGER_BAR, "let go: the caught rod holds the spring"),
        W(0.6, (tx, ty_ease - 0.020, z_hover), FINGER_OPEN, "lift clear"),
        W(1.0, (0.140, -0.320, TOP + 0.150), FINGER_OPEN, "park"),
        W(8.0, (0.140, -0.320, TOP + 0.150), FINGER_OPEN, "hold parked clear"),
    ]
    t_right = sum(w.duration for w in right)
    left.append(W(t_right - sum(w.duration for w in left), (0.140, 0.320, TOP + 0.150), FINGER_OPEN, "hold parked clear"))
    return {"left": left, "right": right}
