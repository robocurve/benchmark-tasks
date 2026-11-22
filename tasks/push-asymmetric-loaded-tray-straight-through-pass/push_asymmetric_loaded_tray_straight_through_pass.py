"""push-asymmetric-loaded-tray-straight-through-pass -- planar yaw control of a loaded cafeteria tray.

A counter worker slides a 12x16 in cafeteria tray -- a heavy glass baking dish on one side, a light plastic
tumbler on the other -- straight through the 18 in pass-through window between the prep counter and the
service counter. Because the load is off-centre, the tray's centre of friction sits ~69 mm off its geometric
midline: pushing on the midline applies an unbalanced frictional yaw torque that spins the tray crooked and
wedges it in the aperture. The skill is pushing along the offset centre of friction so the tray tracks
straight through the opening.
"""
from __future__ import annotations

import math
import pathlib
import sys

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import numpy as np

import arena_assets_fix  # noqa: F401,E402
import labkit  # noqa: E402
import pitch_demo  # noqa: E402
from labkit import Joint, PartState, on_reset, on_step, part  # noqa: E402,F401

SCENE = "station"
CHECK = SCENE

from yam_arena import demo as _demo  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402

INSTRUCTION = (
    "Push the loaded tray cleanly through the counter pass-through window onto the far side "
    "without wedging or spilling either item."
)

TOP = station.TABLE_TOP

# -- the counter slide surface --------------------------------------------------------------------------
# 0.82 x 0.90 m polypropylene service mat the tray rides on (friction vs the PP tray: dynamic ~0.44).
MAT_T = 0.005
MAT_POS = (0.41, 0.0, TOP + MAT_T / 2.0)
MAT_SIZE = (0.82, 0.90, MAT_T)
MAT_FRICTION = 0.44

# -- the tray (Cambro-style 12x16 fast-food tray, 0.359 kg, dark charcoal PP) ------------------------------
TW, TD, TH, WALL, FLR, PAD_H = 0.302, 0.410, 0.020, 0.005, 0.003, 0.002
TX0, TY0 = 0.195, 0.075               # spawn: rear_x = 0.044, front_x = 0.346 -- clear of the left arm base
TRAY_MASS, TRAY_FRICTION = 0.359, 0.44
TRAY_POS = (TX0, TY0, TOP + MAT_T)    # corner pads bottom flush on the mat top

# -- the load: asymmetric on purpose ---------------------------------------------------------------------
DISH_MASS, DISH_FRICTION = 1.45, 0.75     # 8 in square glass baking dish
DISH_E = 0.203                            # envelope edge
DISH_H = 0.064
DISH_Y = +0.095                           # local offset from tray midline -> world y = +0.170
TUM_MASS, TUM_FRICTION = 0.073, 0.65      # 12.6 oz clear tumbler
TUM_R, TUM_H = 0.038, 0.130
TUM_Y = -0.105                            # local offset -> world y = -0.030

# Centre of friction of the loaded tray (tray's own weight is symmetric, so the offset is the load's).
LOAD_CF = (DISH_MASS * DISH_Y + TUM_MASS * TUM_Y) / (DISH_MASS + TUM_MASS + TRAY_MASS)  # ~ +0.069 m
Y_CF = TY0 + LOAD_CF                  # ~ +0.144 -- the straight-push line

# -- the counter pass-through partition (Ready Access 275-style, 18 in clear aperture) --------------------
# CHUTE_X = 0.355 puts the frame face at x = 0.349: the arms' ready pose pads reach x ~ 0.337, so anything
# closer would interpenetrate at spawn.
CHUTE_X, CHUTE_Y, CHUTE_W = 0.355, 0.075, 0.4572
FRAME_T = 0.012                       # wall thickness, X in [0.349, 0.361]
WALL_H = 0.48                         # partition height above the counter
LINTEL_Z0, LINTEL_Z1 = 0.44, 0.48     # transom beam (clear of arms and the top cameras)
AP_Y0, AP_Y1 = CHUTE_Y - CHUTE_W / 2.0, CHUTE_Y + CHUTE_W / 2.0   # -0.1536 .. +0.3036
JAMB_Y0, JAMB_Y1 = -0.600, +0.600     # partition spans the whole counter -- no bypass

# -- scoring constants -----------------------------------------------------------------------------------
YAW_WEDGE = 12.0                      # wedged-in-aperture yaw cutoff (deg)
YAW_OK = 8.0                          # straight-through target
TILT_TRAY = 10.0                      # tray dumped
TILT_ITEM = 15.0                      # item spilled
TILT_ITEM_OK = 10.0
BYPASS_Y = 0.12                       # lateral drift that means a bypass attempt
FRONT_MAX = 0.800                     # far-counter delivery limit
REAR_MIN = CHUTE_X + 0.006            # rear edge fully past the frame = 0.361
PARTIAL_FRONT = CHUTE_X + 0.02        # engaged in the window = 0.375
HAND_CLEAR = 0.040                    # both grippers >= 40 mm clear

PASS_SCORE = 1.0
S_PARTIAL = 0.5
REAL_COST_USD = 12.99                 # tray $3.02 + dish $8.83 + tumbler $1.14 (counter structure/mat reused)

CHARCOAL = (0.16, 0.16, 0.17)
CHARCOAL_DARK = (0.11, 0.11, 0.12)
GLASS_AQUA = (0.76, 0.88, 0.90)
SAN_FROST = (0.72, 0.84, 0.90)
WALL_GREY = (0.30, 0.28, 0.26)
BRONZE = (0.23, 0.16, 0.10)
STEEL = (0.62, 0.64, 0.66)
MAT_BLUE = (0.13, 0.15, 0.19)


# -- objects ----------------------------------------------------------------------------------------------
def _tray():
    """Rigid tray: 4 non-overlapping corner pads + floor + 4 rims + corner ribs."""
    rim_z = PAD_H + FLR + (TH - FLR) / 2.0
    rim_h = TH - FLR
    boxes = [
        ((0.0, 0.0, PAD_H + FLR / 2.0), (TW, TD, FLR), CHARCOAL),                            # floor
        ((0.0, TD / 2.0 - WALL / 2.0, rim_z), (TW, WALL, rim_h), CHARCOAL_DARK),             # +y rim
        ((0.0, -TD / 2.0 + WALL / 2.0, rim_z), (TW, WALL, rim_h), CHARCOAL_DARK),            # -y rim
        ((TW / 2.0 - WALL / 2.0, 0.0, rim_z), (WALL, TD - 2 * WALL, rim_h), CHARCOAL_DARK),  # +x rim
        ((-TW / 2.0 + WALL / 2.0, 0.0, rim_z), (WALL, TD - 2 * WALL, rim_h), CHARCOAL_DARK), # -x rim
    ]
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            boxes.append(((sx * (TW / 2.0 - 0.025), sy * (TD / 2.0 - 0.025), PAD_H / 2.0),
                          (0.035, 0.035, PAD_H), CHARCOAL_DARK))                             # 4 corner pads
            boxes.append(((sx * (TW / 2.0 - 0.011), sy * (TD / 2.0 - 0.011), PAD_H + FLR + 0.0065),
                          (0.012, 0.012, 0.013), CHARCOAL))                                  # corner ribs
    return part("tray", TRAY_POS, boxes=boxes, mass=TRAY_MASS, friction=TRAY_FRICTION)


def _dish():
    """Hollow glass baking dish: base + 4 walls + 2 grip ears; translucent pale aqua."""
    t, h = 0.004, DISH_H
    a = DISH_E / 2.0
    boxes = [
        ((0.0, 0.0, 0.003), (DISH_E, DISH_E, 0.006), GLASS_AQUA),                             # base
        ((0.0, a - t / 2.0, h / 2.0), (DISH_E, t, h), GLASS_AQUA),
        ((0.0, -a + t / 2.0, h / 2.0), (DISH_E, t, h), GLASS_AQUA),
        ((a - t / 2.0, 0.0, h / 2.0), (t, DISH_E - 2 * t, h), GLASS_AQUA),
        ((-a + t / 2.0, 0.0, h / 2.0), (t, DISH_E - 2 * t, h), GLASS_AQUA),
        ((a + 0.006, 0.0, h - 0.014), (0.016, 0.052, 0.010), GLASS_AQUA),                     # grip ears
        ((-a - 0.006, 0.0, h - 0.014), (0.016, 0.052, 0.010), GLASS_AQUA),
    ]
    return part("heavy_dish", (TX0, TY0 + DISH_Y, TOP + MAT_T + PAD_H + FLR), boxes=boxes,
                mass=DISH_MASS, friction=DISH_FRICTION)


def _tumbler():
    """Stepped tumbler: non-overlapping base disc + tapered body + rim ring; frosted blue-clear SAN."""
    return part("tumbler", (TX0, TY0 + TUM_Y, TOP + MAT_T + PAD_H + FLR), cylinders=[
        ((0.0, 0.0, 0.004), 0.036, 0.008, SAN_FROST),                                        # base z in [0.000, 0.008]
        ((0.0, 0.0, 0.063), 0.0365, 0.110, SAN_FROST),                                       # body z in [0.008, 0.118]
        ((0.0, 0.0, TUM_H - 0.006), TUM_R, 0.012, SAN_FROST),                                # rim ring z in [0.118, 0.130]
    ], mass=TUM_MASS, friction=TUM_FRICTION)


def _pass_frame():
    """Full-width partition with an 18 in aperture, sill plate and bronze trim."""
    def jamb(yc, w):
        return ((CHUTE_X, yc, TOP + MAT_T + WALL_H / 2.0), (FRAME_T, w, WALL_H), WALL_GREY)
    boxes = [
        jamb((AP_Y1 + JAMB_Y1) / 2.0, JAMB_Y1 - AP_Y1),                                      # +y jamb
        jamb((AP_Y0 + JAMB_Y0) / 2.0, AP_Y0 - JAMB_Y0),                                      # -y jamb
        ((CHUTE_X, CHUTE_Y, TOP + MAT_T + (LINTEL_Z0 + LINTEL_Z1) / 2.0),
         (FRAME_T, CHUTE_W, LINTEL_Z1 - LINTEL_Z0), WALL_GREY),                              # transom
        ((CHUTE_X + 0.005, CHUTE_Y, TOP + MAT_T - 0.0005), (0.022, CHUTE_W, 0.001), STEEL),  # flush sill
        ((CHUTE_X - FRAME_T / 2.0 - 0.001, AP_Y0 - 0.006, TOP + MAT_T + LINTEL_Z0 / 2.0),
         (0.006, 0.012, LINTEL_Z0), BRONZE),                                               # -y trim
        ((CHUTE_X - FRAME_T / 2.0 - 0.001, AP_Y1 + 0.006, TOP + MAT_T + LINTEL_Z0 / 2.0),
         (0.006, 0.012, LINTEL_Z0), BRONZE),                                               # +y trim
        ((CHUTE_X - FRAME_T / 2.0 - 0.001, CHUTE_Y, TOP + MAT_T + LINTEL_Z0 + 0.010),
         (0.006, CHUTE_W + 0.024, 0.020), BRONZE),                                         # head trim
    ]
    return part("pass_frame", (0.0, 0.0, 0.0), boxes=boxes, fixed=True, friction=MAT_FRICTION)


OBJECTS = [
    part("slide_mat", MAT_POS, boxes=[((0.0, 0.0, 0.0), MAT_SIZE, MAT_BLUE)], fixed=True,
         friction=MAT_FRICTION),
    _pass_frame(),
    _tray(),
    _dish(),
    _tumbler(),
]

# NOTE: `part` positions each box in the part frame around `pos`, so the tray's boxes are defined around its
# origin: labkit treats `pos` as the body origin and shapes are centred on it. The constants above already
# place each shape correctly relative to the tray/dish origins.


# -- trial state ------------------------------------------------------------------------------------------
_trial: dict = {"tcps": None, "step": -1}
_ids: dict = {}


def _set_test_trial(*, tcps=None) -> None:
    """Explicit hook for offline scoring tests (never called by demo())."""
    if tcps is not None:
        _trial["tcps"] = tuple(np.asarray(p, float) for p in tcps)


@on_reset
def _reset(_env) -> None:
    _trial.update(tcps=None, step=-1)
    _ids.clear()


def _arm_tcps(env):
    """Both TCPs from arm joint positions (same FK the official action layer uses)."""
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
            out.append(np.asarray(tcp_p, float))
        return tuple(out)
    except Exception:
        return None


@on_step
def _watch(env) -> None:
    _trial["step"] += 1
    _trial["tcps"] = _arm_tcps(env)


# -- scoring helpers --------------------------------------------------------------------------------------
def _rotm(state) -> np.ndarray:
    """Rotation matrix from a state quat (works on plain ObjectState and labkit PartState alike)."""
    x, y, z, w = state.quat_xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _tilt_deg(state) -> float:
    return float(np.degrees(np.arccos(np.clip(_rotm(state)[2, 2], -1.0, 1.0))))


def _to_local(state, world) -> np.ndarray:
    return _rotm(state).T @ (np.asarray(world, float) - state.pos)


def _yaw_deg(tray: PartState) -> float:
    """Tray yaw about the vertical, from its rotation matrix."""
    r = _rotm(tray)
    return float(math.degrees(math.atan2(r[1, 0], r[0, 0])))


def _extent_x(yaw_rad: float) -> float:
    """Half-extent of the rotated tray footprint along X (the fore/aft reach)."""
    c, s = abs(math.cos(yaw_rad)), abs(math.sin(yaw_rad))
    return (TW / 2.0) * c + (TD / 2.0) * s


def _rear_x(tray: PartState) -> float:
    return float(tray.pos[0]) - _extent_x(math.radians(abs(_yaw_deg(tray))))


def _front_x(tray: PartState) -> float:
    return float(tray.pos[0]) + _extent_x(math.radians(abs(_yaw_deg(tray))))


def _in_tray(item: PartState, tray: PartState) -> bool:
    """Item seated inside the tray walls: local xy within the interior (2 cm grace) and not sunk below the
    floor."""
    l = _to_local(tray, item.pos)
    return (abs(float(l[0])) <= TW / 2.0 + 0.02 and abs(float(l[1])) <= TD / 2.0 + 0.02
            and float(l[2]) > -0.01)


def _grippers_clear(tray: PartState, dish: PartState, tum: PartState, tcps) -> bool:
    """Both TCPs outside every object's world box (+ HAND_CLEAR)."""
    if tcps is None or len(tcps) < 2:
        return False
    for t in tcps:
        t = np.asarray(t, float)
        for o in (tray, dish, tum):
            lo = np.asarray(o.low, float) - HAND_CLEAR
            hi = np.asarray(o.high, float) + HAND_CLEAR
            if np.all(t >= lo) and np.all(t <= hi):
                return False
    return True


def _at_rest(o: PartState) -> bool:
    """Still: horizontal speed < 10 mm/s, spin < 0.5 rad/s, vertical speed < 0.12 m/s (above PhysX 1-substep g*dt = 0.082 m/s)."""
    v = np.asarray(o.lin_vel, float)
    w = np.asarray(o.ang_vel, float)
    return bool(float(np.linalg.norm(v[:2])) < 0.01 and abs(float(v[2])) < 0.12 and float(np.linalg.norm(w)) < 0.5)


def components(objects: dict[str, PartState]) -> dict:
    tray, dish, tum = objects["tray"], objects["heavy_dish"], objects["tumbler"]
    yaw = _yaw_deg(tray)
    rear, front = _rear_x(tray), _front_x(tray)
    items_in = _in_tray(dish, tray) and _in_tray(tum, tray)
    items_up = _tilt_deg(dish) <= TILT_ITEM_OK and _tilt_deg(tum) <= TILT_ITEM_OK
    return {
        "yaw_deg": yaw,
        "rear_x": rear,
        "front_x": front,
        "wedged_yaw": abs(yaw) > YAW_WEDGE,
        "tray_dumped": _tilt_deg(tray) > TILT_TRAY,
        "item_spilled": not items_in or _tilt_deg(dish) > TILT_ITEM or _tilt_deg(tum) > TILT_ITEM,
        "bypassed": abs(float(tray.pos[1]) - TY0) > BYPASS_Y or front > FRONT_MAX + 0.01,
        "items_seated": items_in and items_up,
        "through": rear >= REAR_MIN and front <= FRONT_MAX,
        "straight": abs(yaw) <= YAW_OK,
        "resting": _at_rest(tray) and _at_rest(dish) and _at_rest(tum),
        "grippers_clear": _grippers_clear(tray, dish, tum, _trial["tcps"]),
    }


def score(objects: dict[str, PartState]) -> float:
    """Everything measurable with a tape measure / level / overhead photo -- no force sensing.

    0.0: tray dumped (>10 deg) or an item spilled/tipped (>15 deg) or fell off the tray; the tray skewed
         past the wedge limit (>12 deg) or drifted out of the push lane / off the far edge.
    0.5: the tray's front edge is engaged in the window (>= 0.375) but its rear edge is not yet past the
         frame (0.361), still straight (<=8 deg) with both items seated upright.
    1.0: rear edge fully past the pass-through exit (>= 0.361), front within the delivery limit
         (<= 0.800), yaw <= 8 deg, both items upright inside the tray, everything at rest, both grippers
         >= 40 mm clear of all three objects.
    """
    c = components(objects)
    if c["tray_dumped"] or c["item_spilled"] or c["wedged_yaw"] or c["bypassed"]:
        return 0.0
    if c["through"] and c["straight"] and c["items_seated"] and c["resting"] and c["grippers_clear"]:
        return 1.0
    if c["front_x"] >= PARTIAL_FRONT and c["rear_x"] < REAR_MIN and c["straight"] and c["items_seated"]:
        return S_PARTIAL
    return 0.0


def success(objects: dict[str, PartState]) -> bool:
    return score(objects) >= PASS_SCORE


def distance(objects: dict[str, PartState]) -> float:
    """Remaining straight-track distance for the tray's rear edge to clear the frame (m) plus lateral drift."""
    tray = objects["tray"]
    return max(0.0, REAR_MIN - _rear_x(tray)) + abs(float(tray.pos[1]) - TY0)


# -- scripted demo -----------------------------------------------------------------------------------------
# A 35-deg forward-pitched horizontal two-finger push (roll 0 deg, half-open -0.020 -> both jaw pads span a
# ~34 mm lateral baseline at TIP_Z_OFF = MAT_T + 0.008 = 13 mm above the table, i.e. 8 mm over the 5 mm mat
# and 14 mm below the 27 mm rim top) prevents rolling and directs contact into the tray's rear rim without
# camming over it. Pushing along PUSH_Y = 0.144 (Y_CF = TY0 + LOAD_CF) tracks straight across the friction
# band; a geometric-midline push (y = TY0 = 0.075) spins the tray +42..+44 deg and wedges it in the aperture.
PUSH_PITCH = math.radians(35.0)
PUSH_ROLL = 0.0
R_PUSH = pitch_demo.rotation(yaw=0.0, pitch=PUSH_PITCH, roll=PUSH_ROLL)
PUSH_FINGER = -0.020                  # symmetric two-pad horizontal baseline against rear rim
TIP_Z_OFF = MAT_T + 0.008             # pad tips 8 mm over the 5 mm mat -> solid mid-rim contact
PX0 = 0.026                           # approach cleanly behind the 0.044 rear rim face (even with +-5 mm jitter)
PX1 = 0.485                           # push end: rear edge ~0.44..0.46 past 0.361, front ~0.76 <= 0.800
PUSH_Y = 0.144                        # the straight-push line at theoretical Y_CF = 0.144


def _push_tcp(x: float, y: float, z_off: float) -> tuple[float, float, float]:
    tip = np.array([x, y, TOP + z_off], dtype=float)
    return tuple(float(v) for v in (tip - R_PUSH[:, 2] * PAD_TIP_PAST_TCP))


def demo():
    """Left arm pushes the tray's rear rim along the loaded tray's centre-of-friction line; the right arm stays
    parked off the counter. Waypoints are constants -- demo() reads no simulator state."""
    W = pitch_demo.Waypoint
    left = [
        W(2.2, _push_tcp(PX0, PUSH_Y, TIP_Z_OFF + 0.08), PUSH_FINGER, "above rear rim", rot=R_PUSH),
        W(1.0, _push_tcp(PX0, PUSH_Y, TIP_Z_OFF), PUSH_FINGER, "down to rim height", rot=R_PUSH),
        W(0.6, _push_tcp(0.044, PUSH_Y, TIP_Z_OFF), PUSH_FINGER, "touch rear rim", rot=R_PUSH),
        W(1.8, _push_tcp(0.180, PUSH_Y, TIP_Z_OFF), PUSH_FINGER, "push segment 1", rot=R_PUSH),
        W(1.8, _push_tcp(0.320, PUSH_Y, TIP_Z_OFF), PUSH_FINGER, "push segment 2", rot=R_PUSH),
        W(2.0, _push_tcp(PX1, PUSH_Y, TIP_Z_OFF), PUSH_FINGER, "push segment 3", rot=R_PUSH),
        W(0.5, _push_tcp(PX1 - 0.06, PUSH_Y, TIP_Z_OFF), PUSH_FINGER, "back off rim", rot=R_PUSH),
        W(0.8, (0.300, 0.340, TOP + 0.14), PUSH_FINGER, "swing outboard clear", yaw=0.0),
        W(0.8, (0.150, 0.340, TOP + 0.14), PUSH_FINGER, "park outboard", yaw=0.0),
    ]
    right = [
        W(1.5, (0.20, -0.35, TOP + 0.14), FINGER_OPEN, "park right", yaw=0.0),
    ]
    return {"left": left, "right": right}
