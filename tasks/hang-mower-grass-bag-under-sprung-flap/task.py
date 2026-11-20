"""Fit the rear grass bag of a 14 in corded electric push mower under its spring-loaded ejector flap.

A 14 in corded electric walk-behind mower (Sun Joe MJ401E class: 16.5 in wide, 29 lb) stands on the bench on its four
wheels, rear toward the robots, handle folded forward over the deck. A spring-loaded ejector flap hangs over the rear
discharge opening; two coupling hooks stand out from the rear of the mower body under the flap's hinge. The empty
rear grass bag (hard top with a carry handle, fabric bag on a wire frame, 10.6 gal; one rigid body here) lies on the
bench between the arms, mouth toward the mower. A steel support rod runs across the top of the bag's mouth.

The bag is fitted when its support rod rests in both coupling hooks, nobody holds it, and the ejector flap has come
down to rest on it, as in the mower's manual: raise the ejector flap, set the bag over the coupling hooks, release
the flap. The hooks hold the bag only by contact: the rod sits in each hook's open-topped slot under gravity.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from robokit import _ENV, Slider, _part_state, on_reset, on_step, part, set_target, write_pose  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Fit the grass bag to the back of the mower, ready to mow, with the ejector flap resting on it."

GREEN, DARK, BLACK, STEEL, GREY, FABRIC = ((0.16, 0.55, 0.20), (0.20, 0.21, 0.22), (0.05, 0.05, 0.05),
                                           (0.55, 0.56, 0.58), (0.35, 0.36, 0.37), (0.12, 0.13, 0.12))

# ---- mower (frame: origin on the table under the centre of the rear face, +x toward the front of the mower) -------
XF = 0.43                 # rear face, world x
R_HOOK = 0.33            # centre of the bag's rod when it sits in the hooks, above the wheel contact plane (estimate)
HZ = R_HOOK + 0.06        # ejector flap hinge axis (estimate)
HINGE_X = -0.042
HOOK_Y = 0.08             # coupling hooks at y +-8 cm (estimate)
SLOT = (-0.021, 0.0)      # each hook's slot between its outer lip and the rear face: 21 mm wide, open at the top
FLAP_L, FLAP_W, FLAP_T = 0.15, 0.24, 0.004
LIP = 0.02
MOWER_MASS = 13.2         # 29 lb assembled
WHEEL_R = 0.089           # 7 in wheels (estimate)
HALF_W = 0.2095           # 16.5 in overall width


def _hook(s):
    y = s * HOOK_Y
    zb = R_HOOK - 0.005                                                   # slot floor under the 10 mm rod
    return [
        ((-0.0125, y, zb - 0.002), (0.025, 0.016, 0.004), STEEL),          # floor plate standing out from the body
        ((-0.023, y, zb + 0.005), (0.004, 0.016, 0.018), STEEL),           # outer lip
        ((-0.0285, y, zb + 0.0185), (0.004, 0.016, 0.014), STEEL, ("Y", -40.0)),  # lead-in, flared outward
    ]


MOWER_BOXES = [
    ((0.23, 0.0, 0.17), (0.42, 0.33, 0.22), GREEN),                       # deck
    ((0.27, 0.0, 0.33), (0.24, 0.24, 0.10), GREEN),                       # motor housing
    ((0.02, 0.0, (0.06 + HZ + 0.02) / 2), (0.04, 0.26, HZ + 0.02 - 0.06), GREEN),   # rear discharge housing
    ((-0.0175, 0.125, HZ), (0.035, 0.012, 0.016), DARK),                  # flap hinge brackets
    ((-0.0175, -0.125, HZ), (0.035, 0.012, 0.016), DARK),
    ((-0.0005, 0.0, R_HOOK - 0.11), (0.001, 0.20, 0.16), BLACK, None, None, "visual"),  # discharge opening
    *_hook(1), *_hook(-1),
]
MOWER_CYL = [
    ((0.07, HALF_W - 0.0225, WHEEL_R), WHEEL_R, 0.045, BLACK, "Y"),
    ((0.07, -HALF_W + 0.0225, WHEEL_R), WHEEL_R, 0.045, BLACK, "Y"),
    ((0.36, HALF_W - 0.0225, WHEEL_R), WHEEL_R, 0.045, BLACK, "Y"),
    ((0.36, -HALF_W + 0.0225, WHEEL_R), WHEEL_R, 0.045, BLACK, "Y"),
    ((0.25, 0.17, 0.40), 0.0125, 0.40, BLACK, "X"),                       # handle, folded forward over the deck
    ((0.25, -0.17, 0.40), 0.0125, 0.40, BLACK, "X"),
]
FLAP_MAX = math.radians(125)
FLAP_RATE = 0.25          # N m/rad (estimate)
FLAP_PRELOAD = 0.6        # rad of wind-up when closed: 0.15 N m closing torque at rest (estimate)
FLAP = Slider("flap", rest=(HINGE_X, 0.0, HZ), kind="revolute", axis="Y", sign=1, travel=FLAP_MAX,
              rate=FLAP_RATE, damping=0.02, mass=0.15,
              boxes=(((HINGE_X, 0.0, HZ - FLAP_L / 2), (FLAP_T, FLAP_W, FLAP_L), DARK),
                     ((HINGE_X - LIP / 2, 0.0, HZ - FLAP_L + FLAP_T / 2), (LIP, FLAP_W, FLAP_T), DARK)),
              cylinders=(((HINGE_X, 0.0, HZ), 0.005, FLAP_W, DARK, "Y"),))
MOWER = part("mower", pos=(XF, 0.0, TOP), mass=MOWER_MASS, color=GREEN, friction=1.0, boxes=MOWER_BOXES,
             cylinders=MOWER_CYL, sliders=(FLAP,))

# ---- grass bag (frame: origin at the centre of its support rod; +x toward the mower) ---------------------------
# 10.6 gal (40 l): 38 x 36 x 29 cm body (estimate from the capacity), 1.0 kg (estimate).
BAG_L, BAG_W, BAG_H = 0.38, 0.36, 0.29
BAG_MASS = 1.0
ROD_R, ROD_LEN = 0.005, 0.30
X_BAG = -0.032                           # front of the bag body: the rod stands 32 mm ahead of it on two arms
X_BACK = X_BAG - BAG_L
Z_TOP, Z_BOT = 0.006, -0.284            # top of the hard top and bottom of the bag
Z_MID = (Z_TOP + Z_BOT) / 2
_xm = (X_BAG + X_BACK) / 2
_BODY = [
    ((_xm, 0.0, Z_TOP - 0.004), (BAG_L, BAG_W, 0.008), DARK),            # hard top
    ((_xm, 0.0, Z_BOT + 0.002), (BAG_L, BAG_W, 0.004), FABRIC),          # bag: bottom, sides, back
    ((_xm, BAG_W / 2 - 0.002, Z_MID - 0.004), (BAG_L, 0.004, BAG_H - 0.008), FABRIC),
    ((_xm, -BAG_W / 2 + 0.002, Z_MID - 0.004), (BAG_L, 0.004, BAG_H - 0.008), FABRIC),
    ((X_BACK + 0.002, 0.0, Z_MID - 0.004), (0.004, BAG_W, BAG_H - 0.008), FABRIC),
    ((X_BAG - 0.004, 0.0, Z_BOT + 0.006), (0.008, BAG_W, 0.012), GREY),  # mouth frame, bottom and sides
    ((X_BAG - 0.004, BAG_W / 2 - 0.006, Z_MID), (0.008, 0.012, BAG_H - 0.012), GREY),
    ((X_BAG - 0.004, -BAG_W / 2 + 0.006, Z_MID), (0.008, 0.012, BAG_H - 0.012), GREY),
    ((X_BAG / 2, 0.14, 0.0), (-X_BAG, 0.010, 0.010), GREY),             # the rod's two arms
    ((X_BAG / 2, -0.14, 0.0), (-X_BAG, 0.010, 0.010), GREY),
]
_ROD = [((0.0, 0.0, 0.0), ROD_R, ROD_LEN, STEEL, "Y")]


def _com(boxes, cyls):
    v = [np.prod(b[1]) for b in boxes] + [math.pi * c[1] ** 2 * c[2] for c in cyls]
    c = [np.array(b[0]) for b in boxes] + [np.array(c[0]) for c in cyls]
    return np.average(np.array(c), axis=0, weights=np.array(v))


HANDLE_Z = 0.085                         # carry handle on the hard top: 16 mm bar along the bag, 9 cm long, 8 cm up
HANDLE_X = float(_com(_BODY, _ROD)[0])
for _ in range(3):                       # it sits over the centre of mass
    _h = [((HANDLE_X + 0.035, 0.0, 0.0415), (0.012, 0.014, 0.071), DARK),
          ((HANDLE_X - 0.035, 0.0, 0.0415), (0.012, 0.014, 0.071), DARK)]
    _c = _ROD + [((HANDLE_X, 0.0, HANDLE_Z), 0.008, 0.09, GREY, "X")]
    HANDLE_X = float(_com(_BODY + _h, _c)[0])
BAG_BOXES = _BODY + _h
BAG_CYL = _c

# start: lying on the bench between the arms behind the mower, square to it, rod 17.5 cm short of the hooks
SEAT_X = XF + sum(SLOT) / 2              # world x of the rod centre when it sits in the hooks
START_X = SEAT_X - 0.175
START_Y = 0.0
START_Z = TOP - Z_BOT + 0.001
BAG = part("bag", pos=(START_X, START_Y, START_Z), mass=BAG_MASS, color=FABRIC, friction=0.8,
           boxes=BAG_BOXES, cylinders=BAG_CYL)
OBJECTS = [MOWER, BAG]

# ---- scoring ------------------------------------------------------------------------------------------------------
SEAT_TOL_X = (SLOT[0] + ROD_R - 0.0005, SLOT[1] - ROD_R + 0.0005)   # rod centre inside the slot (mower frame x)
SEAT_TOL_Z = (R_HOOK - 0.002, R_HOOK + 0.004)                       # rod resting on the slot floor
FLAP_REST = (math.radians(40), math.radians(88))                    # flap lying on the bag's hard top
HANDS_CLEAR = 0.06
MOWER_TIP = math.radians(15)            # safety gate: a wheel off the bench (mower tipping)
MOWER_END = 0.05                         # final placement: mower within 5 cm of where it started
HOOK_HALF = 0.008                        # each hook (outer lip) is 16 mm wide along the rod
_STATE: dict = {}


@on_reset
def _reset(environment=None) -> None:
    _STATE.update(m0=None, gate=None, tcp={}, flap=0.0, max_flap=0.0, start=None)
    if environment is None:
        return
    set_target(environment, "mower", "flap", -FLAP_PRELOAD)
    seed = _ENV.get("seed")
    if seed is None:
        return
    # where the bag was set down varies per trial: 0-6 mm away from the mower, +-1 cm across, +-2 deg (seeded).
    # The ranges keep it clear of both parked grippers, the camera stand and the flap's swing.
    rng = np.random.default_rng(int(seed) + 7919)
    dx, dy, yaw = rng.uniform(-0.006, 0.0), rng.uniform(-0.01, 0.01), math.radians(rng.uniform(-2.0, 2.0))
    _STATE["start"] = (round(dx, 4), round(dy, 4), round(math.degrees(yaw), 2))
    c, s = math.cos(yaw), math.sin(yaw)       # turn about the handle
    hx, hy = START_X + HANDLE_X, START_Y
    ox, oy = hx - c * HANDLE_X, hy - s * HANDLE_X
    write_pose(environment, "bag", (ox + dx, oy + dy, START_Z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))


_reset()


def _tcps(environment):
    from yam_arena import ik, station

    base = environment.unwrapped
    out = {}
    for side in ("left", "right"):
        rob = base.scene[f"{side}_robot"]
        q = np.asarray(getattr(rob.data.joint_pos, "torch", rob.data.joint_pos)[0].detach().cpu())[:6]
        out[side] = ik.tcp_pose(q, station.BASES[side])[0]
    return out


@on_step
def _watch(environment) -> None:
    mower = _part_state(environment, MOWER)
    bag = _part_state(environment, BAG)
    if _STATE["m0"] is None:
        _STATE["m0"] = mower.pos.copy()
    if mower.rotation()[2, 2] < math.cos(MOWER_TIP):
        _STATE["gate"] = _STATE["gate"] or "mower tipped"
    if bag.pos[2] < TOP - 0.05:
        _STATE["gate"] = _STATE["gate"] or "bag fell off the bench"
    if bag.rotation()[2, 2] < math.cos(math.radians(60)):
        _STATE["gate"] = _STATE["gate"] or "bag tipped over"
    set_target(environment, "mower", "flap", -FLAP_PRELOAD)
    _STATE["tcp"] = _tcps(environment)
    _STATE["flap"] = mower.sliders["flap"]
    _STATE["max_flap"] = max(_STATE["max_flap"], mower.sliders["flap"])


def rod_in_hooks(objects) -> dict:
    """Where the bag's finite rod crosses each hook's plane: (x, z) in the mower frame and u, the crossing's position
    along the rod from its centre (metres). The rod is 30 cm long; a hook counts only if the rod itself spans it."""
    mower, bag = objects["mower"], objects["bag"]
    out = {}
    a = mower.to_local(bag.to_world((0.0, -ROD_LEN / 2, 0.0)))   # the rod's two ends
    b = mower.to_local(bag.to_world((0.0, ROD_LEN / 2, 0.0)))
    for s in (1, -1):
        dy = b[1] - a[1]
        t = (s * HOOK_Y - a[1]) / dy if abs(dy) > 1e-6 else -1.0
        p = a + t * (b - a)
        out["L" if s > 0 else "R"] = (float(p[0]), float(p[2]), float((t - 0.5) * ROD_LEN))
    return out


def seated(objects) -> bool:
    """The rod rests in both hooks: the rod itself spans each hook's whole 16 mm width, and where it crosses the hook
    its axis is inside the 21 mm slot and on the slot floor."""
    return all(SEAT_TOL_X[0] <= x <= SEAT_TOL_X[1] and SEAT_TOL_Z[0] <= z <= SEAT_TOL_Z[1]
               and abs(u) <= ROD_LEN / 2 - HOOK_HALF for x, z, u in rod_in_hooks(objects).values())


def _near_bag(bag, t) -> bool:
    p = bag.to_local(t)
    return bool(X_BACK - 0.03 <= p[0] <= 0.04 and abs(p[1]) <= BAG_W / 2 + 0.03 and Z_BOT - 0.03 <= p[2] <= HANDLE_Z + 0.008 + 0.03)


def _near_flap(mower, t) -> bool:
    phi = mower.sliders["flap"]
    hinge = mower.to_world((HINGE_X, 0.0, HZ))
    d = mower.rotation() @ np.array([-math.sin(phi), 0.0, -math.cos(phi)])
    r = np.asarray(t) - hinge
    s = float(np.clip(r @ d, 0.0, FLAP_L))
    return abs(r[1]) < FLAP_W / 2 + HANDS_CLEAR and float(np.linalg.norm((r - s * d)[[0, 2]])) < HANDS_CLEAR


def _update(objects):
    if _STATE.get("gate"):
        return 0.0, False
    bag, mower = objects["bag"], objects["mower"]
    tcp = list((_STATE.get("tcp") or {}).values())
    still = float(np.linalg.norm(bag.lin_vel)) < 0.01 and float(np.linalg.norm(bag.ang_vel)) < 0.05
    if not (seated(objects) and still) or any(_near_bag(bag, t) for t in tcp):
        return 0.0, False
    if _STATE.get("m0") is not None and float(np.linalg.norm((mower.pos - _STATE["m0"])[:2])) > MOWER_END:
        return 0.0, False
    phi = mower.sliders["flap"]
    if FLAP_REST[0] <= phi <= FLAP_REST[1] and not any(_near_flap(mower, t) for t in tcp):
        return 1.0, True
    return 0.5, False


def success(objects) -> bool:
    return _update(objects)[1]


def score(objects) -> float:
    """0 after a gate (mower tipped past 15 deg, bag off the bench or tipped past 60 deg). Otherwise, at the end, with
    the mower within 5 cm of where it started: 0.5 when the bag's 30 cm rod rests in both coupling hooks (the rod itself
    spans each hook's 16 mm width; there its axis is inside the 21 mm slot, within 2 mm below to 4 mm above its resting
    height on the slot floor), the bag is still and no fingertip is within 3 cm of it; 1.0 (success, held 1 s)
    when, in addition, the ejector flap rests between 40 and 88 deg on the bag and no fingertip is within 6 cm of the
    flap. Anything else scores 0."""
    return _update(objects)[0]


def report(objects) -> dict:
    return {"start_offset": _STATE.get("start"),
            "rod_in_hooks_mm": {k: [round(v * 1000, 1) for v in xz] for k, xz in rod_in_hooks(objects).items()},
            "mower_shift_mm": (round(float(np.linalg.norm((objects["mower"].pos - _STATE["m0"])[:2])) * 1000, 1)
                               if _STATE.get("m0") is not None else None),
            "seated": seated(objects), "flap_deg": round(math.degrees(_STATE.get("flap", 0.0)), 1),
            "max_flap_deg": round(math.degrees(_STATE.get("max_flap", 0.0)), 1), "gate": _STATE.get("gate"),
            "score": _update(objects)[0], "bag_pos_mm": [round(float(v) * 1000, 1) for v in objects["bag"].pos],
            "mower_pos_mm": [round(float(v) * 1000, 1) for v in objects["mower"].pos]}


def debug_line(objects) -> str:
    h = rod_in_hooks(objects)
    b = objects["bag"]
    return (f"flap {math.degrees(_STATE.get('flap', 0)):.1f} rod " +
            " ".join(f"{k}:({x*1000:.1f},{z*1000:.1f},u{u*1000:.0f})" for k, (x, z, u) in h.items()) +
            f" bag {np.round(b.pos, 3)} v {np.round(b.lin_vel, 3)} tilt {math.degrees(math.acos(min(1.0, b.rotation()[2, 2]))):.1f}"
            f" gate {_STATE.get('gate')} score {_update(objects)[0]}")
