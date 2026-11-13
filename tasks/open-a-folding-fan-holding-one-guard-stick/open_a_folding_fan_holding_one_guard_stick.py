"""On the station table stands a low round display stand topped with burgundy felt. On it lies a lacquered folding
fan, half open: two black guard sticks with gilt tips and six bamboo ribs between them, all on one brass rivet,
with a painted paper leaf pleated across the ribs.

Task: open the fan fully. The fan's rivet is stiff (the ribs stay where they are left), and the fan is light: pushed
round by one guard stick alone, the whole fan turns on the felt instead of opening. So one hand holds one guard stick
still by its tip while the other carries the other guard stick round the rivet until the leaf is spread wide. Then
both hands let go, the fan lying open on its stand, and both arms go back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=open_a_folding_fan_holding_one_guard_stick.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge, on_reset,  # noqa: E402
                     on_step, rotation)
from bodykit import set_hinge_drive  # noqa: E402
from yam_arena.constants import FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the display stand: a round beech stand 25 cm across, 3 cm tall, topped with felt; fixed --------------------------
RC = np.array([0.20, 0.0])  # the stand's middle, and the fan's rivet over it
STAND_R, STAND_H = 0.125, 0.030
S_TOP = TOP + STAND_H

# -- the fan: 8 sticks on one rivet (guard A, six ribs, guard B), from guard A at +78 degrees round to guard B -------
N_STICKS = 8
A_DEG = 78.0  # guard A's direction (from +x, counter-clockwise seen from above)
START_OPEN = 75.0  # the fan lies half open: guard B at A_DEG - 75
FULL_OPEN = 150.0  # the leaf spread wide: each fold opens to 150/7 degrees
GAP_START, GAP_FULL = START_OPEN / (N_STICKS - 1), FULL_OPEN / (N_STICKS - 1)
OPEN_ENOUGH = 135.0  # counts as fully open
HALF_OPEN = 110.0
GUARD_L, GUARD_W, GUARD_T = 0.175, 0.013, 0.006  # guard sticks: from the rivet out to their gilt tips
RIB_L0, RIB_L1, RIB_W, RIB_T = 0.010, 0.150, 0.007, 0.003  # ribs: from 1 cm out to 15 cm
LEAF_R0, LEAF_R1 = 0.070, 0.150  # the paper leaf, from 7 to 15 cm out
GUARD_MASS, RIB_MASS = 0.010, 0.004
# the rivet's stiffness: each fold has a spring (N m per degree) whose rest angle moves to wherever the fold is pushed
# once it has been pushed SLIP degrees past it, so a fold stays where it is left and resists 0.04 N m
FOLD_SPRING, SLIP = 0.04, 1.0

LACQUER, GILT, BAMBOO, BRASS, FELT, BEECH = (0.08, 0.07, 0.07), (0.85, 0.68, 0.25), (0.80, 0.66, 0.42), \
    (0.78, 0.62, 0.26), (0.42, 0.08, 0.12), (0.72, 0.55, 0.36)
LEAF_COLOURS = [(0.80, 0.20, 0.22), (0.95, 0.85, 0.62)]  # crimson and cream pleats
BLOSSOM, BRANCH = (0.97, 0.62, 0.70), (0.30, 0.20, 0.15)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0
STICKS = ["guard_a", *[f"rib_{k}" for k in range(1, N_STICKS - 1)], "guard_b"]


def _dir(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([math.cos(a), math.sin(a)])


def _start_deg(i: int) -> float:
    return A_DEG - i * GAP_START


def _stand() -> Body:
    shapes = [CylShape((0.0, 0.0, STAND_H / 2 - 0.002), STAND_R, STAND_H - 0.004, BEECH),
              CylShape((0.0, 0.0, STAND_H - 0.002), STAND_R - 0.004, 0.004, FELT),
              CylShape((0.0, 0.0, 0.004), STAND_R + 0.006, 0.008, (0.55, 0.40, 0.25), collide=False)]
    return Body.build("fan_stand", pos=(RC[0], RC[1], TOP), mass=1.0, shapes=shapes, color=BEECH, kinematic=True,
                      friction=(0.5, 0.4))


def _stick(i: int) -> Body:
    """Stick ``i`` (0 = guard A ... 7 = guard B): its body origin on the rivet, unrotated; its shapes laid out along its
    starting direction. Each stick but guard B carries the half of the leaf toward guard B, each but guard A the half
    toward guard A, so the leaf tiles exactly when the fan is fully open."""
    name = STICKS[i]
    deg = _start_deg(i)
    d = _dir(deg)
    yaw = math.radians(deg)
    guard = i in (0, N_STICKS - 1)
    t = GUARD_T if guard else RIB_T
    shapes = []
    if guard:
        mid = (GUARD_L - 0.02) / 2  # from 2 cm behind the rivet (the handle end) to the tip
        shapes.append(BoxShape((d[0] * mid, d[1] * mid, t / 2), (GUARD_L + 0.02, GUARD_W, t), LACQUER, yaw=yaw))
        tip = GUARD_L - 0.012
        shapes.append(BoxShape((d[0] * tip, d[1] * tip, t + 0.0003), (0.022, GUARD_W * 0.9, 0.0005), GILT, yaw=yaw,
                               collide=False))
        shapes.append(BoxShape((d[0] * 0.035, d[1] * 0.035, t + 0.0003), (0.03, GUARD_W * 0.6, 0.0005), GILT, yaw=yaw,
                               collide=False))
    else:
        mid = (RIB_L0 + RIB_L1) / 2
        shapes.append(BoxShape((d[0] * mid, d[1] * mid, t / 2), (RIB_L1 - RIB_L0, RIB_W, t), BAMBOO, yaw=yaw))
    z_leaf = RIB_T + 0.0004 + 0.0002 * i
    colour = LEAF_COLOURS[i % 2]
    for side, ok in ((-1.0, i < N_STICKS - 1), (1.0, i > 0)):  # -1: toward guard B (clockwise), +1: toward guard A
        if not ok:
            continue
        half = GAP_FULL / 2
        for r0, r1 in ((LEAF_R0, (LEAF_R0 + LEAF_R1) / 2), ((LEAF_R0 + LEAF_R1) / 2, LEAF_R1)):
            rm = (r0 + r1) / 2
            w = rm * math.radians(half)  # the panel's width at its middle
            at = _dir(deg + side * half / 2) * rm
            shapes.append(BoxShape((at[0], at[1], z_leaf), (r1 - r0, w * 1.04, 0.0006), colour,
                                   yaw=math.radians(deg + side * half / 2), collide=False))
    if i in (2, 3, 4):  # a painted plum branch with blossoms across the middle of the leaf (seen only)
        at = _dir(deg - GAP_FULL / 4) * 0.115
        shapes.append(SphereShape((at[0], at[1], z_leaf + 0.0006), 0.006, BLOSSOM, collide=False))
        at = _dir(deg - GAP_FULL / 4) * 0.095
        shapes.append(BoxShape((at[0], at[1], z_leaf + 0.0004), (0.03, 0.003, 0.0004), BRANCH,
                               yaw=math.radians(deg + 70.0), collide=False))
    if i == 0:  # the rivet (seen only) and a red tassel cord at the handle end
        shapes.append(CylShape((0.0, 0.0, GUARD_T + 0.004), 0.0055, 0.004, BRASS, collide=False))
        back = -d * 0.024
        shapes.append(SphereShape((back[0], back[1], 0.004), 0.005, (0.75, 0.10, 0.12), collide=False))
    r_com = (GUARD_L - 0.02) / 2 if guard else 0.08
    return Body.build(name, pos=(RC[0], RC[1], S_TOP), mass=GUARD_MASS if guard else RIB_MASS, shapes=shapes,
                      color=LACQUER if guard else BAMBOO, friction=(0.45, 0.4), damping=(0.5, 2.0),
                      com=(float(d[0] * r_com), float(d[1] * r_com), t / 2), ignore=tuple(STICKS[:i]))


OBJECTS = [_stand(), *[_stick(i) for i in range(N_STICKS)]]
FOLDS = []
for _i in range(1, N_STICKS):
    # each fold: 0 as it lies; opening turns the stick clockwise (negative about +z) against its spring, up to the full
    # spread; closing up to the sticks lying together
    FOLDS.append(hinge(name=f"fold_{_i}", body0=STICKS[_i - 1], body1=STICKS[_i], anchor=(RC[0], RC[1], S_TOP + 0.002),
                       axis="Z", limits_deg=(-(GAP_FULL - GAP_START), GAP_START), spring=FOLD_SPRING,
                       damping=FOLD_SPRING * 0.02, target_deg=0.0).name)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"tcps": None, "s": {}, "targets": [0.0] * len(FOLDS)}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("FAN_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _yaw(r: np.ndarray) -> float:
    return math.degrees(math.atan2(float(r[1, 0]), float(r[0, 0])))


def _fan_state(base, origin) -> dict:
    poses = [_pose(base, origin, n) for n in STICKS]
    folds = []
    for k in range(1, N_STICKS):
        rel = poses[k - 1][1].T @ poses[k][1]
        folds.append(math.degrees(math.atan2(float(rel[1, 0]), float(rel[0, 0]))))
    da = poses[0][1] @ np.array([*_dir(_start_deg(0)), 0.0])
    db = poses[-1][1] @ np.array([*_dir(_start_deg(N_STICKS - 1)), 0.0])
    da2, db2 = da[:2] / max(np.linalg.norm(da[:2]), 1e-9), db[:2] / max(np.linalg.norm(db[:2]), 1e-9)
    spread = math.degrees(math.acos(max(-1.0, min(1.0, float(da2 @ db2)))))
    rivet = poses[0][0]
    on_stand = (float(np.linalg.norm(rivet[:2] - RC)) <= STAND_R - 0.01 and rivet[2] <= S_TOP + 0.012 and
                all(float(r[2, 2]) > math.cos(math.radians(15.0)) for _, r in poses))
    return {"folds": folds, "spread": spread, "rivet": rivet, "on_stand": on_stand}


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    st = _fan_state(base, origin)
    # the stiff rivet: a fold pushed more than SLIP degrees past its spring's rest angle has given; it rests there now
    for k, (name, ang) in enumerate(zip(FOLDS, st["folds"])):
        if abs(ang - _EP["targets"][k]) > SLIP:
            _EP["targets"][k] = ang
            set_hinge_drive(name, target_deg=ang)
    _EP["tcps"] = tcps = _tcps(base, origin)
    _EP["s"] = st
    _DEBUG["steps"] += 1
    if _DEBUG["on"] and _DEBUG["steps"] % 10 == 0:
        tl, tr = tcps
        print(f"[fan] t={_DEBUG['steps'] / 20:5.1f}s spread={st['spread']:6.1f} folds="
              f"{[round(f, 1) for f in st['folds']]} rivet={np.round((st['rivet'][:2] - RC) * 1000).astype(int).tolist()}"
              f" z={(st['rivet'][2] - S_TOP) * 1000:5.1f} on_stand={st['on_stand']} home={_arms_home()} "
              f"L={np.round(tl * 1000).astype(int).tolist()} R={np.round(tr * 1000).astype(int).tolist()} "
              f"score={_points(_status())}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    spread = float(st.get("spread", 0.0))
    return {"open": spread >= OPEN_ENOUGH, "half": spread >= HALF_OPEN, "on_stand": bool(st.get("on_stand")),
            "home": _arms_home()}


def _points(s) -> float:
    if s["open"] and s["on_stand"] and s["home"]:
        return 1.0
    if s["open"] and s["on_stand"]:
        return 0.5
    return 0.25 if s["half"] else 0.0


def score(objects) -> float:
    """Judged at the end. The fan's spread is the angle between its two guard sticks (75 degrees as it lies, 150 at
    most). It lies on its stand when its rivet is on the stand's top (within 11.5 cm of the stand's middle, at most
    1.2 cm above the felt) and no stick is tilted more than 15 degrees. 1.0 with the fan spread at least 135 degrees,
    lying on its stand, and both arms back where they started (each TCP within 5 cm of its start); 0.5 with the fan
    spread at least 135 degrees and lying on its stand; 0.25 with it spread at least 110 degrees; else 0.0."""
    return _points(_status())


_HOME: dict = {}


def _arms_home() -> bool:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def success(objects) -> bool:
    """Full score with every stick of the fan still."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in STICKS)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on guard A's tip, beyond the stand's edge, fingers across the stick, and pinches
# it. The right does the same on guard B's tip and carries it round the rivet, turning with it, 78 degrees clockwise:
# the folds give one after another and the leaf spreads. The right lets go and rises, then the left; both arms go back
# to where they started.

PINCH_R = 0.167  # where along the guard sticks they are pinched: 4 cm beyond the stand's edge
PINCH_Z = S_TOP + 0.006  # the TCP level with the guard stick's top: the fingertips reach just below it
STICK_OPEN = -0.016  # about 3 cm between the fingers round the 1.3 cm stick
PINCH = -0.001  # a firm pinch
HOVER = 0.05  # above the tips before coming down (the right reaches no higher out there)
ARC_STEP, ARC_S = 5.0, 0.30
OVERSHOOT = 3.0
LEAD_S, LEAD_STEPS = 3.0, 15


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = _good_ik(arm, pos, rot), np.array(READY, float)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, "back to where it started", r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _at(deg: float, z: float) -> list[float]:
    p = RC + _dir(deg) * PINCH_R
    return [float(p[0]), float(p[1]), z]


def demo():
    a_deg, b0 = A_DEG, _start_deg(N_STICKS - 1)
    b1 = A_DEG - FULL_OPEN - OVERSHOOT
    la = grip_rotation(math.radians(a_deg))
    left = _lead_in("left", _at(a_deg, PINCH_Z + HOVER), la, "high over guard A's tip", STICK_OPEN)
    left += [_w(1.4, _at(a_deg, PINCH_Z), STICK_OPEN, "straight down round the tip, beyond the stand", la),
             _w(0.4, _at(a_deg, PINCH_Z), STICK_OPEN, "settle", la),
             _w(0.6, _at(a_deg, PINCH_Z), PINCH, "pinch guard A's tip", la)]
    rb = grip_rotation(math.radians(b0))
    right = _lead_in("right", _at(b0, PINCH_Z + HOVER), rb, "high over guard B's tip", STICK_OPEN)
    right += [_w(1.4, _at(b0, PINCH_Z), STICK_OPEN, "straight down round the tip", rb),
              _w(0.4, _at(b0, PINCH_Z), STICK_OPEN, "settle", rb),
              _w(0.6, _at(b0, PINCH_Z), PINCH, "pinch guard B's tip", rb)]
    t_l, t_r = sum(w.duration for w in left), sum(w.duration for w in right)
    if t_l + 0.3 > t_r:
        right.insert(len(right) - 1, _w(t_l + 0.3 - t_r, _at(b0, PINCH_Z), STICK_OPEN, "wait for guard A to be held",
                                        rb))
    deg = b0
    while deg > b1 + 1e-6:
        deg = max(deg - ARC_STEP, b1)
        right.append(_w(ARC_S, _at(deg, PINCH_Z), PINCH, "carry guard B round the rivet: the leaf spreads",
                        grip_rotation(math.radians(deg))))
    rf = grip_rotation(math.radians(b1))
    right += [_w(0.6, right[-1].pos, PINCH, "hold", rf),
              _w(0.6, right[-1].pos, STICK_OPEN, "let go", rf),
              _w(0.8, _at(b1, PINCH_Z + HOVER), STICK_OPEN, "rise", rf)]
    t_up = sum(w.duration for w in right)
    left.append(_w(max(t_up - sum(w.duration for w in left), 0.3), _at(a_deg, PINCH_Z), PINCH,
                   "hold guard A still while the fan opens", la))
    left += [_w(0.6, _at(a_deg, PINCH_Z), STICK_OPEN, "let go", la),
             _w(0.8, _at(a_deg, PINCH_Z + HOVER), STICK_OPEN, "rise", la)]
    left += _go_home("left", left[-1].pos, la, FINGER_OPEN)
    right += _go_home("right", right[-1].pos, rf, FINGER_OPEN)
    total = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 3.0
    for arm in (left, right):
        arm.append(_w(max(total - sum(w.duration for w in arm), 0.5), arm[-1].pos, FINGER_OPEN,
                      "hold still: the trial ends once the full score has held 1 s", arm[-1].yaw))
    return {"left": left, "right": right}
