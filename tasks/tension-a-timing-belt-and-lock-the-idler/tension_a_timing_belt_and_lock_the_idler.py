"""On the station table, a belt drive module, fixed: a base plate with two toothed pulleys and the timing belt round
them. In front of the belt, a tensioner lever pivots on a bolt; an idler wheel on the lever presses on the belt when
the lever is swung back. A spring holds the lever forward, off the belt (slack). The bolt carries a quarter-turn lock
knob; a scale beside the lever's tip shows where the belt is tensioned (the green band).

Task: swing the tensioner lever back by the peg at its tip until its pointer is in the green band (the idler against
the belt), hold it there, turn the lock knob a quarter turn clockwise to lock the lever, and let go. The spring swings
the lever off the belt as soon as it is let go, and turned first, the knob locks it slack: one hand holds the lever at
the mark while the other turns the knob.

Run it:  inspect-robots run --embodiment yam_arena -E task=tension_a_timing_belt_and_lock_the_idler.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step,  # noqa: E402
                     rotation, set_hinge_drive)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the module (fixed): a base plate, two pulleys and the belt along y; the tensioner's pivot bolt in front ----------
MX, MY = 0.30, 0.0  # the middle of the belt drive
PLATE = (0.20, 0.40, 0.012)  # along x, along y, thick
PLATE_TOP = TOP + PLATE[2]
PULLEY_R, PULLEY_H, PULLEY_Y = 0.020, 0.018, 0.14  # the two pulleys at y = +-PULLEY_Y
BELT_T, BELT_H = 0.003, 0.010
BELT_Z = PLATE_TOP + 0.006 + PULLEY_H / 2  # the belt's middle height
PIVOT_X, PIVOT_Y = MX + 0.060, -0.070  # the tensioner's pivot bolt
ALU, DARK, BELT_C, GREEN, RED, WHITE = (0.74, 0.75, 0.77), (0.14, 0.14, 0.15), (0.10, 0.10, 0.11), (0.15, 0.70, 0.25), \
    (0.85, 0.15, 0.12), (0.95, 0.95, 0.95)

# -- the tensioner lever: hinged at the pivot bolt about z, along +y from it; its frame: origin on the pivot axis at the
# plate's top. Turned by +theta (counterclockwise from above), its tip swings back (-x), toward the belt ----------------
LEVER_L, LEVER_W, LEVER_T = 0.180, 0.016, 0.008
LEVER_Z = 0.012  # the lever's middle above the plate (on a spacer)
IDLER_AT, IDLER_R, IDLER_H = 0.100, 0.012, 0.012  # the idler wheel, this far along the lever
IDLER_GAP = 0.011  # at rest, the idler stands this far off the belt
PEG_R, PEG_H = 0.007, 0.040  # a peg standing up at the lever's tip, to take it by
LEVER_MASS = 0.15
SPRING, PRELOAD_DEG = 0.03, 3.0  # N m per degree, toward -PRELOAD_DEG: about 1.6 N at the peg to hold it at the mark
MAX_DEG = 12.0

# -- the lock knob on the pivot bolt: hinged about z; a quarter turn clockwise locks the lever where it is -------------
KNOB_STEM_H, KNOB_R, KNOB_H = 0.035, 0.009, 0.014
LOCK_DEG = 75.0  # turned this far clockwise: locked
LOCK_SPRING = 5.0  # N m per degree: the lever held where it was locked

# -- scoring ----------------------------------------------------------------------------------------------------------
BAND = (4.5, 8.5)  # degrees: the green band (the idler against the belt at about 6.3)
HELD_S = 1.0
LET_GO = 0.03
SUCCESS_HOLD_S = 1.0


def _idler_offset() -> float:
    """The idler's middle, this far in front of (+x) the lever's line, so that it stands IDLER_GAP off the belt."""
    belt_front = MX + PULLEY_R + BELT_T / 2
    return belt_front + IDLER_R + IDLER_GAP - PIVOT_X


def _module() -> Body:
    shapes = [BoxShape((0.0, 0.0, PLATE[2] / 2), PLATE, ALU)]
    for s in (-1.0, 1.0):  # the pulleys, on short hubs
        shapes += [CylShape((0.0, s * PULLEY_Y, PLATE[2] + 0.003), 0.008, 0.006, DARK),
                   CylShape((0.0, s * PULLEY_Y, PLATE[2] + 0.006 + PULLEY_H / 2), PULLEY_R, PULLEY_H, DARK)]
        # the belt: its two runs, and round each pulley's outer end
        shapes.append(BoxShape((s * (PULLEY_R + BELT_T / 2), 0.0, BELT_Z - TOP), (BELT_T, 2 * PULLEY_Y, BELT_H), BELT_C,
                               collide=False))
        shapes.append(BoxShape((0.0, s * (PULLEY_Y + PULLEY_R + BELT_T / 2), BELT_Z - TOP),
                               (2 * PULLEY_R + 2 * BELT_T, BELT_T, BELT_H), BELT_C, collide=False))
    px, py = PIVOT_X - MX, PIVOT_Y - MY
    shapes += [CylShape((px, py, PLATE[2] + LEVER_Z / 2), 0.010, LEVER_Z, DARK)]  # the pivot boss (under the lever)
    # the scale beside the lever's tip: a white strip, the green band, a red stop mark at the slack end
    tip_y = py + LEVER_L
    for a0, a1, c, z in ((-1.0, MAX_DEG, WHITE, 0.0004), (BAND[0], BAND[1], GREEN, 0.0008)):
        x0 = px - (LEVER_L + 0.012) * math.sin(math.radians(a1))
        x1 = px - (LEVER_L + 0.012) * math.sin(math.radians(a0))
        shapes.append(BoxShape(((x0 + x1) / 2, tip_y + 0.012, PLATE[2] + z / 2), (x1 - x0, 0.012, z), c, collide=False))
    return Body.build("module", pos=(MX, MY, TOP), mass=3.0, shapes=shapes, kinematic=True, friction=(0.5, 0.4),
                      color=ALU)


def _lever() -> Body:
    off = _idler_offset()
    shapes = [BoxShape((0.0, LEVER_L / 2, LEVER_Z), (LEVER_W, LEVER_L, LEVER_T), ALU),
              BoxShape((off / 2, IDLER_AT, LEVER_Z), (abs(off), 0.012, LEVER_T), ALU),  # the idler's arm
              CylShape((off, IDLER_AT, LEVER_Z), IDLER_R, IDLER_H, DARK),
              CylShape((0.0, LEVER_L - 0.006, LEVER_Z + LEVER_T / 2 + PEG_H / 2), PEG_R, PEG_H, RED),
              BoxShape((0.0, LEVER_L + 0.006, LEVER_Z), (0.004, 0.012, LEVER_T), RED, collide=False)]  # the pointer
    return Body.build("lever", pos=(PIVOT_X, PIVOT_Y, PLATE_TOP), mass=LEVER_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=ALU, com=(0.0, 0.07, LEVER_Z), ignore=("module",))


def _knob() -> Body:
    z0 = LEVER_Z + LEVER_T / 2 + 0.001
    shapes = [CylShape((0.0, 0.0, z0 + KNOB_STEM_H / 2), 0.005, KNOB_STEM_H, ALU),
              CylShape((0.0, 0.0, z0 + KNOB_STEM_H + KNOB_H / 2), KNOB_R, KNOB_H, DARK),
              BoxShape((KNOB_R * 0.6, 0.0, z0 + KNOB_STEM_H + KNOB_H + 0.0004), (KNOB_R * 1.0, 0.003, 0.0008), WHITE,
                       collide=False)]  # a mark on top, to see it turn
    return Body.build("lock_knob", pos=(PIVOT_X, PIVOT_Y, PLATE_TOP), mass=0.02, shapes=shapes, friction=(1.2, 1.0),
                      color=DARK, ignore=("module", "lever"))


MODULE, LEVER, KNOB = _module(), _lever(), _knob()
OBJECTS = [MODULE, LEVER, KNOB]
hinge("pivot", "module", "lever", anchor=(PIVOT_X, PIVOT_Y, PLATE_TOP), axis="Z", limits_deg=(0.0, MAX_DEG),
      spring=SPRING, target_deg=-PRELOAD_DEG, damping=0.002)
hinge("lock", "module", "lock_knob", anchor=(PIVOT_X, PIVOT_Y, PLATE_TOP), axis="Z", limits_deg=(-90.0, 0.0),
      damping=0.0005)
KNOB_TOP_Z = PLATE_TOP + LEVER_Z + LEVER_T / 2 + 0.001 + KNOB_STEM_H + KNOB_H


# -- followed after every control step: the lever's angle, the knob, the lock ------------------------------------------

def _fresh() -> dict:
    return {"locked": None, "band_steps": 0, "held": False, "tcps": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("BELT_DEBUG")), "steps": 0}


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


def _lever_deg(rot) -> float:
    return math.degrees(math.atan2(-float(rot[0, 1]), float(rot[1, 1])))


def _knob_deg(rot) -> float:
    return math.degrees(math.atan2(float(rot[1, 0]), float(rot[0, 0])))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    lv = as_numpy(base.scene["lever"].data.root_link_pose_w)[0]
    kb = as_numpy(base.scene["lock_knob"].data.root_link_pose_w)[0]
    theta, phi = _lever_deg(rotation(lv[3:7])), _knob_deg(rotation(kb[3:7]))
    _EP["tcps"] = _tcps(base, origin)
    turned = -phi  # clockwise from above
    if _EP["locked"] is None and turned >= LOCK_DEG:  # locked where the lever is now
        _EP["locked"] = theta
        set_hinge_drive("pivot", target_deg=theta, spring=LOCK_SPRING, damping=0.05)
    elif _EP["locked"] is not None and turned < LOCK_DEG - 15.0:  # unlocked: the spring again
        _EP["locked"] = None
        set_hinge_drive("pivot", target_deg=-PRELOAD_DEG, spring=SPRING, damping=0.002)  # (as authored)
    in_band = BAND[0] <= theta <= BAND[1]
    _EP["band_steps"] = _EP["band_steps"] + 1 if in_band else 0
    if _EP["band_steps"] >= round(HELD_S * 20):
        _EP["held"] = True
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status_from(theta)
            tl, tr = _EP["tcps"]
            print(f"[belt] t={_DEBUG['steps'] / 20:5.1f}s lever={theta:5.2f}deg knob={turned:5.1f}deg "
                  f"locked={None if _EP['locked'] is None else round(_EP['locked'], 2)} held={_EP['held']} "
                  f"let_go={st['let_go']} left TCP-peg={np.round((tl - _peg(theta)) * 1000).astype(int).tolist()} "
                  f"right TCP-knob={np.round((tr - np.array([PIVOT_X, PIVOT_Y, KNOB_TOP_Z - KNOB_H / 2])) * 1000).astype(int).tolist()} "
                  f"score={_points(st)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _peg(theta_deg: float) -> np.ndarray:
    t = math.radians(theta_deg)
    r = LEVER_L - 0.006
    return np.array([PIVOT_X - r * math.sin(t), PIVOT_Y + r * math.cos(t), PLATE_TOP + LEVER_Z + LEVER_T / 2 + PEG_H / 2])


def _status_from(theta: float) -> dict:
    let_go = True
    if _EP["tcps"] is not None:
        knob = np.array([PIVOT_X, PIVOT_Y, KNOB_TOP_Z - KNOB_H / 2])
        for t in _EP["tcps"]:
            let_go &= float(np.linalg.norm(t - knob)) >= LET_GO and float(np.linalg.norm(t - _peg(theta))) >= LET_GO
    locked = _EP["locked"]
    return {"theta": theta, "tensioned": locked is not None and BAND[0] <= locked <= BAND[1], "let_go": let_go,
            "held": _EP["held"]}


def _status(objects) -> dict:
    return _status_from(_lever_deg(rotation(objects["lever"].quat_xyzw)))


def _points(st) -> float:
    if st["tensioned"] and st["let_go"]:
        return 1.0
    if st["tensioned"]:
        return 0.5
    return 0.25 if st["held"] else 0.0


def score(objects) -> float:
    """1.0 with the lever locked with its pointer in the green band and both grippers let go; 0.5 with it locked in the
    band; 0.25 if it was held in the band for 1 s at some point; else 0.0."""
    return _points(_status(objects))


def success(objects) -> bool:
    """Full score with the lever and the knob still."""
    return _points(_status(objects)) >= 1.0 and objects["lever"].resting() and objects["lock_knob"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down round the peg at the lever's tip and swings the lever back until the idler meets
# the belt (the pointer in the green band), and holds it. The right hand comes straight down on the lock knob, grips it
# and turns it a quarter turn clockwise. Then the right hand lets go and rises, and the left hand lets go and rises.

GRIP = grip_rotation(0.0)  # straight down, fingers closing along y
PEG_IN_HAND = 0.010  # the left TCP this far below the peg's top
KNOB_IN_HAND = 0.003  # the right TCP this far above the knob's middle
AIM_DEG = 11.0  # swing the peg toward this angle (the arm gives ~4.5 degrees under the spring)
YAW0 = math.radians(45.0)  # the right grip's yaw on the knob before the turn (it turns to -45)
LEAD_S, LEAD_STEPS = 3.0, 15


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed), base_pos=station.BASES[arm],
                             iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, label: str) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), FINGER_OPEN, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _peg_tcp(theta_deg: float) -> np.ndarray:
    return _peg(theta_deg) + [0.0, 0.0, PEG_H / 2 - PEG_IN_HAND]


SWING = [1.0, 0.8, 2.0]  # down round the peg, grip, swing it back
SWUNG_AT = LEAD_S + sum(SWING)


def _left(release_at: float, total: float) -> list[Waypoint]:
    start = _peg_tcp(0.0)
    out = _lead_in("left", start + [0.0, 0.0, 0.05], GRIP, "over the peg at the lever's tip")
    out += [_w(SWING[0], start, FINGER_OPEN, "down round the peg", GRIP),
            _w(SWING[1], start, FINGER_CLOSED, "grip it", GRIP)]
    for k in range(1, 5):  # along the arc
        out.append(_w(SWING[2] / 4, _peg_tcp(AIM_DEG * k / 4), FINGER_CLOSED, "swing the lever back: the idler onto the belt",
                      GRIP))
    held = _peg_tcp(AIM_DEG)
    t = sum(w.duration for w in out)
    out.append(_w(max(release_at - t, 0.5), held, FINGER_CLOSED, "hold it there while the knob is turned", GRIP))
    out += [_w(0.5, held, FINGER_OPEN, "let go", GRIP),
            _w(1.0, held + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", GRIP)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), held + [0.0, 0.0, 0.05], FINGER_OPEN, "wait", GRIP))
    return out


def _right() -> list[Waypoint]:
    knob = np.array([PIVOT_X, PIVOT_Y, KNOB_TOP_Z - KNOB_H / 2 + KNOB_IN_HAND])
    rot0 = grip_rotation(YAW0)
    out = _lead_in("right", knob + [0.0, 0.0, 0.05], rot0, "over the lock knob")
    t = LEAD_S
    out.append(_w(max(SWUNG_AT - t, 0.2) + 0.3, knob + [0.0, 0.0, 0.05], FINGER_OPEN, "wait for the lever to be swung back",
                  rot0))
    out += [_w(1.0, knob, FINGER_OPEN, "down round the knob", rot0),
            _w(0.6, knob, FINGER_CLOSED, "grip it", rot0)]
    for k in range(1, 6):
        out.append(_w(0.4, knob, FINGER_CLOSED, "turn it a quarter turn clockwise: locked", _rz(-math.pi / 2 * k / 5) @ rot0))
    end = _rz(-math.pi / 2) @ rot0
    out += [_w(0.5, knob, FINGER_OPEN, "let go", end),
            _w(1.0, knob + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", end)]
    return out


def demo():
    right = _right()
    total = sum(w.duration for w in right)
    left = _left(total, total + 4.0)
    right.append(_w(4.0 + 1.5, right[-1].pos, FINGER_OPEN, "wait", right[-1].yaw))
    return {"left": left, "right": right}
