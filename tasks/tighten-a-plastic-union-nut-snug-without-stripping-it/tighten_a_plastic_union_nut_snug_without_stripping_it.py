"""On the station table, a short plastic pipe (16 mm) lies in two fixed supports, along the left-right line. On its
threaded right end a plastic union nut is started, a turn and a half short of snug against the pipe's collar.

Task: tighten the nut until it is snug against the collar, and stop there: plastic threads strip if forced. Then let
go. The pipe lies loose in its supports and turns with the nut, so one hand holds the pipe while the other turns the
nut. Snug, the nut gets much harder to turn; turned on half a turn past snug, the thread strips (score 0).

Run it:  inspect-robots run --embodiment yam_arena -E task=tighten_a_plastic_union_nut_snug_without_stripping_it.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation,  # noqa: E402
                     set_slide_target, set_turn_damping, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the pipe. Its frame: origin on its axis at its threaded (right) end, +y along the pipe (to the left) -------------
PX, Y_END, Z_AX = 0.30, -0.09, TOP + 0.11  # its axis's x, its right end's y, its axis's height
PIPE_R, PIPE_L = 0.008, 0.24
THREAD_L = 0.020  # the threaded length at its end
COLLAR_R, COLLAR_L = 0.013, 0.008  # a collar the nut tightens against, just past the thread
PIPE_MASS = 0.15
SUPPORT_YS = (-0.02, 0.13)  # two fixed supports under it (world y)

# -- the union nut. Its frame: origin on its axis at its outer (right) face, +y along the pipe -------------------------
NUT_R, NUT_L = 0.010, 0.016  # 20 mm across (narrow enough for a centred grip), 16 mm long
NUT_MASS = 0.02
PITCH = 0.004  # m per turn
X_SNUG = THREAD_L - NUT_L  # the nut's outer face this far in from the pipe's end once snug against the collar
TO_SNUG = 1.5  # turns from the start to snug
X_START = X_SNUG - PITCH * TO_SNUG
THREAD_DRAG, TIGHT_DRAG, STRIPPED_DRAG = 0.3, 0.8, 0.02  # N m s/rad: running, snug, stripped
STRIP = 0.5  # turns past snug that strip the thread

PVC, DARK, NUT_C, SUPPORT_C = (0.86, 0.87, 0.88), (0.45, 0.46, 0.48), (0.93, 0.93, 0.95), (0.30, 0.30, 0.32)

# -- scoring ----------------------------------------------------------------------------------------------------------
SNUG_TOL = 0.1  # turns short of snug that still count as snug
TURNED = (0.25, 1.0)  # turns in for 0.25 and 0.5
LET_GO = 0.03
SUCCESS_HOLD_S = 1.0


def _pipe() -> Body:
    shapes = [CylShape((0.0, PIPE_L / 2, 0.0), PIPE_R, PIPE_L, PVC, axis="Y"),
              CylShape((0.0, THREAD_L / 2, 0.0), PIPE_R + 0.0006, THREAD_L, DARK, axis="Y", collide=False),
              CylShape((0.0, THREAD_L + COLLAR_L / 2, 0.0), COLLAR_R, COLLAR_L, DARK, axis="Y")]
    return Body.build("pipe", pos=(PX, Y_END, Z_AX), mass=PIPE_MASS, shapes=shapes, friction=(1.0, 0.9), color=PVC)


def _nut() -> Body:
    shapes = [CylShape((0.0, NUT_L / 2, 0.0), NUT_R, NUT_L, NUT_C, axis="Y")]
    for k in range(6):  # grip ribs, so the turn shows
        a = 2 * math.pi * k / 6
        shapes.append(BoxShape((NUT_R * math.cos(a), NUT_L / 2, NUT_R * math.sin(a)), (0.002, NUT_L - 0.002, 0.002),
                               DARK, collide=False))
    pos = (PX, Y_END + X_START, Z_AX)
    return Body.build("nut", pos=pos, mass=NUT_MASS, shapes=shapes, friction=(1.2, 1.0), color=NUT_C,
                      ignore=("pipe",))


def _supports() -> Body:
    base_top = Z_AX - PIPE_R - TOP
    shapes = []
    for y in SUPPORT_YS:
        dy = y - SUPPORT_YS[0]
        shapes += [BoxShape((0.0, dy, base_top / 2), (0.05, 0.02, base_top), SUPPORT_C),
                   *[BoxShape((s * (PIPE_R + 0.004), dy, base_top + 0.004), (0.004, 0.02, 0.008), SUPPORT_C)
                     for s in (-1.0, 1.0)]]
    return Body.build("supports", pos=(PX, SUPPORT_YS[0], TOP), mass=1.0, shapes=shapes, kinematic=True,
                      friction=(0.02, 0.02), slick=True, color=SUPPORT_C)  # smooth: the pipe turns in them freely


SUPPORTS, PIPE, NUT = _supports(), _pipe(), _nut()
OBJECTS = [SUPPORTS, PIPE, NUT]
# the nut on the pipe's thread: its outer face this far in from the pipe's end, drawn in PITCH per turn by the task file
thread("union", "pipe", "nut", anchor0=(0.0, 0.0, 0.0), axis0=(0.0, 1.0, 0.0), anchor1=(0.0, 0.0, 0.0),
       axis1=(0.0, 1.0, 0.0), slide_limits=(X_START - 0.004, X_SNUG), turn_damping=THREAD_DRAG, slide_stiffness=3000.0,
       slide_damping=20.0, slide_max_force=50.0, always_on=True)


# -- followed after every control step: the nut's turns on the thread, the grippers ------------------------------------

def _fresh() -> dict:
    return {"turns": 0.0, "phi": None, "snug": False, "stripped": False, "tcps": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("UNION_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    set_slide_target("union", X_START)
    set_turn_damping("union", THREAD_DRAG)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _phi(p_rot, n_rot) -> float:
    """The nut's turn against the pipe about the pipe's axis (+y of the pipe: clockwise seen from the nut's face)."""
    axis = p_rot @ np.array([0.0, 1.0, 0.0])
    u = p_rot @ np.array([0.0, 0.0, 1.0])
    w = n_rot @ np.array([0.0, 0.0, 1.0])
    return math.atan2(float(np.cross(u, w) @ axis), float(u @ w))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    p = as_numpy(base.scene["pipe"].data.root_link_pose_w)[0]
    n = as_numpy(base.scene["nut"].data.root_link_pose_w)[0]
    pr, nr = rotation(p[3:7]), rotation(n[3:7])
    phi = _phi(pr, nr)
    _EP["tcps"] = _tcps(base, origin)
    if _EP["phi"] is not None and not _EP["stripped"]:
        _EP["turns"] += ((phi - _EP["phi"] + math.pi) % (2 * math.pi) - math.pi) / (2 * math.pi)
        over = _EP["turns"] - TO_SNUG
        if over >= STRIP:  # forced past snug: the thread strips; the nut spins loose where it is
            _EP["stripped"] = True
            set_turn_damping("union", STRIPPED_DRAG)
        else:
            snug = over >= -SNUG_TOL
            if snug != _EP["snug"]:
                set_turn_damping("union", TIGHT_DRAG if snug else THREAD_DRAG)
                _EP["snug"] = snug
            set_slide_target("union", min(max(X_START + PITCH * _EP["turns"], X_START - 0.004), X_SNUG))
    _EP["phi"] = phi
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status_from(p[:3] - origin, pr, n[:3] - origin)
            tl, tr = _EP["tcps"]
            pipe_spin = math.degrees(math.atan2(float(pr[0, 2]), float(pr[2, 2])))
            x = float((n[:3] - p[:3]) @ (pr @ np.array([0.0, 1.0, 0.0])))
            grip = p[:3] - origin + pr @ np.array([0.0, LEFT_AT, 0.0])
            print(f"[union] t={_DEBUG['steps'] / 20:5.1f}s pipe={np.round((p[:3] - origin) * 1000).astype(int).tolist()} "
                  f"pipe_spin={pipe_spin:6.1f} turns={_EP['turns']:5.2f} nut_in={x * 1000:5.1f}mm (snug {X_SNUG * 1000:.1f}) "
                  f"snug={st['snug']} stripped={st['stripped']} let_go={st['let_go']} "
                  f"left TCP-pipe={np.round((tl - grip) * 1000).astype(int).tolist()} "
                  f"right TCP-nut={np.round((tr - (n[:3] - origin)) * 1000).astype(int).tolist()} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status_from(p_pos, p_rot, n_pos) -> dict:
    let_go = True
    tcps = _EP["tcps"]
    if tcps is not None:
        a = np.asarray(p_pos, float)
        d = p_rot @ np.array([0.0, 1.0, 0.0])
        mid = np.asarray(n_pos, float) + d * (NUT_L / 2)
        for t in tcps:
            v = np.asarray(t, float) - a
            s = float(np.clip(v @ d, 0.0, PIPE_L))
            let_go &= float(np.linalg.norm(v - s * d)) >= LET_GO and float(np.linalg.norm(t - mid)) >= LET_GO
    return {"turns": _EP["turns"], "snug": _EP["snug"], "stripped": _EP["stripped"], "let_go": let_go}


def _status(objects) -> dict:
    p, n = objects["pipe"], objects["nut"]
    return _status_from(p.pos, rotation(p.quat_xyzw), n.pos)


def _points(st) -> float:
    if st["stripped"]:
        return 0.0
    if st["snug"] and st["let_go"]:
        return 1.0
    if st["turns"] >= TURNED[1]:
        return 0.5
    return 0.25 if st["turns"] >= TURNED[0] else 0.0


def score(objects) -> float:
    """0.0 if the thread was stripped (the nut forced half a turn past snug). Otherwise 1.0 with the nut snug against the
    collar and both grippers let go (3 cm clear of the pipe and the nut); 0.5 with it tightened at least 1 turn; 0.25
    with it tightened at least a quarter turn; else 0.0."""
    return _points(_status(objects))


def success(objects) -> bool:
    """Full score with the pipe and the nut still."""
    return _points(_status(objects)) >= 1.0 and objects["pipe"].resting() and objects["nut"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the pipe near its left end and holds it. The right hand, level and pointing
# along the pipe, comes down onto the nut (its fingers either side), closes on it and tightens it in strokes of a third of a turn (the wrist rolls
# only so far): roll, let go, roll back open, take the nut again, following it along the thread. Stopped at snug, both
# hands let go and rise.


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


DOWN_L = grip_rotation(-math.pi / 2)  # straight down, fingers closing along x
SIDE_R = _rx(math.pi / 2) @ grip_rotation(math.pi / 2)  # level, pointing +y, fingers closing along x
LEFT_AT = 0.20  # the left hand holds the pipe this far from its right end (15 cm from the other hand at least)
IN_HAND = 0.003  # the left TCP this far above the pipe's axis
NUT_IN_HAND = 0.002  # the right TCP this far in from the nut's face
ROLL0, STROKE, STROKES = math.radians(-40.0), math.radians(120.0), 6
SLIP = 0.7  # the nut turns about this much of each stroke (it slips in the fingers)
LEAD_S, LEAD_STEPS = 3.0, 15
PICK = [1.0, 0.8, 0.3]
TURN_S, RELEASE_S, BACK_S, REGRIP_S = 1.6, 0.4, 0.8, 0.4
OFF = 0.05  # the right hand comes down onto the nut from this far above it


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


def _nut_tcp(turned: float) -> np.ndarray:
    """The right TCP on the nut after the wrist has rolled ``turned`` (rad) in all."""
    x = min(X_START + PITCH * SLIP * turned / (2 * math.pi), X_SNUG)
    return np.array([PX, Y_END + x + NUT_IN_HAND, Z_AX])


def _aim(roll: float) -> np.ndarray:
    return SIDE_R @ _rz(roll)


def _right() -> list[Waypoint]:
    start = _nut_tcp(0.0)
    out = _lead_in("right", start + [0.0, 0.0, OFF], _aim(ROLL0), "level, over the nut, pointing along the pipe")
    out += [_w(1.5, start, FINGER_OPEN, "down round the nut", _aim(ROLL0)),
            _w(PICK[1], start, FINGER_CLOSED, "grip it", _aim(ROLL0))]
    turned = 0.0
    for k in range(STROKES):
        for j in range(1, 5):  # a quarter of the stroke at a time
            done = turned + STROKE * j / 4
            out.append(_w(TURN_S / 4, _nut_tcp(done), FINGER_CLOSED, f"tighten ({k + 1})", _aim(ROLL0 + STROKE * j / 4)))
        turned += STROKE
        at = _nut_tcp(turned)
        out.append(_w(RELEASE_S, at, FINGER_OPEN, "let go", _aim(ROLL0 + STROKE)))
        if k < STROKES - 1:
            for j in range(1, 5):
                out.append(_w(BACK_S / 4, at, FINGER_OPEN, "roll back, open", _aim(ROLL0 + STROKE * (1 - j / 4))))
            out.append(_w(REGRIP_S, at, FINGER_CLOSED, "take the nut again", _aim(ROLL0)))
    out.append(_w(1.0, _nut_tcp(turned) + [0.0, 0.0, OFF], FINGER_OPEN, "snug: stop, and rise", _aim(ROLL0 + STROKE)))
    return out


def _left(release_at: float, total: float) -> list[Waypoint]:
    grip = np.array([PX, Y_END + LEFT_AT, Z_AX + IN_HAND])
    out = _lead_in("left", grip + [0.0, 0.0, 0.05], DOWN_L, "over the pipe near its left end")
    out += [_w(PICK[0], grip, FINGER_OPEN, "down round the pipe", DOWN_L),
            _w(PICK[1], grip, FINGER_CLOSED, "grip it", DOWN_L)]
    t = LEAD_S + PICK[0] + PICK[1]
    out.append(_w(max(release_at - t, 0.5), grip, FINGER_CLOSED, "hold the pipe still", DOWN_L))
    out += [_w(0.5, grip, FINGER_OPEN, "let go", DOWN_L),
            _w(1.0, grip + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", DOWN_L)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), grip + [0.0, 0.0, 0.05], FINGER_OPEN, "wait", DOWN_L))
    return out


def demo():
    right = _right()
    total = sum(w.duration for w in right)
    left = _left(total - 1.0, total + 4.0)
    right.append(_w(4.0, right[-1].pos, FINGER_OPEN, "wait", right[-1].yaw))
    return {"left": left, "right": right}
