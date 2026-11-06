"""On the station table runs a short pipe line, held on brackets: a fixed pipe on the right, a union nut joining it to
the next pipe piece, and that piece screwed into a fixed elbow on the left. Two wrenches are already on: one on the union
nut, its handle up to the right; a backup wrench on the pipe piece's hex, its handle up to the left.

Task: break the union loose. One hand presses the union nut's wrench down, turning the nut loose; the other holds the
backup wrench down, so the pipe piece cannot turn. With the backup wrench not held, the union nut's turn drags the pipe
piece round with it and loosens the wrong joint, at the elbow (its thread is the less stiff of the two).

Run it:  inspect-robots run --embodiment yam_arena -E task=break_loose_a_pipe_union_with_two_wrenches.py \\
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
from yam_arena.constants import FINGER_CLOSED, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the pipe line, along x at y = 0 ---------------------------------------------------------------------------------
PZ = TOP + 0.07  # its axis
PIPE_R = 0.012
NEAR_X = (0.15, 0.264)  # the fixed pipe (right... toward the arms) and its union tail
NUT_X = (0.265, 0.290)  # the union nut
BODY_X = (0.291, 0.360)  # the pipe piece, with its hex
HEX_X = (0.330, 0.352)
ELBOW_X = (0.361, 0.40)  # the fixed elbow

# -- the wrenches: each a handle 14 cm long from its nut's axis, up 25 degrees from level, with a flat pad at its end ---
HANDLE_L, HANDLE_UP = 0.14, math.radians(25.0)
PAD = (0.030, 0.030, 0.008)

# -- the threads: each holds until turned hard enough, then turns against a drag; only the loosening way (+ about +x).
# A spring holds each where it last stopped; pushed SLIP past that, it gives (the spring's hold moves to it) ----------
NUT_HOLD, ELBOW_HOLD = 0.10, 0.03  # N m per degree: the union nut holds to about 0.2 N m, the elbow joint 0.06
SLIP = 2.0  # degrees
NUT_DRAG = 0.05  # N m per degree/s, while turning
ELBOW_DRAG = 0.017
TURN_MAX = 120.0

STEEL, DARK, BRASS, RED, BLUE = (0.62, 0.64, 0.66), (0.20, 0.21, 0.23), (0.78, 0.64, 0.30), (0.75, 0.15, 0.12), \
    (0.15, 0.30, 0.70)

# -- scoring ----------------------------------------------------------------------------------------------------------
LOOSE = 25.0  # the union nut turned at least this far loose: broken loose
WRONG, WRONG_CLEAN = 15.0, 8.0  # the elbow joint turned this far: loosened (0 points); under this: left alone
LET_GO = 0.04
SUCCESS_HOLD_S = 1.0


def _qx(a: float) -> tuple[float, float, float, float]:
    return (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))


def _handle(side: float, x: float, color) -> list:
    """A wrench's handle and pad, from the axis toward ``side`` (-1: -y, right; +1: +y, left), up HANDLE_UP."""
    a = side * HANDLE_UP  # about +x: +y side turned up by +a, -y side up by -a
    d = np.array([0.0, side * math.cos(HANDLE_UP), math.sin(HANDLE_UP)])
    mid, end = d * (0.02 + HANDLE_L) / 2, d * HANDLE_L
    return [BoxShape((x, float(mid[1]), float(mid[2])), (0.016, HANDLE_L - 0.02, 0.010), color, quat_xyzw=_qx(a)),
            BoxShape((x, float(end[1]), float(end[2]) + PAD[2] / 2), PAD, DARK, quat_xyzw=_qx(a))]


def _near() -> Body:
    """The fixed pipe and its union tail, on a bracket; and the elbow with its riser. Origin on the axis at x = 0."""
    shapes = [CylShape(((NEAR_X[0] + NEAR_X[1]) / 2, 0.0, 0.0), PIPE_R, NEAR_X[1] - NEAR_X[0], STEEL, axis="X"),
              BoxShape((0.18, 0.0, -(PZ - TOP) / 2), (0.02, 0.03, PZ - TOP), DARK),  # its bracket
              BoxShape(((ELBOW_X[0] + ELBOW_X[1]) / 2, 0.0, 0.0), (ELBOW_X[1] - ELBOW_X[0], 0.036, 0.036), BRASS),
              CylShape((ELBOW_X[1] - 0.018, 0.0, -(PZ - TOP) / 2), PIPE_R, PZ - TOP, STEEL)]  # the elbow's riser
    return Body.build("pipeline", pos=(0.0, 0.0, PZ), mass=5.0, shapes=shapes, kinematic=True, color=STEEL)


def _body() -> Body:
    """The pipe piece with its hex and the backup wrench; origin on the axis at x = 0 (mass on the axis)."""
    shapes = [CylShape(((BODY_X[0] + BODY_X[1]) / 2, 0.0, 0.0), PIPE_R, BODY_X[1] - BODY_X[0], STEEL, axis="X"),
              CylShape(((HEX_X[0] + HEX_X[1]) / 2, 0.0, 0.0), 0.018, HEX_X[1] - HEX_X[0], DARK, axis="X"),
              *_handle(+1.0, (HEX_X[0] + HEX_X[1]) / 2, BLUE)]
    return Body.build("pipe_piece", pos=(0.0, 0.0, PZ), mass=0.4, shapes=shapes, friction=(1.0, 0.9), color=STEEL,
                      com=((BODY_X[0] + BODY_X[1]) / 2, 0.0, 0.0))


def _nut() -> Body:
    """The union nut with its wrench; origin on the axis at x = 0 (mass on the axis)."""
    shapes = [CylShape(((NUT_X[0] + NUT_X[1]) / 2, 0.0, 0.0), 0.021, NUT_X[1] - NUT_X[0], BRASS, axis="X"),
              *_handle(-1.0, (NUT_X[0] + NUT_X[1]) / 2, RED)]
    return Body.build("union_nut", pos=(0.0, 0.0, PZ), mass=0.2, shapes=shapes, friction=(1.0, 0.9), color=BRASS,
                      com=((NUT_X[0] + NUT_X[1]) / 2, 0.0, 0.0), ignore=("pipeline",))  # it turns on the union's tail


OBJECTS = [_near(), _body(), _nut()]
# the turn limits stop each joint at its tight end (1 degree past where it starts): nothing turns the tightening way
hinge("elbow_thread", "pipeline", "pipe_piece", anchor=(BODY_X[1], 0.0, PZ), axis="X", limits_deg=(-1.0, TURN_MAX),
      spring=ELBOW_HOLD, damping=ELBOW_DRAG, target_deg=0.0)
hinge("union_thread", "pipe_piece", "union_nut", anchor=(NUT_X[1], 0.0, PZ), axis="X", limits_deg=(-1.0, TURN_MAX),
      spring=NUT_HOLD, damping=NUT_DRAG, target_deg=0.0)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"tcps": None, "s": {}, "worst_elbow": 0.0, "held": {"elbow_thread": 0.0, "union_thread": 0.0}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("UNION_DEBUG")), "steps": 0}


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


def _roll(r: np.ndarray) -> float:
    """A body's turn about +x, degrees."""
    return math.degrees(math.atan2(float(r[2, 1]), float(r[1, 1])))


def _pad(side: float, x: float, turn_deg: float) -> np.ndarray:
    a = side * HANDLE_UP + math.radians(turn_deg) if side > 0 else math.pi - HANDLE_UP + math.radians(turn_deg)
    return np.array([x, HANDLE_L * math.cos(a), PZ + HANDLE_L * math.sin(a)])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    p = as_numpy(base.scene["pipe_piece"].data.root_link_pose_w)[0]
    n = as_numpy(base.scene["union_nut"].data.root_link_pose_w)[0]
    elbow = _roll(rotation(p[3:7]))
    nut = _roll(rotation(n[3:7])) - elbow
    _EP["worst_elbow"] = max(_EP["worst_elbow"], abs(elbow))
    for name, angle in (("elbow_thread", elbow), ("union_thread", nut)):  # turned hard past its hold: it gives
        if angle - _EP["held"][name] > SLIP:
            _EP["held"][name] = angle
            set_hinge_drive(name, target_deg=angle)
    _EP["tcps"] = _tcps(base, origin)
    pads = [_pad(-1.0, (NUT_X[0] + NUT_X[1]) / 2, elbow + nut), _pad(+1.0, (HEX_X[0] + HEX_X[1]) / 2, elbow)]
    clear = all(float(np.linalg.norm(t - q)) >= LET_GO for t in _EP["tcps"] for q in pads)
    _EP["s"] = {"elbow": elbow, "nut": nut, "clear": clear}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            print(f"[union] t={_DEBUG['steps'] / 20:5.1f}s nut_loose={nut:6.1f}deg elbow={elbow:6.1f}deg "
                  f"worst_elbow={_EP['worst_elbow']:5.1f} clear={clear} "
                  f"rightTCP-nut_pad={np.round((tr - pads[0]) * 1000).astype(int).tolist()} "
                  f"leftTCP-backup_pad={np.round((tl - pads[1]) * 1000).astype(int).tolist()} score={_points(_status())}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"nut": st.get("nut", 0.0), "elbow": max(_EP["worst_elbow"], abs(st.get("elbow", 0.0))),
            "clear": bool(st.get("clear"))}


def _points(s) -> float:
    if s["elbow"] >= WRONG:
        return 0.0
    if s["nut"] >= LOOSE and s["elbow"] < WRONG_CLEAN and s["clear"]:
        return 1.0
    if s["nut"] >= LOOSE:
        return 0.5
    return 0.25 if s["nut"] >= 10.0 else 0.0


def score(objects) -> float:
    """0.0 if the elbow joint was ever turned 15 degrees loose (the wrong joint). Else 1.0 with the union nut turned at
    least 25 degrees loose, the elbow joint under 8 degrees, and both hands clear of the wrenches; 0.5 with the nut 25
    degrees loose otherwise; 0.25 with it 10 degrees loose; else 0.0."""
    return _points(_status())


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the backup wrench's pad, fingers closed, and presses it down, holding the pipe
# piece. The right hand comes straight down on the union nut's wrench pad, fingers closed, and presses it down along its
# arc, turning the nut 40 degrees loose. Then both hands rise.

PRESS = 0.005  # aimed this far into a pad
TIP = 0.013  # closed fingertips below the TCP
GRIP = grip_rotation(0.0)
TURN_TO = 40.0
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


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _on_pad(pad: np.ndarray, press: float) -> np.ndarray:
    return pad + [0.0, 0.0, PAD[2] + TIP - press]


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    pad = _pad(+1.0, (HEX_X[0] + HEX_X[1]) / 2, 0.0)
    out = _lead_in("left", _on_pad(pad, -0.03), GRIP, "over the backup wrench's pad", FINGER_CLOSED)
    out += [_w(1.0, _on_pad(pad, 0.0), FINGER_CLOSED, "down onto it", GRIP),
            _w(0.6, _on_pad(pad, PRESS), FINGER_CLOSED, "press it down: the pipe piece cannot turn", GRIP)]
    held_at = sum(w.duration for w in out)
    out += [_w(max(release_at - held_at, 0.3), _on_pad(pad, PRESS), FINGER_CLOSED, "hold it down", GRIP),
            _w(1.0, _on_pad(pad, -0.015) + [0.0, 0.04, 0.0], FINGER_CLOSED, "rise and draw back", GRIP)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), _on_pad(pad, -0.015) + [0.0, 0.04, 0.0], FINGER_CLOSED,
                  "hold still: the trial ends once the full score has held 1 s", GRIP))
    return out, held_at


def _right(start: float) -> list[Waypoint]:
    x = (NUT_X[0] + NUT_X[1]) / 2
    out = _lead_in("right", _on_pad(_pad(-1.0, x, 0.0), -0.03), GRIP, "over the union nut wrench's pad", FINGER_CLOSED)
    out += [_w(max(start - LEAD_S, 0.3), _on_pad(_pad(-1.0, x, 0.0), 0.0), FINGER_CLOSED, "down onto it", GRIP)]
    for k in range(1, 9):
        out.append(_w(0.5, _on_pad(_pad(-1.0, x, TURN_TO * k / 8), PRESS), FINGER_CLOSED,
                      "press it down along its swing: the nut turns loose", GRIP))
    end = _pad(-1.0, x, TURN_TO)
    out += [_w(0.5, _on_pad(end, 0.0), FINGER_CLOSED, "ease off", GRIP),
            _w(1.0, _on_pad(end, -0.05) + [0.0, 0.03, 0.0], FINGER_CLOSED, "rise", GRIP)]
    return out


def demo():
    _, held_at = _left(0.0, 0.0)
    right = _right(held_at)
    t_r = sum(w.duration for w in right)
    total = t_r + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_CLOSED, "hold still", GRIP))
    left, _ = _left(t_r - 0.8, total)
    return {"left": left, "right": right}
