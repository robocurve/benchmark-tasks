"""On the station table, in front of the arms, a manual coffee grinder stands loose: a light steel body with a crank on
top, whose arm turns about the body's axis against the grinding resistance, and an upright knob at the arm's end that
spins freely on its pin.

Task: grind: turn the crank three full turns clockwise (seen from above) and leave the grinder standing. The grinder is
light and the burrs resist: turned with one hand, the whole grinder spins, slides or tips over, so the other hand has to
hold its body.

Run it:  inspect-robots run --embodiment yam_arena -E task=grind_coffee_in_a_hand_grinder.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step, rotation  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the grinder (origin at the centre of its underside), the crank and the knob -----------------------------------
AXIS_XY = (0.24, -0.02)
BODY_R, BODY_H, BODY_MASS = 0.033, 0.180, 0.30
HUB_R, HUB_H = 0.012, 0.016  # on top of the body, turning with the crank
ARM_LEN, ARM_W, ARM_T = 0.065, 0.014, 0.010  # from the axis to the knob's pin; wide; thick
ARM_Z = BODY_H + 0.008  # the arm's centre height above the body's underside
KNOB_R, KNOB_H = 0.009, 0.070  # the knob stands on the arm's end
KNOB_Z0 = ARM_Z + ARM_T / 2  # the knob's underside
CRANK_MASS, KNOB_MASS = 0.04, 0.015
GRIND = 0.15  # N m s/rad: the burrs' resistance, about 0.3 N m at one turn every 3 s
START = math.radians(-90)  # the arm points to the right at the start

STEEL, BAND, BLACK, WOOD = (0.72, 0.73, 0.75), (0.25, 0.18, 0.12), (0.08, 0.08, 0.09), (0.62, 0.42, 0.24)

# -- scoring --------------------------------------------------------------------------------------------------------
FULL_TURNS, HALF_TURNS, SOME_TURNS = 3.0, 2.0, 1.0
UPRIGHT = math.radians(10)
SUCCESS_HOLD_S = 1.0

AXIS = np.array([AXIS_XY[0], AXIS_XY[1], TOP + 0.0005])
KNOB_XY0 = (AXIS[0] + ARM_LEN * math.cos(START), AXIS[1] + ARM_LEN * math.sin(START))


def _grinder() -> Body:
    shapes = [CylShape((0.0, 0.0, BODY_H / 2), BODY_R, BODY_H, STEEL),
              CylShape((0.0, 0.0, 0.035), BODY_R + 0.0008, 0.030, BAND, collide=False),  # the grounds drawer's band
              CylShape((0.0, 0.0, BODY_H - 0.004), BODY_R + 0.0008, 0.008, BLACK, collide=False)]
    return Body.build("grinder", pos=tuple(AXIS), mass=BODY_MASS, shapes=shapes, friction=(0.5, 0.4), color=STEEL)


def _crank() -> Body:
    c, s = math.cos(START), math.sin(START)
    shapes = [CylShape((0.0, 0.0, BODY_H + HUB_H / 2 + 0.0005), HUB_R, HUB_H, BLACK),
              BoxShape((c * ARM_LEN / 2, s * ARM_LEN / 2, ARM_Z), (ARM_LEN + ARM_W, ARM_W, ARM_T), BLACK, yaw=START)]
    return Body.build("crank", pos=tuple(AXIS), mass=CRANK_MASS, shapes=shapes, friction=(0.5, 0.4), color=BLACK)


def _knob() -> Body:
    shapes = [CylShape((0.0, 0.0, KNOB_H / 2), KNOB_R, KNOB_H, WOOD)]
    return Body.build("knob", pos=(*KNOB_XY0, AXIS[2] + KNOB_Z0 + 0.0005), mass=KNOB_MASS, shapes=shapes,
                      friction=(0.8, 0.7), color=WOOD)


GRINDER, CRANK, KNOB = _grinder(), _crank(), _knob()
hinge("axle", "grinder", "crank", (AXIS[0], AXIS[1], AXIS[2] + BODY_H), "Z", damping=GRIND)
hinge("knob_pin", "crank", "knob", (KNOB_XY0[0], KNOB_XY0[1], AXIS[2] + KNOB_Z0), "Z")
OBJECTS = [GRINDER, CRANK, KNOB]


# -- scoring ------------------------------------------------------------------------------------------------------
# on_step follows the crank's turn against the body every control step (it turns well under half a turn per step) and
# adds it up: clockwise, seen from above, counts positive.

_EPISODE = {"turns": 0.0, "last": None}
_DEBUG = {"on": bool(os.environ.get("GRIND_DEBUG")), "calls": 0}


@on_reset
def _new_episode() -> None:
    _EPISODE.update(turns=0.0, last=None)
    _DEBUG["calls"] = 0


def _crank_angle(body_quat, crank_quat) -> float:
    rel = rotation(body_quat).T @ rotation(crank_quat)
    return math.atan2(rel[1, 0], rel[0, 0])


@on_step
def _count_turns(env) -> None:
    scene = env.unwrapped.scene
    body = as_numpy(scene["grinder"].data.root_link_pose_w)[0]
    crank = as_numpy(scene["crank"].data.root_link_pose_w)[0]
    a = _crank_angle(body[3:7], crank[3:7])
    if _EPISODE["last"] is not None:
        d = (a - _EPISODE["last"] + math.pi) % (2 * math.pi) - math.pi
        _EPISODE["turns"] -= d / (2 * math.pi)  # clockwise from above is a negative turn about +z
    _EPISODE["last"] = a


def _tally(objects) -> dict:
    g = objects["grinder"]
    tilt = math.acos(max(-1.0, min(1.0, rotation(g.quat_xyzw)[2, 2])))
    standing = tilt < UPRIGHT and abs(g.pos[2] - TOP) < 0.01
    return {"turns": _EPISODE["turns"], "standing": standing}


def _points(t) -> float:
    if not t["standing"]:
        return 0.0
    if t["turns"] >= FULL_TURNS:
        return 1.0
    if t["turns"] >= HALF_TURNS:
        return 0.5
    if t["turns"] >= SOME_TURNS:
        return 0.25
    return 0.0


def score(objects) -> float:
    """0 unless the grinder ends standing upright on the table. Otherwise 1.0 if the crank has been turned at least 3
    full turns clockwise (seen from above, net, against the grinder's body), 0.5 if at least 2, 0.25 if at least 1,
    else 0."""
    t = _tally(objects)
    if _DEBUG["on"]:  # GRIND_DEBUG=1: one line a second (at 20 Hz) on stderr
        _DEBUG["calls"] += 1
        if _DEBUG["calls"] % 20 == 0:
            g = objects["grinder"]
            yaw = math.degrees(math.atan2(rotation(g.quat_xyzw)[1, 0], rotation(g.quat_xyzw)[0, 0]))
            print(f"[grind] t={_DEBUG['calls'] / 20:5.1f}s turns={t['turns']:.2f} standing={t['standing']} "
                  f"body_cm={np.round(np.asarray(g.pos[:2]) * 100, 1).tolist()} body_yaw={yaw:.0f}deg "
                  f"score={_points(t)}", file=sys.stderr, flush=True)
    return _points(t)


def success(objects) -> bool:
    """Full score with the grinder still."""
    return _points(_tally(objects)) >= 1.0 and objects["grinder"].resting()


# -- scripted demo --------------------------------------------------------------------------------------------------
# The left arm slides in level from its side and grips the grinder's body, fingers closing front and back. The right
# arm comes straight down on the knob, fingers open on either side of it, grips it near its top and carries it round
# the axis, clockwise, for three and a quarter turns; the knob spins on its pin, so the wrist need not turn.


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


SIDE_GRIP = {
    "left": _rx(-math.pi / 2) @ grip_rotation(-math.pi / 2),  # pointing -y
    "right": _rx(math.pi / 2) @ grip_rotation(math.pi / 2),  # pointing +y
}
BODY_GRIP = np.array([AXIS[0], AXIS[1], TOP + 0.140])  # a level grip reaches no lower than about 14 cm
KNOB_GRIP_Z = AXIS[2] + KNOB_Z0 + KNOB_H - 0.020
OUT, ABOVE = 0.07, 0.04
TURNS, TURN_S, PER_TURN = 3.25, 3.2, 16  # demo: turns, seconds a turn, waypoints a turn
LEAD_S, LEAD_STEPS = 3.0, 15


def _knob_at(theta: float) -> np.ndarray:
    return np.array([AXIS[0] + ARM_LEN * math.cos(theta), AXIS[1] + ARM_LEN * math.sin(theta), KNOB_GRIP_Z])


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


GRAB = [1.5, 0.8]
TURN_START = 0.5  # after both grips


def _left() -> list[Waypoint]:
    rot = SIDE_GRIP["left"]
    side = np.array([0.0, OUT, 0.0])
    out = _lead_in("left", BODY_GRIP + side, rot, "level, beside the grinder")
    out += [_w(GRAB[0], BODY_GRIP, FINGER_OPEN, "slide in round the body", rot),
            _w(GRAB[1], BODY_GRIP, FINGER_CLOSED, "grip the body: hold it still", rot),
            _w(TURN_START + TURNS * TURN_S + 3.0, BODY_GRIP, FINGER_CLOSED, "keep holding", rot)]
    return out


def _right() -> list[Waypoint]:
    rot = SIDE_GRIP["right"]
    start = _knob_at(START)
    out = _lead_in("right", start + [0.0, 0.0, ABOVE], rot, "level, over the knob")
    out += [_w(GRAB[0], start, FINGER_OPEN, "down round the knob", rot),
            _w(GRAB[1], start, FINGER_CLOSED, "grip the knob", rot),
            _w(TURN_START, start, FINGER_CLOSED, "settle", rot)]
    n = int(round(TURNS * PER_TURN))
    for k in range(1, n + 1):
        out.append(_w(TURN_S / PER_TURN, _knob_at(START - 2 * math.pi * k / PER_TURN), FINGER_CLOSED,
                      "turn the crank clockwise", rot))
    out.append(_w(3.0, _knob_at(START - 2 * math.pi * n / PER_TURN), FINGER_CLOSED, "hold still", rot))
    return out


def demo():
    return {"left": _left(), "right": _right()}
