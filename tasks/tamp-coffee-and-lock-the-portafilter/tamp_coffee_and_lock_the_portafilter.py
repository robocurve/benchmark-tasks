"""On the station table, an espresso machine stands at the back with its group head facing down. On the counter in front
of it, a portafilter (a basket filled level with loose ground coffee, a spout under it and a long handle) rests on its
spout and on the end of its handle, tipped about 8 degrees; a tamper stands to the right.

Task: tamp the coffee level with the tamper, then lock the portafilter into the group head (ears up past the head's
two lips, a turn of the handle so that they rest on them) and let go. Resting on the counter, the basket is tipped,
and a tamp is only good if it is pressed square into a level basket: the basket has to be held level while the
tamper presses. Only pressing done by the tamper's base, square to the basket and before the portafilter goes into
the group head, counts as tamping.

Run it:  inspect-robots run --embodiment yam_arena -E task=tamp_coffee_and_lock_the_portafilter.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset,  # noqa: E402
                     on_step, rotation, set_slide_target, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the portafilter. Its frame: origin on the basket's axis at the rim, z up along the axis, the handle along +y ------
BASKET_R, BASKET_WALL, BASKET_H, BASKET_FLOOR = 0.035, 0.003, 0.025, 0.004  # outer radius, wall, height, floor
SPOUT = (0.014, 0.024, 0.022)  # a block under the basket: it stands on this and on the handle's end
HANDLE_L, HANDLE_W, HANDLE_H, HANDLE_Z = 0.150, 0.020, 0.022, -0.012  # from the basket's side, out along +y
EAR = (0.012, 0.014, 0.004)  # two bayonet ears on the rim, at +-x: radial, along the rim, tall
PF_MASS = 0.08
PF_COM = (0.0, 0.015, -0.015)  # near the basket: a steel basket and brass body outweigh the handle
SPOUT_EDGE = (SPOUT[1] / 2, -(BASKET_H + SPOUT[2]))  # (y, z) of the spout's bottom edge on the handle side
HANDLE_END = (BASKET_R + HANDLE_L, HANDLE_Z - HANDLE_H / 2)  # (y, z) of the handle end's bottom edge
REST_TILT = math.atan2(SPOUT_EDGE[1] - HANDLE_END[1], HANDLE_END[0] - SPOUT_EDGE[0])  # about x: the handle end down
BX, BY = 0.26, 0.04  # the basket's axis on the counter at the start
GRIP_Y = 0.155  # where the demo holds the handle, from the basket's axis (the two wrists need ~15 cm between them)

# -- the ground coffee: a disc filling the basket, its top level with the rim; a joint lets it be pressed down the
# basket's axis and keeps it where it is pressed (it springs back 1 mm when the tamper lifts) --------------------------
BED_R, BED_T = 0.031, 0.006
BED_START = -BED_T / 2  # its centre below the rim at the start
BED_TRAVEL = 0.012  # it can be pressed down at most 12 mm
BED_K, BED_D, BED_SET = 4000.0, 20.0, 0.001  # spring (N/m) and damping (N s/m) toward where it was last pressed

# -- the tamper: a steel base, a wooden neck, a knob; frame at the base's underside ----------------------------------
TAMPER_R, TAMPER_BASE_H, NECK_R, NECK_H, KNOB_R = 0.028, 0.012, 0.009, 0.050, 0.015
TAMPER_MASS = 0.15
TAMPER_XY = (0.30, -0.15)

# -- the espresso machine: a group head facing down at (HX, HY), its lips' underside at Z0. The head is a ring with two
# lips at its bottom, with gaps between them; the portafilter's ears go up through the gaps, the handle is turned and
# the ears rest on the lips -----------------------------------------------------------------------------------------
HX, HY, Z0 = 0.36, 0.10, TOP + 0.11
LIP_IN, RING_IN, RING_T = 0.040, 0.050, 0.003  # lips' inner radius (5 mm round the basket), ring's inner radius
LIP_H, CEILING = 0.003, 0.014  # lips 3 mm thick; the head's ceiling 14 mm above their underside
BOSS = ((0.029, 0.005), (0.026, 0.003), (0.022, 0.003))  # under the ceiling, a stepped boss (radius, height of each
# step, top down) that the basket's rim goes up round: it centres the basket, 3 mm round it at the top
LIPS = ((30.0, 68.0), (210.0, 248.0))  # degrees round the head from +x: the two lips the ears end up on; elsewhere
# open, so that the ears pass up at +-x and the handle (at +y, turning toward the front) passes under the head
STEEL, CHROME, BLACK, WOOD, COFFEE, HANDLE_C = (0.70, 0.71, 0.74), (0.80, 0.81, 0.84), (0.13, 0.13, 0.14), \
    (0.50, 0.33, 0.18), (0.30, 0.18, 0.10), (0.10, 0.10, 0.11)

# -- scoring --------------------------------------------------------------------------------------------------------
DEPTH_TAMPED = 0.006  # pressed at least 6 mm down by the tamper
TAMP_OFF = 0.007  # the tamper's base centre within this of the basket's axis (it is 8 mm narrower than the basket)
TAMP_GAP = 0.004  # ... its underside within this of the coffee's surface (or pressing into it)
TAMP_SQUARE = math.radians(5.0)  # ... and its axis within this of the basket's
HEAD_NEAR = 0.03  # the portafilter has gone into the group head once its basket's axis is this close to the head's
LEVEL_TILT = math.radians(5.0)  # the basket within 5 degrees of level whenever the coffee was being pressed down
LOCK_TURN = (35.0, 60.0)  # degrees the handle is turned from where the ears pass up: the ears over the lips
LOCK_OFF, LOCK_TILT = 0.006, math.radians(8.0)
LET_GO = 0.04
SUCCESS_HOLD_S = 1.0


def _quat_x(a: float) -> tuple[float, float, float, float]:
    return (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))


def _ring(r_mid, thick, z0, z1, color, n=16, sectors=None) -> list:
    """A ring of boxes about the z axis (or only over the ``sectors``, in degrees from +x)."""
    out = []
    spans = sectors or ((0.0, 360.0),)
    for a0, a1 in spans:
        k = max(2, round((a1 - a0) / (360.0 / n)))
        step = (a1 - a0) / k
        chord = 2 * r_mid * math.tan(math.radians(step) / 2) * 1.05
        for i in range(k):
            a = math.radians(a0 + step * (i + 0.5))
            out.append(BoxShape((r_mid * math.cos(a), r_mid * math.sin(a), (z0 + z1) / 2), (thick, chord, z1 - z0), color,
                                yaw=a))
    return out


def _portafilter() -> Body:
    r_mid = BASKET_R - BASKET_WALL / 2
    shapes = _ring(r_mid, BASKET_WALL, -BASKET_H, 0.0, STEEL)
    shapes += [CylShape((0.0, 0.0, -BASKET_H + BASKET_FLOOR / 2), BASKET_R, BASKET_FLOOR, STEEL),
               BoxShape((0.0, 0.0, -BASKET_H - SPOUT[2] / 2), SPOUT, CHROME),
               BoxShape((0.0, BASKET_R + HANDLE_L / 2, HANDLE_Z), (HANDLE_W, HANDLE_L, HANDLE_H), HANDLE_C)]
    shapes += [BoxShape((s * (BASKET_R + EAR[0] / 2), 0.0, -EAR[2] / 2), EAR, CHROME) for s in (-1.0, 1.0)]
    y, z = SPOUT_EDGE
    c, s = math.cos(REST_TILT), math.sin(REST_TILT)
    lift = -(y * s + z * c)  # the origin's height above the counter with the spout edge and handle end on it
    return Body.build("portafilter", pos=(BX, BY, TOP + lift + 0.0005), mass=PF_MASS, shapes=shapes, friction=(1.2, 1.0),
                      color=STEEL, com=PF_COM, start_xyzw=_quat_x(REST_TILT))


def _coffee() -> Body:
    pf = PORTAFILTER
    pos = np.asarray(pf.pos) + rotation(pf.start_xyzw) @ np.array([0.0, 0.0, BED_START])
    shapes = [CylShape((0.0, 0.0, 0.0), BED_R, BED_T, COFFEE)]
    return Body.build("coffee", pos=tuple(pos.tolist()), mass=0.005, shapes=shapes, friction=(0.8, 0.7), color=COFFEE,
                      start_xyzw=pf.start_xyzw, ignore=("portafilter", "machine"))


def _tamper() -> Body:
    shapes = [CylShape((0.0, 0.0, TAMPER_BASE_H / 2), TAMPER_R, TAMPER_BASE_H, STEEL),
              CylShape((0.0, 0.0, TAMPER_BASE_H + NECK_H / 2), NECK_R, NECK_H, WOOD),
              SphereShape((0.0, 0.0, TAMPER_BASE_H + NECK_H + KNOB_R * 0.6), KNOB_R, WOOD)]
    return Body.build("tamper", pos=(TAMPER_XY[0], TAMPER_XY[1], TOP + 0.0005), mass=TAMPER_MASS, shapes=shapes,
                      friction=(1.2, 1.0), color=STEEL)


def _boss(top: float) -> list:
    out = []
    for r, h in BOSS:
        out.append(CylShape((0.0, 0.0, top - h / 2), r, h, CHROME))
        top -= h
    return out


def _machine() -> Body:
    """The espresso machine, frame at (HX, HY) on the counter."""
    z0 = Z0 - TOP
    shapes = _ring((LIP_IN + RING_IN) / 2, RING_IN - LIP_IN, z0, z0 + LIP_H, CHROME, n=36, sectors=LIPS)
    shapes += _ring(RING_IN - RING_T / 2, RING_T, z0 + LIP_H, z0 + CEILING, CHROME, n=36, sectors=LIPS)  # lips' posts
    shapes += [CylShape((0.0, 0.0, z0 + CEILING + 0.003), RING_IN, 0.006, CHROME),  # the ceiling (gasket)
               *_boss(z0 + CEILING),  # the shower screen's boss
               CylShape((0.0, 0.0, z0 + CEILING + 0.006 + 0.020), 0.040, 0.040, CHROME),  # the group above it
               BoxShape((0.075, 0.0, z0 + 0.035), (0.070, 0.050, 0.030), CHROME),  # its neck to the body
               BoxShape((0.160, 0.0, (z0 + 0.12) / 2), (0.110, 0.200, z0 + 0.12), BLACK),  # the body
               BoxShape((0.020, 0.0, 0.010), (0.090, 0.150, 0.020), BLACK),  # the drip tray
               BoxShape((0.020, 0.0, 0.0202), (0.080, 0.140, 0.0004), CHROME, collide=False)]  # its grate
    return Body.build("machine", pos=(HX, HY, TOP), mass=5.0, shapes=shapes, kinematic=True, friction=(0.05, 0.05),
                      slick=True, color=BLACK)  # polished: the portafilter turns in it easily


MACHINE = _machine()
PORTAFILTER = _portafilter()
COFFEE_BED = _coffee()
TAMPER = _tamper()
OBJECTS = [MACHINE, PORTAFILTER, COFFEE_BED, TAMPER]
thread("bed", "portafilter", "coffee", anchor0=(0.0, 0.0, BED_START), axis0=(0.0, 0.0, -1.0), anchor1=(0.0, 0.0, 0.0),
       axis1=(0.0, 0.0, -1.0), slide_limits=(0.0, BED_TRAVEL), turn_damping=0.5, slide_stiffness=BED_K,
       slide_damping=BED_D, slide_max_force=1.0e3, always_on=True)


# -- followed after every control step: how far the coffee is pressed, how level the basket was while it was pressed,
# and where the grippers are ---------------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"tcps": None, "set": 0.0, "depth": 0.0, "tilt": 0.0, "tamped": 0.0, "square": True, "in_head_once": False}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("COFFEE_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    set_slide_target("bed", 0.0)  # the coffee back level with the rim


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _depth(pf_pos, pf_rot, bed_pos) -> float:
    """How far the coffee is pressed down the basket's axis."""
    start = np.asarray(pf_pos, float) + pf_rot @ np.array([0.0, 0.0, BED_START])
    return float((np.asarray(bed_pos, float) - start) @ (pf_rot @ np.array([0.0, 0.0, -1.0])))


def _tilt(pf_rot) -> float:
    return float(math.acos(max(-1.0, min(1.0, pf_rot[2, 2]))))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _EP["tcps"] = _tcps(base, origin)
    p = as_numpy(base.scene["portafilter"].data.root_link_pose_w)[0]
    b = as_numpy(base.scene["coffee"].data.root_link_pose_w)[0]
    pr = rotation(p[3:7])
    depth = _depth(p[:3] - origin, pr, b[:3] - origin)
    pf_pos = p[:3] - origin
    if math.hypot(pf_pos[0] - HX, pf_pos[1] - HY) <= HEAD_NEAR and pf_pos[2] >= Z0 - 0.02:
        _EP["in_head_once"] = True  # gone (at least part way) into the group head: no tamping after this
    t = as_numpy(base.scene["tamper"].data.root_link_pose_w)[0]
    tq = pr.T @ (t[:3] - origin - pf_pos)  # the tamper base's underside centre, in the portafilter's frame
    t_axis = rotation(t[3:7])[:, 2]
    square = math.acos(max(-1.0, min(1.0, float(t_axis @ pr[:, 2]))))
    tamper_on = (math.hypot(tq[0], tq[1]) <= TAMP_OFF and float(tq[2]) <= -max(_EP["depth"], depth) + TAMP_GAP
                 and not _EP["in_head_once"])
    if depth > _EP["depth"] + 0.0002 and depth > 0.001:  # being pressed down: how level is the basket?
        _EP["tilt"] = max(_EP["tilt"], _tilt(pr))
        if tamper_on:  # pressed by the tamper's base: tamping; was it square to the basket?
            _EP["tamped"] = max(_EP["tamped"], depth)
            _EP["square"] = _EP["square"] and square <= TAMP_SQUARE
    _EP["depth"] = max(_EP["depth"], depth)
    if depth > _EP["set"] + BED_SET:  # pressed further: it stays there (springing back by BED_SET)
        _EP["set"] = depth - BED_SET
        set_slide_target("bed", _EP["set"])
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            t = as_numpy(base.scene["tamper"].data.root_link_pose_w)[0]
            st = _status_from(p[:3] - origin, pr, b[:3] - origin)
            tl, tr = _EP["tcps"]
            grip = p[:3] - origin + pr @ np.array([0.0, GRIP_Y, HANDLE_Z])
            tb = pr.T @ (t[:3] - origin - (p[:3] - origin))  # the tamper's underside in the portafilter's frame
            print(f"[coffee] t={_DEBUG['steps'] / 20:5.1f}s tamped={_EP['tamped'] * 1000:4.1f}mm square={_EP['square']} "
                  f"head={_EP['in_head_once']} tamper_on={tamper_on} sq={math.degrees(square):4.1f} "
                  f"pf={np.round((p[:3] - origin) * 1000).astype(int).tolist()} "
                  f"tamper in pf={np.round(tb * 1000).astype(int).tolist()} "
                  f"tilt={math.degrees(_tilt(pr)):4.1f} (handle {math.degrees(math.asin(pr[2, 1])):+4.1f}) depth={depth * 1000:4.1f}mm (max {_EP['depth'] * 1000:4.1f}, "
                  f"tilt while pressed {math.degrees(_EP['tilt']):4.1f}) left TCP-grip="
                  f"{np.round((tl - grip) * 1000).astype(int).tolist()} right TCP-tamper="
                  f"{np.round((tr - (t[:3] - origin)) * 1000).astype(int).tolist()} turn={st['turn']:5.1f} "
                  f"in_head={st['in_head']} locked={st['locked']} let_go={st['let_go']} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------


def _status_from(pf_pos, pf_rot, bed_pos) -> dict:
    pf_pos = np.asarray(pf_pos, float)
    depth = max(_EP["depth"], _depth(pf_pos, pf_rot, bed_pos))
    handle = pf_rot @ np.array([0.0, 1.0, 0.0])
    turn = math.degrees(math.atan2(-handle[0], handle[1]))  # the handle's turn from +y, counterclockwise from above
    in_head = (math.hypot(pf_pos[0] - HX, pf_pos[1] - HY) <= LOCK_OFF and _tilt(pf_rot) <= LOCK_TILT
               and Z0 + LIP_H <= pf_pos[2] <= Z0 + CEILING + 0.001)
    locked = in_head and LOCK_TURN[0] <= turn <= LOCK_TURN[1]
    let_go = True
    tcps = _EP["tcps"]
    if tcps is not None:
        a = pf_pos + pf_rot @ np.array([0.0, BASKET_R, HANDLE_Z])
        d = pf_rot @ np.array([0.0, 1.0, 0.0])
        for t in tcps:
            v = np.asarray(t, float) - a
            s = float(np.clip(v @ d, 0.0, HANDLE_L))
            let_go &= float(np.linalg.norm(v - s * d)) >= LET_GO and float(np.linalg.norm(t - pf_pos)) >= LET_GO
    return {"depth": depth, "tamped": _EP["tamped"], "level": _EP["tilt"] <= LEVEL_TILT and _EP["square"],
            "turn": turn, "in_head": in_head, "locked": locked, "let_go": let_go}


def _status(objects) -> dict:
    p, b = objects["portafilter"], objects["coffee"]
    return _status_from(p.pos, rotation(p.quat_xyzw), b.pos)


def _points(st) -> float:
    tamped = st["tamped"] >= DEPTH_TAMPED
    if tamped and st["level"] and st["locked"] and st["let_go"]:
        return 1.0
    if tamped and st["level"]:
        return 0.5
    return 0.25 if tamped else 0.0


def score(objects) -> float:
    """Tamped: the coffee pressed at least 6 mm down by the tamper, counting only pressing done with the tamper's base
    on the coffee (its centre within 7 mm of the basket's axis, its underside within 4 mm of the coffee's surface)
    before the portafilter has gone into the group head (its basket within 3 cm of the head's axis). Level: the
    basket within 5 degrees of level whenever the coffee was being pressed down, and the tamper within 5 degrees of
    square to the basket while it pressed. 1.0 with the coffee tamped, level, the portafilter locked in the group
    head (ears over the lips) and both grippers let go; 0.5 tamped and level; 0.25 tamped; else 0.0."""
    return _points(_status(objects))


def success(objects) -> bool:
    """Full score with the portafilter still."""
    return _points(_status(objects)) >= 1.0 and objects["portafilter"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the portafilter's handle, fingers closing across it, and lifts the handle until
# the basket is level, its spout still on the counter. The right hand takes the tamper by its neck, sets it in the
# basket and presses the coffee down, lifts it out and stands it back. The left hand lifts the portafilter level under
# the group head, ears at the gaps, pushes it up into the head, turns the handle a little over 50 degrees, lets go and
# backs away.


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


DOWN_L, DOWN_R = grip_rotation(-math.pi / 2), grip_rotation(math.pi / 2)  # straight down, fingers closing along x
L_FIX = np.array([0.0, 0.0, 0.0])  # where the left hand misses by, holding the portafilter: aim the other way
R_FIX = np.array([0.0, 0.0, 0.0])  # ... and the right hand, holding the tamper
L_PITCH = 0.0  # rad: tip the left grip against the portafilter's droop in it, once it is off the counter
L_YAW = -0.078  # rad: the grip holds the handle turned 4.5 degrees about the vertical: turn it back
LEVEL_UP = 0.004  # the hand sags under the handle: aim it this much higher to hold the basket level
TAMPER_IN_HAND = TAMPER_BASE_H + NECK_H / 2  # the right TCP this far above the tamper's underside
PRESS = 0.010 + 0.008  # the tamper's underside aimed this far below the rim (the arm gives: it gets ~8 mm; harder
# presses make the portafilter slip in the other hand)
TURN = math.radians(55.0)  # the handle lags the hand and springs back ~7 degrees each: it ends near 45
INSERT_PAST = 0.012  # aim the rim this far past the head's ceiling: the hand sags ~6 mm and holds the handle 4 mm
# above its middle; the ceiling stops it
LEAD_S, LEAD_STEPS = 3.0, 15
PICK = [1.0, 0.8, 0.3]
ABOVE = 0.05


def _pf_level_origin() -> np.ndarray:
    """The portafilter's origin with its basket level over where it lay, on its spout (which slides a little)."""
    return np.array([BX, BY, TOP - SPOUT_EDGE[1] + 0.0005])


def _grip_rest() -> np.ndarray:
    """The left TCP to take the handle as it lies: 4 mm above its middle (the fingertips clear of the counter)."""
    pf = PORTAFILTER
    return np.asarray(pf.pos) + rotation(pf.start_xyzw) @ np.array([0.0, GRIP_Y, HANDLE_Z]) + [0.0, 0.0, 0.004]


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


def _right() -> tuple[list[Waypoint], float, float]:
    """The right hand's waypoints, their total time and when it has lifted the tamper out of the basket."""
    rot = DOWN_R
    pick = np.array([TAMPER_XY[0], TAMPER_XY[1], TOP + TAMPER_IN_HAND])
    rim = _pf_level_origin()
    over = np.array([rim[0], rim[1], rim[2] + TAMPER_IN_HAND]) + R_FIX  # the tamper's underside at the rim
    out = _lead_in("right", pick + [0.0, 0.0, ABOVE], rot, "over the tamper")
    out += [_w(PICK[0], pick, FINGER_OPEN, "down round its neck", rot),
            _w(PICK[1], pick, FINGER_CLOSED, "grip it", rot),
            _w(PICK[2], pick, FINGER_CLOSED, "settle", rot),
            _w(1.0, pick + [0.0, 0.0, 0.08], FINGER_CLOSED, "lift it", rot),
            _w(2.0, over + [0.0, 0.0, 0.03], FINGER_CLOSED, "over the basket", rot),
            _w(1.0, over + [0.0, 0.0, 0.002], FINGER_CLOSED, "into the basket, on the coffee", rot),
            _w(1.5, over - [0.0, 0.0, PRESS], FINGER_CLOSED, "press the coffee down", rot),
            _w(0.6, over - [0.0, 0.0, PRESS], FINGER_CLOSED, "hold the press", rot),
            _w(1.0, over + [0.0, 0.0, 0.05], FINGER_CLOSED, "lift it out", rot)]
    out_at = sum(w.duration for w in out)
    out += [_w(2.0, pick + [0.0, 0.0, 0.05], FINGER_CLOSED, "back over its place", rot),
            _w(1.0, pick + [0.0, 0.0, 0.002], FINGER_CLOSED, "stand it down", rot),
            _w(0.5, pick + [0.0, 0.0, 0.002], FINGER_OPEN, "let go", rot),
            _w(1.0, pick + [0.0, 0.0, 0.08], FINGER_OPEN, "up and away", rot)]
    return out, sum(w.duration for w in out), out_at


RIGHT, RIGHT_S, TAMPED_AT = _right()


def _left() -> list[Waypoint]:
    level = _pf_level_origin()
    grip_level = level + np.array([0.0, GRIP_Y, HANDLE_Z + LEVEL_UP]) + L_FIX
    rest = _grip_rest()
    level_rot = _rz(L_YAW) @ _rx(-REST_TILT) @ DOWN_L  # the grip holds the handle as it lay: level it
    carry = _rx(-L_PITCH) @ level_rot

    def head_grip(turn: float, rim_z: float) -> np.ndarray:
        """The left TCP with the basket's axis on the head's, its rim at ``rim_z``, the handle turned ``turn``."""
        return np.array([HX - GRIP_Y * math.sin(turn), HY + GRIP_Y * math.cos(turn), rim_z + HANDLE_Z]) + L_FIX

    out = _lead_in("left", rest + [0.0, 0.0, ABOVE], DOWN_L, "over the portafilter's handle")
    out += [_w(PICK[0], rest, FINGER_OPEN, "down round the handle", DOWN_L),
            _w(PICK[1], rest, FINGER_CLOSED, "grip it", DOWN_L),
            _w(PICK[2], rest, FINGER_CLOSED, "settle", DOWN_L),
            _w(1.5, grip_level, FINGER_CLOSED, "lift the handle: the basket level, on its spout", level_rot)]
    held = LEAD_S + sum(PICK) + 1.5
    out += [_w(max(TAMPED_AT - held, 0.5) + 0.3, grip_level, FINGER_CLOSED, "hold it level while the coffee is tamped",
               level_rot),
            _w(1.2, grip_level + [0.0, 0.0, 0.05], FINGER_CLOSED, "lift it off the counter", carry),
            _w(2.5, head_grip(0.0, Z0 - 0.03), FINGER_CLOSED, "under the group head, ears at the gaps", carry),
            _w(1.2, head_grip(0.0, Z0 + CEILING + INSERT_PAST), FINGER_CLOSED, "up into the head", carry)]
    for k in range(1, 6):  # turn the handle about the head's axis (a fifth at a time)
        a = TURN * k / 5
        out.append(_w(0.5, head_grip(a, Z0 + CEILING + INSERT_PAST), FINGER_CLOSED, "turn the handle to lock it",
                      _rz(a) @ carry))
    end = head_grip(TURN, Z0 + CEILING + INSERT_PAST)
    out += [_w(0.6, end, FINGER_OPEN, "let go", _rz(TURN) @ carry),
            _w(1.2, end + [0.0, 0.0, 0.06], FINGER_OPEN, "back away", _rz(TURN) @ carry),
            _w(4.0, end + [0.0, 0.0, 0.06], FINGER_OPEN, "wait", _rz(TURN) @ carry)]
    return out


def demo():
    left = _left()
    total = sum(w.duration for w in left)
    right = list(RIGHT) + [_w(max(total - RIGHT_S, 0.5), RIGHT[-1].pos, FINGER_OPEN, "wait", RIGHT[-1].yaw)]
    return {"left": left, "right": right}
