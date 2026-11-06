"""On the station table stands a care-training manikin (head and shoulders, fixed), its mouth open. In front of it a
feeding spoon with a spoonful of soup (four beads) lies on a spoon rest, and a soup bowl with a long handle stands to
the left.

Task: feed the manikin the spoonful as a carer does: hold the bowl under its chin with one hand, to catch what drips,
while the other brings the spoon to its mouth and tips the soup in. A spoonful given without the bowl under the chin,
or with soup spilt on the table, is not done right.

Run it:  inspect-robots run --embodiment yam_arena -E task=feed_a_manikin_a_spoonful_over_a_held_bowl.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset, on_step,  # noqa: E402
                     plate, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the manikin (fixed): shoulders, neck and a blocky head with an open mouth and a cavity behind it ------------------
MX, MY = 0.47, 0.0  # the head's middle
FACE_X = 0.405  # the face's front
HEAD_TOP = TOP + 0.215
MOUTH_Z0, MOUTH_Z1, MOUTH_HW = TOP + 0.114, TOP + 0.144, 0.0225  # the mouth (wide open): floor, top, half width
CAVITY_X1 = FACE_X + 0.055  # the cavity reaches this far back
PIT = 0.015  # behind the lower lip the mouth's floor drops this far: soup tipped in falls in there
LIP_T = 0.006
JAW_Z0 = TOP + 0.090  # the chin's underside
SKIN, HAIR, DARK, LIP = (0.86, 0.70, 0.60), (0.30, 0.22, 0.15), (0.25, 0.08, 0.08), (0.70, 0.35, 0.35)

# -- the spoon: a long-handled feeding spoon; its frame: origin at the middle of its bowl's floor, the handle along -x
SPOON_BOWL = (0.040, 0.022)  # floor, along x and y
SPOON_WALL_H, SPOON_T = 0.004, 0.002
HANDLE_L, HANDLE_W, HANDLE_H = 0.140, 0.012, 0.010
SPOON_MASS = 0.03
SPOON_AT = (0.33, -0.13)  # its bowl's middle, on the rest
REST_Z = TOP + 0.060  # the spoon's floor on the rest
BEAD_R, BEAD_MASS, N_BEADS = 0.0045, 0.0005, 4

# -- the soup bowl: a bowl with a long stick handle (+y); its frame: origin at the middle of its underside -------------
BOWL_R, BOWL_H, BOWL_WALL = 0.055, 0.040, 0.003
BOWL_HANDLE = (0.016, 0.120, 0.016)  # across, long, tall: from the rim, along +y
BOWL_AT = (0.30, 0.12)
BOWL_MASS = 0.15
CHINA, SOUP, STEEL, REST_C = (0.94, 0.94, 0.92), (0.85, 0.55, 0.20), (0.75, 0.76, 0.78), (0.30, 0.30, 0.32)

# -- scoring ----------------------------------------------------------------------------------------------------------
EATEN_FULL = 3
UNDER_CHIN = 0.07  # the bowl's middle within this (horizontally) of the point under the chin, its rim below the mouth
CHIN_POINT = (FACE_X - 0.03, MY)
SPILT_Z = TOP + 0.045  # a bead lower than this, not in the bowl: spilt
SUCCESS_HOLD_S = 1.0


def _manikin() -> Body:
    ox, oz = MX, TOP  # body frame at the head's middle, on the table
    hw = 0.055  # the head's half width
    back = MX + 0.065
    shapes = [BoxShape((0.0, 0.0, 0.035), (0.10, 0.22, 0.07), SKIN),  # shoulders
              CylShape((0.0, 0.0, 0.08), 0.030, 0.02, SKIN),  # neck
              # the head, built round the mouth and its cavity
              BoxShape(((FACE_X + back) / 2 - ox, 0.0, (MOUTH_Z1 + HEAD_TOP) / 2 - oz), (back - FACE_X, 2 * hw, HEAD_TOP - MOUTH_Z1), SKIN),
              BoxShape(((FACE_X + back) / 2 - ox, 0.0, (JAW_Z0 + MOUTH_Z0 - PIT) / 2 - oz),
                       (back - FACE_X, 2 * hw, MOUTH_Z0 - PIT - JAW_Z0), SKIN),  # the jaw, under the pit
              BoxShape((FACE_X + LIP_T / 2 - ox, 0.0, (MOUTH_Z0 - PIT + MOUTH_Z0) / 2 - oz), (LIP_T, 2 * MOUTH_HW, PIT), LIP),
              *[BoxShape(((FACE_X + back) / 2 - ox, s * (MOUTH_HW + hw) / 2, (MOUTH_Z0 - PIT + MOUTH_Z1) / 2 - oz),
                         (back - FACE_X, hw - MOUTH_HW, MOUTH_Z1 - MOUTH_Z0 + PIT), SKIN) for s in (-1.0, 1.0)],
              BoxShape(((CAVITY_X1 + back) / 2 - ox, 0.0, (MOUTH_Z0 - PIT + MOUTH_Z1) / 2 - oz),
                       (back - CAVITY_X1, 2 * MOUTH_HW, MOUTH_Z1 - MOUTH_Z0 + PIT), DARK),  # the back of the mouth
              BoxShape(((FACE_X + LIP_T + CAVITY_X1) / 2 - ox, 0.0, MOUTH_Z0 - PIT + 0.0005 - oz),
                       (CAVITY_X1 - FACE_X - LIP_T, 2 * MOUTH_HW, 0.001), DARK, collide=False),  # the pit's floor, dark
              BoxShape((back - 0.02 - ox, 0.0, HEAD_TOP + 0.006 - oz), (0.11, 2 * hw + 0.004, 0.012), HAIR),
              *[CylShape((FACE_X - 0.0004 - ox, s * 0.022, TOP + 0.172 - oz), 0.006, 0.001, DARK, axis="X",
                         collide=False) for s in (-1.0, 1.0)]]  # eyes
    return Body.build("manikin", pos=(ox, MY, oz), mass=5.0, shapes=shapes, kinematic=True, friction=(0.5, 0.4),
                      color=SKIN)


def _spoon() -> Body:
    fx, fy = SPOON_BOWL
    shapes = [BoxShape((0.0, 0.0, SPOON_T / 2), (fx, fy, SPOON_T), STEEL),
              *[BoxShape((s * (fx / 2 + SPOON_T / 2), 0.0, SPOON_WALL_H / 2 + SPOON_T / 2), (SPOON_T, fy + 2 * SPOON_T, SPOON_WALL_H + SPOON_T), STEEL)
                for s in (-1.0, 1.0)],
              *[BoxShape((0.0, s * (fy / 2 + SPOON_T / 2), SPOON_WALL_H / 2 + SPOON_T / 2), (fx, SPOON_T, SPOON_WALL_H + SPOON_T), STEEL)
                for s in (-1.0, 1.0)],
              BoxShape((-fx / 2 - SPOON_T - HANDLE_L / 2, 0.0, HANDLE_H / 2), (HANDLE_L, HANDLE_W, HANDLE_H), STEEL)]
    return Body.build("spoon", pos=(SPOON_AT[0], SPOON_AT[1], REST_Z), mass=SPOON_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=STEEL, com=(-0.05, 0.0, 0.004))


def _rest() -> Body:
    """Two small blocks the spoon lies on: under its bowl and under its handle's end."""
    h = REST_Z - TOP
    end = -SPOON_BOWL[0] / 2 - SPOON_T - HANDLE_L + 0.012
    shapes = [BoxShape((0.0, 0.0, h / 2), (0.03, 0.04, h), REST_C), BoxShape((end, 0.0, h / 2), (0.02, 0.03, h), REST_C)]
    return Body.build("spoon_rest", pos=(SPOON_AT[0], SPOON_AT[1], TOP), mass=1.0, shapes=shapes, kinematic=True,
                      friction=(0.6, 0.5), color=REST_C)


def _bowl() -> Body:
    n = 24
    r_mid = BOWL_R - BOWL_WALL / 2
    shapes = [CylShape((0.0, 0.0, 0.003), BOWL_R - 0.002, 0.006, CHINA)]
    for k in range(n):
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        shapes.append(plate((r_mid * u[0], r_mid * u[1], 0.0), (r_mid * u[0], r_mid * u[1], BOWL_H),
                            2 * BOWL_R * math.tan(math.pi / n) + 0.001, BOWL_WALL, u, CHINA))
    shapes.append(BoxShape((0.0, BOWL_R + BOWL_HANDLE[1] / 2 - 0.004, BOWL_H - BOWL_HANDLE[2] / 2), BOWL_HANDLE, CHINA))
    return Body.build("bowl", pos=(BOWL_AT[0], BOWL_AT[1], TOP + 0.0005), mass=BOWL_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=CHINA, com=(0.0, 0.03, 0.015))


def _beads() -> list[Body]:
    out = []
    for i, (dx, dy) in enumerate(((-0.009, -0.005), (-0.009, 0.005), (0.009, -0.005), (0.009, 0.005))[:N_BEADS]):
        out.append(Body.build(f"soup_{i}", pos=(SPOON_AT[0] + dx, SPOON_AT[1] + dy, REST_Z + SPOON_T + BEAD_R + 0.0005),
                              mass=BEAD_MASS, shapes=[SphereShape((0.0, 0.0, 0.0), BEAD_R, SOUP)], friction=(0.3, 0.25),
                              damping=(0.05, 0.2), color=SOUP))
    return out


MANIKIN, REST, BOWL, SPOON = _manikin(), _rest(), _bowl(), _spoon()
BEADS = _beads()
OBJECTS = [MANIKIN, REST, BOWL, SPOON, *BEADS]
BEAD_NAMES = [b.name for b in BEADS]


# -- followed after every control step: the beads, the bowl, the grippers ------------------------------------------------

def _fresh() -> dict:
    return {"first_in": None, "bowl_ok": False, "tcps": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("FEED_DEBUG")), "steps": 0}


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


def _in_mouth(p) -> bool:
    """Swallowed: in the pit behind the lower lip (not still on the spoon)."""
    return (FACE_X + LIP_T - 0.002 < p[0] < CAVITY_X1 and abs(p[1] - MY) < MOUTH_HW and
            MOUTH_Z0 - PIT - 0.002 < p[2] < MOUTH_Z0)


def _in_bowl(p, b_pos, b_rot) -> bool:
    v = b_rot.T @ (np.asarray(p, float) - np.asarray(b_pos, float))
    return math.hypot(v[0], v[1]) < BOWL_R and -0.002 < v[2] < BOWL_H + 0.01


def _bowl_under_chin(b_pos) -> bool:
    return (math.hypot(b_pos[0] - CHIN_POINT[0], b_pos[1] - CHIN_POINT[1]) <= UNDER_CHIN and
            b_pos[2] + BOWL_H < MOUTH_Z0 and b_pos[2] > TOP + 0.02)


def _tally(beads, b_pos, b_rot) -> dict:
    eaten = sum(_in_mouth(p) for p in beads)
    spilt = sum(p[2] < SPILT_Z and not _in_bowl(p, b_pos, b_rot) for p in beads)
    return {"eaten": int(eaten), "spilt": int(spilt)}


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    bw = as_numpy(base.scene["bowl"].data.root_link_pose_w)[0]
    b_pos, b_rot = bw[:3] - origin, rotation(bw[3:7])
    beads = [as_numpy(base.scene[n].data.root_link_pose_w)[0][:3] - origin for n in BEAD_NAMES]
    _EP["tcps"] = _tcps(base, origin)
    t = _tally(beads, b_pos, b_rot)
    if t["eaten"] > 0 and _EP["first_in"] is None:  # the first soup goes in: is the bowl under the chin?
        _EP["first_in"] = True
        _EP["bowl_ok"] = _bowl_under_chin(b_pos)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            sp = as_numpy(base.scene["spoon"].data.root_link_pose_w)[0]
            tl, tr = _EP["tcps"]
            print(f"[feed] t={_DEBUG['steps'] / 20:5.1f}s bowl={np.round(b_pos * 1000).astype(int).tolist()} "
                  f"under_chin={_bowl_under_chin(b_pos)} spoon={np.round((sp[:3] - origin) * 1000).astype(int).tolist()} "
                  f"beads={[np.round(p * 1000).astype(int).tolist() for p in beads]} eaten={t['eaten']} spilt={t['spilt']} "
                  f"bowl_ok={_EP['bowl_ok']} left TCP={np.round(tl * 1000).astype(int).tolist()} "
                  f"right TCP={np.round(tr * 1000).astype(int).tolist()} score={_points(t)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _points(t) -> float:
    if t["eaten"] >= EATEN_FULL and t["spilt"] == 0 and _EP["bowl_ok"]:
        return 1.0
    if t["eaten"] >= EATEN_FULL and t["spilt"] <= 1:
        return 0.5
    return 0.25 if t["eaten"] >= 1 else 0.0


def score(objects) -> float:
    """1.0 with at least 3 of the 4 beads of soup in the manikin's mouth, none spilt (lower than 4.5 cm above the table and
    not in the bowl), and the bowl held under the chin when the first went in; 0.5 with at least 3 in the mouth and at
    most 1 spilt; 0.25 with at least 1 in the mouth; else 0.0."""
    b = objects["bowl"]
    return _points(_tally([objects[n].pos for n in BEAD_NAMES], np.asarray(b.pos, float), rotation(b.quat_xyzw)))


def success(objects) -> bool:
    """Full score with the soup still."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in BEAD_NAMES)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the bowl's handle near its end, lifts the bowl and holds it under the manikin's
# chin. The right hand comes straight down on the spoon's handle, lifts the spoon level off its rest, carries it to the
# mouth, puts it in and tips it (a turn about the handle), so the soup runs off into the mouth; then it brings the spoon
# out level.


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


SPOON_GRIP = grip_rotation(0.0)  # straight down, fingers closing along y (across the handle)
BOWL_GRIP = grip_rotation(-math.pi / 2)  # straight down, fingers closing along x (across the bowl's handle)
GRIP_BACK = 0.10  # the right hand holds the spoon's handle this far behind its bowl's middle
HANDLE_IN_HAND = -0.005  # the right TCP at the handle's middle height (the spoon tips about it)
BOWL_GRIP_AT = BOWL_R + BOWL_HANDLE[1] - 0.025  # the left hand holds the bowl's handle this far from its middle
BOWL_HOLD = (CHIN_POINT[0] - 0.02, CHIN_POINT[1], MOUTH_Z0 - 0.035 - BOWL_H)  # the bowl's underside, under the chin
IN_MOUTH = 0.050  # how far the spoon goes in from just outside (its bowl then wholly past the lip)
TIP = math.radians(60.0)
LEAD_S, LEAD_STEPS = 3.0, 15


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


def _spoon_tcp(bowl_mid) -> np.ndarray:
    """The right TCP on the handle with the spoon's bowl (floor middle) at bowl_mid, level."""
    return np.asarray(bowl_mid, float) + [-GRIP_BACK, 0.0, HANDLE_H + HANDLE_IN_HAND - 0.0]


def _bowl_tcp(under) -> np.ndarray:
    """The left TCP on the bowl's handle with the bowl's underside middle at ``under``."""
    return np.asarray(under, float) + [0.0, BOWL_GRIP_AT, BOWL_H - BOWL_HANDLE[2] / 2 + 0.003]


BOWL_READY = LEAD_S + 1.0 + 0.8 + 1.2 + 2.0


def _left(total: float) -> list[Waypoint]:
    start = np.array([BOWL_AT[0], BOWL_AT[1], TOP + 0.0005])
    g0 = _bowl_tcp(start)
    out = _lead_in("left", g0 + [0.0, 0.0, 0.05], BOWL_GRIP, "over the bowl's handle")
    out += [_w(1.0, g0, FINGER_OPEN, "down round the handle", BOWL_GRIP),
            _w(0.8, g0, FINGER_CLOSED, "grip it", BOWL_GRIP),
            _w(1.2, g0 + [0.0, 0.0, 0.05], FINGER_CLOSED, "lift the bowl", BOWL_GRIP),
            _w(2.0, _bowl_tcp(BOWL_HOLD), FINGER_CLOSED, "under the manikin's chin", BOWL_GRIP)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), _bowl_tcp(BOWL_HOLD), FINGER_CLOSED, "hold it there, to catch any drips", BOWL_GRIP))
    return out


def _right() -> list[Waypoint]:
    rest = np.array([SPOON_AT[0], SPOON_AT[1], REST_Z])
    mouth_z = MOUTH_Z0 + 0.008  # the spoon's floor, in the mouth (over the pit)
    outside = np.array([FACE_X - SPOON_BOWL[0] / 2 - 0.02, MY, mouth_z])
    inside = outside + [0.02 + IN_MOUTH, 0.0, 0.0]
    out = _lead_in("right", _spoon_tcp(rest) + [0.0, 0.0, 0.05], SPOON_GRIP, "over the spoon's handle")
    out += [_w(1.0, _spoon_tcp(rest), FINGER_OPEN, "down round the handle", SPOON_GRIP),
            _w(0.8, _spoon_tcp(rest), FINGER_CLOSED, "grip it", SPOON_GRIP),
            _w(1.2, _spoon_tcp(rest + [0.0, 0.0, 0.03]), FINGER_CLOSED, "lift the spoon level", SPOON_GRIP)]
    t = sum(w.duration for w in out)
    out.append(_w(max(BOWL_READY - t, 0.3) + 0.5, _spoon_tcp(rest + [0.0, 0.0, 0.03]), FINGER_CLOSED,
                  "wait for the bowl under the chin", SPOON_GRIP))
    out += [_w(2.5, _spoon_tcp(outside), FINGER_CLOSED, "carry it level to the mouth", SPOON_GRIP),
            _w(1.2, _spoon_tcp(inside), FINGER_CLOSED, "into the mouth", SPOON_GRIP)]
    for k in range(1, 4):  # tip it about the handle (toward the left)
        out.append(_w(0.5, _spoon_tcp(inside), FINGER_CLOSED, "tip the soup in", _rx(TIP * k / 3) @ SPOON_GRIP))
    out += [_w(1.0, _spoon_tcp(inside), FINGER_CLOSED, "let it run off", _rx(TIP) @ SPOON_GRIP),
            _w(0.8, _spoon_tcp(inside), FINGER_CLOSED, "level again", SPOON_GRIP),
            _w(1.2, _spoon_tcp(outside), FINGER_CLOSED, "bring the spoon out", SPOON_GRIP),
            _w(3.0, _spoon_tcp(outside), FINGER_CLOSED, "hold it", SPOON_GRIP)]
    return out


def demo():
    right = _right()
    total = sum(w.duration for w in right)
    return {"left": _left(total), "right": right}
