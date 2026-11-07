"""On the station table stands a shoe-lacing practice board: an upright board with two metal eyelets side by side. A
lace is knotted behind the first eyelet and comes through it toward the arms, hangs in a loop and ends in a long stiff
tip (an aglet, 9 cm long and 1 cm thick, like a children's lacing needle) that rests on two pegs, pointing at the
second eyelet.

Task: lace the second eyelet. One hand pushes the aglet through the eyelet from the front and keeps hold of it; behind
the board the other hand takes the tip that comes through, and the first lets go; the second pulls the lace on through.
The handover through the hole is the point: the aglet is never let go of on its way through.

Run it:  inspect-robots run --embodiment yam_arena -E task=lace_a_board_through_the_next_eyelet.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, ball, on_reset,  # noqa: E402
                     on_step, plate, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the board: upright, in the x-z plane at y = 0, its front (toward the arms) at +y --------------------------------
BOARD_X = (0.20, 0.40)
BOARD_T = 0.004
EZ = 0.12  # the eyelets' middles above the table
E1, E2 = 0.26, 0.34  # the eyelets, along x
HOLE = 0.0125  # half the square hole each grommet sits in
RING_RI, RING_T, RING_L = 0.011, 0.0015, 0.008  # the grommets: 22 mm inside, 8 mm long through the board
FUNNEL_R, FUNNEL_L = 0.018, 0.006  # the second eyelet's lead-in on the board's front: 36 mm across at its mouth
BOARD_TOP = EZ + HOLE + 0.008
WOOD, METAL, DARK = (0.72, 0.58, 0.40), (0.70, 0.71, 0.74), (0.20, 0.21, 0.23)

# -- the lace: a chain of short stiff pieces on ball joints, knotted behind the first eyelet, ending in the aglet ---
LACE_W = 0.006
LACE_MASS = 0.0012  # each piece (about 2 cm)
KNOT_R = 0.014
AGLET_L, AGLET_R, AGLET_MASS = 0.09, 0.005, 0.008
Z = TOP + EZ
AGLET_BACK = np.array([E2, 0.135, Z])  # the aglet on its pegs: its back end; its tip 9 cm nearer the board
PATH = [np.array(p) for p in ((E1, -0.006, Z), (E1, 0.02, Z), (E1 + 0.02, 0.08, Z - 0.08), (E2, 0.165, Z - 0.05),
                              tuple(AGLET_BACK))]  # the lace at the start, from the knot to the aglet: it hangs
# back and down from the aglet's back end, behind the pegs, so it pulls the aglet back and down, not sideways
SEG = 0.02
PEG_Y = (0.048, 0.123)  # the aglet's two pegs, along y: under its tip, and 1.2 cm from its back end
WHITE, BLUE = (0.93, 0.93, 0.90), (0.15, 0.30, 0.70)

# -- scoring ----------------------------------------------------------------------------------------------------------
BACK_Y = -RING_L / 2  # the eyelets' back faces
PULLED = 0.012  # the aglet's back end this far past them: all of it through, the lace following
THROUGH = 0.01  # the aglet's tip this far past them
HELD = 0.03  # held: a TCP within this of the aglet; from its tip coming through until it is pulled through, it must be
SUCCESS_HOLD_S = 1.0


def _quat_xyzw_x_to(d) -> tuple[float, float, float, float]:
    d = np.asarray(d, float) / np.linalg.norm(d)
    c = float(d[0])
    axis = np.cross([1.0, 0.0, 0.0], d)
    s = float(np.linalg.norm(axis))
    if s < 1e-9:
        return (0.0, 0.0, 0.0, 1.0) if c > 0 else (0.0, 0.0, 1.0, 0.0)
    a = math.atan2(s, c)
    axis = axis / s * math.sin(a / 2)
    return (float(axis[0]), float(axis[1]), float(axis[2]), math.cos(a / 2))


def _board() -> Body:
    x0, x1 = BOARD_X
    cx = (x0 + x1) / 2
    shapes = [BoxShape((0.0, 0.0, (EZ - HOLE) / 2), (x1 - x0, BOARD_T, EZ - HOLE), WOOD),
              BoxShape((0.0, 0.0, (EZ + HOLE + BOARD_TOP) / 2), (x1 - x0, BOARD_T, BOARD_TOP - EZ - HOLE), WOOD),
              BoxShape((0.0, 0.0, 0.006), (x1 - x0, 0.06, 0.012), WOOD)]  # its foot
    for a, b in ((x0, E1 - HOLE), (E1 + HOLE, E2 - HOLE), (E2 + HOLE, x1)):
        shapes.append(BoxShape(((a + b) / 2 - cx, 0.0, EZ), (b - a, BOARD_T, 2 * HOLE), WOOD))
    n = 10
    for ex in (E1, E2):
        for k in range(n):
            a = 2 * math.pi * k / n
            u = (math.cos(a), 0.0, math.sin(a))
            r = RING_RI + RING_T / 2
            p = (ex - cx + r * u[0], 0.0, EZ + r * u[2])
            width = 2 * r * math.tan(math.pi / n) + 0.001
            shapes.append(plate((p[0], -RING_L / 2, p[2]), (p[0], RING_L / 2, p[2]), width, RING_T, u, METAL))
            if ex == E2:  # its funnel
                r1 = FUNNEL_R + RING_T / 2
                q = (ex - cx + r1 * u[0], 0.0, EZ + r1 * u[2])
                shapes.append(plate((p[0], RING_L / 2, p[2]), (q[0], RING_L / 2 + FUNNEL_L, q[2]),
                                    2 * r1 * math.tan(math.pi / n) + 0.001, RING_T, u, METAL))
    return Body.build("board", pos=(cx, 0.0, TOP), mass=1.0, shapes=shapes, kinematic=True, friction=(0.15, 0.15),
                      slick=True, color=WOOD)  # polished: the aglet slides along the funnel into the eyelet


def _pegs() -> Body:
    """The rear peg (behind where the aglet is taken) has low lips either side of it; the front one, under the tip, is a
    slim fork that the closed fingers pass either side of as they push the aglet on."""
    h = EZ - AGLET_R
    dy = PEG_Y[1] - AGLET_BACK[1]
    lip = AGLET_R + 0.004  # the lips' middles: 2 mm clear of the aglet either side
    shapes = [BoxShape((0.0, dy, h / 2), (2 * lip + 0.004, 0.008, h), DARK),
              BoxShape((lip, dy, h + 0.0015), (0.004, 0.008, 0.003), DARK),  # low lips: the fingers pass above them
              BoxShape((-lip, dy, h + 0.0015), (0.004, 0.008, 0.003), DARK)]
    dy = PEG_Y[0] - AGLET_BACK[1]
    for x in (-0.0025, 0.0025):  # two 3 mm pins, the aglet cradled between their tops
        shapes.append(BoxShape((x, dy, (h + 0.0012) / 2), (0.003, 0.004, h + 0.0012), DARK))
    return Body.build("pegs", pos=(E2, AGLET_BACK[1], TOP), mass=0.5, shapes=shapes, kinematic=True,
                      friction=(0.4, 0.35), color=DARK)


def _lace_points() -> list[np.ndarray]:
    """Points along PATH about SEG apart, from the knot end to the aglet's back end."""
    lengths = [float(np.linalg.norm(b - a)) for a, b in zip(PATH, PATH[1:])]
    total = sum(lengths)
    n = max(1, round(total / SEG))
    out = []
    for k in range(n + 1):
        s = min(total * k / n, total - 1e-9)
        for (a, b), ln in zip(zip(PATH, PATH[1:]), lengths):
            if s <= ln:
                out.append(a + (b - a) * (s / ln))
                break
            s -= ln
    out[-1] = PATH[-1].copy()
    return out


LACE_PTS = _lace_points()
SEG_DIRS = [b - a for a, b in zip(LACE_PTS, LACE_PTS[1:])]  # each piece's direction, in its own (unrotated) frame


def _lace() -> list[Body]:
    """Each piece's body starts unrotated, origin at its middle; only its shape is turned along the lace."""
    out = []
    for k, (a, b) in enumerate(zip(LACE_PTS, LACE_PTS[1:])):
        mid, d = (a + b) / 2, b - a
        shapes = [BoxShape((0.0, 0.0, 0.0), (float(np.linalg.norm(d)) + 0.002, LACE_W, LACE_W), WHITE,
                           quat_xyzw=_quat_xyzw_x_to(d))]
        if k == 0:  # the knot, behind the first eyelet
            shapes.append(SphereShape(tuple((a - mid - d / np.linalg.norm(d) * 0.004).tolist()), KNOT_R, WHITE))
        out.append(Body.build(f"lace_{k:02d}", pos=tuple(mid.tolist()), mass=LACE_MASS, shapes=shapes, color=WHITE,
                              friction=(0.4, 0.35), damping=(1.0, 2.0)))
    return out


def _aglet() -> Body:
    """Origin at its back end; it points -y, at the second eyelet."""
    shapes = [CylShape((0.0, -AGLET_L / 2, 0.0), AGLET_R, AGLET_L, BLUE, axis="Y")]
    return Body.build("aglet", pos=tuple(AGLET_BACK.tolist()), mass=AGLET_MASS, color=BLUE, friction=(1.0, 0.9),
                      damping=(0.05, 0.5), shapes=shapes)


BOARD, PEGS, LACE, AGLET = _board(), _pegs(), _lace(), _aglet()
LACE_NAMES = [b.name for b in LACE]
OBJECTS = [BOARD, PEGS, *LACE, AGLET]
for _k, (_a, _b) in enumerate(zip(LACE, LACE[1:] + [AGLET])):
    ball(f"lace_joint_{_k:02d}", _a.name, _b.name, anchor=tuple(LACE_PTS[_k + 1].tolist()),
         axis=tuple(SEG_DIRS[_k].tolist()), swing_damping=0.0005, twist_damping=0.0005)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"tcps": None, "best": 0.0, "dropped": False}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("LACE_DEBUG")), "steps": 0}


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


def _status_from(lace_poses, a_pos, a_rot) -> dict:
    """Where the lace crosses the board's plane (y = 0), and how far the aglet is through."""
    pts = []
    for (p, r), d in zip(lace_poses, SEG_DIRS):
        pts.append(np.asarray(p) - r @ d / 2)
    pts.append(np.asarray(a_pos, float))
    tip = np.asarray(a_pos, float) + a_rot @ np.array([0.0, -AGLET_L, 0.0])
    pts.append(tip)
    via2 = False
    for a, b in zip(pts, pts[1:]):
        if (a[1] - 0.0) * (b[1] - 0.0) < 0.0:
            t = a[1] / (a[1] - b[1])
            c = a + (b - a) * t
            if math.hypot(c[0] - E2, c[2] - Z) < RING_RI + 0.002:
                via2 = True
    in_ring = abs(tip[0] - E2) < RING_RI and abs(tip[2] - Z) < RING_RI
    return {"via2": via2, "tip_in": in_ring and tip[1] < RING_L / 2, "tip_through": via2 and tip[1] < BACK_Y - THROUGH,
            "pulled": via2 and a_pos[1] < BACK_Y - PULLED, "tip": tip, "back": np.asarray(a_pos, float)}


def _gap_to_aglet(p, a_pos, a_rot) -> float:
    v = a_rot.T @ (np.asarray(p, float) - np.asarray(a_pos, float))
    t = min(0.0, max(-AGLET_L, float(v[1])))
    return float(np.linalg.norm(v - np.array([0.0, t, 0.0])))


def _points(st) -> float:
    if st["pulled"] and not _EP["dropped"]:
        return 1.0
    if st["tip_through"]:
        return 0.5
    return 0.25 if (st["tip_in"] or _EP["best"] >= 0.25) else 0.0


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])

    def pose(name):
        p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
        return p[:3] - origin, rotation(p[3:7])

    a_pos, a_rot = pose("aglet")
    st = _status_from([pose(n) for n in LACE_NAMES], a_pos, a_rot)
    _EP["tcps"] = _tcps(base, origin)
    _EP["best"] = max(_EP["best"], 0.25 if st["tip_in"] else 0.0)
    if st["tip_through"] and not st["pulled"] and all(_gap_to_aglet(p, a_pos, a_rot) > HELD for p in _EP["tcps"]):
        _EP["dropped"] = True  # let go of in the eyelet: it was not handed over
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            print(f"[lace] t={_DEBUG['steps'] / 20:5.1f}s tip={np.round(st['tip'] * 1000).astype(int).tolist()} "
                  f"back={np.round(st['back'] * 1000).astype(int).tolist()} via2={st['via2']} tip_in={st['tip_in']} "
                  f"through={st['tip_through']} pulled={st['pulled']} dropped={_EP['dropped']} "
                  f"leftTCP-back={np.round((tl - a_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-tip={np.round((tr - st['tip']) * 1000).astype(int).tolist()} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0 with the lace through the second eyelet and pulled on through, the aglet's back end at least 1.2 cm behind
    the board, handed over (never let go of between its tip coming through and its being pulled through); 0.5 with the
    aglet's tip through the second eyelet (1 cm behind the board); 0.25 if the tip went into the eyelet at some point;
    else 0.0."""
    a = objects["aglet"]
    st = _status_from([(np.asarray(objects[n].pos, float), rotation(objects[n].quat_xyzw)) for n in LACE_NAMES],
                      np.asarray(a.pos, float), rotation(a.quat_xyzw))
    return _points(st)


def success(objects) -> bool:
    """Full score with the lace still."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("aglet", *LACE_NAMES))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the aglet near its back end, fingers closing across it, lifts it just off its
# pegs and pushes it straight on through the second eyelet, about 4 cm out behind the board. The right hand, level and
# pointing at the board from behind it, fingers closing across the tip, takes it there; the left lets go and backs away,
# and the right pulls the lace on through, 8 cm.

GRIP_L = grip_rotation(-math.pi / 2)  # straight down, fingers closing along x (across the aglet)


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


SIDE_R = _rx(math.pi / 2) @ grip_rotation(math.pi / 2)  # level, pointing +y (at the board), fingers closing along x
HOLD_AT = 0.030  # the left TCP this far from the aglet's back end (its fingers clear of the back peg)
LIFT = 0.004  # lift the aglet just off its pegs: it is already in line with the eyelet, and its middle must stay clear
# of the front fork as it goes over it
PUSH_TO = 0.020  # push until the left TCP is this far in front of the board (its fingertips clear the funnel)
R_TAKE = -0.028  # the right TCP behind the board, its fingertips 1.5 cm clear of the grommet
PULL = 0.08
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


def _left() -> tuple[list[Waypoint], float, list]:
    pick = np.array([E2, AGLET_BACK[1] - HOLD_AT, Z])
    up = pick + [0.0, 0.0, LIFT]
    pushed = np.array([E2, PUSH_TO, Z + LIFT])
    out = _lead_in("left", pick + [0.0, 0.0, 0.05], GRIP_L, "over the aglet", FINGER_OPEN)
    out += [_w(1.0, pick, FINGER_OPEN, "down round it", GRIP_L),
            _w(0.8, pick, FINGER_CLOSED, "take it", GRIP_L),
            _w(0.6, up, FINGER_CLOSED, "lift it just off its pegs", GRIP_L),
            _w(2.5, pushed, FINGER_CLOSED, "push it straight through, about 4 cm out behind the board", GRIP_L)]
    return out, sum(w.duration for w in out), pushed


def demo():
    left, t_pushed, pushed = _left()
    take = np.array([E2, R_TAKE, Z])
    away = take + [0.0, -0.06, 0.0]
    right = _lead_in("right", away, SIDE_R, "level behind the board, at the second eyelet", FINGER_OPEN)
    t = sum(w.duration for w in right)
    right += [_w(max(t_pushed - t, 0.2) + 0.3, away, FINGER_OPEN, "wait for the tip to come through", SIDE_R),
              _w(1.2, take, FINGER_OPEN, "in round the tip", SIDE_R),
              _w(0.8, take, FINGER_CLOSED, "take it", SIDE_R)]
    t_taken = sum(w.duration for w in right)
    right += [_w(1.0, take, FINGER_CLOSED, "hold it while the other hand lets go", SIDE_R),
              _w(3.0, take + [0.0, -PULL, 0.0], FINGER_CLOSED, "pull the lace on through", SIDE_R),
              _w(3.0, take + [0.0, -PULL, 0.0], FINGER_CLOSED,
                 "hold still: the trial ends once the full score has held 1 s", SIDE_R)]
    left += [_w(max(t_taken - t_pushed, 0.2), pushed, FINGER_CLOSED, "hold it while the other hand takes the tip", GRIP_L),
             _w(0.6, pushed, FINGER_OPEN, "let go", GRIP_L),
             _w(1.0, pushed + [0.0, 0.05, 0.03], FINGER_OPEN, "back away", GRIP_L)]
    t_l, t_r = sum(w.duration for w in left), sum(w.duration for w in right)
    left.append(_w(max(t_r - t_l, 0.5), left[-1].pos, FINGER_OPEN, "wait", GRIP_L))
    return {"left": left, "right": right}
