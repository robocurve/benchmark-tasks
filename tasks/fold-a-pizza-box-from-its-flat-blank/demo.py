"""Demo for the pizza box task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 2800] [--no-record] [--mode full|still|walls|tabs|front|lock|...]

Every target comes from the layout constants in the task file and the sizes in ``boxkit.py``. The only feedback is
each hand's own measured position and attitude (the robot's proprioception), used to correct for the arm's sag. The
demo never reads the blank's state.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import json
import math
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
FULL_OPEN = 0.0939                 # m between the pads at grip command 1.0


def frame(approach, closing) -> np.ndarray:
    """Tool rotation whose approach axis (z) and finger-closing axis (x) point along these world directions."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def gap(width: float) -> float:
    """Grip command (1 open .. 0 closed) for a gap between the pads in metres."""
    return min(1.0, max(0.0, width / FULL_OPEN))


class Hand:
    """One arm: smooth moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = self.worst_deg = 0.0

    def _solve(self, pos, rot) -> None:
        from yam_arena import ik

        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.worst_deg = max(self.worst_deg, float(np.degrees(re)))
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, duration: float, rot=None, grip: float | None = None):
        """Generator: go in a straight line to ``pos`` (world, tool point) and ``rot``; ``grip`` 1 open .. 0 closed."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        r1 = r0 if rot is None else rot
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos, float) - p0), ik.slerp_matrix(r0, r1, s))
            yield

    def set_grip(self, grip: float, duration: float = 0.5):
        """Generator: open or close to ``grip`` in place."""
        from yam_arena import ik

        g0 = self.grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self.grip = g0 + ik.smoothstep(k / n) * (grip - g0)
            self.cmd = np.r_[self.q, self.grip]
            yield

    def move_joints(self, q_goal, duration: float, grip: float | None = None):
        """Generator: swing to joint angles ``q_goal``, every joint moving together."""
        from yam_arena import ik

        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q, self.grip = q0 + s * (np.asarray(q_goal, float) - q0), g0 + s * (g1 - g0)
            self.pos, self.rot = ik.tcp_pose(self.q, self.base)
            self.cmd = np.r_[self.q, self.grip]
            yield


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def chain(*gens):
    for g in gens:
        yield from g


def together(*gens):
    gens = [iter(g) for g in gens]
    while gens:
        for g in list(gens):
            try:
                next(g)
            except StopIteration:
                gens.remove(g)
        if gens:
            yield


def together_keep(other, main):
    """Generator: run ``other`` alongside ``main`` and return what ``main`` returns."""
    other, out = iter(other), None
    main = iter(main)
    alive = True
    while True:
        try:
            next(main)
        except StopIteration as stop:
            out = stop.value
            break
        if alive:
            try:
                next(other)
            except StopIteration:
                alive = False
        yield
    return out                           # ``other`` (a hold) simply ends when ``main`` does


def best_ik(hand, pos, rot):
    """Joint angles for ``pos``/``rot`` from the best of a few seeds (the hand's own, and its wrist turned either way)."""
    from yam_arena import ik

    best = None
    for q6 in (None, -1.0, 1.0):
        seed = hand.q.copy()
        if q6 is not None:
            seed[5] = q6
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), seed, base_pos=hand.base, iters=400)
        cost = pe + 0.05 * re
        if best is None or cost < best[1]:
            best = (q, cost)
    return best[0]


TOOL: dict = {}                          # where each hand's tool point actually is (updated every control step)
GAIN, BIAS_MAX = 0.25, 0.015             # how fast, and how far, a hand corrects for its own sag (a small limit:
                                         # a hand that is pushing on something must not wind its correction up)

ROT: dict = {}                           # each hand's measured attitude (updated every control step)
RGAIN, TURN_MAX = 0.20, math.radians(10.0)   # how fast, and how far, a hand corrects its own droop in attitude


def rotvec(m) -> np.ndarray:
    """The turn (axis times angle, rad) that a rotation matrix makes."""
    m = np.asarray(m, float)
    c = max(-1.0, min(1.0, (m[0, 0] + m[1, 1] + m[2, 2] - 1.0) / 2.0))
    ang = math.acos(c)
    v = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
    n = float(np.linalg.norm(v))
    return np.zeros(3) if n < 1e-9 else v / n * ang


def turned(w) -> np.ndarray:
    """The rotation matrix of a turn ``w`` (axis times angle, rad)."""
    ang = float(np.linalg.norm(w))
    if ang < 1e-9:
        return np.eye(3)
    k = np.asarray(w, float) / ang
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(ang) * kx + (1.0 - math.cos(ang)) * (kx @ kx)


class Sag:
    """Slowly offsets a hand's command by its measured error, in place and in attitude, so that its tool point and
    its fingers end up where they are wanted under load."""

    def __init__(self, hand):
        self.hand, self.bias, self.want = hand, np.zeros(3), None
        self.turn, self.rot = np.zeros(3), None

    def to(self, pos, rot):
        name = self.hand.name
        if self.want is not None:
            self.bias = np.clip(self.bias + GAIN * (self.want - TOOL[name]), -BIAS_MAX, BIAS_MAX)
            have = ROT.get(name)
            if have is not None and self.rot is not None:
                err = rotvec(self.rot @ np.asarray(have, float).T)          # from where the hand is to where it is wanted
                if float(np.linalg.norm(err)) < math.radians(20.0):
                    self.turn = self.turn + RGAIN * err
                    size = float(np.linalg.norm(self.turn))
                    if size > TURN_MAX:
                        self.turn *= TURN_MAX / size
        self.want, self.rot = np.asarray(pos, float), np.asarray(rot, float)
        self.hand._solve(self.want + self.bias, turned(self.turn) @ self.rot)


def glide(sag, target, seconds: float, rot):
    """Generator: take the hand's tool point in a straight line to ``target``, correcting for sag on the way."""
    from yam_arena import ik

    start = sag.want if sag.want is not None else sag.hand.pos.copy()
    n = max(1, round(seconds * CONTROL_HZ))
    for k in range(1, n + 1):
        sag.to(start + ik.smoothstep(k / n) * (np.asarray(target, float) - start), rot)
        yield



# -- the fold ---------------------------------------------------------------------------------------------------------
TIP = 0.0127                             # the closed fingertips' end, beyond the tool point along the hand's axis
LEAN = math.radians(45.0)
WALL_R, WALL_D = 0.030, 0.007            # a wall is pushed 30 mm out from its crease, the fingertips' end 7 mm off its face
WALL_X = -0.020                          # ... at this place along a side wall (box frame)
TAB_R, TAB_D = 0.042, 0.011
FRONT_Y = -0.108                         # where along the front wall the right hand works
LEAF_Y = 0.095                           # ... and the left hand


def unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


SEEDS = ([0, 1, 1, -0.6, 0, 0], [0, 1, 1, -0.6, 0, 1.57], [0, 1, 1, -0.6, 0, -1.57], [0.6, 1.2, 1.0, 0, 0.8, 0],
         [-0.6, 1.2, 1.0, 0, -0.8, 0], [0, 0.6, 0.6, 0.6, 0, 0], [0.8, 1.5, 1.5, -0.3, 1.2, 1.0],
         [-0.8, 1.5, 1.5, -0.3, -1.2, -1.0], [0.5, 2.0, 2.0, 0.5, 1.0, -1.0], [-0.5, 2.0, 2.0, 0.5, -1.0, 1.0],
         [1.2, 1.0, 1.2, 0.3, 1.4, 0.5], [-1.2, 1.0, 1.2, 0.3, -1.4, -0.5])


def pick_rot(hand, pos, approach, closings, also=(), signs=(1.0, -1.0), wide: bool = False):
    """The tool rotation with this approach axis whose closing axis (one of ``closings``, either sign) leaves the arm
    the most joint room at ``pos`` and, moving on from there, at the tool points in ``also``. ``wide``: also try the
    arm's other ways of reaching ``pos`` (elbow and wrist the other way round), not only the one nearest its pose now."""
    from yam_arena import ik

    best = None
    for c in closings:
        for sgn in signs:
            rot = frame(approach, sgn * np.asarray(c, float))
            quat = ik.matrix_to_quat_xyzw(rot)
            starts = [best_ik(hand, pos, rot)]
            if wide:
                starts += [ik.solve(pos, quat, np.array(sd, float), base_pos=hand.base, iters=400)[0] for sd in SEEDS]
            for q in starts:
                err, room, qk = 0.0, 9.0, q
                for k, pt in enumerate([pos, *also]):
                    qk = q if k == 0 else ik.solve(pt, quat, qk, base_pos=hand.base, iters=60 if wide else 400)[0]
                    p, r = ik.tcp_pose(qk, hand.base)
                    err = max(err, float(np.linalg.norm(p - pt)) * 1000 + float(np.degrees(np.linalg.norm(rotvec(rot @ r.T)))))
                    room = min(room, float(np.minimum(qk - ik.LOWER, ik.UPPER - qk).min()))
                key = (err > 1.5, -room)
                if best is None or key < best[0]:
                    best = (key, rot, q)
    return best[1], best[2]


class Tip:
    """A hand steered by the END of its closed fingertips, at a fixed attitude, correcting for sag."""

    def __init__(self, hand, rot):
        self.hand, self.rot, self.a, self.sag = hand, rot, rot[:, 2], Sag(hand)
        self.at = hand.pos + TIP * self.a

    def to(self, end):
        self.at = np.asarray(end, float)
        self.sag.to(self.at - TIP * self.a, self.rot)

    def go(self, end, seconds: float):
        from yam_arena import ik

        p0, p1 = self.at.copy(), np.asarray(end, float)
        n = max(1, round(seconds * CONTROL_HZ))
        for k in range(1, n + 1):
            self.to(p0 + ik.smoothstep(k / n) * (p1 - p0))
            yield

    def along(self, fn, a0: float, a1: float, seconds: float):
        from yam_arena import ik

        n = max(1, round(seconds * CONTROL_HZ))
        for k in range(1, n + 1):
            self.to(fn(a0 + ik.smoothstep(k / n) * (a1 - a0)))
            yield

    def stay(self, seconds: float):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            self.to(self.at)
            yield

    def lift(self, dz: float, seconds: float = 0.8):
        """Straight up from where the command is now, with no sag correction (a pressing hand's correction is wound
        up and would jerk it sideways as it lets go)."""
        yield from self.hand.move(self.hand.pos + (0.0, 0.0, dz), seconds)

    def hold(self, seconds: float):
        """Keep the last command as it is (no sag correction: a hand pressing on something would wind it up)."""
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield


SAFE: dict = {}                          # the height (world z) a hand goes up to before it swings to a new place
WRIST: dict = {}                         # remembered wrist attitudes and joint angles, by hand and name


def reach_to(hand, end, approach, closings, seconds: float = 2.5, grip: float = 0.0, also=(), rise: float = 9.0,
             signs=(1.0, -1.0), keep: str = "", use: str = "", wide: bool = False):
    """Generator: swing the hand so that its fingertips' end is at ``end`` with this approach axis; returns a Tip.
    ``also``: later fingertip positions the chosen attitude has to reach as well. ``keep`` remembers the wrist's
    attitude and the arm's joint angles under that name; ``use`` goes back to a remembered one (same ``end``)."""
    pos = np.asarray(end, float) - TIP * unit(approach)
    if use and (hand.name, use) in WRIST:
        rot, q = WRIST[(hand.name, use)]
    else:
        rot, q = pick_rot(hand, pos, unit(approach), closings, [np.asarray(e, float) - TIP * unit(approach) for e in also], signs, wide)
    if keep:
        WRIST[(hand.name, keep)] = (rot, np.array(q, float))
    if SAFE.get("z") and hand.pos[2] < SAFE["z"] - 0.005:                 # straight up first: a swing from low down
        yield from hand.move(np.array([hand.pos[0], hand.pos[1], min(SAFE["z"], hand.pos[2] + rise)]), 0.8)   # can sweep through the box
    yield from hand.move_joints(q, seconds, grip=grip)
    t = Tip(hand, rot)
    t.at = np.asarray(end, float)
    yield from t.stay(0.3)
    return t


PLACE_AIM = (0.0, 0.0)                   # where the side pushes aim to leave the box's centre line, from its marks (m)
PLACE = {"A": (0.0, -0.0006, 0.0), "B": (0.0005, -0.0005, 0.0)}   # where the pushes after the lock do leave the box, by
                                         # layout: from its marks (m: x, y) and turned (degrees, anticlockwise from above).
                                         # Found by trial; the same every run.


def place(T) -> tuple:
    env = __import__("os").environ.get("BOX_PLACE")          # (probes only: try another)
    return tuple(float(v) for v in env.split(",")) if env else PLACE[T.LOT]


WHERE: dict = {"off": (0.0, 0.0, 0.0)}        # where the script takes the box to be, from its marks: nothing is read from the
                                         # simulator; before the pushes it is taken to be on its marks
MARKS = tuple(float(v) for v in (__import__("os").environ.get("BOX_MARKS") or "0,0").split(","))   # probes only: start
                                         # the box this far off the marks the script works from


def B(T, x_in: float, y: float, z: float) -> np.ndarray:
    """A world point from box coordinates: x in from the front crease, y left of the centre line, z above the board's
    middle plane."""
    o = box_origin(T)
    vx, vy = T.K.XF + x_in, y
    cy, sy = math.cos(math.radians(WHERE["off"][2])), math.sin(math.radians(WHERE["off"][2]))
    return np.array([o[0] + cy * vx - sy * vy, o[1] + sy * vx + cy * vy, o[2] + z])


def box_origin(T) -> np.ndarray:
    """The blank frame's origin (the base's centre) where the script takes the box to be."""
    c = T.blank_pos()
    return np.array([c[0] - MARKS[0] + WHERE["off"][0], c[1] - MARKS[1] + WHERE["off"][1], c[2]])


def box_turn() -> np.ndarray:
    """The box's turn about the upright axis, as the script takes it to be."""
    cy, sy = math.cos(math.radians(WHERE["off"][2])), math.sin(math.radians(WHERE["off"][2]))
    return np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])


def wall_path(T, s: float):
    K = T.K

    def fn(deg):
        t = math.radians(deg)
        r = np.array([0.0, s * math.cos(t), math.sin(t)])
        n = np.array([0.0, s * math.sin(t), -math.cos(t)])
        return B(T, -K.XF + WALL_X, s * K.YS, 0.0) + WALL_R * r + WALL_D * n
    return fn


PREBREAK = 125.0                         # a side wall is folded this far, well past upright, to pre-break its score


def fold_wall(T, hand, s: float, to_deg: float = PREBREAK):
    """One hand, leaning 45 degrees with its fingertips pointing in under the side wall, sweeps the wall up."""
    fn = wall_path(T, s)
    a = (0.0, -s * math.sin(LEAN), -math.cos(LEAN))
    start = fn(-8.0)
    t = yield from reach_to(hand, start + (0.0, s * 0.070, 0.050), a, ((0.0, -s * math.cos(LEAN), math.sin(LEAN)),), also=(start, fn(to_deg)))   # the fingers' flat faces toward the wall
    yield from t.go(start + (0.0, s * 0.070, -0.010), 1.0)
    yield from t.stay(0.4)
    yield from t.go(start, 1.0)
    yield from t.along(fn, -8.0, to_deg, 4.0)
    yield from t.hold(0.4)
    yield from hand.move(hand.pos + (0.0, s * 0.050, 0.060), 1.0)       # out and up, clear of the wall as it comes back
    return t


def tab_path(T, s: float):
    K = T.K

    def fn(deg):
        b = math.radians(deg)
        d = np.array([-math.cos(b), -s * math.sin(b), 0.0])
        n = np.array([-math.sin(b), s * math.cos(b), 0.0])
        return B(T, K.TAB_IN, s * (K.YS + 0.001), 0.014) + TAB_R * d + TAB_D * n
    return fn


STAND_IN = 0.0095                        # the finger's middle, in from the wall's line, when it stands the wall upright


def fold_tab(T, hand, s: float, to_deg: float = 62.0):
    """One hand, straight down, fingers closed. First, from inside the box, it pushes the pre-broken side wall out to
    upright (the wall comes back from its pre-break leaning in a little, and a tab on a wall that leans in digs into
    the front wall as it turns). Then it sweeps the tab at the wall's front end round into the front."""
    K = T.K
    fn = tab_path(T, s)
    start = fn(0.0)
    inside = B(T, 0.045, s * (K.YS - 0.035), 0.022)
    t = yield from reach_to(hand, inside + (0.0, 0.0, 0.090), (0, 0, -1), ((0, 1, 0),), seconds=2.0,
                            also=(inside, start, fn(to_deg)))
    yield from t.go(inside, 0.8)
    yield from t.go(B(T, 0.045, s * (K.YS - STAND_IN), 0.022), 1.0)
    yield from t.hold(0.2)
    yield from t.go(inside + (0.0, 0.0, 0.090), 0.8)
    yield from t.go(start + (0.0, s * 0.045, 0.080), 1.0)
    yield from t.go(start + (0.0, s * 0.045, 0.0), 0.8)
    yield from t.go(start, 0.8)
    yield from t.along(fn, 0.0, to_deg, 2.5)
    yield from t.stay(0.2)
    yield from t.go(fn(to_deg) + (-0.050, 0.0, 0.0), 0.8)
    yield from t.go(fn(to_deg) + (-0.050, 0.0, 0.090), 0.8)
    return t


FRONT_R = 0.040                          # the front wall is pushed this far out from its crease


def front_path(T, y: float):
    def fn(deg):
        t = math.radians(deg)
        r = np.array([-math.cos(t), 0.0, math.sin(t)])
        n = np.array([-math.sin(t), 0.0, -math.cos(t)])
        return B(T, 0.0, y, 0.0) + FRONT_R * r + WALL_D * n
    return fn


def lift_front(T, hand, to_deg: float = 86.0):
    """The right hand, leaning 45 degrees in from the box's right side with its fingertips under the front wall's right
    end, sweeps the front wall up and stays against it. (A hand leaning forward cannot do this: its fingers would lie
    under the cap and the inner leaf and lift those first.)"""
    fn = front_path(T, FRONT_Y)
    a = (0.0, math.sin(LEAN), -math.cos(LEAN))
    start = fn(-8.0)
    # fingers side by side along the box's front-to-back line: thin that way, so they stop the wall at upright
    t = yield from reach_to(hand, start + (0.0, -0.075, 0.050), a, ((1, 0, 0),), also=(start, fn(to_deg)))
    yield from t.go(start + (0.0, -0.075, -0.010), 1.0)
    yield from t.stay(0.4)
    yield from t.go(start, 1.0)
    yield from t.along(fn, -8.0, to_deg, 3.5)
    return t


HOLD_X, HOLD_Y, HOLD_PRESS = 0.140, 0.050, 0.005


HOLD_FRONT = (0.045, -0.030)             # where the left hand presses while the front wall is lifted at its right end


def hold_base(T, hand, s: float, at=None, press: float = 0.0, thin: bool = False):
    """One hand, fingers closed, leaning 30 degrees out to its own side, presses the base to the board (by default on
    its own side of the centre line)."""
    K = T.K
    p = B(T, HOLD_X, s * HOLD_Y, K.T / 2) if at is None else B(T, at[0], at[1], K.T / 2)
    a = (0.0, -s * math.sin(math.radians(30.0)), -math.cos(math.radians(30.0)))
    closings = ((1, 0, 0),) if thin else ((1, 0, 0), (0.0, -s * math.cos(math.radians(30.0)), math.sin(math.radians(30.0))))
    t = yield from reach_to(hand, p + (0.0, 0.0, 0.120), a, closings, seconds=2.0, also=(p,))   # thin: hand's body clear of a standing side wall
    yield from t.go(p + (0.0, 0.0, 0.020), 0.8)
    yield from t.go(p + (0.0, 0.0, -(press or HOLD_PRESS)), 0.8)
    return t


ROLL_R = 0.035                           # the inner leaf is pushed this far from its crease


LEAF_FIRST = 112.0                       # the inner leaf turns this far at its own crease before the cap's crease takes over


def roll_path(T):
    """The fingertips' end as the inner leaf is rolled over: ``a`` degrees in all, the leaf's own crease first, then the
    cap's."""
    K = T.K

    def fn(a):
        leaf = math.radians(min(a, LEAF_FIRST))
        cap = math.radians(max(0.0, a - LEAF_FIRST))
        edge = np.array([K.CAP * math.sin(cap), 0.0, K.H + K.CAP * math.cos(cap)])       # the cap's inner edge
        t = cap + leaf
        point = edge + ROLL_R * np.array([math.sin(t), 0.0, math.cos(t)])
        n = np.array([-math.cos(t), 0.0, math.sin(t)])
        return B(T, 0.0, LEAF_Y, 0.0) + point + 0.008 * n
    return fn


def roll_leaf(T, hand, to_deg: float = 172.0):
    """The left hand, straight down, pushes the inner leaf over the top of the front wall and down inside."""
    fn = roll_path(T)
    t = yield from reach_to(hand, fn(0.0) + (-0.020, 0.0, 0.030), (0, 0, -1), ((1, 0, 0),), seconds=2.0,
                            also=[fn(a) for a in (0.0, 60.0, 120.0, to_deg)])
    yield from t.go(fn(0.0) + (-0.012, 0.0, 0.0), 0.8)
    yield from t.along(fn, 0.0, to_deg, 4.0)
    yield from t.stay(0.3)
    yield from t.go(fn(to_deg) + (0.030, 0.0, 0.100), 1.2)
    return t


SANDWICH_X = 0.0040                      # the middle of wall, tab and leaf side by side, in from the front crease
STRADDLE_OPEN, STRADDLE_SHUT = 0.064, 0.0106
STRADDLE_Z = 0.014                       # the fingertips' ends above the base's top when the hand first closes
PRESS = 0.008                            # both hands then go down this far: one drags the leaf, the other presses the cap


def straddle(T, hand):
    """The left hand, straight down and open, comes down astride the front wall (one finger outside it, one inside
    behind the leaf) and closes: wall, tabs and leaf are squeezed together and centred between its fingers."""
    K = T.K
    hi, lo = B(T, SANDWICH_X, LEAF_Y, 0.135), B(T, SANDWICH_X, LEAF_Y, K.T / 2 + STRADDLE_Z)
    t = yield from reach_to(hand, hi, (0, 0, -1), ((1, 0, 0),), seconds=2.0, grip=gap(STRADDLE_OPEN), also=(lo,))
    yield from t.go(lo, 1.5)
    yield from hand.set_grip(gap(STRADDLE_SHUT), 1.5)
    return t


def press_cap(T, hand):
    """The right hand, straight down, fingers closed, comes to just above the cap."""
    K = T.K
    hi = B(T, SANDWICH_X, FRONT_Y, K.H + 0.060)
    lo = B(T, SANDWICH_X, FRONT_Y, K.H + K.T / 2 + 0.004)
    lean = math.radians(30.0)            # leaning out to its own side, so the two wrists stay apart
    t = yield from reach_to(hand, hi, (0.0, math.sin(lean), -math.cos(lean)), ((1, 0, 0),), seconds=2.0, also=(lo,))
    yield from t.go(lo, 1.0)
    return t


# -- the lid ----------------------------------------------------------------------------------------------------------
FLAP_R, FLAP_D = 0.042, 0.007            # a lid flap is pushed 42 mm out from its crease, near its free edge
FLAP_LEAN, FLAP_TURN = 65.0, 70.0        # the hand leans 65 degrees from upright, in from the side and a little forward
FLAP_AT = 0.022                          # ... this far along the flap from its rear end
BACK_R = 0.030                           # the back wall is pushed 30 mm from its crease (higher, the fingers meet the trailing lid)
BACK_R_UP = 0.014
BACK_AT = 0.106                          # ... under its end that hangs over the board
BACK_Y = 0.106
LID_R = 0.045                            # the lid is pushed over from behind, this far up from its hinge
SUPPORT_D = 0.215                        # the lid comes down onto a fingertip under its front corner, this far from its hinge
SUPPORT_Y = 0.095
SUPPORT_FROM, SUPPORT_TO = 25.0, 6.0     # (the lid in fact comes to rest higher, on the fingers' flank: LID_REST)     # degrees the lid is open when it lands on the fingertip, and when the fingertip leaves


TRAIL = 12.0                             # degrees below level: the lid trailing from the standing back wall's top to the board


BACK_LEAN = 80.0                         # degrees: where the raised back wall settles, propped by the trailing lid
BACK_STOP = 94.6                         # ... and where lifting the lid carries it: hard up against the side walls' rear ends
LID_REST = 41.3                          # degrees open: where the lid comes to rest on the supporting fingers


def lid_pose(T, state: str = "flat"):
    """The lid's (rotation, origin) in the world, as the script takes it to be at each stage (nothing is read from the
    simulator): ``flat`` on the board; ``trailing`` from the top of the back wall, which leans at BACK_LEAN, down to the
    board; ``standing`` (only its hinge matters) and ``resting`` on the supporting fingers, the back wall at BACK_STOP."""
    folds = {"flat": {}, "trailing": {"back": BACK_LEAN, "lid": -(BACK_LEAN + TRAIL)},
             "standing": {"back": BACK_STOP, "lid": 0.0},
             "resting": {"back": BACK_STOP, "lid": 180.0 - LID_REST - BACK_STOP}}[state]
    R, p = T.K.fk(folds)["lid"]
    Z = box_turn()
    return Z @ np.asarray(R, float), Z @ np.asarray(p, float) + box_origin(T)


def lid_flap_path(T, s: float):
    """The fingertips' end under a lid side flap folded ``deg`` degrees, on the lid as it lies."""
    K = T.K
    R, p = lid_pose(T, "flat")
    crease = K.panel_points("lid", (R, p), (K.XB + K.H + K.SIDE_FLAP[0] + FLAP_AT, s * K.LID_W, 0.0))[0]
    across, out = s * R[:, 1], R[:, 2]                       # along the flat flap away from the lid; out of the lid's inside face

    def fn(deg):
        t = math.radians(deg)
        return crease + FLAP_R * (math.cos(t) * across + math.sin(t) * out) + FLAP_D * (math.sin(t) * across - math.cos(t) * out)
    return fn


def fold_lid_flap(T, hand, s: float, to_deg: float = 105.0):
    """One hand, leaning well over from the box's side, slides its fingertips in under the free edge of a lid side flap
    and turns the flap up and in. (A hand leaning forward has to pass through the flap to get under it, and its body
    meets the side wall's rear end.)"""
    fn = lid_flap_path(T, s)
    le, tu = math.radians(FLAP_LEAN), math.radians(FLAP_TURN)
    a = np.array([math.sin(le) * math.cos(tu), -s * math.sin(le) * math.sin(tu), -math.cos(le)])
    c1 = unit(np.cross(a, (1.0, 0.0, 0.0)))
    c2 = np.cross(a, c1)                                      # fingers side by side along the flap (one above the other,
    start = fn(-8.0)                                          # the lower one would be in the table)
    out = start + (0.0, s * 0.060, -0.012)                    # outside the flap's free edge, below it
    t = yield from reach_to(hand, out + (0.0, 0.0, 0.080), a, (tuple(c2), tuple(-c2)), also=(out, start, fn(to_deg)))
    yield from t.go(out, 1.0)
    yield from t.go(start, 1.0)
    yield from t.along(fn, -8.0, to_deg, 3.0)
    yield from t.stay(0.2)
    yield from t.go(fn(to_deg) + (0.0, s * 0.050, 0.050), 1.0)
    return t


LID_HOLD = (0.090, -0.040)               # where the right hand presses the base while the back wall and lid are lifted
LID_FROM, LID_OVER = -20.0, 112.0        # degrees above level: where the lid is picked up, and where it is let fall


def lid_path(T, s: float = 1.0, state: str = "trailing"):
    """The fingertips' end behind the lid, the lid ``deg`` degrees above level (90: upright), about its hinge."""
    K = T.K
    hinge = K.panel_points("lid", lid_pose(T, state), (K.XB + K.H, s * BACK_Y, 0.0))[0]

    def fn(deg):
        b = math.radians(deg)
        return hinge + (LID_R * math.cos(b) + WALL_D * math.sin(b), 0.0, LID_R * math.sin(b) - WALL_D * math.cos(b))
    return fn


def back_path(T, s: float = 1.0):
    """The fingertips' end behind the back wall folded ``deg`` degrees. The push starts 30 mm from the crease, for
    leverage, and slides down to 14 mm as the wall comes upright: higher up, the fingers would meet the lid, which
    trails from the wall's top."""
    K = T.K
    xb = K.XB - K.XF

    def fn(deg):
        t = math.radians(deg)
        u = min(1.0, max(0.0, (deg - 45.0) / 40.0))
        r = BACK_R - (BACK_R - BACK_R_UP) * u * u * (3.0 - 2.0 * u)
        return B(T, xb + r * math.cos(t) + WALL_D * math.sin(t), s * BACK_AT, r * math.sin(t) - WALL_D * math.cos(t))
    return fn


BACK_UP = 84.0                           # the back wall is pushed up this far; it stops near 80, propped by the trailing lid
LID_MID = 84.0                           # the lid is lifted this far, the base held (unheld, the box slides); it stands there


def _behind(s: float):
    lean = math.radians(60.0)                                # leaning well over; fingers side by side along the box's depth,
    return (0.0, -s * math.sin(lean), -math.cos(lean))       # thin that way, so they stay clear of the base's rear edge


def raise_lid(T, hand, s: float = 1.0):
    """One hand, leaning 60 degrees in from the box's side, lifts the back wall by the end that hangs over the board
    until it stands (the lid, trailing from its top to the board, props it), then moves back under the lid and
    starts to lift it."""
    wall = back_path(T, s)
    a = _behind(s)
    start = wall(-8.0)
    lid0 = lid_path(T, s)                                    # (for the wrist's choice only: the lid as it will trail)
    # the arm has to be able to go on from here through the whole lift and the push over the top, wherever between
    # leaning back and hard up against the side walls the back wall then stands: try its other ways round as well
    t = yield from reach_to(hand, start + (0.0, s * 0.070, 0.050), a, ((1, 0, 0), (-1, 0, 0)), wide=True,
                            also=(start + (0.0, s * 0.070, -0.016), start + (0.0, 0.0, -0.016), start, wall(60.0), wall(BACK_UP),
                                  lid0(LID_FROM), lid0(40.0), lid0(LID_MID), lid0(LID_OVER), lid0(LID_OVER) + (-0.016, 0.0, 0.0)))
    yield from t.go(start + (0.0, s * 0.070, -0.016), 1.2)
    yield from t.go(start + (0.0, 0.0, -0.016), 1.0)          # in under the wall's end, well below it
    yield from t.stay(0.3)
    yield from t.go(wall(-8.0), 0.8)
    yield from t.along(wall, -8.0, BACK_UP, 3.5)
    yield from t.stay(0.3)
    yield from t.go(wall(BACK_UP) + (0.035, 0.0, 0.004), 1.0) # back from the wall, under the trailing lid
    yield from t.stay(0.5)                                    # the wall settles back, leaning on the trailing lid
    lid = lid_path(T, s)
    yield from t.go(lid(LID_FROM) + (0.0, 0.0, -0.012), 0.8)
    yield from t.go(lid(LID_FROM), 0.5)
    yield from t.along(lid, LID_FROM, LID_MID, 3.5)
    yield from t.stay(0.4)
    return t


def push_lid_over(T, t, s: float = 1.0):
    """The hand behind the lid pushes it on over toward the robot until it falls, then backs off. (Lifting the lid has
    carried the back wall forward against the side walls' rear ends: the lid's hinge is taken there.)"""
    lid = lid_path(T, s, "standing")
    yield from t.go(lid(LID_MID + 4.0), 1.0)
    yield from t.along(lid, LID_MID + 4.0, LID_OVER, 2.0)
    yield from t.stay(0.5)
    yield from t.hand.move(t.hand.pos - 0.035 * np.asarray(_behind(s)), 0.8)   # out along the fingers' own line
    return t


def support_path(T, s: float):
    """The fingertips' end under the lid's front corner (in the notch between the side flap's front end and the lid's
    front edge), the lid ``deg`` degrees open, about its hinge with the back wall hard up against the side walls."""
    K = T.K
    hinge = K.panel_points("lid", lid_pose(T, "standing"), (K.XB + K.H, s * SUPPORT_Y, 0.0))[0]

    def fn(deg):
        al = math.radians(deg)
        return hinge + (-SUPPORT_D * math.cos(al), 0.0, SUPPORT_D * math.sin(al) - K.T / 2 - 0.004)
    return fn


SUPPORT_LEAN = math.radians(70.0)         # the supporting hand lies nearly flat: on a steeper finger the lid slides off


def support_lid(T, hand, s: float = -1.0):
    """One hand, leaning 70 degrees in from the box's side, waits with its fingertips where the falling lid's front
    corner will land (in the notch between the side flap's front end and the lid's front edge)."""
    fn = support_path(T, s)
    a = (0.0, -s * math.sin(SUPPORT_LEAN), -math.cos(SUPPORT_LEAN))
    t = yield from reach_to(hand, fn(SUPPORT_FROM) + (0.0, s * 0.040, 0.040), a,
                            ((0.0, -s * math.cos(SUPPORT_LEAN), math.sin(SUPPORT_LEAN)),), also=(fn(SUPPORT_FROM), fn(SUPPORT_TO)))
    yield from t.go(fn(SUPPORT_FROM), 1.0)
    return t


def front_flap_path(T, y: float):
    """The fingertips' end for the lid's front flap folded ``deg`` degrees, the lid resting on the supporting fingers."""
    K = T.K
    pose = lid_pose(T, "resting")
    hinge = K.panel_points("lid", pose, (K.XB + K.H, y, 0.0))[0]
    crease = K.panel_points("lid", pose, (K.XB + K.H + K.LID_L, y, 0.0))[0]
    u = unit(crease - hinge)                                 # along the lid, from its hinge to its front edge
    n_in = np.asarray(pose[0], float)[:, 2]                  # out of the lid's inside face

    def fn(deg):
        p = math.radians(deg)
        d = math.cos(p) * u + math.sin(p) * n_in
        o = -math.cos(p) * n_in + math.sin(p) * u
        return crease + 0.034 * d + 0.008 * o
    return fn


def fold_front_flap(T, hand, s: float = 1.0, to_deg: float = 112.0):
    """One hand, leaning 30 degrees in, pushes the lid's front flap down and under while the lid rests part open."""
    fn = front_flap_path(T, s * 0.050)
    lean = math.radians(30.0)
    a = (0.0, -s * math.sin(lean), -math.cos(lean))
    t = yield from reach_to(hand, fn(0.0) + (-0.020, 0.0, 0.040), a, ((1, 0, 0),), also=(fn(0.0), fn(to_deg)), rise=0.030)
    yield from t.go(fn(0.0), 1.0)
    yield from t.along(fn, 0.0, to_deg, 2.5)
    yield from t.stay(0.2)
    yield from t.go(fn(to_deg) + (-0.050, 0.0, 0.020), 0.8)
    return t


def press_lid(T, hand, s: float = 1.0, stay: bool = False):
    """One hand, leaning 30 degrees in, presses the lid's front down."""
    K = T.K
    lean = math.radians(30.0)
    p = B(T, 0.060, s * 0.050, K.H + K.T / 2)
    t = yield from reach_to(hand, p + (0.0, 0.0, 0.050), (0.0, -s * math.sin(lean), -math.cos(lean)), ((1, 0, 0),), seconds=2.0, also=(p,))
    yield from t.go(p + (0.0, 0.0, -0.004), 1.0)
    yield from t.hold(0.5)
    if not stay:
        yield from t.go(p + (0.0, 0.0, 0.060), 0.8)
    return t


SIDE_AT = 0.127                          # the side pushes are made at the middle of the box's length: the base's centre
                                         # then ends on its line even if the push has turned the box a little
SIDE_STOP = 0.1349                       # m from the centre line: where the pushing fingertips' end stops when the side
                                         # wall it pushes is where it belongs (found by trial: the fingers are fat)
SIDE_DRAG = float(__import__("os").environ.get("BOX_DRAG") or 0.016)   # m: over its last 26 mm the left hand's push also moves
                                         # this far toward the robot, and the box, held to the fingers by friction, comes with
                                         # it (the lock's folds also leave the box about a centimetre too far away)
PUSH_FROM = 0.045                        # the pushes start this far out: further than the lock ever slides the box


def push_side(T, hand, s: float, drag: float = 0.0, at: float = SIDE_AT):
    """One hand, fingers closed, pushes the box sideways by the foot of its side wall until the hand is at a fixed place.
    Wherever the lock left the box on that side of its marks, it ends at the same place. With ``drag``, the last part
    of the push also goes that far toward the robot."""
    y_stop = SIDE_STOP + s * PLACE_AIM[1]
    a = (0.0, -s * math.sin(math.radians(30.0)), -math.cos(math.radians(30.0)))
    start, mid = B(T, at, s * (y_stop + PUSH_FROM), 0.008), B(T, at, s * (y_stop + 0.026), 0.008)
    stop = B(T, at - drag, s * y_stop, 0.008)
    t = yield from reach_to(hand, start + (0.0, 0.0, 0.090), a, ((1, 0, 0),), seconds=2.0, also=(start, stop))
    yield from t.go(start, 1.0)
    yield from t.go(mid, 1.0)
    yield from t.go(stop, 1.6)
    yield from t.hold(0.3)
    yield from t.hand.move(t.hand.pos + (0.0, s * 0.020, 0.0), 0.5)
    yield from t.hand.move(t.hand.pos + (0.0, 0.0, 0.090), 0.8)


def square_box(T, left, right):
    """The dragging push leaves the box turned a little, its right front and left rear corners sticking out. The right
    hand at the foot of the right wall near the front, and the left hand at the foot of the left wall near the back,
    push in together until each is at its fixed place: the box turns back square, on its centre line."""
    tips = []
    for hand, s, at in ((left, 1.0, 0.210), (right, -1.0, 0.040)):
        y_stop = SIDE_STOP + s * PLACE_AIM[1]
        a = (0.0, -s * math.sin(math.radians(30.0)), -math.cos(math.radians(30.0)))
        start, stop = B(T, at, s * (y_stop + PUSH_FROM), 0.008), B(T, at, s * y_stop, 0.008)
        tips.append([hand, a, start, stop, s])
    got = []
    for hand, a, start, stop, s in tips:                      # one hand at a time into place above its start ...
        t = yield from reach_to(hand, start + (0.0, 0.0, 0.090), a, ((1, 0, 0),), seconds=2.0, also=(start, stop))
        got.append((t, start, stop, s))
    yield from together(*(t.go(start, 1.0) for t, start, _, _ in got))                   # ... then both together
    yield from together(*(t.go(stop, PUSH_FROM / 0.020) for t, _, stop, _ in got))
    yield from together(*(t.hold(0.5) for t, _, _, _ in got))
    yield from together(*(t.hand.move(t.hand.pos + (0.0, s * 0.020, 0.0), 0.5) for t, _, _, s in got))
    yield from together(*(t.hand.move(t.hand.pos + (0.0, 0.0, 0.090), 0.8) for t, _, _, _ in got))


PINCH_Y = float(__import__("os").environ.get("BOX_PY") or 0.085)                          # the two hands take the front wall this far either side of the centre line
PINCH_LEAN = float(__import__("os").environ.get("BOX_PLEAN") or 20.0)
PINCH_SIGN = {"left": -1.0, "right": 1.0}  # the wrist the way round each arm can reach (mirror images of each other)
PINCH_SHUT = 0.0100                      # ... and close until their fingers are this far apart. Pressed together, wall, tab
                                         # and leaf are 9.5 mm thick: the fingers centre them, without squeezing
PINCH_X = 0.0032                         # m in from the front crease: the middle of wall, tab and leaf pressed together


def pinch_front(T, left, right):
    """Both hands, open, come down astride the locked front wall (one finger outside it, one inside behind the leaf) and
    close on it at a fixed place. The closing fingers centre the wall between them, so the box ends at a distance from
    the robot, and square to it, that is known from the hands' own positions, wherever it was before. Then they open
    and lift."""
    K = T.K
    got = []
    for hand, s in ((left, 1.0), (right, -1.0)):
        hi, lo = B(T, PINCH_X, s * PINCH_Y, 0.135), B(T, PINCH_X, s * PINCH_Y, K.T / 2 + STRADDLE_Z)
        lean = math.radians(PINCH_LEAN)                      # leaning out to its own side, so the two forearms stay apart
        sg = float(__import__("os").environ.get("BOX_PSIGN_" + hand.name[0].upper()) or PINCH_SIGN[hand.name])
        t = yield from reach_to(hand, hi, (0.0, -s * math.sin(lean), -math.cos(lean)), ((1, 0, 0),), seconds=2.0,
                                grip=gap(STRADDLE_OPEN), also=(lo,), signs=(sg,))
        got.append((t, lo))
    yield from together(*(t.go(lo, 1.5) for t, lo in got))
    yield from together(*(t.stay(1.2) for t, _ in got))       # until each hand has settled exactly where it is meant to be
    yield from together(*(t.hand.set_grip(gap(PINCH_SHUT), 2.0) for t, _ in got))
    yield from together(*(t.hold(0.4) for t, _ in got))
    yield from together(*(t.hand.set_grip(gap(0.050), 0.8) for t, _ in got))
    yield from together(*(t.lift(0.090, 1.2) for t, _ in got))


HOME: dict = {}                          # each hand's tool point and attitude at the start


def go_home(hand, seconds: float = 2.5, lift: float = 0.10):
    """Generator: straight up first (clear of the box), then back to the hand's starting joint angles."""
    from yam_arena.constants import READY

    lift = min(lift, max(0.0, HOME[hand.name][0][2] - 0.02 - hand.pos[2]))      # no higher than where it started
    if lift > 0.005:
        yield from hand.move(hand.pos + (0.0, 0.0, lift), 1.0)
    yield from hand.move_joints(np.array(READY, float), seconds, grip=1.0)


def script(T, left, right, mode: str = "full", idle_s: float = 45.0):
    """The demo as a generator: one yield per control step."""
    if mode == "still":
        yield from wait(idle_s)
        return
    HOME.update({h.name: (h.pos.copy(), h.rot.copy()) for h in (left, right)})
    SAFE["z"] = float(T.blank_pos()[2] + 0.150)
    WHERE["off"] = (0.0, 0.0, 0.0)
    if mode in ("lid", "push", "flaps", "raise", "noflap", "held") and __import__("os").environ.get("BOX_START"):
        yield from lid_stage(T, left, right, "full" if mode == "lid" else mode)      # probe: the box starts locked
        return
    skip = set(filter(None, __import__("os").environ.get("BOX_SKIP", "").split(",")))
    if mode == "notabs":                                     # control: the tabs are left sticking out forward
        skip.add("tabs")
    # one hand always presses the base to the board while the other folds
    quick = {"walls", "tabs"} <= skip
    over = 88.0 if mode == "noprebreak" else PREBREAK        # control: the walls are folded just to upright
    rt = None if quick else (yield from hold_base(T, right, -1.0))
    if "walls" not in skip:
        yield from together_keep(rt.hold(60.0), fold_wall(T, left, 1.0, over))
    if "tabs" not in skip:
        yield from together_keep(rt.hold(60.0), fold_tab(T, left, 1.0))
    if not quick:
        lt = yield from together_keep(rt.hold(60.0), hold_base(T, left, 1.0))
        yield from together(lt.hold(1.0), rt.lift(0.090, 1.0))
    if "walls" not in skip:
        yield from together_keep(lt.hold(60.0), fold_wall(T, right, -1.0, over))
    if mode == "walls":
        yield from together(go_home(left), go_home(right))
        yield from wait(3.0)
        return
    if "tabs" not in skip:
        yield from together_keep(lt.hold(60.0), fold_tab(T, right, -1.0))
    if mode == "tabs":
        yield from together(go_home(left), go_home(right))
        yield from wait(3.0)
        return
    if not quick:
        yield from lt.lift(0.060)
    lt = yield from hold_base(T, left, 1.0, at=HOLD_FRONT)                    # next to the wall that is lifted next
    rt = yield from together_keep(lt.hold(60.0), lift_front(T, right))
    if mode == "front":
        yield from rt.stay(1.0)
        yield from together(go_home(left), go_home(right))
        yield from wait(3.0)
        return
    yield from together(rt.hold(1.0), lt.lift(0.090, 1.0))
    lt = yield from together_keep(rt.hold(60.0), roll_leaf(T, left))
    if mode == "roll":
        yield from together(go_home(left), go_home(right))
        yield from wait(3.0)
        return
    lt = yield from together_keep(rt.hold(60.0), straddle(T, left))
    yield from together(lt.hold(1.0), right.move(right.pos + (-0.050, 0.0, 0.020), 1.0))
    rt = yield from together_keep(lt.hold(60.0), press_cap(T, right))
    yield from together(lt.lift(-PRESS, 1.2), rt.lift(-0.004 - PRESS, 1.2))
    yield from together(lt.hold(0.6), rt.hold(0.6))
    yield from together(left.set_grip(gap(0.050), 0.6), rt.hold(0.6))
    yield from together(lt.lift(0.060, 1.2), rt.hold(1.2))
    yield from rt.lift(0.080, 1.0)
    if mode in ("lock", "noprebreak", "notabs"):
        yield from together(go_home(left), go_home(right))
        yield from wait(3.0)
        return
    yield from lid_stage(T, left, right, mode)


def lid_stage(T, left, right, mode: str = "full"):
    skip = set(filter(None, __import__("os").environ.get("BOX_SKIP", "").split(",")))
    # the lock's folds slide the box a centimetre or two; the lid's moves need it in a known place
    if "push" not in skip:
        yield from push_side(T, left, 1.0, SIDE_DRAG)       # back sideways, and toward the robot (this turns the box a little)
        yield from square_box(T, left, right)
    yield from pinch_front(T, left, right)                  # to a known distance: the pushes cannot bring it near enough
    yield from together(go_home(left, lift=0.0), go_home(right, lift=0.0))  # (the pushes leave the arms folded a way the
    WHERE["off"] = place(T)                                                  # flap folds cannot start from)
    if mode == "push":
        yield from wait(2.0)
        return
    if "flaps" not in skip:
        yield from together(fold_lid_flap(T, left, 1.0), fold_lid_flap(T, right, -1.0))
    if mode == "flaps":
        yield from together(go_home(left), go_home(right))
        yield from wait(3.0)
        return
    rt = yield from hold_base(T, right, -1.0, at=LID_HOLD, press=0.010, thin=True)      # or the box's back comes up too
    lt = yield from together_keep(rt.hold(60.0), raise_lid(T, left, 1.0))
    yield from together(lt.hold(1.0), rt.lift(0.090, 1.0))
    rt = yield from together_keep(lt.hold(60.0), support_lid(T, right, -1.0))
    yield from together_keep(rt.hold(60.0), push_lid_over(T, lt, 1.0))
    if mode == "raise":
        yield from wait(2.0)
        yield from together(go_home(left), go_home(right, lift=0.0))
        yield from wait(3.0)
        return
    yield from together(rt.hold(0.6), wait(0.6))
    if mode != "noflap":                                                     # (control: the front flap is left unfolded)
        yield from together_keep(rt.hold(60.0), fold_front_flap(T, left, 1.0))
    yield from rt.along(support_path(T, -1.0), SUPPORT_FROM, SUPPORT_TO, 2.0)
    yield from right.move(right.pos + (0.0, -0.075 * math.sin(SUPPORT_LEAN), 0.075 * math.cos(SUPPORT_LEAN)), 1.2)   # out along the fingers' own line
    yield from together(go_home(right, lift=0.0), press_lid(T, left, 1.0, stay=(mode == "held")))   # (control: the hand stays on the lid)
    if mode != "held":
        yield from go_home(left)
    yield from wait(3.0)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 2800, record: bool = True, window: bool = False,
        hold_s: float = 2.5, mode: str = "full", every: int = 5, cam: tuple = (320, 176), top_cam: str | None = None) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam[0],
                               cam_height=cam[1], headless=not window, **({'top_cam': top_cam} if top_cam else {}))
    T = emb.task.module
    obs = emb.reset(Scene(id="box-demo", instruction=T.INSTRUCTION))
    left, right = Hand("left", station.BASES["left"]), Hand("right", station.BASES["right"])
    gen = script(T, left, right, mode, idle_s=max_steps / CONTROL_HZ)
    frames: dict[str, list] = {}
    trace = []

    def snap_frames(o) -> None:
        if record:
            for name, img in o.images.items():
                frames.setdefault(name, []).append(np.asarray(img))

    def states():
        from yam_arena.scene import object_states   # importable only once the simulator has started

        return object_states(emb._env, [T.BLANK])

    def measure() -> None:
        from boxkit import finger_points, hand_rotation

        TOOL["left"], TOOL["right"] = (np.asarray(p, float) for p in finger_points(emb._env))
        for h in ("left", "right"):
            ROT[h] = hand_rotation(emb._env, h)

    def hands() -> dict:
        from boxkit import _np

        out_ = {}
        c = T.blank_pos()
        for h in (left, right):
            robot = emb._env.unwrapped.scene[f"{h.name}_robot"]
            q, jn = _np(robot.data.joint_pos)[0], list(robot.joint_names)
            tip = TOOL[h.name] + TIP * ROT[h.name][:, 2]
            out_[h.name] = {"tip_box_mm": [round(1000 * float(v), 1) for v in tip - c + [-T.K.XF, 0.0, 0.0]],
                            "err_mm": [round(1000 * float(v), 1) for v in TOOL[h.name] - h.pos],
                            "gap_mm": round(1000 * sum(abs(float(q[jn.index(j)])) for j in ("joint7", "joint8")), 1),
                            "droop_deg": round(math.degrees(float(np.linalg.norm(rotvec(h.rot @ ROT[h.name].T)))), 1)}
        return out_

    snap_frames(obs)
    measure()
    start = {"task": T.report(states())}
    steps, ended, success, info = 0, None, False, {}
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(Action(data=pack(left.cmd, right.cmd)))
        steps += 1
        measure()
        info = res.info
        snap_frames(res.observation)
        if steps % every == 0:
            Rb = np.asarray(states()[T.BLANK].panels["base"][0], float)     # (for the log only: the script reads none of this)
            trace.append({"step": steps, "score": info.get("score"), "task": T.report(states()), "hands": hands(),
                          "yaw_deg": round(math.degrees(math.atan2(float(Rb[1, 0]), float(Rb[0, 0]))), 2)})
        if steps % 40 == 0 and trace:
            t = trace[-1]
            print(f"step {steps} score {t['score']} {t['task']['folds']} base {t['task']['base_mm']} {t['hands']}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    result = {"lot": T.LOT, "mode": mode, "task_score": 1.0 if success else float(info.get("score", 0.0) or 0.0),
              "success": success, "steps": steps, "script_steps": ended,
              "worst_ik": {h.name: {"mm": round(h.worst_mm, 2), "deg": round(h.worst_deg, 2)} for h in (left, right)},
              "start": start, "end": {"task": T.report(states())}, "trace": trace}
    if record:
        import imageio

        for name, fs in frames.items():
            _write_mp4(fs, out / f"demo_{name}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2, default=lambda o: o.item()))
    print(json.dumps({k: v for k, v in result.items() if k != "trace"}), flush=True)
    return result


MODES = ("full", "push", "still", "walls", "tabs", "front", "roll", "lock", "lid", "flaps", "raise", "noprebreak", "notabs", "noflap", "held")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2800)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--mode", default="full", choices=MODES)
    ap.add_argument("--trace-every", type=int, default=5, help="steps between entries of the trace in result.json")
    ap.add_argument("--cam", default="320x176", help="recorded frame size, e.g. 640x352")
    ap.add_argument("--top-cam", default=None, choices=("calibrated", "demo"), help="the station's overhead camera setting")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, mode=a.mode, every=a.trace_every,
        cam=tuple(int(v) for v in a.cam.split("x")), top_cam=a.top_cam)
