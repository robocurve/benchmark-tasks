"""Demo for the play-a-card task, run in the yam_arena embodiment.

    python demo.py task.py out_dir [--window] [--max-steps 1000] [--no-record] [--mode full|hold|tight|loose|still]

The left gripper starts holding the fan of five cards, pinched at their corners (the task's start). Before it moves,
the demo plans everything from the task's layout (``plan``): where the right hand pinches the ace (its pads on the
ace and clear of every other card, its palm clear of every card and of the left hand, a pose the arm reaches), where
it comes in from, how far the ace is drawn, and where it is let go.

The right hand comes down beside the fan with its fingers open, slides in along the fan's plane either side of the
ace's free edge, and closes on it. The left hand eases its grip: commanded a little inside the five cards' thickness
(0.13 mm each side), it presses on them lightly, enough to keep them but not to stop the ace sliding. The right hand
draws the ace out along its length until it has left the left pads (40 mm); the left hand closes again on the four
cards left; the right hand draws the ace on until it is clear of them (110 mm in all), lifts it off the fan, carries
it face up over the blue cross, tipped 30 degrees, lowers its lowest corner to 4 mm above the table and lets it go.
It falls flat, face up.

Controls: ``--mode hold`` keeps both arms still. ``--mode tight`` does as the demo but never eases the left hand's
grip. ``--mode loose`` opens the left hand instead of easing it. ``--mode still`` is the same as ``hold``.

The script is open loop. Every target comes from the layout in the task file and the card's size; the only feedback
is each hand's own measured position (the robot's proprioception), to correct for the arm's sag. It never reads the
state of a card.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
FULL_OPEN = 0.0939                 # m between the pads at grip command 1.0


def gap(width: float) -> float:
    """Grip command (1 open .. 0 closed) for a gap between the pads in metres."""
    return min(1.0, max(0.0, width / FULL_OPEN))


class Hand:
    """One arm: smooth moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos, q=None, grip: float = 1.0):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY if q is None else q, float)
        self.grip = grip
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


def both(*gens):
    """Generator: run several moves side by side until all have finished."""
    live = list(gens)
    while live:
        for g in list(live):
            try:
                next(g)
            except StopIteration:
                live.remove(g)
        if live:
            yield


def best_ik(hand, pos, rot):
    """Joint angles for ``pos``/``rot`` from the best of many seeds: the one that leaves the joints most room."""
    from yam_arena import ik

    rng = np.random.default_rng(1)
    seeds = [hand.q.copy()] + [rng.uniform(ik.LOWER, ik.UPPER) for _ in range(30)]
    best = None
    for seed in seeds:
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), seed, base_pos=hand.base, iters=300)
        room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
        moved = float(np.abs(q - hand.q).sum())
        cost = (pe > 5e-4 or re > 5e-3, round(moved, 1), -room)
        if best is None or cost < best[1]:
            best = (q, cost)
    return best[0]


TOOL: dict = {}                          # where each hand's tool point actually is (updated every control step)
TOOLROT: dict = {}                       # ... and which way the hand actually points
PHASE: dict = {"name": "start"}
SAGS: dict = {}
GAIN, BIAS_MAX = 0.25, 0.025             # how fast, and how far, a hand corrects for its own sag
TURN_GAIN, TURN_MAX = 0.15, math.radians(8.0)


def _rotvec(rot) -> np.ndarray:
    a = math.acos(float(np.clip((np.trace(rot) - 1.0) / 2.0, -1.0, 1.0)))
    if a < 1e-9:
        return np.zeros(3)
    return np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]]) * (a / (2.0 * math.sin(a)))


def _turn(vec) -> np.ndarray:
    a = float(np.linalg.norm(vec))
    if a < 1e-12:
        return np.eye(3)
    k = np.asarray(vec, float) / a
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(a) * kx + (1.0 - math.cos(a)) * (kx @ kx)


def _square(rot) -> np.ndarray:
    out = np.zeros((3, 3))
    m = np.abs(rot).copy()
    for _ in range(3):
        i, j = np.unravel_index(int(np.argmax(m)), m.shape)
        out[i, j] = math.copysign(1.0, rot[i, j])
        m[i, :] = -1.0
        m[:, j] = -1.0
    return out


class Sag:
    """Slowly offsets a hand's command by its measured error, so that its tool point ends up where it is wanted, and
    turns the command by the hand's measured error in direction (as in the card-dealing task's demo)."""

    def __init__(self, hand):
        self.hand, self.bias, self.want = hand, np.zeros(3), None
        self.trim, self.want_rot, self.frame, self.off = np.zeros(3), None, None, 0.0
        self.live = True

    def measured(self):
        rot = TOOLROT.get(self.hand.name)
        if rot is None:
            return None
        if self.frame is None:
            self.frame = _square(rot.T @ self.hand.rot)
        return rot @ self.frame

    def to(self, pos, rot):
        if self.want is not None and self.hand.name in TOOL and self.live:
            self.bias = np.clip(self.bias + GAIN * (self.want - TOOL[self.hand.name]), -BIAS_MAX, BIAS_MAX)
        got = self.measured()
        if self.want_rot is not None and got is not None and self.live:
            err = _rotvec(self.want_rot @ got.T)
            self.off = math.degrees(float(np.linalg.norm(err)))
            trim = self.trim + TURN_GAIN * err
            size = float(np.linalg.norm(trim))
            self.trim = trim if size <= TURN_MAX else trim * (TURN_MAX / size)
        self.want, self.want_rot = np.asarray(pos, float), rot
        self.hand._solve(self.want + self.bias, _turn(self.trim) @ rot)


def glide(sag, target, seconds: float, rot, rot_to=None):
    """Generator: take the hand's tool point in a straight line to ``target`` (turning from ``rot`` to ``rot_to``),
    correcting for sag on the way."""
    from yam_arena import ik

    start = sag.want if sag.want is not None else sag.hand.pos.copy()
    n = max(1, round(seconds * CONTROL_HZ))
    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        r = rot if rot_to is None else ik.slerp_matrix(rot, rot_to, s)
        sag.to(start + s * (np.asarray(target, float) - start), r)
        yield


def go(sag, pos, rot, seconds: float, grip=None):
    """Generator: swing the arm to a pose it is not yet near, by its joints, then settle on it."""
    yield from sag.hand.move_joints(best_ik(sag.hand, pos, rot), seconds, grip=grip)
    sag.want = None
    yield from glide(sag, pos, 0.5, rot)


# -- the plan, from the task's layout ---------------------------------------------------------------------------------
# The gripper's shape in the fan's plane (from its collision meshes): the pads' faces, and the parts that cross the
# plane further back (the finger carriages and the palm). Along the approach from the tool point (m), and half width.
PAD_ZONE = (-0.005, 0.013, 0.0136)
BODY_ZONE = (-0.165, 0.016, 0.037)       # everything of the gripper that can touch the plane: pads to palm
PALM_ZONE = (-0.165, -0.053, 0.037)
PAD_CLEAR = 0.004                        # the right pads stay this far from every card but the ace, in the plane
PALM_CLEAR = 0.006                       # the palm this far from every card
LEFT_CLEAR = 0.015                       # and the right hand this far from the left gripper
AREA_MIN = 200e-6                        # m2 of pad on the ace
OPEN = 0.028                             # the right hand comes in with its pads this far apart
ABOVE = 0.060                            # ... from this far off the fan's plane, on the faces' side
PULL_SPEED = 0.03                        # m/s while the ace is drawn
EASE = float(os.environ.get("CARDS_EASE", "0.00013"))  # the left hand is commanded this much less than the stack's
#                                        thickness, per side, while the ace leaves its pads
DRAW_MARGIN = 0.006                      # the ace is drawn this much past where it clears the left pads
LIFT = 0.040                             # the drawn ace is lifted this far off the fan
TILT = math.radians(float(os.environ.get("CARDS_TILT", "30")))   # it is let go face up, tipped this far
EDGE_UP = 0.004                          # ... with its lowest corner this far above the table
GRIP_UP = 0.012                          # and every part of the gripper at least this far above it
# Points of the right gripper in its own frame (x closing, y across, z approach; tool point at the origin), for the
# clearance from the table: fingertips, finger bodies, carriages, palm.
GRIPPER_PTS = np.array([[sx * dx, sy * dy, z] for sx in (-1, 1) for sy in (-1, 1)
                        for dx, dy, z in ((0.010, 0.0136, 0.013), (0.010, 0.0136, -0.005), (0.013, 0.020, -0.040),
                                          (0.048, 0.034, -0.061), (0.048, 0.034, -0.081), (0.050, 0.045, -0.120),
                                          (0.050, 0.045, -0.160))])


def frame(pos, rot) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = rot, pos
    return m


def _unit2(ang: float) -> np.ndarray:
    return np.array([math.cos(ang), math.sin(ang)])


def _rect(c2, ang: float, zone) -> np.ndarray:
    """A gripper zone in the fan's plane: tool point ``c2``, approach ``ang`` (in the fan's frame)."""
    a = _unit2(ang)
    s = np.array([-a[1], a[0]])
    z0, z1, h = zone
    return np.array([c2 + z0 * a - h * s, c2 + z1 * a - h * s, c2 + z1 * a + h * s, c2 + z0 * a + h * s])


def _overlap(p, q, margin: float = 0.0) -> bool:
    """Whether two convex polygons come within ``margin`` of each other (separating axes)."""
    for poly in (p, q):
        for i in range(len(poly)):
            e = poly[(i + 1) % len(poly)] - poly[i]
            n = np.array([e[1], -e[0]]) / np.linalg.norm(e)
            a, b = p @ n, q @ n
            if a.min() > b.max() + margin or b.min() > a.max() + margin:
                return False
    return True


def _inpoly(P, poly, margin: float = 0.0) -> np.ndarray:
    """Which of the points ``P`` (..., 2) are inside the anticlockwise convex polygon, or within ``margin`` of it."""
    ok = np.ones(P.shape[:-1], bool)
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        e = b - a
        ok &= ((P - a) @ (np.array([e[1], -e[0]]) / np.linalg.norm(e))) <= margin
    return ok


def _solve(pos, rot, seeds, base) -> tuple:
    """IK from several seeds: (joints, reached) for the best."""
    from yam_arena import ik

    best = None
    for seed in seeds:
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), seed, base_pos=base, iters=300)
        room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
        cost = (not (pe < 1e-3 and re < 0.01), -room)
        if best is None or cost < best[0]:
            best = (cost, q)
        if not cost[0] and room > 0.15:
            break
    return best[1], not best[0][0]


def _candidates(T, left2, res: float = 0.001) -> list:
    """Pinch places on the ace: for each approach (every 5 degrees in the fan's frame), the tool point nearest the
    ace's middle at which the pads lie on the ace (at least AREA_MIN of them) and clear of every other card, and the
    palm clear of every card; the right hand clear of the left gripper. Returns (distance from the ace's middle,
    approach, tool point in the fan's frame, pad area on the ace), nearest first."""
    polys = [T.card_polygon(k) for k in range(T.N)]
    left = _rect(left2, T.HOLD_APPROACH, BODY_ZONE)
    mid = T._r2(T.fan_angle(T.PLAY)) @ T.CARD_MID
    span = np.arange(-0.22, 0.22, res)
    n = len(span)
    I, J = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
    out = []
    for deg in range(0, 360, 5):
        ang = math.radians(deg)
        a = _unit2(ang)
        s = np.array([-a[1], a[0]])
        P = span[:, None, None] * a + span[None, :, None] * s
        ace = _inpoly(P, polys[T.PLAY]).astype(float)
        oth = _inpoly(P, left, LEFT_CLEAR).astype(float)
        allc = oth.copy()
        for k, p in enumerate(polys):
            allc = np.maximum(allc, _inpoly(P, p, PALM_CLEAR))
            if k != T.PLAY:
                oth = np.maximum(oth, _inpoly(P, p, PAD_CLEAR))

        def sums(m, zone):
            S = np.zeros((n + 1, n + 1))
            S[1:, 1:] = m.cumsum(0).cumsum(1)
            i0, i1 = I + int(round(zone[0] / res)), I + int(round(zone[1] / res))
            w = int(round(zone[2] / res))
            j0, j1 = J - w, J + w + 1
            ok = (i0 >= 0) & (i1 <= n) & (j0 >= 0) & (j1 <= n)
            v = np.full(I.shape, np.nan)
            v[ok] = S[i1[ok], j1[ok]] - S[i0[ok], j1[ok]] - S[i1[ok], j0[ok]] + S[i0[ok], j0[ok]]
            return v

        on_ace, on_other, palm = sums(ace, PAD_ZONE) * res * res, sums(oth, PAD_ZONE), sums(allc, PALM_ZONE)
        good = (on_other == 0) & (palm == 0) & (on_ace >= AREA_MIN)
        if not good.any():
            continue
        tcp = span[I][..., None] * a + span[J][..., None] * s
        dist = np.linalg.norm(tcp - mid, axis=-1)
        dist[~good] = np.inf
        i, j = np.unravel_index(int(np.argmin(dist)), dist.shape)
        out.append((float(dist[i, j]), ang, tcp[i, j], float(on_ace[i, j])))
    return sorted(out, key=lambda c: c[0])


def _draw(T, tcp2, ang, left2) -> tuple | None:
    """How far the ace is drawn along its length (m): to where it clears the left pads (``s1``), then to where it
    clears every other card (``s2``). None if the right hand's pads or palm would pass over another card on the way,
    or the palm over the left gripper."""
    d2 = T._r2(T.fan_angle(T.PLAY)) @ np.array([0.0, 1.0])
    ace = T.card_polygon(T.PLAY)
    others = [T.card_polygon(k) for k in range(T.N) if k != T.PLAY]
    lpad = _rect(left2, T.HOLD_APPROACH, PAD_ZONE)
    left = _rect(left2, T.HOLD_APPROACH, BODY_ZONE)
    s1 = s2 = None
    for s in np.arange(0.0, 0.16, 0.001):
        moved = ace + s * d2
        if s1 is None and not _overlap(moved, lpad, 0.002):
            s1 = s + DRAW_MARGIN
        if s1 is not None and s >= s1 and not any(_overlap(moved, o, 0.004) for o in others):
            s2 = s
            break
    if s1 is None or s2 is None:
        return None
    for s in np.arange(0.0, s2 + 1e-9, 0.002):
        pad, palm = _rect(tcp2 + s * d2, ang, PAD_ZONE), _rect(tcp2 + s * d2, ang, PALM_ZONE)
        if any(_overlap(pad, o, 0.002) or _overlap(palm, o, 0.004) for o in others) or _overlap(palm, left, 0.010):
            return None
    return d2, s1, s2


def _place(T, card_in_tcp, base, seeds) -> tuple | None:
    """Where the right tool point goes to let the ace go: the ace face up, tipped TILT about a level line so that the
    part ahead of the tool point is lowest, that corner EDGE_UP above the table, its middle over the cross, every part
    of the gripper at least GRIP_UP above the table, and the arm able to reach it. Turned about the vertical as little
    as will do. Returns (tool point, rotation, joints) or None."""
    face = T.FACE / np.linalg.norm(T.FACE)
    axis = np.cross(face, [0.0, 0.0, 1.0])
    level = _turn(axis / np.linalg.norm(axis) * math.acos(float(np.clip(face[2], -1.0, 1.0))))
    _, quat = T.card_start(T.PLAY)
    flat0 = level @ T.K._rotation(quat)                          # face up, level, as it lay in the fan
    tool_in_card = np.linalg.inv(card_in_tcp)
    local = np.array([[sx * T.K.CARD_W / 2, sy * T.K.CARD_L / 2, 0.0] for sx in (-1, 1) for sy in (-1, 1)])
    for yaw in sorted(range(-180, 180, 10), key=abs):
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        flat = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ flat0
        tool_flat = flat @ card_in_tcp[:3, :3].T                 # the tool's rotation with the card lying so
        a = tool_flat[:, 2]
        h = np.cross([0.0, 0.0, 1.0], a)
        if np.linalg.norm(h) < 1e-6:
            continue
        tilt = _turn(h / np.linalg.norm(h) * TILT)               # dips the approach, and the card ahead of it
        rot_card = tilt @ flat
        low = float((local @ rot_card.T)[:, 2].min())
        pos = np.array([T.CROSS_XY[0], T.CROSS_XY[1], T.TOP + EDGE_UP - low])
        tool = frame(pos, rot_card) @ tool_in_card
        if float((GRIPPER_PTS @ tool[:3, :3].T + tool[:3, 3])[:, 2].min()) < T.TOP + GRIP_UP:
            continue
        q, ok = _solve(tool[:3, 3], tool[:3, :3], seeds, base)
        if ok:
            return tool[:3, 3], tool[:3, :3], q
    return None


def plan(T) -> dict:
    """The whole demo's plan from the task's layout alone: where the right hand pinches the ace, from where it comes
    in, how far the ace is drawn, and where it is let go. Deterministic."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    base = station.BASES["right"]
    rng = np.random.default_rng(3)
    seeds = [np.array(READY, float)] + [rng.uniform(ik.LOWER, ik.UPPER) for _ in range(24)]
    left2 = T.to_plane(T.hold_frame()[0])[:2]
    face = T.FACE / np.linalg.norm(T.FACE)
    tried = 0
    for dist, ang, tcp2, area in _candidates(T, left2):
        tried += 1
        drawn = _draw(T, tcp2, ang, left2)
        if drawn is None:
            continue
        d2, s1, s2 = drawn
        # from where it comes in: back along its approach until all of it is clear of the cards and the left hand
        polys = [T.card_polygon(k) for k in range(T.N)]
        left = _rect(left2, T.HOLD_APPROACH, BODY_ZONE)
        back = None
        for b in np.arange(0.02, 0.20, 0.002):
            body = _rect(tcp2 - b * _unit2(ang), ang, BODY_ZONE)
            if not any(_overlap(body, p, 0.006) for p in polys) and not _overlap(body, left, 0.015):
                back = float(b)
                break
        if back is None:
            continue
        z = T.plane_dir(_unit2(ang))
        tcp = T.to_world(tcp2)
        d = T.plane_dir(d2)
        for x in (face, -face):                                   # either way round: the gripper is symmetric
            rot = np.column_stack([x, np.cross(z, x), z])
            q_pinch, ok = _solve(tcp, rot, seeds, base)
            if not ok:
                continue
            chain = [tcp - back * z + ABOVE * face, tcp - back * z, tcp + s2 * d, tcp + s2 * d + LIFT * face]
            if not all(_solve(p, rot, [q_pinch] + seeds[:6], base)[1] for p in chain):
                continue
            pos, quat = T.card_start(T.PLAY)
            card_in_tcp = np.linalg.inv(frame(tcp, rot)) @ frame(pos, T.K._rotation(quat))
            placed = _place(T, card_in_tcp, base, [q_pinch] + seeds)
            if placed is None:
                continue
            return {"tcp": tcp, "rot": rot, "approach_deg": math.degrees(ang), "tcp2_mm": (1000 * tcp2).round(1),
                    "pad_on_ace_mm2": round(area * 1e6), "from_middle_mm": round(1000 * dist, 1), "back": back,
                    "draw_dir": d, "s1": float(s1), "s2": float(s2), "place_tcp": placed[0], "place_rot": placed[1],
                    "candidates_tried": tried}
    raise RuntimeError(f"no pinch on the ace that the right arm can carry through ({tried} places tried)")


PLAN: dict = {}


def script(T, left, right, mode: str = "full", idle_s: float = 45.0):
    """The demo as a generator: one yield per control step."""
    if mode in ("still", "hold"):
        yield from wait(idle_s)
        return
    ls, rs = Sag(left), Sag(right)
    ls.live = False                          # the left hand stays where the task put it
    SAGS.update(left=ls, right=rs)
    P = PLAN
    tcp, rot, face = P["tcp"], P["rot"], T.FACE / np.linalg.norm(T.FACE)
    back = tcp - P["back"] * rot[:, 2]
    PHASE["name"] = "reach"
    yield from go(rs, back + ABOVE * face, rot, 2.5, grip=gap(OPEN))
    yield from glide(rs, back + ABOVE * face, 0.4, rot)
    PHASE["name"] = "down"
    yield from glide(rs, back, 1.2, rot)
    yield from glide(rs, back, 0.3, rot)
    PHASE["name"] = "in"
    yield from glide(rs, tcp, max(1.0, P["back"] / 0.04), rot)
    yield from glide(rs, tcp, 0.5, rot)
    PHASE["name"] = "pinch"
    yield from right.set_grip(0.0, 0.8)
    yield from wait(0.5)
    if mode == "loose":                      # control: the left hand lets the whole hand go
        PHASE["name"] = "let the hand go"
        yield from left.set_grip(gap(T.stack_thickness() + 0.004), 0.6)
    elif mode != "tight":
        PHASE["name"] = "ease"
        yield from left.set_grip(gap(T.stack_thickness() - 2 * EASE), 0.6)
        yield from wait(0.3)
    d, s1, s2 = P["draw_dir"], P["s1"], P["s2"]
    PHASE["name"] = "draw"
    yield from glide(rs, tcp + s1 * d, s1 / PULL_SPEED, rot)
    if mode != "loose":
        PHASE["name"] = "close"
        yield from left.set_grip(0.0, 0.5)
    PHASE["name"] = "draw on"
    yield from glide(rs, tcp + s2 * d, max(0.5, (s2 - s1) / PULL_SPEED), rot)
    PHASE["name"] = "lift"
    lifted = tcp + s2 * d + LIFT * face
    yield from glide(rs, lifted, 1.0, rot)
    PHASE["name"] = "carry"
    ptcp, prot = P["place_tcp"], P["place_rot"]
    above = ptcp + np.array([0.0, 0.0, 0.05])
    yield from glide(rs, above, 3.0, rot, prot)
    yield from glide(rs, above, 0.5, prot)
    PHASE["name"] = "lower"
    yield from glide(rs, ptcp, 1.5, prot)
    yield from glide(rs, ptcp, 0.5, prot)
    PHASE["name"] = "let go"
    yield from right.set_grip(gap(0.030), 0.4)
    yield from wait(0.5)
    PHASE["name"] = "away"
    yield from glide(rs, ptcp - 0.06 * prot[:, 2], 1.0, prot)
    yield from glide(rs, ptcp - 0.06 * prot[:, 2] + np.array([0.0, 0.0, 0.08]), 1.0, prot)
    yield from wait(2.0)


def _write_mp4(frames, path: Path) -> None:
    import imageio

    with imageio.get_writer(path, fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=None,
                            ffmpeg_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"]) as w:
        for f in frames:
            w.append_data(f)


def run(task_path: str, out_dir: str, max_steps: int = 900, record: bool = True, window: bool = False,
        hold_s: float = 2.5, mode: str = "full", every: int = 5, cam: tuple = (320, 176)) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import station
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam[0],
                               cam_height=cam[1], headless=not window)
    T = emb.task.module
    obs = emb.reset(Scene(id="cards-demo", instruction=T.INSTRUCTION))
    state = np.asarray(obs.state["joint_pos"], float)
    left = Hand("left", station.BASES["left"], q=state[:6], grip=0.0)
    right = Hand("right", station.BASES["right"], q=state[7:13], grip=1.0)
    PLAN.clear()
    if mode not in ("still", "hold"):
        import time

        t0 = time.time()
        PLAN.update(plan(T))
        print("plan", json.dumps({k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in PLAN.items()}),
              f"({time.time() - t0:.0f} s)", flush=True)
    gen = script(T, left, right, mode, idle_s=max_steps / CONTROL_HZ)
    frames: dict[str, list] = {}
    trace = []

    def snap_frames(o) -> None:
        if record:
            for name, img in o.images.items():
                frames.setdefault(name, []).append(np.asarray(img))

    def box() -> dict:
        from yam_arena.scene import object_states

        st = object_states(emb._env, list(T.CARDS))
        out_ = {}
        for k, n in enumerate(T.CARDS):
            p, q = st[n].pos, st[n].quat_xyzw
            p0, _ = T.card_start(k)
            out_[n[-1]] = [round(1000 * float(v), 1) for v in (p - p0)] + [round(T.K.back_up(q), 2)]
        return out_

    def measure() -> None:
        from cardkit import finger_frames, finger_points

        TOOL["left"], TOOL["right"] = (np.asarray(p, float) for p in finger_points(emb._env))
        TOOLROT["left"], TOOLROT["right"] = finger_frames(emb._env)

    def hands() -> dict:
        from cardkit import finger_gaps

        g = finger_gaps(emb._env)
        out_ = {h.name: {"tool_mm": [round(1000 * float(v), 1) for v in TOOL[h.name] - [0.0, 0.0, T.TOP]],
                         "want_mm": [round(1000 * float(v), 1) for v in h.pos - [0.0, 0.0, T.TOP]],
                         "gap_mm": round(1000 * g[i], 2), "grip": round(float(h.grip), 4),
                         "off_deg": round(SAGS[h.name].off, 2) if h.name in SAGS else None}
                for i, h in enumerate((left, right))}
        # how far each right pad is from each card (mm): which cards the right hand touches
        from yam_arena.scene import object_states

        st = object_states(emb._env, list(T.CARDS))
        for side, key in ((-1.0, "rpad_a_mm"), (1.0, "rpad_b_mm")):
            pts = T.pad_points(emb._env, "right", sides=(side,))
            out_["right"][key] = [round(1000 * T.K.card_distance(pts, st[n].pos, st[n].quat_xyzw), 2) for n in T.CARDS]
        return out_

    snap_frames(obs)
    measure()
    start = {"task": T.report(None), "box": box(), "hands": hands()}
    print("start", json.dumps(start), flush=True)
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
            trace.append({"step": steps, "phase": PHASE["name"], "score": info.get("score"), "task": T.report(None),
                          "hands": hands(), "box": box()})
        if steps % 20 == 0 and trace:
            t = trace[-1]
            print(f"step {steps} {t['phase']} score {t['score']} hold {t['task']['hold_mm']} cards {t['box']} "
                  f"L {t['hands']['left']} R {t['hands']['right']}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    result = {"mode": mode, "ease_mm": 1000 * EASE, "ace_at": T.PLAY, "recline_deg": round(math.degrees(T.RECLINE)),
              "plan": {k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in PLAN.items()}, "task_score": 1.0 if success else float(info.get("score", 0.0) or 0.0),
              "success": success, "steps": steps, "script_steps": ended,
              "worst_ik": {h.name: {"mm": round(h.worst_mm, 2), "deg": round(h.worst_deg, 2)} for h in (left, right)},
              "start": start, "end": {"task": T.report(None), "box": box(), "hands": hands()}, "trace": trace}
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


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=900)
    ap.add_argument("--window", action="store_true", help="show the Isaac Sim window on the instance's desktop")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--mode", default="full", choices=("full", "hold", "tight", "loose", "still"))
    ap.add_argument("--trace-every", type=int, default=5, help="steps between entries of the trace in result.json")
    ap.add_argument("--cam", default="320x176", help="recorded frame size, e.g. 640x352")
    a = ap.parse_args()
    sys.path.insert(0, str(Path(a.task).resolve().parent))
    run(a.task, a.out, a.max_steps, record=not a.no_record, window=a.window, mode=a.mode, every=a.trace_every,
        cam=tuple(int(v) for v in a.cam.split("x")))
