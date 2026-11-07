"""Scoring of the conveyor photo-build task from object states. Pure Python and NumPy: no Isaac Sim.

A state is anything with ``pos`` (centre), ``quat_xyzw``, ``lin_vel`` and ``ang_vel`` (world frame), as
yam_arena's ``ObjectState``. Every test uses the part's centre, orientation and true extent (box corners,
cylinder rims), never its world-aligned bounding box.

The model in the photo: the two tall green cylinders (``post_a``, ``post_b``) standing on the marks, the
yellow beam (``beam``) across them and the red cube (``cap``) on the middle of the beam. The extra parts are
the green square post, the orange beam, the blue cube and the short green cylinder (``short_post``).

Rules (also in task.json):
- Jam gate: a part held back on the running belt jams the line, and the trial scores 0 from then on. Held
  back: it rests on the belt (its lowest point within 5 mm of the surface) for 3 s in a row and goes less
  than 30 mm downstream in that time; the belt carries a free part 120 mm. A part knocked over on the belt
  (its centre shifts by up to half its height) is not held back by that alone.
- Gate: each post in place 0.2 (two posts), the beam 0.3, the cap 0.3.
  * post: standing (its 80 mm axis within 5 degrees of vertical), lowest point within 4 mm of the mark's
    top, centre within 10 mm of a mark's centre horizontally; one post per mark; either post on either mark.
  * beam: both posts in place; lying flat (its 30 mm axis within 5 degrees of vertical), its long axis
    within 10 degrees of the line through the marks, either way round; lowest point within 4 mm of the
    lower post top; centre within 15 mm horizontally of the midpoint between the two posts.
  * cap: the beam in place; one of its axes within 5 degrees of vertical; lowest point within 4 mm of the
    beam's top; centre within 10 mm horizontally of the beam's centre.
  * every counted part at rest: under 10 mm/s and 0.5 rad/s.
- Extra parts: 0.25 off for each of the four extra parts that has not ridden off the downstream end of the
  belt (centre past the table's right edge and 50 mm or more below the table top, as in the tote), never
  below 0.
- Success: the four model parts in place, all four extra parts gone downstream, no jam; held 1 s.
"""
from __future__ import annotations

import math

import numpy as np

from layout import (BEAM_H, BEAM_TOP, BELT_END, BELT_TOP, BELT_UP, BELT_W, BELT_X, EXTRA, GATE_X, GATE_Y, MARK_TOP,
                    MARKS, NEEDED, ORDER, PARTS, POST_H, POST_TOP, TABLE_EDGE_Y, TOP, rotation, start_y,
                    z_extent)

CONTROL_DT = 0.05  # 20 Hz
POST_POINTS, BEAM_POINTS, CAP_POINTS = 0.2, 0.3, 0.3
EXTRA_PENALTY = 0.25
UPRIGHT = math.radians(5.0)
ALONG = math.radians(10.0)
LEVEL = 0.004  # lowest point to the surface it stands on
MARK_TOL = 0.010
BEAM_TOL = 0.015
CAP_TOL = 0.010
EPS = 1e-4  # an offset exactly at a tolerance counts
REST_SPEED, REST_SPIN = 0.010, 0.5
JAM_TRAVEL = 0.030  # held back: less than this downstream ...
JAM_S = 3.0  # ... in 3 s resting on the belt, which carries a free part 120 mm; stopped about 2.25 s or more
ON_BELT = 0.005  # resting on the belt: lowest point within this of its surface
FRESH = 0.025  # every part within this of its start: a new trial
GONE_DROP = 0.05  # gone downstream: fallen at least this far below the table top, past its right edge


def _vertical_axis(state, local):
    """Angle (rad) between a part's local axis and the world vertical, either way up."""
    axis = rotation(state.quat_xyzw)[:, local]
    return math.acos(min(1.0, abs(float(axis[2]))))


def _lowest(name, state) -> float:
    return z_extent(name, state.pos, state.quat_xyzw)[0]


def _highest(name, state) -> float:
    return z_extent(name, state.pos, state.quat_xyzw)[1]


def _flat_xy(a, b) -> float:
    return float(np.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1])))


def resting(state) -> bool:
    return bool(np.linalg.norm(state.lin_vel) < REST_SPEED and np.linalg.norm(state.ang_vel) < REST_SPIN)


def on_belt(name, state) -> bool:
    x, y = float(state.pos[0]), float(state.pos[1])
    return bool(abs(x - BELT_X) <= BELT_W / 2 and BELT_END <= y <= BELT_UP
                and abs(_lowest(name, state) - BELT_TOP) <= ON_BELT)


def gone_downstream(state) -> bool:
    """Rode off the end of the belt: past the table's right edge and fallen below its top (into the tote)."""
    return bool(float(state.pos[1]) < -TABLE_EDGE_Y and float(state.pos[2]) < TOP - GONE_DROP)


def post_mark(name, state):
    """Index of the mark the post stands on, or None."""
    if _vertical_axis(state, 2) > UPRIGHT + EPS or abs(_lowest(name, state) - MARK_TOP) > LEVEL + EPS:
        return None
    for i, mark in enumerate(MARKS):
        if _flat_xy(state.pos, mark) <= MARK_TOL + EPS:
            return i
    return None


def posts_in_place(objects) -> list[str]:
    marks = {}
    for name in ('post_a', 'post_b'):
        s = objects[name]
        i = post_mark(name, s)
        if i is not None and resting(s):
            marks.setdefault(i, name)
    return sorted(marks.values())


def beam_in_place(objects, posts) -> bool:
    if len(posts) < 2:
        return False
    s = objects['beam']
    if not resting(s) or _vertical_axis(s, 2) > UPRIGHT + EPS:
        return False
    long_axis = rotation(s.quat_xyzw)[:, 1]
    line = np.array([MARKS[0][0] - MARKS[1][0], MARKS[0][1] - MARKS[1][1]])
    cos = abs(float(long_axis[:2] @ line)) / (float(np.linalg.norm(long_axis[:2])) * float(np.linalg.norm(line)) + 1e-12)
    if math.acos(min(1.0, cos)) > ALONG + EPS:
        return False
    tops = [_highest(p, objects[p]) for p in posts]
    if abs(_lowest('beam', s) - min(tops)) > LEVEL + EPS:
        return False
    mid = (np.asarray(objects[posts[0]].pos) + np.asarray(objects[posts[1]].pos)) / 2
    return _flat_xy(s.pos, mid) <= BEAM_TOL + EPS


def cap_in_place(objects) -> bool:
    s, beam = objects['cap'], objects['beam']
    if not resting(s) or min(_vertical_axis(s, i) for i in range(3)) > UPRIGHT + EPS:
        return False
    beam_top = _highest('beam', beam)
    if abs(_lowest('cap', s) - beam_top) > LEVEL + EPS:
        return False
    return _flat_xy(s.pos, beam.pos) <= CAP_TOL + EPS


def gate(objects) -> dict:
    posts = posts_in_place(objects)
    beam = beam_in_place(objects, posts)
    cap = beam and cap_in_place(objects)
    return {'posts': posts, 'beam': beam, 'cap': cap}


def extras_left(objects) -> list[str]:
    return [n for n in EXTRA if not gone_downstream(objects[n])]


class Trial:
    """Watches the belt for a part held back on it. Cleared when every part is back at its start, which only a
    reset does: the running belt carries every untouched part 40 mm downstream each second."""

    def __init__(self):
        self.jammed = False
        self.track = {n: [] for n in ORDER}  # y of each step a part has rested on the belt, newest last
        self.steps = 0

    def fresh(self, objects) -> bool:
        for n in ORDER:
            p = objects[n].pos
            if abs(float(p[0]) - BELT_X) > FRESH or abs(float(p[1]) - start_y(n)) > FRESH:
                return False
        return True

    def observe(self, objects) -> None:
        fresh = self.fresh(objects)
        if fresh and not getattr(self, '_was_fresh', False):
            self.__init__()
        self._was_fresh = fresh
        self.steps += 1
        window = int(round(JAM_S / CONTROL_DT))
        for n in ORDER:
            s = objects[n]
            if not on_belt(n, s):
                self.track[n] = []
                continue
            # Downstream travel while resting on the belt, over the last 3 s.
            track = self.track[n]
            track.append(float(s.pos[1]))
            del track[:-(window + 1)]
            if len(track) > window and track[0] - track[-1] < JAM_TRAVEL:
                self.jammed = True

    def score(self, objects) -> float:
        if self.jammed:
            return 0.0
        g = gate(objects)
        points = POST_POINTS * len(g['posts']) + BEAM_POINTS * g['beam'] + CAP_POINTS * g['cap']
        return max(0.0, points - EXTRA_PENALTY * len(extras_left(objects)))

    def success(self, objects) -> bool:
        if self.jammed:
            return False
        g = gate(objects)
        return bool(len(g['posts']) == 2 and g['beam'] and g['cap'] and not extras_left(objects))

    def distance(self, objects) -> float:
        places = {'post_a': (*MARKS[0], MARK_TOP + POST_H / 2), 'post_b': (*MARKS[1], MARK_TOP + POST_H / 2),
                  'beam': (GATE_X, GATE_Y, POST_TOP + BEAM_H / 2), 'cap': (GATE_X, GATE_Y, BEAM_TOP + 0.02)}
        return float(sum(np.linalg.norm(np.asarray(objects[n].pos, dtype=float) - np.asarray(p))
                         for n, p in places.items()))

    def report(self, objects) -> dict:
        g = gate(objects)
        return {'jammed': self.jammed, 'posts': g['posts'], 'beam': bool(g['beam']), 'cap': bool(g['cap']),
                'extras_left': extras_left(objects), 'score': self.score(objects)}


assert set(NEEDED) | set(EXTRA) == set(PARTS)
