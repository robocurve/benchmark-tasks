"""Latch rule and scorer of the rack-rails task (engine-independent, numpy only).

Latch (per rail end, evaluated every physics step by the runtime from the measured member pose, ``Latch.step``):
* While the end's pegs are in its flange's holes (peg axis within 1.2 mm of the hole pair's axis, peg direction within
  1.5 deg of the post normal) and its bracket is within the last 2.5 mm of travel to the flange, the catch pushes the
  member back with 10 N (the robot must push harder than that to close the gap: a light touch does not latch it, and
  an end pushed only part way springs back out of the zone when let go).
* When the bracket face reaches the flange (gap <= 0.4 mm) under those conditions, the catch drops in: the end is
  latched, at whatever U it is in. From then on it cannot back out: a stiff spring (20 kN/m, up to 150 N) holds the
  bracket on the flange. Nothing pulls a free end in, centres it or holds it sideways; the pegs in the square holes do.

Score (value at the end of the trial, ``Scorer.step`` every control step):
  0     if either end is latched or pegged at any U other than the target (a sloped or misplaced rail);
  0.25  one end latched at the target U;
  0.5   one end latched at the target U and the other end's pegs in the flange at the target U (not latched), or both
        latched with a hand still on / within 10 cm of the rail;
  1.0   both ends latched at the target U, every fingertip 10 cm or more from the rail, rail still for 1 s (ends the
        trial);
  0     otherwise (rail in the cradle, held in the air, pegs not in).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import layout as L
import numpy as np

CLEAR_M = 0.100  # hands 10 cm clear of the rail (the overhead camera then sees both ends)
STILL_V_M_S = 0.002
HOLD_STEPS = 20  # 1 s at 20 Hz
ENDS = ("rear", "front")


def in_zone(st):
    return (
        st["gap"] <= L.LATCH_ZONE
        and st["gap"] > -0.002
        and abs(st["axis_dx"]) <= L.LATCH_ALIGN_M
        and abs(st["axis_dz"]) <= L.LATCH_ALIGN_M
        and st["tilt_deg"] <= L.LATCH_TILT_DEG
    )


@dataclass
class Latch:
    """Per-end catch state. ``step`` returns the force (N, along the end's outward post normal; + = toward the post)
    that the catch puts on the member this physics step."""

    latched: dict = field(default_factory=lambda: {e: False for e in ENDS})
    latched_u: dict = field(default_factory=lambda: {e: None for e in ENDS})
    events: list = field(default_factory=list)

    def step(self, end, st, v_out, t=0.0):
        """st: layout.end_state(...) of the end; v_out: the member's velocity along the outward normal (m/s)."""
        if self.latched[end]:
            back = max(st["gap"], 0.0)
            f = L.LATCH_K * back - L.LATCH_C * min(v_out, 0.0)
            return float(min(max(f, 0.0), L.LATCH_HOLD_N))
        if not in_zone(st):
            return 0.0
        if st["gap"] <= L.SEAT_GAP:
            self.latched[end] = True
            self.latched_u[end] = st["u"]
            self.events.append(dict(end=end, u=st["u"], t=round(t, 4)))
            return 0.0
        return -L.LATCH_PUSH_N


def pegged(st):
    """The end's pegs are through the flange into the holes (gap below PEG_L - 1 mm, axis inside the holes' play)."""
    return bool(st["in_holes"])


def box_distance(p, x0, x1, y0, y1, z0, z1):
    d = np.maximum(np.maximum([x0 - p[0], y0 - p[1], z0 - p[2]], 0.0), [p[0] - x1, p[1] - y1, p[2] - z1])
    return float(np.linalg.norm(np.maximum(d, 0.0)))


def rail_distance(point, po, ro, pi, ri):
    """Distance from a point to the rail (outer member with its rear bracket, inner member with its front bracket)."""
    q = ro.T @ (np.asarray(point, float) - po)
    d1 = box_distance(q, L.OUTER_X[0], L.OUTER_X[1], -L.OUTER_LEN, 0.0, -L.OUTER_Z, L.OUTER_Z)
    d2 = box_distance(q, L.BRACKET_X[0], L.BRACKET_X[1], -L.BRACKET_T, L.PEG_L, -L.BRACKET_Z, L.BRACKET_Z)
    q = ri.T @ (np.asarray(point, float) - pi)
    d3 = box_distance(q, L.INNER_X[0], L.INNER_X[1], 0.0, L.INNER_LEN, -L.INNER_Z, L.INNER_Z)
    d4 = box_distance(q, L.BRACKET_X[0], L.BRACKET_X[1], -L.PEG_L, L.BRACKET_T, -L.BRACKET_Z, L.BRACKET_Z)
    return min(d1, d2, d3, d4)


@dataclass
class Measure:
    target_u: int
    rear: dict
    front: dict
    latched: dict
    latched_u: dict
    tip_mm: float
    extension_mm: float


def measure(spec, po, ro, pi, ri, latch: Latch, tips):
    rear = L.end_state("rear", po, ro, spec)
    front = L.end_state("front", pi, ri, spec)
    tip = min(rail_distance(t, po, ro, pi, ri) for t in tips) if tips else 1.0
    ext = float(-(ro.T @ (pi - po))[1] - L.L0)
    return Measure(
        target_u=spec["target_u"],
        rear=rear,
        front=front,
        latched=dict(latch.latched),
        latched_u=dict(latch.latched_u),
        tip_mm=tip * 1e3,
        extension_mm=ext * 1e3,
    )


def level_of(m: Measure, clear: bool, still_steps: int):
    tu = m.target_u
    ends = {"rear": m.rear, "front": m.front}
    wrong = False
    for e, st in ends.items():
        if m.latched[e] and m.latched_u[e] != tu:
            wrong = True
        if pegged(st) and st["u"] != tu:
            wrong = True
    if wrong:
        return 0.0
    lat = [e for e in ENDS if m.latched[e] and m.latched_u[e] == tu]
    if len(lat) == 2:
        return 1.0 if (clear and still_steps >= HOLD_STEPS) else 0.5
    if len(lat) == 1:
        other = "front" if lat[0] == "rear" else "rear"
        return 0.5 if (pegged(ends[other]) and ends[other]["u"] == tu) else 0.25
    return 0.0


class Scorer:
    def __init__(self):
        self.still = 0
        self.level = 0.0
        self.last = {}

    def step(self, m: Measure, lin_v):
        clear = m.tip_mm >= CLEAR_M * 1e3
        both = all(m.latched[e] and m.latched_u[e] == m.target_u for e in ENDS)
        self.still = self.still + 1 if (both and clear and lin_v < STILL_V_M_S) else 0
        level = level_of(m, clear, self.still)
        self.level = level
        r = lambda d: {k: (round(v, 5) if isinstance(v, float) else v) for k, v in d.items()}  # noqa: E731
        self.last = dict(
            level=level,
            still_steps=self.still,
            measure=dict(
                target_u=m.target_u,
                rear=r(m.rear),
                front=r(m.front),
                latched=m.latched,
                latched_u=m.latched_u,
                tip_mm=round(m.tip_mm, 2),
                extension_mm=round(m.extension_mm, 3),
            ),
            lin_v_mm_s=round(lin_v * 1e3, 3),
        )
        return level


def angle_deg(r):
    return math.degrees(math.acos(float(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0))))
