"""When a wire nut bites on the bare black splice, and when it covers it. Pure NumPy.

A real wire nut has a tapered square-wire spring inside: pushed over the twisted conductors and turned clockwise
(seen from its closed end), the spring threads onto the copper and draws it in and twists the conductors together;
turned until snug (several turns), it holds, and a light tug does not pull it off. The simulation does not model the spring. It stands in a rule for it:

- the nut is ENGAGED while the copper's end is at least 6 mm into its bore, within 2.5 mm of the bore's axis, with the
  nut's axis within 25 deg of the copper's;
- while engaged, the nut's turning about its own axis relative to the copper is added up (clockwise seen from the
  closed end counts forward, the other way back); pulling the copper out of the bore starts the count again;
- once the count reaches two full turns the nut BITES: it is glued to the copper where it is (task.py arms a bond that
  breaks above 30 N, a hard pull).

The splice is CAPPED while a nut that bit is still on it (its bond intact) with the copper's end at least 10.5 mm into the
bore: all 9.5 mm (3/8 in) of bare copper inside the nut. A nut that bit nearer the mouth leaves copper showing, as a real one
started crooked or too shallow does.

Nut frame: its mouth's centre at the origin, its axis +x into the bore (toward the closed end).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

import layout as L


@dataclass
class Engagement:
    depth: float  # m: how far the copper's end is into the bore, along the nut's axis
    radial: float  # m: how far it is from the axis
    angle: float  # rad: between the nut's axis and the copper's direction (root to end)

    @property
    def engaged(self) -> bool:
        return self.depth >= L.ENGAGE_DEPTH and self.radial <= L.RADIAL_MAX and self.angle <= L.ALIGN_MAX

    @property
    def inside(self) -> bool:
        """The copper's end is in the bore at all (deeper than 2 mm, within the bore's radius)."""
        return self.depth >= 2.0 * L.MM and self.radial <= L.NUT_MOUTH_IN


def engagement(nut_R, nut_o, tip, tip_dir) -> Engagement:
    """``nut_R``/``nut_o``: the nut frame's rotation and origin (world). ``tip``: the copper's end point; ``tip_dir``:
    the unit direction the copper points (from the splice's root toward its end)."""
    R = np.asarray(nut_R, float)
    local = R.T @ (np.asarray(tip, float) - np.asarray(nut_o, float))
    axis = R[:, 0]
    d = np.asarray(tip_dir, float)
    d = d / np.linalg.norm(d)
    angle = math.acos(max(-1.0, min(1.0, float(axis @ d))))
    return Engagement(float(local[0]), float(math.hypot(local[1], local[2])), angle)


def turn_angle(nut_R, tip_R) -> float:
    """The nut's turn about its own axis, seen in the copper's (the tip link's) frame: an angle that grows when the
    nut turns right-handedly about its +x axis (mouth to closed end). Only its changes matter."""
    rn, rt = np.asarray(nut_R, float), np.asarray(tip_R, float)
    a = rt.T @ rn[:, 0]  # the nut's axis, in the tip frame
    e = rt.T @ rn[:, 1]  # a mark on the nut's side, in the tip frame
    helper = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([0.0, 1.0, 0.0])
    b1 = np.cross(helper, a)
    b1 /= np.linalg.norm(b1)
    b2 = np.cross(a, b1)
    return math.atan2(float(e @ b2), float(e @ b1))


def _wrap(x: float) -> float:
    return (x + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class NutTrack:
    twist: float = 0.0  # rad turned on (clockwise seen from the closed end) while engaged
    last: float | None = None  # the last turn_angle while engaged
    bit: bool = False
    bit_step: int | None = None


@dataclass
class Capper:
    """Follows every loose nut against the black splice's copper, step by step."""

    nuts: dict = field(default_factory=dict)  # name -> NutTrack

    def reset(self) -> None:
        self.nuts.clear()

    def step(self, name: str, nut_R, nut_o, tip, tip_dir, tip_R, step: int = 0) -> bool:
        """Advance one nut by one control step. True on the step it bites."""
        t = self.nuts.setdefault(name, NutTrack())
        if t.bit:
            return False
        e = engagement(nut_R, nut_o, tip, tip_dir)
        if not e.inside:
            t.twist, t.last = 0.0, None  # off the copper: a fresh start next time
            return False
        if not e.engaged:
            t.last = None  # in the mouth but crooked or shallow: the spring does not catch; keep the count
            return False
        now = turn_angle(nut_R, tip_R)
        if t.last is not None:
            # Clockwise seen from the closed end (looking along -x of the nut) is a right-handed turn about -x: the
            # angle about +x decreases.
            t.twist += -_wrap(now - t.last)
        t.last = now
        if t.twist >= L.TWIST_NEED:
            t.bit, t.bit_step = True, step
            return True
        return False


def covers(depth: float) -> bool:
    """A nut whose bore holds the copper's end this deep covers all the bare copper."""
    return depth >= L.CAPPED_DEPTH
