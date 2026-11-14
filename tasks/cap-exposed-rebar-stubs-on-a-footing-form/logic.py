"""Scoring logic for cap-exposed-rebar-stubs-on-a-footing-form. Pure Python and NumPy: task.py feeds it measured states
every control step, and the unit tests feed it made-up ones.

End-state scoring: the score is that of the current step, from how many of the five stubs wear a seated cap. Nothing is
latched: knocking a cap off a stub lowers the score again, because that stub is bare again.
"""

from __future__ import annotations

import math

import numpy as np

import geometry as g

CONTROL_HZ = 20


def rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt(quat_xyzw) -> float:
    """Angle (rad) between the body's z axis and the world z axis."""
    return math.acos(max(-1.0, min(1.0, float(rotation(quat_xyzw)[2, 2]))))


def seated_on(cap_pos, cap_quat, stub: int) -> bool:
    """Whether a cap (its origin is on the bore axis at the mouth) is seated on stub ``stub``: the bar is at least
    SEAT_INSIDE_MIN inside the bore, the bore axis within SEAT_OFFSET_MAX of the bar's, and the cap within SEAT_TILT_MAX
    of upright. A cap whose mouth is above the bar tip, or off to the side, is not on the bar."""
    x, y, z = (float(v) for v in cap_pos)
    off = math.hypot(x - g.FORM_X, y - g.STUB_Y[stub])
    inside = g.BAR_TOP - z                                # how far the bar's tip is up inside the bore
    return bool(off <= g.SEAT_OFFSET_MAX and inside >= g.SEAT_INSIDE_MIN and inside <= g.CAP_BORE_DEPTH + 0.002
                and tilt(cap_quat) <= g.SEAT_TILT_MAX)


def capped_stubs(caps: dict) -> list[bool]:
    """For each of the five stubs: does some cap sit seated on it. ``caps`` maps a cap's name to (pos, quat_xyzw)."""
    return [any(seated_on(p, q, i) for p, q in caps.values()) for i in range(len(g.STUB_Y))]


def arms_clear(tcps) -> bool:
    """Every tool point is at least CLEAR_DIST from every stub axis (horizontally), or higher than CLEAR_Z."""
    for tcp in tcps:
        x, y, z = (float(v) for v in tcp)
        if z >= g.CLEAR_Z:
            continue
        if min(math.hypot(x - g.FORM_X, y - sy) for sy in g.STUB_Y) < g.CLEAR_DIST:
            return False
    return True


class Scorer:
    """Feed :meth:`step` once per control step; read ``score`` and ``success`` from the result."""

    def __init__(self, hold_s: float = g.HOLD_S, hz: int = CONTROL_HZ):
        self.hold_steps = int(round(hold_s * hz))
        self.reset()

    def reset(self) -> None:
        self.hold = 0
        self.why: list[str] = []

    def step(self, *, caps: dict, tcps) -> dict:
        capped = capped_stubs(caps)
        n = sum(capped)
        clear = arms_clear(tcps)
        # the ladder reads the number of capped stubs: the two that wore a cap at the start give 0
        score = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.25, 4: 0.5, 5: 0.75}[n]
        holding = n == len(g.STUB_Y) and clear
        self.hold = self.hold + 1 if holding else 0
        success = self.hold >= self.hold_steps
        if success:
            score = 1.0
        return {"score": score, "success": success, "capped": capped, "n_capped": n, "arms_clear": clear,
                "hold_steps": self.hold}
