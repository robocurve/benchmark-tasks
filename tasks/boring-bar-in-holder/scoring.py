"""The scorer, as plain functions of the bar's, the key's and the arms' state. NumPy only.

``Tracker.update(states, arms, thread)`` -> Result(score, success, details), fed one observation per control step.
``thread`` is the set-screw model's snapshot ({"tight", "engaged", "weld", "clamped_away", "turns", "record"}); the
rest is read from the states.

    STICK-OUT   s, how far the centre of the bar's cutting-end face is past the holder's chuck-side face, along the
                axis (y), mm
    TOWARD      the bar's cutting-end direction within 30 deg of +y (toward the chuck)
    IN BORE     the bar's axis within 1.5 mm of the bore's axis at both faces of the holder, the plain end behind the
                holder's back face and the cutting end past its front face
    FLAT UP     the flat's normal within 10 deg of vertical (up)
    INSERT      the insert's direction within 15 deg of the operator side (-x)
    TIGHT       both screws tight (setscrew.py)
    KEY OUT     the key is engaged in neither socket
    CLEAR       every finger zone of both grippers at least 30 mm from the bar (unreadable arms: not clear)
    STILL       over the last 20 steps the bar's two end points each moved 1 mm or less from where they were at the
                window's start

    1.0   on each of the last 20 steps: TIGHT, TOWARD, FLAT UP, INSERT, 45 <= s <= 48, KEY OUT, CLEAR; and STILL
    0.5   TIGHT and TOWARD (any of the other 1.0 conditions missing)
    0.25  IN BORE and TOWARD, the screws not both tight
    0     anything else. Latched 0 for the rest of the trial: the bar was clamped with its cutting end away from the
          chuck (the screw model's record), or the bar or the key went 5 cm or more below the bench top.

Fail closed: a step whose states or arms cannot be read counts against every tier and restarts the 20-step window.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

import layout as L
import rigstate
import setscrew

BAR, KEY, HOLDER = "bar", "key", "holder"


@dataclass
class Result:
    score: float = 0.0
    success: bool = False
    details: dict = field(default_factory=dict)


def axis_point_at_y(origin, axis, y_m: float):
    if abs(axis[1]) < 1e-9:
        return None
    return origin + axis * ((y_m - origin[1]) / axis[1])


def in_bore(origin, rot) -> bool:
    a = rot[:, 0]
    if a[1] < math.cos(math.radians(L.TOWARD_DEG)):
        return False
    tip = origin + a * L.BAR_L / 2 * L.MM
    rear = origin - a * L.BAR_L / 2 * L.MM
    if not (rear[1] <= L.Y_BACK * L.MM - 0.002 and tip[1] >= L.Y_FACE * L.MM + 0.002):
        return False
    for y_mm in (L.Y_BACK, L.Y_FACE):
        p = axis_point_at_y(origin, a, y_mm * L.MM)
        if p is None:
            return False
        off = math.hypot(p[0] - L.X_AX * L.MM, p[2] - (L.TOP + L.Z_AX * L.MM))
        if off > L.IN_BORE_MM * L.MM:
            return False
    return True


class Tracker:
    def __init__(self) -> None:
        self.last = Result()
        self.reset()

    def reset(self) -> None:
        self.steps = 0
        self.win: deque = deque(maxlen=L.WIN_STEPS)
        self.latched = ""
        self.last = Result()

    def _unreadable(self, why: str) -> Result:
        self.win.clear()
        self.last = Result(0.0, False, {"unreadable": why})
        return self.last

    def update(self, states, arms, thread) -> Result:
        self.steps += 1
        try:
            bar, key = states[BAR], states[KEY]
            origin = np.asarray(bar.origin, dtype=float)
            rot = np.asarray(bar.rotation(), dtype=float)
            low = min(float(np.asarray(bar.low, dtype=float)[2]), float(np.asarray(key.low, dtype=float)[2]))
            tight, engaged = [bool(v) for v in thread["tight"]], [bool(v) for v in thread["engaged"]]
            away = bool(thread["clamped_away"])
        except (AttributeError, KeyError, TypeError, ValueError):
            return self._unreadable("state")
        if not (np.all(np.isfinite(origin)) and np.all(np.isfinite(rot)) and math.isfinite(low)) \
                or not rigstate.readable(arms):
            return self._unreadable("numbers")
        if away and not self.latched:
            self.latched = "clamped with the cutting end away from the chuck"
        if low < L.TOP - L.OFF_TABLE_M and not self.latched:
            self.latched = "a part left the table"
        a = rot[:, 0]
        tip = origin + a * L.BAR_L / 2 * L.MM
        rear = origin - a * L.BAR_L / 2 * L.MM
        s_mm = L.stickout_mm(float(tip[1]))
        toward = bool(a[1] >= math.cos(math.radians(L.TOWARD_DEG)))
        flat_err = setscrew.flat_err_deg(rot)
        d = rot @ L.INSERT_D
        insert_err = math.degrees(math.acos(max(-1.0, min(1.0, float(d @ L.OPERATOR)))))
        both = all(tight)
        key_out = not any(engaged)
        read = rigstate.read_arms(arms)
        gap = min(rigstate.zone_gap(a_, bar.distance) for a_ in read)
        clear = gap >= L.CLEAR_M - 1e-9
        band = L.S_BAND[0] - 1e-9 <= s_mm <= L.S_BAND[1] + 1e-9
        ok1 = (both and toward and flat_err <= L.FLAT_UP_DEG and insert_err <= L.INSERT_DEG and band and key_out
               and clear)
        self.win.append((ok1, tip, rear))
        still = full = False
        if len(self.win) == L.WIN_STEPS:
            full = all(w[0] for w in self.win)
            t0, r0 = self.win[0][1], self.win[0][2]
            move = max(max(float(np.linalg.norm(w[1] - t0)), float(np.linalg.norm(w[2] - r0))) for w in self.win)
            still = move <= L.STILL_MM * L.MM + 1e-12
        bore = in_bore(origin, rot)
        if self.latched:
            score = 0.0
        elif full and still:
            score = 1.0
        elif both and toward:
            score = 0.5
        elif bore and toward:
            score = 0.25
        else:
            score = 0.0
        details = {"s_mm": round(s_mm, 2), "toward": toward, "in_bore": bore, "flat_err_deg": round(flat_err, 1),
                   "insert_err_deg": round(insert_err, 1), "tight": tight, "engaged": engaged,
                   "turns": thread.get("turns"), "weld": bool(thread.get("weld")), "gap_mm": round(gap * 1000, 1),
                   "clear": clear, "band": band, "still": still, "window_ok": full, "latched": self.latched}
        self.last = Result(score, score >= 1.0, details)
        return self.last
