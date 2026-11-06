"""Scorer for indexing one of 7 equal divisions on a dividing head. Pure Python (no Isaac, no NumPy).

Fed once per control step (20 Hz) with measured state only:
    spindle_deg    workpiece rotation from its start (deg, unwrapped; either direction counts)
    circle         the hole circle the plunger pin is on (20 or 21 holes), or None when it is on neither radius
    seated         the pin is down in a hole (its bottom at least 2.5 mm below the plate face: only a hole allows it)
    crank_dps      crank speed
    grip_open      both grippers' openings, 0 closed .. 1 open
    tcp_clear_m    each tool point's distance from the knob centre

A "rest state" is: pin seated, both grippers >= 80 % open with their tool points >= 5 cm from the knob, crank under
1 deg/s. Each tier is awarded (and latched, best so far) only after a rest state has held for 1 s:
    1.0  pin in the 21-hole circle and |spindle| within TOL of 360/7 = 51.43 deg (5 turns + 15 holes). The trial ends.
    0.5  pin in the 21-hole circle and 5 whole crank turns (|spindle| in [44.8, 53.8) deg), wrong hole.
    0.25 5 whole crank turns on the 20-hole circle (the turns right, the circle wrong).
    0    otherwise: every state with fewer (or more) than 5 whole turns, including the workpiece unmoved.
Zero, latched: the crank turned more than FORCED_DEG while the pin stayed seated (the pin forced along the plate),
or a non-finite input.
"""

from __future__ import annotations

import math

OPEN_MIN = 0.8
CLEAR_MIN_M = 0.05
STILL_DPS = 1.0
HOLD_S = 1.0


class DivisionScore:
    def __init__(self, control_hz: float, target_deg: float, tol_deg: float, turns_band_deg: tuple[float, float],
                 right_circle: int, forced_deg: float):
        self.dt = 1.0 / float(control_hz)
        self.target, self.tol, self.band = target_deg, tol_deg, turns_band_deg
        self.right_circle, self.forced_deg = right_circle, forced_deg
        self.reset()

    def reset(self) -> None:
        self.best = 0.0
        self.failed = self.success = False
        self.window_s = 0.0
        self.window_tier = None
        self.seat_crank = None  # crank angle when the pin last seated
        self.reason = "start"

    def tier(self, spindle_deg: float, circle) -> float:
        a = abs(spindle_deg)
        right = circle == self.right_circle
        turns = self.band[0] <= a < self.band[1]
        if right and abs(a - self.target) <= self.tol:
            return 1.0
        if right and turns:
            return 0.5
        if turns and circle is not None:
            return 0.25
        return 0.0  # fewer or more than 5 whole turns: nothing, whatever circle the pin is in

    def info(self) -> dict:
        return {"best": self.best, "forced_or_invalid": self.failed, "success": self.success,
                "hold_window_s": round(self.window_s, 3), "score_reason": self.reason}

    def update(self, spindle_deg, crank_deg, circle, seated, crank_dps, grip_open, tcp_clear_m) -> float:
        if self.failed:
            return 0.0
        nums = [spindle_deg, crank_deg, crank_dps, *grip_open, *tcp_clear_m]
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in nums):
            self.failed, self.reason, self.window_s = True, "non-finite state", 0.0
            return 0.0
        if self.success:
            return 1.0
        if seated:
            if self.seat_crank is None:
                self.seat_crank = crank_deg
            elif abs(crank_deg - self.seat_crank) > self.forced_deg:
                self.failed, self.reason, self.window_s = True, "crank turned with the pin seated (forced)", 0.0
                return 0.0
        else:
            self.seat_crank = None
        released = all(g >= OPEN_MIN for g in grip_open) and all(c >= CLEAR_MIN_M for c in tcp_clear_m)
        rest = seated and released and abs(crank_dps) < STILL_DPS
        tier = self.tier(spindle_deg, circle) if rest else None
        if rest and tier == self.window_tier:
            self.window_s += self.dt
        else:
            self.window_s = self.dt if rest else 0.0
        self.window_tier = tier
        if rest and self.window_s >= HOLD_S - 1e-9:
            self.best = max(self.best, tier)
            if tier >= 1.0:
                self.success, self.reason = True, "one division of 7: 21-hole circle, right hole, hands off, still"
                return 1.0
        self.reason = (f"best {self.best:.2f}; " + ("at rest, tier %.2f holding" % tier if rest else "not at rest"))
        return self.best
