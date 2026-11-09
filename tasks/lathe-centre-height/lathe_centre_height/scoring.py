"""Scorer for setting a lathe tool to centre height. Pure Python (no Isaac, no NumPy).

Fed once per control step (20 Hz) with measured state only:
    err_m        the cutting edge's height above the spindle centre (m, signed), from the tool's measured pose
    still        the tool and both cranks are at rest
    grip_open    both grippers' openings, 0 closed .. 1 open
    tcp_clear_m  each tool point's distance from the nearest crank knob

A "rest state" is: tool and cranks still, both grippers >= 80 % open with their tool points >= 5 cm from both crank
knobs. A tier is awarded (and kept, best so far) only after a rest state has held for 1 s:
    credit = clip((e0 - 0.05 mm - |err|) / (e0 - 0.05 mm - full), 0, 1), where e0 is the measured start error
    (0.75 mm) and full is 0.25 mm.
So the start (and anything within 0.05 mm of it) scores 0, an edge at or within 0.25 mm of centre scores 1.0
(success: the trial ends), and anything farther from centre than the start scores 0 (turning the screw the wrong
way). A non-finite input latches 0.
"""

from __future__ import annotations

import math

DEADBAND_M = 0.00005  # credit starts only 0.05 mm closer to centre than the start
OPEN_MIN = 0.8
CLEAR_MIN_M = 0.05
HOLD_S = 1.0


class CentreHeightScore:
    def __init__(self, control_hz: float, e0_m: float, full_m: float):
        self.dt = 1.0 / float(control_hz)
        self.e0, self.full = abs(float(e0_m)), float(full_m)
        self.reset()

    def reset(self) -> None:
        self.best = 0.0
        self.failed = self.success = False
        self.window_s = 0.0
        self.reason = "start"

    def credit(self, err_m: float) -> float:
        e = abs(err_m)
        if e <= self.full:
            return 1.0
        start = self.e0 - DEADBAND_M  # trapping the rule alone moves the edge ~0.01 mm: that earns nothing
        return max(0.0, min(1.0, (start - e) / (start - self.full)))

    def info(self) -> dict:
        return {"best": self.best, "invalid": self.failed, "success": self.success, "reason": self.reason}

    def update(self, err_m, still, grip_open, tcp_clear_m) -> float:
        if self.failed:
            return 0.0
        vals = [err_m, *grip_open, *tcp_clear_m]
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals):
            self.failed, self.best, self.reason = True, 0.0, "non-finite state"
            return 0.0
        rest = bool(still) and min(grip_open) >= OPEN_MIN and min(tcp_clear_m) >= CLEAR_MIN_M
        if rest:
            self.window_s += self.dt
        else:
            self.window_s = 0.0
        tier = self.credit(err_m)
        if rest and self.window_s >= HOLD_S and tier > self.best:
            self.best = tier
            self.reason = f"rest at {err_m * 1000:+.3f} mm: credit {tier:.3f}"
            if tier >= 1.0:
                self.success = True
        elif not rest:
            self.reason = "moving or hands on"
        return self.best
