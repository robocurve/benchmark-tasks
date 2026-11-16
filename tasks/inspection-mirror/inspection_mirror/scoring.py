"""Scorer for marking a hidden leak found with an inspection mirror. Pure Python (no Isaac, no NumPy).

Fed once per control step (20 Hz) with measured state only:
    marks_x      x of every paint mark on the cover strip so far (paint stays where the marker's tip touched)
    leak_x       x of the leak trace's centre on the hidden face
    smeared      the mirror head has touched the hidden face (it wipes the trace: the evidence is gone)
    still        the mirror and the marker at rest
    grip_open    both grippers' openings, 0 closed .. 1 open

Per mark: credit 1.0 within FULL_MM of the trace (along the gap), falling linearly to 0 at ZERO_MM. The value is the
lowest credit over all marks (a stray mark sends the repair to the wrong place), 0 with no mark, and 0 once the trace
is smeared. Score = that value, taken whenever a rest state (mirror and marker still, both grippers >= 80 % open) has
held 1 s; it follows the latest rest, so marks made after a full score still count. 1.0 ends the trial.
The leak sits at one of five places 20 mm apart and ZERO_MM < 20 mm, so one blind mark averages 0.2.
"""

from __future__ import annotations

FULL_MM = 6.0
ZERO_MM = 14.0
OPEN_MIN = 0.8
HOLD_S = 1.0


def mark_credit(err_mm: float) -> float:
    e = abs(float(err_mm))
    if e <= FULL_MM:
        return 1.0
    if e >= ZERO_MM:
        return 0.0
    return (ZERO_MM - e) / (ZERO_MM - FULL_MM)


def value(marks_x, leak_x: float, smeared: bool) -> float:
    if smeared or not marks_x:
        return 0.0
    return min(mark_credit((m - leak_x) * 1000.0) for m in marks_x)


class LeakMarkScore:
    def __init__(self, control_hz: float, leak_x: float):
        self.dt = 1.0 / float(control_hz)
        self.leak_x = float(leak_x)
        self.reset()

    def reset(self) -> None:
        self.score = 0.0
        self.success = False
        self.window_s = 0.0
        self.reason = "start"

    def info(self) -> dict:
        return {"score_now": self.score, "success": self.success, "reason": self.reason}

    def update(self, marks_x, smeared, still, grip_open) -> float:
        rest = bool(still) and min(grip_open) >= OPEN_MIN
        self.window_s = self.window_s + self.dt if rest else 0.0
        if rest and self.window_s >= HOLD_S:
            v = value(marks_x, self.leak_x, smeared)
            self.score = v
            errs = [round((m - self.leak_x) * 1000.0, 1) for m in marks_x]
            self.reason = f"rest: marks {errs} mm from the trace, smeared={bool(smeared)} -> {v:.3f}"
            if v >= 1.0:
                self.success = True
        elif not rest:
            self.reason = "moving or a gripper closed"
        return self.score
