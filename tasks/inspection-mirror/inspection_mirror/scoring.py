"""Scorer for marking a hidden leak found with an inspection mirror. Pure Python (no Isaac, no NumPy).

Fed once per control step (20 Hz) with measured state only:
    marks_x   x of every paint mark on the cover strip so far (a mark is made only where the marker's tip touches it)
    smeared   something (the mirror, the marker or the robot) has touched the hidden face: the evidence is gone
    put_down  both tools down: each still, released (no part of the robot within 5 mm), not on the housing, and both
              grippers at least 80 % open
    paused    the mirror head's centre is in the gap and the mirror is held still (under 1 cm/s and 0.1 rad/s)
    seen      a wrist camera sees the trace's middle in the mirror's glass, unblocked

Look: a pause is an unbroken run of paused steps lasting LOOK_S. A mark counts only if the LAST pause before it
showed the trace (seen) for LOOK_S without a break. A mark with no pause before it, or whose last pause did not show
the trace (a pause elsewhere, a wrong-facing or blocked mirror), earns 0. A look after the mark does not help.
Per mark: credit 1.0 within FULL_MM of the trace (along the gap), falling linearly to 0 at ZERO_MM; 0 without a look.
The value is the lowest credit over all marks (a stray mark sends the repair to the wrong place), 0 with no mark.
Score = that value, taken whenever the put-down state has held HOLD_S. At any step: a smear sets the score to 0 for
the rest of the trial, and a new mark lowers the score at once to the new value if that is lower. 1.0 ends the trial.
"""

from __future__ import annotations

FULL_MM = 6.0
ZERO_MM = 14.0
HOLD_S = 1.0
LOOK_S = 0.5


def mark_credit(err_mm: float) -> float:
    e = abs(float(err_mm))
    if e <= FULL_MM:
        return 1.0
    if e >= ZERO_MM:
        return 0.0
    return (ZERO_MM - e) / (ZERO_MM - FULL_MM)


def value(marks_x, leak_x: float, smeared: bool, looked=None) -> float:
    """Lowest mark credit. ``looked`` gives each mark's look flag (a mark without a look earns 0); the live scorer
    always passes it. Left out, only the distances count (the geometry checks)."""
    if smeared or not marks_x:
        return 0.0
    if looked is None:
        looked = [True] * len(marks_x)
    return min(mark_credit((m - leak_x) * 1000.0) if ok else 0.0 for m, ok in zip(marks_x, looked))


class LeakMarkScore:
    def __init__(self, control_hz: float, leak_x: float):
        self.dt = 1.0 / float(control_hz)
        self.look_steps = max(1, int(round(LOOK_S * float(control_hz))))
        self.leak_x = float(leak_x)
        self.reset()

    def reset(self) -> None:
        self.score = 0.0
        self.success = False
        self.window_s = 0.0
        self.reason = "start"
        self.smeared = False
        self.pause_run = 0  # steps in the current unbroken pause
        self.seen_run = 0  # steps in the current pause with the trace seen, unbroken
        self.last_pause_saw: bool | None = None  # None: no pause yet; else: did the latest pause show the trace
        self.looked: list[bool] = []  # per mark, frozen when the mark is made

    def info(self) -> dict:
        return {"score_now": self.score, "success": self.success, "reason": self.reason,
                "marks_looked": list(self.looked), "last_pause_saw": self.last_pause_saw}

    def update(self, marks_x, smeared, put_down, paused=False, seen=False) -> float:
        marks_x = list(marks_x)
        new = len(marks_x) > len(self.looked)
        while len(self.looked) < len(marks_x):  # judged on the steps before this one
            self.looked.append(self.last_pause_saw is True)
        self.pause_run = self.pause_run + 1 if paused else 0
        self.seen_run = self.seen_run + 1 if paused and seen else 0
        if self.pause_run >= self.look_steps:
            self.last_pause_saw = self.seen_run >= self.look_steps
        self.smeared = self.smeared or bool(smeared)
        if self.smeared:
            self.score, self.success = 0.0, False
            self.reason = "the hidden face was touched: trace smeared -> 0"
            return self.score
        v = value(marks_x, self.leak_x, False, self.looked)
        if new:
            self.score = min(self.score, v)
        self.window_s = self.window_s + self.dt if put_down else 0.0
        if put_down and self.window_s >= HOLD_S:
            self.score = v
            errs = [round((m - self.leak_x) * 1000.0, 1) for m in marks_x]
            self.reason = f"rest: marks {errs} mm from the trace, looked {self.looked} -> {v:.3f}"
            if v >= 1.0:
                self.success = True
        elif new:
            self.reason = f"new mark: score now {self.score:.3f}"
        elif not put_down:
            self.reason = "tools not put down"
        return self.score
