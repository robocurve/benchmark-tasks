"""This task's score, from plain positions, so it can be tested without the simulator: see test_alarm_score.py."""

from __future__ import annotations

import numpy as np
from yam_arena.task_file import TABLE_TOP

import alarm_usd as u

TOP = TABLE_TOP["station"]
THROUGH = 0.003  # a fingertip inside the window's outline this far through the glass breaks it, at any speed
PULLED = 0.028  # the handle counts as pulled at this far along its 0.03 m of travel: at the bottom. It latches there and the strobes flash
SCORE_GLASS, SCORE_PULLED = 0.3, 1.0


def slope_coords(points: np.ndarray) -> np.ndarray:
    """World points, one a row, as distance up the slope, across it, and out of it, from the slope's middle."""
    o = np.array(u.ORIGIN) + np.array([0.0, 0.0, TOP])
    d = points - o
    return np.column_stack([d @ u.UP_SLOPE, d[:, 1], d @ u.OUT])


class Scorer:
    """Keeps what stays true for the rest of a trial, and scores each control step."""

    def __init__(self, step_dt: float):
        self.step_dt = step_dt
        self.reset()

    def reset(self) -> None:
        self.steps = 0
        self.broken_at: float | None = None
        self.pulled_at: float | None = None
        self.deepest = 0.0

    def update(self, tips: np.ndarray, handle: float) -> dict:
        """One control step. `tips` holds both grippers' fingertips, one point a row. `handle` is how far the handle has slid towards its stop, in metres, zero at the start."""
        self.steps += 1
        t = round(self.steps * self.step_dt, 3)
        c = slope_coords(tips)
        w0, w1, ww = u.WINDOW
        depth = (u.RIM + u.PANE_THICK) - c[:, 2]  # how far through the plane of the glass
        inside = (c[:, 0] >= w0) & (c[:, 0] <= w1) & (np.abs(c[:, 1]) <= ww)
        if inside.any():
            self.deepest = max(self.deepest, float(depth[inside].max()))
        if self.broken_at is None and bool((inside & (depth >= THROUGH)).any()):
            self.broken_at = t
        if self.broken_at is not None and self.pulled_at is None and handle >= PULLED:
            self.pulled_at = t
        broken, pulled = self.broken_at is not None, self.pulled_at is not None
        score = SCORE_PULLED if pulled else SCORE_GLASS if broken else 0.0
        return {"step": self.steps, "t": t, "score": score, "broken": broken, "pulled": pulled, "broken_at": self.broken_at, "pulled_at": self.pulled_at, "handle_mm": round(handle * 1000.0, 2),
                "deepest_mm": round(self.deepest * 1000.0, 1), "success_now": pulled}
