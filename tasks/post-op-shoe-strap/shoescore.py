"""The scorer, as plain functions and a small state machine over per-step measurements (NumPy only).

One strap is scored (the post-op shoe's instep strap). Drawn back through its D-ring (at least 10 mm more strap
through the D-ring than at the start, still through it): 0.25. Also snug (the gap between the upper's edges under the
strap 12 mm or less): 0.5. Also fastened (at least 35 mm of hook patch engaged on the loop patch): 0.75. Also holding a 5 N tab pull along the
strap (the tab moves under 3 mm): 1.0. Measured with every part of the robot at least 3 cm from the shoe for 1 s.
Zero, held: the leg more than 5 cm off its mark, or the strap out of its D-ring, at any time; the robot not clear of the
shoe at the end. The functions take per-strap lists, so a shoe with more straps would average them.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

DRAWN_M = 0.010
SNUG_GAP_M = 0.012
OVERLAP_M = 0.035  # 1.4 strap widths; the first ~11 mm of hook past the D-ring lies on the fold round its bar
PULL_N = 5.0
PULL_MOVE_M = 0.003  # the tab may move this much under the pull and still hold
LEG_MARK_M = 0.05
CLEAR_M = 0.03
HOLD_S = 1.0
OUT_S = 0.25  # s a strap must stay out of its loop to count as out
TIERS = (0.25, 0.5, 0.75, 1.0)  # drawn, snug, pressed closed, holds the pull


def ring_crossing(loc: np.ndarray, link_len: float, half_w: float, half_h: float):
    """Where the strap passes the loop. ``loc``: (n, 3) plate centres in the loop's frame as (across the strap,
    through the loop, out of the flap). Returns the strap's arclength (m from the anchor) at the loop's plane, or None
    if no stretch of the strap crosses that plane inside the opening."""
    best = None
    for i in range(len(loc) - 1):
        a, b = loc[i, 1], loc[i + 1, 1]
        if a == b or (a > 0) == (b > 0):
            continue
        f = a / (a - b)
        p = loc[i] + f * (loc[i + 1] - loc[i])
        if abs(p[0]) <= half_w and abs(p[2]) <= half_h:
            s = (i + 0.5 + f) * link_len
            best = s if best is None else min(best, s)
    return best


def overlap_m(pairs, link_len: float) -> float:
    """Fastened length, as a ruler measures it: from the first to the last hook plate fastened to the loop patch."""
    hooks = {h for h, _ in pairs}
    return (max(hooks) - min(hooks) + 1) * link_len if hooks else 0.0


def strap_tier(drawn_m, gap_m, overlap, pull_ok) -> float:
    if drawn_m is None or drawn_m < DRAWN_M:
        return 0.0
    if gap_m > SNUG_GAP_M:
        return TIERS[0]
    if overlap < OVERLAP_M:
        return TIERS[1]
    return TIERS[3] if pull_ok else TIERS[2]


@dataclass
class Frame:
    t: float
    s_ring: tuple  # per strap: arclength at the loop (m) or None (out of the loop)
    gap: tuple  # per strap, m
    overlap: tuple  # per strap, m
    leg_off: float  # m from its mark
    robot_dist: float  # m, nearest robot point to the shoe


@dataclass
class Scorer:
    n_straps: int = 1
    s0: list = field(default_factory=list)
    zero: str = ""
    clear_since: float | None = None
    pull: list | None = None  # per strap: bool (holds), after the pull test
    want_pull: bool = False
    final: bool = False
    last: dict = field(default_factory=dict)
    score: float = 0.0
    out_since: dict = field(default_factory=dict)

    def update(self, f: Frame) -> float:
        if not self.s0:
            self.s0 = [s for s in f.s_ring]
        if f.leg_off > LEG_MARK_M and not self.zero:
            self.zero = f"leg {f.leg_off * 100:.1f} cm off its mark at t {f.t:.2f}"
        for k, s in enumerate(f.s_ring):  # out of its loop for OUT_S running (one step's reading can miss it)
            if s is None:
                self.out_since.setdefault(k, f.t)
                if f.t - self.out_since[k] >= OUT_S - 1e-9 and not self.zero:
                    self.zero = f"strap {k} out of its loop at t {self.out_since[k]:.2f}"
            else:
                self.out_since.pop(k, None)
        if f.robot_dist >= CLEAR_M:
            if self.clear_since is None:
                self.clear_since = f.t
        else:
            self.clear_since = None
            if not self.final:
                self.pull = None  # a pull test counts only for the hands-off state it measured
        clear = self.clear_since is not None and f.t - self.clear_since >= HOLD_S
        tiers = []
        for k in range(self.n_straps):
            drawn = None if (f.s_ring[k] is None or self.s0[k] is None) else self.s0[k] - f.s_ring[k]
            ok = bool(self.pull[k]) if self.pull is not None else False
            tiers.append(strap_tier(drawn, f.gap[k], f.overlap[k], ok))
        self.want_pull = clear and self.pull is None and not self.zero and any(
            o >= OVERLAP_M for o in f.overlap)
        self.last = {"tiers": tiers, "clear": clear, "zero": self.zero, "drawn": [
            None if (s is None or s0 is None) else round(1000 * (s0 - s), 1) for s, s0 in zip(f.s_ring, self.s0)],
            "gap_mm": [round(1000 * g, 1) for g in f.gap], "overlap_mm": [round(1000 * o, 1) for o in f.overlap],
            "leg_cm": round(100 * f.leg_off, 2), "robot_cm": round(100 * f.robot_dist, 1), "pull": self.pull}
        self.score = 0.0 if (self.zero or not clear) else float(sum(tiers)) / self.n_straps
        return self.score

    def pull_result(self, holds: list) -> None:
        self.pull = list(holds)

    @property
    def success(self) -> bool:
        return self.score >= 1.0 - 1e-9
