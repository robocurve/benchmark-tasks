"""Scoring for pry-board-off-joists: pure functions of the board's pose, the nails' state and the arms' let-go check.

The real trial is scored the same way with a ruler, a photo and the trial video:

- how far the board's underside stands off each joist's top at its nail (a ruler, at the nail);
- whether the board is off the joists (both nails out of the joists: it lifts away freely);
- whether it lies flat on the table at the end, clear of the frame, with both grippers let go.

No rule requires the bar: a gripper cannot loosen a nail without it (a fresh nail takes about 150 N, more than a
pinch grip on planed wood holds or the arm lifts, and the board cannot be levered on a joist while both nails hold).
Once the bar has freed one end, working the board up by that end to pull the second nail is a normal technique and
counts. The simulator still records how much of each nail's slide happened with the bar levering at it (``Nails``,
``bar_at_nail``), for the trace and the README; it does not change the score.

Tiers (the highest that holds at the end of the trial):
  1.0   the board is off the joists and lies flat on the table (its face within 20 deg of horizontal, its lowest
        point within 10 mm of the table and its highest within 104 mm: not propped on a joist or the bar), both
        grippers at least 3 cm from it and it is still;
  0.75  the board is off the joists (both nails out), but not laid flat on the table, or still held;
  0.5   both nails loosened: the board's underside at least 6 mm off the joist's top at each;
  0.25  one nail loosened that far;
  0     otherwise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

import layout as L

NAIL_NAMES = ("left", "right")


def board_to_world(rotation, origin, local) -> np.ndarray:
    return np.asarray(origin, float) + np.asarray(rotation, float) @ np.asarray(local, float)


def board_corners(rotation, origin) -> np.ndarray:
    hx, hy, hz = L.BOARD_W / 2, L.BOARD_L / 2, L.BOARD_T / 2
    return np.array([board_to_world(rotation, origin, (sx * hx, sy * hy, sz * hz))
                     for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])


def nail_gap(rotation, origin, name: str) -> float:
    """How far the board's underside stands off the joist's top at a nail (m): the ruler reading."""
    p = board_to_world(rotation, origin, L.board_local_nail(name))
    return float(p[2]) - L.JOIST_TOP


def face_normal(rotation) -> np.ndarray:
    return np.asarray(rotation, float) @ np.array([0.0, 0.0, 1.0])


def laid_flat(rotation, origin) -> bool:
    """Lying on a broad face on the table, on the table's top (not propped on a joist or the bar)."""
    if abs(float(face_normal(rotation)[2])) < math.cos(L.FLAT_TILT):
        return False
    c = board_corners(rotation, origin)
    low, high = float(c[:, 2].min()) - L.TOP, float(c[:, 2].max()) - L.TOP
    if not (-0.005 <= low <= L.ON_TABLE and high <= L.ON_TABLE_TOP):
        return False
    return (L.TABLE_X[0] <= float(c[:, 0].min()) and float(c[:, 0].max()) <= L.TABLE_X[1]
            and L.TABLE_Y[0] <= float(c[:, 1].min()) and float(c[:, 1].max()) <= L.TABLE_Y[1])


@dataclass
class Reading:
    score: float
    details: dict = field(default_factory=dict)


def bar_at_nail(tip, heel, name: str) -> bool:
    """The bar is levering at this nail: its blade (edge and heel, world points) is beside the nail's joist, the edge
    under the board's footprint (within its width, toward the nail) and no higher than the board's top, and the heel
    on the robot's side of it. What a person checks on the video: the blade under the board at that joist."""
    tip, heel = np.asarray(tip, float), np.asarray(heel, float)
    y = L.NAILS[name]
    near = abs(tip[1] - y) <= L.BAR_AT_NAIL_Y and abs(heel[1] - y) <= L.BAR_AT_NAIL_Y
    under = (L.BOARD_X - L.BOARD_W / 2 - 0.005 <= tip[0] <= L.BOARD_X + L.BOARD_W / 2
             and tip[2] <= L.JOIST_TOP + L.BAR_AT_NAIL_Z)
    return bool(near and under and heel[0] < tip[0])


def judge(rotation, origin, nails_out: dict, released: bool, still: bool, by_bar: dict | None = None) -> Reading:
    """The score of one moment. ``nails_out``: nail name -> True once it has left its joist. ``by_bar`` (nail name ->
    the bar did most of its loosening) is reported in the details only."""
    rotation = np.asarray(rotation, float)
    if not (np.all(np.isfinite(rotation)) and np.all(np.isfinite(origin))):
        return Reading(0.0, {"valid_state": False})
    gaps = {n: nail_gap(rotation, origin, n) for n in NAIL_NAMES}
    loose = [n for n in NAIL_NAMES if nails_out.get(n) or gaps[n] >= L.LOOSE]
    off = all(nails_out.get(n) for n in NAIL_NAMES)
    flat = laid_flat(rotation, origin)
    if off and flat and released and still:
        score = 1.0
    elif off:
        score = 0.75
    elif len(loose) == 2:
        score = 0.5
    elif len(loose) == 1:
        score = 0.25
    else:
        score = 0.0
    return Reading(score, {"valid_state": True, "gap_mm": {n: round(g * 1000, 1) for n, g in gaps.items()},
                           "nails_out": dict(nails_out), "by_bar": by_bar, "off": off, "laid_flat": flat,
                           "released": released, "still": still})


class Nails:
    """The two nails' withdrawal, as the simulator's rule keeps it (task.py): each nail an elastic-then-sliding grip
    on the board, its resistance falling with the length still in the joist. Pure Python, so it is tested offline.

    ``step(lift, tip)`` takes the carrier's joints (m, rad) and returns the joints' spring targets (m, rad)."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.slid = {n: 0.0 for n in NAIL_NAMES}  # how far each nail has slid out of its joist (m)
        self.out = {n: False for n in NAIL_NAMES}
        self.slid_bar = {n: 0.0 for n in NAIL_NAMES}  # how much of that slide happened with the bar levering there
        self.slid_other = {n: 0.0 for n in NAIL_NAMES}

    @staticmethod
    def displacement(lift: float, tip: float) -> dict:
        """How far the board has risen at each nail, from the carrier's joints: the left nail (+y) rises as the
        carrier turns +tip about x."""
        a = L.NAIL_DY * math.sin(tip)
        return {"left": lift + a, "right": lift - a}

    @staticmethod
    def resistance(slid: float) -> float:
        """The force (N) that slides a nail that has come ``slid`` out: proportional to the length still in."""
        return L.PULL_FULL * max(0.0, 1.0 - slid / L.EMBED)

    def step(self, lift: float, tip: float, bar_at: dict | None = None) -> tuple[float, float]:
        """``bar_at``: nail name -> True when the bar is levering at that nail this step (None: credit nothing)."""
        d = self.displacement(lift, tip)
        for n in NAIL_NAMES:
            if self.out[n]:
                self.slid[n] = d[n]  # gone: it follows the board and holds nothing
                continue
            before = self.slid[n]
            f = self.resistance(self.slid[n])
            stretch = d[n] - self.slid[n]
            if L.NAIL_K * stretch > f:
                self.slid[n] = d[n] - f / L.NAIL_K
            elif L.NAIL_K * stretch < -f:
                self.slid[n] = d[n] + f / L.NAIL_K
            self.slid[n] = max(self.slid[n], 0.0)  # the board cannot go into the joist
            moved = max(0.0, min(self.slid[n], L.OUT_AT) - before)
            if bar_at and bar_at.get(n):
                self.slid_bar[n] += moved
            else:
                self.slid_other[n] += moved
            if self.slid[n] >= L.OUT_AT:
                self.out[n] = True
                self.slid[n] = d[n]
        lift_t = (self.slid["left"] + self.slid["right"]) / 2.0
        tip_t = math.asin(max(-1.0, min(1.0, (self.slid["left"] - self.slid["right"]) / (2.0 * L.NAIL_DY))))
        lift_t = min(max(lift_t, L.LIFT_RANGE[0]), L.LIFT_RANGE[1])
        tip_t = min(max(tip_t, L.TIP_RANGE[0]), L.TIP_RANGE[1])
        return lift_t, tip_t

    @property
    def both_out(self) -> bool:
        return all(self.out.values())

    def by_bar(self) -> dict:
        """Nail name -> True if the bar did most of its loosening (and at least LOOSE of it)."""
        return {n: self.slid_bar[n] >= L.LOOSE and self.slid_bar[n] >= self.slid_other[n] for n in NAIL_NAMES}
