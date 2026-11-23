"""tip_rules: the task's scoring, as plain functions of what a person with a ruler and a video would measure. No simulator.

A tip is **seated** when it is on its nozzle and its lower end is within 1 mm of FULL_SEAT_MM below the manifold's
underside. The seated tips are **level** when their ends are within 1 mm of each other. The pipette is **held** when it is
in a gripper, within 10 deg of vertical, with every tip end on it at least 3 cm above the box rim. Tiers (task.json):
eight column-4 tips seated, level and held for 2 s: 1.0; seven: 0.5; six: 0.25; else 0. Any rule broken at any time
(``FAILS``) scores 0.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from tiplaw import FULL_SEAT_MM

SEAT_TOL_MM = 1.0
LEVEL_TOL_MM = 1.0
MAX_TILT_DEG = 10.0
CLEAR_ABOVE_RIM_M = 0.03
HOLD_S = 2.0
BOX_SHIFT_M = 0.005
CORNER_LIFT_M = 0.005
TIERS = {8: 1.0, 7: 0.5, 6: 0.25}

FAILS = {
    "wrong_tip": "a tip from another column (or another row's tip) is on a nozzle",
    "tip_touched": "a gripper touched a column-4 tip",
    "box_moved": "the box moved more than 5 mm from its marks",
    "corner_lifted": "a box corner lifted more than 5 mm",
    "rack_out": "the rack insert was pushed out of its ledges (over 80 N)",
    "dropped": "the pipette was dropped",
    "table": "the pipette or a tip on it touched the table",
}


def seated(on_nozzle: bool, end_below_mm: float) -> bool:
    """A tip on its nozzle whose lower end is within 1 mm of FULL_SEAT_MM below the manifold underside."""
    return bool(on_nozzle) and abs(end_below_mm - FULL_SEAT_MM) <= SEAT_TOL_MM


def level(ends_below_mm) -> bool:
    """The seated tips' ends within 1 mm of each other (true for none or one)."""
    ends = list(ends_below_mm)
    return len(ends) < 2 or max(ends) - min(ends) <= LEVEL_TOL_MM


def held_now(in_gripper: bool, tilt_deg: float, lowest_tip_end_z: float | None, rim_z: float) -> bool:
    """In a gripper, within 10 deg of vertical, and every tip end on the pipette at least 3 cm above the box rim."""
    clear = lowest_tip_end_z is None or lowest_tip_end_z >= rim_z + CLEAR_ABOVE_RIM_M
    return bool(in_gripper) and tilt_deg <= MAX_TILT_DEG and clear


@dataclass
class Snapshot:
    """One control step's measurements. ``tips``: for each column-4 tip (rows A-H), (on its own nozzle, its end below
    the manifold underside in mm, its end's height in m). ``other_on_nozzle``: a tip from anywhere else is on a nozzle."""

    tips: list
    in_gripper: bool
    tilt_deg: float
    rim_z: float
    fails: set = field(default_factory=set)

    def seated_ends(self) -> list:
        return [end for on, end, _ in self.tips if seated(on, end)]

    def lowest_end_z(self) -> float | None:
        zs = [z for on, _, z in self.tips if on]
        return min(zs) if zs else None

    def holding(self) -> bool:
        return held_now(self.in_gripper, self.tilt_deg, self.lowest_end_z(), self.rim_z)


def tier(n_seated: int, is_level: bool, held_s: float, fails) -> float:
    """The score of an end state."""
    if fails or not is_level or held_s < HOLD_S - 1e-9:
        return 0.0
    return TIERS.get(int(n_seated), 0.0)


class Tracker:
    """Follows a trial: latches every broken rule, and times how long the held state has lasted without a break."""

    def __init__(self, dt: float = 0.05):
        self.dt = dt
        self.reset()

    def reset(self) -> None:
        self.fails: set = set()
        self.held_s = 0.0
        self.last: Snapshot | None = None
        self.steps = 0

    def update(self, snap: Snapshot) -> None:
        self.steps += 1
        self.fails |= set(snap.fails)
        self.held_s = self.held_s + self.dt if snap.holding() else 0.0
        self.last = snap

    def counts(self) -> tuple[int, bool]:
        if self.last is None:
            return 0, True
        ends = self.last.seated_ends()
        return len(ends), level(ends)

    def score(self) -> float:
        n, lev = self.counts()
        return tier(n, lev, self.held_s, self.fails)

    def success_now(self) -> bool:
        """The full-score state at this step (the harness then requires it to hold for SUCCESS_HOLD_S)."""
        n, lev = self.counts()
        return not self.fails and n == 8 and lev and self.last is not None and self.last.holding()
