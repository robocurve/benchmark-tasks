"""Scoring: where the mix ends up, by mass, as a real trial weighs it. NumPy only.

A real trial weighs the bag of mix before (bag + mix), the empty pot before, and after the trial the pot (its gain is
the mix transferred) and the bag (what is left in it); 0.80 kg minus both is the mix spilled. Here every grain carries
its mass and is placed by its centre:

- in the bag: inside the bag's film (its triangle mesh closed by a lid across the mouth);
- in the pot: otherwise, inside the pot's wall and below its rim (in the pot's frame);
- spilled: everything else (bench, floor, the pot's rim or outside, the robot).

Levels (fractions of the starting mass ``M``), judged at the end of the trial:

- 1.0: at least 95 % in the pot, at most 3 % spilled, the pot upright, and the bag let go of and clear of the pot;
- 0.5: at least 70 % in the pot and at most 8 % spilled, the pot upright;
- 0.25: at least 30 % in the pot and at most 15 % spilled, the pot upright;
- else 0.

"The pot upright": tilted less than 20 degrees. "The bag let go of": both grippers open at least 20 mm. "Clear of the
pot": no vertex of the bag's film, and no corner of a gripped tab, inside the pot's rim circle (plus 10 mm) lower than
20 mm above its rim.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

import bagplan as p
from soilkit import grain_volume

LEVELS = ((1.0, 0.95, 0.03), (0.5, 0.70, 0.08), (0.25, 0.30, 0.15))  # (score, in pot >=, spilled <=)
UPRIGHT = math.radians(20.0)
RELEASED_GAP = 0.020
CLEAR_R = 0.010
CLEAR_Z = 0.020


def grain_mass(radii) -> np.ndarray:
    return p.GRAIN_DENSITY * grain_volume(np.asarray(radii), "icosahedron")


def in_bag(pos, film) -> np.ndarray:
    """``film``: the film's vertex positions (V x 3), in the same frame as ``pos``."""
    import bagkit

    return bagkit.in_bag(pos, film)


def in_pot(pos, pot_pos, pot_rot) -> np.ndarray:
    local = (pos - pot_pos) @ pot_rot
    z = local[:, 2]
    r_wall = p.POT_R_BASE + (p.POT_R_TOP - p.POT_R_BASE) * np.clip(z / p.POT_H, 0, 1) - p.POT_WALL
    return (z > 0.0) & (z < p.POT_H) & (np.linalg.norm(local[:, :2], axis=1) < r_wall)


def bag_points(film, tabs) -> np.ndarray:
    """The film's vertices and every corner of each tab. ``tabs``: [(position, rotation, size)]."""
    pts = [np.asarray(film, float)]
    for tp, tr, size in tabs:
        h = np.asarray(size) / 2
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h
        pts.append(np.asarray(tp) + corners @ np.asarray(tr).T)
    return np.concatenate(pts)


def bag_clear(film, tabs, pot_pos, pot_rot) -> bool:
    local = (bag_points(film, tabs) - pot_pos) @ pot_rot
    near = np.linalg.norm(local[:, :2], axis=1) < p.POT_R_TOP + p.POT_RIM + CLEAR_R
    return not bool((near & (local[:, 2] < p.POT_H + CLEAR_Z)).any())


def tilt(rot) -> float:
    return float(math.acos(max(-1.0, min(1.0, rot[2, 2]))))


def grade(frac_pot: float, frac_spill: float, upright: bool, released: bool, clear: bool) -> float:
    if not upright:
        return 0.0
    for level, pot_min, spill_max in LEVELS:
        if level == 1.0 and not (released and clear):
            continue
        if frac_pot >= pot_min and frac_spill <= spill_max:
            return level
    return 0.0


@dataclass
class Tally:
    mass0: float | None = None
    spilled: np.ndarray | None = None  # which grains counted as spilled at the last update
    metrics: dict = field(default_factory=dict)

    def update(self, pos, radii, film, tabs, pot_pos, pot_rot, finger_gaps: dict) -> float:
        mass = grain_mass(radii)
        if self.mass0 is None:
            self.mass0 = float(mass.sum())
        bag = in_bag(pos, film)
        pot = in_pot(pos, pot_pos, pot_rot) & ~bag
        self.spilled = ~(bag | pot)
        if not np.isfinite(pos).all():
            bag = pot = np.zeros(len(pos), bool)
        frac_pot = float(mass[pot].sum()) / self.mass0
        frac_bag = float(mass[bag].sum()) / self.mass0
        frac_spill = 1.0 - frac_pot - frac_bag
        upright = tilt(pot_rot) < UPRIGHT
        released = all(g >= RELEASED_GAP for g in finger_gaps.values())
        clear = bag_clear(film, tabs, pot_pos, pot_rot)
        value = grade(frac_pot, frac_spill, upright, released, clear)
        self.metrics = {"score": value, "in_pot_kg": frac_pot * self.mass0, "in_bag_kg": frac_bag * self.mass0,
                        "spilled_kg": frac_spill * self.mass0, "frac_pot": frac_pot, "frac_spill": frac_spill,
                        "pot_tilt_deg": math.degrees(tilt(pot_rot)), "released": released, "bag_clear": clear}
        return value
