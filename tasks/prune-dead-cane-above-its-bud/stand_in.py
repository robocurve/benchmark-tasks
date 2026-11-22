"""The real-world stand-in canes and the bench check that ties them to the simulated cut.

The simulated dead cane parts when the blade presses it with ``task.F_CUT`` (142.25 N, a published rose-stem shear strength
times the 4.5 mm section). Real trials use a non-perishable stand-in instead of a fresh stem: dry 4.5 mm round rattan core,
painted, with a bead for the bud. So that a real trial asks for the same squeeze as the simulation, each bundle of stand-in core
is checked once on the bench with the trial's own secateurs and an ordinary digital luggage scale (peak hold):

1. Hold the lower handle flat on the bench with one hand, jaws over the edge. Mark the upper handle ``PULL_R`` from the pivot.
2. Hook the scale's strap round the upper handle at the mark and pull straight down, square to the handle, until the jaws are
   4 mm apart at the middle of the cutting edge; note the reading (the spring alone).
3. Put a spare painted stand-in in the jaws against the hook, ``CUT_R`` from the pivot, and pull the same way until it parts;
   note the peak.
4. The peak minus the spring's reading must lie in ``check_band()``. Check three spares; all three must pass.

Run ``python stand_in.py`` to print the figures.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import scene_assets as sa  # noqa: E402

SHEAR_STRENGTH = 8.944e6   # Pa: rose upper stem, published fixture shear strength (see README, Sources)
CUT_R = 0.0246             # m from the pivot: where the cane sits against the hook in the demo's cut (task.CUT_RADIUS)
PULL_R = 0.120             # m from the pivot: where the scale pulls on the upper handle, inside the red grip
TOLERANCE = 0.25           # the stand-in's cutting torque must be within this fraction of the simulated one

STAND_IN_DEAD_D = 2 * sa.R0          # 4.5 mm round rattan core
STAND_IN_LIVE_D = 0.007              # 7 mm round rattan core
BUD_BEAD_D, BUD_BEAD_H = 0.004, 0.002  # 4 mm red flat-back half-round bead, 2 mm tall


def cut_force() -> float:
    """N the blade must press on the cane to part it (the same value as task.F_CUT)."""
    return SHEAR_STRENGTH * math.pi * sa.R0 ** 2


def cut_torque() -> float:
    """N m about the pivot that the cut needs, with the cane at CUT_R."""
    return cut_force() * CUT_R


def jaw_angle_on_cane(d: float = STAND_IN_DEAD_D, r: float = CUT_R) -> float:
    """rad still open between blade and hook when both touch a cane of diameter d at radius r (small-angle chord)."""
    return 2.0 * math.asin(d / (2.0 * r))


def spring_torque_at_cut() -> float:
    """N m from the opening spring when the jaws close onto the cane (preload plus travel from the open stop)."""
    travel = sa.JAW_OPEN - jaw_angle_on_cane()
    return sa.SPRING_K * (sa.SPRING_PRELOAD + travel)


def check_band(pull_r: float = PULL_R, tol: float = TOLERANCE) -> tuple[float, float]:
    """N: the range the luggage-scale peak, less the spring-only reading, must fall in."""
    t = cut_torque()
    return t * (1.0 - tol) / pull_r, t * (1.0 + tol) / pull_r


if __name__ == "__main__":
    lo, hi = check_band()
    print(f"cut force at the blade      {cut_force():7.2f} N")
    print(f"cut torque at {CUT_R * 1000:.1f} mm        {cut_torque():7.3f} N m")
    print(f"spring torque at the cut    {spring_torque_at_cut():7.3f} N m")
    print(f"pull at {PULL_R * 100:.0f} cm, cut only      {cut_torque() / PULL_R:7.1f} N")
    print(f"pull at {PULL_R * 100:.0f} cm, with spring   {(cut_torque() + spring_torque_at_cut()) / PULL_R:7.1f} N")
    print(f"check band (peak - spring)  {lo:5.1f} to {hi:4.1f} N")
