"""tiplaw: the cone friction fit of one 200 uL tip on one pipette nozzle, and the task's numbers. Pure Python and NumPy:
no simulator, so the law and the scoring can be tested offline (test_tips.py).

Lengths are in millimetres and forces in newtons unless a name ends in _M (metres).

The fit (one nozzle in one tip):
- ``s`` is how far the nozzle's end is below the tip's mouth, along the tip's axis. The cone first wedges in the mouth
  at ``s = S_CONTACT``; ``u = s - S_CONTACT + h`` is the depth past that point, where ``h`` is the tip's own seat
  height (moulding tolerance, +/- 0.3 mm).
- Loading is plastic: the tip slides up the cone, and the force needed rises with the depth reached, ``F = K_CONE u``,
  until the tip reaches full seat at ``U_FULL`` (the force there is ``F_SEAT``). The depth reached, ``d``, stays: the
  tip does not slide back down the cone.
- Below ``d`` the fit unloads elastically and stiffly (``K_ELASTIC``), so a tip pressed once carries no load until the
  nozzle comes back down to about where it was.
- Past full seat the tip is hard against the nozzle's stop; what gives is the rack plate under it, ``K_HARD``.

``K_HARD`` is fitted so that a straight push needs ``PEAK_STRAIGHT_N`` to bring all eight tips of a column to full seat,
given the seat heights below; see ``straight_push_total()``.
"""

from __future__ import annotations

import math

import numpy as np

# -- the simulation's force and size values, from vendor data (README, "Recalibrating from a measurement") --------------
FULL_SEAT_MM = 54.0  # tip lower end below the manifold underside, tip hard-seated
PEAK_STRAIGHT_N = 32.0  # all eight seated by a straight push
PEAK_ROCK_N = 12.0  # all eight seated by pressing and rocking +/- 6-8 deg
ARM_PUSH_N = 67.0  # N, the right arm's downward push at the seating pose: static Jacobian check in the simulator's arm model (README); offline tests only, not used in the simulation

# -- the fit ------------------------------------------------------------------------------------------------------------
F_SEAT = 3.5  # N, the load that brings one tip to full seat
U_FULL = 2.0  # mm, the travel up the cone from first wedging to full seat
K_CONE = F_SEAT / U_FULL  # 1.75 N/mm, the cone stiffness while the tip slides up
K_ELASTIC = 15.0  # N/mm, unloading and reloading below the depth already reached
HOLD_FRACTION = 0.4  # a tip pushed to at least 0.4 F_SEAT stays on the nozzle when it lifts
SHORT_MM = 1.5  # such a tip, short of full seat, locks this much short

# geometry of the PMC8-200's nozzles and of a universal 200 uL tip
NOZZLE_LEN = 12.0  # mm below the manifold underside
TIP_LEN = 51.0
S_FULL = NOZZLE_LEN + TIP_LEN - FULL_SEAT_MM  # 9.0 mm: the nozzle end below the tip's mouth at full seat
S_CONTACT = S_FULL - U_FULL  # 7.0 mm: where the cone first wedges in the mouth
S_SHORT = S_FULL - SHORT_MM  # 7.5 mm
CAPTURE_MM = 0.7  # a nozzle enters a tip's mouth if its end is this close to the tip's axis as it reaches the mouth
CAPTURE_DEG = 4.0  # and its axis this close to the tip's
RIM_MM = 4.6  # further out than CAPTURE_MM but within this (half a pitch), the nozzle lands on a tip's rim or collar and jams
K_RIM = 3.0  # N/mm, a nozzle standing on a tip's rim
K_LAT = 0.0  # N/mm: no sideways centring force (an explicit sideways spring on a friction-held pipette and a loose box is unstable at 240 Hz)
LAT_MAX_N = 5.0
SLIP_MM = 1000.0  # an entered nozzle stays in its tip (the tip goes with it) until it rises out of the mouth
DAMP = 0.005  # N per mm/s, along the axis, while loaded
LOCK_RISE_MM = 9.0  # the nozzle has risen this far above where the tip would touch it (to about the tip's mouth): the tip lifts out with it
RACK_LIMIT_N = 80.0  # total load on the rack insert above which it pops out of its ledges

# the eight rows' seat heights (mm, + stands higher): the rack's moulding, crowned 0.3 mm in the middle rows
SEAT_H = 0.3 * np.array([-1.0, -1 / 3, 1 / 3, 1.0, 1.0, 1 / 3, -1 / 3, -1.0])
K_HARD = (PEAK_STRAIGHT_N - 8 * F_SEAT) / float(np.sum(SEAT_H - SEAT_H.min()))  # 1.67 N/mm

PITCH = 9.0  # mm, nozzles and tip rows
ROW_Y = np.array([31.5 - PITCH * i for i in range(8)])  # rows A..H, mm from the row's middle


def fit_force(u: float, d: float) -> tuple[float, float]:
    """(load in N, new depth reached) for a tip at depth ``u`` past first wedging (mm) that has already been pushed to
    ``d``."""
    if u >= U_FULL:
        return F_SEAT + K_HARD * (u - U_FULL), U_FULL
    if u >= d:
        return K_CONE * max(u, 0.0), max(u, d)
    return max(0.0, K_CONE * d - K_ELASTIC * (d - u)), d


def fit_forces(u: np.ndarray, d: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """``fit_force`` for arrays."""
    u, d = np.asarray(u, float), np.asarray(d, float)
    f = np.where(u >= U_FULL, F_SEAT + K_HARD * (u - U_FULL),
                 np.where(u >= d, K_CONE * np.maximum(u, 0.0), np.maximum(0.0, K_CONE * d - K_ELASTIC * (d - u))))
    dn = np.where(u >= U_FULL, U_FULL, np.maximum(d, np.minimum(np.maximum(u, 0.0), U_FULL)))
    return f, dn


def row_depths(s_mid: float, roll_deg: float) -> np.ndarray:
    """``u`` of the eight tips when the nozzle row's middle is ``s_mid`` below the nominal mouths and the row is rolled
    ``roll_deg`` (+: the row A end lower)."""
    return s_mid - S_CONTACT + ROW_Y * math.tan(math.radians(roll_deg)) + SEAT_H


def straight_push_total() -> float:
    """The total force at which a level, straight push first has all eight tips at full seat."""
    s = S_CONTACT + U_FULL - SEAT_H.min()
    f, _ = fit_forces(row_depths(s, 0.0), np.zeros(8))
    return float(f.sum())


def best_rock_peak(max_roll: float = 8.0) -> tuple[float, list]:
    """The lowest peak total force of a quasi-static press-and-rock that seats the eight tips one at a time, over seat
    orders that begin with the ends, rolls of -8..8 deg: the sim's best case for PEAK_ROCK_N."""
    import itertools

    best = None
    for head in ((0, 7, 1, 6), (0, 1, 7, 6)):
        for tail in itertools.permutations((2, 3, 4, 5)):
            d, peak, log = np.zeros(8), 0.0, []
            for t in head + tail:
                cands = []
                for th in np.arange(-max_roll, max_roll + 0.01, 0.25):
                    s = S_CONTACT + U_FULL - ROW_Y[t] * math.tan(math.radians(th)) - SEAT_H[t]
                    cands.append((float(fit_forces(row_depths(s, th), d)[0].sum()), float(th), s))
                tot, th, s = min(cands)
                d = fit_forces(row_depths(s, th), d)[1]
                peak = max(peak, tot)
                log.append((t, th, round(tot, 2)))
            if best is None or peak < best[0]:
                best = (peak, log)
    return best
