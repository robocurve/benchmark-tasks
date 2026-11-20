"""Bending stiffness of the real insole from a 5-minute cantilever test (README "Measure the insole's stiffness").

Lay the insole sole down on a table edge, toe end under a heavy book, heel end overhanging by a (mm), and measure with a
steel ruler how far the heel tip drops below the table top, d (mm). Optionally tape a coin of mass p (g) at the
heel tip and measure again. Weigh the insole (g) and measure its length (mm).

    python fit_stiffness.py --mass 40 --length 267 80:2.1 120:9.5 160:27 [--tip-g 5.0 80:3.0 ...]

prints the foam modulus E (Pa) that reproduces the drops with the task's outline (geom.width) and 7 mm thickness,
and the drops the task's current E predicts. Large-deflection beam (exact angle, no small-slope approximation).
Pure numpy; run from this folder.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "isaac"))
import geom  # noqa: E402


def drop(E, a, mass_kg, length, tip_kg=0.0, n=400):
    """Tip drop (m) of the heel end overhanging by a (m): width w(s) from the heel, distributed weight in proportion
    to the width, clamped flat at the table edge."""
    s = np.linspace(0.0, a, n + 1)                       # arc length from the heel tip (free end) to the edge
    w = geom.width(s / length)
    q = mass_kg * 9.81 * w / geom.area()                 # N/m
    EI = E * w * geom.T ** 3 / 12
    ds = a / n
    # shear/moment from the free end; then integrate the angle from the clamp outward (shooting not needed: the
    # moment about each section uses the deformed horizontal lever, so iterate a few times)
    x = s.copy()
    for _ in range(30):
        M = np.zeros(n + 1)
        for i in range(n + 1):
            lever = x[i] - x[:i + 1]                     # horizontal distance to the load points beyond section i
            M[i] = np.sum(q[:i + 1] * ds * lever) + tip_kg * 9.81 * (x[i] - x[0])
        kappa = M / EI
        th = np.zeros(n + 1)                             # angle below horizontal, 0 at the clamp (s = a)
        for i in range(n - 1, -1, -1):
            th[i] = th[i + 1] + 0.5 * (kappa[i] + kappa[i + 1]) * ds
        xn = np.zeros(n + 1)
        z = np.zeros(n + 1)
        for i in range(n - 1, -1, -1):
            xn[i] = xn[i + 1] + math.cos(0.5 * (th[i] + th[i + 1])) * ds
            z[i] = z[i + 1] + math.sin(0.5 * (th[i] + th[i + 1])) * ds
        x = xn[0] - xn                                    # measured from the free end, increasing toward the clamp
    return float(z[0])


def fit(pairs, mass_kg, length, tip_kg=0.0):
    def err(logE):
        return sum((drop(math.exp(logE), a, mass_kg, length, tip_kg) - d) ** 2 for a, d in pairs)
    lo, hi = math.log(1e5), math.log(1e9)
    for _ in range(80):                                  # golden-section on log E
        m1, m2 = lo + 0.382 * (hi - lo), lo + 0.618 * (hi - lo)
        lo, hi = (lo, m2) if err(m1) < err(m2) else (m1, hi)
    return math.exp((lo + hi) / 2)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--mass", type=float, default=geom.MASS * 1000, help="insole mass, g")
    ap.add_argument("--length", type=float, default=geom.L * 1000, help="insole length, mm")
    ap.add_argument("--tip-g", type=float, default=0.0, help="coin taped at the heel tip, g")
    ap.add_argument("pairs", nargs="*", help="overhang_mm:drop_mm")
    a = ap.parse_args()
    m, Lm = a.mass / 1000, a.length / 1000
    print("task E = %.2g Pa, EI (mean width) = %.2e N m^2" % (geom.E, geom.ei_bend()))
    for ov in (80, 120, 160):
        print("  predicted drop at %d mm overhang: %.1f mm (no coin), %.1f mm (5.0 g coin)" % (
            ov, 1000 * drop(geom.E, ov / 1000, m, Lm), 1000 * drop(geom.E, ov / 1000, m, Lm, 0.005)))
    if a.pairs:
        pairs = [(float(p.split(":")[0]) / 1000, float(p.split(":")[1]) / 1000) for p in a.pairs]
        E = fit(pairs, m, Lm, a.tip_g / 1000)
        print("fitted E = %.3g Pa  (set geom.E to this value)" % E)
        for ov, d in pairs:
            print("  overhang %.0f mm: measured %.1f mm, fitted %.1f mm" % (ov * 1000, d * 1000,
                                                                         1000 * drop(E, ov, m, Lm, a.tip_g / 1000)))
