"""Rest mesh of the loaded drawstring liner, folded over the bin rim.

Profile (metres, bag frame: origin at the liner bottom centre): flat bottom of
radius BODY_R, a side wall up to the rim, a short arc over the rim, then the
liner surplus (cuff) hanging down outside the bin to the hem, which holds the
drawstring. Total profile length matches a 400 mm tall liner; hem
circumference 2*pi*HEM_R = 0.584 m matches a 29 cm wide liner. Positions are
authored only at reset.
"""
import numpy as np

SUBDIVISION = 0
N_AROUND = 80
BODY_R = .076          # inside the 159 mm pot
HEM_R = .093           # outside the bin wall
RIM_Z = .144           # pot rim (z 0.902) above the liner bottom (z 0.758)
HEM_Z = .015           # hem 23 mm above the mat
PLEATS = 10            # loose folds of the 4 cm surplus hem length around the rim
PLEAT_AMP = .005       # m, at the hem


def profile():
    rings = [(BODY_R * t / 4, 0.) for t in range(1, 5)]
    side = 14
    rings += [(BODY_R, (RIM_Z - .002) * t / side) for t in range(1, side + 1)]
    c = np.array([(BODY_R + HEM_R) / 2, RIM_Z + .001])
    a = (HEM_R - BODY_R) / 2
    for ang in np.linspace(np.pi, 0, 7)[1:-1]:
        rings.append((float(c[0] + a * np.cos(ang)), float(c[1] + a * np.sin(ang))))
    cuff = 14
    rings += [(HEM_R, float(RIM_Z - .002 - (RIM_Z - .002 - HEM_Z) * t / cuff)) for t in range(0, cuff + 1)]
    return rings


FOLD_RING = 4 + 14 + 2   # index (in profile()) of the top of the arc over the rim (5 arc rings)


def dirty_mesh():
    n = N_AROUND
    rings = profile()
    v = [[0., 0., 0.]]
    first_cuff = 4 + 14 + 5
    for k, (r, z) in enumerate(rings):
        amp = PLEAT_AMP * max(0., (k - first_cuff + 1) / (len(rings) - first_cuff))
        th = np.arange(n) * 2 * np.pi / n
        rr = r + amp * np.cos(PLEATS * th)
        v.extend([[a * np.cos(b), a * np.sin(b), z] for a, b in zip(rr, th)])
    f = [[0, 1 + (j + 1) % n, 1 + j] for j in range(n)]
    for k in range(len(rings) - 1):
        a = 1 + k * n
        b = a + n
        for j in range(n):
            q = (j + 1) % n
            f.extend([[a + j, a + q, b + j], [a + q, b + q, b + j]])
    return np.array(v), np.array(f), np.arange(1 + (len(rings) - 1) * n, 1 + len(rings) * n)


def fold_ring_ids():
    """Vertex ids of the ring at the top of the fold (the rim while folded)."""
    return np.arange(1 + FOLD_RING * N_AROUND, 1 + (FOLD_RING + 1) * N_AROUND)


def profile_length():
    p = np.array([(0., 0.)] + profile())
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
