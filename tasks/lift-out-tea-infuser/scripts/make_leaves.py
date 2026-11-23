"""tasks/lift-out-tea-infuser/assets/leaves_start.npz: the wet tea leaves' start layout (centres, radii in the station
frame) in the basket, a loose jittered lattice that settles in the first second.  python make_leaves.py <x> <y> <z of
the pot's foot> <out.npz>"""
import sys

import numpy as np

x0, y0, z0 = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
rng = np.random.default_rng(0)
RM, RMAX, FILL_R = 0.0025, 0.0035, 0.034
FLOOR = 0.0075  # the basket floor's top over the pot's foot
d = 2 * RMAX * 1.04
pts = []
for k, z in enumerate(np.arange(FLOOR + RMAX + 0.001, FLOOR + 0.05, d * 0.82)):
    off = (k % 2) * d / 4
    for x in np.arange(-FILL_R + off, FILL_R, d):
        for y in np.arange(-FILL_R + off, FILL_R, d):
            if np.hypot(x, y) + RMAX < FILL_R:
                pts.append((x, y, z))
    if len(pts) >= 200:
        break
pts = np.array(pts[:200]) + rng.uniform(-1, 1, (min(len(pts), 200), 3)) * (d / 2 - RMAX) * 0.5
pts = pts + np.array([x0, y0, z0])
rad = rng.uniform(RM, RMAX, len(pts))
np.savez(sys.argv[4], pos=pts, radius=rad)
m = 1000.0 * 4 / 3 * np.pi * rad ** 3
print(len(pts), "leaves", f"{m.sum() * 1000:.1f} g", f"z {(pts[:, 2].min() - z0) * 1000:.1f}-{(pts[:, 2].max() - z0) * 1000:.1f} mm")
