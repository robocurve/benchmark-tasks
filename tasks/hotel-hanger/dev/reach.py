"""Development check: which gripper directions reach a point. Usage: python dev/reach.py <arm> x y z"""
import math
import sys

import numpy as np
from yam_arena import ik, station
from yam_arena.constants import READY

arm = sys.argv[1]
p = np.array([float(v) for v in sys.argv[2:5]])


def frame(approach, close):
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - z * float(np.dot(close, z))
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


rows = []
for az in range(-180, 180, 45):
    for el in (-45, -20, 0, 20):
        a = np.array([math.cos(math.radians(el)) * math.cos(math.radians(az)),
                      math.cos(math.radians(el)) * math.sin(math.radians(az)), math.sin(math.radians(el))])
        for roll in (0, 90):
            c0 = np.cross(a, [0, 0, 1.0]) if abs(a[2]) < 0.99 else np.array([0, 1.0, 0])
            c = c0 if roll == 0 else np.cross(a, c0)
            best = 1e9
            for q0 in (READY, [0.0, 1.5, 1.2, -0.3, 0.0, 0.0]):
                q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(frame(a, c)), np.array(q0, float),
                                     base_pos=station.BASES[arm], iters=60)
                best = min(best, pe * 1000 + math.degrees(re))
            rows.append((best, az, el, roll))
rows.sort()
print(arm, p, "base", station.BASES[arm])
for r in rows[:14]:
    print(f"  err {r[0]:6.1f}  approach az {r[1]:5d} el {r[2]:4d} roll {r[3]}")
for want in ((0, 0), (90, 0), (-90, 0)):
    print("  az/el", want, [round(r[0], 1) for r in rows if (r[1], r[2]) == want])
