"""Revision reach check (no simulation): 16 oz bottle, a counter-top cabinet with a left-hinged front door.
Usage: python dev/reach2.py [cab_x_front cab_y_left bottle_x bottle_y place_x place_y]"""
import math
import sys

import numpy as np
from yam_arena import ik, station
from yam_arena.constants import READY

TOP = 0.7475


def frame(approach, close):
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - z * float(np.dot(close, z))
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


SEEDS = (READY, [0.0, 1.5, 1.2, -0.3, 0.0, 0.0], [0.5, 1.2, 1.0, -0.5, 0.0, 1.5], [-0.5, 1.2, 1.0, -0.5, 0.0, -1.5],
         [0.3, 0.6, 0.6, 0.0, 0.0, 0.0], [-0.3, 0.6, 0.6, 0.0, 0.0, 0.0], [0.6, 1.8, 1.6, -0.2, 0.0, 0.0],
         [-0.6, 1.8, 1.6, -0.2, 0.0, 0.0])


def err(arm, p, r):
    best = (1e9, None)
    for q0 in SEEDS:
        q, pe, re = ik.solve(np.asarray(p, float), ik.matrix_to_quat_xyzw(r), np.array(q0, float),
                             base_pos=station.BASES[arm], iters=150)
        e = pe * 1000 + math.degrees(re)
        if e < best[0]:
            best = (e, q)
    return best


def main():
    a = [float(v) for v in sys.argv[1:]] or [0.40, 0.505, 0.28, 0.10, 0.52, 0.30]
    xf, yl, bx, by, px, py = a
    rows = []
    noz_z, noz_x = 0.195 + 0.032, -0.0537 - 0.004
    for arm in ("right", "left"):
        for roll in (0, 90, -90):
            c = np.array([0.0, math.cos(math.radians(roll)), math.sin(math.radians(roll))])
            rows.append((arm, f"nozzle pinch roll {roll}", (bx + noz_x, by, TOP + noz_z), frame((1, 0, 0), c)))
    gz = 0.08
    for arm in ("left", "right"):
        rf = frame((1, 0, 0), (0, 1, 0))
        rows.append((arm, "body grip from front", (bx, by, TOP + gz), rf))
        rows.append((arm, "lifted 4 cm", (bx, by, TOP + gz + 0.04), rf))
        rows.append((arm, "at the cabinet mouth", (xf - 0.02, py, TOP + gz + 0.04), rf))
        rows.append((arm, "inside, over the floor", (px, py, TOP + gz + 0.04), rf))
        rows.append((arm, "set down on floor", (px, py, TOP + 0.016 + gz), rf))
        rows.append((arm, "backed out 12 cm", (px - 0.12, py, TOP + 0.016 + gz), rf))
    # door push: hinge at (xf, yl), contact 0.25 m out, hand 0.02 m off the outer face, at 15 cm height
    for th in (100, 80, 60, 40, 20, 5, 0):
        t = math.radians(th)
        edge = np.array([xf - 0.25 * math.sin(t), yl - 0.25 * math.cos(t)])
        n = np.array([-math.cos(t), math.sin(t)])
        p = edge + 0.03 * n
        for z in (0.12, 0.20):
            r = frame((-n[0], -n[1], 0.0), (0, 0, 1))
            rows.append(("left", f"door push th {th} z {z}", (p[0], p[1], TOP + z), r))
    for arm, label, p, r in rows:
        e, q = err(arm, p, r)
        flag = "OK " if e < 3 else "BAD"
        print(f"{flag} {arm:5s} {label:28s} ({p[0]:.3f},{p[1]:.3f},{p[2] - TOP:.3f}) err {e:7.1f}  q {np.round(q, 2)}")


if __name__ == "__main__":
    main()
