"""Revision reach check 2 (no simulation): cabinet front x 0.36, y -0.20..0.15, door hinged at its left edge.
Right arm: nozzle turn and neck carry. Left arm: pushes the door shut from its outer (+y) face."""
import math, sys
import numpy as np
sys.path.insert(0, "dev")
from reach2 import err, frame, TOP

def az(a):
    a = math.radians(a)
    return frame((math.cos(a), math.sin(a), 0), (math.sin(a), -math.cos(a), 0))

def main():
    xf, yr, yl = 0.36, -0.20, 0.15
    bx, by = float(sys.argv[1]) if len(sys.argv) > 1 else 0.22, float(sys.argv[2]) if len(sys.argv) > 2 else -0.12
    px, py = float(sys.argv[3]) if len(sys.argv) > 3 else 0.48, float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    NECK = 0.194
    rows = []
    for roll in (90, -90):
        c = np.array([0.0, math.cos(math.radians(roll)), math.sin(math.radians(roll))])
        rows.append(("right", f"nozzle pinch roll {roll}", (bx - 0.058, by, TOP + 0.227), frame((1, 0, 0), c)))
    for a in (90, 60, 45, 30):
        rows.append(("right", f"neck grip az {a}", (bx, by, TOP + NECK), az(a)))
        rows.append(("right", f"neck lifted 3cm az {a}", (bx, by, TOP + NECK + 0.03), az(a)))
        rows.append(("right", f"neck at mouth az {a}", (xf - 0.06, py, TOP + NECK + 0.03), az(a)))
        rows.append(("right", f"neck placed az {a}", (px, py, TOP + NECK + 0.018), az(a)))
        d = np.array([math.cos(math.radians(a)), math.sin(math.radians(a)), 0])
        q = np.array([px, py, TOP + NECK + 0.018]) - 0.08 * d
        rows.append(("right", f"backed 8cm az {a}", tuple(q), az(a)))
    for th in (100, 85, 70, 55, 40, 25, 10, 0):
        t = math.radians(th)
        for rad in (0.22, 0.30):
            edge = np.array([xf - rad * math.sin(t), yl - rad * math.cos(t)])
            n = np.array([-math.cos(t), math.sin(t)])
            p = edge + 0.03 * n
            for z in (0.20, 0.26):
                r = frame((-n[0], -n[1], 0.0), (0, 0, 1))
                rows.append(("left", f"door th {th} r {rad} z {z}", (p[0], p[1], TOP + z), r))
                r2 = frame((-n[0], -n[1], -0.6), (n[1], -n[0], 0))
                rows.append(("left", f"door th {th} r {rad} z {z} down", (p[0], p[1], TOP + z), r2))
    for arm, label, p, r in rows:
        e, qq = err(arm, p, r)
        print(f"{'OK ' if e < 3 else 'BAD'} {arm:5s} {label:30s} ({p[0]:.3f},{p[1]:.3f},{p[2]-TOP:.3f}) err {e:6.1f}")


if __name__ == "__main__":
    main()
