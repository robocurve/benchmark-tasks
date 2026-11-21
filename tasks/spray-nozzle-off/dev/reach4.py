"""Reach check 3: right-arm nozzle pinch at the new start, caddy placement for the 'caddy' variant, door at 75 deg."""
import math, sys
import numpy as np
sys.path.insert(0, "dev")
from reach2 import err, frame, TOP
from reach3 import az
bx, by = 0.26, -0.10
rows = []
for x in (0.24, 0.26, 0.28):
    for roll in (90, -90, 60, -60):
        c = np.array([0.0, math.cos(math.radians(roll)), math.sin(math.radians(roll))])
        for arm in ("right", "left"):
            rows.append((arm, f"pinch bx {x} roll {roll}", (x - 0.058, by, TOP + 0.227), frame((1, 0, 0), c)))
for a in (45, 60, 90):
    rows.append(("right", f"caddy over az {a}", (0.3625, -0.43, TOP + 0.194 + 0.17), az(a)))
    rows.append(("right", f"caddy down az {a}", (0.3625, -0.43, TOP + 0.004 + 0.194 + 0.004), az(a)))
for th in (80, 75):
    t = math.radians(th)
    for rad in (0.25, 0.30):
        edge = np.array([0.36 - rad * math.sin(t), 0.15 - rad * math.cos(t)])
        n = np.array([-math.cos(t), math.sin(t)])
        p = edge + 0.03 * n
        for tilt in (0.0, -0.3, -0.6):
            r = frame((-n[0], -n[1], tilt), (n[1], -n[0], 0) if tilt else (0, 0, 1))
            rows.append(("left", f"door th {th} r {rad} tilt {tilt}", (p[0], p[1], TOP + 0.22), r))
for arm, label, p, r in rows:
    e, qq = err(arm, p, r)
    print(f"{'OK ' if e < 3 else 'BAD'} {arm:5s} {label:30s} ({p[0]:.3f},{p[1]:.3f},{p[2]-TOP:.3f}) err {e:6.1f}")
