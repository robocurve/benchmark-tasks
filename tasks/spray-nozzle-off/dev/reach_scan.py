"""Development check: the right hand's neck grip, by approach azimuth, at the pick, lift, compartment and set-down."""
import math, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import layout as L
from reach import err, frame
hold = L.neck_tcp()
for az in (0, 15, 30, 45, 60, 75, 90):
    a = np.array([math.cos(math.radians(az)), math.sin(math.radians(az)), 0.0])
    c = np.array([-a[1], a[0], 0.0])
    out = []
    for p in (hold, hold + [0, 0, L.LIFT], [L.SLOT_XY[0], L.SLOT_XY[1], hold[2] + L.LIFT],
              [L.SLOT_XY[0], L.SLOT_XY[1], hold[2] + L.CADDY_FLOOR + 0.004]):
        out.append(round(min(err("right", np.asarray(p, float) - 0.004 * a, frame(a, sg * c))[0] for sg in (1, -1)), 1))
    print(f"az {az:3d}: pick {out[0]}, lifted {out[1]}, over slot {out[2]}, set down {out[3]}")
e, q = err("left", L.nozzle_tcp(), frame((1, 0, 0), (0, -1, 0)))
print("left nozzle roll 0 via close -y:", round(e, 1), np.round(q, 2))
