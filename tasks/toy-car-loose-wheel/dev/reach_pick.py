"""Development check: body grip across the car's width (closing along y), the hand leaning forward by a pitch, at the
pick, lift and drop points of both arms."""
import math, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import layout as L
from reach import err, frame

pts = [(f"pick y{y:+.2f}", (L.CAR_X, y, L.PICK_TCP_Z)) for y in L.SLOTS_Y]
pts += [(f"lift y{y:+.2f}", (L.CAR_X, y, L.LIFT_Z)) for y in L.SLOTS_Y]
for (x, y) in L.BASKET_SPOTS + (L.BOX_SPOT,):
    pts += [(f"over ({x:.2f},{y:+.2f})", (x, y, L.LIFT_Z)), (f"drop ({x:.2f},{y:+.2f})", (x, y, L.FLOOR + L.DROP_Z))]
for arm in ("left", "right"):
    for label, p in pts:
        row = []
        for pitch in (0, 20, 35, 50):
            t = math.radians(pitch)
            a = np.array([math.sin(t), 0.0, -math.cos(t)])
            e = min(err(arm, np.array([p[0], p[1], L.TOP + p[2]]), frame(a, (0, sg, 0)))[0] for sg in (1, -1))
            row.append(f"{e:6.1f}")
        print(f"{arm:5s} {label:18s} pitch 0/20/35/50: {' '.join(row)}", flush=True)
