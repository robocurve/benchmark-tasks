"""Reach check 4: the right hand's nozzle pinch rolls at the start place; the left hand's too."""
import math, sys
import numpy as np
sys.path.insert(0, "dev")
from reach2 import err, frame, TOP
for arm in ("right", "left"):
    for roll in (0, 30, -30, 120, -120, 150, -150, 180):
        c = np.array([0.0, math.cos(math.radians(roll)), math.sin(math.radians(roll))])
        for dx, lab in ((0.0, "pinch"), (-0.06, "back")):
            p = (0.26 - 0.058 + dx, -0.10, TOP + 0.227)
            e, q = err(arm, p, frame((1, 0, 0), c))
            print(f"{'OK ' if e < 3 else 'BAD'} {arm} roll {roll:4d} {lab}: err {e:6.1f} q {np.round(q, 2)}")
