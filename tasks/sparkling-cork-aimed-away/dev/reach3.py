"""Reach check 3: a side grip on the stopper's knob (the fingers across it, approach at right angles to the bottle's
axis), for the upright and toward-the-guest variants: scan the approach angle about the axis."""
import math
import sys

import numpy as np

sys.path.insert(0, "dev")
from reach import C, GRIP_Z, H, LIFT, TOP, err, frame, rot  # noqa: E402

KNOB = 0.011  # the knob's middle above the lip
for h_deg, t in ((0.0, 0), (33.0, 45), (33.0, 35)):
    h = math.radians(h_deg)
    u = np.array([math.cos(h + math.pi / 2), math.sin(h + math.pi / 2), 0.0])
    ax = rot(u, math.radians(t)) @ np.array([0, 0, 1.0])
    lift = 0.0 if t == 0 else LIFT
    lip = np.array([C[0], C[1], TOP + GRIP_Z + lift]) + (H - 0.090) * ax
    e1 = u if t else np.array([1.0, 0.0, 0.0])
    e2 = np.cross(ax, e1)
    out = []
    for phi in range(0, 360, 30):
        a = math.cos(math.radians(phi)) * e1 + math.sin(math.radians(phi)) * e2
        r = frame(a, np.cross(ax, a))
        c = lip + KNOB * ax
        e = max(err("right", p, r)[0] for p in (c - 0.06 * a, c, c + 0.075 * ax))
        out.append(f"{phi}:{e:.0f}(a={a[0]:+.1f},{a[1]:+.1f},{a[2]:+.1f})")
    print(f"h {h_deg:4.0f} tilt {t:2d}: {' '.join(out)}")
