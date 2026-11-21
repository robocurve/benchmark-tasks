"""Reach check 2: the stopper hand on the knob for the upright and toward-the-guest variants, scanning the grip roll and
the tilt; and the bottle hand's pre-grasp distance."""
import math
import sys

import numpy as np

sys.path.insert(0, "dev")
from reach import C, GRIP_Z, H, LIFT, TOP, err, frame, rot  # noqa: E402

for h_deg, tilts in ((0.0, (0,)), (33.0, (30, 45)), (60.0, (30,)), (10.0, (30,))):
    h = math.radians(h_deg)
    u = np.array([math.cos(h + math.pi / 2), math.sin(h + math.pi / 2), 0.0])
    for t in tilts:
        ax = rot(u, math.radians(t)) @ np.array([0, 0, 1.0])
        lift = 0.0 if t == 0 else LIFT
        lip = np.array([C[0], C[1], TOP + GRIP_Z + lift]) + (H - 0.090) * ax
        for arm in ("right", "left"):
            out = []
            for roll in range(0, 180, 30):
                c = rot(ax, math.radians(roll)) @ (u if t else np.array([1.0, 0, 0]))
                r1 = frame(-ax, c)
                e = max(err(arm, p, r1)[0] for p in (lip + 0.10 * ax, lip + 0.018 * ax, lip + 0.09 * ax))
                out.append(f"{roll}:{e:.0f}")
            print(f"h {h_deg:4.0f} tilt {t:2d} {arm}: {' '.join(out)}")
a = math.radians(-10)
d = np.array([math.cos(a), math.sin(a), 0.0])
r0 = frame(d, (math.sin(a), -math.cos(a), 0.0))
g0 = np.array([C[0], C[1], TOP + GRIP_Z]) - 0.004 * d
for back, up in ((0.10, 0.03), (0.07, 0.0), (0.07, 0.03), (0.05, 0.05)):
    print("left pre-grasp", back, up, round(err("left", g0 - back * d + [0, 0, up], r0)[0], 1))
