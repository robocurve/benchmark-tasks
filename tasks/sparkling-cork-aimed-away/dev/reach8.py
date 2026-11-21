"""Reach check 8 (the 'turn' variant): the demo's grips mirrored. The RIGHT hand holds the bottle's body at the coaster
(approach +10 deg), lifted 35 mm and tilted 45 deg toward heading h; the LEFT hand holds the stopper along the axis
(twisted 30 deg), drawn 0 to 42 mm. h swept from toward the child (95) to away from it (125). Worst IK error, mm+deg."""
import math
import sys

import numpy as np

sys.path.insert(0, "dev")
from reach import GRIP_Z, H, LIFT, TOP, err, frame, rot  # noqa: E402

cx, cy = 0.34, 0.02
a = math.radians(10.0)
d = np.array([math.cos(a), math.sin(a), 0.0])
r0 = frame(d, (math.sin(a), -math.cos(a), 0.0))
g1 = np.array([cx, cy, TOP + GRIP_Z + LIFT]) - 0.004 * d
for h_deg in (95, 110, 125):
    h = math.radians(h_deg)
    u = np.array([math.cos(h + math.pi / 2), math.sin(h + math.pi / 2), 0.0])
    ax = rot(u, math.radians(45)) @ np.array([0, 0, 1.0])
    be = err("right", g1, rot(u, math.radians(45)) @ r0)[0]
    lip = g1 + 0.004 * d + ax * (H - 0.090)
    r1 = rot(ax, math.radians(30)) @ frame(-ax, u)
    se = max(err("left", lip + ax * (0.018 + k), r1)[0] for k in (0.0, 0.042))
    hd = math.degrees(math.atan2(0.73 - lip[1], 0.66 - lip[0]))
    hs = math.degrees(math.atan2(-lip[1], -lip[0]))
    print(f"h {h_deg:4d} bottle {be:4.0f} stopper {se:4.0f} lip ({lip[0]:.2f},{lip[1]:.2f}) to-head {hd:.0f} to-self {hs:.0f}")
