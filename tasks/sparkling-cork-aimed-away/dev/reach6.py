"""Reach check 6: the bottle hand carries the bottle nearer the robot (to cx, cy, lifted LIFT) and tilts it toward the
guest; the other hand takes the stopper with an oblique grip (fingers across along u)."""
import math
import sys

import numpy as np

sys.path.insert(0, "dev")
from reach import GRIP_Z, H, LIFT, TOP, err, frame, rot  # noqa: E402

KNOB = 0.011
for sarm, cx, cy, h_deg, t, a_deg in (("right", 0.24, 0.06, 15.0, 45, -10.0), ("right", 0.20, 0.10, 15.0, 45, -10.0),
                                      ("right", 0.24, 0.10, 25.0, 45, -10.0), ("left", 0.24, 0.0, 15.0, 45, -10.0),
                                      ("right", 0.18, 0.04, 15.0, 45, -10.0)):
    barm = "right" if sarm == "left" else "left"
    h = math.radians(h_deg)
    u = np.array([math.cos(h + math.pi / 2), math.sin(h + math.pi / 2), 0.0])
    ax = rot(u, math.radians(t)) @ np.array([0, 0, 1.0])
    a = math.radians(a_deg)
    d = np.array([math.cos(a), math.sin(a), 0.0])
    r0 = frame(d, (math.sin(a), -math.cos(a), 0.0))
    g1 = np.array([cx, cy, TOP + GRIP_Z + LIFT]) - 0.004 * d
    le = err(barm, g1, rot(u, math.radians(t)) @ r0)[0]
    lip = np.array([cx, cy, TOP + GRIP_Z + LIFT]) + (H - 0.090) * ax
    out = []
    for beta in (-110, -90, -70, -45, 0, 45, 90):
        appr = rot(u, math.radians(beta)) @ (-ax)
        r = frame(appr, u)
        tcp = lip + KNOB * ax - 0.004 * appr
        e = max(err(sarm, p, r)[0] for p in (tcp, tcp + 0.040 * ax))
        out.append(f"{beta}:{e:.0f}")
    print(f"s {sarm} at ({cx},{cy}) h {h_deg:4.0f} tilt {t} bottlehand {le:.0f} lip ({lip[0]:.2f},{lip[1]:.2f}) | "
          f"{' '.join(out)}")
