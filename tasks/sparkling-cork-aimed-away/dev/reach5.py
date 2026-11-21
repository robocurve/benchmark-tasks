"""Reach check 5: the bottle tilted toward the guest. Either hand on the stopper (oblique grip, fingers across along u);
the other hand holds the body. Prints the bottle hand's worst error and the stopper hand's error per approach angle."""
import math
import sys

import numpy as np

sys.path.insert(0, "dev")
from reach import C, GRIP_Z, H, LIFT, TOP, err, frame, rot  # noqa: E402

KNOB = 0.011
for sarm, h_deg, t, a_deg in (("left", 20.0, 45, 40.0), ("left", 20.0, 45, 70.0), ("left", 0.0, 45, 40.0),
                              ("right", -20.0, 40, -10.0), ("right", -25.0, 35, -10.0), ("left", -20.0, 45, 40.0)):
    barm = "right" if sarm == "left" else "left"
    h = math.radians(h_deg)
    u = np.array([math.cos(h + math.pi / 2), math.sin(h + math.pi / 2), 0.0])
    ax = rot(u, math.radians(t)) @ np.array([0, 0, 1.0])
    a = math.radians(a_deg)
    d = np.array([math.cos(a), math.sin(a), 0.0])
    r0 = frame(d, (math.sin(a), -math.cos(a), 0.0))
    g0 = np.array([C[0], C[1], TOP + GRIP_Z]) - 0.004 * d
    le = max(err(barm, g0 + [0, 0, LIFT], rot(u, math.radians(t)) @ r0)[0], err(barm, g0, r0)[0])
    lip = np.array([C[0], C[1], TOP + GRIP_Z + LIFT]) + (H - 0.090) * ax
    out = []
    for beta in (-135, -110, -90, -70, -45, 45, 70, 90, 110, 135):
        appr = rot(u, math.radians(beta)) @ (-ax)
        r = frame(appr, u)
        tcp = lip + KNOB * ax - 0.004 * appr
        e = max(err(sarm, p, r)[0] for p in (tcp, tcp + 0.040 * ax))
        out.append(f"{beta}:{e:.0f}")
    print(f"s {sarm} h {h_deg:5.0f} tilt {t} bottle-a {a_deg:4.0f} bottlehand {le:.0f} | {' '.join(out)}")
