"""Reach check 4: an oblique grip on the stopper's knob. The fingers close across the knob along u (normal to the plane
of the bottle's tilt); the approach lies in the tilt plane, at an angle beta from the bottle's axis (-ax). Scans beta for
the careless cases: the bottle upright in the coaster, and tilted toward the guest. Also the left hand at its grip."""
import math
import sys

import numpy as np

sys.path.insert(0, "dev")
from reach import C, GRIP_Z, H, LIFT, TOP, err, frame, rot  # noqa: E402

KNOB = 0.011
for h_deg, t, a_deg in ((0.0, 0, -10.0), (180.0, 0, -10.0), (90.0, 0, -10), (-90.0, 0, -10), (20.0, 45, -10.0),
                        (20.0, 35, -10.0), (0.0, 45, -10.0), (-20.0, 45, -10.0), (20.0, 45, -40.0)):
    h = math.radians(h_deg)
    u = np.array([math.cos(h + math.pi / 2), math.sin(h + math.pi / 2), 0.0])
    w = np.array([math.cos(h), math.sin(h), 0.0])  # the heading direction
    ax = rot(u, math.radians(t)) @ np.array([0, 0, 1.0])
    lift = 0.0 if t == 0 else LIFT
    a = math.radians(a_deg)
    d = np.array([math.cos(a), math.sin(a), 0.0])
    r0 = frame(d, (math.sin(a), -math.cos(a), 0.0))
    g0 = np.array([C[0], C[1], TOP + GRIP_Z]) - 0.004 * d
    le = max(err("left", g0 + [0, 0, lift], rot(u, math.radians(t)) @ r0)[0], err("left", g0, r0)[0])
    lip = np.array([C[0], C[1], TOP + GRIP_Z + lift]) + (H - 0.090) * ax
    out = []
    for beta in range(-90, 91, 30):
        appr = rot(u, math.radians(beta)) @ (-ax)
        r = frame(appr, u)
        c = lip + KNOB * ax
        tcp = c - 0.004 * appr
        e = max(err("right", p, r)[0] for p in (tcp, tcp + 0.040 * ax))
        out.append(f"{beta}:{e:.0f}")
    print(f"h {h_deg:5.0f} tilt {t:2d} lefta {a_deg:4.0f} left {le:.0f} lipz {lip[2]-TOP:.3f} | {' '.join(out)}")
