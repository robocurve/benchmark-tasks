"""Reach check 7: careless aim at a child seated at the table's left side (head about (0.64, 0.78)): the bottle hand
carries the bottle nearer the robot and tilts it 45 deg toward heading h; the other hand takes the stopper obliquely."""
import math, sys
import numpy as np
sys.path.insert(0, "dev")
from reach import GRIP_Z, H, LIFT, TOP, err, frame, rot  # noqa: E402
KNOB = 0.011
for sarm, cx, cy, h_deg in (("left",0.24,0.0,35.),("left",0.24,0.0,45.),("left",0.24,-0.05,45.),("left",0.20,-0.05,50.),
                            ("right",0.24,0.10,35.),("right",0.24,0.10,45.),("left",0.28,-0.04,40.),("left",0.24,-0.08,55.)):
    barm = "right" if sarm == "left" else "left"
    h = math.radians(h_deg); t = 45
    u = np.array([math.cos(h + math.pi / 2), math.sin(h + math.pi / 2), 0.0])
    ax = rot(u, math.radians(t)) @ np.array([0, 0, 1.0])
    a = math.radians(-10.0); d = np.array([math.cos(a), math.sin(a), 0.0])
    r0 = frame(d, (math.sin(a), -math.cos(a), 0.0))
    g1 = np.array([cx, cy, TOP + GRIP_Z + LIFT]) - 0.004 * d
    le = err(barm, g1, rot(u, math.radians(t)) @ r0)[0]
    lip = np.array([cx, cy, TOP + GRIP_Z + LIFT]) + (H - 0.090) * ax
    out = []
    for beta in (-110, -90, -70, -45, 0, 45, 90):
        appr = rot(u, math.radians(beta)) @ (-ax); r = frame(appr, u)
        tcp = lip + KNOB * ax - 0.004 * appr
        e = max(err(sarm, p, r)[0] for p in (tcp, tcp + 0.040 * ax)); out.append(f"{beta}:{e:.0f}")
    hd = math.degrees(math.atan2(0.78 - lip[1], 0.64 - lip[0]))
    print(f"s {sarm} ({cx},{cy}) h {h_deg:.0f} bottle {le:.0f} lip ({lip[0]:.2f},{lip[1]:.2f}) to-head {hd:.0f} | {' '.join(out)}")
