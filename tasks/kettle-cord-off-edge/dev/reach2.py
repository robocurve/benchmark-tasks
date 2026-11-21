"""Reach check for the kettle moved forward to x 0.44 (left hand)."""
import math, numpy as np
import sys; sys.argv = ["x"]
exec(open("dev/reach.py").read().split("# reach for the plan")[0])
TIP = 0.013
def press(kx, tilt, hz, dy):
    a = np.array([0.0, math.sin(math.radians(tilt)), -math.cos(math.radians(tilt))])
    x = np.array([1.0, 0, 0]); rp = np.column_stack([x, np.cross(a, x), a])
    tip = np.array([kx, 0.394, T + 0.0205 + 0.030 + 0.0045 + 0.003]); g = tip - a * TIP
    return err('left', (g[0], g[1] + dy, T + hz), rp), err('left', g, rp), err('left', g + a * 0.014, rp)
for kx in ():
    for tilt in (30, 45):
        print(kx, tilt, [f"{v:.0f}" for v in press(kx, tilt, 0.20, -0.04)], [f"{v:.0f}" for v in press(kx, tilt, 0.22, -0.04)])
dn = np.column_stack([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])
for lab, p in () and (("pinch .26", (0.26, 0.62, T + 0.014)), ("lift .26", (0.26, 0.62, T + 0.11)), ("mid", (0.20, 0.52, T + 0.15))):
    print(lab, f"{err('left', p, dn):.0f}")
for hz in (0.10, 0.14):
    for dy in (-0.06, -0.09):
        print("high", hz, dy, [f"{v:.0f}" for v in press(0.44, 30, hz, dy)][0])
