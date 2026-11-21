"""No-physics reach check with both tools leaning away from each other: python dev/reach_tilt.py <lean deg> <knob tilt factor>"""
import math, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import layout as L
from yam_arena import ik, station
from yam_arena.constants import READY
from yam_arena.demo import grip_rotation
lean, tilt = math.radians(float(sys.argv[1])), float(sys.argv[2])
def ry(a):
    c, s = math.cos(a), math.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def rx(a):
    c, s = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
TIP, PRESS = 0.013, 0.0025
bp = np.array([L.X, L.STICK_Y, L.AXIS_Z])
axis = bp + np.array([0, L.BARREL_L + L.KNOB_L / 2 + 0.008, 0])
out = []
for th in np.radians(np.linspace(-45, 45, 7)):
    r = ry(tilt * th) @ rx(-lean) @ grip_rotation(-math.pi / 2)
    tip = axis + (L.KNOB_R - PRESS) * np.array([math.sin(th), 0, math.cos(th)])
    out.append(("left", f"knob {math.degrees(th):.0f}", tip - TIP * r[:, 2], r))
for arm, along, sgn, yaw in (("right", 0.012, 1, math.pi / 2), ("left", 0.065, -1, -math.pi / 2)):
    r = rx(sgn * lean) @ grip_rotation(yaw)
    out.append((arm, f"barrel pinch {along}", bp + [0, along, 0] - 0.004 * r[:, 2], r))
cap = np.array([L.CAP_X, L.CAP_Y, L.TOP + L.CAP_R])
r = rx(lean) @ grip_rotation(math.pi / 2)
c = cap + np.array([0, -(L.CAP_DEPTH + L.CAP_END) / 2 - 0.003, 0.0])
for name, dy in (("cap pick", 0.0), ("cap pushed", (L.STICK_Y + 0.006) - L.CAP_Y)):
    out.append(("right", name, c + [0, dy, 0] - 0.003 * r[:, 2], r))
worst = 0
for arm, name, p, r in out:
    q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(r), np.array(READY, float), base_pos=station.BASES[arm], iters=300)
    worst = max(worst, pe)
    print(f"{arm:5s} {name:22s} {pe*1000:6.2f} mm {math.degrees(re):5.2f} deg tcp={np.round(p, 3)}")
print(f"lean {math.degrees(lean):.0f} worst {worst*1000:.2f} mm")
