"""No-physics reach check of the left hand's knob strokes: python dev/reach_knob.py <yaw deg> <from deg> <to deg> <tilt>"""
import math, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import layout as L
from yam_arena import ik, station
from yam_arena.constants import READY
from yam_arena.demo import grip_rotation
yaw, a0, a1, tilt = (float(v) for v in sys.argv[1:5])
TIP, PRESS = 0.013, 0.0025
def ry(a):
    c, s = math.cos(a), math.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
down = grip_rotation(math.radians(yaw))
axis = np.array([L.X, L.STICK_Y + L.BARREL_L + L.KNOB_L / 2 + 0.004, L.AXIS_Z])
q = np.array(READY, float); worst = (0, 0)
for th in np.radians(np.linspace(a0, a1, 12)):
    p = axis + (L.KNOB_R - PRESS + TIP) * np.array([math.sin(th), 0, math.cos(th)])
    q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(ry(tilt * th) @ down), q, base_pos=station.BASES["left"], iters=300)
    worst = max(worst[0], pe * 1000), max(worst[1], math.degrees(re))
    print(f"th {math.degrees(th):6.1f} err {pe*1000:5.2f} mm {math.degrees(re):5.2f} deg q={np.round(q, 2)}")
print("worst", np.round(worst, 2))
