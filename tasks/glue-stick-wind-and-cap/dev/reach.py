"""No-physics reach check of the demo's poses (IK only). Run from the task folder: python dev/reach.py"""
import math
import pathlib
import sys
import types

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import layout as L  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import READY  # noqa: E402
from yam_arena.demo import grip_rotation  # noqa: E402

TIP, PRESS, SWEEP, KNOB_TIP_Y = 0.013, 0.0025, math.radians(float(sys.argv[1]) if len(sys.argv) > 1 else 55.0), 0.004


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


down = grip_rotation(math.pi / 2)
bp = np.array([L.X, L.STICK_Y, L.AXIS_Z])
axis = bp + np.array([0, L.BARREL_L + L.KNOB_L / 2 + KNOB_TIP_Y, 0])
poses = []
for th in np.linspace(-SWEEP, SWEEP, 11):
    for rho in (L.KNOB_R - PRESS, L.KNOB_R + 0.012):
        poses.append(("left", f"knob th={math.degrees(th):.0f} rho={rho*1000:.1f}",
                      axis + (rho + TIP) * np.array([math.sin(th), 0, math.cos(th)]), ry(th) @ down))
poses.append(("left", "knob top clear", axis + (L.KNOB_R - PRESS + 0.020 + TIP) * np.array([0, 0, 1]), down))
for arm, along in (("right", 0.012), ("left", 0.065)):
    g = bp + np.array([0, along, 0.004])
    poses += [(arm, f"barrel pinch {along}", g, down), (arm, f"over barrel {along}", g + [0, 0, 0.06], down)]
cap = np.array([L.CAP_X, L.CAP_Y, L.TOP + L.CAP_R])
g = cap + np.array([0, -(L.CAP_DEPTH + L.CAP_END) / 2 - 0.003, 0.003])
for dy in (0.0, 0.110 - 0.032 + L.STICK_Y * 0 - (0.110 + L.STICK_Y - 0.032), (-L.STICK_Y - 0.032 + 0.110) , (-L.STICK_Y + 0.006 + 0.110 - 0.032 + 0.032)):
    pass
rim_targets = {"cap pick": 0.0, "cap aim (gap 32 mm)": (L.STICK_Y - 0.032) - L.CAP_Y,
               "cap pushed (gap -6 mm)": (L.STICK_Y + 0.006) - L.CAP_Y}
for name, dy in rim_targets.items():
    poses.append(("right", name, g + np.array([0, dy, 0]), down))
worst = 0
q0 = {a: np.array(READY, float) for a in ("left", "right")}
for arm, name, p, r in poses:
    q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(r), q0[arm], base_pos=station.BASES[arm], iters=300)
    worst = max(worst, pe)
    print(f"{arm:5s} {name:28s} pos err {pe*1000:6.2f} mm rot err {math.degrees(re):5.2f} deg  q={np.round(q, 2)}")
print(f"worst position error {worst*1000:.2f} mm")
