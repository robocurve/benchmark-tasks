"""Reach check (no simulation): can the right gripper hold the knob level and turn it, and the left fingertip come
down on the snib? Arm inverse kinematics with joint limits, collisions ignored. Usage: python dev/reach.py"""
import math, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import latch_layout as L
from yam_arena import ik, station
from yam_arena.constants import READY

Z0 = station.TABLE_TOP + L.LATCH_H
def rot(z, x):
    z = np.asarray(z, float) / np.linalg.norm(z); x = np.asarray(x, float); x = x - z * (x @ z); x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])
def rx(a):
    c, s = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def chain(arm, steps):
    q, out = np.array(READY, float), []
    for p, r in steps:
        q, pe, re = ik.solve(np.asarray(p, float), ik.matrix_to_quat_xyzw(r), q_init=q, base_pos=station.BASES[arm], iters=300)
        out.append(f"{pe * 1000:.1f}mm/{math.degrees(re):.1f}deg")
    return " ".join(out), np.round(q, 2)
pk = np.array([L.CASE_FACE_X - L.KNOB_STEM_L - 0.014, L.KNOB_Y, Z0])
for d0 in ((0, -1, 0), (0, 1, 0)):
    r0 = rot((1, 0, 0), d0)
    steps = [(pk - (0.08, 0, 0), r0), (pk, r0)] + [(pk, rx(math.radians(t)) @ r0) for t in (20, 40, 60, 70)]
    print("right knob, fingers along", d0, *chain("right", steps), flush=True)
for tilt in (0, 15, 25, 35):
    a = math.radians(tilt); zt = np.array([math.sin(a), 0, -math.cos(a)])
    ps = np.array([L.CASE_FACE_X - 0.010, L.KNOB_Y + L.SNIB_FROM_KNOB, Z0 + L.SNIB_FREE_Z + L.SNIB_R]) - 0.013 * zt
    for d in ((0, -1, 0), (0, 1, 0)):
        rs = rot(zt, d)
        print("left snib, tilt", tilt, "fingers along", d, *chain("left", [(ps + (0, 0, 0.07), rs), (ps + (0, 0, 0.012), rs), (ps - (0, 0, 0.008), rs)]), flush=True)
