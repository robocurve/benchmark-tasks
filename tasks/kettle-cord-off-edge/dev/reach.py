"""Reach check for the demo plan (no simulation): left hand, straight-down grips."""

import math, numpy as np
from yam_arena import ik, station
from yam_arena.constants import READY
T = station.TABLE_TOP
SEEDS = (READY, [0.0, 1.5, 1.2, -0.3, 0.0, 0.0], [0.5, 1.2, 1.0, -0.5, 0.0, 1.5], [-0.5, 1.2, 1.0, -0.5, 0.0, -1.5],
         [0.3, 0.6, 0.6, 0.0, 0.0, 0.0], [-0.3, 0.6, 0.6, 0.0, 0.0, 0.0], [0.6, 1.8, 1.6, -0.2, 0.0, 0.0],
         [-0.6, 1.8, 1.6, -0.2, 0.0, 0.0], [0.0, 1.2, 0.8, 0.6, 0.0, 0.0], [0.0, 0.9, 1.4, -1.2, 0.0, 0.0])
def err(arm, p, r):
    best = 1e9
    for q0 in SEEDS:
        q, pe, re = ik.solve(np.asarray(p, float), ik.matrix_to_quat_xyzw(r), np.array(q0, float), base_pos=station.BASES[arm], iters=150)
        best = min(best, pe * 1000 + math.degrees(re))
    return best
# reach for the plan: pinch near B, lift, lay-down target, the switch press (jug handle toward -y)
print("plan reach (left):")
TIP = 0.013
for lab, p in (("pinch", (0.16, 0.62, T + 0.014)), ("lift", (0.16, 0.62, T + 0.11)), ("up-mid", (0.15, 0.54, T + 0.15)), ("up-mid2", (0.15, 0.54, T + 0.20)),
               ("up-tgt", (0.13, 0.38, T + 0.15)), ("up-tgt-b", (0.16, 0.40, T + 0.15)), ("up-tgt-c", (0.18, 0.38, T + 0.18)), ("lay", (0.13, 0.38, T + 0.024)), ("lay-b", (0.16, 0.40, T + 0.024)),
               ("switch-above", (0.32, 0.394, T + 0.02 + 0.03 + 0.012 + 0.06)), ("switch-press", (0.32, 0.394, T + 0.02 + 0.03 + TIP - 0.008))):
    r = np.column_stack([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])
    r2 = np.column_stack([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])
    print(f"  {lab}: x-close {err('left', p, r):.0f}  y-close {err('left', p, r2):.0f}")

import math as _m
for tilt in (20, 30, 40):
    a = np.array([0.0, _m.sin(_m.radians(tilt)), -_m.cos(_m.radians(tilt))])
    z = a; x = np.array([1.0, 0, 0]); rp = np.column_stack([x, np.cross(z, x), z])
    tip = np.array([0.32, 0.394, T + 0.0205 + 0.030 + 0.0045 + 0.003])
    g = tip - a * TIP
    print(f"  press tilt {tilt}: high {err('left', (g[0], g[1] - 0.06, T + 0.30), rp):.0f} on {err('left', g, rp):.0f} "
          f"pressed {err('left', g + a * 0.014, rp):.0f}")
lay = np.column_stack([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])
print(f"  lay-in (0.13, 0.35): {err('left', (0.13, 0.35, T + 0.024), lay):.0f}")
for tilt in (30,):
    a = np.array([0.0, _m.sin(_m.radians(tilt)), -_m.cos(_m.radians(tilt))])
    z = a; x = np.array([1.0, 0, 0]); rp = np.column_stack([x, np.cross(z, x), z])
    tip = np.array([0.32, 0.394, T + 0.0205 + 0.030 + 0.0045 + 0.003])
    g = tip - a * TIP
    for hz in (0.20, 0.22, 0.24, 0.26):
        for dy in (-0.04, -0.08):
            print(f"  high z {hz} dy {dy}: {err('left', (g[0], g[1] + dy, T + hz), rp):.0f}")
