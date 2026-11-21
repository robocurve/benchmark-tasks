"""Reach and top-camera check for moving the cord's edge crossings into the top camera's view (no simulation)."""
import math, numpy as np
from yam_arena import ik, station
from yam_arena.constants import READY
cam = station.top_camera()
R = ik.quat_xyzw_to_matrix(np.array(cam.rot_xyzw))
fx, fy, cx, cy = station.TOP_CAM_K
T = station.TABLE_TOP
def proj(p):
    d = R.T @ (np.asarray(p, float) - np.asarray(cam.pos))
    return (round(cx + fx * d[0] / -d[2]), round(cy - fy * d[1] / -d[2])) if d[2] < 0 else None
print("bases", {k: np.round(v, 3).tolist() for k, v in station.BASES.items()})
for arm in ("left", "right"):
    p, _ = ik.tcp_pose(np.array(READY[:6] if len(READY) > 6 else READY, float), station.BASES[arm])
    print("ready tcp", arm, np.round(p, 3), proj(p))
SEEDS = (READY, [0.0, 1.5, 1.2, -0.3, 0.0, 0.0], [0.5, 1.2, 1.0, -0.5, 0.0, 1.5], [-0.5, 1.2, 1.0, -0.5, 0.0, -1.5],
         [0.3, 0.6, 0.6, 0.0, 0.0, 0.0], [-0.3, 0.6, 0.6, 0.0, 0.0, 0.0], [0.6, 1.8, 1.6, -0.2, 0.0, 0.0],
         [-0.6, 1.8, 1.6, -0.2, 0.0, 0.0], [0.0, 1.2, 0.8, 0.6, 0.0, 0.0], [0.0, 0.9, 1.4, -1.2, 0.0, 0.0])
def err(arm, p, r):
    best = 1e9
    for q0 in SEEDS:
        q, pe, re = ik.solve(np.asarray(p, float), ik.matrix_to_quat_xyzw(r), np.array(q0, float), base_pos=station.BASES[arm], iters=150)
        best = min(best, pe * 1000 + math.degrees(re))
    return best
rx = np.column_stack([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])
for lab, p in (("pinch", (0.45, 0.57, T + 0.014)), ("pinch2", (0.45, 0.59, T + 0.014)), ("lift", (0.45, 0.57, T + 0.11)),
               ("mid", (0.36, 0.55, T + 0.15)), ("up-tgt", (0.25, 0.50, T + 0.15)), ("lay", (0.25, 0.47, T + 0.024)),
               ("up-tgt2", (0.28, 0.52, T + 0.15)), ("lay2", (0.28, 0.49, T + 0.024)), ("backoff", (0.25, 0.47, T + 0.09))):
    print(lab, f"{err('left', p, rx):.0f}")
TIP = 0.013
a = np.array([0.0, math.sin(math.radians(30)), -math.cos(math.radians(30))])
xx = np.array([1.0, 0, 0]); rp = np.column_stack([xx, np.cross(a, xx), a])
for kx, ky in ((0.40, 0.31), (0.42, 0.31), (0.38, 0.31)):
    tip = np.array([kx, ky - 0.126, T + 0.0205 + 0.030 + 0.0045 + 0.003]); g = tip - a * TIP
    out = [f"on {err('left', g, rp):.0f}", f"pressed {err('left', g + a * 0.014, rp):.0f}", f"pre {err('left', g - a * 0.04, rp):.0f}"]
    for d in (0.08, 0.12):
        out.append(f"axis{d} {err('left', g - a * d, rp):.0f}")
    for dy, hz in ((-0.06, 0.12), (-0.03, 0.12), (0.0, 0.12), (0.0, 0.16), (0.03, 0.14)):
        out.append(f"h{dy},{hz} {err('left', (g[0], g[1] + dy, T + hz), rp):.0f}")
    print("kettle", kx, ky, " ".join(out))
