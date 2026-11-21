"""Geometry check (no simulation): table edges, arm bases, which table points the top camera sees, and IK reach."""
import math, numpy as np
from yam_arena import ik, station
from yam_arena.constants import READY
print("bases", {k: np.round(v, 3).tolist() for k, v in station.BASES.items()})
print("top", station.TABLE_TOP, "size", station.TABLE_SIZE, "centre", np.round(station.TABLE_CENTER, 3))
cam = station.top_camera()
print("cam", np.round(cam.pos, 3), np.round(cam.rot_xyzw, 3), cam.lens if hasattr(cam, "lens") else "")
R = ik.quat_xyzw_to_matrix(np.array(cam.rot_xyzw)) if hasattr(ik, "quat_xyzw_to_matrix") else None
fx, fy, cx, cy = station.TOP_CAM_K
def proj(p):  # opengl camera: looks down -z, y up
    d = R.T @ (np.asarray(p) - np.asarray(cam.pos))
    if d[2] >= 0: return None
    u = cx + fx * d[0] / -d[2]; v = cy - fy * d[1] / -d[2]
    return round(u), round(v)
T = station.TABLE_TOP
c = np.array(station.TABLE_CENTER); sx, sy, _ = station.TABLE_SIZE
x0, x1, y0, y1 = c[0]-sx/2, c[0]+sx/2, c[1]-sy/2, c[1]+sy/2
print("table x", round(x0,3), round(x1,3), "y", round(y0,3), round(y1,3))
for lab, p in [("near-left", (x0, y1, T)), ("near-right", (x0, y0, T)), ("far-left", (x1, y1, T)), ("far-right", (x1, y0, T))]:
    print(lab, proj(p))
for x in (0.0, 0.2, 0.4, 0.6):
    print("right edge x", x, proj((x, y0, T)), "10cm below", proj((x, y0, T-0.10)), "| left edge", proj((x, y1, T)))
for y in (-0.3, 0.0, 0.3):
    print("far edge y", y, proj((x1, y, T)), "below", proj((x1, y, T-0.1)))
SEEDS = (READY, [0.0, 1.5, 1.2, -0.3, 0.0, 0.0], [0.5, 1.2, 1.0, -0.5, 0.0, 1.5], [-0.5, 1.2, 1.0, -0.5, 0.0, -1.5],
         [0.3, 0.6, 0.6, 0.0, 0.0, 0.0], [-0.3, 0.6, 0.6, 0.0, 0.0, 0.0], [0.6, 1.8, 1.6, -0.2, 0.0, 0.0],
         [-0.6, 1.8, 1.6, -0.2, 0.0, 0.0], [0.0, 1.2, 0.8, 0.6, 0.0, 0.0], [0.0, 0.9, 1.4, -1.2, 0.0, 0.0])
def err(arm, p, r):
    best = 1e9
    for q0 in SEEDS:
        q, pe, re = ik.solve(np.asarray(p, float), ik.matrix_to_quat_xyzw(r), np.array(q0, float), base_pos=station.BASES[arm], iters=150)
        best = min(best, pe * 1000 + math.degrees(re))
    return best
down = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])
def side(dy):  # approach horizontally along +-y (fingers close along z)
    z = np.array([0, dy, 0.0]); x = np.array([0, 0, 1.0]); return np.column_stack([x, np.cross(z, x), z])
def fwd():
    z = np.array([1.0, 0, 0]); x = np.array([0, 0, 1.0]); return np.column_stack([x, np.cross(z, x), z])
import sys
for arm, yy in (("right", y0), ("left", y1)):
    s = 1 if arm == "left" else -1
    for x in (0.15, 0.25, 0.35, 0.45):
        out = []
        for lab, p, r in (("on-top", (x, yy - s*0.04, T + 0.01), down), ("edge-down", (x, yy - s*0.005, T + 0.005), down),
                          ("past-edge-down", (x, yy + s*0.02, T - 0.03), down), ("below-side", (x, yy + s*0.02, T - 0.06), side(s)),
                          ("below-side-15", (x, yy + s*0.02, T - 0.15), side(s))):
            out.append(f"{lab}:{err(arm, p, r):.0f}")
        print(arm, "x", x, " ".join(out))
for arm in ("left", "right"):
    for y in (-0.2, 0.0, 0.2):
        out = []
        for lab, p, r in (("far-on", (x1 - 0.04, y, T + 0.01), down), ("far-past", (x1 + 0.02, y, T - 0.03), down),
                          ("far-below-fwd", (x1 + 0.02, y, T - 0.06), fwd())):
            out.append(f"{lab}:{err(arm, p, r):.0f}")
        print(arm, "far y", y, " ".join(out))
