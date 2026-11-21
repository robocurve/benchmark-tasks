"""Development check (no simulation): can each arm reach every grasp and end pose of the plan, within joint limits?
Rim grips close across the wall (normal to it) and may lean only along the wall; handle grips close across the grip.
Usage: PYTHONPATH=<yam repo> python dev/reach.py"""
import math
import pathlib
import sys

import numpy as np
from yam_arena import ik, station
from yam_arena.constants import READY

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import layout as L  # noqa: E402

SEEDS = (READY, [0.0, 1.5, 1.2, -0.3, 0.0, 0.0], [0.5, 1.2, 1.0, -0.5, 0.0, 1.5], [-0.5, 1.2, 1.0, -0.5, 0.0, -1.5])


def frame(approach, close):
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - z * float(np.dot(close, z))
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def lean(direction, deg):
    t = math.radians(deg)
    d = np.asarray(direction, float)
    return np.array([d[0] * math.sin(t), d[1] * math.sin(t), -math.cos(t)])


def err(arm, p, r):
    best = (1e9, None)
    for q0 in SEEDS:
        for flip in (1.0, -1.0):
            rr = r @ np.diag([flip, flip, 1.0])
            q, pe, re = ik.solve(np.asarray(p, float), ik.matrix_to_quat_xyzw(rr), np.array(q0, float),
                                 base_pos=station.BASES[arm], iters=150)
            e = pe * 1000 + math.degrees(re)
            if e < best[0]:
                best = (e, flip)
    return best


def best_lean(arm, p, close, lean_dir, leans=(0, 15, 30, 40)):
    out = []
    for d in leans:
        e, _ = err(arm, p, frame(lean(lean_dir, d), close))
        out.append(f"{d}:{e:.0f}")
    return " ".join(out)


def _main():
    T = L.TOP
    rows = []
    c8, s8 = math.cos(math.radians(L.GRIP_PITCH_DEG)), math.sin(math.radians(L.GRIP_PITCH_DEG))
    rim = T + L.PAN_H - 0.012
    for i, y in enumerate(L.SLOTS_Y):
        for arm in ("left", "right"):
            s = 1.0 if arm == "left" else -1.0
            h = np.array([L.PAN_X + L.HINGE_X - 0.07 * c8, y, T + L.HINGE_Z + 0.07 * s8])
            rows.append((arm, f"slot {i+1} handle (rock)", h, (0, 1, 0), (1, 0, 0)))
            rows.append((arm, f"slot {i+1} handle up 12mm", h + [0, 0, 0.012], (0, 1, 0), (1, 0, 0)))
            side = np.array([L.PAN_X, y + s * (L.PAN_R - L.WALL_T / 2), rim])  # hold at the side nearer this arm
            rows.append((arm, f"slot {i+1} near side rim (hold)", side, (0, 1, 0), (1, 0, 0)))
            back = np.array([L.PAN_X + L.PAN_R - L.WALL_T / 2, y, rim])
            rows.append((arm, f"slot {i+1} back rim (carry)", back, (1, 0, 0), (0, s, 0)))
            rows.append((arm, f"slot {i+1} back rim lifted", back + [0, 0, 0.10], (1, 0, 0), (0, s, 0)))
    for (bx, by, _), arm, sgn in ((L.BURNERS[0], "left", -1.0), (L.BURNERS[1], "right", 1.0)):
        g = np.array([bx, by + sgn * (L.PAN_R - L.WALL_T / 2), T + L.BURNER_TOP + L.PAN_H - 0.012])
        rows.append((arm, f"burner {arm} inner rim place", g, (0, 1, 0), (1, 0, 0)))
        rows.append((arm, f"burner {arm} inner rim above", g + [0, 0, 0.025], (0, 1, 0), (1, 0, 0)))
        rows.append((arm, f"burner {arm} back off", g + [-0.04, 0, 0.06], (0, 1, 0), (1, 0, 0)))
    g = np.array([L.TRAY_C[0] + L.PAN_R - L.WALL_T / 2, L.TRAY_C[1], T + 0.003 + L.PAN_H - 0.012])
    rows.append(("right", "tray back rim place", g, (1, 0, 0), (0, -1, 0)))
    rows.append(("right", "tray back rim above", g + [0, 0, 0.05], (1, 0, 0), (0, -1, 0)))
    mid = np.array([L.PAN_X + L.PAN_R - L.WALL_T / 2, -0.02, rim])
    rows.append(("left", "relay: set down at y -0.02", mid, (1, 0, 0), (0, 1, 0)))
    rows.append(("right", "relay: pick up at y -0.02", mid, (1, 0, 0), (0, -1, 0)))
    for arm, label, p, close, ld in rows:
        print(f"{arm:5s} {label:34s} lean->err {best_lean(arm, p, close, ld)}", flush=True)


if __name__ == "__main__":
    _main()
