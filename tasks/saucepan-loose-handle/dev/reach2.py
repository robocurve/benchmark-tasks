"""Development check, plan 2: carry by the side rim (closing across the wall at the pan's +-y side, hand leaning
forward), then turn the pan on its burner by sweeping its handle 90 degrees outward. Usage: python dev/reach2.py"""
import math, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from reach import best_lean  # noqa: E402
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import layout as L  # noqa: E402

T = L.TOP
rim = T + L.PAN_H - 0.012
c8, s8 = math.cos(math.radians(L.GRIP_PITCH_DEG)), math.sin(math.radians(L.GRIP_PITCH_DEG))
rows = []
for i, y in enumerate(L.SLOTS_Y):
    for arm in ("left", "right"):
        for side in (1.0, -1.0):
            p = np.array([L.PAN_X, y + side * (L.PAN_R - L.WALL_T / 2), rim])
            rows.append((arm, f"slot {i+1} side {side:+.0f} rim", p, (0, 1, 0), (1, 0, 0)))
            rows.append((arm, f"slot {i+1} side {side:+.0f} rim lifted", p + [0, 0, 0.10], (0, 1, 0), (1, 0, 0)))
for (bx, by, _), arm in ((L.BURNERS[0], "left"), (L.BURNERS[1], "right")):
    for side in (1.0, -1.0):
        g = np.array([bx, by + side * (L.PAN_R - L.WALL_T / 2), T + L.BURNER_TOP + L.PAN_H - 0.012])
        rows.append((arm, f"burner {arm} side {side:+.0f} place", g, (0, 1, 0), (1, 0, 0)))
        rows.append((arm, f"burner {arm} side {side:+.0f} above", g + [0, 0, 0.03], (0, 1, 0), (1, 0, 0)))
    s = 1.0 if arm == "left" else -1.0
    for th in (0, 30, 60, 90):
        t = math.radians(th)
        d = np.array([-math.cos(t), s * math.sin(t), 0.0])
        g = np.array([bx, by, T + L.BURNER_TOP + L.HINGE_Z + 0.07 * s8]) + d * (-L.HINGE_X + 0.07 * c8)
        close = np.array([-d[1], d[0], 0.0])
        rows.append((arm, f"burner {arm} handle sweep {th}", g, close, d))
        rows.append((arm, f"burner {arm} handle sweep {th} up", g + [0, 0, 0.05], close, d))
for side in (1.0, -1.0):
    g = np.array([L.TRAY_C[0], L.TRAY_C[1] + side * (L.PAN_R - L.WALL_T / 2), T + 0.003 + L.PAN_H - 0.012])
    rows.append(("right", f"tray side {side:+.0f} place", g, (0, 1, 0), (1, 0, 0)))
    rows.append(("right", f"tray side {side:+.0f} above", g + [0, 0, 0.06], (0, 1, 0), (1, 0, 0)))
for arm, label, p, close, ld in rows:
    print(f"{arm:5s} {label:34s} lean->err {best_lean(arm, p, close, ld, leans=(0, 20, 30, 40))}", flush=True)
