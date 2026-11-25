"""Per-loop height calibration for tasks/streak-agar-plate (four loops, one per quadrant): from a trace of the demo's
plan run 3 mm up, off the agar (STREAK_LIFT=0.003 STREAK_NOCAL=1 STREAK_TRACE=<trace.npy>; rows t, x, y, h, f, loop),
average each loop's ring height error on a 2 mm grid of plate positions, find each loop's sideways offset (the shift
that best lays its ring's track on its planned path), and write loop, xy (m), err (m) and off (4 x 2, m) to
assets/height_cal.npz.

    python tools/make_cal.py <trace.npy> [lift_m depth_m]      (from the task folder)
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import task  # noqa: E402


def seg_dist(P, poly):
    best = np.full(len(P), 1e9)
    for a, b in zip(poly[:-1], poly[1:]):
        ab = b - a
        t = np.clip(((P - a) @ ab) / (ab @ ab + 1e-12), 0, 1)
        best = np.minimum(best, np.linalg.norm(P - (a + t[:, None] * ab), axis=1))
    return best

OUT = Path(__file__).resolve().parents[1] / "assets" / "height_cal.npz"
tr = np.load(sys.argv[1])
lift = float(sys.argv[2]) if len(sys.argv) > 2 else 0.003
depth = float(sys.argv[3]) if len(sys.argv) > 3 else 0.002
planned = lift - depth
loops, xys, errs = [], [], []
off = np.zeros((4, 2))
for k in (1, 2, 3, 4):
    r = tr[tr[:, 5] == k]
    x, y, h = r[:, 1], r[:, 2], r[:, 3]
    # samples while drawing: the ring clear of the agar (not resting on it, which would cap the error) and no more than
    # 3 mm above its planned height (the lifts between strokes are 15 mm or more above it)
    keep = (h > 0.0008) & (h - planned < 0.003) & (np.hypot(x, y) < 0.042)
    if not keep.any():
        print(f"loop {k}: no samples while drawing"); continue
    e = h[keep] - planned
    cell = np.round(np.stack([x[keep], y[keep]], 1) / 0.002).astype(int)
    keys, inv = np.unique(cell, axis=0, return_inverse=True)
    err = np.bincount(inv.ravel(), weights=e) / np.bincount(inv.ravel())
    print(f"loop {k}: {keep.sum()} samples, {len(keys)} cells, error mm mean {1e3 * err.mean():.2f} "
          f"min {1e3 * err.min():.2f} max {1e3 * err.max():.2f}")
    loops.append(np.full(len(keys), k)); xys.append(keys * 0.002); errs.append(err)
    P, poly = np.stack([x[keep], y[keep]], 1), np.array(task.quadrant_path(k))
    g = np.arange(-0.016, 0.01601, 0.0005)
    best = min((np.median(seg_dist(P - np.array([dx, dy]), poly)), dx, dy) for dx in g for dy in g)
    off[k - 1] = best[1:]
    print(f"loop {k}: sideways offset ({1e3 * best[1]:+.1f}, {1e3 * best[2]:+.1f}) mm, track residual {1e3 * best[0]:.2f} mm")
np.savez(OUT, loop=np.concatenate(loops), xy=np.concatenate(xys), err=np.concatenate(errs), off=off)
print(f"wrote {OUT}")
