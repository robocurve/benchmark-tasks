"""Build the wrapped start state offline (wrap_start.npz), the way a person wraps the pack: the sheet lies flat as a
diamond under the tray; the near corner is folded up and over the tray and its tip folded back into a tab, then the
left corner, then the right, then the far corner last (its tab pointing back at the far side). During each fold the
part of the flap that lies over the tray is carried along a hinge-by-hinge fold (kinematic), the rest drapes freely;
then everything is released and settles under gravity."""
import math
import pathlib
import sys
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import wrap as wr  # noqa: E402

TABLE_Z = 0.7475
CENTER = np.array([0.30, 0.0])
ORDER = ["near", "left", "right", "far"]
FOLD_U = {"near": -0.06, "far": -0.06, "left": -0.03, "right": -0.03}   # tab fold line (from the tray centre)
FAR = np.zeros((2, 3)) + [0, 0, 5]
EYE = np.array([np.eye(3)] * 2)
OPEN = np.full(2, 0.09)


def run(w, n):
    for _ in range(n):
        w.step(FAR, EYE, OPEN)


def fold_xy(w, name, k, th1, th2, th3):
    n2 = wr.DIRS[name]
    t2 = np.array([-n2[1], n2[0]])
    h = wr.TRAY[0] / 2 if name in ("near", "far") else wr.TRAY[1] / 2
    half_t = wr.TRAY[1] / 2 if name in ("near", "far") else wr.TRAY[0] / 2
    off = (k + 1) * wr.LAYER
    u0 = h + off
    s1 = wr.TRAY[2] + off
    d_tab = s1 + u0 - FOLD_U[name]
    fx = wr.flat_xy(w.ab)
    un = fx @ n2
    ut = fx @ t2
    sel = (w.flap == wr.FLAPS.index(name)) & (np.abs(ut) <= half_t - 0.004) & (un > u0)
    d = un[sel] - u0
    segs = [(np.minimum(d, s1), th1), (np.clip(d - s1, 0, d_tab - s1), th1 + th2),
            (np.maximum(d - d_tab, 0), th1 + th2 - th3)]
    u = np.full(len(d), u0); z = np.full(len(d), wr.THICK)
    for ln, ph in segs:
        u = u + ln * math.cos(ph); z = z + ln * math.sin(ph)
    xy = CENTER + u[:, None] * n2 + ut[sel][:, None] * t2
    return sel, np.c_[xy, TABLE_Z + z]


def main():
    w = wr.Wrap(TABLE_Z)
    fx = wr.flat_xy(w.ab)
    x0 = np.c_[CENTER + fx, np.full(w.n, TABLE_Z + wr.THICK)]
    w.place(x0, np.r_[CENTER, TABLE_Z], 0.0)
    t0 = time.time()
    N = 16
    for k, name in enumerate(ORDER):
        for phase in range(3):
            for s in range(1, N + 1):
                f = s / N
                f = f * f * (3 - 2 * f)
                th = [0.0, 0.0, 0.0]
                for p in range(phase):
                    th[p] = math.pi / 2 if p < 2 else math.radians(172)
                th[phase] = f * (math.pi / 2 if phase < 2 else math.radians(172))
                sel, pos = fold_xy(w, name, k, *th)
                w.fixed = sel
                w.x[sel] = pos
                w.step(FAR, EYE, OPEN)
        print(f"{name} folded {time.time() - t0:.1f}s strain {w.strain():.2f} p99 {w.strain(99):.3f} bad {w.bad}",
              flush=True)
        w.fixed = np.zeros(w.n, bool); w.v[:] = 0
        run(w, 10)
    run(w, 80)
    print(f"settled {time.time() - t0:.1f}s strain {w.strain():.3f} p99 {w.strain(99):.3f} bad {w.bad}")
    np.savez(HERE / "wrap_start.npz", x=w.x)


if __name__ == "__main__":
    main()
