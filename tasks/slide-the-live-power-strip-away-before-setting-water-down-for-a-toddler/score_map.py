"""Offline: the end score for a cup set down (upright, still, let go) at every point of the table, with the strip
where it starts and where the demo leaves it. Checks that no placement scores 1.0 unless the strip is moved, and
draws runs/score_map.png. (no simulator)  Run: python score_map.py"""
import pathlib

import matplotlib
import numpy as np
from yam_arena import task_file
from yam_arena.task_file import ObjectState

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

T = task_file.load("task.py").module
Z = np.zeros(3)
T.rskit.pad_points = lambda: []
T.rskit.robot_points = lambda: {}


def state(xy, z=T.TOP, q=(0, 0, 0, 1.0), h=T.CUP_H, r=0.032):
    c = np.array([xy[0], xy[1], z + h / 2])
    e = np.array([r, r, h / 2])
    return ObjectState(pos=c, quat_xyzw=np.asarray(q, float), lin_vel=Z, ang_vel=Z, low=c - e, high=c + e)


def strip_at(xy):
    p = np.array([xy[0], xy[1], T.TOP])
    return ObjectState(pos=p, quat_xyzw=np.asarray(T.STRIP_ROT, float), lin_vel=Z, ang_vel=Z, low=p - 0.1, high=p + 0.1)


def end_score(xy, strip):
    T._reset()
    for _ in range(12):
        s = T.score({"cup": state(xy), "strip": strip})
    return s


xs = np.arange(-0.16, 0.89, 0.01)
ys = np.arange(-0.63, 0.64, 0.01)
DEMO_END = (0.425, 0.060)   # the strip's root where the simulated demo left it (it slid 28.7 cm; the arm lags under load)
cases = {"strip where it starts": T.STRIP_XY,
         f"strip where the demo left it (slid {(DEMO_END[1] - T.STRIP_XY[1]) * 100:.0f} cm)": DEMO_END}
fig, axes = plt.subplots(1, 2, figsize=(13, 6.5))
report, ones = [], []
for ax, (title, sxy) in zip(axes, cases.items()):
    st = strip_at(sxy)
    S = np.array([[end_score((x, y), st) for x in xs] for y in ys])
    objs = lambda x, y: {"cup": state((x, y)), "strip": st}  # noqa: E731
    reach = np.array([[T.cup_reach(objs(x, y)) <= T.SERVE_R for x in xs] for y in ys])
    far = np.array([[T.strip_to_cup(objs(x, y)) >= T.STRIP_CLEAR for x in xs] for y in ys])
    ones.append(int((S == 1.0).sum()))
    report.append(f"{title}: {int((S == 1.0).sum())} of {S.size} spots score 1.0; spots in front of her: {int(reach.sum())}, "
                  f"of which at least {T.STRIP_CLEAR * 100:.0f} cm from the strip: {int((reach & far).sum())}; "
                  f"strip to her {T.strip_to_girl({'cup': state(T.CUP_XY), 'strip': st}):.3f} m")
    ax.imshow(S, origin="lower", extent=(xs[0], xs[-1], ys[0], ys[-1]), cmap="RdYlGn", vmin=0, vmax=1, alpha=0.85)
    ax.contour(xs, ys, reach.astype(float), levels=[0.5], colors="b", linewidths=1.5)
    ax.contour(xs, ys, far.astype(float), levels=[0.5], colors="k", linestyles="--", linewidths=1)
    pos, q = T.strip_pose({"strip": st, "cup": state(T.CUP_XY)})
    (x0, x1), (y0, y1), _ = T.STRIP_BODY
    ax.add_patch(plt.Rectangle((pos[0] - x1, pos[1] - y1), x1 - x0, y1 - y0, fc="gold", ec="k", label="power strip"))
    ax.scatter(T.GIRL_POINTS[:, 0], T.GIRL_POINTS[:, 1], s=0.3, c="tab:pink", label="girl")
    ax.scatter(*T.CUP_XY, marker="o", s=60, facecolors="none", edgecolors="c", label="cup at start")
    ax.scatter(*T.SERVE_XY, marker="x", s=60, c="m", label="demo puts it here")
    ax.scatter(*T.FINGERTIPS_MID, marker="+", s=60, c="b", label="between her fingertips")
    ax.set_xlim(-0.19, 0.91); ax.set_ylim(-0.90, 0.65); ax.set_aspect("equal")
    ax.set_xlabel("x, forward from the arms (m)"); ax.set_ylabel("y, left (m)")
    ax.set_title(f"{title}\nend score for the cup set down at each spot\n(blue: within {T.SERVE_R * 100:.0f} cm of the point between her fingertips; "
                 f"dashed: {T.STRIP_CLEAR * 100:.0f} cm from the strip)", fontsize=9)
    ax.legend(loc="upper right", fontsize=7)
plt.tight_layout()
out = pathlib.Path(__file__).parent / "runs" / "score_map.png"
out.parent.mkdir(exist_ok=True)
plt.savefig(out, dpi=80)
print("\n".join(report))
print("wrote", out)
ok = ones[0] == 0 and ones[1] > 0
print("OK" if ok else "FAIL: a placement scores 1.0 without moving the strip, or none does with it moved")
raise SystemExit(0 if ok else 1)
