"""Offline: in the top camera's first frame (arms in their ready pose), how much of the nurse's hand, the tray, the
syringe and the cap does the arm hide? Rasterises the arms' link meshes at 320 x 180 and tests each point's pixel.
(no simulator)   python check_view.py"""
import sys

import numpy as np

sys.argv = sys.argv[:1] + ["/dev/null"]
exec(open("preview_view.py").read().split("nurse = np.load")[0])      # the projection and arm meshes


def arm_mask(q_left=READY):
    mask = np.zeros((180, 320), bool)
    for side in ("left", "right"):
        for v, f, _ in arm(q_left if side == "left" else READY, station.BASES[side]):
            for t in project(v)[0][f]:
                x0, y0 = np.maximum(np.floor(t.min(0)).astype(int), 0)
                x1, y1 = np.minimum(np.ceil(t.max(0)).astype(int), (319, 179))
                if x1 < x0 or y1 < y0:
                    continue
                X, Y = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
                a, b, c = t
                v0, v1 = b - a, c - a
                d = v0[0] * v1[1] - v1[0] * v0[1]
                if abs(d) < 1e-12:
                    continue
                px, py = X - a[0], Y - a[1]
                l1, l2 = (px * v1[1] - v1[0] * py) / d, (v0[0] * py - px * v0[1]) / d
                mask[y0:y1 + 1, x0:x1 + 1] |= (l1 >= 0) & (l2 >= 0) & (l1 + l2 <= 1)
    return mask


from yam_arena.demo import ArmPlan  # noqa: E402

M = arm_mask()


def hidden(pts):
    uv, _ = project(pts)
    ok = (uv[:, 0] >= 0) & (uv[:, 0] < 320) & (uv[:, 1] >= 0) & (uv[:, 1] < 180)
    h = np.zeros(len(pts), bool)
    h[ok] = M[uv[ok, 1].astype(int), uv[ok, 0].astype(int)]
    return h | ~ok


tray = np.c_[T.TRAY_XY + T.RIM_LOCAL @ T._yaw_matrix(T.TRAY_ROT)[:2, :2].T, np.full(len(T.RIM_LOCAL), T.TOP + 0.019)]
so = T.syringe_origin_for_grip(T.SYR_START, T.SYR_ROT_START)
ax = np.r_[so, T.TOP + 0.001] + np.c_[T._PROF_X, 0 * T._PROF_X, 0 * T._PROF_X + T.SYR_AXIS_Z] @ T._yaw_matrix(T.SYR_ROT_START).T
for name, pts in (("her hand", T.HAND_POINTS), ("tray rim", tray), ("syringe", ax)):
    h = hidden(pts)
    uv, _ = project(pts)
    print(f"{name}: {100 * h.mean():.0f}% hidden by the arms at the start; image x {uv[:, 0].min():.0f}..{uv[:, 0].max():.0f}, "
          f"y {uv[:, 1].min():.0f}..{uv[:, 1].max():.0f} (of 320 x 180)")
plan = ArmPlan(T.demo()["left"], station.BASES["left"])
M = arm_mask(plan.action7(plan.duration)[0][:6])          # the left arm where the demo leaves it
so = T.syringe_origin_for_grip(T.GRIP_IN_TRAY)
ax = np.r_[so, T.TOP + 0.002] + np.c_[T._PROF_X, 0 * T._PROF_X, 0 * T._PROF_X + T.SYR_AXIS_Z] @ T._yaw_matrix(T.SYR_ROT).T
for name, pts in (("her hand", T.HAND_POINTS), ("tray rim", tray), ("syringe in the tray", ax)):
    print(f"{name}: {100 * hidden(pts).mean():.0f}% hidden by the arms at the demo's end")
