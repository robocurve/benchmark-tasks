"""Reach check (no simulator): IK, joint-limit margins and clearances along the demo's paths for all three layouts.

    python reach_check.py            # prints a table and exits 1 if any limit fails

Limits: IK position error <= 1 mm, rotation error <= 1 deg, every joint >= 5 deg inside its limits, and >= 5 mm
clearance from the table, the device (base plate, heel cup, guide) and the form, for every arm link; the
gripper's two finger links are excluded only against the part they hold (post, pointer fin, bar fin). The
arm holding the post is also checked against the other arm while that arm works the sliders.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import footform as F  # noqa: E402
import layout as LY  # noqa: E402
import plan  # noqa: E402
from sizing import DEVICE as D  # noqa: E402
from yam_arena import analysis, ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS  # noqa: E402

FINGERS = ("tip_left", "tip_right")
STEP = 0.005
TABLE = (np.array(station.TABLE_CENTER), np.eye(3), np.array(station.TABLE_SIZE))


def dev_box(u0, u1, v0, v1, z0, z1):
    a, b = LY.to_world(u0, v0, z0), LY.to_world(u1, v1, z1)
    return ((a + b) / 2, np.eye(3), np.abs(b - a))


def device_boxes():
    bu, bv, t = D["base_u"], D["base_v"], D["base_t"]
    r, w, h = D["cup_r"], D["cup_wall"], D["cup_h"]
    return {
        "base": dev_box(bu[0], bu[1], bv[0], bv[1], -t, 0.0),
        "cup": dev_box(-w, D["cup_side_u"], -r - w, r + w, 0.0, h),  # the cup's outer box (conservative)
        "guide": dev_box(D["guide_u"][0], D["guide_u"][1], D["guide_v"], D["guide_v"] + D["guide_w"], 0.0,
                         D["guide_h"]),
    }


def form_hulls(k: int, seated: bool) -> list:
    form, pos, q = LY.form_start(k)
    fo = F.Form(form["L"], form["A"], form["W"])
    psi = math.radians(LY.LAYOUTS[k]["yaw"]) if not seated else 0.0
    th = -math.pi / 2 + psi
    R = np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1]])
    p = LY.to_world(0, 0, 0) if seated else pos
    out = []
    for hv in fo.hulls():
        out.append(R @ (hv.T / 1000.0) + p[:, None])
    return [h.T for h in out]


def pts_clearance(hulls_arm, point_sets) -> float:
    """Min distance (m) from arm hull vertices to convex point sets (approx: vertex-to-hull via planes)."""
    from scipy.spatial import ConvexHull

    best = np.inf
    for P in point_sets:
        hull = ConvexHull(P)
        n, d = hull.equations[:, :3], hull.equations[:, 3]
        lo, hi = P.min(0) - 0.05, P.max(0) + 0.05
        for name, h in hulls_arm:
            v = h.vertices
            m = np.all((v > lo) & (v < hi), axis=1)
            if not m.any():
                continue
            s = (v[m] @ n.T + d).max(axis=1)  # signed distance lower bound (exact inside, approx outside)
            best = min(best, float(s.min()))
            s2 = min((P @ h.planes[:, :3].T - h.planes[:, 3]).max(axis=1))
            best = min(best, float(s2))
    return best


def run_path(side, poses, k, holding=None, other=None):
    base = np.array(station.BASES[side])
    q = None
    worst = dict(ik_mm=0.0, ik_deg=0.0, margin_deg=180.0, table=np.inf, device=np.inf, form=np.inf, arm=np.inf)
    lower, upper = ik.LOWER, ik.UPPER
    boxes = device_boxes()
    qs = []
    for (p0, y0, t0), (p1, y1, t1) in zip(poses[:-1], poses[1:]):
        n = max(1, int(np.linalg.norm(np.subtract(p1, p0)) / STEP))
        for s in np.linspace(0, 1, n + 1):
            p = np.asarray(p0) + s * (np.asarray(p1) - np.asarray(p0))
            yaw = y0 + s * (y1 - y0)
            rot = ik.matrix_to_quat_xyzw(plan.grip_matrix(yaw, t0 + s * (t1 - t0)))
            q, pe, re = ik.solve(p, rot, q, base_pos=base, iters=200)
            qs.append(q)
            worst["ik_mm"] = max(worst["ik_mm"], pe * 1000)
            worst["ik_deg"] = max(worst["ik_deg"], math.degrees(re))
            worst["margin_deg"] = min(worst["margin_deg"], math.degrees(min((q - lower).min(), (upper - q).min())))
            hulls = [h for h in analysis.world_hulls(analysis.fk_link_poses_world(dict(zip(ARM_JOINTS, q)), base))
                     if h[0] != "base"]  # the arm's own fixed base stands on the table
            arm_only = [h for h in hulls if h[0] not in FINGERS]
            worst["table"] = min(worst["table"], analysis.hulls_to_box(hulls, *TABLE)[0])
            for name, box in boxes.items():
                worst["device"] = min(worst["device"], analysis.hulls_to_box(hulls if holding is None or
                                                                            holding != "base" else arm_only, *box)[0])
            fh = form_hulls(k, seated=True)
            worst["form"] = min(worst["form"], pts_clearance(arm_only if holding == "post" else hulls, fh))
            if other is not None:
                worst["arm"] = min(worst["arm"], analysis.hull_set_distance(hulls, other)[0])
    return worst, qs


def main() -> int:
    ok = True
    rows = []
    for k in range(3):
        P = plan.key_poses(k)
        wl, ql = run_path("left", P["left"], k, holding="post")
        base_l = np.array(station.BASES["left"])
        hp, hy, ht = P["left"][2]
        qh, _, _ = ik.solve(hp, ik.matrix_to_quat_xyzw(plan.grip_matrix(hy, ht)), ql[-1], base_pos=base_l, iters=300)
        hold_hulls = [h for h in analysis.world_hulls(analysis.fk_link_poses_world(dict(zip(ARM_JOINTS, qh)), base_l))
                      if h[0] != "base"]
        wr, _ = run_path("right", P["right"], k, other=hold_hulls)
        for side, w in (("left", wl), ("right", wr)):
            good = (w["ik_mm"] <= 1.0 and w["ik_deg"] <= 1.0 and w["margin_deg"] >= 5.0 and w["table"] >= 0.005
                    and w["device"] >= 0.005 and w["form"] >= 0.005 and w["arm"] >= 0.005)
            ok &= good
            rows.append((k, side, w, good))
    print("layout arm   IK_mm  IK_deg  margin_deg  table_mm  device_mm  form_mm  other_arm_mm  pass")
    for k, side, w, good in rows:
        print(f"{k:6d} {side:5s} {w['ik_mm']:6.2f} {w['ik_deg']:7.2f} {w['margin_deg']:11.1f} {w['table']*1000:9.1f}"
              f" {w['device']*1000:10.1f} {w['form']*1000:8.1f} {w['arm']*1000:13.1f}  {'PASS' if good else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
