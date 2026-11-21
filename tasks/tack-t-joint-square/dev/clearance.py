"""Dev only, no simulator: play the demo's plan with perfect tracking and report how close each arm's collision hulls
come to the torch, the coupon, the bench and the other arm. python dev/clearance.py [step_s]"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import task as t  # noqa: E402
from tack_t_joint import mech as M  # noqa: E402
from yam_arena import analysis as A  # noqa: E402
from yam_arena.constants import ARM_JOINTS  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.station import BASES  # noqa: E402

STEP = float(sys.argv[1]) if len(sys.argv) > 1 else 0.1


def box_points(lo, hi, n=5):
    g = [np.linspace(lo[i], hi[i], n) for i in range(3)]
    pts = np.array([(x, y, z) for x in g[0] for y in g[1] for z in g[2]])
    edge = (np.isclose(pts, lo) | np.isclose(pts, hi)).any(axis=1)
    return pts[edge]


def tilted_box(a0, a1, half):
    """Points of a torch box along the head's axis (the head, cup, tungsten colliders), torch frame."""
    c = M.TIP + (a0 + a1) / 2 * M.UP
    pts = box_points((-half, -half, -(a1 - a0) / 2), (half, half, (a1 - a0) / 2))
    ca, sa = math.cos(-M.TILT), math.sin(-M.TILT)
    Ry = np.array(((ca, 0, sa), (0, 1, 0), (-sa, 0, ca)))
    return pts @ Ry.T + c


TORCH_PTS = {n: box_points(*b) for n, b in M.TORCH_BOXES.items()}
for _n, (_a0, _a1, _half) in M.AXIS_BOXES.items():
    TORCH_PTS[_n] = tilted_box(_a0, _a1, _half)
COUPON_PTS = box_points(-M.COUPON_HALF, M.COUPON_HALF, 7)
Z = t.TOP
FIXED = {
    "table": ((-0.3, -0.7, Z - 0.05), (0.9, 0.7, Z)),
    "slab": ((M.SLAB["x"][0], M.SLAB["y"][0], Z), (M.SLAB["x"][1], M.SLAB["y"][1], Z + M.SLAB["t"])),
    "base": ((M.BASE["x"][0], M.BASE["y"][0], Z), (M.BASE["x"][1], M.BASE["y"][1], Z + M.BT)),
    "rest block": ((M.REST["x"] - M.REST["sx"] / 2, M.REST["y"] - M.REST["sy"] / 2, Z), (M.REST["x"] + M.REST["sx"] / 2, M.REST["y"] + M.REST["sy"] / 2, Z + M.REST["h"])),
    "clamp0": ((0.149, M.CLAMPS[0][1] - 0.010, Z), (0.213, M.CLAMPS[0][1] + 0.010, Z + M.BT + 0.014)),
    "clamp1": ((0.327, M.CLAMPS[1][1] - 0.010, Z), (0.391, M.CLAMPS[1][1] + 0.010, Z + M.BT + 0.014)),
    "torch posts": ((0.125, 0.185, Z), (0.255, 0.235, Z + M.TORCH_POST_H)),
}


def to_hulls(points, hulls, skip=()):
    best, who = np.inf, None
    for name, h in hulls:
        if name in skip:
            continue
        s = float(A._signed(points, h).min())
        if s < best:
            best, who = s, name
    return best, who


def main():
    t.HOLD_CAL.update(x=0.0, y=0.0, z=0.0, tilt=0.0, yaw=0.0)  # perfect tracking: no give to aim against
    t.TORCH_CAL.update({i: (0.0, 0.0, 0.0) for i in range(3)})
    wps = t.demo()
    plans = {a: ArmPlan(wps[a], BASES[a]) for a in wps}
    T = max(p.duration for p in plans.values())
    pose = {"coupon": (np.array((M.COUPON_START[0], M.COUPON_START[1], t.TOP + M.COUPON_START[2])), np.eye(3)),
            "torch": (np.array((M.TORCH_REST["x"], M.TORCH_REST["y"], t.TOP + M.TORCH_REST_Z)), t._rz(M.TORCH_REST["lean"]))}
    hand = {"coupon": "right", "torch": "left"}
    closed_at = {"coupon": t._fingers(0.074), "torch": t.TORCH_OPEN / 2}
    grasp, worst = {}, {}

    def note(key, d, who, tt, label):
        if key not in worst or d < worst[key][0]:
            worst[key] = (d, who, tt, label)

    for k in range(int(round(T / STEP)) + 1):
        tt = k * STEP
        hulls, tgt, labels = {}, {}, {}
        for a, pl in plans.items():
            pos, R, finger, _ = pl.target(tt)
            q, label = pl.action7(tt)
            joints = {n: float(v) for n, v in zip(ARM_JOINTS, q[:6])}
            joints.update(joint7=finger, joint8=finger)
            hulls[a] = A.world_hulls(A.fk_link_poses_world(joints, BASES[a]))
            tgt[a], labels[a] = (pos, R, finger), label
        for part, a in hand.items():
            pos, R, finger = tgt[a]
            p0, R0 = pose[part]
            if finger > closed_at[part] and part not in grasp and np.linalg.norm(pos - p0) < 0.012:
                grasp[part] = (R.T @ (p0 - pos), R.T @ R0)
            if finger <= closed_at[part]:
                grasp.pop(part, None)
            if part in grasp:
                pose[part] = (pos + R @ grasp[part][0], R @ grasp[part][1])
        tp, tR = pose["torch"]
        cp, cR = pose["coupon"]
        fingers = ("tip_left", "tip_right", "gripper")
        for n, pts in TORCH_PTS.items():
            w = pts @ tR.T + tp
            for a in plans:
                skip = fingers if a == "left" and (n == "grip" or "torch" in grasp or np.linalg.norm(tgt["left"][0] - tp) < 0.06) else ()
                d, who = to_hulls(w, hulls[a], skip)
                note(f"{a} arm (not its fingers when holding) - torch {n}", d, who, tt, labels[a])
            if "torch" in grasp:
                for name, (lo, hi) in FIXED.items():
                    d = float(A.box_signed_distance(w, (np.array(lo) + hi) / 2, np.eye(3), np.array(hi) - lo).min())
                    note(f"held torch {n} - {name}", d, name, tt, labels["left"])
                d = float(A.box_signed_distance(w, cp, cR, 2 * M.COUPON_HALF).min())
                note(f"held torch {n} - coupon", d, "coupon", tt, labels["left"])
        w = COUPON_PTS @ cR.T + cp
        for a in plans:
            near = a == "right" and np.linalg.norm(tgt["right"][0] - cp) < 0.09
            d, who = to_hulls(w, hulls[a], ("tip_left", "tip_right") if near else ())
            note(f"{a} arm{' (not its fingers)' if a == 'right' else ''} - coupon", d, who, tt, labels[a])
        if "coupon" in grasp:
            for name, (lo, hi) in FIXED.items():
                if name == "rest block" and tt < 9.0:
                    continue
                d = float(A.box_signed_distance(w, (np.array(lo) + hi) / 2, np.eye(3), np.array(hi) - lo).min())
                note(f"held coupon - {name}", d, name, tt, labels["right"])
        for a in plans:
            for name, (lo, hi) in FIXED.items():
                d, who = A.hulls_to_box(hulls[a], (np.array(lo) + hi) / 2, np.eye(3), np.array(hi) - lo)
                note(f"{a} arm - {name}", d, who, tt, labels[a])
        d, pair = A.hull_set_distance(hulls["left"], hulls["right"])
        if pair:
            note("left arm - right arm", d, pair, tt, labels["left"])
    for key in sorted(worst, key=lambda k: worst[k][0]):
        d, who, tt, label = worst[key]
        if d < 0.02:
            print(f"{1000 * d:7.1f} mm  {key}  [{who}] at t={tt:.1f} ({label})")


main()
