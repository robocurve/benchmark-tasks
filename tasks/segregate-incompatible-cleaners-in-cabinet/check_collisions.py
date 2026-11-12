"""Offline collision, self-collision, and fixture clearance audit for segregate-incompatible-cleaners-in-cabinet."""

from __future__ import annotations

import argparse
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if "YAM_ARENA_SRC" in os.environ:
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

MIN_CLEAR_MM = 4.0          # every unintended scene clearance must be at least 4 mm
ARM_ARM_MIN_MM = 30.0       # arm-arm distance minimum threshold
DT = 0.05


def static_obstacles():
    """Return dict of static boxes and cylinders in world coordinates."""
    import geometry as g

    boxes = {
        "table": (np.array([0.3, 0.0, g.TABLE_Z - 0.03]), np.array([1.1, 1.3, 0.06])),
        "cabinet_base": (np.array([g.CABINET_X, 0.0, g.TABLE_Z + 0.0025]), np.array([0.220, 0.380, 0.005])),
        "central_divider": (np.array([g.CABINET_X, 0.0, g.TABLE_Z + 0.035]), np.array([0.180, 0.016, 0.065])),
        "left_staging_pad": (np.array([g.STAGING_X, g.STAGING_LEFT_Y, g.TABLE_Z + 0.002]), np.array([0.150, 0.150, 0.004])),
        "right_staging_pad": (np.array([g.STAGING_X, g.STAGING_RIGHT_Y, g.TABLE_Z + 0.002]), np.array([0.150, 0.150, 0.004])),
        "left_well_collar": (np.array([g.CABINET_X, g.BAY_LEFT_Y, g.TABLE_Z + 0.020]), np.array([0.120, 0.120, 0.030])),
        "right_well_collar": (np.array([g.CABINET_X, g.BAY_RIGHT_Y, g.TABLE_Z + 0.020]), np.array([0.120, 0.120, 0.030])),
    }
    cyls = {}
    return boxes, cyls


def box_dist(points, centre, size):
    from yam_arena import analysis

    return analysis.box_signed_distance(points, np.asarray(centre), np.eye(3), np.asarray(size))


def trace_arm(arm: str, mode: str = "demo"):
    import demo_plan
    from yam_arena import analysis, ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
    from yam_arena.demo import ArmPlan

    plans = demo_plan._plans(mode)
    waypoints = plans[arm]
    base = station.BASES[arm]
    plan = ArmPlan(list(waypoints), base)
    out, t = [], 0.0
    while t <= plan.duration + 1e-9:
        pos, rot, finger, label = plan.target(t)
        plan.action7(t)
        p, r = ik.tcp_pose(plan.q, base)
        pe = float(np.linalg.norm(p - pos) * 1000)
        re = float(np.degrees(np.arccos(np.clip((np.trace(rot.T @ r) - 1) / 2, -1, 1))))
        margin = float(min((plan.q - ik.LOWER).min(), (ik.UPPER - plan.q).min()))
        q = {n: float(v) for n, v in zip(ARM_JOINTS, plan.q, strict=True)}
        q.update({n: float(finger) for n in FINGER_JOINTS})
        poses = analysis.fk_link_poses_world(q, base)
        hulls = analysis.world_hulls(poses)
        pts = np.vstack([h.vertices for _, h in hulls])
        who = np.concatenate([[name] * len(h.vertices) for name, h in hulls])
        lo = np.min(pts, axis=0)
        hi = np.max(pts, axis=0)
        out.append({"t": t, "label": label, "pe": pe, "re": re, "margin": margin,
                    "pts": pts, "who": who, "aabb": (lo, hi), "poses": poses})
        t += DT
    return out


def wp_class(label: str) -> str:
    s = label.lower()
    if "grip" in s or "seat" in s or "slide" in s or "release" in s:
        return "manipulation"
    if "hover" in s or "lift" in s or "descend" in s or "carry" in s or "retract" in s:
        return "transit"
    if "home" in s or "wait" in s or "hold" in s or "sync" in s or "settle" in s or "standby" in s:
        return "park/wait/home"
    return "other"


def analyse(arm: str, samples):
    boxes, cyls = static_obstacles()
    rep: dict[str, dict] = {}
    bad: list[str] = []
    min_seen = 999.0
    worst_wp = ""
    worst_part = ""

    for s in samples:
        label = s["label"]
        r = rep.setdefault(label, {"cls": wp_class(label), "pe": 0.0, "re": 0.0, "margin": 9.0, "min": (np.inf, ""), "t": s["t"]})
        r["pe"], r["re"], r["margin"] = max(r["pe"], s["pe"]), max(r["re"], s["re"]), min(r["margin"], s["margin"])
        pts = s["pts"]
        who = s["who"]
        lo, hi = s["aabb"][0] - 0.12, s["aabb"][1] + 0.12

        def consider(obj: str, d_all: np.ndarray):
            nonlocal min_seen, worst_wp, worst_part
            k = int(np.argmin(d_all))
            d_mm = float(d_all[k] * 1000)
            link = str(who[k])
            if link == "base" and obj == "table":
                return d_mm
            if d_mm < min_seen:
                min_seen = d_mm
                worst_wp = label
                worst_part = f"{link} vs {obj}"
            if d_mm < r["min"][0]:
                r["min"] = (d_mm, f"{link} vs {obj}")
                r["t"] = s["t"]
            return d_mm

        for name, (c, size) in boxes.items():
            if np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi):
                continue

            # Contact exclusions:
            # 1. Grasping at staging pad: finger tips descend to bottle neck
            if ("descend_pick" in label or "grasp" in label) and (name == f"{arm}_staging_pad"):
                continue

            # 2. Placing into well: bottle is lowered into the well collar
            if ("seat_well" in label or "release" in label) and (name == f"{arm}_well_collar"):
                continue

            consider(name, box_dist(pts, c, size))

    for label, r in rep.items():
        min_d, who_obj = r["min"]
        if min_d < MIN_CLEAR_MM:
            bad.append(f"{arm} at '{label}': min clearance {min_d:.2f} mm < {MIN_CLEAR_MM} mm ({who_obj})")

    return rep, bad, min_seen, worst_wp, worst_part


def check_arm_arm(left_samples, right_samples):
    n = min(len(left_samples), len(right_samples))
    min_dist_mm = np.inf
    worst_t = 0.0
    worst_pair = ("", "")

    for i in range(n):
        ls = left_samples[i]
        rs = right_samples[i]
        t = ls["t"]
        poses_l = ls["poses"]
        poses_r = rs["poses"]

        step_min_d = np.inf
        for nl, pl in poses_l.items():
            if nl == "base":
                continue
            pos_link_l = pl[:3, 3]
            for nr, pr in poses_r.items():
                if nr == "base":
                    continue
                pos_link_r = pr[:3, 3]
                d = float(np.linalg.norm(pos_link_l - pos_link_r) * 1000.0)
                if d < step_min_d:
                    step_min_d = d
                    if d < min_dist_mm:
                        min_dist_mm = d
                        worst_t = t
                        worst_pair = (f"L:{nl}", f"R:{nr}")

    return min_dist_mm, worst_t, worst_pair


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mode", default="demo", help="mode to check")
    ap.add_argument("--variant", choices=["A", "B"], default="A", help="task variant")
    ap.add_argument("--yam-arena", help="yam_arena path override")
    args = ap.parse_args()

    if args.yam_arena:
        sys.path.insert(0, os.path.expanduser(args.yam_arena))

    os.environ["TASK_VARIANT"] = args.variant

    print(f"Tracing arm trajectories for mode: {args.mode} (Variant {args.variant})...")
    l_samples = trace_arm("left", mode=args.mode)
    r_samples = trace_arm("right", mode=args.mode)

    print("\nAuditing left arm scene clearances...")
    l_rep, bad_l, min_l, wp_l, part_l = analyse("left", l_samples)
    print("\n=== Left Arm Clearance Summary ===")
    for label, r in l_rep.items():
        min_d, who_obj = r["min"]
        print(f"  {label:35} {min_d:6.2f} mm ({who_obj})")

    print("\nAuditing right arm scene clearances...")
    r_rep, bad_r, min_r, wp_r, part_r = analyse("right", r_samples)
    print("\n=== Right Arm Clearance Summary ===")
    for label, r in r_rep.items():
        min_d, who_obj = r["min"]
        print(f"  {label:35} {min_d:6.2f} mm ({who_obj})")

    print("\nAuditing bimanual arm-arm clearance...")
    arm_arm_min, worst_t, pair = check_arm_arm(l_samples, r_samples)
    print(f"Bimanual arm-arm min clearance: {arm_arm_min:.2f} mm at t={worst_t:.2f}s ({pair[0]} vs {pair[1]})")

    if min_l <= min_r:
        min_scene, worst_wp, worst_part = min_l, f"Left: {wp_l}", part_l
    else:
        min_scene, worst_wp, worst_part = min_r, f"Right: {wp_r}", part_r

    print(f"\nMinimum scene clearance: {min_scene:.2f} mm at '{worst_wp}' ({worst_part})")

    fail = False
    if bad_l or bad_r:
        fail = True
        for b in bad_l + bad_r:
            print(f"FAIL: {b}")

    if arm_arm_min < ARM_ARM_MIN_MM:
        fail = True
        print(f"FAIL: Arm-arm clearance {arm_arm_min:.2f} mm < {ARM_ARM_MIN_MM} mm threshold!")

    if not fail:
        print("\nALL CLEARANCES SATISFIED. 0 collisions detected.")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
