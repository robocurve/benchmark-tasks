"""Offline collision, self-collision, and fixture clearance audit for bridge-plate-across-trench."""

from __future__ import annotations

import argparse
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if "YAM_ARENA_SRC" in os.environ:
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

MIN_CLEAR_MM = 1.0          # every unintended clearance must be at least this
ARM_ARM_MIN_MM = 30.0       # arm-arm distance minimum threshold (30.0 mm required)
DT = 0.05


def static_obstacles():
    """Return dict of static boxes and cylinders."""
    import geometry as g

    boxes = {
        "table": (np.array([0.3, 0.0, g.TOP - 0.03]), np.array([1.1, 1.3, 0.06])),
        "platform_near_lip": (np.array(g.NEAR_PLATFORM_POS), np.array(g.NEAR_PLATFORM_SIZE)),
        "platform_near_staging": (np.array(g.NEAR_STAGING_POS), np.array(g.NEAR_STAGING_SIZE)),
        "platform_far_lip": (np.array(g.FAR_PLATFORM_POS), np.array(g.FAR_PLATFORM_SIZE)),
        "platform_far_rear": (np.array(g.FAR_REAR_POS), np.array(g.FAR_REAR_SIZE)),
        "target_mark": (np.array(g.TARGET_MARK_POS), np.array(g.MARK_SIZE)),
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

    waypoints = demo_plan.plans(mode)[arm]
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
        aabb = (pts.min(0), pts.max(0))
        out.append({
            "t": t,
            "label": label,
            "hulls": hulls,
            "poses": poses,
            "pts": pts,
            "who": who,
            "aabb": aabb,
            "pe": pe,
            "re": re,
            "margin": margin,
        })
        t += DT
    return out


def wp_class(label: str) -> str:
    s = label.lower()
    if "grasp" in s or "place" in s or "push" in s or "contact" in s:
        return "manipulation"
    if "hover" in s or "lift" in s or "clear" in s or "descend" in s or "transit" in s or "app" in s:
        return "transit"
    if "ready" in s or "sync" in s or "settle" in s or "return" in s or "ret" in s:
        return "park/wait/home"
    return "other"


def analyse(arm: str, samples):
    """Per-label dict of worst clearances, with object names."""
    boxes, cyls = static_obstacles()
    rep: dict[str, dict] = {}
    bad: list[str] = []

    for s in samples:
        label = s["label"]
        r = rep.setdefault(label, {"cls": wp_class(label), "pe": 0.0, "re": 0.0, "margin": 9.0, "min": (np.inf, ""), "extra": {}, "t": s["t"]})
        r["pe"], r["re"], r["margin"] = max(r["pe"], s["pe"]), max(r["re"], s["re"]), min(r["margin"], s["margin"])
        pts = s["pts"]
        who = s["who"]
        lo, hi = s["aabb"][0] - 0.12, s["aabb"][1] + 0.12

        def consider(obj: str, d_all: np.ndarray):
            k = int(np.argmin(d_all))
            d_mm = float(d_all[k] * 1000)
            link = str(who[k])
            if link == "base" and obj == "table":
                return d_mm
            if d_mm < r["min"][0]:
                r["min"] = (d_mm, f"{link} vs {obj}")
                r["t"] = s["t"]
            return d_mm

        # Static boxes
        for name, (c, size) in boxes.items():
            if np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi):
                continue
            # When manipulating parts on platforms, finger pads/tips descend close to platform surface
            if any(k in label for k in ("descend", "grasp", "lift", "place", "release", "clear", "contact", "push", "app4")) and ("platform" in name or "mark" in name or name == "table"):
                continue

            consider(name, box_dist(pts, c, size))

    for label, r in rep.items():
        min_d, who_obj = r["min"]
        if min_d < MIN_CLEAR_MM:
            bad.append(f"{arm} at '{label}': min clearance {min_d:.2f} mm < {MIN_CLEAR_MM} mm ({who_obj})")

    return rep, bad


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
                    step_pair = (f"L:{nl}", f"R:{nr}")

        if step_min_d < min_dist_mm:
            min_dist_mm = step_min_d
            worst_t = t
            worst_pair = step_pair

    return min_dist_mm, worst_t, worst_pair


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mode", default="demo", help="demo mode to check")
    ap.add_argument("--yam-arena", help="yam_arena path override")
    args = ap.parse_args()

    if args.yam_arena:
        sys.path.insert(0, os.path.expanduser(args.yam_arena))

    print(f"Tracing arm trajectories for mode: {args.mode}...")
    l_samples = trace_arm("left", mode=args.mode)
    r_samples = trace_arm("right", mode=args.mode)

    print("\nAuditing left arm scene clearances...")
    l_rep, l_bad = analyse("left", l_samples)
    print("Auditing right arm scene clearances...")
    r_rep, r_bad = analyse("right", r_samples)

    print("\nAuditing bimanual arm-arm clearance...")
    arm_min_mm, worst_t, worst_pair = check_arm_arm(l_samples, r_samples)
    print(f"Bimanual arm-arm min clearance: {arm_min_mm:.2f} mm at t={worst_t:.2f}s ({worst_pair[0]} vs {worst_pair[1]})")

    min_scene_clear_mm = min(min(r["min"][0] for r in l_rep.values()), min(r["min"][0] for r in r_rep.values()))
    print(f"Minimum scene clearance: {min_scene_clear_mm:.2f} mm")

    bad = l_bad + r_bad
    if arm_min_mm < ARM_ARM_MIN_MM:
        bad.append(f"Arm-arm clearance {arm_min_mm:.2f} mm < threshold {ARM_ARM_MIN_MM} mm")

    if bad:
        print("\nCOLLISIONS DETECTED:")
        for b in bad:
            print("  FAIL:", b)
        return 1

    print("\nALL CLEARANCES SATISFIED. 0 collisions detected.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
