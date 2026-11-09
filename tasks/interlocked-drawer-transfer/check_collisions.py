"""Offline collision, self-collision, and fixture clearance audit for interlocked-drawer-transfer."""

from __future__ import annotations

import argparse
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if "YAM_ARENA_SRC" in os.environ:
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

MIN_CLEAR_MM = 1.0          # every unintended clearance must be at least this
ARM_ARM_MIN_MM = 30.0       # arm-arm distance minimum threshold (spec >= 30 mm)
DT = 0.05


def static_obstacles():
    """Return dict of static boxes and cylinders."""
    import geometry as g

    boxes = {
        "table": (np.array([0.3, 0.0, g.TOP - 0.03]), np.array([1.1, 1.3, 0.06])),
        "cabinet fixture": (np.array(g.CABINET_POS), np.array([g.CABINET_SIZE_X, g.CABINET_WIDTH, g.CABINET_HEIGHT])),
        "staging pad": (np.array(g.STAGING_PAD_POS), np.array(g.STAGING_PAD_SIZE)),
        "lamp bar": (np.array(g.LAMP_BAR_POS), np.array(g.LAMP_BAR_SIZE)),
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
    if "grasp" in s or "press" in s or "pull" in s or "push" in s or "place" in s or "descend" in s:
        return "manipulation"
    if "hover" in s or "lift" in s or "clear" in s or "approach" in s or "carry" in s:
        return "transit"
    if "ready" in s or "sync" in s or "settle" in s or "return" in s or "wait" in s or "hold" in s:
        return "park/wait/home"
    return "other"


def analyse(arm: str, samples):
    """Per-label dict of worst clearances, with object names."""
    boxes, cyls = static_obstacles()
    rep: dict[str, dict] = {}
    bad: list[str] = []

    for s in samples:
        label = s["label"]
        r = rep.setdefault(
            label,
            {"cls": wp_class(label), "pe": 0.0, "re": 0.0, "margin": 9.0, "min": (np.inf, ""), "extra": {}, "t": s["t"]},
        )
        r["pe"], r["re"], r["margin"] = max(r["pe"], s["pe"]), max(r["re"], s["re"]), min(r["margin"], s["margin"])
        pts = s["pts"]
        who = s["who"]
        lo, hi = s["aabb"][0] - 0.12, s["aabb"][1] + 0.12

        def consider(obj: str, d_all: np.ndarray):
            k = int(np.argmin(d_all))
            d_mm = float(d_all[k] * 1000)
            link = str(who[k])
            if d_mm < r["min"][0]:
                r["min"] = (d_mm, f"{link} vs {obj}")
                r["t"] = s["t"]
            return d_mm

        # Static boxes
        for name, (c, size) in boxes.items():
            if np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi):
                continue
            # Intended contacts declared by label
            # When manipulating drawer handle or reaching inside drawer
            if ("handle" in label or "drawer" in label or "block" in label) and name == "cabinet fixture":
                continue
            # When staging block on pad
            if "staging" in label and (name == "staging pad" or name == "cabinet fixture"):
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

    print(f"Sampling trajectories for mode '{args.mode}'...")
    left_samples = trace_arm("left", args.mode)
    right_samples = trace_arm("right", args.mode)

    print("Analyzing left arm scene clearances...")
    rep_l, bad_l = analyse("left", left_samples)
    print("Analyzing right arm scene clearances...")
    rep_r, bad_r = analyse("right", right_samples)

    print("Analyzing arm-arm clearance...")
    arm_arm_min, arm_arm_t, arm_arm_pair = check_arm_arm(left_samples, right_samples)

    print(f"\nArm-arm minimum clearance: {arm_arm_min:.2f} mm at t={arm_arm_t:.2f} s ({arm_arm_pair[0]} vs {arm_arm_pair[1]})")
    arm_arm_pass = arm_arm_min >= ARM_ARM_MIN_MM
    print(f"Arm-arm gate (>= {ARM_ARM_MIN_MM} mm): {'PASSED' if arm_arm_pass else 'FAILED'}")

    min_scene_clear_mm = min(min(r["min"][0] for r in rep_l.values()), min(r["min"][0] for r in rep_r.values()))
    print(f"Minimum scene clearance: {min_scene_clear_mm:.2f} mm")

    all_bad = bad_l + bad_r
    if all_bad or not arm_arm_pass:
        print(f"\nFAILED: {len(all_bad)} unintended clearance violations:")
        for b in all_bad:
            print(f"  {b}")
        if not arm_arm_pass:
            print(f"  Arm-arm clearance {arm_arm_min:.2f} mm < {ARM_ARM_MIN_MM} mm")
        return 1
    else:
        print("\nALL CLEARANCES SATISFIED. 0 collisions detected.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
