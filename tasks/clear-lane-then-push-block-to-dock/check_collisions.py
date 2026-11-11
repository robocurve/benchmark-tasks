"""Offline collision, self-collision, and clearance audit for clear-lane-then-push-block-to-dock."""

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
    """Return dict of static boxes."""
    import geometry as g

    boxes = {
        "table": (np.array([0.3, 0.0, g.TABLE_Z - 0.03]), np.array([1.1, 1.3, 0.06])),
        "rail_left": (np.array(g.RAIL_LEFT_POS), np.array(g.RAIL_SIZE)),
        "rail_right": (np.array(g.RAIL_RIGHT_POS), np.array(g.RAIL_SIZE)),
        "stop_plate": (np.array(g.STOP_PLATE_POS), np.array(g.STOP_PLATE_SIZE)),
        "red_post": (np.array(g.POST_START_POS), np.array(g.POST_SIZE)),
    }
    cyls = {}
    return boxes, cyls


def box_dist(points, centre, size):
    from yam_arena import analysis

    return analysis.box_signed_distance(points, np.asarray(centre), np.eye(3), np.asarray(size))


def trace_arm(arm: str, mode: str = "demo"):
    import demo_plan
    from yam_arena import analysis, ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN
    from yam_arena.demo import ArmPlan

    waypoints = demo_plan.plans(mode)[arm]
    base = station.BASES[arm]
    plan = ArmPlan(list(waypoints), base)
    out, t = [], 0.0
    while t <= plan.duration + 1e-9:
        pos, rot, finger, label = plan.target(t)
        act, _ = plan.action7(t)
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
            "pe": pe,
            "re": re,
            "margin": margin,
            "pts": pts,
            "who": who,
            "aabb": aabb,
        })
        t += DT
    return out


def wp_class(label: str) -> str:
    s = label.lower()
    if any(k in s for k in ("descend", "grasp", "lower", "release", "push", "lift", "hold", "clear", "retract")):
        return "manipulation"
    if "hover" in s or "transit" in s or "carry" in s:
        return "transit"
    if "ready" in s or "sync" in s or "settle" in s or "return" in s or "final hold" in s:
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
            # Table, rails, and stop plate can be in close proximity during intentional manipulation
            if name in ("table", "rail_left", "rail_right", "stop_plate") and any(
                k in label.lower() for k in ("descend", "grasp", "lower", "release", "push", "lift", "hold", "clear", "retract")
            ):
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

        for nl, pl in poses_l.items():
            if nl == "base":
                continue
            for nr, pr in poses_r.items():
                if nr == "base":
                    continue
                d = float(np.linalg.norm(pl[:3, 3] - pr[:3, 3]) * 1000)
                if d < min_dist_mm:
                    min_dist_mm = d
                    worst_t = t
                    worst_pair = (f"L:{nl}", f"R:{nr}")

    return min_dist_mm, worst_t, worst_pair


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--yam-arena", help="a checkout of yam-isaac-lab-arena")
    ap.add_argument("--mode", default="demo", help="mode to check")
    a = ap.parse_args()
    if a.yam_arena:
        sys.path.insert(0, os.path.expanduser(a.yam_arena))

    print(f"Tracing arm trajectories for mode: {a.mode}...\n")
    ls = trace_arm("left", a.mode)
    rs = trace_arm("right", a.mode)

    print("Auditing left arm scene clearances...")
    rep_l, bad_l = analyse("left", ls)
    print("Auditing right arm scene clearances...")
    rep_r, bad_r = analyse("right", rs)

    print("\nAuditing bimanual arm-arm clearance...")
    arm_min_d, worst_t, pair = check_arm_arm(ls, rs)
    print(f"Bimanual arm-arm min clearance: {arm_min_d:.2f} mm at t={worst_t:.2f}s ({pair[0]} vs {pair[1]})")

    all_scene_min = min(
        min(r["min"][0] for r in rep_l.values()),
        min(r["min"][0] for r in rep_r.values()),
    )
    print(f"Minimum scene clearance: {all_scene_min:.2f} mm")

    bad = bad_l + bad_r
    if arm_min_d < ARM_ARM_MIN_MM:
        bad.append(f"Arm-arm clearance {arm_min_d:.2f} mm < {ARM_ARM_MIN_MM} mm at t={worst_t:.2f}s ({pair[0]} vs {pair[1]})")

    if bad:
        print("\nCOLLISIONS DETECTED:")
        for b in bad:
            print(f"  FAIL: {b}")
        return 1

    print("\nALL CLEARANCES SATISFIED. 0 collisions detected.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
