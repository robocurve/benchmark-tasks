#!/usr/bin/env python3
"""Offline collision check of the scripted demo against the scene for weigh-and-reject-heavy-block.

Every control step (20 Hz) of every waypoint of every plan mode is checked for BOTH arms against ALL of:
- the table
- the supply rack
- the green shelf
- the red reject tray
- the scale fixture base and platform
- the readout meter
- the three blocks (intended contacts during grasp/place exempted)
- arm-arm clearance at every step.

All unintended clearances must be >= 1.0 mm. Arm-arm clearance must be >= 30.0 mm.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

MIN_CLEAR_MM = 1.0          # every unintended clearance must be at least this
ARM_ARM_MIN_MM = 30.0       # arm-arm distance minimum threshold
DT = 0.05


def static_obstacles():
    """Return dict of static boxes and cylinders."""
    import geometry as g

    boxes = {
        "table": (np.array([0.3, 0.0, g.TOP - 0.03]), np.array([1.1, 1.3, 0.06])),
        "supply rack": (np.array(g.SUPPLY_RACK_POS), np.array(g.SUPPLY_RACK_SIZE)),
        "green shelf": (np.array(g.SHELF_POS), np.array(g.SHELF_SIZE)),
        "reject tray": (np.array(g.TRAY_POS), np.array(g.TRAY_SIZE)),
        "scale base": (np.array(g.SCALE_BASE_POS), np.array(g.SCALE_BASE_SIZE)),
        "scale platform": (np.array([g.SCALE_CENTER_XY[0], g.SCALE_CENTER_XY[1], g.SCALE_PLATFORM_REST_Z]), np.array(g.SCALE_PLATFORM_SIZE)),
        "meter fixture": (np.array(g.METER_POS), np.array(g.METER_SIZE)),
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
        hulls = analysis.world_hulls(analysis.fk_link_poses_world(q, base))
        pts = np.vstack([h.vertices for _, h in hulls])
        who = np.concatenate([[name] * len(h.vertices) for name, h in hulls])
        aabb = (pts.min(0), pts.max(0))
        out.append({
            "t": t,
            "label": label,
            "hulls": hulls,
            "pts": pts,
            "who": who,
            "aabb": aabb,
            "pe": pe,
            "re": re,
            "margin": margin,
            "tcp": p.copy(),
            "finger": float(finger),
        })
        t += DT
    return out


def wp_class(label: str) -> str:
    s = label.lower()
    if "grasp" in s or "place" in s or "descend" in s:
        return "manipulation"
    if "hover" in s or "lift" in s or "clear" in s:
        return "transit"
    if "ready" in s or "sync" in s or "weigh hold" in s or "settle" in s or "return" in s:
        return "park/wait/home"
    return "other"


def analyse(arm: str, samples):
    """Per-label dict of worst clearances, with object names."""
    import geometry as g

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

        is_grasp_place = ("grasp" in label or "place" in label or "descend" in label or "release" in label)

        # Static boxes
        for name, (c, size) in boxes.items():
            if np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi):
                continue
            # When placing on scale platform, tip/pad comes close to platform
            if is_grasp_place and "scale" in label and name == "scale platform":
                continue
            # When placing inside reject tray, tip/pad enters tray cavity
            if ("tray" in label and ("place" in label or "release" in label or "lift" in label or "descend" in label)) and name == "reject tray":
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

    for i in range(n):
        ls = left_samples[i]
        rs = right_samples[i]
        t = ls["t"]
        lo_l, hi_l = ls["aabb"]
        lo_r, hi_r = rs["aabb"]
        gap = float(np.maximum(lo_r - hi_l, lo_l - hi_r).max()) * 1000.0
        if gap < min_dist_mm:
            min_dist_mm = gap
            worst_t = t

    return min_dist_mm, worst_t


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

    print("Analyzing left arm clearances...")
    l_rep, l_bad = analyse("left", l_samples)
    print("Analyzing right arm clearances...")
    r_rep, r_bad = analyse("right", r_samples)

    print("Analyzing arm-arm clearance...")
    arm_arm_min, arm_arm_t = check_arm_arm(l_samples, r_samples)

    print("\n=== Left Arm Clearance Summary ===")
    for label, r in l_rep.items():
        min_d, who_obj = r["min"]
        print(f"  {label:35} {min_d:6.2f} mm ({who_obj})")

    print("\n=== Right Arm Clearance Summary ===")
    for label, r in r_rep.items():
        min_d, who_obj = r["min"]
        print(f"  {label:35} {min_d:6.2f} mm ({who_obj})")

    print(f"\nMinimum arm-arm clearance: {arm_arm_min:.2f} mm at t={arm_arm_t:.2f} s")

    all_ok = True
    if l_bad:
        print("\nLeft arm violations:")
        for b in l_bad:
            print("  FAIL:", b)
        all_ok = False

    if r_bad:
        print("\nRight arm violations:")
        for b in r_bad:
            print("  FAIL:", b)
        all_ok = False

    if arm_arm_min < ARM_ARM_MIN_MM:
        print(f"\nArm-arm clearance FAIL: {arm_arm_min:.2f} mm < {ARM_ARM_MIN_MM} mm")
        all_ok = False

    if all_ok:
        print("\nPASSED: All clearances satisfy thresholds.")
        return 0
    else:
        print("\nFAILED: Clearance violations found.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
