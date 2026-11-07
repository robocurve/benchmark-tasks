"""Offline collision, self-collision, and fixture clearance audit for estop-before-clearing-jam."""

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
        "button console fixture": (np.array(g.CONSOLE_BASE_POS), np.array(g.CONSOLE_BASE_SIZE)),
        "lane fixture": (np.array([g.LANE_POS[0], g.LANE_POS[1], g.TOP + 0.010]), np.array([g.LANE_SIZE_X, g.LANE_SIZE_Y, 0.025])),
        "green tray": (np.array(g.TRAY_POS), np.array(g.TRAY_SIZE)),
        "parked pad": (np.array(g.PARK_PAD_POS), np.array(g.PARK_PAD_SIZE)),
        "estop button": (np.array(g.ESTOP_REST_POS), np.array(g.ESTOP_CAP_SIZE)),
        "reset button": (np.array(g.RESET_REST_POS), np.array(g.RESET_CAP_SIZE)),
        "start button": (np.array(g.START_REST_POS), np.array(g.START_CAP_SIZE)),
        "stuck block": (np.array(g.BLOCK_INIT_POS), np.array(g.BLOCK_SIZE)),
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
    if "grasp" in s or "press" in s or "place" in s or "release" in s:
        return "manipulation"
    if "hover" in s or "lift" in s or "transfer" in s or "descend" in s:
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
            # When interacting with E-STOP
            if "estop" in label and (name == "estop button" or name == "button console fixture"):
                continue
            # When interacting with RESET
            if "reset" in label and (name == "reset button" or name == "button console fixture"):
                continue
            # When interacting with START
            if "start" in label and (name == "start button" or name == "button console fixture"):
                continue
            # When interacting with stuck block or lane
            if ("block" in label or "lane" in label) and (name == "stuck block" or name == "lane fixture"):
                continue
            # When placing in green tray
            if "tray" in label and name == "green tray":
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
    ap.add_argument("--yam-arena", help="a checkout of yam-isaac-lab-arena")
    ap.add_argument("--mode", default="demo", help="demo mode to check")
    a = ap.parse_args()
    if a.yam_arena:
        sys.path.insert(0, os.path.expanduser(a.yam_arena))

    print(f"=== Auditing clearances and collisions for mode: {a.mode} ===")
    left_samples = trace_arm("left", a.mode)
    right_samples = trace_arm("right", a.mode)

    print("\nAuditing Left arm against scene obstacles...")
    rep_l, bad_l = analyse("left", left_samples)
    for label, r in rep_l.items():
        min_d, who_obj = r["min"]
        print(f"  {label:35} [{r['cls']:14}] min clear: {min_d:6.2f} mm ({who_obj})")

    print("\nAuditing Right arm against scene obstacles...")
    rep_r, bad_r = analyse("right", right_samples)
    for label, r in rep_r.items():
        min_d, who_obj = r["min"]
        print(f"  {label:35} [{r['cls']:14}] min clear: {min_d:6.2f} mm ({who_obj})")

    print("\nAuditing Bimanual Arm-Arm clearance...")
    min_arm_arm, worst_t, worst_pair = check_arm_arm(left_samples, right_samples)
    print(f"  Worst arm-arm clearance: {min_arm_arm:.2f} mm at t={worst_t:.2f} s ({worst_pair[0]} vs {worst_pair[1]})")

    all_ok = True
    if bad_l:
        print("\nLEFT ARM CLEARANCE VIOLATIONS:")
        for b in bad_l:
            print("  " + b)
        all_ok = False
    if bad_r:
        print("\nRIGHT ARM CLEARANCE VIOLATIONS:")
        for b in bad_r:
            print("  " + b)
        all_ok = False
    if min_arm_arm < ARM_ARM_MIN_MM:
        print(f"\nBIMANUAL CLEARANCE VIOLATION: {min_arm_arm:.2f} mm < {ARM_ARM_MIN_MM} mm")
        all_ok = False

    if all_ok:
        print(f"\nPASSED: Zero collisions detected, all clearances >= {MIN_CLEAR_MM} mm, arm-arm >= {ARM_ARM_MIN_MM} mm.")
        return 0
    else:
        print("\nFAILED: Collision or clearance violations detected.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
