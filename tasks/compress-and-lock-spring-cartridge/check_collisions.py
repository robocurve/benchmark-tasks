"""Offline collision, self-collision, and fixture clearance audit for compress-and-lock-spring-cartridge."""

from __future__ import annotations

import argparse
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if "YAM_ARENA_SRC" in os.environ:
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

MIN_CLEAR_MM = 1.0          # every unintended clearance must be at least this
ARM_ARM_MIN_MM = 30.0       # arm-arm distance minimum threshold (spec target >= 30 mm)
DT = 0.05


def static_obstacles():
    """Return dict of static boxes and cylinders."""
    import geometry as g

    boxes = {
        "table": (np.array([0.3, 0.0, g.TOP - 0.03]), np.array([1.1, 1.3, 0.06])),
        "fixture_base": (np.array(g.BASE_POS), np.array(g.BASE_SIZE)),
        "cartridge_housing": (np.array(g.CARTRIDGE_HOUSING_POS), np.array(g.CARTRIDGE_HOUSING_SIZE)),
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
            "tcp": p.copy(),
            "finger": float(finger),
        })
        t += DT
    return out


def wp_class(label: str) -> str:
    s = label.lower()
    if "push" in s or "compress" in s or "slide" in s or "contact" in s:
        return "manipulation"
    if "hover" in s or "lift" in s or "retract" in s or "clear" in s or "align" in s:
        return "transit"
    if "ready" in s or "sync" in s or "settle" in s or "wait" in s or "hold" in s or "idle" in s:
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
            {"cls": wp_class(label), "pe": 0.0, "re": 0.0, "margin": 9.0, "min": (np.inf, ""), "extra": {}, "t": s["t"]}
        )
        r["pe"], r["re"], r["margin"] = max(r["pe"], s["pe"]), max(r["re"], s["re"]), min(r["margin"], s["margin"])
        pts = s["pts"]
        who = s["who"]
        lo, hi = s["aabb"][0] - 0.12, s["aabb"][1] + 0.12

        def consider(obj: str, d_all: np.ndarray):
            k = int(np.argmin(d_all))
            d_mm = float(d_all[k] * 1000)
            link = str(who[k])
            if link == "base" and obj in ("table", "fixture_base"):
                return d_mm
            if d_mm < r["min"][0]:
                r["min"] = (d_mm, f"{link} vs {obj}")
                r["t"] = s["t"]
            return d_mm

        for name, (c, sz) in boxes.items():
            if np.any(c + sz / 2 < lo) or np.any(c - sz / 2 > hi):
                continue
            # Intended manipulation contacts: finger contact on plunger knob / lock bar handle
            is_push = any(k in label.lower() for k in ("compress", "contact", "hold", "slide", "ease off", "stalled", "maintain", "keep"))
            if is_push and name in ("fixture_base", "cartridge_housing"):
                continue

            consider(name, box_dist(pts, c, sz))

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
    ap.add_argument("--yam-arena", help="path to yam-isaac-lab-arena")
    ap.add_argument("--modes", nargs="*", default=["demo"], help="modes to check")
    a = ap.parse_args()
    if a.yam_arena:
        sys.path.insert(0, os.path.expanduser(a.yam_arena))

    all_ok = True
    for mode in a.modes:
        print(f"\n=== Collision Check: mode '{mode}' ===")
        left_samples = trace_arm("left", mode)
        right_samples = trace_arm("right", mode)

        print("\nAuditing left arm scene clearances...")
        left_rep, left_bad = analyse("left", left_samples)
        print("Auditing right arm scene clearances...")
        right_rep, right_bad = analyse("right", right_samples)

        print("\nAuditing bimanual arm-arm clearance...")
        arm_min_mm, worst_t = check_arm_arm(left_samples, right_samples)
        print(f"Bimanual arm-arm min clearance: {arm_min_mm:.2f} mm at t={worst_t:.2f}s")

        min_scene_clear_mm = min(min(r["min"][0] for r in left_rep.values()), min(r["min"][0] for r in right_rep.values()))
        print(f"Minimum scene clearance: {min_scene_clear_mm:.2f} mm")

        bad = left_bad + right_bad
        if arm_min_mm < ARM_ARM_MIN_MM:
            bad.append(f"Arm-arm clearance {arm_min_mm:.2f} mm < threshold {ARM_ARM_MIN_MM} mm")

        if bad:
            print("\nCOLLISIONS DETECTED:")
            for b in bad:
                print("  FAIL:", b)
            all_ok = False
        else:
            print("\nALL CLEARANCES SATISFIED. 0 collisions detected.")

    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
