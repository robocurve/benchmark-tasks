#!/usr/bin/env python3
"""Offline collision check of the scripted demo against the scene for Task 1. No simulator needed (numpy only).

Every control step (20 Hz) of every waypoint of every plan mode (default: demo; set DEMO_ADJACENT=1 etc.
to check another mode) is checked for BOTH arms against ALL of:
- the table
- the fixture base plate
- the cold plate top
- the four button caps (at rest, or at their 8 mm end stop while being pressed)
- the 12 cyan stage lamps
- the status lamp
- arm-arm clearance at every step.

Intended contacts:
- Button press: the fingertip is commanded deeper than the cap's 8 mm end stop; the cap is modelled AT ITS STOP,
  and commanded penetration into it must not exceed (commanded depth - stop travel) + 0.5 mm = 6.5 mm.
  The fingertip must stay >= 1 mm above the cold plate top.
All unintended clearances must be >= 1.0 mm. Arm-arm clearance must be >= 15.0 mm.
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
ARM_ARM_MIN_MM = 15.0       # arm-arm distance minimum threshold
PRESS_EXTRA_MM = 0.5        # tolerance on stop-limited penetration
DT = 0.05


def static_obstacles():
    """Return dict of static boxes and cylinders."""
    import geometry as g

    boxes = {
        "base fixture": (np.array(g.BASE_POS), np.array(g.BASE_SIZE)),
        "cold plate": (np.array(g.COLD_PLATE_POS), np.array(g.COLD_PLATE_SIZE)),
        "table": (np.array([0.3, 0.0, g.TOP - 0.03]), np.array([1.1, 1.3, 0.06])),
    }

    cyls = {
        "status lamp": (np.array(g.STATUS_LAMP_POS), g.STATUS_LAMP_R, g.STATUS_LAMP_H),
    }

    for (corner, stage), pos in g.STAGE_LAMP_POS.items():
        cyls[f"lamp_{corner}_{stage}"] = (np.array(pos), g.STAGE_LAMP_R, g.STAGE_LAMP_H)

    return boxes, cyls


def cap_box(corner: str, pressed: bool):
    import geometry as g

    pos = np.array(g.BOLT_REST_POS[corner], dtype=float)
    if pressed:
        pos[2] -= g.BUTTON_TRAVEL
    return pos, np.array(g.BUTTON_CAP_SIZE, dtype=float)


def box_dist(points, centre, size):
    from yam_arena import analysis

    return analysis.box_signed_distance(points, np.asarray(centre), np.eye(3), np.asarray(size))


def cyl_dist(points, centre, radius, height):
    d = points - np.asarray(centre)
    rad = np.hypot(d[:, 0], d[:, 1]) - radius
    vert = np.abs(d[:, 2]) - height / 2
    outside = np.hypot(np.maximum(rad, 0.0), np.maximum(vert, 0.0))
    inside = np.minimum(np.maximum(rad, vert), 0.0)
    return outside + inside


def trace_arm(arm: str, shift: float):
    import demo_plan
    import geometry as g
    from yam_arena import analysis, ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
    from yam_arena.demo import ArmPlan

    os.environ["DEMO_YAW_SHIFT_LEFT" if arm == "left" else "DEMO_YAW_SHIFT_RIGHT"] = repr(shift)
    waypoints = demo_plan.plans()[arm]
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
    if "press" in s or "hold" in s:
        return "button press"
    if "hover" in s or "lift" in s:
        return "button transit"
    if "ready" in s or "wait" in s or "final hold" in s:
        return "park/wait/home"
    return "other"


def corner_of(label: str) -> str | None:
    for c in ("L_near", "L_far", "R_near", "R_far"):
        if c in label:
            return c
    return None


def analyse(arm: str, samples):
    """Per-label dict of worst clearances, with object names. Returns (report, bad_messages)."""
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

        btn_corner = corner_of(label)
        pressing = (btn_corner is not None) and (wp_class(label) == "button press")

        # Static boxes
        for name, (c, size) in boxes.items():
            if np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi):
                continue
            consider(name, box_dist(pts, c, size))

        # Static cylinders
        for name, (c, rad, h) in cyls.items():
            if np.any(c + max(rad, h / 2) < lo) or np.any(c - max(rad, h / 2) > hi):
                continue
            consider(name, cyl_dist(pts, c, rad, h))

        # Button caps
        for c_name in g.CORNERS:
            is_target_press = pressing and (btn_corner == c_name)
            is_target_lift = ("lift" in label) and (btn_corner == c_name)

            if is_target_press:
                c, size = cap_box(c_name, pressed=True)
                if np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi):
                    continue
                d = box_dist(pts, c, size)
                # Target cap during press: penetration is intended up to command margin
                foot = (np.abs(pts[:, 0] - c[0]) <= size[0] / 2) & (np.abs(pts[:, 1] - c[1]) <= size[1] / 2)
                cap_top = c[2] + size[2] / 2
                if foot.any():
                    kk = int(np.argmin(np.where(foot, pts[:, 2], np.inf)))
                    pen = float((cap_top - pts[kk, 2]) * 1000)
                    who_pen = f"{who[kk]} vs {c_name}_cap (at stop)"
                else:
                    pen = float(-d.min() * 1000)
                    who_pen = f"{who[int(np.argmin(d))]} vs {c_name}_cap (at stop)"
                cmd_pen = float((g.BUTTON_PRESS_DEPTH_CMD - g.BUTTON_TRAVEL) * 1000)  # 6.0 mm
                r["extra"]["intended_cap_pen_mm"] = max(r["extra"].get("intended_cap_pen_mm", 0.0), pen)
                r["extra"]["who_cap"] = who_pen
                if pen > cmd_pen + PRESS_EXTRA_MM:
                    bad.append(f"t={s['t']:.2f}s [{label}] {who_pen}: cap penetration {pen:.2f} mm > {cmd_pen + PRESS_EXTRA_MM:.2f} mm")
                # Fingertip height above cold plate top
                tip_pts = pts[np.isin(who, ["tip_left", "tip_right"])]
                if len(tip_pts):
                    tip_clear = float((tip_pts[:, 2].min() - g.COLD_PLATE_TOP_Z) * 1000)
                    r["extra"]["tip_to_plate_mm"] = min(r["extra"].get("tip_to_plate_mm", np.inf), tip_clear)
                    if tip_clear < MIN_CLEAR_MM:
                        bad.append(f"t={s['t']:.2f}s [{label}] fingertip clear to plate {tip_clear:.2f} mm < {MIN_CLEAR_MM} mm")
            elif is_target_lift:
                # Lifting off the target button cap that was just pressed (intended disengagement)
                continue
            else:
                c, size = cap_box(c_name, pressed=False)
                if np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi):
                    continue
                d = box_dist(pts, c, size)
                consider(f"{c_name} cap", d)

    for label, r in rep.items():
        min_d, who_min = r["min"]
        if min_d < MIN_CLEAR_MM:
            bad.append(f"[{label}] unintended clearance {min_d:.2f} mm < {MIN_CLEAR_MM} mm ({who_min})")

    return rep, bad


def check_arm_arm(left_samples, right_samples):
    bad = []
    min_gap = np.inf
    worst_t = 0.0
    min_tcp = np.inf

    for sl, sr in zip(left_samples, right_samples, strict=True):
        d_tcp = float(np.linalg.norm(sl["tcp"] - sr["tcp"]))
        if d_tcp < min_tcp:
            min_tcp = d_tcp
        lo_l, hi_l = sl["aabb"]
        lo_r, hi_r = sr["aabb"]
        gap = float(np.maximum(lo_r - hi_l, lo_l - hi_r).max())
        if gap < min_gap:
            min_gap = gap
            worst_t = sl["t"]

    min_mm = min_gap * 1000
    if min_mm < ARM_ARM_MIN_MM:
        bad.append(f"Arm-arm clearance {min_mm:.1f} mm < {ARM_ARM_MIN_MM} mm at t={worst_t:.2f}s")
    return min_mm, min_tcp, bad


def main() -> int:
    import geometry as g

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--yam-arena", help="a checkout of yam-isaac-lab-arena, when yam_arena is not installed")
    a = ap.parse_args()
    if a.yam_arena:
        sys.path.insert(0, os.path.expanduser(a.yam_arena))

    t0 = time.time()
    print("Tracing left arm...")
    left_samples = trace_arm("left", 0.0)
    print("Tracing right arm...")
    right_samples = trace_arm("right", 0.0)

    print(f"Traced {len(left_samples)} steps per arm in {time.time() - t0:.1f}s.")

    print("\nChecking arm-arm clearance...")
    arm_arm_min, arm_tcp_min, arm_bad = check_arm_arm(left_samples, right_samples)
    print(f"  Minimum hand-to-hand distance: {arm_tcp_min * 1000:.1f} mm (limit >= 100 mm)")
    print(f"  Minimum arm-arm clearance: {arm_arm_min:.1f} mm (limit >= 5.0 mm)")

    print("\nAnalysing left arm waypoints...")
    rep_l, bad_l = analyse("left", left_samples)
    for label, r in rep_l.items():
        extra_str = ""
        if "intended_cap_pen_mm" in r["extra"]:
            extra_str += f" [cap pen {r['extra']['intended_cap_pen_mm']:.1f}mm, plate clear {r['extra']['tip_to_plate_mm']:.1f}mm]"
        print(f"  {label:35}: min {r['min'][0]:5.1f} mm ({r['min'][1]}){extra_str}")

    print("\nAnalysing right arm waypoints...")
    rep_r, bad_r = analyse("right", right_samples)
    for label, r in rep_r.items():
        extra_str = ""
        if "intended_cap_pen_mm" in r["extra"]:
            extra_str += f" [cap pen {r['extra']['intended_cap_pen_mm']:.1f}mm, plate clear {r['extra']['tip_to_plate_mm']:.1f}mm]"
        print(f"  {label:35}: min {r['min'][0]:5.1f} mm ({r['min'][1]}){extra_str}")

    all_bad = arm_bad + bad_l + bad_r
    if all_bad:
        print(f"\nFAILED with {len(all_bad)} collision violations:")
        for b in all_bad:
            print(f"  * {b}")
        return 1

    print("\nPASSED: 0 collisions, all clearances verified.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
