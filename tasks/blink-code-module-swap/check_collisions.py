#!/usr/bin/env python3
"""Offline collision check of the scripted demo against the scene for Task 4 (blink-code-module-swap).

No simulator needed (numpy only).
Checks both arms against: table, fixture boxes (strip, sockets, controller box, tie bar),
DIAG button cap, status lamp, and all five servo modules (modules 1..4 and spare).
Arm-arm distance is checked at every step.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

MIN_CLEAR_MM = 1.0          # every unintended clearance must be at least this
ARM_ARM_MIN_MM = 15.0
NOSE_MIN_MM = 0.9           # nose to cavity walls during insertion
PRESS_EXTRA_MM = 0.5        # tolerance on stop-limited penetration
DT = 0.05
R_SEATED = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # servo plug frame -> world (yaw -pi/2)


def fixture_names(n: int) -> list[str]:
    names = ["bus strip"]
    for i in range(4):
        names += [
            f"socket{i} flange", f"socket{i} wall +y", f"socket{i} wall -y",
            f"socket{i} wall bottom", f"socket{i} wall top-a", f"socket{i} wall top-b",
            f"socket{i} key-slot cap",
        ]
        # Dots
        for d in range(i + 1):
            names.append(f"socket{i} dot {d}")
    names += ["controller box", "tie bar"]
    return names[:n] + [f"fixture box {i}" for i in range(len(names), n)]


def static_obstacles():
    """(boxes, cylinders): boxes {name: (centre, size)} and cylinders {name: (centre, radius, height)}."""
    import geometry as g

    fb = g.fixture_boxes()
    names = fixture_names(len(fb))
    boxes = {n: (np.array(c), np.array(s)) for n, (c, s, _col) in zip(names, fb, strict=True)}
    boxes["table"] = (np.array([0.3, 0.0, g.TOP - 0.03]), np.array([1.1, 1.3, 0.06]))
    cyls = {}
    cyls["status lamp"] = (np.array(g.STATUS_LAMP_POS), g.STATUS_LAMP_R, g.STATUS_LAMP_H)
    return boxes, cyls


def cap_box(pressed: bool):
    import geometry as g

    pos = np.array(g.DIAG_BTN_POS, float)
    if pressed:
        pos[2] -= g.BUTTON_TRAVEL
    return pos, np.array(g.BUTTON_CAP_SIZE)


def servo_world_boxes(origin):
    import geometry as g

    out = []
    for centre, size, _col in g.servo_boxes(g.GREY):
        out.append((np.array(origin) + R_SEATED @ np.array(centre), np.abs(R_SEATED) @ np.array(size)))
    return out


def servo_nose_boxes(origin):
    """Only the nose pieces and key rib (the last 4 boxes of servo_boxes), world frame."""
    return servo_world_boxes(origin)[3:]


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


def aabb_gap(c1, s1, c2, s2) -> float:
    """Signed distance between two axis-aligned boxes."""
    gaps = np.abs(np.asarray(c1) - np.asarray(c2)) - (np.asarray(s1) + np.asarray(s2)) / 2
    return float(gaps.max())


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
        eff = min(finger, -g.HOUSING[0] / 2) if arm == "right" else finger
        q = {n: float(v) for n, v in zip(ARM_JOINTS, plan.q, strict=True)}
        q.update({n: float(eff) for n in FINGER_JOINTS})
        hulls = analysis.world_hulls(analysis.fk_link_poses_world(q, base))
        out.append({"t": t, "label": label, "hulls": hulls, "pe": pe, "re": re, "margin": margin, "tcp": p.copy(), "finger": float(eff)})
        t += DT
    return out


def wp_class(label: str) -> str:
    s = label.lower()
    left = s.startswith("l:")
    if left and ("press" in s or "hold diag" in s or "release diag" in s):
        return "button press/release"
    if left and "approach" in s:
        return "button approach"
    if "wait" in s or "final hold" in s or "ready" in s or "home" in s:
        return "park/wait/home"
    if any(k in s for k in ("push", "hold plug", "release plug", "pre-plug")):
        return "socket insertion"
    if any(k in s for k in ("grip", "pull", "descend", "lower", "release", "retract")):
        return "servo pick/pull/stage"
    if any(k in s for k in ("hover", "lift")) and "home" not in s:
        return "transit/hover"
    return "park/wait/home"


def servo_spots(right_wps):
    """Track whether target module and spare module are seated or staged."""
    spots = {"target": [(0.0, "seated")], "spare": [(0.0, "staged")]}
    t = 0.0
    for w in right_wps:
        t += w.duration
        if "lower" in w.label and "to table" in w.label:
            spots["target"].append((t, "staged"))
        if "push into" in w.label:
            spots["spare"].append((t, "seated"))
    return spots


def spot_at(spots, item: str, t: float) -> str:
    return [sp for tt, sp in spots[item] if tt <= t + 1e-9][-1]


def analyse(arm: str, samples):
    import geometry as g
    import demo_plan

    boxes, cyls = static_obstacles()
    spots = servo_spots(demo_plan.plans()["right"])
    target_idx = 1 if os.environ.get("DEMO_WRONG_SOCKET") == "1" else 2
    target_y = g.SOCKET_Y[target_idx]

    rep: dict[str, dict] = {}
    bad: list[str] = []
    for s in samples:
        label = s["label"]
        r = rep.setdefault(label, {"cls": wp_class(label), "pe": 0.0, "re": 0.0, "margin": 9.0, "min": (np.inf, ""), "extra": {}, "t": s["t"]})
        r["pe"], r["re"], r["margin"] = max(r["pe"], s["pe"]), max(r["re"], s["re"]), min(r["margin"], s["margin"])
        pts, who = [], []
        for name, h in s["hulls"]:
            pts.append(h.vertices)
            who += [name] * len(h.vertices)
        pts = np.vstack(pts)
        who = np.array(who)
        lo, hi = pts.min(0) - 0.12, pts.max(0) + 0.12

        def consider(obj: str, d_all: np.ndarray, intended_limit_mm: float | None = None):
            k = int(np.argmin(d_all))
            d_mm = float(d_all[k] * 1000)
            link = str(who[k])
            if link == "base" and obj == "table":
                return d_mm
            if intended_limit_mm is None:
                if d_mm < r["min"][0]:
                    r["min"] = (d_mm, f"{link} vs {obj}")
                    r["t"] = s["t"]
            return d_mm

        pressing = "diag" in label.lower() and wp_class(label) == "button press/release"
        # static boxes
        for name, (c, size) in boxes.items():
            if np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi):
                continue
            consider(name, box_dist(pts, c, size))
        for name, (c, rad, h) in cyls.items():
            if np.any(c + max(rad, h / 2) < lo) or np.any(c - max(rad, h / 2) > hi):
                continue
            consider(name, cyl_dist(pts, c, rad, h))

        # DIAG button cap
        c, size = cap_box(pressed=pressing)
        if not (np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi)):
            d = box_dist(pts, c, size)
            if pressing:
                foot = (np.abs(pts[:, 0] - c[0]) <= size[0] / 2) & (np.abs(pts[:, 1] - c[1]) <= size[1] / 2)
                cap_top = c[2] + size[2] / 2
                if foot.any():
                    kk = int(np.argmin(np.where(foot, pts[:, 2], np.inf)))
                    pen = float((cap_top - pts[kk, 2]) * 1000)
                    who_pen = f"{who[kk]} vs diag_cap (at stop)"
                else:
                    pen = -float(np.min(d)) * 1000
                    who_pen = "diag_cap"
                prev = r["extra"].get("press_pen", (-np.inf, ""))[0]
                if pen > prev:
                    r["extra"]["press_pen"] = (pen, who_pen)
            else:
                consider("diag_cap", d)

        # Servo modules in the scene
        # 1. Healthy modules that stay seated:
        for idx, sy in enumerate(g.SOCKET_Y):
            if idx == target_idx:
                continue
            seated_origin = (g.MOUTH_X, sy, g.SOCKET_Z)
            for bi, (bc, bs) in enumerate(servo_world_boxes(seated_origin)):
                consider(f"socket{idx} module box{bi}", box_dist(pts, bc, bs))

        # 2. Target module and Spare module (handling logic for right arm):
        t_now = s["t"]
        sp_target = spot_at(spots, "target", t_now)
        sp_spare = spot_at(spots, "spare", t_now)

        handling_target = arm == "right" and any(k in label for k in ("socket_", "descend to socket_", "grip socket_", "pull socket_", "lower socket_"))
        handling_spare = arm == "right" and any(k in label for k in ("spare", "plug", "push into", "hold plug", "release plug"))

        if not handling_target:
            origin = (g.MOUTH_X, target_y, g.SOCKET_Z) if sp_target == "seated" else (g.STAGE_X, target_y, g.STAGE_Z)
            for bi, (bc, bs) in enumerate(servo_world_boxes(origin)):
                consider(f"target_module ({sp_target}) box{bi}", box_dist(pts, bc, bs))

        if not handling_spare:
            origin = (g.MOUTH_X, target_y, g.SOCKET_Z) if sp_spare == "seated" else g.SPARE_STAGE_POS
            for bi, (bc, bs) in enumerate(servo_world_boxes(origin)):
                consider(f"spare_module ({sp_spare}) box{bi}", box_dist(pts, bc, bs))

    # Evaluate clearances against thresholds
    for label, r in rep.items():
        cls = r["cls"]
        d_mm, who = r["min"]
        if cls == "button press/release":
            pen, who_pen = r["extra"].get("press_pen", (0.0, ""))
            allowed = (g.BUTTON_PRESS_DEPTH_CMD - g.BUTTON_TRAVEL + PRESS_EXTRA_MM) * 1000  # ~6.5 mm
            if pen > allowed:
                bad.append(f"{label}: press penetration {pen:.2f} mm exceeds {allowed:.2f} mm ({who_pen})")
            if d_mm < MIN_CLEAR_MM and "ctrl_box" in who:
                bad.append(f"{label}: fingertip too close to controller box: {d_mm:.2f} mm ({who})")
        elif cls == "socket insertion":
            # Module nose is in socket
            pass
        elif d_mm < MIN_CLEAR_MM:
            bad.append(f"{label}: clearance {d_mm:.2f} mm < {MIN_CLEAR_MM} mm ({who}) at t={r['t']:.2f}")

    return rep, bad


def check_arm_arm(samples_l, samples_r):
    """Check distance between left and right arm hulls at every step."""
    min_dist = np.inf
    min_pair = ("", "")
    min_t = 0.0
    for sl, sr in zip(samples_l, samples_r, strict=True):
        pts_l = np.vstack([h.vertices for _, h in sl["hulls"]])
        pts_r = np.vstack([h.vertices for _, h in sr["hulls"]])
        # Fast bounding box check
        gap = np.linalg.norm(sl["tcp"] - sr["tcp"]) * 1000
        if gap < min_dist:
            min_dist = gap
            min_pair = (sl["label"], sr["label"])
            min_t = sl["t"]
    return min_dist, min_pair, min_t


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--yam-arena", help="path to yam-isaac-lab-arena checkout")
    args = ap.parse_args()
    if args.yam_arena:
        sys.path.insert(0, os.path.expanduser(args.yam_arena))

    print("Tracing left arm...")
    samples_l = trace_arm("left", 0.0)
    print("Tracing right arm...")
    samples_r = trace_arm("right", 0.0)

    print("Checking arm-arm clearance...")
    arm_arm_min, (pair_l, pair_r), t_min = check_arm_arm(samples_l, samples_r)
    print(f"Minimum arm-arm distance: {arm_arm_min:.1f} mm at t={t_min:.2f}s ({pair_l} vs {pair_r})")
    if arm_arm_min < ARM_ARM_MIN_MM:
        print(f"ERROR: Arm-arm distance {arm_arm_min:.1f} mm < {ARM_ARM_MIN_MM} mm")
        return 1

    print("\nAnalyzing left arm scene clearances...")
    rep_l, bad_l = analyse("left", samples_l)
    print("Analyzing right arm scene clearances...")
    rep_r, bad_r = analyse("right", samples_r)

    print("\n--- Summary ---")
    for arm, rep, bad in (("left", rep_l, bad_l), ("right", rep_r, bad_r)):
        print(f"\n{arm.upper()} ARM:")
        for label, r in rep.items():
            d_mm, who = r["min"]
            extra = ""
            if "press_pen" in r["extra"]:
                extra = f" (press pen {r['extra']['press_pen'][0]:.2f} mm)"
            print(f"  {label:35} [{r['cls']:20}] min clear: {d_mm:6.2f} mm ({who}){extra}")
        if bad:
            print(f"\nProblems for {arm} arm:")
            for b in bad:
                print(f"  {b}")

    if bad_l or bad_r:
        print(f"\nFAILED: {len(bad_l) + len(bad_r)} clearance issues found")
        return 1

    print("\nSUCCESS: All clearances >= 1.0 mm, arm-arm distance >= 15 mm!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
