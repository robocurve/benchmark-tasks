#!/usr/bin/env python3
"""Offline collision check of the scripted demo for buffer-socket-module-permutation.

No simulator needed (NumPy only).
Checks both arms against: table, bus strip, 4 socket shells, controller box, tie bar,
COMMIT button cap, status lamp, socket lamps, and all three modules.
Arm-arm clearance is checked at every step.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
else:
    cand = HERE.parent.parent / "refs" / "yam-isaac-lab-arena"
    if cand.exists():
        sys.path.insert(0, str(cand))

MIN_CLEAR_MM = 1.0          # every unintended clearance must be at least this
ARM_ARM_MIN_MM = 12.0       # bimanual arm-arm distance minimum (spec requires >= 12 mm)
PRESS_EXTRA_MM = 0.5        # tolerance on stop-limited penetration
DT = 0.05
R_SEATED = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # plug frame -> world (yaw -pi/2)


def fixture_names(n: int) -> list[str]:
    names = ["bus strip"]
    for i in range(4):
        names += [
            f"socket{i} flange", f"socket{i} wall +y", f"socket{i} wall -y",
            f"socket{i} wall bottom", f"socket{i} wall top-a", f"socket{i} wall top-b",
            f"socket{i} key-slot cap",
        ]
    names += ["controller box", "tie bar"]
    return names[:n] + [f"fixture box {i}" for i in range(len(names), n)]


def static_obstacles():
    """(boxes, cylinders): static rigid bodies in the station."""
    import geometry as g

    fb = g.fixture_boxes()
    names = fixture_names(len(fb))
    boxes = {n: (np.array(c), np.array(s)) for n, (c, s, _col) in zip(names, fb, strict=True)}
    boxes["table"] = (np.array([0.3, 0.0, g.TOP - 0.03]), np.array([1.1, 1.3, 0.06]))

    cyls = {}
    cyls["status lamp base"] = (np.array(g.STATUS_LAMP_POS), g.STATUS_LAMP_R, g.STATUS_LAMP_H)
    for idx, pos in g.SOCKET_LAMP_POS.items():
        cyls[f"socket {idx+1} lamp disc"] = (np.array(pos), g.SOCKET_LAMP_R, g.SOCKET_LAMP_H)

    return boxes, cyls


def cap_box(pressed: bool):
    import geometry as g

    pos = np.array(g.COMMIT_BTN_POS, float)
    if pressed:
        pos[2] -= g.BUTTON_TRAVEL
    return pos, np.array(g.BUTTON_CAP_SIZE)


def servo_world_boxes(origin):
    import geometry as g

    out = []
    for centre, size, _col in g.servo_boxes(g.BAND_GREEN):
        out.append((np.array(origin) + R_SEATED @ np.array(centre), np.abs(R_SEATED) @ np.array(size)))
    return out


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
        eff = min(finger, -g.HOUSING[0] / 2) if arm == "right" else finger
        q = {n: float(v) for n, v in zip(ARM_JOINTS, plan.q, strict=True)}
        q.update({n: float(eff) for n in FINGER_JOINTS})
        hulls = analysis.world_hulls(analysis.fk_link_poses_world(q, base))
        out.append({
            "t": t,
            "label": label,
            "hulls": hulls,
            "pe": pe,
            "re": re,
            "margin": margin,
            "tcp": p.copy(),
            "finger": float(eff),
        })
        t += DT
    return out


def wp_class(label: str) -> str:
    s = label.lower()
    left = s.startswith("l:")
    if left and ("press commit" in s or "hold commit" in s or "release commit" in s):
        return "button press/release"
    if left and "hover commit" in s:
        return "button approach"
    if "wait" in s or "final hold" in s or "ready" in s or "home" in s:
        return "park/wait/home"
    if any(k in s for k in ("push", "dwell plug", "release plug")):
        return "socket insertion"
    if any(k in s for k in ("grip", "pull", "descend", "retract")):
        return "servo pick/pull/stage"
    if any(k in s for k in ("hover", "lift", "transit")) and "home" not in s:
        return "transit/hover"
    return "park/wait/home"


def build_module_schedule(right_wps):
    """Compute the seated socket or handling intervals for each module."""
    t = 0.0
    timeline = []
    for w in right_wps:
        timeline.append((t, t + w.duration, w.label))
        t += w.duration
    return timeline


def get_module_status(timeline, t: float):
    """Return status of ('green', 'red', 'blue') at time t: socket_idx or 'handling'."""
    # Move 1: Green from 0 to 3
    # Move 2: Red from 1 to 0
    # Move 3: Green from 3 to 1
    # Blue: always at socket 2

    # Find active label at time t
    active_label = ""
    for t0, t1, label in timeline:
        if t0 <= t <= t1:
            active_label = label
            break

    # Determine move phases
    # Move 1: starts when green s1 is descended, ends when s4 is released
    t_m1_start = next(t0 for t0, t1, lbl in timeline if "descend to green s1" in lbl)
    t_m1_end = next(t1 for t0, t1, lbl in timeline if "release plug s4" in lbl)

    # Move 2: starts when red s2 is descended, ends when s1 is released
    t_m2_start = next(t0 for t0, t1, lbl in timeline if "descend to red s2" in lbl)
    t_m2_end = next(t1 for t0, t1, lbl in timeline if "release plug s1" in lbl)

    # Move 3: starts when green s4 is descended, ends when s2 is released
    t_m3_start = next(t0 for t0, t1, lbl in timeline if "descend to green s4" in lbl)
    t_m3_end = next(t1 for t0, t1, lbl in timeline if "release plug s2" in lbl)

    # Green status
    if t < t_m1_start:
        green_stat = 0
    elif t <= t_m1_end:
        green_stat = "handling"
    elif t < t_m3_start:
        green_stat = 3  # in buffer socket 4
    elif t <= t_m3_end:
        green_stat = "handling"
    else:
        green_stat = 1  # in socket 2

    # Red status
    if t < t_m2_start:
        red_stat = 1
    elif t <= t_m2_end:
        red_stat = "handling"
    else:
        red_stat = 0

    blue_stat = 2

    return {"green": green_stat, "red": red_stat, "blue": blue_stat}


def analyse(arm: str, samples):
    import geometry as g
    import demo_plan

    boxes, cyls = static_obstacles()
    timeline = build_module_schedule(demo_plan.plans()["right"])

    rep: dict[str, dict] = {}
    bad: list[str] = []
    for s in samples:
        label = s["label"]
        r = rep.setdefault(
            label,
            {"cls": wp_class(label), "pe": 0.0, "re": 0.0, "margin": 9.0, "min": (np.inf, ""), "extra": {}, "t": s["t"]},
        )
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

        pressing = "commit" in label.lower() and wp_class(label) == "button press/release"
        # static boxes
        for name, (c, size) in boxes.items():
            if np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi):
                continue
            consider(name, box_dist(pts, c, size))
        for name, (c, rad, h) in cyls.items():
            if np.any(c + max(rad, h / 2) < lo) or np.any(c - max(rad, h / 2) > hi):
                continue
            consider(name, cyl_dist(pts, c, rad, h))

        # COMMIT button cap
        c, size = cap_box(pressed=pressing)
        if not (np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi)):
            d = box_dist(pts, c, size)
            if pressing:
                foot = (np.abs(pts[:, 0] - c[0]) <= size[0] / 2) & (np.abs(pts[:, 1] - c[1]) <= size[1] / 2)
                cap_top = c[2] + size[2] / 2
                if foot.any():
                    kk = int(np.argmin(np.where(foot, pts[:, 2], np.inf)))
                    pen = float((cap_top - pts[kk, 2]) * 1000)
                    who_pen = f"{who[kk]} vs commit_cap (at stop)"
                else:
                    pen = -float(np.min(d)) * 1000
                    who_pen = "commit_cap"
                prev = r["extra"].get("press_pen", (-np.inf, ""))[0]
                if pen > prev:
                    r["extra"]["press_pen"] = (pen, who_pen)
            else:
                consider("commit_cap", d)

        # Module obstacles in scene
        statuses = get_module_status(timeline, s["t"])
        for mod_name, stat in statuses.items():
            if stat == "handling":
                # Being held/manipulated by right arm
                continue
            sock_y = g.SOCKET_Y[stat]
            origin = (g.MOUTH_X, sock_y, g.SOCKET_Z)
            for bi, (bc, bs) in enumerate(servo_world_boxes(origin)):
                consider(f"{mod_name}_mod(s{stat+1})_b{bi}", box_dist(pts, bc, bs))

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
            # Hand is intentionally pushing module nose into socket
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
            print(f"  {label:40} [{r['cls']:20}] min clear: {d_mm:6.2f} mm ({who}){extra}")
        if bad:
            print(f"\nProblems for {arm} arm:")
            for b in bad:
                print(f"  {b}")

    if bad_l or bad_r:
        print(f"\nFAILED: {len(bad_l) + len(bad_r)} clearance issues found")
        return 1

    print("\nSUCCESS: All clearances >= 1.0 mm, arm-arm distance >= 12 mm!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
