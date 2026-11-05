#!/usr/bin/env python3
"""Offline collision check of the scripted demo against the scene for the thermal soak task.

Every control step (20 Hz) along each plan mode is checked for BOTH arms against ALL of:
the table, every fixture box, the status lamp and 9 bar lamps, the two button caps
(at rest, or at their 8 mm end stop while pressed), and both sample blocks (at staging,
on hot plate, or on cool pad).
Fingers are modelled at their commanded opening.
Arm-arm distance is checked at every step (must be >= 15 mm).
Unintended clearances must be >= 1.0 mm.
Intended contacts:
- Button press: fingertip penetration into cap at stop <= (commanded depth - stop travel) + 0.5 mm (6.5 mm).
- Grasp: fingers clamping the 60 mm sample block.

Exit status 0 if all checks pass, 1 otherwise.
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

MIN_CLEAR_MM = 1.0
ARM_ARM_MIN_MM = 15.0
PRESS_EXTRA_MM = 0.5
DT = 0.05


def static_obstacles():
    """Static boxes and cylinders composing the fixture, pads, and indicator bases."""
    import geometry as g

    fb = g.fixture_boxes()
    names = [
        "fixture_base",
        "sample_pad",
        "bar_strip",
        "cool_pad",
        "staging_mat",
    ]
    boxes = {names[i]: (np.array(b[0]), np.array(b[1])) for i, b in enumerate(fb)}
    boxes["table"] = (np.array([0.3, 0.0, g.TOP - 0.03]), np.array([1.1, 1.3, 0.06]))

    fc = g.fixture_cylinders()
    cyls = {"status_lamp_base": (np.array(fc[0][0]), float(fc[0][1]), float(fc[0][2]))}
    for i, c in enumerate(fc[1:]):
        cyls[f"bar_lamp_base_{i+1}"] = (np.array(c[0]), float(c[1]), float(c[2]))

    return boxes, cyls


def cap_box(which: str, pressed: bool = False):
    import geometry as g

    pos = g.HEAT_BTN_POS if which == "HEAT" else g.OFF_BTN_POS
    z = pos[2] - (g.BUTTON_TRAVEL if pressed else 0.0)
    return np.array([pos[0], pos[1], z]), np.array(g.BUTTON_CAP_SIZE)


def sample_box(pos):
    import geometry as g

    return np.array(pos), np.array(g.SAMPLE_SIZE)


def box_dist(pts: np.ndarray, c: np.ndarray, size: np.ndarray) -> np.ndarray:
    """Signed distance from Nx3 points to an axis-aligned box (centre c, edge lengths size). Negative inside."""
    d = np.abs(pts - c) - size / 2.0
    outside = np.linalg.norm(np.maximum(d, 0.0), axis=1)
    inside = np.minimum(np.max(d, axis=1), 0.0)
    return outside + inside


def cyl_dist(pts: np.ndarray, c: np.ndarray, rad: float, h: float) -> np.ndarray:
    """Signed distance to vertical cylinder with axis Z."""
    dxy = np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1]) - rad
    dz = np.abs(pts[:, 2] - c[2]) - h / 2.0
    outside = np.hypot(np.maximum(dxy, 0.0), np.maximum(dz, 0.0))
    inside = np.minimum(np.maximum(dxy, dz), 0.0)
    return outside + inside


def trace_arm(arm: str, shift: float, mode: str = "demo"):
    import demo_plan
    import geometry as g
    from yam_arena import analysis, ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
    from yam_arena.demo import ArmPlan

    os.environ["DEMO_YAW_SHIFT_LEFT" if arm == "left" else "DEMO_YAW_SHIFT_RIGHT"] = repr(shift)
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
        out.append({
            "t": t,
            "label": label,
            "hulls": hulls,
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
    if "press" in s or "hold" in s or "approach" in s or ("release" in s and ("heat" in s or "off" in s)):
        return "button press/release"
    if "grip" in s or "release" in s:
        return "sample grip/release"
    if "descend" in s or "lower" in s:
        return "sample descend"
    if "lift" in s:
        return "sample lift"
    if "hover" in s or "carry" in s:
        return "sample transit/hover"
    return "park/wait/home"


def sample_locations(wps_left, wps_right):
    """Estimate whereabouts of sample 1 and sample 2 over time from waypoint transitions."""
    import geometry as g

    s1_plate_z = g.PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0
    s1_cool_z = g.COOL_PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0
    s1_locs = [(0.0, g.START_POS_1)]
    s2_locs = [(0.0, g.START_POS_2)]

    t = 0.0
    for w in wps_left:
        t += w.duration
        if "release s1 on plate" in w.label:
            s1_locs.append((t, (g.PAD_POS[0], g.PAD_POS[1], s1_plate_z)))
        elif "release s2 on plate" in w.label:
            s2_locs.append((t, (g.PAD_POS[0], g.PAD_POS[1], s1_plate_z)))

    t = 0.0
    for w in wps_right:
        t += w.duration
        if "release s1 on cool" in w.label:
            s1_locs.append((t, (g.COOL_SPOT_1[0], g.COOL_SPOT_1[1], s1_cool_z)))
        elif "release s2 on cool" in w.label:
            s2_locs.append((t, (g.COOL_SPOT_2[0], g.COOL_SPOT_2[1], s1_cool_z)))

    return s1_locs, s2_locs


def loc_at(locs, t: float):
    return [pos for tt, pos in locs if tt <= t + 1e-6][-1]


def analyse(arm: str, samples, s1_locs, s2_locs):
    import geometry as g

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

        pts, who = [], []
        for name, h in s["hulls"]:
            pts.append(h.vertices)
            who += [name] * len(h.vertices)
        pts = np.vstack(pts)
        who = np.array(who)
        lo, hi = pts.min(0) - 0.10, pts.max(0) + 0.10

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

        is_btn = "press" in label.lower() or "hold" in label.lower() or ("release" in label.lower() and ("heat" in label.lower() or "off" in label.lower()))
        active_btn = "HEAT" if "heat" in label.lower() else ("OFF" if "off" in label.lower() else None)

        # 1. Static boxes
        for name, (c, size) in boxes.items():
            if np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi):
                continue
            consider(name, box_dist(pts, c, size))

        # 2. Static cylinders
        for name, (c, rad, h) in cyls.items():
            if np.any(c + max(rad, h / 2) < lo) or np.any(c - max(rad, h / 2) > hi):
                continue
            consider(name, cyl_dist(pts, c, rad, h))

        # 3. Button caps
        for which in ("HEAT", "OFF"):
            pressing = is_btn and (active_btn == which)
            c, size = cap_box(which, pressed=pressing)
            if np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi):
                continue
            d = box_dist(pts, c, size)
            if pressing:
                foot = (np.abs(pts[:, 0] - c[0]) <= size[0] / 2) & (np.abs(pts[:, 1] - c[1]) <= size[1] / 2)
                cap_top = c[2] + size[2] / 2
                if foot.any():
                    kk = int(np.argmin(np.where(foot, pts[:, 2], np.inf)))
                    pen = float((cap_top - pts[kk, 2]) * 1000)
                    who_pen = f"{who[kk]} vs {which}_cap (at stop)"
                else:
                    pen, who_pen = -np.inf, ""
                if pen > r["extra"].get("cap_pen", -np.inf):
                    r["extra"]["cap_pen"] = pen
                    r["extra"]["cap_pen_who"] = who_pen
                lim = (g.BUTTON_PRESS_DEPTH_CMD - g.BUTTON_TRAVEL) * 1000 + PRESS_EXTRA_MM
                if pen > lim:
                    bad.append(
                        f"{arm} '{label}' t={s['t']:.2f}: fingertip {pen:.1f} mm below {which} cap top at stop (limit {lim:.1f})"
                    )
            else:
                consider(f"{which}_cap", d)

        # 4. Sample blocks
        is_handling_s1 = "s1" in label.lower()
        is_handling_s2 = "s2" in label.lower()
        for idx, (is_handled, locs, s_name) in enumerate([
            (is_handling_s1, s1_locs, "sample_1"),
            (is_handling_s2, s2_locs, "sample_2"),
        ]):
            c, size = sample_box(loc_at(locs, s["t"]))
            if np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi):
                continue
            d = box_dist(pts, c, size)
            if is_handled and r["cls"] in ("sample grip/release", "sample descend"):
                # Intended grasp contact
                pass
            elif is_handled and r["cls"] in ("sample lift", "sample transit/hover"):
                # Carried sample: finger pads in contact with block
                pass
            else:
                consider(s_name, d)

        if r["min"][0] < MIN_CLEAR_MM:
            bad.append(f"{arm} '{label}' t={s['t']:.2f}: {r['min'][0]:.2f} mm ({r['min'][1]}) < {MIN_CLEAR_MM} mm")

    # Deduplicate
    seen, uniq = set(), []
    for m in bad:
        key = m.split(" t=")[0] + m.split(": ", 1)[1][:25]
        if key not in seen:
            seen.add(key)
            uniq.append(m)
    return rep, uniq


def reachable(rep) -> bool:
    return all(r["pe"] <= 3.0 and r["re"] <= 3.0 and r["margin"] >= 0.02 for r in rep.values())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--yam-arena", help="a checkout of yam-isaac-lab-arena, when yam_arena is not installed")
    ap.add_argument("--mode", default="demo", help="plan mode to check")
    a = ap.parse_args()
    if a.yam_arena:
        sys.path.insert(0, os.path.expanduser(a.yam_arena))

    from yam_arena import analysis
    import demo_plan
    import geometry as g

    t0 = time.time()
    shift_l = float(os.environ.get("DEMO_YAW_SHIFT_LEFT", 0.0))
    shift_r = float(os.environ.get("DEMO_YAW_SHIFT_RIGHT", 0.0))
    print(f"plan mode: {a.mode}")

    p = demo_plan.plans(a.mode)
    if not p:
        print("Empty plan (settle) - no motions to check.")
        return 0

    s1_locs, s2_locs = sample_locations(p.get("left", []), p.get("right", []))

    traces, problems = {}, []
    cls_worst: dict[str, tuple[float, str, str]] = {}

    for arm, shift in (("left", shift_l), ("right", shift_r)):
        traces[arm] = trace_arm(arm, shift, mode=a.mode)
        rep, bad = analyse(arm, traces[arm], s1_locs, s2_locs)
        problems += bad
        print(f"\n=== {arm} arm: {'reachable' if reachable(rep) else 'NOT reachable'}; {len(traces[arm])} samples ===")
        for label, r in rep.items():
            d, what = r["min"]
            extra = ""
            if "cap_pen" in r["extra"]:
                extra = f"  | cap penetration at stop: {r['extra']['cap_pen']:.1f} mm ({r['extra']['cap_pen_who']})"
            flag = "   <-- PROBLEM" if d < MIN_CLEAR_MM else ""
            print(f"  {label:34} [{r['cls']:22}] ik {r['pe']:5.2f} mm {r['re']:5.2f} deg margin {r['margin']:5.3f} rad  "
                  f"clearance {d:7.1f} mm ({what}){extra}{flag}")
            c = cls_worst.get(f"{arm}: {r['cls']}", (np.inf, "", ""))
            if d < c[0]:
                cls_worst[f"{arm}: {r['cls']}"] = (d, what, label)

        if not reachable(rep):
            problems.append(f"{arm}: waypoints not reachable")

    # Arm-against-arm clearance at every control step
    best = (np.inf, None, 0.0, "")
    n = min(len(traces["left"]), len(traces["right"]))
    for k in range(n):
        # Broadphase AABB gap along y between left arm (positive y) and right arm (negative y)
        lo_a = min(h.vertices[:, 1].min() for _, h in traces["left"][k]["hulls"])
        hi_b = max(h.vertices[:, 1].max() for _, h in traces["right"][k]["hulls"])
        if lo_a - hi_b > 0.06:
            continue
        d, pair = analysis.hull_set_distance(traces["left"][k]["hulls"], traces["right"][k]["hulls"])
        if d < best[0]:
            best = (d, pair, traces["left"][k]["t"], f"{traces['left'][k]['label']} / {traces['right'][k]['label']}")

    if best[1] is None:
        gap_min = np.inf
        for k in range(n):
            ha_boxes = np.array([[ha.vertices.min(0), ha.vertices.max(0)] for _, ha in traces["left"][k]["hulls"]])
            hb_boxes = np.array([[hb.vertices.min(0), hb.vertices.max(0)] for _, hb in traces["right"][k]["hulls"]])
            lo_a, hi_a = ha_boxes[:, 0, :], ha_boxes[:, 1, :]
            lo_b, hi_b = hb_boxes[:, 0, :], hb_boxes[:, 1, :]
            diff1 = lo_b[None, :, :] - hi_a[:, None, :]
            diff2 = lo_a[:, None, :] - hi_b[None, :, :]
            gap_k = float(np.maximum(diff1, diff2).max(axis=-1).min())
            if gap_k < gap_min:
                gap_min = gap_k
        arm_mm = max(gap_min, 0.05) * 1000
        print(f"\n=== left arm vs right arm ===\n  every step checked; all link pairs > 50 mm apart (min AABB gap {gap_min * 1000:.1f} mm)")
    else:
        arm_mm = best[0] * 1000
        print(f"\n=== left arm vs right arm ===\n  closest approach {arm_mm:.1f} mm at t={best[2]:.2f}s ({best[1]}) during '{best[3]}'")

    if arm_mm < ARM_ARM_MIN_MM:
        problems.append(f"arm-arm clearance {arm_mm:.1f} mm < {ARM_ARM_MIN_MM}")

    print("\n=== Worst clearance per waypoint class ===")
    for k, (d, what, label) in sorted(cls_worst.items()):
        print(f"  {k:36} {d:7.1f} mm  {what}  (waypoint '{label}')")

    print(f"\nrun time {time.time() - t0:.1f} s")
    if problems:
        print(f"\nPROBLEMS FOUND ({len(problems)}):")
        for m in problems:
            print("  -", m)
        return 1

    print(f"\nALL CHECKS PASSED: every unintended clearance >= {MIN_CLEAR_MM} mm, arm-arm >= {ARM_ARM_MIN_MM} mm, "
          "intended contacts within their stroke")
    return 0


if __name__ == "__main__":
    sys.exit(main())
