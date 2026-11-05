#!/usr/bin/env python3
"""Offline collision check of the scripted demo against the scene for Task F. No simulator needed (numpy only).

Every control step (20 Hz) of every waypoint of every plan mode (default: the demo; set DEMO_RECOVER=1 etc. to check
another mode) is checked, with no label filters, for BOTH arms against ALL of: the table, every fixture box (strip,
socket shells, flanges, controller box, readout board, ties), the two button caps (at rest, or at their end stop while
being pressed), the status lamp, the 9 readout lamps, and the three servo modules (at their sockets and at their
staging spots; the module a gripper is handling is the intended contact). Fingers are modelled at their real commanded
opening (the right gripper cannot close past the 45 mm housing it holds). The carried module is also checked against the
fixture. Arm-arm distance is checked at every step.

Intended contacts, and how they are judged:
- Button press: the fingertip is commanded deeper than the cap's 8 mm end stop; the cap is modelled AT ITS STOP, and the
  commanded penetration into it must not exceed (commanded depth - stop travel) + 0.5 mm. The fingertip must stay
  >= 1 mm above the controller box.
- Socket insertion: the module's housing front face may touch the shell mouth (hard stop) up to the commanded overshoot;
  the nose must clear the cavity walls by >= 0.9 mm (the cavity gives 1.0 mm per side).
Everything else must be >= 1 mm. Exit status is non-zero if any limit is violated.
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
ARM_ARM_MIN_MM = 15.0
NOSE_MIN_MM = 0.9           # nose to cavity walls during insertion (design clearance 1.0 mm)
PRESS_EXTRA_MM = 0.5        # tolerance on the stop-limited penetration
DT = 0.05
R_SEATED = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # servo plug frame -> world (yaw -pi/2)
COLOURS = ("red", "green", "blue")


def fixture_names(n: int) -> list[str]:
    names = ["bus strip"]
    for i in range(3):
        names += [f"socket{i} flange", f"socket{i} wall +y", f"socket{i} wall -y", f"socket{i} wall bottom",
                  f"socket{i} wall top-a", f"socket{i} wall top-b", f"socket{i} key-slot cap"]
    names += ["controller box", "readout board"] + [f"readout column dot {i}" for i in range(6)] + ["tie bar (strip)", "tie bar (readout)"]
    return names[:n] + [f"fixture box {i}" for i in range(len(names), n)]


def static_obstacles():
    """(boxes, cylinders): boxes {name: (centre, size)} and cylinders {name: (centre, radius, height)} (vertical axis)."""
    import geometry as g

    fb = g.fixture_boxes()
    names = fixture_names(len(fb))
    boxes = {n: (np.array(c), np.array(s)) for n, (c, s, _col) in zip(names, fb, strict=True)}
    boxes["table"] = (np.array([0.3, 0.0, g.TOP - 0.03]), np.array([1.1, 1.3, 0.06]))
    cyls = {}
    cyls["status lamp"] = (np.array(g.STATUS_LAMP_POS), g.STATUS_LAMP_R, g.STATUS_LAMP_H)
    i = 0
    for rname, rx in g.READOUT_ROWS.items():
        for cname, cy in g.READOUT_COLS.items():
            cyls[f"readout lamp {rname}{cname}"] = (np.array([rx, cy, g.READOUT_LAMP_Z]), g.READOUT_LAMP_R, g.READOUT_LAMP_H)
            i += 1
    return boxes, cyls


def cap_box(which: str, pressed: bool):
    import geometry as g

    pos = np.array(g.ASSIGN_BTN_POS if which == "assign" else g.RESET_BTN_POS, float)
    if pressed:
        pos[2] -= g.BUTTON_TRAVEL
    return pos, np.array(g.BUTTON_CAP_SIZE)


def servo_world_boxes(colour: str, origin):
    import geometry as g

    out = []
    for centre, size, _col in g.servo_boxes(getattr(g, colour.upper())):
        out.append((np.array(origin) + R_SEATED @ np.array(centre), np.abs(R_SEATED) @ np.array(size)))
    return out


def servo_nose_boxes(colour: str, origin):
    """Only the nose pieces and key rib (the last 4 boxes of servo_boxes), world frame."""
    return servo_world_boxes(colour, origin)[3:]


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
    """Signed distance between two axis-aligned boxes (negative = penetration depth along the shallowest axis)."""
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
        # Real commanded opening; only the right gripper is stopped by the 45 mm housing it holds.
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
    if left and ("press" in s or ("hold" in s and ("assign" in s or "reset" in s)) or ("release" in s and ("assign" in s or "reset" in s))):
        return "button press/release"
    if left and "approach" in s:
        return "button approach"
    if "wait" in s or "final hold" in s or "ready" in s or "home" in s or "park" in s:
        return "park/wait/home"
    if any(k in s for k in ("push", "to axis", "seated")):
        return "socket insertion"
    if any(k in s for k in ("grip", "pull", "descend", "lower", "release", "lift from")):
        return "servo pick/pull/stage"
    if any(k in s for k in ("hover", "lift clear", "to ")) and "home" not in s:
        return "transit/hover"
    return "park/wait/home"


def handled_colour(label: str):
    for c in COLOURS:
        if c in label:
            return c
    return None


def button_of(label: str):
    s = label.lower()
    if not s.startswith("l:"):
        return None
    if "assign" in s:
        return "assign"
    if "reset" in s:
        return "reset"
    return None


def servo_spots(right_wps):
    """{colour: [(time, 'seated'|'staged'), ...]} from the right arm's waypoint labels (pulled out -> staged until pushed in)."""
    import re

    spots = {c: [(0.0, "seated")] for c in COLOURS}
    t = 0.0
    for w in right_wps:
        t += w.duration
        m = re.match(r"r: pull (\w+) out", w.label)
        if m:
            spots[m.group(1)].append((t, "staged"))
        m = re.match(r"r: push (\w+) into socket", w.label)
        if m:
            spots[m.group(1)].append((t, "seated"))
    return spots


def spot_at(spots, colour: str, t: float) -> str:
    return [sp for tt, sp in spots[colour] if tt <= t + 1e-9][-1]


def analyse(arm: str, samples):
    """Per-label dict of worst clearances, with object names. Returns (report, bad_messages)."""
    import geometry as g
    import demo_plan

    boxes, cyls = static_obstacles()
    spots = servo_spots(demo_plan.plans()["right"])
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

        btn = button_of(label)
        pressing = btn is not None and wp_class(label) == "button press/release"
        # static boxes (caps handled below)
        for name, (c, size) in boxes.items():
            if np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi):
                continue
            consider(name, box_dist(pts, c, size))
        for name, (c, rad, h) in cyls.items():
            if np.any(c + max(rad, h / 2) < lo) or np.any(c - max(rad, h / 2) > hi):
                continue
            consider(name, cyl_dist(pts, c, rad, h))
        # caps: pressed cap is at its end stop while being pressed; penetration up to the commanded excess is intended
        for which in ("assign", "reset"):
            c, size = cap_box(which, pressed=pressing and btn == which)
            if np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi):
                continue
            d = box_dist(pts, c, size)
            if pressing and btn == which:
                # vertical depth of the lowest fingertip point over the cap footprint below the cap's top at its stop
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
                    bad.append(f"{arm} '{label}' t={s['t']:.2f}: fingertip {pen:.1f} mm below the {which} cap top at its stop (limit {lim:.1f})")
            else:
                consider(f"{which}_cap", d)
        # servos: handled one is the intended contact; others (both seated and staged spots) are obstacles
        handled = handled_colour(label) if arm == "right" else None
        for colour in COLOURS:
            if colour == handled:
                continue
            spot = spot_at(spots, colour, s["t"])
            for origin in (g.START_POS[colour] if spot == "seated" else g.STAGE_POS[colour],):
                for i, (c, size) in enumerate(servo_world_boxes(colour, origin)):
                    if np.any(c + size / 2 < lo) or np.any(c - size / 2 > hi):
                        continue
                    consider(f"servo_{colour} ({spot})", box_dist(pts, c, size))
        # carried servo vs fixture / other servos (right arm only)
        if arm == "right" and handled is not None and wp_class(label) in ("socket insertion", "servo pick/pull/stage"):
            carried = s["finger"] > -0.0226 and s["finger"] <= -0.0224  # held at the 45 mm housing
            if not label.startswith("r: descend to") or "stage" in label:
                origin = s["tcp"] - np.array([demo_plan.HOUSING_TCP_OFFSET_X, 0.0, demo_plan.PAD_TO_TCP])
                nose_gap = np.inf
                housing_gap = np.inf
                in_socket = ("push" in label or "seated" in label or label.startswith(f"r: grip {handled}") and "stage" not in label
                             or "pull" in label)
                pushing = in_socket
                for i, (c, size) in enumerate(servo_world_boxes(handled, origin)):
                    is_nose = i >= 3
                    for fname, (fc, fs) in boxes.items():
                        if fname == "table":
                            continue
                        gap = aabb_gap(c, size, fc, fs) * 1000
                        if is_nose:
                            nose_gap = min(nose_gap, gap)
                        else:
                            housing_gap = min(housing_gap, gap)
                r["extra"]["carried_nose_gap"] = min(r["extra"].get("carried_nose_gap", np.inf), nose_gap)
                r["extra"]["carried_housing_gap"] = min(r["extra"].get("carried_housing_gap", np.inf), housing_gap)
                if pushing:
                    over = demo_plan._f("DEMO_OVERSHOOT", 0.0025) * 1000 if ("push" in label or "seated" in label) else 0.0
                    if nose_gap < NOSE_MIN_MM:
                        bad.append(f"{arm} '{label}' t={s['t']:.2f}: servo_{handled} nose {nose_gap:.2f} mm from socket (min {NOSE_MIN_MM})")
                    if housing_gap < -(over + PRESS_EXTRA_MM):
                        bad.append(f"{arm} '{label}' t={s['t']:.2f}: servo_{handled} housing {-housing_gap:.1f} mm into the shell mouth (stop allows {over:.1f})")
                else:
                    gap = min(nose_gap, housing_gap)
                    if gap < MIN_CLEAR_MM:
                        bad.append(f"{arm} '{label}' t={s['t']:.2f}: carried servo_{handled} only {gap:.2f} mm from the fixture")
        # unintended-clearance rule
        if r["min"][0] < MIN_CLEAR_MM:
            bad.append(f"{arm} '{label}' t={s['t']:.2f}: {r['min'][0]:.2f} mm ({r['min'][1]}) < {MIN_CLEAR_MM} mm")
    # dedupe messages per label
    seen, uniq = set(), []
    for m in bad:
        key = m.split(" t=")[0] + m.split(": ", 1)[1][:25]
        if key not in seen:
            seen.add(key)
            uniq.append(m)
    return rep, uniq


def reachable(rep) -> bool:
    return all(r["pe"] <= 3.0 and r["re"] <= 3.0 and r["margin"] >= 0.02 for r in rep.values())


def press_summary(arm_rep):
    """Tip-to-controller-box clearance for press waypoints, computed directly (the generic minimum also contains the cap)."""
    return {k: v for k, v in arm_rep.items() if v["cls"] == "button press/release"}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--yam-arena", help="a checkout of yam-isaac-lab-arena, when yam_arena is not installed")
    a = ap.parse_args()
    if a.yam_arena:
        sys.path.insert(0, os.path.expanduser(a.yam_arena))

    from yam_arena import analysis
    import geometry as g

    t0 = time.time()
    shift_l = float(os.environ.get("DEMO_YAW_SHIFT_LEFT", 0.0))
    shift_r = float(os.environ.get("DEMO_YAW_SHIFT_RIGHT", 0.0))
    mode = next((m for m in ("DEMO_RECOVER", "DEMO_WRONG_ORDER", "DEMO_ALL_PLUGGED") if os.environ.get(m) == "1"), "demo")
    print(f"plan mode: {mode}")
    traces, problems = {}, []
    cls_worst: dict[str, tuple[float, str, str]] = {}
    for arm, shift in (("left", shift_l), ("right", shift_r)):
        traces[arm] = trace_arm(arm, shift)
        rep, bad = analyse(arm, traces[arm])
        problems += bad
        print(f"\n=== {arm} arm (yaw shift {shift:+.2f}): {'reachable' if reachable(rep) else 'NOT reachable'}; {len(traces[arm])} samples ===")
        for label, r in rep.items():
            d, what = r["min"]
            extra = ""
            if "cap_pen" in r["extra"]:
                extra = (f"  | into cap at its stop: {r['extra']['cap_pen']:.1f} mm ({r['extra']['cap_pen_who']}); "
                         f"limit {(g.BUTTON_PRESS_DEPTH_CMD - g.BUTTON_TRAVEL) * 1000 + PRESS_EXTRA_MM:.1f}")
            if "carried_nose_gap" in r["extra"]:
                extra += f"  | carried servo: nose {r['extra']['carried_nose_gap']:.2f} mm, housing {r['extra']['carried_housing_gap']:.2f} mm from fixture"
            flag = "   <-- PROBLEM" if d < MIN_CLEAR_MM else ""
            print(f"  {label:34} [{r['cls']:22}] ik {r['pe']:5.2f} mm {r['re']:5.2f} deg margin {r['margin']:5.3f} rad  "
                  f"clearance {d:7.1f} mm ({what}){extra}{flag}")
            c = cls_worst.get(f"{arm}: {r['cls']}", (np.inf, "", ""))
            if d < c[0]:
                cls_worst[f"{arm}: {r['cls']}"] = (d, what, label)
        if not reachable(rep):
            problems.append(f"{arm}: waypoints not reachable (use check_reach.py)")

    # arm-arm at every control step
    best = (np.inf, None, 0.0, "")
    n = min(len(traces["left"]), len(traces["right"]))
    for k in range(n):
        d, pair = analysis.hull_set_distance(traces["left"][k]["hulls"], traces["right"][k]["hulls"])
        if d < best[0]:
            best = (d, pair, traces["left"][k]["t"], f"{traces['left'][k]['label']} / {traces['right'][k]['label']}")
    if best[1] is None:
        # all link pairs farther than 5 cm apart (hull_set_distance skips them)
        gap_min = np.inf
        for k in range(n):
            for _na, ha in traces["left"][k]["hulls"]:
                lo_a, hi_a = ha.vertices.min(0), ha.vertices.max(0)
                for _nb, hb in traces["right"][k]["hulls"]:
                    gap_min = min(gap_min, float(np.maximum(hb.vertices.min(0) - hi_a, lo_a - hb.vertices.max(0)).max()))
        print(f"\n=== left arm vs right arm ===\n  every step checked; all link pairs > 50 mm apart (min AABB gap {gap_min * 1000:.1f} mm)")
        arm_mm = max(gap_min, 0.05) * 1000
    else:
        arm_mm = best[0] * 1000
        print(f"\n=== left arm vs right arm ===\n  closest approach {arm_mm:.1f} mm at t={best[2]:.2f}s ({best[1]}) during '{best[3]}' (every step checked)")
    if arm_mm < ARM_ARM_MIN_MM:
        problems.append(f"arm-arm clearance {arm_mm:.1f} mm < {ARM_ARM_MIN_MM}")

    print("\n=== Worst clearance per waypoint class (object names) ===")
    for k, (d, what, label) in sorted(cls_worst.items()):
        print(f"  {k:36} {d:7.1f} mm  {what}  (waypoint '{label}')")

    print(f"\nrun time {time.time() - t0:.0f} s")
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
