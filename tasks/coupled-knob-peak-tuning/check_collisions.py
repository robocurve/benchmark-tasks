#!/usr/bin/env python3
"""Offline collision check of EVERY waypoint of every plan of Task G (no label filters).

For each plan (mode and variant) and each arm it samples the plan at 20 Hz, models the arm hulls with the fingers at the
commanded opening, and measures the signed distance to every object: each fixture box (named), both knobs at their nominal
settings for that moment, the READ cap, a needle sweep volume, the table and the other arm.

Per waypoint class it prints the worst clearance, the object and the arm link. Exit code 1 if
  * an UNINTENDED clearance is below 1.0 mm,
  * an INTENDED contact is deeper than its stroke allows (knob push: PUSH_OVERSHOOT + 0.8 mm; READ press: the commanded
    press overshoot + 0.5 mm; the cap itself is then at its end stop),
  * the arm-arm distance is below 5 mm, or a waypoint is not reachable (IK error or joint margin).
Intended contacts: the pushed knob during 'push to' and 'lift push'; the READ cap during 'press btn', 'hold btn' and
'release btn'; the arm base link against the table (permanent, constant, printed once).

    python check_collisions.py                    # all plans, 3 worker processes
    python check_collisions.py --plans demo:0 no_read:0 --jobs 1
"""

from __future__ import annotations

import argparse
import os as _os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):  # one core per process: the lab PC is shared
    _os.environ.setdefault(_v, "1")
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

MIN_CLEAR_MM = 1.0
ARM_ARM_MIN_MM = 5.0
POS_TOL_MM, ROT_TOL_DEG, LIMIT_MARGIN_RAD = 3.0, 3.0, 0.02
ALL_PLANS = ["demo:0", "demo:1", "demo:2", "recover:0", "axis_search:0", "axis_search:1", "axis_search:2",
             "no_read:0", "memorize:0", "settle:0"]
INTENDED_KINDS = {"push_contact": "knob", "push_lift": "knob", "btn_press": "cap"}
CLASS_ORDER = ["push_hover", "push_descend", "push_contact", "push_lift", "btn_hover", "btn_press", "transit", "wait", "hold"]


def static_boxes():
    import geometry as g

    boxes = {name: (np.array(c), np.eye(3), np.array(sz)) for name, c, sz, _col in g.fixture_named_boxes_world()}
    boxes["table"] = (np.array([0.3, 0.0, g.TOP - 0.03]), np.eye(3), np.array([1.1, 1.3, 0.06]))
    # needle: anywhere along the scale bar, 34 x 12 x 8 mm above it
    boxes["needle sweep volume"] = (np.array([g.NEEDLE_CX, 0.0, g.NEEDLE_CZ]), np.eye(3), np.array([g.NEEDLE_LX, 0.172, g.NEEDLE_LZ]))
    return boxes


def knob_boxes(settings):
    import geometry as g

    out = {}
    for side, s in zip(("left", "right"), settings, strict=True):
        out[f"knob {side}"] = (np.array([g.knob_cx_from_setting_mm(s), g.TRACK_Y[side], g.KNOB_CZ]), np.eye(3), np.array(g.KNOB_SIZE))
    return out


def cap_box(pressed: bool):
    import geometry as g

    z = g.BUTTON_CAP_REST_CZ - (g.BUTTON_TRAVEL if pressed else 0.0)
    return {"READ cap": (np.array([g.BUTTON_BASE_CX, g.BUTTON_BASE_CY, z]), np.eye(3), np.array(g.BUTTON_CAP_SIZE))}


_FK_CACHE: dict = {}


def trace_arm(waypoints, arm, base, dt=0.05):
    from yam_arena import analysis, ik
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
    from yam_arena.demo import ArmPlan

    plan = ArmPlan(list(waypoints), base)
    out, t = [], 0.0
    while t <= plan.duration + dt:
        pos, rot, finger, label = plan.target(t)
        plan.action7(t)
        p, r = ik.tcp_pose(plan.q, base)
        pe = float(np.linalg.norm(p - pos) * 1000)
        re = float(np.degrees(np.arccos(np.clip((np.trace(rot.T @ r) - 1) / 2, -1, 1))))
        margin = float(min((plan.q - ik.LOWER).min(), (ik.UPPER - plan.q).min()))
        key = (arm, tuple(np.round(plan.q, 6)), round(float(finger), 6))
        if key not in _FK_CACHE:
            q = {n: float(v) for n, v in zip(ARM_JOINTS, plan.q, strict=True)}
            q.update({n: float(finger) for n in FINGER_JOINTS})  # fingers at the commanded opening
            hulls = analysis.world_hulls(analysis.fk_link_poses_world(q, base))
            V = np.vstack([h.vertices for _n, h in hulls])
            L = np.concatenate([[n] * len(h.vertices) for n, h in hulls])
            fine = [h for n, h in hulls if n in FINE_LINKS]
            _FK_CACHE[key] = (hulls, V, L, fine)
        hulls, V, L, fine = _FK_CACHE[key]
        out.append((t, label, hulls, (V, fine), L, pe, re, margin))
        t += dt
    return out


FINE_LINKS = ("tip_left", "tip_right", "gripper", "link5")
SAMPLE_MM = 1.5


def _surface_points(c, size, lo, hi, h):
    """Points on the box surface (axis-aligned box) inside the axis-aligned region [lo, hi], spacing h."""
    b_lo, b_hi = c - size / 2, c + size / 2
    lo2, hi2 = np.maximum(b_lo, lo), np.minimum(b_hi, hi)
    if np.any(hi2 < lo2):
        return np.zeros((0, 3))
    pts = []
    for a in range(3):
        o = [i for i in range(3) if i != a]
        g1 = np.arange(lo2[o[0]], hi2[o[0]] + h / 2, h)
        g2 = np.arange(lo2[o[1]], hi2[o[1]] + h / 2, h)
        G1, G2 = np.meshgrid(g1, g2)
        for face in (b_lo[a], b_hi[a]):
            if lo[a] <= face <= hi[a]:
                P = np.zeros((G1.size, 3))
                P[:, a], P[:, o[0]], P[:, o[1]] = face, G1.ravel(), G2.ravel()
                pts.append(P)
    return np.vstack(pts) if pts else np.zeros((0, 3))


def nearest(VF, L, boxes):
    """(distance m, object name, link) of the closest box: hull vertices against the box, and (for boxes within 15 mm of
    the fingers / wrist) box surface samples against the finger and wrist hulls, so an edge poking between hull vertices
    (the knob's top edge against the slanted finger face) is not missed."""
    from yam_arena import analysis

    V, fine = VF if isinstance(VF, tuple) else (VF, [])
    best = (np.inf, "", "")
    if fine:
        FV = np.vstack([h.vertices for h in fine])
        flo, fhi = FV.min(0) - 0.015, FV.max(0) + 0.015
    for name, (c, rot, size) in boxes.items():
        lo, hi = V.min(0), V.max(0)
        if np.any(np.maximum(c - size / 2 - hi, lo - (c + size / 2)) > 0.05):
            continue  # box (axis-aligned) more than 5 cm from the whole arm: no contact candidate
        s = analysis.box_signed_distance(V, c, rot, size)
        i = int(np.argmin(s))
        if s[i] < best[0]:
            best = (float(s[i]), name, str(L[i]))
        if fine:
            P = _surface_points(c, size, flo, fhi, SAMPLE_MM / 1000.0)
            if len(P):
                sm = min(float(analysis._signed(P, h).min()) for h in fine)
                if sm < best[0] and sm < 0.003:
                    best = (sm, name, "finger/wrist hull (box edge sampled)")
    return best


def check_plan(spec: str, bases) -> tuple[int, list[str]]:
    import demo_plan
    import geometry as g
    import logic
    from yam_arena import analysis

    mode, variant = spec.split(":")
    variant = int(variant)
    logic.set_variant(variant)
    seqs, meta = demo_plan.plans(mode, variant, with_meta=True)
    fixed = static_boxes()
    lines: list[str] = []
    bad = 0
    traces = {}
    base_table_const = []
    worst: dict[tuple[str, str], tuple] = {}  # (arm, class) -> (mm, object, link, label)
    intended: dict[tuple[str, str], tuple] = {}  # (arm, 'knob'|'cap') -> (max penetration mm, label)
    reach_bad = []

    for arm in ("left", "right"):
        side = "l" if arm == "left" else "r"
        traces[arm] = trace_arm(seqs[arm], arm, bases[arm])
        for t, label, hulls, V, L, pe, re, margin in traces[arm]:
            m = meta.get(label, {})
            kind = m.get("kind", "transit")
            if not (pe <= POS_TOL_MM and re <= ROT_TOL_DEG and margin >= LIMIT_MARGIN_RAD):
                reach_bad.append(f"{label} ik {pe:.2f} mm {re:.2f} deg margin {margin:.3f}")
            before, after = m.get("before", (30.0, 30.0)), m.get("after", (30.0, 30.0))
            pushed = m.get("pushed")
            boxes = dict(fixed)
            knobs_b, knobs_a = knob_boxes(before), knob_boxes(after)
            pushed_name = {"l": "knob left", "r": "knob right"}.get(pushed)
            for name in knobs_b:
                boxes[name] = knobs_a[name] if kind in ("push_contact", "push_lift") else knobs_b[name]
            cap_pressed = cap_box(True)["READ cap"]
            cap_rest = cap_box(False)["READ cap"]
            unintended = dict(boxes)
            unintended["READ cap"] = cap_rest
            if kind in ("push_contact", "push_lift") and pushed_name:
                pen_boxes = {pushed_name: unintended.pop(pushed_name)}
                d, name, link = nearest(V, L, pen_boxes)
                key = (arm, "knob")
                depth = max(0.0, -d * 1000)
                if depth > intended.get(key, (-1, ""))[0]:
                    intended[key] = (depth, f"{label} (t={t:.2f}s, {link} into {name})")
            if kind == "btn_press" or "release btn" in label:
                unintended.pop("READ cap", None)
                if "hold btn" in label:
                    d, name, link = nearest(V, L, {"READ cap (pressed, end stop)": cap_pressed})
                    depth = max(0.0, -d * 1000)
                    key = (arm, "cap")
                    if depth > intended.get(key, (-1, ""))[0]:
                        intended[key] = (depth, f"{label} (t={t:.2f}s, {link})")
            # arm base link against the table is permanent: record it once, exclude it from the minimum
            Vb = V[0][L != "base"]
            Lb = L[L != "base"]
            d, name, link = nearest((Vb, V[1]), Lb, unintended)
            cls = kind if kind in CLASS_ORDER else "transit"
            if "hold" in kind or "settle" in label:
                cls = "hold"
            k = (arm, cls)
            if d * 1000 < worst.get(k, (np.inf,))[0]:
                worst[k] = (d * 1000, name, link, f"{label} (t={t:.2f}s)")
            if len(base_table_const) < 1 and (V[0][L == "base"].size):
                dd, nn, ll = nearest(V[0][L == "base"], L[L == "base"], {"table": fixed["table"]})
                base_table_const.append(dd * 1000)

    lines.append(f"\n===== plan {spec}  ({sum(w.duration for w in seqs['left']):.1f} s left, {sum(w.duration for w in seqs['right']):.1f} s right) =====")
    if reach_bad:
        bad += len(reach_bad)
        lines.append(f"  NOT REACHABLE ({len(reach_bad)}): " + "; ".join(reach_bad[:5]))
    else:
        lines.append("  every waypoint sample reachable (IK <= 3 mm / 3 deg, joint margin >= 0.02 rad)")
    for arm in ("left", "right"):
        for cls in CLASS_ORDER:
            if (arm, cls) in worst:
                mm, name, link, label = worst[(arm, cls)]
                flag = ""
                if mm < MIN_CLEAR_MM:
                    flag = "   <-- UNINTENDED CLEARANCE BELOW 1 mm"
                    bad += 1
                lines.append(f"  {arm:5} {cls:13} worst {mm:7.1f} mm  {link} vs {name}  [{label}]{flag}")
    pen_limit = {"knob": g.PUSH_OVERSHOOT * 1000 + 0.8, "cap": g.BUTTON_PRESS_OVERSHOOT * 1000 + 0.5}
    for (arm, what), (depth, where) in sorted(intended.items()):
        flag = ""
        if depth > pen_limit[what]:
            flag = f"   <-- DEEPER THAN ALLOWED ({pen_limit[what]:.1f} mm)"
            bad += 1
        lines.append(f"  {arm:5} intended {what:5} penetration {depth:5.2f} mm (limit {pen_limit[what]:.1f})  {where}{flag}")
    if base_table_const:
        lines.append(f"  (arm base link vs table: constant {base_table_const[0]:.1f} mm, permanent mount, excluded from the minimum)")
    # arm-arm
    best = (np.inf, None, 0.0, "")
    n = min(len(traces["left"]), len(traces["right"]))
    for k in range(0, n, 2):
        d, pair = analysis.hull_set_distance(traces["left"][k][2], traces["right"][k][2])
        if d < best[0]:
            best = (d, pair, traces["left"][k][0], traces["left"][k][1])
    flag = ""
    if best[0] * 1000 < ARM_ARM_MIN_MM:
        flag = "   <-- ARM-ARM BELOW 5 mm"
        bad += 1
    lines.append(f"  arm vs arm closest {best[0] * 1000:.1f} mm at t={best[2]:.2f}s {best[1]} [{best[3]}]{flag}")
    logic.set_variant(0)
    return bad, lines


def _worker(spec):
    from yam_arena import station

    return spec, check_plan(spec, station.BASES)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--yam-arena", help="a checkout of yam-isaac-lab-arena, when yam_arena is not installed")
    ap.add_argument("--plans", nargs="*", default=ALL_PLANS, help="mode:variant, e.g. demo:0 axis_search:1")
    ap.add_argument("--jobs", type=int, default=3)
    a = ap.parse_args()
    if a.yam_arena:
        sys.path.insert(0, os.path.expanduser(a.yam_arena))
    total = 0
    results = {}
    if a.jobs > 1 and len(a.plans) > 1:
        with ProcessPoolExecutor(max_workers=a.jobs) as ex:
            for spec, res in ex.map(_worker, a.plans):
                results[spec] = res
    else:
        for spec in a.plans:
            results[spec] = _worker(spec)[1]
    for spec in a.plans:
        bad, lines = results[spec]
        total += bad
        print("\n".join(lines), flush=True)
    print("\n" + "=" * 60)
    if total:
        print(f"PROBLEMS FOUND: {total} issue(s) across {len(a.plans)} plans.")
        return 1
    print(f"SUCCESS: {len(a.plans)} plans: all reachable, unintended clearances >= {MIN_CLEAR_MM} mm, intended contacts within stroke, arm-arm >= {ARM_ARM_MIN_MM} mm.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
