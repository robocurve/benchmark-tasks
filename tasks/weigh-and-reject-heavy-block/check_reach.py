#!/usr/bin/env python3
"""Offline reachability check for weigh-and-reject-heavy-block:
solves arm IK along every waypoint of every plan mode and reports position/rotation errors
and joint limit margins. Verifies physical relay constraint:
- Left arm reaches supply rack and scale, cannot reach reject tray.
- Right arm reaches scale and reject tray, cannot reach supply rack.
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

POS_TOL_MM, ROT_TOL_DEG, LIMIT_MARGIN_RAD = 3.0, 3.0, 0.02
MODES = ("demo", "no_weigh", "wrong_pick", "weigh_too_short", "settle")


def run(plan_for_arm, base, dt=0.05):
    from yam_arena import ik
    from yam_arena.demo import ArmPlan

    plan = ArmPlan(list(plan_for_arm), base)
    worst: dict[str, list[float]] = {}
    t = 0.0
    while t <= plan.duration + dt:
        pos, rot, _finger, label = plan.target(t)
        plan.action7(t)
        p, r = ik.tcp_pose(plan.q, base)
        pe = float(np.linalg.norm(p - pos) * 1000)
        re = float(np.degrees(np.arccos(np.clip((np.trace(rot.T @ r) - 1) / 2, -1, 1))))
        margin = float(min((plan.q - ik.LOWER).min(), (ik.UPPER - plan.q).min()))
        w = worst.setdefault(label, [0.0, 0.0, 9.0])
        w[0], w[1], w[2] = max(w[0], pe), max(w[1], re), min(w[2], margin)
        t += dt
    return worst


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--yam-arena", help="yam_arena path override")
    ap.add_argument("--modes", nargs="*", default=list(MODES), help="modes to check")
    a = ap.parse_args()
    if a.yam_arena:
        sys.path.insert(0, os.path.expanduser(a.yam_arena))

    from yam_arena import station
    import demo_plan
    import geometry as g

    bases = station.BASES
    all_ok = True
    min_overall_margin = 99.0
    worst_overall_pe = 0.0
    worst_overall_re = 0.0

    # Verify physical relay reach distances
    base_l = np.array([0.0, +0.305, 0.75])
    base_r = np.array([0.0, -0.305, 0.75])
    tray_pos = np.array([g.TRAY_POS[0], g.TRAY_POS[1], g.TOP])
    supply_pos = np.array([g.SUPPLY_RACK_POS[0], g.SUPPLY_RACK_POS[1], g.TOP])

    dist_left_to_tray = float(np.linalg.norm(base_l - tray_pos))
    dist_right_to_supply = float(np.linalg.norm(base_r - supply_pos))
    print(f"Relay check: Left arm to Reject Tray = {dist_left_to_tray:.3f} m (> 0.50 m unreachable)")
    print(f"Relay check: Right arm to Supply Rack = {dist_right_to_supply:.3f} m (> 0.50 m unreachable)")
    assert dist_left_to_tray > 0.55, f"Left arm can reach tray: {dist_left_to_tray} m"
    assert dist_right_to_supply > 0.55, f"Right arm can reach supply rack: {dist_right_to_supply} m"

    for mode in a.modes:
        print(f"\n=== Mode: {mode} ===")
        plans = demo_plan.plans(mode)
        if not plans:
            print("  (empty plan, skipped)")
            continue
        for arm in ("left", "right"):
            wps = plans[arm]
            worst = run(wps, bases[arm])
            arm_ok = all(w[0] <= POS_TOL_MM and w[1] <= ROT_TOL_DEG and w[2] >= LIMIT_MARGIN_RAD for w in worst.values())
            all_ok &= arm_ok
            print(f"  {arm} arm: {'REACHABLE' if arm_ok else 'NOT reachable'}")
            for label, (pe, re, margin) in worst.items():
                worst_overall_pe = max(worst_overall_pe, pe)
                worst_overall_re = max(worst_overall_re, re)
                min_overall_margin = min(min_overall_margin, margin)
                flag = "" if pe <= POS_TOL_MM and re <= ROT_TOL_DEG and margin >= LIMIT_MARGIN_RAD else "   <-- problem"
                print(f"    {label:35} pos {pe:6.2f} mm  rot {re:5.2f} deg  joint margin {margin:5.3f} rad{flag}")

    print(f"\nOverall worst position error: {worst_overall_pe:.2f} mm (limit <= {POS_TOL_MM} mm)")
    print(f"Overall worst rotation error: {worst_overall_re:.2f} deg (limit <= {ROT_TOL_DEG} deg)")
    print(f"Overall minimum joint margin: {min_overall_margin:.3f} rad (limit >= {LIMIT_MARGIN_RAD} rad)")

    if all_ok:
        print("\nPASSED: All waypoints reachable with required tolerances and joint margins.")
        return 0
    else:
        print("\nFAILED: Some waypoints exceeded error tolerances or joint limits.")
        return 1


if __name__ == "__main__":
    sys.exit(main())

