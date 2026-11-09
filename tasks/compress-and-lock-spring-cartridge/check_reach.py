"""Offline kinematic reachability and joint margin audit for compress-and-lock-spring-cartridge."""

from __future__ import annotations

import argparse
import math
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if "YAM_ARENA_SRC" in os.environ:
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

import geometry as g

POS_TOL_MM, ROT_TOL_DEG, LIMIT_MARGIN_RAD = 3.0, 3.0, 0.02
MODES = ("demo", "order_bar_first_arm", "one_arm", "release_early", "bar_first", "no_release", "settle")


def check_bimanual_reach_separation():
    """Verify physical reach separation for bimanual requirement proof."""
    # Right arm to plunger
    d_right_to_plunger = math.hypot(g.PLUNGER_REST_X - g.RIGHT_BASE[0], g.PLUNGER_REST_Y - g.RIGHT_BASE[1])
    print(f"Bimanual check: Right arm to Plunger = {d_right_to_plunger:.3f} m (> {g.MAX_ARM_REACH:.2f} m unreachable)")
    assert d_right_to_plunger > g.MAX_ARM_REACH, "Right arm should not reach plunger"

    # Left arm to lock bar handle
    d_left_to_bar = math.hypot(g.LOCK_BAR_REST_X - g.LEFT_BASE[0], g.LOCK_BAR_REST_Y - g.LEFT_BASE[1])
    print(f"Bimanual check: Left arm to Lock Bar = {d_left_to_bar:.3f} m (> {g.MAX_ARM_REACH:.2f} m unreachable)")
    assert d_left_to_bar > g.MAX_ARM_REACH, "Left arm should not reach lock bar"


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
    ap.add_argument("--yam-arena", help="a checkout of yam-isaac-lab-arena")
    ap.add_argument("--modes", nargs="*", default=list(MODES), help="modes to check")
    a = ap.parse_args()
    if a.yam_arena:
        sys.path.insert(0, os.path.expanduser(a.yam_arena))

    check_bimanual_reach_separation()

    from yam_arena import station
    import demo_plan

    bases = station.BASES
    all_ok = True
    min_overall_margin = 99.0
    worst_overall_pe = 0.0
    worst_overall_re = 0.0

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
                print(f"    {label:35} pos {pe:6.2f} mm  rot {re:5.2f} deg  joint margin {margin:.3f} rad")

    print(f"\nOverall worst position error: {worst_overall_pe:.2f} mm (limit <= {POS_TOL_MM} mm)")
    print(f"Overall worst rotation error: {worst_overall_re:.2f} deg (limit <= {ROT_TOL_DEG} deg)")
    print(f"Overall minimum joint margin: {min_overall_margin:.3f} rad (limit >= {LIMIT_MARGIN_RAD} rad)")

    if all_ok:
        print("\nPASSED: All waypoints reachable with required tolerances and joint margins.")
        return 0
    else:
        print("\nFAILED: Some waypoints violated tolerances or joint limits.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
