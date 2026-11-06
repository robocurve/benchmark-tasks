"""Offline kinematic reachability and joint margin audit for hot-swap-failed-drive-sled."""

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
MODES = ("demo", "no_latch", "wrong_bay", "wrong_spare", "release_early", "settle")


def check_bimanual_relay_reach():
    """Each arm can only reach its own side: the discard pad is the right arm's, the spare stand the left arm's."""
    d_left_to_discard = math.hypot(g.DISCARD_SPOT[0] - g.LEFT_BASE[0], g.DISCARD_SPOT[1] - g.LEFT_BASE[1])
    d_right_to_spare = math.hypot(g.SPARE_POS["blue"][0] - g.RIGHT_BASE[0], g.SPARE_POS["blue"][1] - g.RIGHT_BASE[1])
    d_right_to_discard = math.hypot(g.DISCARD_SPOT[0] - g.RIGHT_BASE[0], g.DISCARD_SPOT[1] - g.RIGHT_BASE[1])
    d_left_to_spare = math.hypot(g.SPARE_POS["blue"][0] - g.LEFT_BASE[0], g.SPARE_POS["blue"][1] - g.LEFT_BASE[1])
    print(f"Relay check: Left arm to Discard Pad = {d_left_to_discard:.3f} m (> {g.MAX_ARM_REACH:.2f} m unreachable)")
    print(f"Relay check: Right arm to Spare Stand = {d_right_to_spare:.3f} m (> {g.MAX_ARM_REACH:.2f} m unreachable)")
    print(f"Relay check: Right arm to Discard Pad = {d_right_to_discard:.3f} m, Left arm to Spare Stand = {d_left_to_spare:.3f} m (reachable)")
    assert d_left_to_discard > g.MAX_ARM_REACH, "Left arm should not reach Discard Pad"
    assert d_right_to_spare > g.MAX_ARM_REACH, "Right arm should not reach Spare Stand"
    assert d_right_to_discard < g.MAX_ARM_REACH and d_left_to_spare < g.MAX_ARM_REACH


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

    check_bimanual_relay_reach()

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
