"""Offline kinematic reachability and joint margin audit for bridge-plate-across-trench."""

from __future__ import annotations

import argparse
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if "YAM_ARENA_SRC" in os.environ:
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

import geometry as g

POS_TOL_MM, ROT_TOL_DEG, LIMIT_MARGIN_RAD = 3.0, 3.0, 0.02
MODES = ("demo", "no_bridge_push", "bridge_short_overlap", "skip_target", "settle")


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

    from yam_arena import station
    import demo_plan

    all_ok = True
    overall_worst_pe = 0.0
    overall_worst_re = 0.0
    overall_min_margin = 999.0

    for mode in a.modes:
        print(f"=== mode: {mode} ===")
        plans = demo_plan.plans(mode)
        for arm in ("left", "right"):
            base = station.BASES[arm]
            w = run(plans[arm], base)
            for label, (pe, re, margin) in sorted(w.items()):
                overall_worst_pe = max(overall_worst_pe, pe)
                overall_worst_re = max(overall_worst_re, re)
                overall_min_margin = min(overall_min_margin, margin)
                bad = pe > POS_TOL_MM or re > ROT_TOL_DEG or margin < LIMIT_MARGIN_RAD
                if bad:
                    all_ok = False
                flag = " FAIL" if bad else ""
                print(f"  {arm} | {label:32s} | pe={pe:5.2f} mm | re={re:5.2f} deg | margin={margin:5.3f} rad{flag}")

    print("-" * 60)
    print(f"Worst pos err: {overall_worst_pe:.3f} mm (limit <= {POS_TOL_MM} mm)")
    print(f"Worst rot err: {overall_worst_re:.3f} deg (limit <= {ROT_TOL_DEG} deg)")
    print(f"Min joint margin: {overall_min_margin:.3f} rad (limit >= {LIMIT_MARGIN_RAD} rad)")
    print(f"Reach check result: {'PASS' if all_ok else 'FAIL'}")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
