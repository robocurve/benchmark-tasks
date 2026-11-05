#!/usr/bin/env python3
"""Offline check that both arms can follow the scripted demo for Task 4: solves the arm IK along every
waypoint and reports the worst position / rotation error and the closest approach to a joint limit, per waypoint.
No simulator needed.
"""

from __future__ import annotations

import argparse
import itertools
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

POS_TOL_MM, ROT_TOL_DEG, LIMIT_MARGIN_RAD = 3.0, 3.0, 0.02


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
    ap.add_argument("--yam-arena", help="a checkout of yam-isaac-lab-arena, when yam_arena is not installed")
    a = ap.parse_args()
    if a.yam_arena:
        sys.path.insert(0, os.path.expanduser(a.yam_arena))
    from yam_arena import station

    import demo_plan

    bases = station.BASES
    bad = 0
    combinations_reachable = []
    orig_l = os.environ.get("DEMO_YAW_SHIFT_LEFT")
    orig_r = os.environ.get("DEMO_YAW_SHIFT_RIGHT")
    try:
        for sl, sr in itertools.product((0.0, -math.pi), repeat=2):
            os.environ["DEMO_YAW_SHIFT_LEFT"], os.environ["DEMO_YAW_SHIFT_RIGHT"] = str(sl), str(sr)
            plans = demo_plan.plans()
            print(f"\n=== left yaw shift {sl:+.2f}, right yaw shift {sr:+.2f} ===")
            ok_all = True
            for arm, wps in plans.items():
                worst = run(wps, bases[arm])
                ok = all(w[0] <= POS_TOL_MM and w[1] <= ROT_TOL_DEG and w[2] >= LIMIT_MARGIN_RAD for w in worst.values())
                ok_all &= ok
                print(f"  {arm} arm: {'REACHABLE' if ok else 'NOT reachable'}")
                for label, (pe, re, margin) in worst.items():
                    flag = "" if pe <= POS_TOL_MM and re <= ROT_TOL_DEG and margin >= LIMIT_MARGIN_RAD else "   <-- problem"
                    print(f"    {label:35} pos {pe:6.2f} mm  rot {re:5.2f} deg  joint margin {margin:5.3f} rad{flag}")
            if ok_all:
                combinations_reachable.append((sl, sr))
            bad += not ok_all
    finally:
        if orig_l is None:
            os.environ.pop("DEMO_YAW_SHIFT_LEFT", None)
        else:
            os.environ["DEMO_YAW_SHIFT_LEFT"] = orig_l
        if orig_r is None:
            os.environ.pop("DEMO_YAW_SHIFT_RIGHT", None)
        else:
            os.environ["DEMO_YAW_SHIFT_RIGHT"] = orig_r

    print(f"\nReachable combinations: {combinations_reachable}")
    return 0 if combinations_reachable else 1


if __name__ == "__main__":
    sys.exit(main())
