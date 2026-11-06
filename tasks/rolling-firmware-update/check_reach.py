#!/usr/bin/env python3
"""Offline reachability check for rolling-firmware-update.

Solves the arm IK along every waypoint of every plan and reports, per waypoint,
the worst position error (<= 3.0 mm), rotation error (<= 3.0 deg), and closest approach
to a joint limit (>= 0.02 rad). Exits with code 1 if any waypoint fails.
No simulator needed.

    python check_reach.py
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

POS_TOL_MM = 3.0
ROT_TOL_DEG = 3.0
LIMIT_MARGIN_RAD = 0.02

ALL_MODES = ["demo", "no_switch", "release_early", "door_only", "settle"]


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
    ap.add_argument("--yam-arena", help="path to yam-isaac-lab-arena checkout")
    ap.add_argument("--modes", nargs="*", default=ALL_MODES)
    a = ap.parse_args()
    if a.yam_arena:
        sys.path.insert(0, os.path.expanduser(a.yam_arena))

    from yam_arena import station
    import demo_plan

    bases = station.BASES
    bad = 0

    for mode in a.modes:
        print(f"\n=== Plan mode: {mode} ===")
        seqs = demo_plan.plans(mode)
        for arm, wps in seqs.items():
            worst = run(wps, bases[arm])
            ok = all(
                w[0] <= POS_TOL_MM and w[1] <= ROT_TOL_DEG and w[2] >= LIMIT_MARGIN_RAD
                for w in worst.values()
            )
            bad += not ok
            status = "REACHABLE" if ok else "NOT REACHABLE"
            print(f"  {arm} arm: {status}")
            for label, (pe, re, margin) in worst.items():
                flag = "" if (pe <= POS_TOL_MM and re <= ROT_TOL_DEG and margin >= LIMIT_MARGIN_RAD) else "  <-- PROBLEM"
                print(f"    {label:35} pos {pe:5.2f} mm  rot {re:5.2f} deg  margin {margin:5.3f} rad{flag}")

    if bad == 0:
        print("\nAll plans and waypoints are 100% reachable within limits!")
    else:
        print(f"\nFAILED: {bad} arm plans had reachability issues.")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
