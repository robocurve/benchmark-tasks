#!/usr/bin/env python3
"""Offline reachability check for Task G: solves the arm IK along every waypoint of every plan and reports, per waypoint
class, the worst position / rotation error and the smallest joint margin. Exit 1 if any sample misses 3 mm / 3 deg or comes
within 0.02 rad of a joint limit. No simulator needed.

    python check_reach.py                         # all modes and variants at the default yaw shift
    python check_reach.py --plans demo:0 --yaw 0.1
"""

from __future__ import annotations

import argparse
import os as _os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):  # one core per process: the lab PC is shared
    _os.environ.setdefault(_v, "1")
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

POS_TOL_MM, ROT_TOL_DEG, LIMIT_MARGIN_RAD = 3.0, 3.0, 0.02
ALL_PLANS = ["demo:0", "demo:1", "demo:2", "recover:0", "axis_search:0", "axis_search:1", "axis_search:2",
             "no_read:0", "memorize:0", "settle:0"]


def run(plan_for_arm, base, meta, dt=0.05):
    from yam_arena import ik
    from yam_arena.demo import ArmPlan

    plan = ArmPlan(list(plan_for_arm), base)
    worst: dict[str, list] = {}
    t = 0.0
    while t <= plan.duration + dt:
        pos, rot, _finger, label = plan.target(t)
        plan.action7(t)
        p, r = ik.tcp_pose(plan.q, base)
        pe = float(np.linalg.norm(p - pos) * 1000)
        re = float(np.degrees(np.arccos(np.clip((np.trace(rot.T @ r) - 1) / 2, -1, 1))))
        margin = float(min((plan.q - ik.LOWER).min(), (ik.UPPER - plan.q).min()))
        kind = meta.get(label, {}).get("kind", "transit")
        w = worst.setdefault(kind, [0.0, 0.0, 9.0, "", 0])
        w[0], w[1] = max(w[0], pe), max(w[1], re)
        if margin < w[2]:
            w[2], w[3] = margin, label
        w[4] += 1
        t += dt
    return worst


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--yam-arena", help="a checkout of yam-isaac-lab-arena, when yam_arena is not installed")
    ap.add_argument("--plans", nargs="*", default=ALL_PLANS)
    ap.add_argument("--yaw", type=float, default=0.0, help="yaw shift applied to both arms (default 0)")
    a = ap.parse_args()
    if a.yam_arena:
        sys.path.insert(0, os.path.expanduser(a.yam_arena))
    os.environ["DEMO_YAW_SHIFT_LEFT"] = os.environ["DEMO_YAW_SHIFT_RIGHT"] = str(a.yaw)
    import demo_plan
    import logic
    from yam_arena import station

    bad = 0
    overall = 9.0
    max_pe = max_re = 0.0
    for spec in a.plans:
        mode, v = spec.split(":")
        logic.set_variant(int(v))
        seqs, meta = demo_plan.plans(mode, int(v), with_meta=True)
        print(f"\n=== plan {spec}  yaw shift {a.yaw:+.2f} ===")
        for arm, wps in seqs.items():
            worst = run(wps, station.BASES[arm], meta)
            ok = all(w[0] <= POS_TOL_MM and w[1] <= ROT_TOL_DEG and w[2] >= LIMIT_MARGIN_RAD for w in worst.values())
            bad += not ok
            print(f"  {arm} arm: {'REACHABLE' if ok else 'NOT reachable'}")
            for kind, (pe, re, margin, label, n) in sorted(worst.items()):
                flag = "" if pe <= POS_TOL_MM and re <= ROT_TOL_DEG and margin >= LIMIT_MARGIN_RAD else "   <-- problem"
                print(f"    {kind:14} {n:4d} samples  pos {pe:6.2f} mm  rot {re:5.2f} deg  min joint margin {margin:5.3f} rad at [{label}]{flag}")
                overall, max_pe, max_re = min(overall, margin), max(max_pe, pe), max(max_re, re)
    logic.set_variant(0)
    print(f"\nOVERALL: max IK position error {max_pe:.2f} mm, max rotation error {max_re:.2f} deg, MIN JOINT MARGIN {overall:.3f} rad "
          f"({'PASS' if not bad else 'FAIL'})")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
