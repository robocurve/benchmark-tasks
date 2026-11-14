"""Offline check of the demo plan: plays every mode's script against the IK (no simulator) and reports the plan's length,
the worst tool-point error, the worst rotation error and the smallest joint margin. NumPy and yam_arena's ik/station
only.

    PYTHONPATH=<yam-isaac-lab-arena checkout> python check_plan.py [--mode MODE]
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demo  # noqa: E402
from demokit import Arm  # noqa: E402

POS_TOL_MM, MARGIN_RAD = 3.0, 0.02


class _Ctx:
    def __init__(self):
        self.left, self.right = Arm("left"), Arm("right")


def check(mode: str) -> dict:
    from yam_arena import ik

    ctx = _Ctx()
    steps, margin, rot_err = 0, 9.0, 0.0
    lo, hi = ik.LOWER, ik.UPPER
    for _ in demo.script(ctx, mode):
        steps += 1
        for arm in (ctx.left, ctx.right):
            margin = min(margin, float(min((arm.q - lo).min(), (hi - arm.q).min())))
            p, r = ik.tcp_pose(arm.q, arm.base)
            rot_err = max(rot_err, float(np.degrees(np.arccos(np.clip((np.trace(arm.rot.T @ r) - 1) / 2, -1, 1)))))
    return {"mode": mode, "steps": steps, "worst_pos_mm": max(ctx.left.worst_mm, ctx.right.worst_mm),
            "worst_rot_deg": rot_err, "min_margin_rad": margin}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default=None, choices=demo.MODES)
    a = ap.parse_args()
    ok = True
    for mode in ([a.mode] if a.mode else demo.MODES):
        r = check(mode)
        bad = r["worst_pos_mm"] > POS_TOL_MM or r["min_margin_rad"] < MARGIN_RAD
        ok &= not bad
        print(f"{mode:14s} steps={r['steps']:4d}  worst pos {r['worst_pos_mm']:6.3f} mm  worst rot {r['worst_rot_deg']:5.2f} deg  "
              f"min joint margin {r['min_margin_rad']:5.3f} rad{'  FAIL' if bad else ''}")
    print("Plan check:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
