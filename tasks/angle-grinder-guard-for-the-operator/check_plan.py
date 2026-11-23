"""Offline reach check of the scripted demo: drains demo.script with the real IK (no simulator) and reports, per mode and
layout, the worst tool-point error, the worst rotation error and the smallest joint margin of each arm.

    python check_plan.py [--modes demo ...] [--variants A B]     # needs yam_arena importable (numpy only)

Pass: every waypoint solved within 3 mm and 3 degrees, every joint at least 0.02 rad from its limit.
"""

from __future__ import annotations

import argparse
import math
import sys
from types import SimpleNamespace

import numpy as np

import demo

POS_TOL_MM, ROT_TOL_DEG, MARGIN_RAD = 3.0, 3.0, 0.02


def drain(mode: str, variant: str) -> dict:
    from yam_arena import ik

    from demokit import Arm

    ctx = SimpleNamespace(left=Arm("left"), right=Arm("right"))
    out = {"steps": 0}
    worst = {s: dict(pe=0.0, re=0.0, margin=9.0, where="") for s in ("left", "right")}
    label = ""
    for _ in demo.script(ctx, mode, variant):
        out["steps"] += 1
        for side, arm in (("left", ctx.left), ("right", ctx.right)):
            p, r = ik.tcp_pose(arm.q, arm.base)
            pe = float(np.linalg.norm(p - arm.pos) * 1000)
            re = float(np.degrees(np.arccos(np.clip((np.trace(arm.rot.T @ r) - 1) / 2, -1, 1))))
            margin = float(min((arm.q - ik.LOWER).min(), (ik.UPPER - arm.q).min()))
            w = worst[side]
            if pe > w["pe"]:
                w["pe"] = pe
            w["re"] = max(w["re"], re)
            if margin < w["margin"]:
                w["margin"], w["where"] = margin, f"step {out['steps']} ({out['steps'] / 20:.1f} s)"
    out["worst"] = worst
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--modes", nargs="*", default=list(demo.MODES))
    ap.add_argument("--variants", nargs="*", default=["A", "B"])
    a = ap.parse_args()
    ok = True
    pe_all = re_all = 0.0
    margin_all = 9.0
    for v in a.variants:
        for mode in a.modes:
            r = drain(mode, v)
            line = [f"layout {v} {mode:12s} {r['steps']:5d} steps ({r['steps'] / 20:5.1f} s)"]
            for side in ("left", "right"):
                w = r["worst"][side]
                good = w["pe"] <= POS_TOL_MM and w["re"] <= ROT_TOL_DEG and w["margin"] >= MARGIN_RAD
                ok &= good
                pe_all, re_all, margin_all = max(pe_all, w["pe"]), max(re_all, w["re"]), min(margin_all, w["margin"])
                line.append(f"{side}: pos {w['pe']:.2f} mm rot {w['re']:.2f} deg margin {w['margin']:.3f} rad"
                            f"{'' if good else '  <-- FAIL'}")
            print("  ".join(line))
    print(f"\nWorst position error {pe_all:.2f} mm (limit {POS_TOL_MM}), worst rotation error {re_all:.2f} deg "
          f"(limit {ROT_TOL_DEG}), smallest joint margin {margin_all:.3f} rad (limit {MARGIN_RAD})")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
