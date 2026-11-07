#!/usr/bin/env python3
"""Offline check of the scripted demo against the scene: for every waypoint it reports whether each arm can reach it,
and the smallest signed distance (mm; negative = overlap) between the arm's collision hulls and the loose block, the
table, and the other arm. No simulator needed.

    python3 check_collisions.py                     # on the VM, where yam_arena is installed
    python3 check_collisions.py --yam-arena <a checkout of yam-isaac-lab-arena>

The block is taken at its starting pose and stays there; the plug is ignored (the right hand holds it). While a gripper
is closed on something its fingers stop at that object's width (plug housing 45 mm, block width), as they do in the
simulator, not at the commanded fully-closed position. A distance under 1 mm marks a probable contact.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

WARN_MM = 1.0
POS_TOL_MM, ROT_TOL_DEG, LIMIT_MARGIN_RAD = 3.0, 3.0, 0.02


def scene_boxes():
    """The static obstacles as ``{name: (centre, rotation, size)}`` in the world frame."""
    import geometry as g

    c, s = math.cos(g.BLOCK_YAW), math.sin(g.BLOCK_YAW)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    origin = np.array(g.BLOCK_POS, float)
    boxes = {f"block box {i}": (origin + rot @ np.array(centre), rot, np.array(size)) for i, (centre, size, _c) in enumerate(g.block_boxes())}
    boxes["table"] = (np.array([0.3, 0.0, g.TOP - 0.03]), np.eye(3), np.array([1.1, 1.3, 0.06]))
    return boxes


def trace_arm(arm: str, shift: float, dt: float = 0.05):
    """Follow one arm's waypoints with the demo's IK. Returns samples ``(t, label, hulls, pos_err_mm, rot_err_deg,
    joint_margin_rad)``."""
    import demo_plan
    import geometry as g
    from yam_arena import analysis, ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
    from yam_arena.demo import ArmPlan

    os.environ["DEMO_YAW_SHIFT_LEFT" if arm == "left" else "DEMO_YAW_SHIFT_RIGHT"] = repr(shift)
    waypoints = demo_plan.plans()[arm]
    held = g.HOUSING[0] if arm == "right" else g.BLOCK_SIZE[0]
    base = station.BASES[arm]
    plan = ArmPlan(list(waypoints), base)
    out, t = [], 0.0
    while t <= plan.duration + dt:
        pos, rot, finger, label = plan.target(t)
        plan.action7(t)
        p, r = ik.tcp_pose(plan.q, base)
        pe = float(np.linalg.norm(p - pos) * 1000)
        re = float(np.degrees(np.arccos(np.clip((np.trace(rot.T @ r) - 1) / 2, -1, 1))))
        margin = float(min((plan.q - ik.LOWER).min(), (ik.UPPER - plan.q).min()))
        q = {n: float(v) for n, v in zip(ARM_JOINTS, plan.q, strict=True)}
        q.update({n: float(min(finger, -held / 2)) for n in FINGER_JOINTS})  # fingers stop at the held object
        hulls = analysis.world_hulls(analysis.fk_link_poses_world(q, base))
        out.append((t, label, hulls, pe, re, margin))
        t += dt
    return out


def scene_report(samples, boxes):
    """Per waypoint label: worst IK error, smallest joint-limit margin, closest hull to the scene (m) and what it is."""
    from yam_arena import analysis

    rep: dict[str, dict] = {}
    for t, label, hulls, pe, re, margin in samples:
        r = rep.setdefault(label, {"pe": 0.0, "re": 0.0, "margin": 9.0, "dist": np.inf, "what": "", "t": t})
        r["pe"], r["re"], r["margin"] = max(r["pe"], pe), max(r["re"], re), min(r["margin"], margin)
        for name, (centre, rot, size) in boxes.items():
            d, link = analysis.hulls_to_box(hulls, centre, rot, size)
            if d < r["dist"]:
                r["dist"], r["what"], r["t"] = d, f"{link} vs {name}", t
    return rep


def pair_closest(left, right, every: int = 2):
    """Closest approach of the two arms' hulls: ``(distance m, hull pair, time s, right waypoint label)``."""
    from yam_arena import analysis

    best = (np.inf, None, 0.0, "")
    for k in range(0, min(len(left), len(right)), every):
        d, pair = analysis.hull_set_distance(left[k][2], right[k][2])
        if d < best[0]:
            best = (d, pair, left[k][0], right[k][1])
    return best


def reachable(rep) -> bool:
    return all(r["pe"] <= POS_TOL_MM and r["re"] <= ROT_TOL_DEG and r["margin"] >= LIMIT_MARGIN_RAD for r in rep.values())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--yam-arena", help="a checkout of yam-isaac-lab-arena, when yam_arena is not installed")
    a = ap.parse_args()
    if a.yam_arena:
        sys.path.insert(0, os.path.expanduser(a.yam_arena))
    import demo_plan

    boxes = scene_boxes()
    traces, bad = {}, 0
    for arm in ("left", "right"):
        traces[arm] = trace_arm(arm, demo_plan.SHIFT_DEFAULT[arm])
        rep = scene_report(traces[arm], boxes)
        print(f"\n=== {arm} arm: {'reachable' if reachable(rep) else 'NOT reachable'} ===")
        for label, r in rep.items():
            mm = r["dist"] * 1000
            flags = []
            if not (r["pe"] <= POS_TOL_MM and r["re"] <= ROT_TOL_DEG and r["margin"] >= LIMIT_MARGIN_RAD):
                flags.append("reach")
            holding = arm == "left" and "block" in label and "to the block" not in label  # the grip itself touches
            if mm < -0.5 or (mm < WARN_MM and not holding):
                flags.append("CONTACT" if mm < 0 else "under 1 mm")
            bad += bool(flags)
            print(f"  {label:30} ik {r['pe']:5.2f} mm {r['re']:5.2f} deg  joint margin {r['margin']:5.3f}  "
                  f"nearest {mm:7.1f} mm ({r['what']}){'   <-- ' + ', '.join(flags) if flags else ''}")
    d, pair, t, label = pair_closest(traces["left"], traces["right"])
    print(f"\n=== left arm vs right arm ===\n  closest approach {d * 1000:.1f} mm at t={t:.2f}s during '{label}' ({pair})")
    bad += d * 1000 < WARN_MM
    print(f"\n{'PROBLEMS FOUND' if bad else 'no contacts predicted, both arms reach every waypoint'}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
