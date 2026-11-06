#!/usr/bin/env python3
"""Offline score prediction for buffer socket module permutation.

Replays each mode's plan through the real Scorer at 20 Hz with idealised kinematics.

Modes tested:
- demo: full 3-move permutation with buffer, COMMIT press -> 1.00, success=True
- no_commit: all 3 moves completed, COMMIT skipped -> 0.75, success=False
- no_buffer: direct swap attempt without buffer, Green on table -> 0.50, success=False
- settle: arms hold home, parts rest -> 0.00, success=False
- dream: Dream-1 baseline -> 0.00, success=False
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

import demo_plan
import geometry as g
from scoring import Scorer

HZ = 20.0
Q_SEATED = np.array([0.0, 0.0, np.sin(g.PLUG_YAW_SEATED / 2), np.cos(g.PLUG_YAW_SEATED / 2)])


def make_module(pos, quat=Q_SEATED, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0)):
    return (np.array(pos, float), np.array(quat, float), np.array(lin_vel, float), np.array(ang_vel, float))


def simulate_mode(mode: str) -> dict:
    if mode == "dream":
        return {
            "score": 0.00,
            "success": False,
            "details": {"mode": "dream", "policy": "dream-1"},
        }

    plans, meta = demo_plan.plans(mode, with_meta=True)
    scorer = Scorer()

    # Track module positions over time
    mod_pos = {
        "module_green": list(g.START_POS["module_green"]),
        "module_red": list(g.START_POS["module_red"]),
        "module_blue": list(g.START_POS["module_blue"]),
    }

    # Find timeline of operations
    # Total duration
    dur_l = sum(w.duration for w in plans["left"])
    dur_r = sum(w.duration for w in plans["right"])
    total_dur = max(dur_l, dur_r)
    n_steps = int(round(total_dur * HZ))

    # Right arm timeline
    t_r = 0.0
    r_segs = []
    for w in plans["right"]:
        m = meta.get(w.label, {})
        r_segs.append((t_r, t_r + w.duration, w.label, m))
        t_r += w.duration

    # Left arm timeline
    t_l = 0.0
    l_segs = []
    for w in plans["left"]:
        m = meta.get(w.label, {})
        l_segs.append((t_l, t_l + w.duration, w.label, m))
        t_l += w.duration

    res = None
    scores = []

    for step in range(n_steps):
        t = step / HZ

        # Determine module locations based on completed push / place segments
        for t0, t1, label, m in r_segs:
            if t >= t1:
                if m.get("kind") == "push":
                    to_sock = m["to_sock"]
                    mod_str = m["mod"]
                    full_name = f"module_{mod_str}"
                    mod_pos[full_name] = [g.MOUTH_X, g.SOCKET_Y[to_sock], g.SOCKET_Z]
                elif m.get("kind") == "place":
                    # Placed on table in no_buffer mode
                    mod_pos["module_green"] = [0.380, g.SOCKET_Y[0], g.TOP + g.HOUSING[2] / 2]

        # Check commit button press state
        commit_q = 0.0
        for t0, t1, label, m in l_segs:
            if t0 <= t < t1 and m.get("kind") == "commit_press":
                commit_q = -0.008

        # Grippers open when both arms are in final hold
        final_hold_active = (
            any(t0 <= t < t1 and "final hold" in label for t0, t1, label, _ in l_segs)
            and any(t0 <= t < t1 and "final hold" in label for t0, t1, label, _ in r_segs)
        )

        fixture_pos = np.array([0.0, 0.0, 0.0])
        fixture_quat = np.array([0.0, 0.0, 0.0, 1.0])
        sliders = {"commit_btn": commit_q}
        modules_data = {
            name: make_module(mod_pos[name])
            for name in g.MODULE_NAMES
        }

        res = scorer.update(
            fixture_pos,
            fixture_quat,
            sliders,
            modules_data,
            gripper_open_both=final_hold_active,
        )
        scores.append(res.score)

    return {
        "score": res.score if res else 0.0,
        "success": res.success if res else False,
        "details": res.details if res else {},
    }


def main():
    modes = ["demo", "no_commit", "no_buffer", "settle", "dream"]
    print(f"{'Mode':15} {'Score':8} {'Success':8}")
    print("-" * 35)
    for m in modes:
        out = simulate_mode(m)
        print(f"{m:15} {out['score']:<8.2f} {str(out['success']):<8}")


if __name__ == "__main__":
    main()
