#!/usr/bin/env python3
"""Offline score prediction for backlash-dial-setpoint.

Replays each mode's plan through the real Scorer at 20 Hz with idealized physics:
- A knob moves to its target setting during push segments.
- LOCK buttons register during press/hold segments.
- Grippers are open during the final hold.

    python predict_scores.py
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

import numpy as np

import demo_plan
import geometry as g
import logic
from scoring import Scorer

HZ = 20.0


def _segments(seqs):
    out = {}
    for arm, wps in seqs.items():
        t, rows = 0.0, []
        for w in wps:
            rows.append((t, t + w.duration, w.label))
            t += w.duration
        out[arm] = rows
    return out


def replay(mode: str, offset: tuple[float, float] = (0.0, 0.0)):
    seqs, meta = demo_plan.plans(mode, with_meta=True)
    segs = _segments(seqs)
    total_dur = max(rows[-1][1] for rows in segs.values())
    n_steps = int(round(total_dur * HZ))

    # Collect push and lock intervals
    pushes = []
    locks = {"l": [], "r": []}

    for arm, rows in segs.items():
        for t0, t1, label in rows:
            m = meta.get(label, {})
            kind = m.get("kind")
            if kind == "push_contact":
                pushes.append((t0, t1, m["side"], m["s_from"], m["s_to"]))
            elif kind == "btn_press":
                side = m["side"]
                locks[side].append((t0, t1))

    pushes.sort()
    scorer = Scorer()
    sl = g.KNOB_START_SETTING_MM["left"]
    sr = g.KNOB_START_SETTING_MM["right"]

    last_res = None
    for k in range(n_steps):
        t = k / HZ

        # Evaluate knob positions
        vel_l = np.zeros(3)
        vel_r = np.zeros(3)

        for t0, t1, side, s_from, s_to in pushes:
            if t0 <= t <= t1:
                frac = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                curr_s = s_from + frac * (s_to - s_from)
                speed = (s_to - s_from) / (t1 - t0) / 1000.0  # m/s
                if side == "l":
                    sl = curr_s
                    vel_l[0] = speed
                else:
                    sr = curr_s
                    vel_r[0] = speed
            elif t > t1:
                if side == "l":
                    sl = s_to
                else:
                    sr = s_to

        # Evaluate button states
        btn_sliders = {
            "lock_btn_left": 0.0,
            "lock_btn_right": 0.0,
        }
        for t0, t1 in locks["l"]:
            if t0 <= t <= t1:
                btn_sliders["lock_btn_left"] = -0.0075  # past -6.4 mm threshold
        for t0, t1 in locks["r"]:
            if t0 <= t <= t1:
                btn_sliders["lock_btn_right"] = -0.0075

        # Poses
        pos_l = np.array([g.knob_cx_from_setting_mm(sl + offset[0]), g.TRACK_Y["left"], g.KNOB_CZ])
        pos_r = np.array([g.knob_cx_from_setting_mm(sr + offset[1]), g.TRACK_Y["right"], g.KNOB_CZ])
        quat = np.array([0.0, 0.0, 0.0, 1.0])

        # Grippers open during final hold
        grippers_open = (t >= total_dur - 2.5 * demo_plan.DEFAULT_TIME_SCALE)

        last_res = scorer.update(
            g.BASE_POS,
            btn_sliders,
            (pos_l, quat, vel_l, np.zeros(3)),
            (pos_r, quat, vel_r, np.zeros(3)),
            grippers_open,
        )

    return last_res


def main():
    modes = ["demo", "naive", "no_lock", "settle"]
    print(f"{'Mode':12} {'Score':7} {'Success':8} {'Disp_L':8} {'Disp_R':8} {'Locked_L':9} {'Locked_R':9}")
    print("-" * 65)
    for m in modes:
        res = replay(m)
        det = res.details
        dl = f"{det['displayed_left']:.1f}"
        dr = f"{det['displayed_right']:.1f}"
        ll = f"{det['locked_in_band_left']}"
        lr = f"{det['locked_in_band_right']}"
        print(f"{m:12} {res.score:<7.2f} {str(res.success):<8} {dl:<8} {dr:<8} {ll:<9} {lr:<9}")


if __name__ == "__main__":
    main()
