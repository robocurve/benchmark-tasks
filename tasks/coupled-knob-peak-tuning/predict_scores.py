#!/usr/bin/env python3
"""Offline score prediction: replays a mode's plan through the REAL Scorer at 20 Hz with idealised physics.

Idealised means: a knob jumps to its target setting (plus an optional placement offset) while its push segment runs, READ
registers while the pad is down on the cap (press and hold segments), grippers are open during the final hold. It cannot
tell us whether the real fingers, friction and arm compliance do this: that is what the GPU run is for.

    python predict_scores.py                 # table of every mode and variant, nominal and +/-1.2 mm placement offsets
"""

from __future__ import annotations

import itertools
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


def replay(mode: str, variant: int = 0, offset=(0.0, 0.0), press_ok: bool = True):
    logic.set_variant(variant)
    seqs, meta = demo_plan.plans(mode, variant, with_meta=True)
    segs = _segments(seqs)
    n = int(round(max(rows[-1][1] for rows in segs.values()) * HZ))
    # knob timeline: (t_start, t_end, side, s_from, s_to) from the 'push to' segments
    pushes = []
    for arm, rows in segs.items():
        for t0, t1, label in rows:
            m = meta.get(label)
            if m and m["kind"] == "push_contact":
                pushes.append((t0, t1, m["side"], m["s_from"], m["s_to"]))
    pushes.sort()
    scorer = Scorer()
    sl = sr = g.KNOB_START_SETTING_MM
    first_success = None
    scores, readings = [], []
    prev = None
    for k in range(n):
        t = k / HZ
        for t0, t1, side, a, b in pushes:
            if t >= t0:
                frac = min(1.0, (t - t0) / max(t1 - t0, 1e-9))
                val = a + frac * (b - a) + (offset[0] if side == "l" else offset[1]) * (1.0 if frac >= 1.0 else frac)
                if side == "l":
                    sl = val
                else:
                    sr = val
        pressed = False
        for arm, rows in segs.items():
            for t0, t1, label in rows:
                if t0 <= t < t1:
                    m = meta.get(label)
                    if m and m["kind"] == "btn_press" and ("hold btn" in label or t >= t0 + 0.15):
                        pressed = True
        # idealised: all open only when both arms are in their final hold
        opened = all(
            any(t0 <= t < t1 and "final" in label for t0, t1, label in rows) for rows in segs.values()
        )
        pos_l = (g.knob_cx_from_setting_mm(sl), g.TRACK_Y["left"], g.KNOB_CZ)
        pos_r = (g.knob_cx_from_setting_mm(sr), g.TRACK_Y["right"], g.KNOB_CZ)
        cur = np.array([pos_l[0], pos_r[0]])
        vel = np.zeros(2) if prev is None else (cur - prev) * HZ
        prev = cur
        res = scorer.update(
            g.BASE_POS,
            {"read_btn": -0.008 if (pressed and press_ok) else 0.0},
            (pos_l, None, (vel[0], 0, 0), None),
            (pos_r, None, (vel[1], 0, 0), None),
            opened,
        )
        scores.append(res.score)
        if res.details["needle_r"] != (readings[-1] if readings else None):
            readings.append(res.details["needle_r"])
        if res.success and first_success is None:
            first_success = k + 1
    return dict(
        mode=mode, variant=variant, final=scores[-1], max=max(scores), plan_steps=n,
        first_success=first_success, run_end=(first_success + int(Scorer().hold_steps) if first_success else n),
        final_settings=(round(sl, 2), round(sr, 2)), final_r=round(logic.reading(sl, sr), 4),
        presses=res.details["press_count"], separated=res.details["separated_presses"],
        needle_values=[round(x, 3) for x in readings[1:]],
    )


def main() -> None:
    cases = [("demo", 0), ("demo", 1), ("demo", 2), ("recover", 0), ("axis_search", 0), ("axis_search", 1),
             ("axis_search", 2), ("no_read", 0), ("memorize", 0), ("memorize", 1), ("memorize", 2), ("settle", 0)]
    eps = demo_plan.g.PUSH_OVERSHOOT * 1000
    print(f"{'mode':12} var  nominal(final,steps,first1.0,end)  worst/best over +/-{eps:.1f} mm offsets  final (x,y) r  presses/sep")
    for mode, v in cases:
        nom = replay(mode, v)
        finals = [replay(mode, v, offset=o)["final"] for o in itertools.product((-eps, eps), repeat=2)]
        print(f"{mode:12} {v}    {nom['final']:.2f}  plan {nom['plan_steps']}  first1.0 {nom['first_success']}  end {nom['run_end']}   "
              f"{min(finals):.2f}..{max(finals):.2f}   {nom['final_settings']} r={nom['final_r']}  {nom['presses']}/{nom['separated']}  needle {nom['needle_values']}")
    logic.set_variant(0)


if __name__ == "__main__":
    main()
