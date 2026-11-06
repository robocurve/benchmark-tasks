"""Predict offline scores of every run.sh mode by replaying the plan through Scorer. Pure Python.

Simulates waypoint kinematics and feeds them at 20 Hz into Scorer:
- demo: 1.00
- naive: 0.75 (core short of target band)
- overshoot: 0.00 (trips FIRE zero-gate > 90 C)
- no_sample1: 0.75 (sample 1 milestone omitted)
- settle: 0.00
- dream: 0.00 (baseline)
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import demo_plan
import geometry as g
from scoring import Scorer
from yam_arena.constants import FINGER_OPEN


def simulate_mode(mode: str) -> dict:
    scorer = Scorer()
    p = demo_plan.plans(mode)

    if not p:
        # Settle mode: 400 steps of parts resting at nominal start
        s1_data = (g.START_POS_1, (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
        s2_data = (g.START_POS_2, (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
        for _ in range(400):
            res = scorer.update((0.0, 0.0, 0.0), {}, s1_data, s2_data, True)
        return {"score": res.score, "success": res.success, "details": res.details}

    dt = 0.05
    s1_plate_pos = (g.PAD_POS[0], g.PAD_POS[1], g.PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0)
    s2_plate_pos = (g.PAD_POS[0], g.PAD_POS[1], g.PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0)
    s1_cool_pos = (g.COOL_SPOT_1[0], g.COOL_SPOT_1[1], g.COOL_PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0)
    s2_cool_pos = (g.COOL_SPOT_2[0], g.COOL_SPOT_2[1], g.COOL_PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0)
    air_pos = (0.28, 0.0, g.TOP + 0.16)

    # Pre-compute exact event timestamps for sample locations
    # S1 transitions:
    # 1. on plate: when left arm releases s1 on plate
    # 2. lifted from plate: when right arm lifts s1 from plate
    # 3. on cool pad: when right arm releases s1 on cool pad
    t_s1_plate = math.inf
    t_s1_lift = math.inf
    t_s1_cool = math.inf

    t = 0.0
    for w in p["left"]:
        if "release s1 on plate" in w.label:
            t_s1_plate = t + w.duration
        t += w.duration

    t = 0.0
    for w in p["right"]:
        if "lift s1 from plate" in w.label:
            t_s1_lift = t
        elif "release s1 on cool" in w.label:
            t_s1_cool = t + w.duration
        t += w.duration

    # S2 transitions:
    # 1. on plate: when left arm releases s2 on plate
    # 2. lifted from plate: when right arm lifts s2 from plate
    # 3. on cool pad: when right arm releases s2 on cool pad
    t_s2_plate = math.inf
    t_s2_lift = math.inf
    t_s2_cool = math.inf

    t = 0.0
    for w in p["left"]:
        if "release s2 on plate" in w.label:
            t_s2_plate = t + w.duration
        t += w.duration

    t = 0.0
    for w in p["right"]:
        if "lift s2 from plate" in w.label:
            t_s2_lift = t
        elif "release s2 on cool" in w.label:
            t_s2_cool = t + w.duration
        t += w.duration

    # Replay waypoint sequence
    duration = sum(w.duration for w in p["left"])
    steps = int(duration / dt)

    wps_left = p["left"]
    wps_right = p["right"]

    idx_l = 0
    t_l = 0.0
    idx_r = 0
    t_r = 0.0

    last_res = None

    for step in range(steps):
        sim_t = step * dt

        while idx_l < len(wps_left) and sim_t >= t_l + wps_left[idx_l].duration:
            t_l += wps_left[idx_l].duration
            idx_l += 1
        wp_l = wps_left[min(idx_l, len(wps_left) - 1)]

        while idx_r < len(wps_right) and sim_t >= t_r + wps_right[idx_r].duration:
            t_r += wps_right[idx_r].duration
            idx_r += 1
        wp_r = wps_right[min(idx_r, len(wps_right) - 1)]

        # Button slider displacements
        heat_z = 0.0
        off_z = 0.0
        if "press HEAT" in wp_r.label or "hold HEAT" in wp_r.label:
            heat_z = -0.007
        if "press OFF" in wp_r.label or "hold OFF" in wp_r.label:
            off_z = -0.007

        # S1 position at sim_t
        if sim_t < t_s1_plate:
            cur_s1 = g.START_POS_1
        elif sim_t < t_s1_lift:
            cur_s1 = s1_plate_pos
        elif sim_t < t_s1_cool:
            cur_s1 = air_pos
        else:
            cur_s1 = s1_cool_pos

        # S2 position at sim_t
        if sim_t < t_s2_plate:
            cur_s2 = g.START_POS_2
        elif sim_t < t_s2_lift:
            cur_s2 = s2_plate_pos
        elif sim_t < t_s2_cool:
            cur_s2 = air_pos
        else:
            cur_s2 = s2_cool_pos

        g_open = abs(wp_l.finger - FINGER_OPEN) < 1e-3 and abs(wp_r.finger - FINGER_OPEN) < 1e-3

        s1_data = (cur_s1, (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
        s2_data = (cur_s2, (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))

        last_res = scorer.update((0.0, 0.0, 0.0), {"heat_btn": heat_z, "off_btn": off_z}, s1_data, s2_data, g_open, dt=dt)

    return {"score": last_res.score, "success": last_res.success, "details": last_res.details}


def main():
    modes = ["demo", "naive", "overshoot", "no_sample1", "settle"]
    for m in modes:
        res = simulate_mode(m)
        print(f"Mode {m:12}: predicted score {res['score']:.2f}, success={res['success']}")


if __name__ == "__main__":
    main()
