# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for gantry-square-stops, pure Python (no simulator). Run from the task folder: pytest -q tests"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

DT = 0.05


def sample(gl=0.0, gr=0.0, speed=0.0, lift=0.0, clear=0.08, tcp=0.0):
    return {"gap": {"left": gl, "right": gr}, "lift": {"left": lift, "right": lift}, "speed": speed, "yaw": 0.0,
            "clear": {"left": clear, "right": clear}, "tcp_speed": {"left": tcp, "right": tcp}}


def run(samples, n_each=1):
    rec = T.Record()
    t = 0.0
    for m in samples:
        for _ in range(n_each):
            t += DT
            rec.update(m, t)
    return rec


def demo_trace(end=None, hold=30):
    out = [sample(40.1, 42.8), sample(20.0, 20.4, speed=0.012), sample(5.0, 5.0, speed=0.008)]
    return out + [end or sample()] * hold


def pose(x, y_yaw_deg=0.0, z=T.TOP):
    c, s = math.cos(math.radians(y_yaw_deg)), math.sin(math.radians(y_yaw_deg))
    P = np.eye(4)
    P[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    P[:3, 3] = (x, 0.0, z)
    return P


def test_demo_shaped_trace_scores_one():
    assert T.ladder(run(demo_trace()))[0] == 1.0


def test_nothing_scores_zero():
    assert T.ladder(run([sample(40.1, 42.8)] * 40))[0] == 0.0


def test_one_end_only_scores_zero():
    assert T.ladder(run([sample(40.1, 42.8), sample(39.0, 41.0)] * 20))[0] == 0.0


def test_gap_is_the_worst_end():
    assert T.ladder(run(demo_trace(sample(0.0, 0.06))))[0] == 0.5
    assert T.ladder(run(demo_trace(sample(0.05, 0.05))))[0] == 1.0


def test_feeler_limit_is_0_05_mm():
    assert T.ladder(run(demo_trace(sample(0.049, 0.0))))[0] == 1.0
    assert T.ladder(run(demo_trace(sample(0.051, 0.0))))[0] == 0.5


def test_near_rung_is_5_mm():
    assert T.ladder(run(demo_trace(sample(4.9, 4.9))))[0] == 0.5
    assert T.ladder(run(demo_trace(sample(5.1, 5.1))))[0] == 0.25


def test_quarter_needs_both_ends_25_mm():
    rec = run([sample(40.0, 42.0), sample(14.0, 16.0), sample(30.0, 30.0)])
    assert T.ladder(rec)[0] == 0.25
    rec = run([sample(40.0, 42.0), sample(14.0, 30.0), sample(30.0, 30.0)])
    assert T.ladder(rec)[0] == 0.0


def test_fast_transient_is_not_a_permanent_zero():
    trace = [sample(40.1, 42.8), sample(10.0, 10.0, speed=0.5)] + demo_trace()
    assert T.ladder(run(trace))[0] == 1.0


def test_lifted_transient_ok_but_unseated_end_is_not_full():
    trace = [sample(40.1, 42.8), sample(30.0, 30.0, lift=5.0)] + demo_trace()
    assert T.ladder(run(trace))[0] == 1.0
    end = demo_trace(sample(lift=1.0))
    assert T.ladder(run(end))[0] == 0.75
    assert T.ladder(run(demo_trace(sample(lift=0.9))))[0] == 1.0


def test_beam_must_be_still_one_second():
    moving = sample(speed=0.004)
    assert T.ladder(run([sample(40.1, 42.8)] + [moving] * 30))[0] == 0.75
    short = [sample(40.1, 42.8)] + [moving] * 30 + [sample()] * 19
    assert T.ladder(run(short))[0] == 0.75


def test_release_needs_30_mm_and_still():
    assert T.ladder(run(demo_trace(sample(clear=0.029))))[0] == 0.75
    assert T.ladder(run(demo_trace(sample(clear=0.030))))[0] == 1.0
    assert T.ladder(run(demo_trace(sample(tcp=0.006))))[0] == 0.75
    assert T.ladder(run([sample(40.1, 42.8)] + [sample()] * 18))[0] == 0.75


def test_reset_clears_the_record():
    rec = run([sample(10.0, 10.0, speed=0.5)])
    assert rec.last is not None
    rec.reset()
    assert rec.last is None and not rec.H.evers and T.ladder(rec)[0] == 0.0


def test_measure_pose_start_gaps():
    m = T.measure_pose(pose(T.XC0, T.RACK_DEG))
    assert abs(m["gap"]["left"] - 40.0) < 0.2 and abs(m["gap"]["right"] - 42.7) < 0.2
    assert abs(m["yaw"] - T.RACK_DEG) < 1e-6 and abs(m["lift"]["left"]) < 1e-6


def test_measure_pose_square_at_stops():
    m = T.measure_pose(pose(T.STOP_X - T.BLOCK_L / 2))
    assert abs(m["gap"]["left"]) < 1e-6 and abs(m["gap"]["right"]) < 1e-6


def test_measure_pose_racked_at_a_stop_reads_the_feeler_gap():
    m = T.measure_pose(pose(T.STOP_X - T.BLOCK_L / 2 - 0.0005, 0.01))
    assert m["gap"]["left"] > 0.4 and m["gap"]["right"] > 0.4


def test_speed_from_pose_change():
    a, b = pose(0.20), pose(0.2004)
    assert abs(T.measure_pose(b, a, 0.05)["speed"] - 0.008) < 1e-9
