# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for harmonic-drive-insert, pure Python (no simulator). Run from the task folder: pytest -q tests"""

import math
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

DT = 0.05


def sample(**kw):
    m = {"g": 0.0, "lat": 0.3, "tilt": 0.5, "phase": 0.1, "fs_speed": 0.0, "fs_spin": 0.0,
         "clear": {"left": 0.3, "right": 0.06}, "tcp_speed": {"left": 0.0, "right": 0.0}}
    m.update(kw)
    return m


def run(samples):
    rec = T.Record()
    for i, m in enumerate(samples):
        rec.update(m, t=(i + 1) * DT)
    return rec


def seq(final, n=30, pre=()):
    return run(list(pre) + [final] * n)


def test_full_score():
    assert T.ladder(seq(sample()))[0] == 1.0


def test_nothing_is_zero():
    far = sample(g=-16.0, lat=150.0, phase=3.46)
    assert T.ladder(seq(far))[0] == 0.0


def test_stalled_on_the_tips_is_zero():
    stall = sample(g=12.0, phase=3.46)
    assert T.ladder(seq(stall))[0] == 0.0


def test_phase_edge_of_025():
    assert T.ladder(seq(sample(g=12.0, phase=1.0)))[0] == 0.25
    assert T.ladder(seq(sample(g=12.0, phase=1.1)))[0] == 0.0


def test_025_needs_the_stall_height():
    assert T.ladder(seq(sample(g=13.0, phase=0.5)))[0] == 0.25
    assert T.ladder(seq(sample(g=13.5, phase=0.5)))[0] == 0.0


def test_lateral_and_tilt_edges_of_025():
    assert T.ladder(seq(sample(g=12.0, lat=1.0, tilt=3.0)))[0] == 0.25
    assert T.ladder(seq(sample(g=12.0, lat=1.1)))[0] == 0.0
    assert T.ladder(seq(sample(g=12.0, tilt=3.1)))[0] == 0.0


def test_meshed_then_pulled_out_scores_half():
    rec = run([sample(g=5.0)] * 5 + [sample(g=30.0, phase=3.0)] * 10)
    assert T.ladder(rec)[0] == 0.5


def test_meshed_edge():
    assert T.ladder(run([sample(g=6.0)] * 5 + [sample(g=30.0)] * 5))[0] == 0.5
    assert T.ladder(run([sample(g=6.1)] * 5 + [sample(g=30.0, phase=3.0)] * 5))[0] == 0.25  # lined up at 6.1 mm, not meshed


def test_seated_edge():
    assert T.ladder(seq(sample(g=0.5)))[0] == 1.0
    assert T.ladder(seq(sample(g=0.6)))[0] == 0.5


def test_cocked_gate_beats_a_seated_end():
    rec = run([sample(g=8.0, tilt=8.0)] * 2 + [sample()] * 40)
    assert T.ladder(rec)[0] == 0.0
    assert "gate" in T.ladder(rec)[1]


def test_cock_edge_and_far_away_tilt():
    assert T.ladder(run([sample(g=8.0, tilt=7.9)] + [sample()] * 40))[0] == 1.0
    assert T.ladder(run([sample(g=8.0, tilt=45.0, lat=30.0)] + [sample()] * 40))[0] == 1.0  # carried tilted, far off


def test_still_needed():
    moving = sample(fs_speed=0.004)
    assert T.ladder(seq(moving))[0] == 0.75
    spinning = sample(fs_spin=math.radians(3.5))
    assert T.ladder(seq(spinning))[0] == 0.75


def test_release_edges():
    assert T.ladder(seq(sample(clear={"left": 0.3, "right": 0.029})))[0] == 0.75
    assert T.ladder(seq(sample(clear={"left": 0.3, "right": 0.030})))[0] == 1.0
    assert T.ladder(seq(sample(tcp_speed={"left": 0.0, "right": 0.006})))[0] == 0.75


def test_release_needs_one_second():
    assert T.ladder(seq(sample(), n=19))[0] == 0.75
    assert T.ladder(seq(sample(), n=22))[0] == 1.0


def test_one_pad_step_restarts_the_timer():
    near = sample(clear={"left": 0.3, "right": 0.02})
    rec = run([sample()] * 30 + [near] + [sample()] * 10)
    assert T.ladder(rec)[0] == 0.75


def test_reset_clears_the_record():
    rec = run([sample(g=8.0, tilt=20.0)] * 3)
    rec.reset()
    assert rec.last is None and not rec.H.evers
    assert T.ladder(rec)[0] == 0.0
    assert T.ladder(seq(sample()))[0] == 1.0


def test_phase_wrapping():
    p = T.PITCH_DEG
    assert T.phase_err_deg(p / 2) == pytest.approx(0.0)
    assert T.phase_err_deg(0.0) == pytest.approx(-p / 2, abs=1e-9) or T.phase_err_deg(0.0) == pytest.approx(p / 2)
    assert T.phase_err_deg(p / 2 + p) == pytest.approx(0.0, abs=1e-9)
    assert T.phase_err_deg(p / 2 - 0.5) == pytest.approx(-0.5)
    assert T.phase_err_deg(p / 2 + 180.0) == pytest.approx(0.0, abs=1e-9)  # 26 pitches: the opposite patch lines up too


def test_pose_rates():
    import numpy as np
    A = np.eye(4)
    B = np.eye(4)
    B[0, 3] = 0.0002
    v, w = T.pose_rates(A, B, DT)
    assert v == pytest.approx(0.004) and w == pytest.approx(0.0, abs=1e-9)
