# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for seat-belt-retractor-lock, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); every test feeds it
synthetic pull-out traces e(t) in metres at 20 Hz.
"""

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402,F401
import task as T  # noqa: E402

DT = 0.05
FAR = {"left": 0.2, "right": 0.2}
STILL = {"left": 0.0, "right": 0.0}
NEAR = {"left": 0.2, "right": 0.0}


def run(segments, clear_end=1.2, home_at_end=True, lock=True):
    """segments: list of (v m/s, seconds). Then 1.2 s standing (hands clear unless clear_end is 0)."""
    rec = T.Record()
    t, e = 0.0, 0.0

    def feed(clear):
        nonlocal t
        rec.update({"e": e, "clear": clear, "tcp_speed": STILL}, t=t)
        t += DT

    feed(NEAR)
    for v, secs in segments:
        for _ in range(round(secs / DT)):
            if not (lock and "jerk" in rec.H.evers):  # the lock holds the webbing once it has caught
                e = max(0.0, e + v * DT)
            feed(NEAR)
    if home_at_end:
        e = 0.0
    for _ in range(round(clear_end / DT) + 2):
        feed(FAR if clear_end else NEAR)
    return rec


def steady(to=0.17, v=0.08):
    return [(v, to / v)]


def score(rec):
    return T.ladder(rec)[0]


def test_good_run_scores_1():
    rec = run(steady() + [(0.0, 0.5), (0.6, 0.1)])
    assert score(rec) == 1.0


def test_no_pull_is_0():
    assert score(run([])) == 0.0


def test_steady_only_is_0():
    assert score(run(steady() + [(0.0, 0.5)])) == 0.0


def test_slow_creep_to_full_travel_is_0():
    assert score(run([(0.2, 1.5)])) == 0.0


def test_jerk_only_gates_to_0():
    rec = run([(0.6, 0.2)])
    assert score(rec) == 0.0 and "early_jerk" in rec.H.evers


def test_jerk_at_about_110mm_gates_but_after_152mm_passes():
    assert score(run([(0.08, 1.0), (0.6, 0.1)])) == 0.0
    assert score(run([(0.08, 1.9), (0.0, 0.3), (0.6, 0.1)])) == 1.0


def test_steady_then_early_jerk_never_recovers():
    rec = run([(0.6, 0.1), (-0.5, 0.3)] + steady() + [(0.0, 0.5), (0.6, 0.1)])
    assert score(rec) == 0.0


def test_speed_is_over_two_steps():
    assert T.speed([0.0, 0.0, 0.03]) == pytest.approx(0.30)
    assert T.speed([0.0, 0.0, 0.029]) < 0.30
    assert T.speed([0.0]) == 0.0


def test_pull_at_0_29_ms_is_not_a_jerk():
    assert score(run(steady() + [(0.0, 0.5), (0.29, 0.4)])) == 0.0


def test_pull_at_0_31_ms_is_a_jerk():
    rec = run(steady() + [(0.0, 0.5), (0.31, 0.1)])
    assert "jerk" in rec.H.evers


def test_slow_phase_at_0_26_ms_is_not_a_steady_pull():
    rec = run([(0.26, 0.7), (0.0, 0.5), (0.6, 0.1)])
    assert "slow_done" not in rec.H.evers and score(rec) == 0.0


def test_ran_on_more_than_15mm_is_025():
    rec = run(steady() + [(0.0, 0.5), (0.7, 0.05), (0.7, 0.05), (0.2, 0.2)], home_at_end=True, lock=False)
    assert rec.H.values["e_jerk"] > 0
    assert (rec.H.peaks["e_after"] - rec.H.values["e_jerk"]) * 1000 > 15
    assert score(rec) == 0.25


def test_not_wound_back_in_is_05():
    rec = run(steady() + [(0.0, 0.5), (0.6, 0.1)], home_at_end=False)
    assert score(rec) == 0.5


def test_5mm_home_tolerance():
    rec = run(steady() + [(0.0, 0.5), (0.6, 0.1)], home_at_end=False)
    rec.update({"e": 0.0051, "clear": FAR, "tcp_speed": STILL})
    assert score(rec) == 0.5
    for _ in range(30):
        rec.update({"e": 0.005, "clear": FAR, "tcp_speed": STILL})
    assert score(rec) == 1.0


def test_hand_29mm_from_tongue_restarts_the_release_timer():
    rec = run(steady() + [(0.0, 0.5), (0.6, 0.1)], clear_end=0.0)
    for _ in range(30):
        rec.update({"e": 0.0, "clear": {"left": 0.2, "right": 0.029}, "tcp_speed": STILL})
    assert score(rec) == 0.75
    for _ in range(20):
        rec.update({"e": 0.0, "clear": {"left": 0.2, "right": 0.030}, "tcp_speed": STILL})
    assert score(rec) == 1.0


def test_hand_moving_6mm_s_is_not_still():
    rec = run(steady() + [(0.0, 0.5), (0.6, 0.1)], clear_end=0.0)
    for _ in range(40):
        rec.update({"e": 0.0, "clear": FAR, "tcp_speed": {"left": 0.0, "right": 0.006}})
    assert score(rec) == 0.75


def test_release_needs_1s():
    rec = run(steady() + [(0.0, 0.5), (0.6, 0.1)], clear_end=0.0)
    for _ in range(19):  # 0.95 s
        rec.update({"e": 0.0, "clear": FAR, "tcp_speed": STILL})
    assert score(rec) == 0.75


def test_reset_clears_the_record_and_nothing_else_does():
    rec = run([(0.6, 0.2)])
    assert score(rec) == 0.0
    rec.update({"e": 0.0, "clear": FAR, "tcp_speed": STILL})
    assert "early_jerk" in rec.H.evers  # returning the webbing does not clear it
    rec.reset()
    assert rec.last is None and not rec.H.evers


def test_lock_trigger_matches_the_scorer_speed():
    rec = T.Record()
    for k, e in enumerate([0.0, 0.0, 0.0, 0.015, 0.031]):
        rec.update({"e": e, "clear": FAR, "tcp_speed": STILL}, t=k * DT)
    assert rec.lock_now()  # (0.031 - 0.0) / 0.1 = 0.31 m/s
