# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for robot-joint-zeroing-pin, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run record (a linkkit History fed every control step) plus ladder(); the tests feed it synthetic
samples with th (arm angle from the zero mark, deg), qp (pin travel, mm), the arm's speed and spin, and both hands'
clearance and speed.
"""

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control


def sample(**kw):
    """The finished state: arm on the pin 0.6 deg under the mark, pin home, still, both hands clear."""
    m = {"th": -0.6, "qp": 26.2, "a_speed": 0.0, "a_spin": 0.0,
         "clear": {"left": 0.08, "right": 0.06}, "tcp_speed": {"left": 0.0, "right": 0.0}}
    m.update(kw)
    return m


def feed(samples, H=None):
    H = H or lk.History()
    t = 0.0
    for m in samples:
        t += DT
        H._tick(t)
        T.record(H, m)
    return H


def secs(m, s):
    return [m] * int(round(s / DT))


def start():
    return sample(th=-30.0, qp=0.0, clear={"left": 0.2, "right": 0.2})


def held(**kw):
    base = dict(th=0.0, qp=0.0, clear={"left": 0.0, "right": 0.2})
    base.update(kw)
    return sample(**base)


def demo_trace():
    out = secs(start(), 2.0) + secs(held(), 3.0) + secs(held(qp=10.0), 1.0) + secs(held(qp=20.0, th=-0.3), 1.0)
    out += secs(held(qp=26.2, th=-0.3), 1.0)
    out += secs(sample(th=-0.6, qp=26.2, clear={"left": 0.02, "right": 0.05}, tcp_speed={"left": 0.05, "right": 0.05}), 1.0)
    return out + secs(sample(), 1.5)


def test_demo_trace_scores_one():
    assert T.ladder(feed(demo_trace()))[0] == 1.0


def test_nothing_scores_zero():
    assert T.ladder(feed(secs(start(), 10.0)))[0] == 0.0


def test_pin_alone_through_empty_air_scores_zero():
    """The pin pushed home while the arm lies on its stop: the arm is 30 deg from the mark, so no rung counts."""
    out = secs(start(), 2.0) + secs(start(), 2.0) + secs(sample(th=-30.0, qp=26.2), 3.0)
    assert T.ladder(feed(out))[0] == 0.0


def test_arm_held_without_the_pin_scores_zero():
    assert T.ladder(feed(secs(start(), 1.0) + secs(held(), 10.0)))[0] == 0.0


@pytest.mark.parametrize("th,score", [(0.99, 1.0), (1.01, 0.0), (-0.99, 1.0), (-1.01, 0.0)])
def test_the_one_degree_window(th, score):
    out = secs(held(th=th, qp=26.2), 1.0) + secs(sample(th=th, qp=26.2), 3.0)
    assert T.ladder(feed(out))[0] == score


@pytest.mark.parametrize("qp,score", [(17.9, 0.0), (18.0, 0.25)])
def test_the_through_rung(qp, score):
    out = secs(held(qp=qp), 1.0) + secs(sample(th=-30.0, qp=0.0), 2.0)  # the pin withdrawn, the arm fell back
    assert T.ladder(feed(out))[0] == score


def test_home_rung_at_26_mm():
    out = secs(held(qp=25.9), 1.0) + secs(sample(th=-0.6, qp=25.9), 3.0)
    assert T.ladder(feed(out))[0] == 0.25
    out = secs(held(qp=26.0), 1.0) + secs(sample(th=-0.6, qp=26.0), 3.0)
    assert T.ladder(feed(out))[0] == 1.0


def test_home_then_pulled_back_keeps_half():
    out = secs(held(qp=26.2), 1.0) + secs(sample(th=-0.6, qp=20.0), 3.0)
    assert T.ladder(feed(out))[0] == 0.5


def test_a_hand_still_close_at_the_end_scores_three_quarters():
    out = secs(held(qp=26.2), 1.0) + secs(sample(clear={"left": 0.029, "right": 0.06}), 3.0)
    assert T.ladder(feed(out))[0] == 0.75


def test_a_hand_moving_at_the_end_scores_three_quarters():
    out = secs(held(qp=26.2), 1.0) + secs(sample(tcp_speed={"left": 0.0, "right": 0.006}), 3.0)
    assert T.ladder(feed(out))[0] == 0.75


def test_release_needs_a_full_second():
    out = secs(held(qp=26.2), 1.0) + secs(sample(), 0.9)
    assert T.ladder(feed(out))[0] == 0.75
    out = secs(held(qp=26.2), 1.0) + secs(sample(), 1.1)
    assert T.ladder(feed(out))[0] == 1.0


def test_arm_still_needs_a_full_second():
    out = secs(held(qp=26.2), 1.0) + secs(sample(a_speed=0.003), 3.0)
    assert T.ladder(feed(out))[0] == 0.75
    out = secs(held(qp=26.2), 1.0) + secs(sample(a_spin=2.1), 3.0)
    assert T.ladder(feed(out))[0] == 0.75


def test_a_fast_transient_does_not_zero_the_run():
    """A fast, controlled swing of the link at any time costs nothing; only the final state is scored."""
    out = demo_trace()
    out[30] = sample(th=-10.0, qp=0.0, a_spin=121.0)
    assert T.ladder(feed(out))[0] == 1.0
    out[30] = sample(th=-10.0, qp=0.0, a_spin=400.0)
    assert T.ladder(feed(out))[0] == 1.0


def test_a_fall_with_no_recovery_still_scores_nothing():
    """The link let fall before the pin is in, and never zeroed with the pin home, has no rung."""
    out = secs(sample(th=-30.0, qp=0.0, a_spin=300.0), 2.0) + secs(sample(th=-30.0, qp=0.0), 4.0)
    assert T.ladder(feed(out))[0] == 0.0


def test_record_is_clean_after_a_reset():
    H = feed(demo_trace())
    H.reset()
    assert T.ladder(feed(secs(start(), 1.0), H))[0] == 0.0


def test_empty_record_is_zero():
    assert T.ladder(lk.History())[0] == 0.0
