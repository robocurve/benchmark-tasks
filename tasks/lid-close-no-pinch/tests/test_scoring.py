# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for lid-close-no-pinch, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(). The tests feed it synthetic
samples at 20 Hz; pinch_zone() is tested on plain poses.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control
S = T.STAY_DEG


def sample(q=S, pull=0.0, clear=0.08, speed=0.0, pinch=None):
    return {"q": q, "pull": pull, "clear": {"left": clear, "right": clear},
            "tcp_speed": {"left": speed, "right": speed}, "pinch": pinch}


def feed(samples, rec=None, t0=0.0):
    rec = rec or T.Record()
    for i, m in enumerate(samples):
        rec.update(m, t=t0 + i * DT)
    return rec


def hold(m, seconds):
    return [dict(m) for _ in range(round(seconds / DT))]


def ramp(q0, q1, seconds, **kw):
    n = round(seconds / DT)
    return [sample(q=q0 + (q1 - q0) * k / n, **kw) for k in range(1, n + 1)]


def demo_trace(clear=0.0, pull=0.013):
    """Knob pulled out to 13 mm, lid lowered slowly from its stay, knob let go, both grippers back out."""
    s = hold(sample(S), 2.0) + hold(sample(S, pull=pull, clear=clear), 1.0)
    s += ramp(S, 0.0, 10.0, pull=pull, clear=clear)
    s += hold(sample(0.0, pull=pull, clear=clear), 1.0)
    s += hold(sample(0.0, pull=0.0, clear=0.1), 2.0)
    return s


def score(rec):
    return T.ladder(rec)[0]


def test_demo_trace_scores_one():
    rec = feed(demo_trace())
    assert T.ladder(rec)[0] == 1.0, T.ladder(rec)
    assert rec.H.peaks["v_peak"] < 0.05


def test_no_steps_scores_zero():
    assert score(T.Record()) == 0.0


def test_doing_nothing_scores_zero():
    assert score(feed(hold(sample(S), 30.0))) == 0.0


def test_lid_stays_on_its_stay_when_only_the_arm_at_the_tab_works():
    # right arm alone: the knob never moves, the pawl holds the lid at its stay
    s = hold(sample(S, clear=0.0), 20.0)
    assert score(feed(s)) == 0.0 and "never left" in T.ladder(feed(s))[1]


def test_knob_arm_alone_is_a_slam():
    # left arm alone: the knob is out, nobody holds the lid, it falls 25 deg in 0.4 s
    s = hold(sample(S, pull=0.013), 1.0) + [sample(S * (1 - k / 8), pull=0.013) for k in range(1, 9)]
    s += hold(sample(0.0, pull=0.013), 2.0) + hold(sample(0.0, pull=0.0, clear=0.1), 3.0)
    rec = feed(s)
    assert "fast" in rec.H.evers and score(rec) == 0.0


def test_speed_gate_9_4_deg_in_a_quarter_second_ok_9_7_gate():
    # 0.15 m/s at a 0.226 m free edge over 0.25 s is 9.5 deg
    for drop, want_gate in ((9.4, False), (9.7, True)):
        s = hold(sample(20, pull=0.013), 1.0) + [sample(20 - drop * k / 5, pull=0.013) for k in range(1, 6)] + hold(
            sample(20 - drop, pull=0.013), 1.0)
        assert ("fast" in feed(s).H.evers) is want_gate, drop


def test_a_slam_is_gated_for_good_even_if_the_lid_ends_shut_and_released():
    slam = [sample(20 * (1 - k / 4), pull=0.013) for k in range(1, 5)]  # 20 deg in 0.2 s
    s = hold(sample(20, pull=0.013), 1.0) + slam + ramp(0, 20, 1.0, pull=0.013) + ramp(20, 0, 12.0, pull=0.013)
    s += hold(sample(0, pull=0.0, clear=0.1), 3.0)
    rec = feed(s)
    assert "fast" in rec.H.evers
    assert score(rec) == 0.0 and "faster" in T.ladder(rec)[1]


def test_lifting_fast_is_gated_too():
    s = hold(sample(S), 1.0) + ramp(S, S + 10, 0.2) + hold(sample(S + 10), 1.0)
    assert "fast" in feed(s).H.evers


def test_pinch_gate_beats_a_perfect_finish():
    s = demo_trace()
    s[300] = dict(s[300], pinch="rim")
    rec = feed(s)
    assert score(rec) == 0.0 and "pinch" in T.ladder(rec)[1]


def test_closed_0_6_deg_and_not_0_7():
    for q, want in ((0.6, 1.0), (0.7, 0.5)):
        s = hold(sample(S), 1.0) + ramp(S, 14.0, 6.0, pull=0.013) + ramp(14, q, 4.0, pull=0.013)
        s += hold(sample(q, pull=0.013, clear=0.0), 1.0) + hold(sample(q, pull=0.0, clear=0.1), 3.0)
        assert score(feed(s)) == want, q


def test_quarter_rung_needs_the_lid_below_20_deg():
    base = hold(sample(S), 1.0)
    assert score(feed(base + ramp(S, 20.5, 3.0, pull=0.013))) == 0.0
    assert score(feed(base + ramp(S, 19.5, 3.0, pull=0.013))) == 0.25


def test_half_rung_needs_the_lid_below_15_deg():
    base = hold(sample(S), 1.0)
    assert score(feed(base + ramp(S, 14.0, 6.0, pull=0.013))) == 0.5
    assert score(feed(base + ramp(S, 16.0, 6.0, pull=0.013))) == 0.25


def test_three_quarter_rung_lid_shut_but_gripper_still_close():
    s = demo_trace()[:-40] + hold(sample(0.0, clear=0.029), 3.0)
    assert score(feed(s)) == 0.75
    s = demo_trace()[:-40] + hold(sample(0.0, clear=0.030), 3.0)
    assert score(feed(s)) == 1.0


def test_knob_not_back_home_is_not_a_release():
    s = demo_trace()[:-40] + hold(sample(0.0, pull=0.003, clear=0.1), 3.0)
    assert score(feed(s)) == 0.75
    s = demo_trace()[:-40] + hold(sample(0.0, pull=0.0019, clear=0.1), 3.0)
    assert score(feed(s)) == 1.0


def test_release_needs_a_full_second():
    s = demo_trace()[:-40] + hold(sample(0.0, clear=0.1), 0.9)
    assert score(feed(s)) == 0.75
    s = demo_trace()[:-40] + hold(sample(0.0, clear=0.1), 1.3)
    assert score(feed(s)) == 1.0


def test_a_moving_gripper_restarts_the_release_timer():
    s = demo_trace()[:-40] + hold(sample(0.0, clear=0.1), 0.8) + hold(sample(0.0, clear=0.1, speed=0.02), 0.1)
    s += hold(sample(0.0, clear=0.1), 0.8)
    assert score(feed(s)) == 0.75


def test_a_lid_creeping_at_6_mm_s_is_not_still():
    step = math.degrees(0.006 / T.DEPTH) * DT  # deg per control step at 6 mm/s
    rec = feed(demo_trace()[:-40])
    t = rec.H.t
    for i in range(60):
        t += DT
        rec.update(sample(0.0 + 0.0 * i + 0.3 + step * i, clear=0.1), t=t)
    assert score(rec) != 1.0


def test_record_resets_clean():
    rec = feed(demo_trace())
    assert score(rec) == 1.0
    rec.reset()
    assert score(rec) == 0.0 and not rec.H.evers


def test_pinch_zone_rim_hinge_and_tab():
    pose = np.eye(4)  # the lid frame: origin on the hinge line, x toward the free edge's negative side
    assert T.pinch_zone([np.array([-0.10, 0.10, -0.02])], pose) == "rim"  # between plate and rim
    assert T.pinch_zone([np.array([-0.10, 0.10, 0.02])], pose) is None  # above the plate
    assert T.pinch_zone([np.array([-0.10, 0.10, -0.05])], pose) is None  # 50 mm below
    assert T.pinch_zone([np.array([-0.10, -0.26, -0.02])], pose) is None  # at the side tab, outside the footprint
    assert T.pinch_zone([np.array([-0.10, 0.275, -0.02])], pose) is None  # at the release knob, outside the footprint
    assert T.pinch_zone([np.array([0.0, 0.10, 0.024])], pose) == "hinge"
    assert T.pinch_zone([np.array([0.0, 0.10, 0.026])], pose) is None
    assert T.pinch_zone([np.array([-0.02, 0.10, 0.03])], pose) is None


def test_pinch_zone_follows_the_lid_when_it_is_open():
    R = lk.axis_rotation((0, 1, 0), math.radians(T.STAY_DEG))
    pose = np.eye(4)
    pose[:3, :3] = R
    assert T.pinch_zone([R @ np.array([-0.10, 0.10, -0.02])], pose) == "rim"
    assert T.pinch_zone([np.array([-0.10, 0.10, -0.02])], pose) is None
