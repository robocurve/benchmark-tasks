# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for cylinder-bench-clamp, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); the tests feed it
synthetic samples shaped like measure() output.
"""

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402,F401
import task as T  # noqa: E402

DT = 0.05


def sample(**kw):
    """The finished state: bottle upright in the V, bolt home, lever closed and locked, hands clear and still."""
    m = {"tilt": 0.5, "err": 1.0, "bolt": 85.0, "lever": 95.0, "locked": True, "speed": 0.0, "spin": 0.0,
         "clear": {"left": 0.08, "right": 0.06}, "tcp_speed": {"left": 0.0, "right": 0.0}}
    m.update(kw)
    return m


def leaning(**kw):
    """The start: bottle leaning back 7.5 deg on the V rods, bolt retracted, lever up."""
    base = dict(tilt=7.5, err=13.0, bolt=0.0, lever=0.0, locked=False, clear={"left": 0.2, "right": 0.2})
    base.update(kw)
    return sample(**base)


def held(**kw):
    base = dict(clear={"left": 0.2, "right": 0.0}, tcp_speed={"left": 0.0, "right": 0.01})
    base.update(kw)
    return sample(**base)


def secs(m, s):
    return [m] * int(round(s / DT))


def feed(samples):
    rec = T.Record()
    t = 0.0
    for m in samples:
        t += DT
        rec.H._tick(t)
        rec.update(m, t)
    return rec


def demo_trace():
    out = secs(leaning(), 2.0)
    out += secs(held(bolt=0.0, lever=0.0, locked=False, tilt=0.5, err=1.0), 3.0)
    out += secs(held(bolt=85.0, lever=0.0, locked=False), 2.0)
    out += secs(held(lever=95.0, locked=True, clear={"left": 0.02, "right": 0.0}), 2.0)
    out += secs(sample(clear={"left": 0.05, "right": 0.01}, tcp_speed={"left": 0.05, "right": 0.05}), 1.0)
    out += secs(sample(), 3.5)
    return out


def score(rec):
    return T.ladder(rec)[0]


def test_demo_trace_scores_one():
    r = feed(demo_trace())
    assert score(r) == 1.0, T.ladder(r)


def test_no_steps_scores_zero():
    assert score(T.Record()) == 0.0


def test_doing_nothing_scores_zero():
    assert score(feed(secs(leaning(), 20))) == 0.0


def test_left_only_leaning_bottle_scores_zero():
    """The operator alone closes bolt and lever on the bottle still leaning at 7.5 deg: no rung."""
    r = feed(secs(leaning(), 5) + secs(leaning(bolt=85.0), 3) + secs(leaning(bolt=85.0, lever=95.0, locked=True), 6))
    assert score(r) == 0.0


def test_right_only_holder_scores_zero():
    """Upright and held, then released with the bolt open: no rung, and the fallen bottle is gated."""
    r = feed(secs(leaning(), 2) + secs(held(bolt=0.0, lever=0.0, locked=False), 8)
             + secs(sample(bolt=0.0, lever=0.0, locked=False, tilt=60.0, err=90.0), 5))
    assert score(r) == 0.0


def test_upright_limit_three_degrees():
    ok = feed(secs(sample(tilt=3.0), 6))
    bad = feed(secs(sample(tilt=3.1), 6))
    assert score(ok) == 1.0 and score(bad) == 0.0


def test_axis_tolerance_four_mm():
    assert score(feed(secs(sample(err=4.0), 6))) == 1.0
    assert score(feed(secs(sample(err=4.1), 6))) == 0.0


def test_bolt_home_at_80_mm():
    assert score(feed(secs(sample(bolt=80.0), 6))) == 1.0
    assert score(feed(secs(sample(bolt=79.9), 6))) == 0.0


def test_lever_closed_at_85_deg():
    assert score(feed(secs(sample(lever=85.0), 6))) == 1.0
    assert score(feed(secs(sample(lever=84.9), 6))) == 0.25  # bolt home and upright, lever short


def test_lock_needed_for_the_top_rungs():
    r = feed(secs(sample(locked=False), 6))
    assert score(r) == 0.25  # upright with the bolt home earns the first rung only


def test_ladder_rungs():
    assert score(feed(secs(held(lever=0.0, locked=False), 3))) == 0.25
    assert score(feed(secs(held(), 3) + secs(held(bolt=40.0), 1))) == 0.5
    assert score(feed(secs(held(), 3))) == 0.75
    assert score(feed(secs(held(), 3) + secs(sample(clear={"left": 0.05, "right": 0.02}), 1))) == 0.75


def test_topple_is_never_cleared():
    r = feed(secs(sample(tilt=40.0, err=90.0), 1) + demo_trace())
    assert score(r) == 0.0 and "gate" in T.ladder(r)[1]
    r = feed(secs(sample(tilt=39.9), 1) + demo_trace())
    assert score(r) == 1.0


def test_the_start_lean_is_no_topple():
    assert score(feed(secs(leaning(), 3) + demo_trace())) == 1.0


def test_hands_must_be_clear_30_mm():
    close = feed(demo_trace()[:-70] + secs(sample(clear={"left": 0.029, "right": 0.06}), 3.5))
    assert score(close) == 0.75
    far = feed(demo_trace()[:-70] + secs(sample(clear={"left": 0.030, "right": 0.06}), 3.5))
    assert score(far) == 1.0


def test_hand_moving_5_mm_per_s_is_not_still():
    assert score(feed(secs(sample(tcp_speed={"left": 0.005, "right": 0.0}), 6))) == 0.75
    assert score(feed(secs(sample(tcp_speed={"left": 0.0049, "right": 0.0}), 6))) == 1.0


def test_release_must_last_one_second():
    r = feed(secs(sample(), 2.0) + secs(sample(clear={"left": 0.01, "right": 0.06}), 0.05) + secs(sample(), 0.9))
    assert score(r) == 0.75
    r = feed(secs(sample(), 2.0) + secs(sample(clear={"left": 0.01, "right": 0.06}), 0.05) + secs(sample(), 1.05))
    assert score(r) == 1.0


def test_bottle_must_be_still_two_seconds():
    assert score(feed(secs(sample(speed=0.003), 6))) == 0.75
    assert score(feed(secs(sample(spin=3.0), 6))) == 0.75
    assert score(feed(secs(sample(), 1.9))) == 0.75
    assert score(feed(secs(sample(), 2.1))) == 1.0


def test_end_state_must_hold_a_released_bolt_loses_it():
    r = feed(demo_trace() + secs(sample(bolt=40.0), 3))
    assert score(r) == 0.5  # reached the closed rung once, but the end state is not secured


def test_record_is_clean_after_reset():
    r = feed(secs(sample(tilt=60.0), 2))
    assert score(r) == 0.0
    r.H.reset()
    r2 = feed(demo_trace())
    assert score(r2) == 1.0


def test_success_only_on_top_rung():
    r = feed(demo_trace())
    assert T.ladder(r)[0] == 1.0
    r = feed(demo_trace()[:-70])
    assert T.ladder(r)[0] < 1.0


def test_measure_geometry_of_an_upright_and_a_leaning_bottle():
    import numpy as np

    class Fake:
        def __init__(self, tilt_deg, x):
            import math
            c, s = math.cos(math.radians(tilt_deg - T.LEAN_DEG)), math.sin(math.radians(tilt_deg - T.LEAN_DEG))
            S = np.eye(4)
            S[:3, :3] = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
            S[:3, 3] = [x, 0.0, T.TOP + T.R_B]
            self.poses = {"body": S}
            self.q = {"slide": 0.085, "cam": 1.6}

    T.lk.braked = lambda *a, **k: True
    up = T.measure(Fake(0.0, T.XC), Fake(0.0, T.XC), {})
    assert up["tilt"] == pytest.approx(0.0, abs=1e-6) and up["err"] == pytest.approx(0.0, abs=1e-6)
    assert up["bolt"] == pytest.approx(85.0) and T.secured(up)
    lean = T.measure(Fake(7.5, T._x_s), Fake(0.0, T.XC), {})
    assert lean["tilt"] == pytest.approx(7.5, abs=1e-6)
    assert lean["err"] == pytest.approx(11.4, abs=0.6)  # leaning 7.5 deg on the plate: 11 mm behind the centre line at 0.20 height
    assert not T.upright(lean)
