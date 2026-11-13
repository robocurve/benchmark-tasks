# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for security-tag-detacher, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run linkkit History fed every control step through update(), plus ladder(). Tests feed it
synthetic samples; the seat_error tests build poses by hand.
"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control


def sample(**kw):
    """A sample of the seated state: pocket on the dome, pin still in, one hand on the detacher, one far away."""
    m = {"dx": 0.0, "dz": 0.0, "gap": 0.0003, "tilt": 0.0, "pin": 0.0, "clear_left": 0.0, "clear_right": 0.08,
         "spd_left": 0.0, "spd_right": 0.0}
    m.update(kw)
    return m


class Run:
    def __init__(self):
        self.h = lk.History()
        self.t = 0.0
        self.releases = 0

    def step(self, n=1, **kw):
        for _ in range(n):
            self.t += DT
            self.h._tick(self.t)
            self.releases += bool(T.update(self.h, sample(**kw)))
        return self

    def rung(self):
        return T.ladder(self.h)[0]


def demo_run():
    r = Run()
    r.step(10, gap=0.1)  # approach
    r.step(10)  # seated for 0.5 s, the clutch releases at 0.3 s
    r.step(10, pin=0.010)  # pulling
    r.step(5, pin=0.0175)  # pin out, hands still on it
    r.step(30, pin=0.017, clear_left=0.06, clear_right=0.08)  # let go, backed off, still
    return r


def test_demo_shaped_trace_scores_1():
    r = demo_run()
    assert r.rung() == 1.0
    assert r.releases == 1  # the clutch releases once, 0.3 s into the seat


def test_nothing_scores_0():
    r = Run().step(40, gap=0.1)
    assert r.rung() == 0.0 and r.releases == 0


@pytest.mark.parametrize("key,ok,bad", [("dx", 0.003, 0.0031), ("dz", 0.002, 0.0021), ("gap", 0.0015, 0.0016),
                                         ("tilt", 6.0, 6.1)])
def test_seat_limits(key, ok, bad):
    assert T.is_seated(sample(**{key: ok}))
    assert not T.is_seated(sample(**{key: bad}))


@pytest.mark.parametrize("key,ok,bad", [("dx", -0.003, -0.0031), ("dz", -0.002, -0.0021)])
def test_seat_limits_are_symmetric(key, ok, bad):
    assert T.is_seated(sample(**{key: ok}))
    assert not T.is_seated(sample(**{key: bad}))


def test_gap_lower_limit():
    assert T.is_seated(sample(gap=-0.0005))
    assert not T.is_seated(sample(gap=-0.0006))


def test_seat_must_hold_0_3_s():
    assert Run().step(5).rung() == 0.0  # 0.25 s
    r = Run().step(6)  # 0.30 s
    assert r.releases == 1 and r.rung() == 0.25
    r = Run().step(4).step(1, dx=0.01).step(4)  # a break restarts the timer
    assert r.releases == 0 and r.rung() == 0.0


def test_forced_pin_gates_to_zero_even_after_a_good_run():
    r = Run().step(3, pin=0.0029, gap=0.1)  # 2.9 mm of stretch is no gate
    assert r.rung() == 0.0 and "forced" not in r.h.evers
    r = Run().step(1, pin=0.0030, gap=0.1)  # 3.0 mm with the clutch locked is
    assert "forced" in r.h.evers
    r.step(20).step(10, pin=0.017).step(30, pin=0.017, clear_left=0.06)
    assert r.rung() == 0.0  # seating and a full pull afterwards cannot undo it


def test_pull_during_the_seat_hold_is_forced():
    r = Run().step(3, pin=0.004)  # 0.15 s seated, the clutch still locked
    assert "forced" in r.h.evers and r.rung() == 0.0


def test_pin_travel_after_release_is_free():
    r = Run().step(6).step(5, pin=0.020)
    assert "forced" not in r.h.evers and r.rung() == 0.75


def test_ladder_rungs():
    assert Run().step(6).rung() == 0.25
    assert Run().step(6).step(2, pin=0.0049).step(2, pin=0.0).rung() == 0.25  # 4.9 mm is not 5
    assert Run().step(6).step(2, pin=0.0050).step(2, pin=0.0).rung() == 0.5  # peak 5 mm, then pushed back in
    assert Run().step(6).step(2, pin=0.0149).rung() == 0.5
    assert Run().step(6).step(2, pin=0.0150).rung() == 0.75


def test_release_gate():
    def final(n=21, **kw):
        r = Run().step(6).step(5, pin=0.016)
        return r.step(n, pin=0.016, **kw).rung()

    assert final(clear_left=0.06) == 1.0
    assert final(clear_left=0.0299) == 0.75  # a pad 29.9 mm from the detacher or tag
    assert final(clear_left=0.030) == 1.0
    assert final(clear_left=0.06, spd_right=0.0051) == 0.75  # 5.1 mm/s
    assert final(clear_left=0.06, spd_right=0.0050) == 1.0
    assert final(n=19, clear_left=0.06) == 0.75  # 0.95 s is not enough
    assert final(n=22, clear_left=0.06) == 1.0  # 1.1 s is
    r = Run().step(6).step(19, pin=0.016, clear_left=0.06).step(1, pin=0.016, clear_left=0.02)
    r.step(15, pin=0.016, clear_left=0.06)
    assert r.rung() == 0.75  # a hand back within 30 mm restarts the timer


def test_pin_pushed_back_in_loses_the_end_rungs():
    r = Run().step(6).step(5, pin=0.017).step(30, pin=0.002, clear_left=0.06)
    assert r.rung() == 0.5


def test_success_only_on_top_rung():
    assert T.ladder(demo_run().h)[0] >= 1.0
    assert T.ladder(Run().step(6).step(5, pin=0.02).h)[0] < 1.0


def test_history_clears_on_reset():
    r = Run().step(1, pin=0.01)
    assert "forced" in r.h.evers
    r.h.reset()
    assert T.ladder(r.h)[0] == 0.0 and "forced" not in r.h.evers


# -- geometry: seat_error from hand-built poses ---------------------------------------------------------------------


def pose(x, y, z=0.0, yaw_deg=0.0):
    a = math.radians(yaw_deg)
    P = np.eye(4)
    P[:3, :3] = lk.axis_rotation((0, 0, 1), a)
    P[:3, 3] = [x, y, z]
    return P


BODY_POSE = pose(0.19, -0.03)


def test_seat_error_aligned():
    e = T.seat_error(BODY_POSE, pose(0.19, -0.03 + 0.023 + 0.0003))
    assert e["dx"] == pytest.approx(0.0, abs=1e-9) and e["dz"] == pytest.approx(0.0, abs=1e-9)
    assert e["gap"] == pytest.approx(0.0003) and e["tilt"] == pytest.approx(0.0, abs=1e-6)
    assert T.is_seated(e)


def test_seat_error_sideways_offset_and_height():
    e = T.seat_error(BODY_POSE, pose(0.19 + 0.0031, -0.03 + 0.023))
    assert e["dx"] == pytest.approx(0.0031) and not T.is_seated(e)
    e = T.seat_error(BODY_POSE, pose(0.19, -0.03 + 0.023, 0.0021))
    assert e["dz"] == pytest.approx(0.0021) and not T.is_seated(e)


def test_seat_error_yaw_tilt():
    assert T.seat_error(BODY_POSE, pose(0.19, -0.03 + 0.023, yaw_deg=6.0))["tilt"] == pytest.approx(6.0, abs=1e-6)
    assert not T.is_seated(T.seat_error(BODY_POSE, pose(0.19, -0.03 + 0.023, yaw_deg=7.0)))


def test_detacher_on_the_wrong_face_never_seats():
    # nose pointing +y on the cuff side, or the handle end against the tag: nowhere near the dome
    behind = pose(0.19, -0.03 - 0.023, yaw_deg=180.0)
    assert not T.is_seated(T.seat_error(BODY_POSE, behind))
    handle_first = pose(0.19, -0.03 + 0.023 + 0.085, yaw_deg=180.0)
    assert not T.is_seated(T.seat_error(BODY_POSE, handle_first))


def test_detacher_inside_the_tag_is_not_seated():
    assert not T.is_seated(T.seat_error(BODY_POSE, pose(0.19, -0.03 + 0.023 - 0.002)))  # 2 mm into the housing


def test_scene_constants_match_the_setup_text():
    assert T.BODY == (0.056, 0.046, 0.022)
    assert (T.PUCK_W, T.PUCK_LEN, T.PUCK_H) == (0.055, 0.085, 0.020)
    assert T.POCKET_W - 2 * T.DOME_R == pytest.approx(0.004)  # 2 mm clearance each side of the 14 mm dome
    assert T.POCKET_H - 2 * T.DOME_R == pytest.approx(0.002)
    assert T.POCKET_D - T.DOME_PROUD == pytest.approx(0.001)  # 1 mm of air behind the dome when the rim touches
    assert T.PIN_STROKE > T.OUT_FULL
