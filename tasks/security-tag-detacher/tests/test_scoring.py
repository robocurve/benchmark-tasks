# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for security-tag-detacher, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run linkkit History fed every control step through update(), plus ladder(). Tests feed it
synthetic samples; the seat_error tests build poses by hand. The latch itself (a spring-loaded plunger and bolt in the
scene) is physics and is only checked here as geometry.
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
    m = {"dx": 0.0, "dz": 0.0, "gap": 0.0003, "tilt": 0.0, "pin": 0.0, "plunger": 0.0014, "clear_left": 0.0,
         "clear_right": 0.08, "spd_left": 0.0, "spd_right": 0.0}
    m.update(kw)
    return m


AWAY = {"gap": 0.1, "plunger": 0.0}  # the detacher 100 mm from the tag: not seated, the plunger sprung out


class Run:
    def __init__(self):
        self.h = lk.History()
        self.t = 0.0

    def step(self, n=1, **kw):
        for _ in range(n):
            self.t += DT
            self.h._tick(self.t)
            T.update(self.h, sample(**kw))
        return self

    def rung(self):
        return T.ladder(self.h)[0]


def demo_run():
    r = Run()
    r.step(10, **AWAY)  # approach
    r.step(10)  # seated for 0.5 s, the plunger pressed
    r.step(10, pin=0.010)  # pulling with the detacher in place
    r.step(5, pin=0.0175)  # pin out, hands still on it
    r.step(30, pin=0.017, clear_left=0.06, clear_right=0.08)  # let go, backed off, still
    return r


def test_demo_shaped_trace_scores_1():
    assert demo_run().rung() == 1.0


def test_nothing_scores_0():
    assert Run().step(40, **AWAY).rung() == 0.0


def test_seating_alone_scores_0():
    assert Run().step(40).rung() == 0.0  # seated, nothing pulled


@pytest.mark.parametrize("key,ok,bad", [("dx", 0.003, 0.0031), ("dz", 0.002, 0.0021), ("gap", 0.0005, 0.0006),
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


# -- the late pull: the detacher has left, the latch has closed again, nothing the pin does afterwards is credited -----


def test_late_pull_scores_0():
    r = Run().step(10).step(10, **AWAY)  # seated, then taken away without a pull
    r.step(10, pin=0.017, **AWAY).step(30, pin=0.017, clear_left=0.06, **AWAY)  # pin out afterwards, hands clear
    assert r.rung() == 0.0
    assert r.h.peaks.get("pin_seated", 0.0) == 0.0


def test_late_pull_without_ever_seating_scores_0():
    assert Run().step(10, **AWAY).step(40, pin=0.02, clear_left=0.06, **AWAY).rung() == 0.0


def test_late_pull_of_any_depth_scores_0():
    for pin in (0.0049, 0.0050, 0.010, 0.030):
        assert Run().step(10).step(10, **AWAY).step(10, pin=pin, **AWAY).rung() == 0.0


def test_pull_finished_after_the_detacher_leaves_gets_only_the_seated_part():
    r = Run().step(10).step(5, pin=0.010)  # 10 mm pulled with the detacher seated
    r.step(10, pin=0.010, **AWAY).step(30, pin=0.017, clear_left=0.06, **AWAY)  # the rest after it left
    assert r.rung() == 0.25  # credit for the 10 mm only, never success
    assert T.ladder(r.h)[0] < 1.0


def test_pull_made_in_place_keeps_its_credit_after_the_detacher_leaves():
    r = Run().step(10).step(5, pin=0.0175)  # out 17.5 mm with the detacher seated
    r.step(30, pin=0.017, clear_left=0.06, **AWAY)
    assert r.rung() == 1.0  # leaving afterwards is the demo's own ending


def test_no_permanent_record_of_a_release():
    r = Run().step(10).step(10, **AWAY)  # seated for a while, then away: no clutch state survives in the record
    assert set(r.h.evers) == set()
    assert r.rung() == 0.0


def test_pin_travel_with_the_detacher_away_is_not_a_violation():
    r = Run().step(3, pin=0.004, **AWAY).step(10).step(5, pin=0.016).step(30, pin=0.016, clear_left=0.06)
    assert r.rung() == 1.0  # a harmless earlier tug does not zero the run


def test_ladder_rungs():
    assert Run().step(6).step(2, pin=0.0049).rung() == 0.0  # 4.9 mm is not 5
    assert Run().step(6).step(2, pin=0.0050).step(2, pin=0.0).rung() == 0.25  # peak 5 mm, then pushed back in
    assert Run().step(6).step(2, pin=0.0149).rung() == 0.25
    assert Run().step(6).step(2, pin=0.0150).step(2, pin=0.0).rung() == 0.5  # 15 mm seated, pushed back in
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
    assert r.h.peaks.get("pin_seated", 0.0) == 0.01
    r.h.reset()
    assert T.ladder(r.h)[0] == 0.0 and not r.h.peaks


def test_variants_listed():
    assert {"nothing", "no_detacher", "offset_detacher", "drag_cuff", "late_pull"} <= set(T.VARIANTS)


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
    assert T.DOME_PROUD - T.POCKET_D == pytest.approx(0.0015)  # the floor presses the dome in 1.5 mm at the rim
    assert T.PLUNGER_TRAVEL == pytest.approx(0.0015)
    assert T.PIN_STROKE > T.OUT_FULL


def test_latch_geometry_holds_and_releases():
    """The bolt sits in the shaft's groove (engaged) and clears the shaft once the dome is pressed in."""
    assert -T.SHAFT_R < T.BOLT_TIP_Y < -T.CORE_R  # tip 0.5 mm inside the shaft outline, clear of the core
    assert T.GROOVE_Z1 - T.GROOVE_Z0 > T.BOLT_H  # the bolt fits the groove
    pressed = 0.0006  # 0.6 mm of dome travel draws the tip out past the shaft surface
    assert T.BOLT_TIP_Y - pressed < -T.SHAFT_R
    # at the loosest seat the rim may be 0.5 mm off the face; the press is still more than the 0.6 mm needed
    assert T.PLUNGER_TRAVEL - T.SEAT_GAP[1] > pressed
    assert 0.006 < T.SHAFT_Z0  # the plunger stem (top at 6 mm) runs under the pin's lower end
    assert T.SHAFT_Z0 + T.OUT_FULL > T.BODY[2]  # a pin out 15 mm has its tip clear of the housing top


def test_a_pulled_pin_stands_clear_of_the_bolt_and_a_part_pulled_pin_does_not():
    bolt = (T.GROOVE_Z0 + 0.0002, T.GROOVE_Z1 - 0.0002)

    def shaft_at_bolt_height(pull):  # the lower shaft piece, shifted up by the pull, overlaps the bolt's height
        return T.SHAFT_Z0 + pull < bolt[1] and T.GROOVE_Z0 + pull > bolt[0]

    assert shaft_at_bolt_height(0.005)  # a 5 mm pull leaves shaft at the bolt: the bolt rides on it, no re-lock
    assert not shaft_at_bolt_height(T.OUT_FULL)  # a 15 mm pull has the whole shaft above the bolt
