# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for tire-pressure-gauge, pure Python (no simulator). Run from the task folder: pytest -q tests"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

DT = 0.05


def sample(**kw):
    m = {"tilt": 0.5, "off": 0.0005, "gap": 0.0001, "core": 0.0015, "reading": 0.035, "pos": (0.3, -0.3, 0.7555), "speed": 0.0,
         "home_xy": 0.0, "home_z": 0.0, "clear": {"left": 0.1, "right": 0.1}, "tcp_speed": {"left": 0.0, "right": 0.0}}
    m.update(kw)
    return m


def feed(rec, rows):
    t = 0.0
    for secs, kw in rows:
        for _ in range(round(secs / DT)):
            t += DT
            rec.update(sample(**kw), t=t)
    return rec


PRESS = dict(home_xy=0.2, home_z=0.05, clear={"left": 0.1, "right": 0.0}, tcp_speed={"left": 0.0, "right": 0.02})
HOME = dict(core=0.0, gap=-0.069)


def test_full_run_scores_one():
    rec = feed(T.Record(), [(2.0, PRESS), (2.5, HOME), (2.5, HOME)])
    assert T.ladder(rec)[0] == 1.0


def test_nothing_is_zero():
    rec = feed(T.Record(), [(5.0, dict(core=0.0, gap=0.05, reading=0.0))])
    assert T.ladder(rec)[0] == 0.0


def test_angled_press_leaks_and_scores_zero():
    rec = feed(T.Record(), [(4.0, dict(PRESS, tilt=5.0, reading=0.012)), (3.0, dict(HOME, reading=0.012))])
    assert T.ladder(rec)[0] == 0.0


def test_light_press_scores_zero():
    rec = feed(T.Record(), [(4.0, dict(PRESS, core=0.0009, reading=0.0)), (3.0, dict(HOME, reading=0.0))])
    assert T.ladder(rec)[0] == 0.0


def test_short_seal_scores_zero():
    rec = feed(T.Record(), [(1.0, PRESS), (3.0, dict(HOME, reading=0.0187))])
    assert T.ladder(rec)[0] == 0.0


def test_stick_pulled_by_hand_without_seal_scores_zero():
    rec = feed(T.Record(), [(3.0, dict(HOME, reading=0.035))])
    assert T.ladder(rec)[0] == 0.0


def test_seal_held_but_reading_out_of_band_is_point_four():
    rec = feed(T.Record(), [(2.0, PRESS), (2.0, dict(HOME, reading=0.031))])
    assert T.ladder(rec)[0] == 0.4


def test_gauge_left_on_valve_is_point_seven():
    rec = feed(T.Record(), [(2.0, PRESS), (3.0, PRESS)])
    assert T.ladder(rec)[0] == 0.7


def test_gripper_still_holding_is_point_seven():
    rec = feed(T.Record(), [(2.0, PRESS), (3.0, dict(HOME, clear={"left": 0.1, "right": 0.0}))])
    assert T.ladder(rec)[0] == 0.7


def test_seal_run_resets_when_the_press_is_interrupted():
    rec = feed(T.Record(), [(1.0, PRESS), (0.5, dict(PRESS, core=0.0)), (1.0, PRESS)])
    assert "seal_hold" not in rec.H.evers


def test_reset_clears_the_whole_run():
    rec = feed(T.Record(), [(2.0, PRESS), (2.5, HOME)])
    rec.reset()
    assert T.ladder(rec)[0] == 0.0


def test_gasket_seated_but_core_not_pushed_down_scores_zero():
    # the old loophole shape: a square, firm press with nothing opening the valve
    rec = feed(T.Record(), [(4.0, dict(PRESS, core=0.0004, reading=0.0)), (3.0, dict(HOME, reading=0.0))])
    assert T.ladder(rec)[0] == 0.0


def test_pin_on_the_barrel_wall_scores_zero():
    # depressor lands on the barrel wall: gasket face 2 mm off the tip, core untouched
    rec = feed(T.Record(), [(4.0, dict(PRESS, gap=0.002, core=0.0, reading=0.0)), (3.0, dict(HOME, reading=0.0))])
    assert T.ladder(rec)[0] == 0.0


def test_press_beside_the_valve_scores_zero():
    rec = feed(T.Record(), [(4.0, dict(PRESS, off=0.040, gap=-0.007, core=0.0, reading=0.0)),
                            (3.0, dict(HOME, reading=0.0))])
    assert T.ladder(rec)[0] == 0.0


def test_valve_open_but_gasket_lifted_only_leaks():
    rec = feed(T.Record(), [(4.0, dict(PRESS, gap=0.001, reading=0.012)), (3.0, dict(HOME, reading=0.012))])
    assert T.ladder(rec)[0] == 0.0


def test_a_harmless_early_wrong_try_does_not_zero_the_run():
    rec = feed(T.Record(), [(1.0, dict(PRESS, tilt=6.0, gap=0.002, core=0.0)), (2.0, PRESS), (2.5, HOME), (2.5, HOME)])
    assert T.ladder(rec)[0] == 1.0


def test_seal_open_and_leak_predicates():
    assert T.sealed(sample())
    assert not T.sealed(sample(tilt=3.5))
    assert not T.sealed(sample(core=0.0010))
    assert not T.sealed(sample(gap=0.0006))
    assert not T.sealed(sample(off=0.003))
    assert T.leaking(sample(tilt=5.0))
    assert T.leaking(sample(gap=0.002))
    assert not T.leaking(sample(core=0.0005, tilt=5.0))
