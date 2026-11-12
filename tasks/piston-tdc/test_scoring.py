"""Check the piston-stop measurement requirement without Isaac Sim."""

import importlib.util
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location(
    "piston_tdc_scoring", Path(__file__).parent / "piston_tdc" / "scoring.py"
)
scoring = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scoring)


@pytest.fixture
def scorer():
    return scoring.TdcScore(20, 28.47)


def contact(scorer, side):
    return scorer.update(side * 28.47, 0.0, True, False, [0.0, 1.0], [0.0, 0.2], 0.0)


def finish(scorer, steps=20, **changes):
    state = dict(theta_deg=0.0, omega_dps=0.0, stop_seated=False, stop_in_tray=True,
                 grip_open=[1.0, 1.0], tcp_clear_m=[0.2, 0.2], gap_mm=5.0)
    state.update(changes)
    for _ in range(steps):
        score = scorer.update(**state)
    return score


def test_direct_turn_to_tdc_without_measurements_earns_zero(scorer):
    assert finish(scorer, steps=100) == 0.0
    assert not scorer.success
    assert scorer.window_s == 0.0


@pytest.mark.parametrize("side", [-1, 1])
def test_one_contact_cannot_complete_the_task(scorer, side):
    assert contact(scorer, side) == 0.25
    assert finish(scorer, steps=100) == 0.25
    assert not scorer.success


def test_repeated_contact_on_one_side_is_not_two_measurements(scorer):
    for _ in range(40):
        contact(scorer, 1)
    assert finish(scorer) == 0.25
    assert not scorer.success


@pytest.mark.parametrize("sides", [(1, -1), (-1, 1)])
def test_both_contacts_and_one_second_at_tdc_complete_the_task(scorer, sides):
    assert contact(scorer, sides[0]) == 0.25
    assert contact(scorer, sides[1]) == 0.5
    assert finish(scorer, steps=19) == 0.5
    assert not scorer.success
    assert finish(scorer, steps=1) == 1.0
    assert scorer.success


@pytest.mark.parametrize("changes", [
    {"theta_deg": 3.0}, {"omega_dps": 2.0}, {"stop_in_tray": False},
    {"grip_open": [0.0, 1.0]}, {"tcp_clear_m": [0.0, 0.2]},
])
def test_both_measurements_do_not_replace_final_conditions(scorer, changes):
    contact(scorer, 1)
    contact(scorer, -1)
    assert finish(scorer, steps=100, **changes) == 0.5
    assert not scorer.success


def test_interrupted_final_hold_restarts_the_one_second_window(scorer):
    contact(scorer, 1)
    contact(scorer, -1)
    finish(scorer, steps=19)
    finish(scorer, steps=1, omega_dps=2.0)
    assert finish(scorer, steps=19) == 0.5
    assert finish(scorer, steps=1) == 1.0


def test_a_gap_or_a_moving_crank_does_not_earn_contact_credit(scorer):
    assert scorer.update(28.47, 0.0, True, False, [0.0, 1.0], [0.0, 0.2], 0.31) == 0.0
    assert scorer.update(-28.47, 2.0, True, False, [0.0, 1.0], [0.0, 0.2], 0.0) == 0.0
    assert finish(scorer) == 0.0


def test_forcing_the_seated_stop_still_fails(scorer):
    contact(scorer, 1)
    assert scorer.update(20.0, 0.0, True, False, [0.0, 1.0], [0.0, 0.2], -1.0) == 0.0
    assert scorer.failed
    assert finish(scorer) == 0.0


def test_reset_clears_both_contact_credits(scorer):
    contact(scorer, 1)
    contact(scorer, -1)
    scorer.reset()
    assert finish(scorer) == 0.0
    assert not scorer.success
