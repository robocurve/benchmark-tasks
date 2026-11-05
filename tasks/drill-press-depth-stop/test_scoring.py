"""Exercise complete proof cycles and common ways to fake the measurement."""

from dataclasses import replace

import pytest

from scoring import Proof, Sample

HOME = Sample(-0.0, -0.005, 0.015, 0.0, True, True, False)


def stroke(proof, start, depth, **changes):
    loaded = Sample(depth + 0.005, depth, 0.0, 0.0, True, True, False)
    for tick in range(41):
        sample = replace(HOME, stop_gap=depth + 0.005)
        if tick == 0:
            sample = replace(sample, pressed=True)
        if 15 <= tick <= 20:
            sample = replace(loaded, **changes)
        proof.step(start + tick / 20, sample)
    return proof.score


@pytest.mark.parametrize("depth, expected", [(0.010, 1.0), (0.006, 0.5), (0.002, 0.0), (0.018, 0.0)])
def test_two_completed_loads(depth, expected):
    proof = Proof()
    assert stroke(proof, 0, depth) == 0.0
    assert stroke(proof, 3, depth) == expected


@pytest.mark.parametrize("fault", [
    {"stop_gap": 0.007},
    {"clear": False},
    {"latch": False},
    {"speed": 0.010},
])
def test_hand_blocking_jam_and_unlocked_strokes_score_zero(fault):
    proof = Proof()
    assert stroke(proof, 0, 0.010, **fault) == 0.0
    assert stroke(proof, 3, 0.010, **fault) == 0.0
    assert proof.readings == []


def test_button_must_return_home_before_a_new_press():
    proof = Proof()
    for tick in range(101):
        proof.step(tick / 20, replace(HOME, pressed=True))
    assert proof.start is None
    assert proof.readings == []


def test_changing_setting_after_proof_revokes_credit():
    proof = Proof()
    stroke(proof, 0, 0.006)
    assert stroke(proof, 3, 0.006) == 0.5
    assert proof.step(6, replace(HOME, latch=False)) == 0.0


def test_second_proof_uses_higher_load():
    proof = Proof()
    proof.step(0, replace(HOME, pressed=True))
    proof.step(0.5, HOME)
    assert proof.force == 8
    stroke(proof, 0, 0.010)
    assert len(proof.readings) == 1
    proof.step(3, replace(HOME, pressed=True))
    proof.step(3.5, HOME)
    assert proof.force == 12


def test_transient_overtravel_limits_score():
    proof = Proof()
    for tick in range(41):
        sample = HOME
        if tick == 0:
            sample = replace(HOME, pressed=True)
        elif tick == 14:
            sample = replace(HOME, depth=0.012, travel=0.017)
        elif 15 <= tick <= 20:
            sample = Sample(0.015, 0.010, 0.0, 0.0, True, True, False)
        proof.step(tick / 20, sample)
    assert stroke(proof, 3, 0.010) == 0.5
