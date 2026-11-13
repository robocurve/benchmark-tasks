"""Offline checks of the scoring rule (no simulator): python -m pytest test_scoring.py

The scorer works from what the task file's step hook records for each syringe (unscrewed, off, cap slid off its hub,
knocked off its place), so the tests set those records directly.
"""

from __future__ import annotations

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task as T  # noqa: E402


@pytest.fixture(autouse=True)
def fresh():
    T._clear()
    yield
    T._clear()


def mark(i, **kw):
    T._STATE[T.NAMES[i]].update(kw)


def score():
    return T.score({})


def test_nothing_done_scores_zero():
    assert score() == 0.0


def test_all_off_scores_one():
    for i in range(T.N):
        mark(i, free=True, off=True)
    assert score() == 1.0
    assert T.success({})


def test_one_off_scores_a_third():
    mark(0, free=True, off=True)
    assert score() == pytest.approx(1 / 3)


def test_unscrewed_but_still_on_scores_half_each():
    mark(0, free=True)
    assert score() == pytest.approx(0.5 / 3)


def test_a_bared_needle_zeroes_everything():
    for i in range(T.N):
        mark(i, free=True, off=True)
    mark(1, exposed=True)
    assert score() == 0.0
    assert not T.success({})


def test_a_knocked_syringe_earns_nothing():
    for i in range(T.N):
        mark(i, free=True, off=True)
    mark(2, knocked=True)
    assert score() == pytest.approx(2 / 3)


def test_constants_are_consistent():
    assert T.CAP_POP < T.EXPOSED          # the fit lets go before the cap is off the hub
    assert T.SEATED < T.OFF <= T.LIFT_MAX
    assert T.CAP_SPRING * T.CAP_POP == pytest.approx(4.0)     # the cap's friction fit holds up to 4 N
