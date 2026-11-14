"""Offline checks of the scoring rule (no simulator): python -m pytest test_scoring.py

The scorer works from what the task file's step hook records for each syringe (unscrewed, off, cap slid off its hub,
knocked off its place), so the tests set those records directly.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
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


def test_all_in_the_container_scores_one():
    for i in range(T.N):
        mark(i, free=True, off=True, in_bin=True)
    assert score() == 1.0
    assert T.success({})


def test_one_in_the_container_scores_a_third():
    mark(0, free=True, off=True, in_bin=True)
    assert score() == pytest.approx(1 / 3)


def test_off_but_not_in_the_container_scores_half():
    mark(0, free=True, off=True)
    assert score() == pytest.approx(0.5 / 3)


def test_unscrewed_but_still_on_scores_half_each():
    mark(0, free=True)
    assert score() == pytest.approx(0.5 / 3)


def test_a_bared_needle_zeroes_everything():
    for i in range(T.N):
        mark(i, free=True, off=True, in_bin=True)
    mark(1, exposed=True)
    assert score() == 0.0
    assert not T.success({})


def test_a_knocked_syringe_earns_nothing():
    for i in range(T.N):
        mark(i, free=True, off=True, in_bin=True)
    mark(2, knocked=True)
    assert score() == pytest.approx(2 / 3)


def test_the_sharps_container():
    mid = np.array([np.mean(T.BIN_X), np.mean(T.BIN_Y), T.TOP + 0.008])
    assert T.in_bin(mid)
    assert not T.in_bin(mid + [0.0, 0.0, T.BIN_H])                       # on or above the lid
    assert not T.in_bin(np.array([T.BIN_X[1] + 0.01, mid[1], mid[2]]))   # beside it
    assert T.BIN_Y[1] < min(T.FLANGE_Y - 0.104, 0.0) + 0.02              # it starts beyond the caps' tips


def test_the_lid_opening_takes_a_capped_needle():
    """The opening lies inside the lid and is wider than a capped needle is long (46 mm) both ways."""
    assert T.BIN_X[0] < T.OPENING_X[0] < T.OPENING_X[1] < T.BIN_X[1]
    assert T.BIN_Y[0] < T.OPENING_Y[0] < T.OPENING_Y[1] < T.BIN_Y[1]
    assert min(T.OPENING_X[1] - T.OPENING_X[0], T.OPENING_Y[1] - T.OPENING_Y[0]) > 0.046 + 0.02
    names = {o.name for o in T.OBJECTS}
    assert {"lid_left", "lid_right", "lid_near", "lid_far"} <= names and not any("tray" in n for n in names)


def test_nothing_holds_the_syringe_when_its_needle_is_swapped():
    """The swap to a loose needle makes the syringe's own needle parts light first; no root pinning is left."""
    import inspect

    assert not hasattr(T, "PIN_STEPS")
    assert "hold_root" not in inspect.getsource(T._watch)
    assert T.SEATED < T.LIGHTEN_AT < T.OFF and T.LIGHT <= 0.01


def test_constants_are_consistent():
    assert T.CAP_POP < T.EXPOSED          # the fit lets go before the cap is off the hub
    assert T.SEATED < T.OFF <= T.LIFT_MAX
    assert T.HIDDEN < 0.0                                                # the syringe's own needle goes into the barrel
    assert T.CAP_SPRING * T.CAP_POP == pytest.approx(4.0)     # the cap's friction fit holds up to 4 N
