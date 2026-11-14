"""Offline checks of the gearing, the scoring rule and the spinner's geometry (no simulator):
python -m pytest test_scoring.py"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import spinnerkit as K  # noqa: E402
import task as T  # noqa: E402

UPRIGHT = (0.0, 0.0, 0.0, 1.0)
CW = K.FORWARD                     # the sign of a clockwise (forward) joint speed


@pytest.fixture(autouse=True)
def fresh():
    T._clear()
    yield
    T._clear()


# -- the gearing: 1:5 through a one-way clutch ---------------------------------------------------------------------------

def test_cranking_forward_drives_the_basket_forward_and_the_crank_feels_it():
    on_basket, on_crank = K.gear(CW * 4.0, 0.0)
    assert on_basket * CW > 0                       # the basket is pushed forward ...
    assert on_crank * CW < 0                        # ... and the crank is resisted
    assert abs(on_crank) > K.RATIO * abs(on_basket) * 0.99 - 1e-9
    assert abs(on_crank) < 0.15                     # N m: about 2 N at the knob, as by hand


def test_at_its_geared_speed_the_basket_is_not_driven_further():
    on_basket, _ = K.gear(CW * 4.0, CW * 4.0 * K.RATIO)
    assert on_basket * CW < 0                       # only the lettuce's drag acts


def test_turning_the_crank_backward_does_not_drive_the_basket():
    on_basket, on_crank = K.gear(-CW * 4.0, 0.0)
    assert on_basket == 0.0
    assert on_crank * CW > 0                        # the gears still resist the crank (now going backward)


def test_the_clutch_spins_the_basket_up_over_a_second_or_two():
    on_basket, _ = K.gear(CW * 4.0, 0.0)
    assert abs(on_basket) <= K.CLUTCH_MAX
    assert K.RATIO * 4.0 * K.BASKET_I / K.CLUTCH_MAX == pytest.approx(1.7, abs=0.3)   # s to the geared speed


def test_a_spinning_basket_coasts_down():
    on_basket, on_crank = K.gear(0.0, CW * 20.0)
    assert on_basket * CW < 0 and on_crank == 0.0
    assert K.BASKET_I / K.DRAG == pytest.approx(3.9, abs=0.3)      # seconds: it coasts for several seconds


# -- scoring --------------------------------------------------------------------------------------------------------------

def test_tiers():
    assert T.score_from(0.0, False, True) == 0.0
    assert T.score_from(0.6, False, False) == 0.25
    assert T.score_from(2.5, False, False) == 0.5
    assert T.score_from(5.2, False, True) == 1.0
    assert T.score_from(5.2, False, False) == 0.5   # still holding the knob: not let go


def test_knocking_the_spinner_about_zeroes_the_trial():
    assert T.score_from(6.0, True, True) == 0.0


def test_score_reads_the_trial_record():
    T._STATE.update(crank_turns=5.0, let_go=True)
    assert T.score({}) == 1.0 and T.success({})
    T._STATE.update(knocked=True)
    assert T.score({}) == 0.0


# -- geometry -------------------------------------------------------------------------------------------------------------

def test_the_knob_goes_round_the_lid_and_inside_its_edge():
    s = T.spinner_start()
    for a in np.linspace(0.0, 2 * math.pi, 9):
        p = K.knob_point(s, UPRIGHT, a)
        assert np.linalg.norm(p[:2] - s[:2]) == pytest.approx(K.KNOB_AT)
        assert p[2] == pytest.approx(s[2] + K.CRANK_Z + K.ARM[2])
    assert K.KNOB_AT + K.KNOB_R < K.LID_R


def test_the_handle_stands_clear_of_the_knobs_circle():
    """The hand on the handle and the hand on the knob never come closer than 8.5 cm (across the table)."""
    handle_end = K.BOWL_R + K.TAB[1] - 0.017
    assert handle_end - K.KNOB_AT > 0.085


def test_lid_sits_on_the_bowl_and_the_crank_on_the_lid():
    assert K.LID_Z[0] == pytest.approx(K.BOWL_H)
    assert K.HUB_Z[0] == pytest.approx(K.LID_Z[1]) and K.CRANK_Z == pytest.approx(K.HUB_Z[1])
    assert K.BASKET_Z[1] < K.BOWL_H and K.BASKET_R < K.BOWL_R - K.WALL
