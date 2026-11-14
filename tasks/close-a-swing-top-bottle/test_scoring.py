"""Offline checks of the bail mechanism, the scoring rule and the bottle's geometry (no simulator):
python -m pytest test_scoring.py"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bottlekit as K  # noqa: E402
import task as T  # noqa: E402

UPRIGHT = (0.0, 0.0, 0.0, 1.0)
D = math.radians


@pytest.fixture(autouse=True)
def fresh():
    T._clear()
    yield
    T._clear()


# -- the bail: an over-centre lever -----------------------------------------------------------------------------------

def test_open_lever_and_hanging_stopper_swing_freely():
    assert K.bail_targets(K.STOPPER_OPEN, K.LEVER_OPEN, False, False) == (None, None, False)


def test_lever_springs_back_short_of_its_centre_with_the_stopper_on():
    t_stop, t_lever, sealed = K.bail_targets(D(-1.0), D(-50.0), False, False)
    assert t_stop is None and t_lever == K.LEVER_TIGHT and not sealed


def test_lever_snapping_over_its_centre_seals_a_seated_stopper():
    t_stop, t_lever, sealed = K.bail_targets(D(-1.0), D(-30.0), False, False)
    assert sealed and t_lever > 0.0 and t_stop < 0.0          # both pressed onto their stops


def test_lever_shut_on_nothing_does_not_seal():
    assert K.bail_targets(K.STOPPER_OPEN, D(0.0), False, False)[2] is False


def test_stopper_put_on_after_the_lever_is_shut_is_not_clamped():
    assert K.bail_targets(D(-1.0), D(0.0), False, True)[2] is False


def test_seal_holds_while_the_lever_stays_over_and_breaks_when_it_is_pulled_back():
    assert K.bail_targets(D(2.0), D(-10.0), True, True)[2] is True
    assert K.bail_targets(D(2.0), D(-60.0), True, True)[2] is False


# -- scoring ------------------------------------------------------------------------------------------------------------

def test_nothing_done_scores_zero():
    assert T.score({}) == 0.0


def test_stopper_on_but_not_sealed_is_half():
    T._STATE.update(seated=True)
    assert T.score({}) == 0.5


def test_half_needs_the_bottle_standing():
    T._STATE.update(seated=True, upright=False)
    assert T.score({}) == 0.0


def test_sealed_upright_and_let_go_for_two_seconds_is_full_marks():
    T._STATE.update(done=True)
    assert T.score({}) == 1.0 and T.success({})


def test_tipping_the_bottle_zeroes_the_trial():
    T._STATE.update(done=True, tipped=True)
    assert T.score({}) == 0.0


def test_tilt_and_standing():
    assert T.tilt(UPRIGHT) == pytest.approx(0.0)
    lean = (math.sin(D(6.0)), 0.0, 0.0, math.cos(D(6.0)))      # 12 degrees about x
    assert T.tilt(lean) == pytest.approx(D(12.0))
    assert T.standing(T.bottle_start(), UPRIGHT)
    assert not T.standing(T.bottle_start(), lean)
    assert not T.standing(T.bottle_start() + [0.0, 0.0, 0.02], UPRIGHT)   # lifted off the table


# -- geometry -----------------------------------------------------------------------------------------------------------

def test_seated_stopper_sits_on_the_mouth():
    b = T.bottle_start()
    p = K.stopper_point(b, UPRIGHT, 0.0)
    assert p[:2] == pytest.approx(b[:2]) and p[2] == pytest.approx(b[2] + K.MOUTH_Z + K.STOPPER_H / 2)


def test_open_stopper_and_lever_lie_either_side_of_the_neck():
    b = T.bottle_start()
    assert K.stopper_point(b, UPRIGHT, K.STOPPER_OPEN)[1] < b[1] - K.LIP_R
    assert K.paddle_point(b, UPRIGHT, K.LEVER_OPEN)[1] > b[1] + K.LIP_R


def test_closed_lever_lies_down_the_neck_outside_the_glass():
    p = K.paddle_point(np.zeros(3), UPRIGHT, 0.0)
    assert p[2] < K.PIVOT_Z and p[1] < -max(r for r, _, _ in K.SHOULDER) and p[1] > -K.BODY_R - 0.012


def test_centre_lies_between_open_and_closed():
    assert K.LEVER_OPEN < K.LEVER_TIGHT < K.LEVER_CENTRE < 0.0
    assert K.BAIL_K * (K.LEVER_CENTRE - K.LEVER_TIGHT) == pytest.approx(0.21, abs=0.01)   # N m at the centre
