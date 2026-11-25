from dataclasses import replace
import math

import pytest

from scorer import Reading, Scorer, WHEEL_RADIUS_M


START = Reading(True, True, True, True, -.042, 2.75, False, False)


def crossing(scorer, slip=False, lift_at=None):
    for step in range(1, 20):
        x = START.axle_x_m + step * .005
        angle = START.spoke_angle_rad + step * .005 / WHEEL_RADIUS_M
        reading = replace(START, axle_x_m=x, spoke_angle_rad=(angle + math.pi) % (2 * math.pi) - math.pi,
                          wheel_supported=step != lift_at)
        if slip:
            reading = replace(reading, spoke_angle_rad=START.spoke_angle_rad)
        scorer.update(reading)
    return reading


def test_clear_roller_crossing_and_finish():
    scorer = Scorer()
    for _ in range(10):
        score = scorer.update(START)
    assert score == .25
    final = crossing(scorer)
    assert scorer.score == .5
    for _ in range(9):
        assert scorer.update(replace(final, rack_in_tub=True, grippers_clear=True)) == .5
    assert scorer.update(replace(final, rack_in_tub=True, grippers_clear=True)) == 1.
    assert scorer.update(replace(final, rack_in_tub=True, grippers_clear=True, rack_supported=False)) == 0.


@pytest.mark.parametrize(('slip','lift_at'), ((True,None),(False,7)))
def test_sliding_or_carrying_never_earns_crossing(slip, lift_at):
    scorer = Scorer()
    scorer.update(START)
    result = crossing(scorer, slip=slip, lift_at=lift_at)
    for _ in range(15):
        assert scorer.update(replace(result, rack_in_tub=True, grippers_clear=True)) <= .25


def test_invalid_state_and_reset():
    scorer = Scorer()
    scorer.update(replace(START, valid=False))
    for _ in range(12):
        assert scorer.update(START) == 0.
    scorer.reset()
    for _ in range(10):
        score = scorer.update(START)
    assert score == .25
