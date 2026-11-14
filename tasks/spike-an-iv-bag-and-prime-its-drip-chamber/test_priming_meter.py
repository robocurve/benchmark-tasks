import pytest

from priming_meter import PrimingMeter


def cycle(meter: PrimingMeter, closed: float = -0.003) -> None:
    meter.observe(-0.047, near=True, locked=True)
    meter.observe(closed, near=True, locked=True)
    meter.observe(-0.047, near=True, locked=True)


def test_two_full_measured_squeezes_reach_half_height():
    meter = PrimingMeter()
    cycle(meter)
    assert meter.fill_height == pytest.approx(0.011)
    assert not meter.primed
    cycle(meter)
    assert meter.fill_height == pytest.approx(0.022)
    assert meter.primed


def test_partial_compression_adds_proportional_height():
    meter = PrimingMeter()
    cycle(meter, closed=-0.0235)
    assert meter.fill_height == pytest.approx(0.0055)


def test_squeeze_before_lock_or_away_from_chamber_does_not_count():
    meter = PrimingMeter()
    meter.observe(-0.003, near=True, locked=False)
    meter.observe(-0.047, near=True, locked=False)
    meter.observe(-0.003, near=False, locked=True)
    meter.observe(-0.047, near=False, locked=True)
    assert meter.fill_height == 0.0


def test_third_full_squeeze_is_over_target_but_not_yet_a_spill():
    meter = PrimingMeter()
    for _ in range(3):
        cycle(meter)
    assert meter.fill_height == pytest.approx(0.033)
    assert not meter.primed
    assert not meter.spilled


def test_fourth_squeeze_latches_spill_and_never_scores():
    meter = PrimingMeter()
    for _ in range(4):
        cycle(meter)
    assert meter.fill_height == pytest.approx(meter.usable_height)
    assert meter.spilled
    assert not meter.primed

