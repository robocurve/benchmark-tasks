"""Check proof timing, end state credit and permanent safety failures."""

from dataclasses import replace

import pytest
from task import Sample, Trial

INSTALLED = Sample(guide="wide_guide", keeper_closed=True, hands_clear=True)


def test_installed_guide_must_clear_sleeve_and_hold_at_mark():
    trial = Trial()
    assert trial.step(0, INSTALLED) == 0.5
    trial.step(1, replace(INSTALLED, press=True))
    assert trial.step(2, replace(INSTALLED, travel=0.050)) == 0.5
    passed = replace(INSTALLED, travel=0.048, sleeve_clear=True)
    assert trial.step(3, passed) == 0.5
    assert trial.step(4.99, passed) == 0.5
    assert trial.step(5, passed) == 1.0


def test_drawer_travel_alone_cannot_score_and_late_fit_cannot_pass():
    trial = Trial()
    assert trial.step(1, Sample(press=True, hands_clear=True)) == 0
    assert trial.step(2, Sample(travel=0.05, sleeve_clear=True, hands_clear=True)) == 0
    assert trial.step(5, replace(INSTALLED, travel=0.05, sleeve_clear=True)) == 0.5


@pytest.mark.parametrize(
    "violation", [{"plug_gap": 0.0021}, {"damaged": True}, {"assisted": True}]
)
def test_violation_remains_zero_after_state_restored(violation):
    trial = Trial()
    trial.step(1, replace(INSTALLED, press=True))
    assert trial.step(1.1, replace(INSTALLED, **violation)) == 0
    assert trial.step(2, replace(INSTALLED, travel=0.05, sleeve_clear=True)) == 0
    assert trial.step(5, replace(INSTALLED, travel=0.05, sleeve_clear=True)) == 0
