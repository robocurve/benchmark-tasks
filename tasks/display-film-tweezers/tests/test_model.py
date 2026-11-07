"""Directional-release regression tests; no simulator or fabricated success trajectory."""
import numpy as np
import pytest
import filmkit
FRONT = f'film_{filmkit.BONDED_FROM:02d}'  # first adhesive strip
from test_scoring import initial, state


class Bond:
    enabled = True

    def GetJointEnabledAttr(self):
        return self

    def Set(self, enabled):
        self.enabled = enabled


@pytest.fixture(autouse=True)
def reset_model(monkeypatch):
    filmkit.trial.reset()
    monkeypatch.setattr(filmkit, '_bonds', [Bond() for _ in range(filmkit.N_BONDED)])


def tilt(o, name, degrees):
    a = np.deg2rad(degrees)/2
    o[name] = state(name, o[name].pos, [np.sin(a),0,0,np.cos(a)])


def test_compression_does_not_release():
    o = initial(); tilt(o, FRONT, -70)
    filmkit.advance(o, False)
    assert filmkit.trial.peeled == 0
    assert filmkit._bonds[0].enabled


def test_middle_strip_cannot_start_peel():
    o = initial(); tilt(o, 'film_05', 70)
    filmkit.advance(o, False)
    assert filmkit.trial.peeled == 0


def test_front_releases_once_and_reset_restores_count():
    o = initial(); tilt(o, FRONT, 70)
    filmkit.advance(o, False)
    assert filmkit.trial.peeled == 1
    assert not filmkit._bonds[0].enabled
    assert filmkit._bonds[1].enabled
    filmkit.advance(initial(), False)
    assert filmkit.trial.peeled == 1
    filmkit.trial.reset()
    assert filmkit.trial.peeled == 0


def test_subthreshold_opening_does_not_release():
    o = initial(); tilt(o, FRONT, 59)
    filmkit.advance(o, False)
    assert filmkit.trial.peeled == 0


def test_separated_seam_is_invalid_and_cannot_be_repaired_for_credit():
    o = initial()
    filmkit.validate_integrity(o)
    assert not filmkit.trial.invalid
    o['film_05'] = state('film_05', o['film_05'].pos + [0,0,.004])
    filmkit.validate_integrity(o)
    filmkit.validate_integrity(initial())
    assert filmkit.trial.invalid


def test_scoring_failure_does_not_freeze_material_physics():
    o = initial(); tilt(o, FRONT, 70)
    filmkit.trial.invalid = True
    filmkit.advance(o, False)
    assert filmkit.trial.peeled == 1
    assert not filmkit._bonds[0].enabled
    assert filmkit.trial.score(o) == 0.


def test_starter_zone_strip_has_no_bond():
    o = initial(); tilt(o, 'film_00', 90)
    filmkit.advance(o, False)
    assert filmkit.trial.peeled == 0
    assert all(b.enabled for b in filmkit._bonds)
