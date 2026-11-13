"""Offline checks of the scoring rule and the disc geometry (no simulator): python -m pytest test_scoring.py"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import disckit as K  # noqa: E402
import task as T  # noqa: E402


@pytest.fixture(autouse=True)
def fresh():
    T._clear()
    yield
    T._clear()


def test_nothing_done_scores_zero():
    assert T.score({}) == 0.0


def test_tiers():
    T._STATE["off_hub"] = True
    assert T.score({}) == 0.25
    T._STATE["in_slot"] = True
    assert T.score({}) == 0.5
    T._STATE["taken"] = True
    assert T.score({}) == 1.0
    assert T.success({})


def test_a_dropped_disc_zeroes_the_trial():
    T._STATE.update(off_hub=True, in_slot=True, taken=True, dropped=True)
    assert T.score({}) == 0.0


def test_geometry_of_a_flat_disc():
    pos = T.disc_home()
    axis, low, depth = T.disc_geometry(pos, (0.0, 0.0, 0.0, 1.0))
    assert axis[2] == pytest.approx(1.0)
    assert low == pytest.approx(pos[2] - K.DISC_T / 2)
    assert depth == pytest.approx(pos[0] + K.DISC_R - T.FACE_X)


def test_geometry_of_an_upright_disc_in_the_slot():
    pos = np.array([T.FACE_X + 0.02, T.SLOT_Y, T.TOP + T.SLOT_Z])
    axis, low, depth = T.disc_geometry(pos, T.UPRIGHT_Q)
    assert abs(axis[1]) == pytest.approx(1.0)
    assert low == pytest.approx(pos[2] - K.DISC_R)
    assert depth == pytest.approx(0.08)
    assert T.centred(pos, axis)
    assert low > T.TOP + T.SLOT_Z - T.SLOT_H / 2                    # a centred disc clears the slot's bottom
    off = pos + [0.0, 0.005, 0.0]
    assert not T.centred(off, axis)
    tilted = (math.sin(math.radians(60)), 0.0, 0.0, math.cos(math.radians(60)))   # 30 degrees off upright
    assert not T.centred(pos, T.disc_geometry(pos, tilted)[0])


def test_constants_are_consistent():
    assert T.IN_SLOT < T.TAKE_AT < 2 * K.DISC_R
    assert T.SEATED - K.DISC_R > T.FACE_X                            # a seated disc is wholly inside
    assert T.SEATED + K.DISC_R < T.FACE_X + T.BAY                    # ... and inside the drive bay
    assert 2 * K.DISC_R < T.SLOT_H and K.DISC_T < T.SLOT_W < T.MOUTH_W
    assert K.HOLE_R > K.POST_R and K.TOOTH_OUT > K.HOLE_R > K.TOOTH_OUT - K.TOOTH_STROKE   # the teeth hold, then let go
    assert K.BUTTON_TRAVEL > -T.PRESSED
