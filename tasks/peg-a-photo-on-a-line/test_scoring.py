"""Offline checks of the scoring rule and the peg geometry (no simulator): python -m pytest test_scoring.py"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import pegkit as K  # noqa: E402
import task as T  # noqa: E402


@pytest.fixture(autouse=True)
def fresh():
    T._clear()
    yield
    T._clear()


def test_nothing_done_scores_zero():
    assert T.score({}) == 0.0


def test_tiers():
    T._STATE["taken"] = True
    assert T.score({}) == 0.25
    T._STATE["on_line"] = True
    assert T.score({}) == 0.5
    T._STATE["held"] = True
    assert T.score({}) == 1.0
    assert T.success({})


def test_knocking_the_photo_off_zeroes_the_trial():
    T._STATE.update(taken=True, on_line=True, held=True, fallen=True)
    assert T.score({}) == 0.0


def test_a_peg_at_the_near_corner_holds_it():
    pos = T.corner_pivot(-1)
    assert T.peg_on_line(pos, T.PEG_QUAT)
    assert T.holds_near_corner(pos, T.PEG_QUAT, T.photo_start(), T.PHOTO_QUAT)


def test_a_peg_elsewhere_does_not():
    on_wire_far_from_photo = T.corner_pivot(-1) + [-0.05, 0.0, 0.0]
    assert T.peg_on_line(on_wire_far_from_photo, T.PEG_QUAT)
    assert not T.holds_near_corner(on_wire_far_from_photo, T.PEG_QUAT, T.photo_start(), T.PHOTO_QUAT)
    above_the_wire = T.corner_pivot(-1) + [0.0, 0.0, 0.03]
    assert not T.peg_on_line(above_the_wire, T.PEG_QUAT)
    on_the_basket = T.spare_start(0)
    assert not T.peg_on_line(on_the_basket, T.PEG_QUAT)


def test_the_first_peg_holds_the_far_corner():
    pos = T.corner_pivot(+1)
    assert T.peg_on_line(pos, T.PEG_QUAT)
    assert not T.holds_near_corner(pos, T.PEG_QUAT, T.photo_start(), T.PHOTO_QUAT)


def test_the_peg_opens_wide_enough_for_wire_and_photo():
    """Squeezed to the 10.7 degrees the demo holds it at, the jaws must clear the wire and the photo where the
    photo's top edge enters them."""
    depth = T.ABOVE + T.CLIP_DEPTH                                         # photo's top edge above the jaw tips
    opening = K.GAP + (K.PIVOT_Z - depth) * math.sin(math.radians(10.7))
    assert opening > 2 * T.LINE_R + T.PHOTO[1] + 0.0004


def test_layout_is_consistent():
    assert T.LINE_X[0] < T.PHOTO_X - T.PHOTO[0] / 2 and T.PHOTO_X + T.PHOTO[0] / 2 < T.LINE_X[1]
    top = T.photo_start()[2] + T.PHOTO[2] / 2
    assert top == pytest.approx(T.TOP + T.LINE_Z + T.ABOVE)
    assert T.photo_start()[2] - T.PHOTO[2] / 2 > T.TOP                    # the photo hangs clear of the table
    assert K.NOTCH_Z[0] - 0.003 < T.CLIP_DEPTH < K.NOTCH_Z[1]
    assert np.allclose(T.photo_top_edge(T.photo_start(), T.PHOTO_QUAT)[2], top)
