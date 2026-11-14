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


CLIPPED = T.OPEN_ON_WIRE                 # a peg's spring angle with its jaws closed on the wire and the photo


def holds(pos, spring=CLIPPED, photo=None):
    return T.holds_near_corner(pos, T.PEG_QUAT, spring, T.photo_start() if photo is None else photo, T.PHOTO_QUAT)


def test_a_peg_at_the_near_corner_holds_it():
    pos = T.corner_pivot(-1)
    assert T.peg_on_line(pos, T.PEG_QUAT)
    g = T.grip_check(pos, T.PEG_QUAT, CLIPPED, T.photo_start(), T.PHOTO_QUAT)
    assert g["closed"] and g["wire"] and g["photo"]
    assert holds(pos)


def test_a_peg_on_the_wire_beside_the_photo_does_not_hold_it():
    """On the wire just past the photo's near end: closed on the wire, but the photo is not in its jaws."""
    beside = T.corner_pivot(-1) + [-0.012, 0.0, 0.0]
    assert T.peg_on_line(beside, T.PEG_QUAT)
    g = T.grip_check(beside, T.PEG_QUAT, CLIPPED, T.photo_start(), T.PHOTO_QUAT)
    assert g["wire"] and not g["photo"]
    assert not holds(beside)
    assert not holds(T.corner_pivot(-1) + [-0.05, 0.0, 0.0])


def test_a_peg_on_the_wire_with_the_photo_outside_its_jaws_does_not_hold_it():
    """At the corner, on the wire, but with the photo hanging outside the jaws (in front of the peg's leg)."""
    photo_outside = T.photo_start() + [0.0, 0.006, 0.0]
    assert T.peg_on_line(T.corner_pivot(-1), T.PEG_QUAT)
    assert not holds(T.corner_pivot(-1), photo=photo_outside)


def test_a_peg_squeezed_open_or_shut_on_nothing_does_not_hold_it():
    assert not holds(T.corner_pivot(-1), spring=math.radians(-10.7))     # still squeezed open in the fingers
    assert not holds(T.corner_pivot(-1), spring=0.0)                     # jaws shut on nothing


def test_a_peg_elsewhere_does_not():
    above_the_wire = T.corner_pivot(-1) + [0.0, 0.0, 0.03]
    assert not T.peg_on_line(above_the_wire, T.PEG_QUAT)
    assert not holds(above_the_wire)
    on_the_basket = T.spare_start(0)
    assert not T.peg_on_line(on_the_basket, T.PEG_QUAT)


def test_the_first_peg_holds_the_far_corner_not_the_near_one():
    pos = T.corner_pivot(+1)
    assert T.peg_on_line(pos, T.PEG_QUAT)
    assert T.grips_wire_and_photo(pos, T.PEG_QUAT, CLIPPED, T.photo_start(), T.PHOTO_QUAT)
    assert not holds(pos)


# -- taking a spare peg off the basket ----------------------------------------------------------------------------------

def test_a_spare_peg_on_the_rim_is_not_taken():
    pos = T.spare_start(0)
    assert T.peg_on_rim(pos, T.PEG_QUAT) and not T.taken_off(pos, T.PEG_QUAT)


def test_a_peg_slid_along_the_rim_is_not_taken():
    for dx in (-0.025, 0.025):
        slid = T.spare_start(0) + [dx, 0.0, 0.0]
        assert T.peg_on_rim(slid, T.PEG_QUAT) and not T.taken_off(slid, T.PEG_QUAT)


def test_a_peg_lifted_off_the_rim_is_taken():
    lifted = T.spare_start(0) + [0.0, 0.0, 0.03]
    assert not T.peg_on_rim(lifted, T.PEG_QUAT) and T.taken_off(lifted, T.PEG_QUAT)
    assert T.taken_off(T.corner_pivot(-1), T.PEG_QUAT)                    # carried to the wire


def test_a_peg_knocked_into_the_basket_is_not_taken():
    dropped = np.array([T.PEGS_X[0], T.BASKET_Y - 0.05, T.TOP + 0.02])
    assert not T.peg_on_rim(dropped, T.PEG_QUAT) and not T.taken_off(dropped, T.PEG_QUAT)


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
