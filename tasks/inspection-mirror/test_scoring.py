"""Scoring checks that matter for the hidden-leak task (pure Python: no Isaac, no GPU)."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from inspection_mirror import design as d  # noqa: E402
from inspection_mirror.scoring import LeakMarkScore, mark_credit, value  # noqa: E402

LEAK = d.LEAK_X[2]


def rest_for(sc, steps, marks, smeared=False):
    for _ in range(steps):
        sc.update(marks, smeared, still=True, grip_open=[1.0, 1.0])
    return sc.score


@pytest.mark.parametrize("err_mm,expected", [(0, 1.0), (6, 1.0), (10, 0.5), (14, 0.0), (20, 0.0), (-10, 0.5)])
def test_credit_band(err_mm, expected):
    assert mark_credit(err_mm) == pytest.approx(expected)


def test_neighbouring_place_earns_nothing():
    # a mark at the next place along (20 mm away) must score 0, or a guess near the middle would collect credit
    assert value([d.LEAK_X[3]], LEAK, False) == 0.0
    assert d.ZERO_MM < min(np.diff(d.LEAK_X)) * 1000


def test_stray_mark_and_carpet_marking():
    assert value([LEAK, LEAK + 0.030], LEAK, False) == 0.0  # one right, one wrong: the repair may go to the wrong spot
    assert value(list(d.LEAK_X), LEAK, False) == 0.0  # marking every place earns nothing
    assert value([LEAK + 0.002, LEAK - 0.003], LEAK, False) == 1.0  # two dabs on the right spot are fine


def test_smear_and_no_mark_score_zero():
    assert value([LEAK], LEAK, True) == 0.0
    assert value([], LEAK, False) == 0.0


def test_blind_guess_averages_point_two():
    for guess in d.LEAK_X:  # any single fixed place, over the five equally likely leak places
        assert np.mean([value([guess], leak, False) for leak in d.LEAK_X]) == pytest.approx(0.2)


def test_score_needs_rest_and_follows_latest_rest():
    sc = LeakMarkScore(20, LEAK)
    for _ in range(40):  # gripper still closed on the marker: nothing counts
        sc.update([LEAK], False, still=True, grip_open=[0.3, 1.0])
    assert sc.score == 0.0
    assert rest_for(sc, 19, [LEAK]) == 0.0  # 0.95 s: not yet
    assert rest_for(sc, 1, [LEAK]) == 1.0 and sc.success
    sc = LeakMarkScore(20, LEAK)
    rest_for(sc, 25, [LEAK + 0.010])
    assert sc.score == pytest.approx(0.5)
    sc.update([LEAK + 0.010, LEAK + 0.030], False, still=False, grip_open=[0.3, 1.0])
    assert rest_for(sc, 25, [LEAK + 0.010, LEAK + 0.030]) == 0.0  # a later stray mark lowers the score


def test_head_clearance_geometry():
    centred = np.array([0.3, d.FACE_Y + d.GAP / 2, 0.85, 0, 0, 0, 1.0])
    clear = d.head_face_clearance(centred)
    # 57 mm head at 45 degrees is 43 mm across in a 55 mm gap: about 6 mm each side
    assert clear == pytest.approx((d.GAP - d.HEAD_D * math.sin(d.HEAD_YAW) - d.HEAD_T * math.cos(d.HEAD_YAW)) / 2,
                                  abs=1e-4)
    shifted = centred.copy()
    shifted[1] -= 0.009
    assert d.head_face_clearance(shifted) < 0  # the touch check run: 9 mm toward the face goes through it
    rolled = centred.copy()
    rolled[3:] = [math.sin(math.radians(-10) / 2), 0, 0, math.cos(math.radians(-10) / 2)]  # -10 deg roll
    assert d.head_face_clearance(rolled) < clear  # rolling the hand swings the hanging head toward the face


def test_leak_place_varies_with_seed():
    places = [d.leak_index(s) for s in range(200)]
    assert set(places) == set(range(len(d.LEAK_X)))
    assert max(np.bincount(places)) / len(places) < 0.3


def test_paint_mark_only_on_strip():
    top = d.TOP + d.CAP_TOP
    assert d.on_strip([0.40, 0.0, top + 0.001])
    assert not d.on_strip([0.40, 0.0, top + 0.004])  # hovering is not a mark
    assert not d.on_strip([0.20, 0.0, top])  # outside the strip
