"""Scoring rules without the simulator: python -m pytest -q tasks/hole-punch/test_scoring.py"""

import numpy as np

from punch_kit import geometry as g
from punch_kit.scoring import PunchScore, release_ok

IDENT = np.array([0.0, 0.0, 0.0, 1.0])
PRESSED = g.BED + g.T          # punch tips down on the card's top face
UP = g.HEAD_REST_TIP


def card(dx=0.0, dy=0.0, dz=0.0):
    return np.r_[g.CARD_HOME + np.array([dx, dy, dz]), IDENT]


def test_release_needs_press_alignment_and_seat():
    assert release_ok(card(), PRESSED, 0) and release_ok(card(), PRESSED, 1)
    assert not release_ok(card(), UP, 0)                       # not pressed
    assert release_ok(card(dy=0.0014), PRESSED, 0)             # 1.4 mm off: inside 1.5 mm
    assert not release_ok(card(dy=0.0025), PRESSED, 0)         # 2.5 mm off its mark
    assert not release_ok(card(dx=-0.030), PRESSED, 0)         # pulled back out of the throat
    assert not release_ok(card(dz=0.002), PRESSED, 0)          # lifted off the bed


def test_score_counts_discs_in_the_die_only():
    s = PunchScore(1.0)
    c = card()
    home = [c[:3] + g.slug_offset(k) for k in (0, 1)]
    score, newly = s.update(0.0, c, PRESSED, home)
    assert newly == [0, 1] and score == 0.0                    # released but still in the card
    out = [h + np.array([0, 0, -0.010]) for h in home]
    for i in range(20):
        score, _ = s.update(0.05 * (i + 1), c, UP, out)
    assert score == 1.0 and not s.success                      # 0.95 s with both holes: not yet
    score, _ = s.update(1.05, c, UP, out)
    assert s.success                                           # held 1 s


def test_unreleased_disc_never_counts():
    s = PunchScore(1.0)
    c = card(dy=0.004)
    away = [c[:3] + g.slug_offset(k) + np.array([0, 0, -0.010]) for k in (0, 1)]
    score, newly = s.update(0.0, c, PRESSED, away)
    assert newly == [] and score == 0.0


def test_one_hole_scores_half():
    s = PunchScore(1.0)
    c = card()
    pos = [c[:3] + g.slug_offset(0) + np.array([0, 0, -0.010]), c[:3] + g.slug_offset(1)]
    s.update(0.0, c, PRESSED, [c[:3] + g.slug_offset(k) for k in (0, 1)])
    score, _ = s.update(0.05, c, UP, pos)
    assert score == 0.5 and not s.success
