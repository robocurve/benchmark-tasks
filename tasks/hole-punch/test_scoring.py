"""Scoring rules without the simulator: python -m pytest -q tasks/hole-punch/test_scoring.py"""

import numpy as np

from punch_kit import geometry as g
from punch_kit.scoring import PunchScore, release_ok, disc_home

IDENT = np.array([0.0, 0.0, 0.0, 1.0])
TOP = g.BED + g.T                       # punch tips down on the card's top face: not yet cut
THROUGH = g.BED - g.PAST - 0.0001       # tips just past the card's underside
UP = g.HEAD_REST_TIP


def card(dx=0.0, dy=0.0, dz=0.0, yaw=0.0):
    return np.r_[g.CARD_HOME + np.array([dx, dy, dz]), g.yaw_quat(yaw)]


def pushed(c, k, depth):
    """Disc k pushed ``depth`` out of card pose c along the card's normal."""
    return disc_home(c, k) - np.array([0.0, 0.0, depth])


def test_release_needs_full_stroke_alignment_and_seat():
    c = card()
    deep = g.T + g.PAST + 0.0001
    assert release_ok(c, THROUGH, pushed(c, 0, deep), 0) and release_ok(c, THROUGH, pushed(c, 1, deep), 1)
    assert not release_ok(c, TOP, pushed(c, 0, 0.0), 0)                # tips on the card's top face (old rule)
    assert not release_ok(c, g.BED, pushed(c, 0, g.T), 0)              # through the thickness, not past it
    assert not release_ok(c, THROUGH, pushed(c, 0, g.T * 0.5), 0)      # tip deep but the disc only half out
    assert not release_ok(c, UP, pushed(c, 0, deep), 0)                # disc moved but the punch is up
    c = card(dy=0.0014)
    assert release_ok(c, THROUGH, pushed(c, 0, deep), 0)               # 1.4 mm off: inside 1.5 mm
    c = card(dy=0.0025)
    assert not release_ok(c, THROUGH, pushed(c, 0, deep), 0)           # 2.5 mm off its mark
    c = card(dx=-0.030)
    assert not release_ok(c, THROUGH, pushed(c, 0, deep), 0)           # pulled back out of the throat
    c = card(dz=0.002)
    assert not release_ok(c, THROUGH, pushed(c, 0, deep), 0)           # lifted off the bed
    c = card(dz=0.0006)
    assert not release_ok(c, THROUGH, pushed(c, 0, deep), 0)           # held 0.6 mm up: not on the bed
    c = card(dz=-0.0015)
    # pressed 1.5 mm into the bed by the punch load (seen in the GPU demo: it blocked a full-stroke cut)
    assert release_ok(c, c[2] - g.T / 2 - g.PAST - 0.0001, pushed(c, 0, deep), 0)


def test_score_counts_discs_in_the_die_only():
    s = PunchScore(1.0)
    c = card()
    deep = [pushed(c, k, g.T + g.PAST + 0.0001) for k in (0, 1)]
    score, newly = s.update(0.0, c, THROUGH, deep)
    assert newly == [0, 1] and score == 0.0                    # cut free but still in the card
    out = [disc_home(c, k) + np.array([0, 0, -0.010]) for k in (0, 1)]
    for i in range(20):
        score, _ = s.update(0.05 * (i + 1), c, UP, out)
    assert score == 0.5 and not s.success                      # 0.95 s with both holes: not yet 1.0
    score, _ = s.update(1.05, c, UP, out)
    assert s.success and score == 1.0                          # held 1 s


def test_unreleased_disc_never_counts():
    s = PunchScore(1.0)
    c = card(dy=0.004)
    away = [disc_home(c, k) + np.array([0, 0, -0.010]) for k in (0, 1)]
    score, newly = s.update(0.0, c, THROUGH, away)
    assert newly == [] and score == 0.0


def test_one_hole_scores_half():
    s = PunchScore(1.0)
    c = card()
    s.update(0.0, c, THROUGH, [pushed(c, 0, g.T + g.PAST + 0.0001), disc_home(c, 1)])
    pos = [disc_home(c, 0) + np.array([0, 0, -0.010]), disc_home(c, 1)]
    score, _ = s.update(0.05, c, UP, pos)
    assert score == 0.5 and not s.success
