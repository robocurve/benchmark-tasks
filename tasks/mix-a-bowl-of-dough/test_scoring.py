"""Offline checks of the mixing and scoring rules (no simulator): python -m pytest test_scoring.py"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task as T  # noqa: E402


@pytest.fixture(autouse=True)
def fresh():
    T._clear()
    T._STATE["in_bowl"] = np.ones(len(T.KINDS), bool)
    yield
    T._clear()


def wet(share):
    n = round(share * len(T.FLOUR))
    T._STATE["kinds"][T.FLOUR[:n]] = 2


def test_nothing_done_scores_zero():
    assert T.score({}) == 0.0


def test_all_mixed_scores_one():
    wet(1.0)
    assert T.score({}) == 1.0
    assert T.success({})


def test_full_marks_from_95_percent():
    wet(0.95)
    assert T.score({}) == 1.0
    T._clear()
    T._STATE["in_bowl"] = np.ones(len(T.KINDS), bool)
    wet(0.5)
    assert T.score({}) == pytest.approx(0.5 / 0.95, abs=0.01)


def test_spill_or_tip_zeroes_the_trial():
    wet(1.0)
    T._STATE["spilled"] = True
    assert T.score({}) == 0.0
    T._STATE["spilled"], T._STATE["tipped"] = False, True
    assert T.score({}) == 0.0


def test_mixed_flour_outside_the_bowl_does_not_count():
    wet(1.0)
    T._STATE["in_bowl"][T.FLOUR[: len(T.FLOUR) // 2]] = False
    assert T.score({}) == pytest.approx(0.5 / 0.95, abs=0.01)


def pair(gap=0.0):
    """One dry flour bead touching one wet bead."""
    pos = np.array([[0.0, 0.0, 0.0], [2 * T.BEAD_R + gap, 0.0, 0.0]])
    return np.array([0, 1]), np.zeros(2), pos


def test_flour_worked_by_the_spoon_is_wetted():
    kinds, rubbed, pos = pair()
    vel = np.array([[0.05, 0.0, 0.0], [0.0, 0.0, 0.0]])          # sliding at 5 cm/s against the liquid
    head = np.zeros(3)
    for _ in range(3):                                            # 3 steps x 0.25 cm = 0.75 cm: not yet
        kinds, rubbed = T.wet_step(kinds, rubbed, pos, vel, head)
    assert kinds[0] == 0
    kinds, rubbed = T.wet_step(kinds, rubbed, pos, vel, head)       # 1 cm in total
    assert kinds[0] == 2


def test_resting_contact_and_far_from_the_spoon_stay_dry():
    kinds, rubbed, pos = pair()
    still = np.zeros((2, 3))
    for _ in range(50):
        kinds, rubbed = T.wet_step(kinds, rubbed, pos, still, np.zeros(3))
    assert kinds[0] == 0                                         # lying against the liquid
    sliding = np.array([[0.05, 0.0, 0.0], [0.0, 0.0, 0.0]])
    far = np.array([0.10, 0.0, 0.0])                             # a shaken bowl: sliding, but the spoon is 10 cm away
    for _ in range(50):
        kinds, rubbed = T.wet_step(kinds, rubbed, pos, sliding, far)
    assert kinds[0] == 0


def test_nothing_counts_while_the_spoon_rests():
    kinds, rubbed, pos = pair()
    vel = np.array([[0.05, 0.0, 0.0], [0.0, 0.0, 0.0]])          # beads churned (by a hand, say) next to a resting spoon
    for _ in range(50):
        kinds, rubbed = T.wet_step(kinds, rubbed, pos, vel, np.zeros(3), head_speed=0.0)
    assert kinds[0] == 0


def test_beads_not_touching_stay_dry():
    kinds, rubbed, pos = pair(gap=0.005)
    vel = np.array([[0.05, 0.0, 0.0], [0.0, 0.0, 0.0]])
    for _ in range(50):
        kinds, rubbed = T.wet_step(kinds, rubbed, pos, vel, np.zeros(3))
    assert kinds[0] == 0


def test_batch_and_spoon_do_not_overlap():
    pos = np.asarray(T.POSITIONS)
    assert len(T.FLOUR) == T.N_FLOUR and int((T.KINDS_NP == 1).sum()) == T.N_WET
    assert not any(T._in_spoon(p, clearance=T.BEAD_R) for p in pos)
    d = np.linalg.norm(pos[:, None] - pos[None], axis=2) + np.eye(len(pos))
    assert d.min() >= 2 * T.BEAD_R - 0.0015     # the layers start at most 1.5 mm into each other; they settle in GRACE
