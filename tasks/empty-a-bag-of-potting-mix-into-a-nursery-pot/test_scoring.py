"""Offline checks of the scorer (NumPy only): python -m pytest -q test_scoring.py"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import bagplan as p  # noqa: E402
import clothbag as cb  # noqa: E402
import scoring as s  # noqa: E402

EYE = np.eye(3)
PTS, FACES, INFO = cb.mesh()


def standing_bag(origin=p.BAG_AT, rot=EYE):
    """(film vertices, tabs) of the bag standing at its rest shape."""
    film = np.asarray(origin) + PTS @ rot.T
    tabs = []
    for name, *_ in cb.TABS:
        c, size, yaw = cb.tab_box(name, PTS, INFO)
        rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        tabs.append((np.asarray(origin) + rot @ c, rot @ rz, size))
    return film, tabs


def test_levels():
    assert s.grade(0.96, 0.02, True, True, True) == 1.0
    assert s.grade(0.95, 0.03, True, True, True) == 1.0
    assert s.grade(0.96, 0.031, True, True, True) == 0.5  # spilled a little too much for full credit
    assert s.grade(0.96, 0.02, True, False, True) == 0.5  # bag still held
    assert s.grade(0.96, 0.02, True, True, False) == 0.5  # bag over the pot
    assert s.grade(0.70, 0.08, True, True, True) == 0.5
    assert s.grade(0.69, 0.02, True, True, True) == 0.25
    assert s.grade(0.30, 0.15, True, True, True) == 0.25
    assert s.grade(0.29, 0.0, True, True, True) == 0.0
    assert s.grade(0.80, 0.16, True, True, True) == 0.0  # most of it in, but too much spilled
    assert s.grade(1.0, 0.0, False, True, True) == 0.0  # pot knocked over


def test_bag_and_pot_regions():
    film, _ = standing_bag()
    inside_bag = p.BAG_AT + [0.0, 0.0, 0.05]
    in_pot = p.POT_AT + [0.0, 0.0, 0.05]
    on_bench = np.array([0.60, 0.0, p.TOP + 0.005])
    over_rim = p.POT_AT + [0.0, 0.0, p.POT_H + 0.02]
    pts = np.array([inside_bag, in_pot, on_bench, over_rim])
    assert s.in_bag(pts, film).tolist() == [True, False, False, False]
    assert s.in_pot(pts, p.POT_AT, EYE).tolist() == [False, True, False, False]
    above_mouth = p.BAG_AT + [0.0, 0.0, p.H + 0.01]
    beside = p.BAG_AT + [0.0, p.W / 2 + 0.01, 0.05]
    assert s.in_bag(np.array([above_mouth, beside]), film).tolist() == [False, False]


def test_turned_over_bag_still_holds_what_is_inside():
    rot = p.rx(math.radians(150))
    origin = p.BAG_AT + [0, 0, 0.3]
    film, _ = standing_bag(origin, rot)
    pts = origin + np.array([[0, 0, 0.05], [0, 0, 0.12], [0, 0, p.H + 0.02]]) @ rot.T
    assert s.in_bag(pts, film).tolist() == [True, True, False]


def test_tipped_pot_still_located_in_its_frame():
    rot = p.rx(math.radians(90))  # the pot lying on its side
    pt = p.POT_AT + rot @ np.array([0.0, 0.0, 0.08])
    assert s.in_pot(pt[None], p.POT_AT, rot).tolist() == [True]
    assert s.tilt(rot) == pytest.approx(math.pi / 2)


def test_bag_clear_of_pot():
    assert s.bag_clear(*standing_bag(), p.POT_AT, EYE)
    over_pot = p.POT_AT + [0.0, 0.0, 0.01]
    assert not s.bag_clear(*standing_bag(over_pot), p.POT_AT, EYE)
    high_above = p.POT_AT + [0.0, 0.0, p.POT_H + 0.05]
    assert s.bag_clear(*standing_bag(high_above), p.POT_AT, EYE)
    film, tabs = standing_bag()
    tab_on_rim = [(p.POT_AT + [p.POT_R_TOP, 0, p.POT_H], EYE, tabs[0][2])]
    assert not s.bag_clear(film, tab_on_rim, p.POT_AT, EYE)  # a tab hanging over the rim counts


def test_tally_masses_add_up():
    film, tabs = standing_bag()
    rng = np.random.default_rng(0)
    bag = p.BAG_AT + rng.uniform([-0.03, -0.05, 0.01], [0.03, 0.05, 0.10], (50, 3))
    pot = p.POT_AT + rng.uniform([-0.03, -0.03, 0.02], [0.03, 0.03, 0.12], (40, 3))
    bench = np.c_[rng.uniform(0.55, 0.65, (10, 2)), np.full(10, p.TOP + 0.005)]
    pos = np.r_[bag, pot, bench]
    radii = np.full(len(pos), 0.005)
    t = s.Tally()
    t.update(pos, radii, film, tabs, p.POT_AT, EYE, {"left": 0.09, "right": 0.09})
    m = t.metrics
    assert m["frac_pot"] == pytest.approx(0.4)
    assert m["in_bag_kg"] / t.mass0 == pytest.approx(0.5)
    assert m["frac_spill"] == pytest.approx(0.1)
    assert m["score"] == 0.25  # 40 % in the pot and 10 % spilled
