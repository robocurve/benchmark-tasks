"""Offline checks of the scoring rule (no simulator): python -m pytest test_scoring.py

Each test builds a carton state the way cartonkit reports it and checks what task.score gives.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task as T  # noqa: E402
from cartonkit import CartonState  # noqa: E402

FLAT_TOP = T.TOP + 0.0082         # the highest point of the flattened carton in the demo
STANDING_TOP = T.TOP + 0.153


def carton(rack=88.0, top=FLAT_TOP, low=T.TOP, speed=0.0, x=T.NEAR_X):
    v = np.array([speed, 0.0, 0.0])
    panels = {p: (np.array([x + 0.1, T.MID_Y, T.TOP + 0.005]), np.array([0.0, 0.0, 0.0, 1.0]), v) for p in T.PANELS}
    return CartonState(pos=np.array([x, T.MID_Y, T.TOP]), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=v,
                       ang_vel=np.zeros(3), low=np.array([x - 0.01, T.MID_Y - 0.15, low]),
                       high=np.array([x + 0.36, T.MID_Y + 0.15, top]), rack_deg=rack, panels=panels)


@pytest.fixture(autouse=True)
def fresh():
    T._clear()
    yield
    T._clear()


def score(c):
    return T.score({"carton": c})


def test_flat_still_let_go_scores_one():
    assert score(carton()) == 1.0
    assert T.success({"carton": carton()})


def test_flat_the_other_way_scores_one():
    assert score(carton(rack=-88.0)) == 1.0


def test_standing_square_scores_zero():
    assert score(carton(rack=0.0, top=STANDING_TOP)) == 0.0


def test_half_flattened_scores_zero():
    assert score(carton(rack=60.0, top=T.TOP + 0.075)) == 0.0


def test_sheared_but_too_high_scores_zero():
    assert score(carton(rack=82.0, top=T.TOP + 0.03)) == 0.0


def test_still_held_scores_zero():
    T._STATE["touched"] = True
    assert score(carton()) == 0.0


def test_moving_scores_zero():
    assert score(carton(speed=0.08)) == 0.0


def test_off_the_table_scores_zero():
    assert score(carton(low=T.TOP - 0.3, top=T.TOP - 0.29)) == 0.0


def test_hand_over_carton_low_counts_as_holding():
    c = carton()
    assert T._held(c, [np.array([T.NEAR_X + 0.15, T.MID_Y, T.TOP + 0.02])])
    assert not T._held(c, [np.array([T.NEAR_X + 0.15, T.MID_Y, T.TOP + 0.08])])
    assert not T._held(c, [np.array([T.NEAR_X - 0.10, T.MID_Y, T.TOP + 0.02])])
