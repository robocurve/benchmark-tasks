"""The knob-to-stylus rule: 35.5 mm per turn, 20 degrees of slack after a reversal, clockwise moves right / up."""

import numpy as np

from rdb_etch_triangle import layout
from rdb_etch_triangle.stylus import TRAVEL_HI, TRAVEL_LO, Stylus, unwrap

HALF = layout.BACKLASH / 2


def turn(stylus, left_deg, right_deg, steps=50):
    start = stylus.knobs.copy()
    end = start + np.radians((left_deg, right_deg))
    for k in range(1, steps + 1):
        stylus.update(start + (end - start) * k / steps)
    return stylus.point.copy()


def test_one_turn_moves_the_line_35_5_mm_after_the_slack():
    s = Stylus()                                     # slack centred: the first 10 degrees do nothing
    assert np.allclose(turn(s, -10., 0.), 0.)
    turn(s, -360., 0.)                               # one clockwise turn of the left knob
    assert np.isclose(s.point[0], .0355, atol=1e-9) and s.point[1] == 0.
    turn(s, 0., -10. - 360.)                         # right knob clockwise: up
    assert np.isclose(s.point[1], .0355, atol=1e-9)


def test_reversal_takes_twenty_degrees_before_the_line_moves_back():
    s = Stylus()
    turn(s, -100., 0.)
    x = s.point[0]
    assert not s.engaged(0, +1) and s.engaged(0, -1)
    turn(s, 19.9, 0.)
    assert np.isclose(s.point[0], x)
    turn(s, 10.1, 0.)
    assert np.isclose(s.point[0], x - np.radians(10.) * layout.PER_RAD)
    assert s.engaged(0, +1)


def test_travel_ends_stop_the_stylus_but_not_the_knob():
    s = Stylus()
    turn(s, 3600., 3600., steps=400)                 # far counter-clockwise: left and down
    assert np.allclose(s.point, TRAVEL_LO)
    turn(s, -20., 0.)
    assert np.allclose(s.point, TRAVEL_LO)           # back off the end: slack first
    turn(s, -36000., -36000., steps=4000)
    assert np.allclose(s.point, TRAVEL_HI)
    assert np.allclose(TRAVEL_HI - TRAVEL_LO, layout.TRAVEL)
    assert (TRAVEL_LO < 0).all() and (TRAVEL_HI > layout.CORNERS.max(axis=0)).all()   # the triangle fits


def test_unwrap_follows_many_turns():
    angle, true = 0., 0.
    for _ in range(400):
        true += np.radians(9.)
        angle = unwrap(angle, (true + np.pi) % (2 * np.pi) - np.pi)
    assert np.isclose(angle, true)
