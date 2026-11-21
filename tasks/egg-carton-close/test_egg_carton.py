"""The damage record on handcrafted states (no Isaac Sim): ordinary latch pressure and a gentle lift pass; a press
onto the eggs, a drop or an upended carton fail.

Run from this folder in Arena's environment:  python -m pytest test_egg_carton.py
"""

import math
import pathlib
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import egg_carton as M  # noqa: E402

FLAT = (0.0, 0.0, 0.0, 1.0)
FORCE = {"lid": 0.0, "flap": 0.0}


@pytest.fixture(autouse=True)
def fresh(monkeypatch):
    monkeypatch.setattr(M, "_leaf_forces", lambda: dict(FORCE))   # no simulator: the hinge forces are set per step
    M._reset()
    yield
    M._reset()


def tilted(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def turned(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def carton(lid, flap, dz=0.0, vz=0.0, quat=FLAT, dx=0.0):
    """Both halves at the start (raised dz, falling at -vz m/s, slid dx along x), the lid lid degrees open, the flap
    flap degrees in."""
    def half(x, sliders):
        return SimpleNamespace(pos=np.array((M.CX + x + dx, 0.0, M.TOP + dz)), quat_xyzw=np.array(quat, float),
                               lin_vel=np.array((0.0, 0.0, vz)), sliders=sliders)
    return {"carton_back": half(0.0, {"lid": lid - M.OPEN}), "carton_front": half(0.0, {"flap": flap + M.REST})}


def step(objects, lid=0.0, flap=0.0):
    FORCE.update(lid=lid, flap=flap)
    M._count()
    return M.score(objects)


def close(press=1.3, latch_bump=4.5, lift=None):
    """The demo's closure: the lid comes down over the tucked flap, the bead snaps in with a brief bump, the arm
    presses it shut for 0.8 s with ``press`` N through the hinges, lets go, and the carton rests for 2 s. ``lift``:
    (height m, speed m/s) of a gentle lift and set-down while the lid is still open."""
    s = 0.0
    for _ in range(10):
        s = step(carton(100.0, -60.0))
    if lift:
        h, v = lift
        n = max(int(h / (v / 20.0)), 1)
        for k in range(1, n + 1):
            s = step(carton(100.0, -60.0, dz=h * k / n, vz=v))
        for k in range(n - 1, -1, -1):
            s = step(carton(100.0, -60.0, dz=h * k / n, vz=-v))
    for c in np.linspace(45.0, 6.0, 10):
        s = step(carton(c, 1.0), lid=2.0, flap=2.0)            # pressing the lid down over the flap: not latched yet
    s = step(carton(2.2, 0.8), lid=latch_bump, flap=0.5)        # the bead snaps in
    s = step(carton(2.2, 0.8), lid=latch_bump, flap=0.5)
    for _ in range(16):
        s = step(carton(2.2, 0.8), lid=press * 0.7, flap=press * 0.3)
    for _ in range(40):
        s = step(carton(2.2, 0.8), lid=0.3, flap=0.2)
    return s


def test_the_demo_closure_scores_one():
    assert close() == 1.0 and M._S["damage"] is None


def test_ordinary_latch_pressure_is_allowed():
    # a firm 4 N press to seat the latch, and a brief 8 N bump as the bead snaps in, are not damage
    assert close(press=4.0, latch_bump=8.0) == 1.0 and M._S["damage"] is None


def test_lifting_the_carton_a_few_millimetres_and_setting_it_down_is_allowed():
    assert close(lift=(0.005, 0.05)) == 1.0 and M._S["damage"] is None
    M._reset()
    assert close(lift=(0.03, 0.2)) == 1.0 and M._S["damage"] is None


def test_a_slight_tip_is_allowed():
    for _ in range(10):
        step(carton(100.0, -60.0))
    for _ in range(10):
        step(carton(100.0, -60.0, quat=tilted(5.0)))
    assert M._S["damage"] is None


def test_pressing_the_latched_lid_down_onto_the_eggs_scores_zero():
    assert close(press=12.0) == 0.0
    assert "onto the eggs" in M._S["damage"]


def test_a_crushing_press_is_remembered_after_it_stops():
    for _ in range(10):
        step(carton(2.2, 0.8))
    for _ in range(6):
        step(carton(2.2, 0.8), lid=9.0, flap=6.0)
    for _ in range(40):
        s = step(carton(2.2, 0.8), lid=0.3, flap=0.2)
    assert s == 0.0 and "onto the eggs" in M._S["damage"]


def test_the_lid_pressed_past_shut_onto_the_egg_tops_scores_zero():
    for _ in range(10):
        step(carton(100.0, -60.0))
    step(carton(-8.0, 0.8))
    assert close() == 0.0 and "lid pressed" in M._S["damage"]


def test_the_flap_pressed_onto_the_front_eggs_scores_zero():
    for _ in range(10):
        step(carton(100.0, -60.0))
    step(carton(100.0, 17.0))
    assert "flap pressed" in M._S["damage"]


def test_dropping_the_carton_scores_zero():
    for _ in range(10):
        step(carton(100.0, -60.0))
    step(carton(100.0, -60.0, dz=0.02, vz=-1.2))
    assert "dropped" in M._S["damage"]


def test_turning_the_carton_on_the_tabletop_is_not_tipping():
    for _ in range(10):
        step(carton(100.0, -60.0))
    for _ in range(10):
        step(carton(100.0, -60.0, quat=turned(40.0)))   # swung 40 degrees about the vertical, still flat
    assert M._S["damage"] is None
    assert M._tilt(turned(90.0), FLAT) < 1e-6
    assert abs(M._tilt(tilted(25.0), FLAT) - 25.0) < 1e-6


def test_a_closed_carton_slid_more_than_1_cm_still_scores_one():
    s = 0.0
    for _ in range(10):
        s = step(carton(100.0, -60.0))
    for _ in range(10):
        s = step(carton(100.0, -60.0, dx=0.03))          # nudged 3 cm along the table before closing
    for c in np.linspace(45.0, 6.0, 10):
        s = step(carton(c, 1.0, dx=0.03), lid=2.0, flap=2.0)
    for _ in range(2):
        s = step(carton(2.2, 0.8, dx=0.03), lid=4.5, flap=0.5)
    for _ in range(40):
        s = step(carton(2.2, 0.8, dx=0.03), lid=0.3, flap=0.2)
    assert s == 1.0 and M._S["damage"] is None


def test_upending_the_carton_scores_zero():
    for _ in range(10):
        step(carton(100.0, -60.0))
    step(carton(100.0, -60.0, quat=tilted(25.0)))
    assert "tipped" in M._S["damage"]
