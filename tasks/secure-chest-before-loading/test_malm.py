"""Scorer tests for malm.py, run without the simulator:  python -m pytest test_malm.py  (or python test_malm.py)."""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import malm as m  # noqa: E402


def _box(pos, size, quat=(0.0, 0.0, 0.0, 1.0), vel=(0.0, 0.0, 0.0)):
    pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
    z = np.zeros(3)
    return ObjectState(pos, np.asarray(quat, float), np.asarray(vel, float), z, pos - half, pos + half)


def scene(bolt_slid=0.0, open_=(0.0, 0.0, m.OPEN0), books=None, tilt_deg=0.0):
    """Object states for a layout: bolt slid toward the robot by bolt_slid, drawers out by open_ (the top one starts
    open)."""
    t = math.radians(tilt_deg) / 2
    s = {
        "carcass_base": _box((m.CX, 0.0, m.TOP + 0.005), (m.D, m.W, 0.01), quat=(0.0, -math.sin(t), 0.0, math.cos(t))),
        "bolt": _box((m.BOLT_X0 - bolt_slid, m.LY, m.BOLT_Z), (m.BOLT_L, 0.01, 0.01)),
        "books": _box(books if books is not None else (*m.BOOKS_XY, m.TOP + m.BOOKS[2] / 2 + 0.001), m.BOOKS),
    }
    for k in range(3):
        s[f"drawer{k}"] = _box((m.F + 0.004 - open_[k], 0.0, m.DRAWER_Z[k] + 0.02), (0.008, 0.124, 0.032))
    return s


SHUT = (0.0, 0.0, 0.0)
IN_TOP = (m.F + 0.008 + 0.025, 0.02, m.DRAWER_Z[2] + m.DRAWER_FLOOR + m.BOOKS[2] / 2 + 0.002)  # drawer shut
IN_TOP_OPEN = (IN_TOP[0] - m.OPEN0, IN_TOP[1], IN_TOP[2])  # in the open drawer


def run(frames):
    m._state.clear()
    m._state.update(m._fresh())
    out = None
    for f in frames:
        out = m.score(f)
    return out


def test_nothing_done_scores_zero():
    assert run([scene()] * 5) == 0.0


def test_secured_only_scores_quarter():
    assert run([scene(), scene(bolt_slid=0.03)]) == 0.25


def test_full_procedure_scores_one():
    frames = [scene(), scene(bolt_slid=0.03), scene(bolt_slid=0.03, books=IN_TOP_OPEN),
              scene(bolt_slid=0.03, open_=SHUT, books=IN_TOP)]
    assert run(frames) == 1.0


def test_books_in_but_drawer_left_open_scores_half():
    assert run([scene(), scene(bolt_slid=0.03), scene(bolt_slid=0.03, books=IN_TOP_OPEN)]) == 0.5


def test_loading_before_securing_scores_zero_even_if_finished_later():
    frames = [scene(), scene(books=IN_TOP_OPEN), scene(bolt_slid=0.03, books=IN_TOP_OPEN),
              scene(bolt_slid=0.03, open_=SHUT, books=IN_TOP)]
    assert run(frames) == 0.0


def test_opening_another_drawer_before_securing_scores_zero():
    frames = [scene(), scene(open_=(0.0, 0.03, m.OPEN0)), scene(open_=(0.0, 0.0, m.OPEN0)), scene(bolt_slid=0.03),
              scene(bolt_slid=0.03, books=IN_TOP_OPEN), scene(bolt_slid=0.03, open_=SHUT, books=IN_TOP)]
    assert run(frames) == 0.0


def test_shutting_the_empty_drawer_first_is_fine():
    frames = [scene(), scene(open_=SHUT), scene(open_=SHUT, bolt_slid=0.03)]
    assert run(frames) == 0.25


def test_tipping_over_scores_zero():
    frames = [scene(), scene(bolt_slid=0.03), scene(bolt_slid=0.03, tilt_deg=15.0), scene(bolt_slid=0.03)]
    assert run(frames) == 0.0


def test_bolt_partly_slid_is_not_secured():
    assert run([scene(), scene(bolt_slid=0.015)]) == 0.0


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print(f"{len(tests)} passed")
