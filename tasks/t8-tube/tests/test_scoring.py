"""Unit tests of the t8-tube scorer (pure Python): python -m pytest tests/ (or python tests/test_scoring.py)."""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))

from scoring import CLEAR_S, Scorer  # noqa: E402

DT = 0.05
NO = dict(lamp=False, carton=False, fixture=False)
HELD = dict(lamp=True, carton=False, fixture=False)


def run(seq):
    s = Scorer()
    lv = 0.0
    for holder, in_carton, speed, contact, dropped, broken in seq:
        lv = s.step(DT, holder, in_carton, speed, contact, dropped=dropped, broken=broken)
    return lv, s


def steps(n, *row):
    return [row] * n


def test_noop_scores_zero():
    lv, _ = run(steps(400, "locked", False, 0.0, NO, False, False))
    assert lv == 0.0


def test_turned_but_not_down_scores_zero():
    lv, _ = run(steps(100, "slot", False, 0.0, HELD, False, False))
    assert lv == 0.0


def test_full_success():
    seq = steps(50, "locked", False, 0.0, HELD, False, False) + steps(50, "out", False, 0.1, HELD, False, False)
    seq += steps(20, "out", True, 0.05, HELD, False, False)
    seq += steps(int(CLEAR_S / DT) + 1, "out", True, 0.0, NO, False, False)
    lv, _ = run(seq)
    assert lv == 1.0


def test_success_needs_hands_off_and_rest():
    seq = steps(50, "out", True, 0.0, HELD, False, False)
    assert run(seq)[0] == 0.0
    seq = steps(int(CLEAR_S / DT) + 5, "out", True, 0.1, NO, False, False)
    assert run(seq)[0] == 0.0


def test_unsupported_or_held_tube_is_zero():
    seq = steps(60, "out", False, 0.0, HELD, False, False)
    assert run(seq)[0] == 0.0
    seq = steps(int(CLEAR_S / DT) + 5, "out", False, 0.0, NO, False, False)
    assert run(seq)[0] == 0.0


def test_drop_vetoes_even_in_carton():
    seq = steps(20, "out", False, 0.0, HELD, False, False) + steps(1, "out", False, 1.2, NO, True, False)
    seq += steps(int(CLEAR_S / DT) + 5, "out", True, 0.0, NO, False, False)
    lv, s = run(seq)
    assert lv == 0.0 and s.veto == "lamp_dropped"


def test_broken_lampholder_vetoes():
    seq = steps(20, "locked", False, 0.0, HELD, False, False) + steps(1, "broken", False, 0.0, HELD, False, True)
    seq += steps(int(CLEAR_S / DT) + 5, "out", True, 0.0, NO, False, False)
    lv, s = run(seq)
    assert lv == 0.0 and s.veto == "lampholder_broken"


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)

def test_four_seconds_start_again_on_surface_change_motion_or_touch():
    s = Scorer()
    for _ in range(100):
        level = s.step(DT, "out", False, 0., NO, on_bench=True)
    assert level == 0.5
    for _ in range(79):
        assert s.step(DT, "out", True, 0., NO) == 0
    assert s.step(DT, "out", True, 0.02, NO) == 0
    for _ in range(79):
        assert s.step(DT, "out", True, 0., NO) == 0
    assert s.step(DT, "out", True, 0., HELD) == 0
    for _ in range(79):
        assert s.step(DT, "out", True, 0., NO) == 0
    assert s.step(DT, "out", True, 0., NO) == 1
    assert s.step(DT, "out", True, 0., NO, dropped=True) == 0
    for _ in range(100):
        assert s.step(DT, "out", False, 0., NO, on_bench=True) == 0


def test_carton_bench_and_unsafe_placements():
    import layout as L
    import numpy as np
    carton = (np.array([0.2, 0., L.TOP]), np.eye(3))
    for x, z, expected in ((0.2, L.BOX_WALL+L.CAP_R, (True, False)),
                           (0.35, L.CAP_R, (False, True)),
                           (0.35, -0.3, (False, False)),
                           (0.255, L.CAP_R, (False, False)),
                           (0.35, 0.2, (False, False))):
        assert L.resting_surfaces(np.array([x, 0., L.TOP + z]), np.eye(3), carton) == expected
