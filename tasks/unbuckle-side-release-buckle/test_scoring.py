"""Offline checks of task.py (no GPU, no Isaac Sim): every scoring outcome from fabricated object states, and the
buckle's geometry (clearances, the catch, and that only both tabs pressed together let it apart).

    python -m pytest test_scoring.py
"""

from __future__ import annotations

import importlib
import itertools
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from bucklekit import PartState  # noqa: E402

T = importlib.import_module("task")

ORIGIN = np.array([*T.BUCKLE_XY, T.TOP])
Q0 = np.array([0.0, 0.0, 0.0, 1.0])


def _quat_z(deg):
    a = math.radians(deg) / 2
    return np.array([0.0, 0.0, math.sin(a), math.cos(a)])


def _state(pos, quat=Q0, sliders=None):
    p = np.asarray(pos, float)
    return PartState(pos=p, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p,
                     high=p, sliders=sliders or {})


def _objects(pull=0.0, press=(0.0, 0.0), female_dxy=(0.0, 0.0), male_yaw=0.0, male_dz=0.0, female_dz=0.0):
    """``pull``: how far (m) the male half has been drawn out along the buckle (-y). ``press``: each tab's press (m)."""
    f = ORIGIN + np.array([*female_dxy, female_dz])
    m = f + np.array([0.0, -pull, male_dz])
    sl = {"prong_a": -press[0], "prong_b": press[1]}
    return {"female": _state(f), "male": _state(m, _quat_z(male_yaw), sl)}


def _run(frames):
    T._reset_state()
    out = (0.0, False)
    for objs in frames:
        ok = T.success(objs)          # the runtime's order: success() then score() on the same states
        out = (T.score(objs), ok)
    return out


BOTH = (0.003, 0.003)


def test_start_is_buckled_and_scores_zero():
    objs = _objects()
    assert abs(T.gap(objs["male"], objs["female"]) - 0.002) < 1e-9
    assert _run([objs]) == (0.0, False)
    assert _run([_objects()] * 40) == (0.0, False)


def test_both_tabs_pressed():
    assert _run([_objects(press=BOTH)])[0] == 0.25
    assert _run([_objects(press=BOTH), _objects()])[0] == 0.25                # sticky
    assert _run([_objects(press=(0.0035, 0.0014))])[0] == 0.0                 # one not far enough
    assert _run([_objects(press=(0.0035, 0.0))])[0] == 0.0                    # one tab only
    assert _run([_objects(press=(0.0035, 0.0)), _objects(press=(0.0, 0.0035))])[0] == 0.0   # one after the other


def test_released():
    assert _run([_objects(press=BOTH), _objects(pull=0.0061, press=BOTH)])[0] == 0.5
    assert _run([_objects(pull=0.0061), _objects()])[0] == 0.5                # sticky
    assert _run([_objects(pull=0.0059)])[0] == 0.0                            # still within the play


def test_apart_succeeds_only_after_one_second():
    apart = _objects(pull=0.051)
    assert _run([apart] * 19) == (0.75, False)
    assert _run([apart] * 20) == (1.0, True)
    assert _run([apart] * 19 + [_objects(pull=0.030)] + [apart] * 19) == (0.75, False)   # broken hold restarts
    assert _run([apart] * 19 + [_objects(pull=0.030)] + [apart] * 20) == (1.0, True)
    assert _run([apart] * 25 + [_objects(pull=0.030)])[0] == 0.75
    assert _run([_objects(pull=0.0495)] * 30)[0] == 0.5                      # tongue not yet clear


def test_apart_counts_in_any_direction_and_anywhere():
    objs = _objects(pull=0.0)
    objs["male"] = _state(ORIGIN + np.array([-0.08, -0.06, 0.0]), _quat_z(70))
    assert _run([objs] * 20) == (1.0, True)
    objs = _objects(pull=0.06, female_dxy=(0.05, 0.10), female_dz=0.10, male_dz=0.12)   # held up in the air, moved
    assert _run([objs] * 20) == (1.0, True)


def test_fallen_off_the_table_is_zero_for_good():
    assert _run([_objects(male_dz=-0.051)] + [_objects(pull=0.06)] * 25) == (0.0, False)
    assert _run([_objects(female_dz=-0.3)] + [_objects(pull=0.06)] * 25) == (0.0, False)
    assert _run([_objects(male_dz=-0.04)] + [_objects(pull=0.06)] * 25) == (1.0, True)   # e.g. hanging off the edge


# -- geometry ---------------------------------------------------------------------------------------------------------

def _boxes(p, press=0.0):
    """(name, lo, hi) in mm in the buckle frame; the prongs pressed in ``press`` mm."""
    out = []
    for b in p.boxes:
        c, s = np.array(b[0]) * 1000, np.array(b[1]) * 1000
        out.append((p.name, c - s / 2, c + s / 2))
    for sl in p.sliders:
        shift = np.array([sl.sign * press, 0, 0])
        for b in sl.boxes:
            c, s = np.array(b[0]) * 1000 + shift, np.array(b[1]) * 1000
            out.append((f"{p.name}:{sl.name}", c - s / 2, c + s / 2))
    return out


def _gap(a, b):
    """Largest separation along any axis (mm); <= 0 means the boxes overlap."""
    return float(max(np.max(b[1] - a[2]), np.max(a[1] - b[2])))


def _female_male(press=0.0, dy=0.0, dx=0.0):
    f = _boxes(T.FEMALE)
    m = [(n, lo + np.array([dx, dy, 0]), hi + np.array([dx, dy, 0])) for n, lo, hi in _boxes(T.MALE, press)]
    return f, m


def _min_gap(press=0.0, dy=0.0, dx=0.0):
    f, m = _female_male(press, dy, dx)
    return min(_gap(a, b) for a in f for b in m)


def test_buckled_parts_clear_each_other_by_at_least_0p75_mm():
    assert _min_gap() >= 0.75 - 1e-9
    assert _min_gap(press=3.0) >= 0.75 - 1e-9


def test_prongs_and_the_rest_of_the_male_do_not_touch_within_an_axis_overlap():
    ms = _boxes(T.MALE)
    assert all(lo[2] >= -1e-9 for _, lo, _ in ms)                               # nothing below the table
    assert all(lo[2] >= -1e-9 for _, lo, _ in _boxes(T.FEMALE))


def _can_slide_out(press_a, press_b, dx=0.0):
    """Can the male be drawn out 55 mm (-y) with the tabs pressed in this far (mm) and shifted ``dx`` sideways?"""
    f = _boxes(T.FEMALE)
    m = []
    for p_name, lo, hi in _boxes(T.MALE):
        m.append((p_name, lo.copy(), hi.copy()))
    for sl, press in zip(T.MALE.sliders, (press_a, press_b)):
        shift = sl.sign * press
        for n_i, (n, lo, hi) in enumerate(m):
            if n == f"male:{sl.name}":
                m[n_i] = (n, lo + np.array([shift, 0, 0]), hi + np.array([shift, 0, 0]))
    for dy in np.arange(0.0, -55.01, -0.25):
        for a in f:
            for b in m:
                lo, hi = b[1] + np.array([dx, dy, 0]), b[2] + np.array([dx, dy, 0])
                if _gap(a, (b[0], lo, hi)) <= 0:
                    return False
    return True


def test_catch_holds_unless_both_tabs_are_pressed_1p5_mm():
    assert not _can_slide_out(0.0, 0.0)
    assert not _can_slide_out(3.5, 0.0)
    assert not _can_slide_out(0.0, 3.5)
    assert not _can_slide_out(1.25, 1.25)
    assert _can_slide_out(1.75, 1.75)
    assert _can_slide_out(3.0, 3.0)
    # the male's sideways play inside the female (its tongue between the ribs) cannot free a tab pressed less
    for dx in (-0.75, 0.75):
        assert _min_gap(dx=dx) > -1e-9
        assert not _can_slide_out(3.5, 0.5, dx=dx) and not _can_slide_out(0.5, 3.5, dx=dx)


def test_pull_with_tabs_out_stops_1_mm_back_on_the_lips():
    assert _min_gap(dy=-0.99) > 0
    assert _min_gap(dy=-1.01) < 0 or _min_gap(dy=-1.0) <= 1e-9


def test_pinch_on_the_window_presses_the_tabs_flush_and_clears_the_lips():
    # A pinch closing across the buckle stops on the female's sides (x = +-19 mm) behind the mouth, which leaves each
    # tab pressed in 3 mm: 1.5 mm inside the lips (x = +-20.5 mm). The mouth (+-27 mm) is only at y 0..10 mm.
    assert T.TAB_OUT - T.BODY_HALF == 3.0
    assert T.TAB_OUT - T.LIP_IN == 1.5 and T.PRESSED == 0.0015
    assert T.PRONG_TRAVEL * 1000 > T.TAB_OUT - T.BODY_HALF
    f = _boxes(T.FEMALE)
    behind = [b for b in f if b[1][1] >= T.WINDOW[0] - 1e-9 and b[2][1] <= 55 + 1e-9]   # up to the webbing bar
    assert max(b[2][0] for b in behind) <= T.BODY_HALF + 1e-9 and min(b[1][0] for b in behind) >= -T.BODY_HALF - 1e-9


def test_gap_metric_thresholds_match_the_geometry():
    tongue = [b for b in _boxes(T.MALE) if b[0] == "male" and abs(b[1][0] + 7) < 1e-6][0]
    tongue_len = tongue[2][1] - (-2.0)
    assert abs(tongue_len - 47.0) < 1e-9
    assert abs(T.APART_GAP * 1000 - (tongue_len + 5.0)) < 1e-9
    assert T.RELEASED_GAP > 0.002 + 0.001 + 0.002   # more than the play (1 mm back, 2 mm in) from buckled


def test_layout_on_the_table():
    assert T.FEMALE.pos == T.MALE.pos
    assert abs(T.FEMALE.pos[2] - T.TOP) < 1e-12
    assert 0.10 <= T.BUCKLE_XY[0] <= 0.45
    names = [o.name for o in T.OBJECTS]
    assert len(set(names)) == len(names)
    for a, b in itertools.combinations(T.OBJECTS, 2):
        assert a.name != b.name
