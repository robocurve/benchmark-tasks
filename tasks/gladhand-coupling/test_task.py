"""Checks on the task's geometry and on the measurements score() is built from. No simulator needed, but they
import the task file, so run them in the yam_arena environment: python -m pytest test_task.py"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import gladkit  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402
import scoring  # noqa: E402
import task as T  # noqa: E402
from gladkit import mated_pose, measure  # noqa: E402

TABLE_X = (-0.19, 0.91)  # the station table's extent
TABLE_Y = (-0.65, 0.65)


def coupling(colour, target, angle, gap):
    pos, rot = mated_pose(T.trailer_pos(target), T.TRAILER_ROT, angle, gap)
    m = measure(T.trailer_pos(target), T.TRAILER_ROT, pos, rot)
    return scoring.Coupling(gap_excess_mm=m["gap_mm"] - gladkit.GAP_LOCKED,
                            angle_from_stop_deg=gladkit.LOCK_ANGLE_DEG - m["angle_deg"],
                            holds_pull=m["lug_under_lip"])


@pytest.mark.parametrize("angle", [-100.0, -60.0, -20.0, 0.5])
@pytest.mark.parametrize("gap", [0.6, 2.0, 10.0])
def test_measure_recovers_a_mated_pose(angle, gap):
    for colour in T.COLOURS:
        pos, rot = mated_pose(T.trailer_pos(colour), T.TRAILER_ROT, angle, gap)
        m = measure(T.trailer_pos(colour), T.TRAILER_ROT, pos, rot)
        assert m["angle_deg"] == pytest.approx(angle, abs=1e-6)
        assert m["gap_mm"] == pytest.approx(gap, abs=1e-6)
        assert m["lateral_mm"] == pytest.approx(0.0, abs=1e-6)
        assert m["tilt_deg"] == pytest.approx(0.0, abs=1e-4)


def test_fully_locked_is_locked():
    assert scoring.is_locked(coupling("red", "red", gladkit.LOCK_ANGLE_DEG, gladkit.GAP_LOCKED))


def test_fifteen_degrees_short_is_not_locked():
    assert not scoring.is_locked(coupling("red", "red", gladkit.LOCK_ANGLE_DEG - 15.0, gladkit.GAP_LOCKED))


def test_three_mm_further_apart_is_not_locked():
    assert not scoring.is_locked(coupling("red", "red", gladkit.LOCK_ANGLE_DEG, gladkit.GAP_LOCKED + 3.0))


def test_before_the_plates_mouth_the_lug_is_not_under_the_lip():
    assert not coupling("red", "red", -100.0, gladkit.GAP_OPEN).holds_pull


def test_held_above_the_lip_is_not_locked():
    # Turned to the stop with the seals 2 cm apart: the lug has passed over the lip, not under it.
    c = coupling("red", "red", gladkit.LOCK_ANGLE_DEG, 20.0)
    assert not c.holds_pull and not scoring.is_locked(c)


def test_resting_beside_the_coupler_is_not_locked():
    pos, rot = mated_pose(T.trailer_pos("red"), T.TRAILER_ROT, gladkit.LOCK_ANGLE_DEG, gladkit.GAP_LOCKED)
    m = measure(T.trailer_pos("red"), T.TRAILER_ROT, pos + np.array([0.0, 0.006, 0.0]), rot)
    assert not m["lug_under_lip"]


def test_there_is_one_red_line_and_two_trailer_gladhands():
    names = [o.name for o in T.OBJECTS]
    assert [n for n in names if n.startswith("line_")] == ["line_red"]
    assert sorted(n for n in names if n.startswith("trailer_")) == ["trailer_blue", "trailer_red"]
    assert [n for n in names if n.startswith("dummy_")] == ["dummy_red"]
    assert scoring.RIGHT == T.LINE == "red"


def test_the_start_scores_zero_and_measures_locked_on_the_dummy():
    for o in T.OBJECTS:
        if isinstance(o, gladkit.AirLine):
            _, _, pos, rot = o.start_layout()
            m = measure(T.dummy_pos(), T.dummy_rot(), pos, rot)
            assert m["lug_under_lip"] and m["gap_mm"] == pytest.approx(gladkit.GAP_LOCKED, abs=1e-6)
            assert np.linalg.norm(pos - T.dummy_pos()) * 1000 < scoring.OFF_DUMMY_MM


def test_the_hose_starts_on_the_table_and_reaches_its_gladhand():
    for o in T.OBJECTS:
        if isinstance(o, gladkit.AirLine):
            centres, headings, pos, rot = o.start_layout()
            assert o.length == pytest.approx(1.0) and o.segments == 20 and o.seg_len == pytest.approx(0.05)
            assert TABLE_X[0] + 0.03 < centres[:, 0].min() and centres[:, 0].max() < TABLE_X[1] - 0.03
            assert TABLE_Y[0] + 0.07 <= centres[:, 1].min() + 1e-3 and centres[:, 1].max() - 1e-3 <= TABLE_Y[1] - 0.07
            end = centres[-1] + o.seg_len / 2 * np.array([math.cos(headings[-1]), math.sin(headings[-1]), 0.0])
            port = pos + rot @ (np.array(gladkit.PORT) * gladkit.MM)
            assert np.linalg.norm(end - port) < 1e-6
            assert max(abs(b) for b in o.bends) < o.max_bend
            # The loop must not start out through the nose plate.
            behind = (centres[:, 0] > T.PLATE_FRONT_X - 0.02) & (np.abs(centres[:, 1]) < T.PLATE_SIZE[1] / 2 + 0.02)
            assert not behind.any()


def test_layout_matches_the_setup_text():
    assert T.TOP == pytest.approx(0.7475)
    assert T.PLATE_FRONT_X == 0.45 and T.PLATE_SIZE == (0.005, 0.40, 0.32)
    assert T.trailer_pos("red")[1:] == pytest.approx([0.10, T.TOP + 0.25])
    assert T.trailer_pos("blue")[1:] == pytest.approx([-0.10, T.TOP + 0.25])
    assert T.dummy_pos() == pytest.approx([0.22, 0.30, T.TOP + 0.12])
    assert (T.POST_X, T.POST_Y, T.POST_SIZE, T.FITTING_H) == (0.10, 0.42, (0.06, 0.06, 0.20), 0.15)
    # The trailer gladhands face the robot with their bodies up; the fitting reaches back to the plate.
    assert T.TRAILER_ROT @ np.array([0.0, 0.0, 1.0]) == pytest.approx([-1.0, 0.0, 0.0])
    assert T.TRAILER_ROT @ np.array([-1.0, 0.0, 0.0]) == pytest.approx([0.0, 0.0, 1.0])
    assert abs(np.linalg.det(T.TRAILER_ROT) - 1.0) < 1e-9
    back = T.trailer_pos("red") + T.TRAILER_ROT @ np.array([0.0, 0.0, (gladkit.HANDLE_Z[0] - 40.0) * gladkit.MM])
    assert back[0] == pytest.approx(T.PLATE_FRONT_X)


def test_gladhand_dimensions_match_the_setup_text():
    assert gladkit.HANDLE_X[0] == -125.0 and gladkit.HEAD_R == 25.0  # 150 mm long, 50 mm across the face
    assert gladkit.HANDLE_Z[1] - gladkit.HANDLE_Z[0] == 30.0  # 30 mm thick at the grip
    assert gladkit.SEAL_R * 2 == 45.0 and gladkit.SEAL_TRAVEL == 3.0
    assert gladkit.BODY_MASS + gladkit.SEAL_MASS == pytest.approx(0.45)
    assert gladkit.GAP_OPEN - gladkit.GAP_LOCKED == pytest.approx(2.9)  # the two seals' compression when locked
    assert (gladkit.STEEL, gladkit.RUBBER) == ((0.4, 0.3), (1.0, 0.8))


def test_the_lip_lets_a_pressed_lug_in_and_keeps_a_loose_one_out():
    lug_back_open = gladkit.GAP_OPEN - gladkit.LUG_Z[0]  # seals just touching
    assert lug_back_open < gladkit.lip_height(gladkit.PLATE_DEG[0])  # it enters the mouth
    assert gladkit.lip_height(-12.0) < gladkit.lip_height(0.0)  # a detent before the seat
    assert gladkit.LIP_TOP > lug_back_open  # it cannot be pushed in over the lip with the seals touching
