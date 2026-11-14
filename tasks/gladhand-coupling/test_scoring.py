"""Synthetic end states for every case in the task's Scoring text. Run: python -m pytest test_scoring.py"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import pytest  # noqa: E402
from scoring import Coupling, EndState, Line, is_locked, score  # noqa: E402

LOCKED = Coupling(gap_excess_mm=0.0, angle_from_stop_deg=0.0, holds_pull=True)
TRAILER_FAR = 400.0  # mm from the dummy once on the trailer


def on_dummy() -> Line:
    return Line(dummy_distance_mm=0.0)


def on_table() -> Line:
    return Line(dummy_distance_mm=150.0)


def on_trailer(colour: str, coupling: Coupling = LOCKED) -> Line:
    return Line(dummy_distance_mm=TRAILER_FAR, couplings={colour: coupling})


def end(line: Line, **kw) -> EndState:
    return EndState(line=line, **kw)


def test_locked_and_released():
    assert score(end(on_trailer("red"), clear_s=1.0)) == 1.0


def test_locked_gripper_still_touching():
    assert score(end(on_trailer("red"), clear_s=0.0)) == 0.8


def test_released_for_less_than_a_second():
    assert score(end(on_trailer("red"), clear_s=0.95)) == 0.8


def test_off_dummy_on_the_table():
    assert score(end(on_table(), clear_s=5.0)) == 0.2


def test_nothing_done():
    assert score(end(on_dummy(), clear_s=5.0)) == 0.0


def test_turned_off_but_left_on_the_dummy_is_not_off():
    assert score(end(Line(dummy_distance_mm=50.0))) == 0.0


def test_fifteen_degrees_short_is_not_locked():
    short = Coupling(gap_excess_mm=0.0, angle_from_stop_deg=15.0, holds_pull=True)
    assert not is_locked(short)
    # It counts only as off its dummy; no release bonus.
    assert score(end(on_trailer("red", short), clear_s=5.0)) == 0.2


def test_ten_degrees_short_is_locked():
    assert is_locked(Coupling(gap_excess_mm=0.0, angle_from_stop_deg=10.0, holds_pull=True))


def test_seals_three_mm_apart_is_not_locked():
    apart = Coupling(gap_excess_mm=3.0, angle_from_stop_deg=0.0, holds_pull=True)
    assert not is_locked(apart)
    assert score(end(on_trailer("red", apart), clear_s=5.0)) == 0.2


def test_two_mm_gap_is_locked():
    assert is_locked(Coupling(gap_excess_mm=2.0, angle_from_stop_deg=0.0, holds_pull=True))


def test_resting_against_the_coupler_without_the_lug_engaged():
    resting = Coupling(gap_excess_mm=0.0, angle_from_stop_deg=0.0, holds_pull=False)
    assert not is_locked(resting)
    assert score(end(on_trailer("red", resting), clear_s=5.0)) == 0.2


def test_locked_to_the_blue_trailer_gladhand():
    assert score(end(on_trailer("blue"), clear_s=5.0)) == 0.0


def test_turned_short_on_the_blue_trailer_gladhand_is_only_off_its_dummy():
    short = Coupling(gap_excess_mm=0.0, angle_from_stop_deg=40.0, holds_pull=True)
    assert score(end(on_trailer("blue", short), clear_s=5.0)) == 0.2


def test_gladhand_touched_the_floor():
    assert score(end(on_trailer("red"), clear_s=5.0, floor_touched=True)) == 0.0


def test_hose_pulled_off_its_post():
    assert score(end(on_trailer("red"), clear_s=5.0, hose_detached=True)) == 0.0


@pytest.mark.parametrize("line", [on_dummy(), on_table(), on_trailer("red"), on_trailer("blue")])
@pytest.mark.parametrize("clear_s", [0.0, 2.0])
def test_only_the_listed_scores_are_possible(line, clear_s):
    assert score(end(line, clear_s=clear_s)) in (0.0, 0.2, 0.8, 1.0)
