"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(str(HERE / "task.py"))
M = TASK.module


@pytest.fixture(autouse=True)
def fresh():
    if hasattr(M.TRIAL, "clear"):
        M.TRIAL.clear()
    M.TRIAL.mem = {}
    yield


def test_geometry_iron_cover_fits_and_small_lid_drops():
    assert M.LID_R < M.WELL_R            # the iron cover goes into the recess
    assert M.LID_R > M.HOLE_R            # ... and rests on the ledge, it cannot fall through
    assert M.DEC_R < M.HOLE_R            # the small lid is smaller than the opening: it drops through
    assert abs(M.SEAT_Z + M.LID_H - M.PAD_H) < 1e-9   # seated top is flush with the slab


def test_idle_scores_zero():
    M.TRIAL.mem = {"seated": False, "clear": False, "rest": 0, "dropped": False}
    assert M.score({}) == 0.0 and M.success({}) is False


def test_cover_on_the_slab_not_in_the_ring_is_zero():
    # Not seated (resting on the slab or rim): the hazard is still there, no partial credit.
    M.TRIAL.mem = {"seated": False, "near": True, "clear": True, "rest": 30, "dropped": False}
    assert M.score({}) == 0.0


def test_seated_but_held_is_half():
    M.TRIAL.mem = {"seated": True, "clear": False, "rest": 0, "dropped": False}
    assert M.score({}) == 0.5 and M.success({}) is False


def test_seated_clear_needs_one_second_at_rest():
    M.TRIAL.mem = {"seated": True, "clear": True, "rest": 19, "dropped": False}
    assert M.score({}) == 0.5 and M.success({}) is False


def test_seated_clear_and_still_is_full():
    M.TRIAL.mem = {"seated": True, "clear": True, "rest": 20, "dropped": False}
    assert M.score({}) == 1.0 and M.success({}) is True


def test_small_lid_dropped_down_the_opening_latches_zero():
    M.TRIAL.mem = {"seated": True, "clear": True, "rest": 40, "dropped": True}
    assert M.score({}) == 0.0 and M.success({}) is False
