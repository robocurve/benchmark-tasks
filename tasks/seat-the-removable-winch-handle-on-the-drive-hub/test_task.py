"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import json
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
    M.TRIAL.clear()
    yield


def test_instruction_matches_task_json():
    d = json.loads((HERE / "task.json").read_text())
    assert d["instruction"] == M.INSTRUCTION


def test_task_json_limits():
    d = json.loads((HERE / "task.json").read_text())
    assert len(d["title"]) <= 120
    assert len(d["fidelity"]) <= 500
    assert len(d["scores"]) <= 300
    assert len(d.get("context", "")) <= 1000


def test_start_on_the_outline_scores_zero():
    M.TRIAL.mem.update(seated=False, partial=False, clear=False, rest=0)
    assert M.score({}) == 0.0
    assert M.success({}) is False


def test_partial_engagement_is_half():
    M.TRIAL.mem.update(seated=False, partial=True, clear=False, rest=0)
    assert M.score({}) == 0.5


def test_seated_but_held_is_half():
    M.TRIAL.mem.update(seated=True, partial=False, clear=False, rest=0)
    assert M.score({}) == 0.5


def test_seated_cleared_and_still_is_full():
    M.TRIAL.mem.update(seated=True, partial=False, clear=True, rest=20)
    assert M.score({}) == 1.0
    assert M.success({}) is True


def test_flagged_hub_and_axial_window():
    # 40 mm hub around a 14 mm-class peg, 12 mm axial window (PhysX contactOffset)
    assert abs(M.TUBE_OUT - 0.040) < 1e-9
    assert abs(M.AXIAL_TOL - 0.012) < 1e-9
