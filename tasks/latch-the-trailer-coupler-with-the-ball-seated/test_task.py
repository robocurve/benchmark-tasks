"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import json
import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(str(HERE / "task.py"))
M = TASK.module
FAR = [{"tcp": np.array([0.20, 0.30, 0.93]), "tips": [np.array([0.20, 0.32, 0.92]), np.array([0.20, 0.28, 0.92])]},
       {"tcp": np.array([0.20, -0.30, 0.93]), "tips": [np.array([0.20, -0.28, 0.92]), np.array([0.20, -0.32, 0.92])]}]
ORIGIN = np.array([M.BX, M.BY, M.TOP])


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = FAR, np.zeros((0, 3)), {}
    yield


def coupler(th_deg, hd_deg, jaw):
    j = {"tongue_joint": math.radians(th_deg), "handle_joint": math.radians(hd_deg), "jaw_joint": float(jaw)}
    z = np.zeros(3)
    return {"coupler": M.m.PartState(pos=ORIGIN.copy(), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=z, ang_vel=z, low=ORIGIN, high=ORIGIN, joints=j)}


def run(objs, seconds=1.2):
    s = 0.0
    for _ in range(M.TRIAL.frames(seconds)):
        M.TRIAL.new_step()
        s = M.score(objs)
    return s, M.success(objs)


def test_start_scores_zero():
    assert run(coupler(M.START_DEG, M.HANDLE_OPEN, 0.0)) == (0.0, False)


def test_seated_and_latched_full():
    assert run(coupler(0.0, 1.0, -M.JAW_TRAVEL)) == (1.0, True)


def test_seated_only_half():
    assert run(coupler(0.0, M.HANDLE_OPEN, 0.0)) == (0.5, False)


def test_seated_handle_jammed_half():
    # naive: handle pushed first, jaw stopped against the ball at 15 deg: seated, not latched
    assert run(coupler(0.0, 13.0, -0.050)) == (0.5, False)


def test_lowered_not_seated_quarter():
    assert run(coupler(2.0, M.HANDLE_OPEN, 0.0)) == (0.25, False)


def test_latched_not_seated_zero():
    assert run(coupler(2.0, 1.0, -M.JAW_TRAVEL)) == (0.0, False)


def test_full_needs_hand_clear():
    pad = M.to_world(M.HP[0] - M.HANDLE_L, 0.0, M.HP[2] + 0.02)
    M.TRIAL.grippers_override = [{"tcp": pad, "tips": [pad, pad]}, FAR[1]]
    assert run(coupler(0.0, 1.0, -M.JAW_TRAVEL))[0] < 1.0


def test_full_needs_one_second():
    assert run(coupler(0.0, 1.0, -M.JAW_TRAVEL), seconds=0.5) == (0.25, False)


def test_undo_after_success_drops():
    run(coupler(0.0, 1.0, -M.JAW_TRAVEL))
    assert run(coupler(0.0, M.HANDLE_OPEN, 0.0)) == (0.5, False)


def test_pocket_geometry_seated_at_zero():
    px, pz = M.pocket_offset(0.0)
    assert abs(px) < 1e-9 and abs(pz - M.BZ) < 1e-9
    px, pz = M.pocket_offset(math.radians(M.START_DEG))
    assert 0.020 < pz - M.BZ < 0.030


def test_task_json_limits():
    d = json.loads((HERE / "task.json").read_text())
    assert len(d["title"]) <= 120 and len(d["fidelity"]) <= 500 and len(d["scores"]) <= 300 and len(d.get("context", "")) <= 1000
    assert d["instruction"] == M.INSTRUCTION


def test_hitch_turned_car_left_trailer_right():
    assert np.allclose(M.to_world(0, 0, 0), ORIGIN)
    car, trailer = M.to_world(M.CAR_X, 0), M.to_world(M.TRAILER_X, 0)
    assert car[1] > 0.55 and trailer[1] < -0.55  # bumper at the bench's left end, trailer front at its right end
    pad = M.to_world(M.HP[0] - M.HANDLE_L, 0, 0)
    assert 0.35 <= pad[0] <= 0.60 and abs(pad[1]) < 0.35  # the handle pad stays in the table middle
