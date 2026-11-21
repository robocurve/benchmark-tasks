"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import ObjectState, load  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(str(HERE / "task.py"))
M = TASK.module

FAR = [{"tcp": np.array([0.12, 0.30, 1.0]), "tips": [np.array([0.12, 0.33, 0.99]), np.array([0.12, 0.27, 0.99])]},
       {"tcp": np.array([0.15, -0.30, 1.0]), "tips": [np.array([0.15, -0.27, 0.99]), np.array([0.15, -0.33, 0.99])]}]
UP = (0.0, 0.0, 0.0, 1.0)
Z = np.zeros(3)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = FAR, np.zeros((0, 3)), {}
    yield


def shaker(pressed, spread=0.0):
    p = np.array([*M.SH_XY, M.TOP])
    j = {"start_joint": -M.BTN_TRAVEL if pressed else 0.0}
    for n, sgn in M.FINGER_OUT.items():
        j[f"{n}_joint"] = math.radians(sgn * spread)
    return M.m.PartState(pos=p, quat_xyzw=np.array(UP), lin_vel=Z, ang_vel=Z, low=p, high=p, joints=j)


def flask(foot_xy, quat=UP, dz=0.0):
    f = np.array([*foot_xy, M.TOP + M.PT + dz]) + M.hhkit.rot(quat) @ np.array([0.0, 0.0, M.F_TOTAL / 2])
    return ObjectState(pos=f, quat_xyzw=np.asarray(quat, float), lin_vel=Z, ang_vel=Z, low=f, high=f)


def scene(pressed, foot_xy, quat=UP, dz=0.0, spread=None):
    if spread is None:
        spread = 6.0 if tuple(foot_xy) == tuple(M.EMPTY_XY) else 0.0
    return {"shaker": shaker(pressed, spread), "flask": flask(foot_xy, quat, dz)}


def run(objs, seconds=1.2):
    s = 0.0
    for _ in range(M.TRIAL.frames(seconds)):
        M.TRIAL.new_step()
        s = M.score(objs)
    return s, M.success(objs)


def test_start_scores_zero():
    assert run(scene(False, M.LOOSE_XY)) == (0.0, False)


def test_clamped_then_started_scores_one():
    run(scene(False, M.EMPTY_XY), 0.3)
    assert run(scene(True, M.EMPTY_XY)) == (1.0, True)


def test_started_with_loose_flask_is_zero_even_if_clamped_later():
    run(scene(True, M.LOOSE_XY), 0.2)
    assert M.TRIAL.mem["violated"]
    assert run(scene(True, M.EMPTY_XY))[0] == 0.0


def test_clamped_only_scores_half():
    assert run(scene(False, M.EMPTY_XY)) == (M.CLAMP_ONLY, False)


def test_off_centre_is_not_seated():
    assert run(scene(False, (M.EMPTY_XY[0] + 0.02, M.EMPTY_XY[1])))[0] == 0.0


def test_resting_on_the_arms_is_not_seated():
    assert run(scene(False, M.EMPTY_XY, dz=M.ARM_H))[0] == 0.0


def test_still_held_does_not_count():
    x, y = M.EMPTY_XY
    zg = M.TOP + M.PT + M.Z_GRIP
    M.TRIAL.grippers_override = [{"tcp": np.array([x, y, zg]), "tips": [np.array([x, y + 0.017, zg])]}, FAR[1]]
    assert run(scene(False, M.EMPTY_XY))[0] == 0.0


def test_tipped_flask_is_a_violation():
    tip = (math.sin(math.radians(30)), 0.0, 0.0, math.cos(math.radians(30)))
    M.TRIAL.new_step()
    M.score(scene(False, M.LOOSE_XY, tip))
    assert run(scene(True, M.EMPTY_XY))[0] == 0.0


def test_layout_in_middle_and_on_platform():
    for x, y in (M.EMPTY_XY, M.LOOSE_XY, M.FULL_XY):
        assert 0.35 <= x <= 0.60 and abs(y) < 0.35
        lx, ly = x - M.SH_XY[0], y - M.SH_XY[1]
        assert M.PL_X0 + M.F_R + 0.01 <= lx <= M.SH_D / 2 - M.F_R - 0.005 and abs(ly) <= M.PL_W / 2 - M.F_R - 0.005


def test_fingers_not_sprung_is_not_seated():
    assert run(scene(False, M.EMPTY_XY, spread=1.0))[0] == 0.0


def test_clamp_is_a_push_fit():
    # the fingers' tops stand inside the flask's radius at rest, their feet just outside it
    assert M.FING_IN0 > M.F_R and M.FINGER_TOP_IN > 0.001
    assert 5.0 < M.LEAN_DEG < 20.0
    # the second (fixed) clamp has room for its flask
    assert M.CLAMP_RAD - M.ARM_R > M.F_R


def test_demo_variants_build():
    import os
    for v in ("demo", "naive", "clamp", "early"):
        os.environ["TASK_DEMO"] = v
        plan = M.demo()
        assert set(plan) <= {"left", "right"} and all(plan.values())
    os.environ.pop("TASK_DEMO")
