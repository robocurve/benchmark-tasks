"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import math
import os
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


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = FAR, np.zeros((0, 3)), {}
    yield


def qx(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def flask(centre, quat=UP, vel=(0.0, 0.0, 0.0)):
    """The flask with its bulb's centre at ``centre``; pos is the axis midpoint (the AABB centre when upright)."""
    c = np.asarray(centre, float)
    R = M.hhkit.rot(quat)
    bottom = c - R @ np.array([0.0, 0.0, M.F_R])
    mid = bottom + R @ np.array([0.0, 0.0, M.F_HEIGHT / 2])
    low = np.array([c[0] - M.F_R, c[1] - M.F_R, c[2] - M.F_R])
    high = mid + (mid - low)
    return ObjectState(pos=mid, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=low, high=high)


START = (*M.M_XY, M.F_START_C)
CORK = (*M.C_XY, M.C_SEAT + M.F_DROP)


def run(states, n=1):
    out = None
    for s in states:
        for _ in range(n):
            M.TRIAL.step += 1
            out = (M.success({"flask": s}), M.score({"flask": s}))
    return out


def test_objects_and_names():
    names = [o.name for o in M.OBJECTS]
    assert len(names) == len(set(names)) and "flask" in names
    assert abs(M.F_HEIGHT - 0.112) < 1e-9 and M.FLASK.part_bands()["bulb"] == (0.002, 0.062)


def test_bulb_centre_upright_and_tilted():
    for q in (UP, qx(20)):
        assert np.allclose(M.bulb_centre(flask(START, q)), START, atol=1e-6)


def test_start_is_in_the_mantle_not_done():
    s = flask(START)
    assert M.in_mantle(s) and not M.in_cork(s)
    assert run([s], 30) == (False, 0.0)
    assert not M.TRIAL.mem["violated"]


def test_in_the_cork_ring_succeeds_after_a_second():
    ok, sc = run([flask(CORK)], M.TRIAL.frames(M.SUCCESS_HOLD_S) + 1)
    assert ok and sc == 1.0


def test_off_centre_or_leaning_in_the_ring_does_not_count():
    assert not M.in_cork(flask((M.C_XY[0] + 0.02, M.C_XY[1], CORK[2])))
    assert not M.in_cork(flask(CORK, qx(20)))


def test_bare_bench_is_a_violation():
    run([flask((*M.BENCH_XY, M.TOP + M.F_R))], 3)
    assert M.TRIAL.mem["violated"] and "bare bench" in M.TRIAL.mem["why"]
    assert run([flask(CORK)], 40)[1] == 0.0


def test_tipping_is_a_violation():
    run([flask((0.25, 0.20, M.TOP + 0.10), qx(35))])
    assert M.TRIAL.mem["violated"] and "tipped" in M.TRIAL.mem["why"]


def test_dropping_is_a_violation():
    run([flask((*M.C_XY, M.TOP + 0.07), vel=(0.0, 0.0, -0.8))])
    assert M.TRIAL.mem["violated"] and "dropped" in M.TRIAL.mem["why"]


def test_slow_lowering_and_carrying_are_fine():
    run([flask((0.27, 0.20, M.LIFT_C), vel=(0.0, 0.0, -0.1)), flask((0.27, 0.20, M.LIFT_C), qx(10))], 5)
    assert not M.TRIAL.mem["violated"]


def test_fell_off_the_bench():
    run([flask((0.30, -0.40, M.TOP - 0.10), qx(10))])
    assert M.TRIAL.mem["violated"]


def test_back_in_the_mantle_scores_zero():
    assert run([flask(START)], 40) == (False, 0.0)


def test_plans():
    for v in ("demo", "naive", "unsafe"):
        os.environ["TASK_DEMO"] = v
        try:
            plan = M.demo()["left"]
        finally:
            os.environ.pop("TASK_DEMO", None)
        assert len(plan) >= 10 and all(w.pos[2] <= M.TOP + 0.20 for w in plan)
        let_go = [w for w in plan if w.label == "let go"][0]
        if v == "unsafe":
            assert let_go.pos[2] - (CORK[2] + M.Z_GRIP) > 0.05
        elif v == "demo":
            assert abs(let_go.pos[0] - M.C_XY[0]) < 0.01 and let_go.pos[2] - (CORK[2] + M.Z_GRIP) < 0.005
