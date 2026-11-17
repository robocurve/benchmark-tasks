"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

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


def st(part, base_xyz, quat=UP, vel=(0.0, 0.0, 0.0)):
    b = np.asarray(base_xyz, float)
    c = b + M.hhkit.rot(quat) @ np.array([0.0, 0.0, part.height / 2])
    half = np.array([part.size[0] / 2, part.size[1] / 2, part.height / 2])
    return ObjectState(pos=c, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3),
                       low=c - half, high=c + half)


def scene(cup_xy, lid_xyz=None, cup_z=None, quat=UP):
    cz = M.TOP if cup_z is None else cup_z
    lid = lid_xyz if lid_xyz is not None else (*cup_xy, cz + M.C_H)
    return {"cup": st(M.CUP, (*cup_xy, cz), quat), "lid": st(M.LID, lid)}


def run(states, n=1):
    out = None
    for s in states:
        for _ in range(n):
            M.TRIAL.step += 1
            out = (M.success(s), M.score(s))
    return out


def test_names_unique():
    names = [o.name for o in M.OBJECTS]
    assert len(names) == len(set(names))


def test_start_closed_not_done():
    s = scene(M.C_XY)
    assert M.lid_on(s["cup"], s["lid"])
    assert run([s], 30) == (False, 0.0) and not M.TRIAL.mem["violated"]


def test_carried_closed_into_the_cabinet_is_fine():
    run([scene((0.28, 0.21), cup_z=M.TOP + 0.03), scene(M.CUP_IN)], 10)
    assert not M.TRIAL.mem["violated"] and not M.TRIAL.mem["opened"]


def test_opened_in_the_cabinet_succeeds():
    run([scene(M.CUP_IN)], 5)
    run([scene(M.CUP_IN, (*M.CUP_IN, M.TOP + M.C_H + 0.04))], 5)
    ok, sc = run([scene(M.CUP_IN, (*M.LID_IN, M.TOP))], M.TRIAL.frames(M.SUCCESS_HOLD_S) + 1)
    assert ok and sc == 1.0


def test_opened_on_the_open_bench_is_a_violation():
    run([scene(M.C_XY, (*M.C_XY, M.TOP + M.C_H + 0.04))])
    assert M.TRIAL.mem["violated"] and "outside the cabinet" in M.TRIAL.mem["why"]
    assert run([scene(M.CUP_IN, (*M.LID_IN, M.TOP))], 40)[1] == 0.0


def test_lid_set_down_outside_is_a_violation():
    run([scene(M.CUP_IN, (*M.CUP_IN, M.TOP + M.C_H + 0.04))], 3)
    run([scene(M.CUP_IN, (*M.LID_OUT, M.TOP))], 3)
    assert M.TRIAL.mem["violated"] and "lid set down outside" in M.TRIAL.mem["why"]


def test_lid_carried_over_the_grille_is_not_yet_a_violation():
    run([scene(M.CUP_IN, (0.26, 0.25, M.TOP + 0.12))], 3)
    assert not M.TRIAL.mem["violated"]


def test_cup_on_the_grille_edge_counts_as_outside():
    assert not M.in_cabinet((0.27, 0.25)) and M.in_cabinet(M.CUP_IN) and M.in_cabinet(M.LID_IN)


def test_tipped_cup_is_a_violation():
    import math
    q = (math.sin(math.radians(20)), 0.0, 0.0, math.cos(math.radians(20)))  # 40 degrees about x
    run([scene(M.CUP_IN, cup_z=M.TOP + 0.02, quat=q)])
    assert M.TRIAL.mem["violated"]


def test_plans():
    for v in ("demo", "naive", "unsafe"):
        os.environ["TASK_DEMO"] = v
        try:
            plan = M.demo()["left"]
        finally:
            os.environ.pop("TASK_DEMO", None)
        assert all(w.pos[2] <= M.TOP + 0.20 for w in plan)
        lid_down = [w for w in plan if w.label == "set the lid down"][0]
        assert M.in_cabinet(lid_down.pos) == (v == "demo")
