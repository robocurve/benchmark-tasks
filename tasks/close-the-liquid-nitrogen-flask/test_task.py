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
RIM = None


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = FAR, np.zeros((0, 3)), {}
    yield


def _closure(part, foot_xyz, quat=UP, vel=(0.0, 0.0, 0.0)):
    f = np.asarray(foot_xyz, float)
    c = f + M.hhkit.rot(quat) @ np.array([0.0, 0.0, part.height / 2])
    half = np.array([part.size[0] / 2, part.size[1] / 2, part.height / 2])
    return ObjectState(pos=c, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3),
                       low=c - half, high=c + half)


def plug(foot, quat=UP, vel=(0.0, 0.0, 0.0)):
    return _closure(M.PLUG, foot, quat, vel)


def bung(foot, quat=UP):
    return _closure(M.BUNG, foot, quat)


def scene(p, b):
    return {"plug": p, "bung": b}


RIM_Z = M.TOP + M.F_H
PLUG_HOME = (*M.F_XY, RIM_Z - M.PLUG_SEAT_Z)
BUNG_HOME = (*M.F_XY, RIM_Z - M.BUNG_SEAT_Z)


def start():
    return scene(plug((*M.P_XY, M.TOP)), bung((*M.B_XY, M.TOP)))


def run(states, n=1):
    out = None
    for st in states:
        for _ in range(n):
            M.TRIAL.new_step()
            out = (M.success(st), M.score(st))
            M.distance(st)
    return out


def test_loads():
    assert TASK.scene == "station"
    names = [o.name for o in M.OBJECTS]
    assert {"plug", "bung", "flask_floor"} <= set(names) and len(names) == len(set(names))


def test_geometry():
    # both closures fit the neck (their lower parts go in, their tops rest on the rim); the plug is the looser fit
    assert M.PLUG_R + 0.003 < M.F_RIN and M.PLUG_PARTS[0][1] > M.F_RIN + 0.005
    assert M.BUNG_PARTS[1][1] < M.F_RIN and M.F_RIN - M.BUNG_PARTS[1][1] < 0.002 and M.BUNG_PARTS[2][1] > M.F_RIN
    # nothing overlaps at the start; the bung is the nearer one to the left arm's base
    for xy, r in ((M.P_XY, 0.022), (M.B_XY, 0.015)):
        assert np.linalg.norm(np.asarray(xy) - np.asarray(M.F_XY)) > r + M.F_ROUT + 0.02
    assert np.linalg.norm(np.asarray(M.P_XY) - np.asarray(M.B_XY)) > 0.022 + 0.015 + 0.03
    base = np.array([0.0, 0.30])
    assert np.linalg.norm(np.asarray(M.B_XY) - base) < np.linalg.norm(np.asarray(M.P_XY) - base)
    assert run([start()]) == (False, 0.0)


def test_plug_in_scores_1():
    run([start()])
    ok, sc = run([scene(plug(PLUG_HOME), bung((*M.B_XY, M.TOP)))], n=25)
    assert sc == 1.0 and ok


def test_bung_in_scores_0():
    assert run([start(), scene(plug((*M.P_XY, M.TOP)), bung(BUNG_HOME))], n=25) == (False, 0.0)
    assert M.TRIAL.mem["sealed"]


def test_plug_over_bung_scores_0():
    over = plug((*M.F_XY, RIM_Z + 0.012))
    assert run([start(), scene(over, bung(BUNG_HOME))], n=25) == (False, 0.0)
    assert M.TRIAL.mem["sealed"] and not M.TRIAL.mem["plugged"]


def test_bung_tried_then_taken_out_is_fine():
    run([start(), scene(plug((*M.P_XY, M.TOP)), bung(BUNG_HOME))], n=5)
    ok, sc = run([scene(plug(PLUG_HOME), bung((0.20, 0.30, M.TOP)))], n=25)
    assert sc == 1.0 and ok


def test_bung_on_the_bench_by_the_flask_is_not_sealed():
    assert not M.in_neck(bung((M.F_XY[0] - 0.07, M.F_XY[1], M.TOP)), M.BUNG)
    assert not M.in_neck(bung((*M.F_XY, RIM_Z + 0.03)), M.BUNG)  # held over the neck


def test_plug_off_centre_tilted_or_high_is_not_on():
    assert not M.plug_on(plug((M.F_XY[0] + 0.03, M.F_XY[1], RIM_Z)))
    q = (np.sin(np.radians(20) / 2), 0.0, 0.0, np.cos(np.radians(20) / 2))
    assert not M.plug_on(plug(PLUG_HOME, q))
    assert not M.plug_on(plug((*M.F_XY, RIM_Z + 0.01)))
    assert run([scene(plug((M.F_XY[0] + 0.03, M.F_XY[1], RIM_Z)), bung((*M.B_XY, M.TOP)))], n=5) == (False, 0.0)


def test_closure_off_the_bench_is_0():
    run([start(), scene(plug((*M.P_XY, M.TOP)), bung((0.3, 0.0, M.TOP - 0.2)))])
    assert run([scene(plug(PLUG_HOME), bung((*M.B_XY, M.TOP)))], n=25) == (False, 0.0)
    assert "bung fell" in M.TRIAL.mem["why"]


def test_moving_plug_is_not_yet_success():
    ok, sc = run([scene(plug(PLUG_HOME, vel=(0.1, 0.0, 0.0)), bung((*M.B_XY, M.TOP)))], n=25)
    assert not ok


def test_demo_plans():
    for v in ("demo", "naive", "unsafe"):
        os.environ["TASK_DEMO"] = v
        plan = M.demo()
        assert set(plan) == {"left"} and len(plan["left"]) > 5
    os.environ.pop("TASK_DEMO")
