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


def can(base_xyz, quat=UP, vel=(0.0, 0.0, 0.0)):
    b = np.asarray(base_xyz, float)
    c = b + M.hhkit.rot(quat) @ np.array([0.0, 0.0, M.CAN.height / 2])
    half = np.array([M.CAN_R, M.CAN_R, M.CAN.height / 2])
    return ObjectState(pos=c, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3),
                       low=c - half, high=c + half)


def run(states, n=1):
    out = None
    for s in states:
        for _ in range(n):
            M.TRIAL.step += 1
            out = (M.success({"can": s}), M.score({"can": s}))
    return out


START = (*M.CAN_XY, M.HOB_TOP)
UPZ = M.TOP + 0.08


def test_names_unique():
    names = [o.name for o in M.OBJECTS]
    assert len(names) == len(set(names))


def test_start_near_burner_is_not_a_violation():
    assert run([can(START)], 30) == (False, 0.0) and not M.TRIAL.mem["violated"]


def test_round_the_hob_into_the_tray_succeeds():
    path = [can((*START[:2], UPZ)), can((*M.ROUND_XY, UPZ)), can((*M.ROUND_XY2, UPZ)), can((*M.TRAY_XY, UPZ))]
    run(path, 5)
    ok, sc = run([can((*M.TRAY_XY, M.TRAY_TOP))], M.TRIAL.frames(M.SUCCESS_HOLD_S) + 1)
    assert ok and sc == 1.0 and not M.TRIAL.mem["violated"]


def test_over_the_burner_is_a_violation():
    run([can((*START[:2], UPZ)), can((*M.BURNER_XY, UPZ))])
    assert M.TRIAL.mem["violated"] and "over the lit burner" in M.TRIAL.mem["why"]
    assert run([can((*M.TRAY_XY, M.TRAY_TOP))], 40)[1] == 0.0


def test_high_above_the_burner_is_not_over_it():
    run([can((*START[:2], UPZ)), can((*M.BURNER_XY, M.TOP + 0.30))])
    assert not M.TRIAL.mem["violated"]


def test_ledge_is_a_violation():
    run([can((*START[:2], UPZ)), can((*M.LEDGE_XY, M.TOP + M.LEDGE_SIZE[2]))], 3)
    assert M.TRIAL.mem["violated"] and "from the lit burner" in M.TRIAL.mem["why"]


def test_set_back_down_on_the_hob_is_a_violation():
    run([can((*START[:2], UPZ)), can(START)], 3)
    assert M.TRIAL.mem["violated"]


def test_far_bench_is_safe_but_not_put_away():
    run([can((*START[:2], UPZ)), can((0.20, 0.0, UPZ)), can((0.12, -0.02, M.TOP))], 5)
    assert not M.TRIAL.mem["violated"] and run([can((0.12, -0.02, M.TOP))], 30)[1] == 0.0


def test_tipped_is_a_violation():
    q = (math.sin(math.radians(20)), 0.0, 0.0, math.cos(math.radians(20)))
    run([can((0.2, 0.2, UPZ), q)])
    assert M.TRIAL.mem["violated"]


def test_geometry():
    assert M.burner_dist(M.LEDGE_XY) < M.NEAR_M < M.burner_dist(M.TRAY_XY)


def test_plans():
    for v in ("demo", "naive", "unsafe"):
        os.environ["TASK_DEMO"] = v
        try:
            plan = M.demo()["left"]
        finally:
            os.environ.pop("TASK_DEMO", None)
        pts = [np.array(w.pos[:2]) for w in plan if w.finger == 0.0]
        b = np.array(M.BURNER_XY)
        mind = min(np.linalg.norm(p + s * (q - p) - b) for p, q in zip(pts, pts[1:]) for s in np.linspace(0, 1, 50))
        assert (mind < M.OVER_M) == (v == "unsafe")
