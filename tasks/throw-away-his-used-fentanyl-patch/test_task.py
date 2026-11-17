"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(str(HERE / "task.py"))
M = TASK.module

import hhpatch  # noqa: E402

FAR = [{"tcp": np.array([0.12, 0.30, 1.0]), "tips": [np.array([0.12, 0.31, 0.99]), np.array([0.12, 0.29, 0.99])]},
       {"tcp": np.array([0.15, -0.30, 1.0]), "tips": [np.array([0.15, -0.29, 0.99]), np.array([0.15, -0.31, 0.99])]}]


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = FAR, np.zeros((0, 3)), {}
    yield


def patch(hinge=0.0, pos=None, vel=(0.0, 0.0, 0.0)):
    return {"patch": hhpatch.offline_state(M.PATCH, hinge, pos, vel)}


def run(states, grips=None, n=1):
    out = None
    for st in states:
        for _ in range(n):
            if grips is not None:
                M.TRIAL.grippers_override = grips
            M.TRIAL.new_step()
            out = (M.success(st), M.score(st))
            M.distance(st)
    return out


def in_can():
    return (M.CAN_XY[0], M.CAN_XY[1], M.PT + 2 * M.T)


def in_bin():
    return (M.BIN_XY[0], M.BIN_XY[1], M.PT + 2 * M.T)


def test_loads():
    assert TASK.scene == "station"
    names = [o.name for o in M.OBJECTS]
    assert {"patch", "shelf", "patient"} <= set(names) and len(names) == len(set(names))
    assert "takeback_n" in names and "trash_n" in names


def test_geometry():
    # the patch lies on the shelf, its near end over the front edge, its middle well on the shelf
    assert M.X0 - M.L / 2 < M.XE < M.X0
    assert M.SHELF_XY[1] - M.SHELF[1] / 2 < M.HY - M.W / 2 and M.HY + M.W / 2 < M.SHELF_XY[1] + M.SHELF[1] / 2
    s = patch()["patch"]
    assert abs(s.a_pos[1] - (M.HY + M.W / 4)) < 1e-3 and abs(s.b_pos[1] - (M.HY - M.W / 4)) < 1e-3  # B on the right
    # the containers stand on the shelf, apart, clear of the patch
    for xy, inner in ((M.CAN_XY, M.CAN_IN), (M.BIN_XY, M.BIN_IN)):
        assert M.XE < xy[0] - inner[0] / 2 and xy[0] + inner[0] / 2 < M.XE + M.SHELF[0]
        assert not M.inside(s.pos, xy, inner)
    assert M.CAN_XY[1] + M.CAN_IN[1] / 2 < M.BIN_XY[1] - M.BIN_IN[1] / 2
    # the patient is clear of the shelf and the robot at rest
    lo, hi = M.PERSON.bounds
    assert lo[1] > M.SHELF_XY[1] + M.SHELF[1] / 2
    assert run([patch()]) == (False, 0.0)


def test_folded_in_takeback_scores_1():
    ok, sc = run([patch(180.0, in_can())], n=25)
    assert sc == 1.0 and ok


def test_success_needs_it_to_rest():
    ok, sc = run([patch(180.0, in_can(), vel=(0.0, 0.0, -0.3))], n=25)
    assert sc == 1.0 and not ok


def test_unfolded_in_takeback_scores_0():
    assert run([patch(0.0, in_can())], n=25) == (False, 0.0)
    assert run([patch(150.0, in_can())], n=25) == (False, 0.0)  # half folded, sticky faces not together


def test_folded_into_trash_scores_0():
    assert run([patch(180.0), patch(180.0, in_bin())], n=25) == (False, 0.0)


def test_folded_but_left_on_shelf_scores_half():
    assert run([patch(180.0)], n=5) == (False, 0.5)


def test_stuck_stays_stuck():
    # once the faces met, the patch counts as stuck even if read back a little open
    assert run([patch(180.0), patch(170.0, in_can())], n=25)[1] == 1.0


def test_dropped_off_the_shelf_scores_0():
    assert run([patch(180.0, (0.25, -0.20, M.TOP + 2 * M.T))], n=5) == (False, 0.0)


def test_robot_near_patient_is_0():
    lo, hi = M.PERSON.bounds
    a, b, r = M.PERSON.capsules[next(iter(M.PERSON.capsules))]
    p = np.asarray(a, float) + np.array([0.0, 0.0, r + 0.02])
    near = [FAR[0], {"tcp": p, "tips": [p, p]}]
    run([patch()], grips=near)
    M.TRIAL.grippers_override = FAR
    assert run([patch(180.0, in_can())], n=25) == (False, 0.0)
    assert M.TRIAL.mem["violated"]


def test_demo_plans():
    import os
    for v in ("demo", "naive", "unsafe"):
        os.environ["TASK_DEMO"] = v
        plan = M.demo()
        assert set(plan) == {"right"} and len(plan["right"]) > 5
    os.environ.pop("TASK_DEMO")
