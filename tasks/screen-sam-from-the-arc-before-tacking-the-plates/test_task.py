"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import ObjectState, load  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(str(HERE / "task.py"))
M = TASK.module

FAR = [{"tcp": np.array([0.05, 0.10, 1.0]), "tips": [np.array([0.05, 0.13, 0.99]), np.array([0.05, 0.07, 0.99])]},
       {"tcp": np.array([0.15, -0.30, 1.0]), "tips": [np.array([0.15, -0.27, 0.99]), np.array([0.15, -0.33, 0.99])]}]
UP = (0.0, 0.0, 0.0, 1.0)
Z = np.zeros(3)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = FAR, np.zeros((0, 3)), {}
    yield


def body(origin, c, quat=UP, vel=(0.0, 0.0, 0.0)):
    p = np.asarray(origin, float) + M.hhkit.rot(quat) @ c
    return ObjectState(pos=p, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float), ang_vel=Z, low=p, high=p)


def plain(p):
    p = np.asarray(p, float)
    return ObjectState(pos=p, quat_xyzw=np.array(UP), lin_vel=Z, ang_vel=Z, low=p, high=p)


def yawq(a):
    return (0.0, 0.0, float(np.sin(a / 2)), float(np.cos(a / 2)))


T_REST = (*M.TORCH_REST, M.TORCH_Z)
S_START = (*M.SCREEN_START, M.TOP + M.FIN_Z)
S_GOAL = (*M.SCREEN_GOAL, M.TOP + M.FIN_Z)
SQ0 = (0.0, 0.0, float(np.sin(M.SCREEN_START_YAW / 2)), float(np.cos(M.SCREEN_START_YAW / 2)))


def torch_at_tip(h, lat=0.0):
    g = np.array([M.TACK[0] + lat, M.TACK[1], M.PLATE_Z + h]) - M.TIP
    return tuple(g)


def scene(torch, screen, sq=UP):
    return {"torch": body(torch, M.TORCH_C), "screen": body(screen, M.SCREEN_C, sq), "sam": plain(Z)}


def hold_torch(torch):
    g = np.asarray(torch, float)
    M.TRIAL.grippers_override = [FAR[0], {"tcp": g + np.array([0, 0, 0.004]), "tips": [g + (0, 0.016, -0.01), g + (0, -0.016, -0.01)]}]


def run(objs, seconds=1.2):
    s = 0.0
    for _ in range(M.TRIAL.frames(seconds)):
        M.TRIAL.new_step()
        s = M.score(objs)
    return s, M.success(objs)


def test_start_does_not_shade_and_goal_does():
    assert not M.shades_face(scene(T_REST, S_START, SQ0)["screen"])
    assert M.shades_face(scene(T_REST, S_GOAL)["screen"])


def test_tungsten_geometry():
    o = scene(T_REST, S_START, SQ0)
    assert abs(M.tungsten(o["torch"])[2] - (M.TOP + 0.015)) < 1e-9
    assert np.allclose(M.tungsten(scene(torch_at_tip(0.0035), S_START)["torch"])[:2], M.TACK)


def test_start_scores_zero():
    assert run(scene(T_REST, S_START, SQ0)) == (0.0, False)


def test_screen_only_half():
    assert run(scene(T_REST, S_GOAL)) == (0.5, False)


def test_screen_off_to_one_side_does_not_shade():
    off = (S_GOAL[0] + 0.25, S_GOAL[1], S_GOAL[2])
    assert not M.shades_face(scene(T_REST, off)["screen"])
    assert not M.shades_face(scene(T_REST, off, yawq(np.pi / 2))["screen"])


def _tack(screen, sq=UP):
    t = torch_at_tip(0.0035)
    hold_torch(t)
    run(scene(t, screen, sq), seconds=M.TACK_S + 0.1)
    M.TRIAL.grippers_override = FAR


def test_tack_unshaded_is_zero_for_good():
    _tack(S_START, SQ0)
    assert M.TRIAL.mem["violated"]
    assert run(scene(T_REST, S_GOAL)) == (0.0, False)


def test_demo_end_state_full():
    _tack(S_GOAL)
    assert M.TRIAL.mem["tacked"] and not M.TRIAL.mem["violated"]
    assert run(scene(T_REST, S_GOAL)) == (1.0, True)


def test_tack_needs_arc_window():
    t = torch_at_tip(0.008)
    hold_torch(t)
    run(scene(t, S_GOAL), seconds=2.0)
    assert not M.TRIAL.mem["tacked"]
    M.TRIAL.grippers_override = FAR
    assert run(scene(T_REST, S_GOAL)) == (0.5, False)


def test_torch_not_held_is_no_arc():
    t = torch_at_tip(0.0035)
    run(scene(t, S_START, SQ0), seconds=2.0)
    assert not M.TRIAL.mem["tacked"] and not M.TRIAL.mem["violated"]


def test_tacked_torch_left_off_rest_is_half():
    _tack(S_GOAL)
    assert run(scene((0.30, -0.05, M.TOP + 0.02), S_GOAL)) == (0.5, False)


def test_screen_moved_away_after_tack_is_zero():
    _tack(S_GOAL)
    assert run(scene(T_REST, S_START, SQ0)) == (0.0, False)


def test_task_json_limits():
    import json
    d = json.loads((HERE / "task.json").read_text())
    assert len(d["title"]) <= 120 and len(d["fidelity"]) <= 500 and len(d["scores"]) <= 300 and len(d.get("context", "")) <= 1000
    assert d["instruction"] == M.INSTRUCTION
