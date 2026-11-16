"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

import hhbungee  # noqa: E402
import hhreplay as R  # noqa: E402
import test_offline as O  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "task.py")
M = O.M
NONE = np.zeros((0, 3))
H = tuple(s / 2 for s in M.FERRULE)
YAW = {"hook_right": 0.0, "hook_left": math.pi}


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = R.PARKED, NONE, None
    yield


def hook(name, x, y, z, vel=(0.0, 0.0, 0.0)):
    return R.state(np.array([x, y, z]), R.hhkit.yaw_quat(YAW[name]), half=H, vel=vel)


def scene(**hooks):
    st = R.initial_states(M)
    st.update(hooks)
    return st


def step(objects, grips=None):
    if grips is not None:
        M.TRIAL.grippers_override = grips
    M.TRIAL.new_step()
    ok = M.success(objects)
    M.distance(objects)
    return ok, M.score(objects)


def held(h):
    return [R.gripper(np.asarray(h.pos) + [0, 0, 0.009]), R.PARKED[1]]


def test_loads():
    assert TASK.scene == "station"
    names = [o.name for o in M.OBJECTS]
    assert {"crate", "lid", "hook_right", "hook_left", "colleague"} <= set(names) and len(names) == len(set(names))
    assert set(TASK.free_objects) == {"hook_right", "hook_left"}


def test_start_stretched_and_seated():
    st = scene()
    assert M.span(st) == pytest.approx(0.302, abs=1e-3)
    f, _u = hhbungee.pull(M.LINE, *hhbungee.attach_points(M.LINE, {n: (st[n].pos, st[n].quat_xyzw, st[n].lin_vel)
                                                                   for n in M.HOOKS}))
    assert f == pytest.approx(135 * 0.062, rel=1e-3)
    assert all(M.seated(st[n], n) for n in M.HOOKS)
    assert step(st) == (False, 0.0)


def test_pull_is_zero_when_slack():
    assert hhbungee.pull(M.LINE, np.zeros(3), np.array([0.0, 0.2, 0.0]))[0] == 0.0
    f, u = hhbungee.pull(M.LINE, np.zeros(3), np.array([0.0, 0.3, 0.0]), vb=np.array([0.0, 1.0, 0.0]))
    assert f == pytest.approx(135 * 0.06 + 0.3) and u[1] == pytest.approx(1.0)


def test_hook_geometry():
    st = scene()
    assert M.tip_bottom(st["hook_right"]) == pytest.approx(M.LID_TOP - 0.022, abs=1e-6)
    lifted = hook("hook_right", M.HOOK_R_XY[0], M.HOOK_R_XY[1] - 0.01, M.SEAT_Z + 0.035)
    assert not M.seated(lifted, "hook_right") and M.tip_bottom(lifted) > M.LID_TOP + M.OFF_M


def test_flying_hook_latches_zero():
    st = scene(hook_right=hook("hook_right", 0.30, 0.0, M.SEAT_Z + 0.035, vel=(0, 5.0, 0)))
    assert step(st) == (False, 0.0)
    assert M.TRIAL.mem["violated"] and "right-hand hook" in M.TRIAL.mem["why"]
    assert step(scene()) == (False, 0.0)  # latched


def test_fast_while_held_is_not_a_fly():
    h = hook("hook_right", 0.30, -0.08, M.SEAT_Z + 0.035, vel=(0, 1.5, 0))
    assert step(scene(hook_right=h), held(h))[1] == 0.25
    assert not M.TRIAL.mem["violated"]


def test_set_down_speed_is_not_a_fly():
    h = hook("hook_right", 0.30, -0.05, M.SEAT_Z + 0.02, vel=(0, 0, -0.77))
    step(scene(hook_right=h))
    assert not M.TRIAL.mem["violated"]


def test_one_hook_off_half():
    h = hook("hook_right", 0.30, -0.048, M.SEAT_Z + 0.02)
    st = scene(hook_right=h)
    assert M.span(st) <= M.SLACK_M
    assert step(st) == (False, 0.5)


def test_both_off_slack_full():
    st = scene(hook_right=hook("hook_right", 0.30, -0.048, M.SEAT_Z + 0.02),
               hook_left=hook("hook_left", 0.30, 0.173, M.SEAT_Z + 0.02))
    assert step(st) == (True, 1.0)


def test_both_off_but_stretched_not_full():
    st = scene(hook_right=hook("hook_right", 0.30, -0.16, M.SEAT_Z + 0.02),
               hook_left=hook("hook_left", 0.30, 0.26, M.SEAT_Z + 0.02))
    assert M.span(st) > M.SLACK_M
    assert step(st) == (False, 0.5)


def test_robot_near_him_latches_zero():
    a, b, r = M.PERSON.capsules["R_forearm"]
    step(scene(), [R.gripper(b + [0, 0, r + 0.03]), R.PARKED[1]])
    assert M.TRIAL.mem["min_body"] < M.BODY_STANDOFF_M and M.TRIAL.mem["violated"]


def test_hand_is_beside_the_cord():
    hand = M.PERSON.points["R_hand"]
    assert hand[:, 0].min() - M.CRATE_XY[0] < 0.03  # fingertips within 3 cm of the cord line
    assert hand[:, 1].min() > M.HOOK_R_XY[1] + 0.10 and hand[:, 1].max() < M.HOOK_L_XY[1] - 0.05


def test_plans_reachable():
    for name in M.PLANS:
        for side, ws in M.PLANS[name]().items():
            assert R.reachable(side, ws), (name, side)


@pytest.mark.parametrize("name,want", [("demo", 1.0), ("naive", 0.0), ("unsafe", 0.0)])
def test_replays(name, want):
    got, why, mem = O.run(name)
    assert got == want, (why, mem)
    if name != "demo":
        assert "flew" in why
    else:
        assert mem["min_body"] > M.BODY_STANDOFF_M + 0.03


def test_seated_hook_settling_is_not_a_fly():
    st = scene()
    st["hook_right"] = hook("hook_right", *M.HOOK_R_XY, M.SEAT_Z, vel=(0, 1.3, 0))
    step(st)
    assert not M.TRIAL.mem["violated"]
