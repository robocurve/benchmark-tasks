"""Observable scoring, scene constraints and irreversible safety regressions."""

from pathlib import Path

import numpy as np
import pytest
from yam_arena.task_file import ObjectState, load


@pytest.fixture
def task():
    return load(Path(__file__).with_name("task.py")).module


def state(pos, tilt=0, spin=0):
    pos = np.asarray(pos, dtype=float)
    q = np.array([np.sin(np.radians(tilt) / 2), 0, 0, np.cos(np.radians(tilt) / 2)])
    half = np.array([0.018, 0.018, 0.05])
    return ObjectState(
        pos, q, np.zeros(3), np.array([0, 0, spin]), pos - half, pos + half
    )


def initial(task):
    return {b.name: state(b.pos) for b in task.BOTTLES}


def sorted_states(task):
    out = initial(task)
    for n in ["red_swap", "blue_swap"]:
        out[n] = state((0.294, 0, task.surface(task.SHELVES[task.GOALS[n]]) + 0.05))
    return out


def settle(task, objects, released=True):
    task.MONITOR.armed = True
    for step in range(41):
        task.MONITOR.update(objects)
        task.MONITOR.observe_control(objects, step / 20, released)


def test_initial_state_zero(task):
    objects = initial(task)
    settle(task, objects)
    assert task.score(objects) == 0 and not task.success(objects)


def test_full_credit_requires_two_second_observable_window(task):
    objects = sorted_states(task)
    task.MONITOR.armed = True
    for step in range(40):
        task.MONITOR.observe_control(objects, step / 20, True)
    assert task.score(objects) == 0 and not task.success(objects)
    task.MONITOR.observe_control(objects, 2.0, True)
    assert task.success(objects) and task.score(objects) == 1


def test_gripper_release_and_regrasp_clear_window(task):
    objects = sorted_states(task)
    settle(task, objects)
    task.MONITOR.observe_control(objects, 2.05, False)
    assert task.score(objects) == 0 and not task.MONITOR.stable


def test_partial_credit(task):
    objects = sorted_states(task)
    objects["blue_swap"] = initial(task)["blue_swap"]
    settle(task, objects)
    assert task.score(objects) == 0.5 and not task.success(objects)


@pytest.mark.parametrize("kind", ["tilt", "fall", "workspace", "nan"])
def test_unsafe_event_latches_after_repair(task, kind):
    objects = sorted_states(task)
    task.MONITOR.armed = True
    p = objects["red_left"].pos.copy()
    if kind == "tilt":
        objects["red_left"] = state(p, tilt=21)
    elif kind == "fall":
        p[2] -= 0.02
        objects["red_left"] = state(p)
    elif kind == "workspace":
        p[0] = 0.5
        objects["red_left"] = state(p)
    else:
        p[0] = float("nan")
        objects["red_left"] = state(p)
    task.MONITOR.update(objects)
    assert task.MONITOR.reasons
    objects = sorted_states(task)
    settle(task, objects)
    assert task.score(objects) == 0 and not task.success(objects)
    task.MONITOR = task.SafetyMonitor()
    settle(task, objects)
    assert task.success(objects)


def test_force_is_telemetry_only(task):
    objects = sorted_states(task)
    settle(task, objects)
    task.MONITOR.update(objects, {"blue_swap": 1000})
    assert task.MONITOR.peak_force_n["blue_swap"] == 1000
    assert task.score(objects) == 1 and not task.MONITOR.reasons
    task.MONITOR.update(objects, {"blue_swap": float("nan")})
    assert task.score(objects) == 1


@pytest.mark.parametrize("kind", ["hover", "edge", "leaning"])
def test_invalid_placement_no_full_credit(task, kind):
    objects = sorted_states(task)
    p = objects["red_swap"].pos.copy()
    if kind == "hover":
        p[2] += 0.02
    elif kind == "edge":
        p[1] = 0.08
    objects["red_swap"] = state(p, tilt=6 if kind == "leaning" else 0)
    settle(task, objects)
    assert not task.success(objects)
    assert task.score(objects) == (0 if kind == "leaning" else 0.5)


def test_visible_drift_invalidates_stability(task):
    objects = sorted_states(task)
    task.MONITOR.armed = True
    for step in range(41):
        shifted = dict(objects)
        p = objects["red_swap"].pos.copy()
        p[0] += 0.006 * step / 40
        shifted["red_swap"] = state(p)
        task.MONITOR.observe_control(shifted, step / 20, True)
    assert not task.MONITOR.stable


def test_vertical_spin_is_irrelevant(task):
    objects = sorted_states(task)
    objects["red_swap"] = state(objects["red_swap"].pos, spin=2)
    settle(task, objects)
    assert task.success(objects)


def test_stock_loader_cannot_bypass_monitor(task):
    assert task.score(sorted_states(task)) == 0


def test_scene_is_stacked_and_crowded(task):
    red, blue = task.SHELVES["red"], task.SHELVES["blue"]
    assert red.pos[:2] == blue.pos[:2]
    assert task.surface(blue) - task.surface(red) == pytest.approx(0.16)
    assert len(task.BOTTLES) == 6
    for b in task.BOTTLES:
        assert b.pos[0] - task.RADIUS - task.RACK_FRONT == pytest.approx(0.006)
    assert 0.085 - (0.060 + task.RADIUS) == pytest.approx(0.007)
    assert 0.060 - 2 * task.RADIUS == pytest.approx(0.024)


def test_falling_toward_lower_shelf_is_unsafe(task):
    objects = initial(task)
    task.MONITOR.armed = True
    s = objects["blue_left"]
    objects["blue_left"] = ObjectState(
        s.pos, s.quat_xyzw, np.array([0, 0, -0.36]), s.ang_vel, s.low, s.high
    )
    task.MONITOR.update(objects)
    assert "blue_left:uncontrolled_fall" in task.MONITOR.reasons


def test_demo_uses_horizontal_jaws(task):
    assert np.allclose(task.HORIZONTAL_ROTATION[:, 2], [1, 0, 0])
    assert np.allclose(abs(task.HORIZONTAL_ROTATION[:, 0]), [0, 1, 0])
    for plan in task.demo().values():
        assert all(np.allclose(w.quat, task.HORIZONTAL_QUAT) for w in plan)
