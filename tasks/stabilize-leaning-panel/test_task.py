"""Scorer regression checks; contact feasibility is checked by probe.py on the GPU."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest
from yam_arena.task_file import ObjectState
from yam_arena.urdf import quat_xyzw_to_matrix

spec = importlib.util.spec_from_file_location(
    "panel_task", Path(__file__).with_name("task.py")
)
task = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task)


def state(pos, size, quat=(0, 0, 0, 1)):
    pos = np.array(pos, dtype=float)
    quat = np.array(quat, dtype=float)
    extent = np.abs(quat_xyzw_to_matrix(quat)) @ (np.array(size) / 2)
    return ObjectState(pos, quat, np.zeros(3), np.zeros(3), pos - extent, pos + extent)


def layout(seated=True):
    return {
        "panel": state(task.PANEL.pos, task.PANEL.size, task.PANEL_QUAT),
        "stopper": state(
            task.STOP_GOAL if seated else task.STOPPER.pos, task.STOPPER.size
        ),
    }


def advance(m, objects, seconds):
    for _ in range(round(seconds * 240)):
        m.physics(objects, 1 / 240)


def monitor():
    m = task.Monitor()
    m.armed = True
    return m


def test_initial_layout_has_no_progress():
    task.MONITOR = monitor()
    assert task.score(layout(False)) == 0


def test_requires_release_and_real_proof_interval():
    m = monitor()
    objects = layout()
    m.control(objects, False)
    advance(m, objects, 4)
    assert m.proof_start is None
    m.control(objects, True)
    advance(m, objects, task.PULL_S + task.SETTLE_S - 0.02)
    assert not m.complete
    advance(m, objects, 0.05)
    assert m.complete


def test_pulse_has_fixed_duration():
    m = monitor()
    objects = layout()
    m.control(objects, True)
    assert m.pulling
    advance(m, objects, task.PULL_S + 0.01)
    assert not m.pulling


def test_returning_hands_fails_irreversibly():
    m = monitor()
    objects = layout()
    m.control(objects, True)
    m.control(objects, False)
    advance(m, objects, 0.1)
    m.control(objects, True)
    advance(m, objects, 4)
    assert "hands_returned_during_proof" in m.reasons
    assert not m.complete


def test_lost_stopper_fails_irreversibly():
    m = monitor()
    objects = layout()
    m.control(objects, True)
    advance(m, layout(False), 0.1)
    advance(m, objects, 4)
    assert "stopper_lost_during_proof" in m.reasons
    assert not m.complete


@pytest.mark.parametrize("kind", ["tilt", "translation", "lift", "nonfinite", "drop"])
def test_invalid_outcomes_latch_zero(kind):
    m = monitor()
    objects = layout()
    if kind == "tilt":
        angle = np.radians(40)
        objects["panel"] = state(
            task.PANEL.pos,
            task.PANEL.size,
            (0, np.sin(angle / 2), 0, np.cos(angle / 2)),
        )
    elif kind == "translation":
        objects["panel"] = state(
            np.array(task.PANEL.pos) + (0.03, 0, 0), task.PANEL.size, task.PANEL_QUAT
        )
    elif kind == "lift":
        objects["panel"] = state(
            np.array(task.PANEL.pos) + (0, 0, 0.02), task.PANEL.size, task.PANEL_QUAT
        )
    elif kind == "nonfinite":
        objects["stopper"].pos[0] = np.nan
    else:
        objects["stopper"] = state((0.2, 0, task.TOP - 0.1), task.STOPPER.size)
    advance(m, objects, 0.01)
    assert m.reasons
    advance(m, layout(), 4)
    task.MONITOR = m
    assert task.score(layout()) == 0


def test_fresh_episode_does_not_inherit_failures():
    old = monitor()
    old.reasons.add("panel_unsafe_tilt")
    fresh = monitor()
    assert not fresh.reasons
    assert fresh.proof_start is None
    assert not fresh.complete


def test_stopper_orientation_and_height_matter():
    good = layout()["stopper"]
    assert task.seated(good)
    bad = state(
        task.STOP_GOAL, task.STOPPER.size, (0, 0, np.sin(np.pi / 8), np.cos(np.pi / 8))
    )
    assert not task.seated(bad)
    assert not task.seated(state(task.STOP_GOAL + (0, 0, 0.02), task.STOPPER.size))


def test_fixed_layout_is_geometrically_consistent():
    assert np.isclose(layout()["panel"].low[2], task.BASE_Z)
    socket_front = next(o for o in task.FIXTURES if o.name == "socket_front")
    assert (
        task.STOP_GOAL[0] - task.STOPPER.size[0] / 2
        >= socket_front.pos[0] + socket_front.size[0] / 2
    )
    assert task.STOP_GOAL[0] + task.STOPPER.size[
        0
    ] / 2 < task.BASE_X - task.THICKNESS / 2 * np.cos(task.ANGLE)


def test_socket_rails_cannot_stop_the_unbraced_panel():
    for o in task.FIXTURES:
        if o.name in {"socket_left", "socket_right"}:
            # Rail ends must allow the foot to cross the unsafe drift threshold.
            assert o.pos[0] + o.size[0] / 2 < task.BASE_X - 0.018 - task.THICKNESS / 2


def test_proof_allows_transient_motion_inside_socket_before_settling():
    m = monitor()
    objects = layout()
    m.control(objects, True)
    objects["stopper"].lin_vel[0] = -0.1
    advance(m, objects, 0.1)
    assert not m.reasons
    assert not m.complete
    objects["stopper"].lin_vel[:] = 0
    advance(m, objects, 3)
    assert m.complete


def test_proof_cannot_finish_while_stopper_is_moving():
    m = monitor()
    objects = layout()
    m.control(objects, True)
    objects["stopper"].lin_vel[0] = 0.1
    advance(m, objects, 3)
    assert not m.reasons
    assert not m.complete
