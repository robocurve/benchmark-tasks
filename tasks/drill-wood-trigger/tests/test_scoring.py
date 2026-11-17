"""Independent scoring checks; no simulator or GPU required."""

from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from inspect_robots.types import Observation
from yam_arena import ik
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.task_file import ObjectState, load
from yam_arena.urdf import axis_angle_matrix

TASK = Path(__file__).resolve().parents[1] / "task.py"


def fixture(*, xy_error=0., height_error=0., tilt=0., travel=.004, trigger_side_error=0.):
    task = load(TASK)
    m = task.module
    rotation = axis_angle_matrix(np.array([1., 0., 0.]), np.radians(tilt)) @ axis_angle_matrix(
        np.array([0., 1., 0.]), -np.pi/2)
    tip = np.r_[m.TARGET_XY + [xy_error, 0.], m.SURFACE_Z + height_error]
    origin = tip - rotation @ m.BIT_TIP_LOCAL
    quat = ik.matrix_to_quat_xyzw(rotation)

    def state(pos):
        pos = np.asarray(pos)
        return ObjectState(pos, quat, np.zeros(3), np.zeros(3), pos-.01, pos+.01)

    objects = {
        "drill": state(origin + rotation @ m.MODEL_CENTRE),
        "trigger": state(origin + rotation @ (m.TRIGGER_LOCAL + [travel, trigger_side_error, 0.])),
    }
    return task, objects


def test_valid_two_arm_trigger_contact_reaches_success_region():
    task, objects = fixture()
    m = task.module.measure(objects, {"right_holding": True, "left_trigger_contact": True, "bit_wood_contact": True})
    assert m["success_now"] and m["tip_on_target"] and m["trigger_pressed"]


@pytest.mark.parametrize("forces, expected", [
    ([2., 2.], True), ([2., 0.], False), ([0., 2.], False),
    ([0., 0.], False), ([2., np.nan], False), ([2.], False),
])
def test_holding_requires_both_fingers_touching_the_drill(forces, expected):
    task = load(TASK)
    assert task.module.holding_handle([.2, 0., .9], [.2, 0., .9], .04, forces) is expected


def test_finger_contact_away_from_handle_does_not_count_as_holding():
    task = load(TASK)
    assert not task.module.holding_handle([.2, 0., .9], [.3, 0., .9], .04, [2., 2.])
    assert not task.module.holding_handle([.2, 0., .9], [.2, 0., .9], .09, [2., 2.])


@pytest.mark.parametrize("robot", [
    {}, {"right_holding": True}, {"left_trigger_contact": True},
    {"right_holding": False, "left_trigger_contact": True},
])
def test_geometry_alone_or_one_arm_cannot_finish(robot):
    task, objects = fixture()
    assert not task.module.measure(objects, robot)["success_now"]


def test_geometric_proximity_without_actual_wood_contact_cannot_finish():
    task, objects = fixture(height_error=.002)
    assert not task.module.measure(objects, {
        "right_holding": True, "left_trigger_contact": True, "bit_wood_contact": False,
    })["success_now"]


@pytest.mark.parametrize("change", [
    {"xy_error": .012}, {"height_error": .008}, {"height_error": -.008},
    {"tilt": 15.}, {"travel": .001}, {"travel": .020}, {"trigger_side_error": .020},
])
def test_off_target_hovering_penetration_tilt_or_unpressed_detached_trigger_fail(change):
    task, objects = fixture(**change)
    assert not task.module.measure(objects, {"right_holding": True, "left_trigger_contact": True, "bit_wood_contact": True})["success_now"]


def test_score_requires_the_uninterrupted_half_second(monkeypatch):
    task, objects = fixture()
    task.module._robot = {"right_holding": True, "left_trigger_contact": True, "bit_wood_contact": True}
    monkeypatch.setitem(sys.modules, "yam_arena.scene", SimpleNamespace(object_states=lambda *_: objects))
    embodiment = YamArenaEmbodiment(task=str(TASK), control_hz=20)
    embodiment.task.module._robot = dict(task.module._robot)
    monkeypatch.setattr(embodiment, "_observation", lambda: Observation())
    for _ in range(9):
        assert not embodiment._task_step().terminated
    embodiment.task.module._robot["left_trigger_contact"] = False
    assert not embodiment._task_step().terminated
    embodiment.task.module._robot["left_trigger_contact"] = True
    for _ in range(9):
        assert not embodiment._task_step().terminated
    assert embodiment._task_step().termination_reason == "success"
