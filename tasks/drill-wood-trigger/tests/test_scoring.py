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


def test_contact_and_trigger_without_a_hole_cannot_finish():
    task, objects = fixture()
    m = task.module.measure(objects, {"right_holding": True, "left_trigger_contact": True, "bit_wood_contact": True})
    assert not m["success_now"] and m["tip_on_target"] and m["trigger_pressed"]


def completed_hole(task, **changes):
    robot={"right_holding":True,"cut_depth_m":.011,"qualified_depth_m":.011,
           "hole_entry_m":[*task.module.TARGET_XY,task.module.SURFACE_Z],"bore_axis":[0.,0.,-1.]}
    robot.update(changes)
    return robot


def test_completed_hole_requires_withdrawal_and_trigger_release():
    task,objects=fixture(height_error=.020,travel=0.)
    assert task.module.measure(objects,completed_hole(task))['success_now']
    for height,travel in ((-.011,0.),(.020,.004),(.003,0.)):
        task,objects=fixture(height_error=height,travel=travel)
        assert not task.module.measure(objects,completed_hole(task))['success_now']


@pytest.mark.parametrize('changes',[
    {'cut_depth_m':.004}, {'qualified_depth_m':.004}, {'cut_depth_m':.016},
    {'hole_entry_m':[.28,-.08,.7775]}, {'bore_axis':[.5,0.,-.866]},
    {'right_holding':False}, {'hole_entry_m':None},
])
def test_unfinished_unqualified_off_target_or_overdrilled_hole_cannot_finish(changes):
    task,objects=fixture(height_error=.020,travel=0.)
    assert not task.module.measure(objects,completed_hole(task,**changes))['success_now']


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
    task, objects = fixture(height_error=.020,travel=0.)
    task.module._robot = completed_hole(task)
    monkeypatch.setitem(sys.modules, "yam_arena.scene", SimpleNamespace(object_states=lambda *_: objects))
    embodiment = YamArenaEmbodiment(task=str(TASK), control_hz=20)
    embodiment.task.module._robot = dict(task.module._robot)
    monkeypatch.setattr(embodiment, "_observation", lambda: Observation())
    for _ in range(9):
        assert not embodiment._task_step().terminated
    embodiment.task.module._robot["right_holding"] = False
    assert not embodiment._task_step().terminated
    embodiment.task.module._robot["right_holding"] = True
    for _ in range(9):
        assert not embodiment._task_step().terminated
    assert embodiment._task_step().termination_reason == "success"
