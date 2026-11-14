"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest prove-voltage-detector-before-and-after-test    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.modules.pop("yam_parts", None)
import yam_parts
sys.modules.pop("yam_joints", None)
import yam_joints
from dataclasses import replace

TASK = load(Path(__file__).with_name("prove_voltage_detector_before_and_after_test.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 2400


def _rot(yaw_deg, tilt_deg, axis):
    y, t = math.radians(yaw_deg), math.radians(tilt_deg)
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    c, s = math.cos(t), math.sin(t)
    rt = np.array([[1, 0, 0], [0, c, -s], [0, s, c]]) if axis == "x" else np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return rz @ rt


def _half(o, r):
    if hasattr(o, "size"):
        return np.abs(r) @ (np.asarray(o.size) / 2)
    if hasattr(o, "height"):
        a = r[:, 2]
        return o.height / 2 * np.abs(a) + o.radius * np.sqrt(np.clip(1 - a * a, 0, 1))
    return np.full(3, o.radius)


def place(name, pos, yaw_deg=0.0, tilt_deg=0.0, tilt_axis="x", vel=(0.0, 0.0, 0.0), quat=None, joints=None):
    """The ObjectState of `name` with its centre at `pos`, turned and tipped; its bounding box follows the turn."""
    r = (_rot(yaw_deg, tilt_deg, tilt_axis) if quat is None else
         ik.quat_xyzw_to_matrix(np.asarray(quat, float) / np.linalg.norm(quat)))
    if isinstance(OBJ[name], yam_joints.Mechanism):
        q = {link.name: float((joints or {}).get(link.name, 0.0)) *
             (math.pi / 180 if isinstance(link.joint, yam_joints.Hinge) else 1.0)
             for link in OBJ[name].links}
        state = OBJ[name].offline_state(pos, math.radians(yaw_deg), q, R=r)
        return replace(state, lin_vel=np.asarray(vel, float))
    if isinstance(OBJ[name], yam_parts.Part):
        origin = np.asarray(pos, float)
        low, high = OBJ[name].aabb(origin, r)
        return yam_parts.PartState((low + high) / 2, np.asarray(ik.matrix_to_quat_xyzw(r), float),
                                   np.asarray(vel, float), np.zeros(3), low, high, origin)
    pos, half = np.asarray(pos, float), _half(OBJ[name], r)
    q = ik.matrix_to_quat_xyzw(r)
    return ObjectState(pos, np.asarray(q, float), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)


def layout(poses):
    """Every object's state: the loose ones as given, the fixed ones where the task file puts them."""
    out = {o.name: place(o.name, o.pos, math.degrees(o.yaw)) for o in T.OBJECTS if o.mass is None}
    out.update({name: place(name, **p) for name, p in poses.items()})
    return out


START_POSES = {
    'detector': {'pos': (0.13, -0.075, 0.8775), 'joints': {'stick': 0.0, 'on_button': 0.0}},
    'tester': {'pos': (0.13, -0.3875, 0.8775), 'joints': {'test_button': 0.0}},
}
GOAL_POSES = {
    'detector': {'pos': (0.13, -0.015, 0.8775), 'joints': {'stick': 0.0, 'on_button': 0.0}},
    'tester': {'pos': (0.13, -0.3875, 0.8775), 'joints': {'test_button': 0.0}},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_RECORD': "{'selftest': True, 'proved': True, 'touched': True, 'reproved': True}"}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'detector'}
    assert TASK.docs and TASK.docs.strip()


def test_the_start_scores_nothing():
    assert T.score(START) == 0.0
    assert not T.success(START)


def test_the_goal_scores_full_and_succeeds(monkeypatch):
    module_values = {}
    for name, expr in GOAL_MODULE.items():
        module_values[name] = eval(expr, {**vars(T), **module_values, "math": math, "__builtins__": {}})
    for name, value in module_values.items():
        monkeypatch.setattr(T, name, value)
    assert T.score(GOAL) == 1.0
    assert T.success(GOAL)

# (name, the layout it starts from, what's moved, the score it should get, module overrides)
CASES = [
    ('untouched start (working tester)', START_POSES, {
    }, 0, {}),
    ('untouched start (flat tester)', START_POSES, {
    }, 0, {'TEST_FLAT': 'True'}),
    ('ON pressed and the self-test finished', START_POSES, {
    }, 0.1, {'TEST_RECORD': "{'selftest': True}"}),
    ('proved on the tester (lights flashed 1 s), bar not tested', START_POSES, {
        'detector': {'pos': (0.13, -0.095, 0.8875), 'joints': {'stick': 0.0, 'on_button': 0.0}},
    }, 0.2, {'TEST_RECORD': "{'selftest': True, 'proved': True}"}),
    ('proved, then the top end held on the bar for 1 s', START_POSES, {
        'detector': {'pos': (0.29, -0.1093, 0.8825), 'joints': {'stick': 0.0, 'on_button': 0.0}},
    }, 0.6, {'TEST_RECORD': "{'selftest': True, 'proved': True, 'touched': True}"}),
    ('proved again after the bar, detector still held at the tester', START_POSES, {
        'detector': {'pos': (0.13, -0.095, 0.8875), 'joints': {'stick': 0.0, 'on_button': 0.0}},
    }, 0.8, {'TEST_RECORD': "{'selftest': True, 'proved': True, 'touched': True, 'reproved': True}", 'TEST_GRIPPERS': "{'left': (np.array([X0, BAL + PLATE_Y - TIP[1], TOP + AXIS_Z + 0.010]), 0.3), 'right': (np.array([X0, TEST_Y, TOP + 0.19]), 0.0)}"}),
    ('flat tester - self-test finished', START_POSES, {
    }, 0.1, {'TEST_FLAT': 'True', 'TEST_RECORD': "{'selftest': True}"}),
    ('flat tester - prove attempted, detector still at the tester', START_POSES, {
        'detector': {'pos': (0.13, -0.095, 0.8875), 'joints': {'stick': 0.0, 'on_button': 0.0}},
    }, 0.3, {'TEST_FLAT': 'True', 'TEST_RECORD': "{'selftest': True, 'attempt': True}"}),
    ('flat tester - prove attempted, detector held 2 cm off the plate', START_POSES, {
        'detector': {'pos': (0.13, -0.015, 0.8875), 'joints': {'stick': 0.0, 'on_button': 0.0}},
    }, 0.6, {'TEST_FLAT': 'True', 'TEST_RECORD': "{'selftest': True, 'attempt': True}", 'TEST_GRIPPERS': "{'left': (np.array([X0, Y0 + AWAY + BAL, TOP + AXIS_Z + 0.010]), 0.3), 'right': (np.array([X0, TEST_Y, TOP + 0.19]), 0.0)}"}),
    ('flat tester - prove attempted, detector laid back on its posts and released', START_POSES, {
        'detector': {'pos': (0.13, -0.015, 0.8775), 'joints': {'stick': 0.0, 'on_button': 0.0}},
    }, 1, {'TEST_FLAT': 'True', 'TEST_RECORD': "{'selftest': True, 'attempt': True}"}),
    ('bar touched before the detector was proved, then everything else done', GOAL_POSES, {
    }, 0, {'TEST_RECORD': "{'selftest': True, 'proved': True, 'touched': True, 'reproved': True, 'bad_touch': True}"}),  # An unproved detector was put on the bar; proving it afterwards does not undo that.
    ('flat tester - bar touched anyway, then laid back down', START_POSES, {
        'detector': {'pos': (0.13, -0.015, 0.8775), 'joints': {'stick': 0.0, 'on_button': 0.0}},
    }, 0, {'TEST_FLAT': 'True', 'TEST_RECORD': "{'selftest': True, 'attempt': True, 'bad_touch': True}"}),
    ('proved once and put back without testing the bar', GOAL_POSES, {
    }, 0.2, {'TEST_RECORD': "{'selftest': True, 'proved': True}"}),
    ('whole sequence done but the detector left leaning on the bar', START_POSES, {
        'detector': {'pos': (0.29, -0.1093, 0.8825), 'joints': {'stick': 0.0, 'on_button': 0.0}},
    }, 0.8, {'TEST_RECORD': "{'selftest': True, 'proved': True, 'touched': True, 'reproved': True}"}),
    ('whole sequence done but the detector fell off the table on the way', GOAL_POSES, {
    }, 0, {'TEST_RECORD': "{'selftest': True, 'proved': True, 'touched': True, 'reproved': True, 'dropped': True}"}),
    ('whole sequence done, detector lying on the floor', START_POSES, {
        'detector': {'pos': (0.13, -0.075, 0.4475), 'joints': {'stick': 0.0, 'on_button': 0.0}},
    }, 0, {'TEST_RECORD': "{'selftest': True, 'proved': True, 'touched': True, 'reproved': True}"}),
    ('flat tester - detector moved clear without ever switching it on or trying the tester', START_POSES, {
        'detector': {'pos': (0.13, -0.015, 0.8775), 'joints': {'stick': 0.0, 'on_button': 0.0}},
    }, 0, {'TEST_FLAT': 'True'}),
    ('whole sequence done but laid back where it lay, top end only 4.6 cm from the bar', START_POSES, {
    }, 0.8, {'TEST_RECORD': "{'selftest': True, 'proved': True, 'touched': True, 'reproved': True}"}),
    ('flat tester - prove attempted, then laid back where it lay, 4.6 cm from the bar', START_POSES, {
    }, 0.3, {'TEST_FLAT': 'True', 'TEST_RECORD': "{'selftest': True, 'attempt': True}"}),
]


@pytest.mark.parametrize(("name", "base", "moved", "expected", "module"), CASES, ids=[c[0] for c in CASES])
def test_named_layouts(name, base, moved, expected, module, monkeypatch):
    module_values = {}
    for name, expr in module.items():
        module_values[name] = eval(expr, {**vars(T), **module_values, "math": math, "__builtins__": {}})
    for name, value in module_values.items():
        monkeypatch.setattr(T, name, value)
    objects = layout({**base, **moved})
    assert T.score(objects) == pytest.approx(expected), name
    assert bool(T.success(objects)) == (expected == 1.0), name


def test_the_demo_is_reachable_within_the_joint_limits():
    plan = TASK.demo()
    assert set(plan) <= {"left", "right"}
    for side, waypoints in plan.items():
        arm = ArmPlan(waypoints, station.BASES[side])
        steps = round(arm.duration * 20) + 5
        actions = np.array([arm.action7((k + 1) / 20)[0] for k in range(steps)])
        assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1, side
        assert np.all(actions[:, :6] >= ik.LOWER - 1e-6) and np.all(actions[:, :6] <= ik.UPPER + 1e-6), side


def test_the_demo_fits_in_the_run_limit():
    longest = max(ArmPlan(w, station.BASES[s]).duration for s, w in TASK.demo().items())
    assert (longest + TASK.success_hold_s) * 20 < RUN_LIMIT_STEPS * 0.6
