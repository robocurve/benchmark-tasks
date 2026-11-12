"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest clip-pulse-oximeter-on-patient-finger    (in an environment with yam_arena installed)
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
sys.modules.pop("yam_mesh", None)
import yam_mesh
from dataclasses import replace

TASK = load(Path(__file__).with_name("clip_pulse_oximeter_on_patient_finger.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 1200


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
        q = {link.name: float((joints or {}).get(link.name, getattr(link.joint, "initial", 0.0))) *
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
    'hand': {'pos': (0.27, -0.075, 0.8312)},
    'oximeter': {'pos': (0.13, -0.0243, 0.8127), 'yaw_deg': -90, 'joints': {'top_jaw': -20.0}},
}
GOAL_POSES = {
    'hand': {'pos': (0.27, -0.075, 0.8312)},
    'oximeter': {'pos': (0.27, -0.0822, 0.8231), 'yaw_deg': -90, 'joints': {'top_jaw': -1.5}},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'oximeter', 'hand'}
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


def test_success_waits_for_the_objects_to_settle(monkeypatch):
    module_values = {}
    for name, expr in GOAL_MODULE.items():
        module_values[name] = eval(expr, {**vars(T), **module_values, "math": math, "__builtins__": {}})
    for name, value in module_values.items():
        monkeypatch.setattr(T, name, value)
    moving = layout({n: {**p, "vel": (0.05, 0.0, 0.0)} for n, p in GOAL_POSES.items()})
    assert not T.success(moving)

# (name, the layout it starts from, what's moved, the score it should get, module overrides)
CASES = [
    ('untouched start', START_POSES, {
    }, 0, {}),
    ('on the index finger 12 mm short of the stop', GOAL_POSES, {
        'oximeter': {'pos': (0.27, -0.0935, 0.8272), 'yaw_deg': -90, 'joints': {'top_jaw': -3.0}},
    }, 0.7, {}),  # Fingertip 22 mm inside the mouth (at least 15 mm) but 12 mm short of the finger stop (more than 5 mm).
    ('seated 4 mm short of the stop', GOAL_POSES, {
        'oximeter': {'pos': (0.27, -0.0859, 0.8245), 'yaw_deg': -90, 'joints': {'top_jaw': -1.5}},
    }, 1, {}),
    ('seated, but the hand was pushed 15 mm across the wedge', GOAL_POSES, {
        'hand': {'pos': (0.285, -0.075, 0.8312)},
        'oximeter': {'pos': (0.285, -0.0822, 0.8231), 'yaw_deg': -90, 'joints': {'top_jaw': -1.5}},
    }, 0.7, {}),  # Seated and clamped, but the hand is more than 10 mm (and under 25 mm) off its mark.
    ('short of the stop and the hand off its mark', GOAL_POSES, {
        'hand': {'pos': (0.285, -0.075, 0.8312)},
        'oximeter': {'pos': (0.285, -0.0935, 0.8272), 'yaw_deg': -90, 'joints': {'top_jaw': -3.0}},
    }, 0.4, {}),
    ('only the first 10 mm of the fingertip in the mouth', GOAL_POSES, {
        'oximeter': {'pos': (0.27, -0.1047, 0.8313), 'yaw_deg': -90, 'joints': {'top_jaw': -3.0}},
    }, 0, {}),
    ('clip shut on the table beside the fingertip', GOAL_POSES, {
        'oximeter': {'pos': (0.27, -0.115, 0.7475), 'yaw_deg': -90, 'joints': {'top_jaw': -20.0}},
    }, 0, {}),
    ('clip on the middle finger instead', GOAL_POSES, {
        'oximeter': {'pos': (0.234, -0.0916, 0.8266), 'yaw_deg': -90, 'joints': {'top_jaw': -1.5}},
    }, 0, {}),
    ('clip seated but still pinched by the right gripper', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': "{'left': (np.array([0.10, 0.25, TOP + 0.20]), 1.0), 'right': (SEAT, 0.25)}"}),
    ('clip seated but the left gripper still holds the wrist', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': "{'left': (STEADY, 0.55), 'right': (np.array([0.20, -0.25, TOP + 0.20]), 1.0)}"}),
    ('jaws held wide open on the finger (squeezed, not clamped)', GOAL_POSES, {
        'oximeter': {'pos': (0.27, -0.0822, 0.8231), 'yaw_deg': -90, 'joints': {'top_jaw': 20.0}},
    }, 0, {}),
    ('display facing down', GOAL_POSES, {
        'oximeter': {'pos': (0.27, -0.0702, 0.856), 'yaw_deg': -90, 'tilt_deg': 140, 'tilt_axis': 'y', 'quat': (-0.6644630243886746, 0.6644630243886748, -0.2418447626479753, -0.2418447626479754), 'joints': {'top_jaw': -1.5}},
    }, 0, {}),
    ('hand dragged 30 mm with the clip on it', GOAL_POSES, {
        'hand': {'pos': (0.3, -0.075, 0.8312)},
        'oximeter': {'pos': (0.3, -0.0822, 0.8231), 'yaw_deg': -90, 'joints': {'top_jaw': -1.5}},
    }, 0, {}),
    ('hand dragged earlier, then put back (sticky)', GOAL_POSES, {
    }, 0, {'TEST_STICKY': "('hand yanked',)"}),
    ('oximeter dropped earlier, then put back (sticky)', GOAL_POSES, {
    }, 0, {'TEST_STICKY': "('oximeter off the table',)"}),
    ('oximeter on the floor', START_POSES, {
        'oximeter': {'pos': (0.3, -0.4, 0.02), 'yaw_deg': -90, 'joints': {'top_jaw': -20.0}},
    }, 0, {}),
    ('seated clip still sliding', GOAL_POSES, {
        'oximeter': {'pos': (0.27, -0.0822, 0.8231), 'yaw_deg': -90, 'vel': (0.0, 0.05, 0.0), 'joints': {'top_jaw': -1.5}},
    }, 0, {}),
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
