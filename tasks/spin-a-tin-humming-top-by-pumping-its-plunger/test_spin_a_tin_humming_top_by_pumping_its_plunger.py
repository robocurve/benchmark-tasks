"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest spin-a-tin-humming-top-by-pumping-its-plunger    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("spin_a_tin_humming_top_by_pumping_its_plunger.py"))
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
    'tray': {'pos': (0.195, 0.05, 0.749)},
    'top': {'pos': (0.12, 0.05, 0.7525), 'joints': {'carrier': 0.0, 'rod': 0.0}},
}
GOAL_POSES = {
    'tray': {'pos': (0.195, 0.05, 0.749)},
    'top': {'pos': (0.12, 0.05, 0.7515), 'joints': {'carrier': 0.0, 'rod': 0.0}},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_TRACK': '{"spun":True,"best":10.,"bad":None}'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'top', 'tray'}
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
    ('untouched top', START_POSES, {
    }, 0, {'TEST_TRACK': 'None'}),
    ('spun but never let go', GOAL_POSES, {
    }, 0.25, {'TEST_TRACK': '{"spun":True,"best":0.,"bad":None}'}),
    ('fell just before two seconds', GOAL_POSES, {
    }, 0.25, {'TEST_TRACK': '{"spun":True,"best":1.95,"bad":None}'}),
    ('exactly two seconds', GOAL_POSES, {
    }, 0.5, {'TEST_TRACK': '{"spun":True,"best":2.,"bad":None}'}),
    ('exactly five seconds', GOAL_POSES, {
    }, 0.75, {'TEST_TRACK': '{"spun":True,"best":5.,"bad":None}'}),
    ('just under ten seconds', GOAL_POSES, {
    }, 0.75, {'TEST_TRACK': '{"spun":True,"best":9.95,"bad":None}'}),
    ('twelve seconds alone then fallen leaning on point and rim', GOAL_POSES, {
        'top': {'pos': (0.12, 0.05, 0.7519), 'tilt_deg': 44, 'tilt_axis': 'x', 'quat': (-0.374606593415912, 0.0, 0.0, 0.9271838545667874), 'joints': {'carrier': 0.0, 'rod': 0.0}},
    }, 1, {'TEST_TRACK': '{"spun":True,"best":12.,"bad":None}'}),  # a top leaning on the tray is still on the table; its bounding box must not trip the off-table rule
    ('twelve seconds alone then lying flat on its side', GOAL_POSES, {
        'top': {'pos': (0.12, 0.05, 0.8145), 'tilt_deg': 90, 'tilt_axis': 'x', 'quat': (-0.7071067811865475, 0.0, 0.0, 0.7071067811865476), 'joints': {'carrier': 0.0, 'rod': 0.0}},
    }, 1, {'TEST_TRACK': '{"spun":True,"best":12.,"bad":None}'}),
    ('rod eight millimetres off its guide line', GOAL_POSES, {
    }, 1, {'TEST_TRACK': '{"spun":True,"best":12.,"bad":None,"rod_out_mm":8.}'}),
    ('stood up and left, never pumped', GOAL_POSES, {
    }, 0, {'TEST_TRACK': '{"spun":False,"best":12.,"bad":None}'}),  # a top that was never spun by its plunger earns nothing, however long it is left alone
    ('twelve seconds alone but the top ended on the floor', GOAL_POSES, {
        'top': {'pos': (0.12, 0.6, 0.4475), 'tilt_deg': 44, 'tilt_axis': 'x', 'quat': (-0.374606593415912, 0.0, 0.0, 0.9271838545667874), 'joints': {'carrier': 0.0, 'rod': 0.0}},
    }, 0, {'TEST_TRACK': '{"spun":True,"best":12.,"bad":None}'}),  # a top knocked off the table overrides the stopwatch
    ('rod pulled twelve millimetres out of its guide', GOAL_POSES, {
    }, 0, {'TEST_TRACK': '{"spun":True,"best":12.,"bad":None,"rod_out_mm":12.}'}),  # a bent or torn-out plunger overrides the stopwatch
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
