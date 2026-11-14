"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest fit-lever-set-to-bored-door    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("fit_lever_set_to_bored_door.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 7200


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


def place(name, pos, yaw_deg=0.0, tilt_deg=0.0, tilt_axis="x", vel=(0.0, 0.0, 0.0), quat=None):
    """The ObjectState of `name` with its centre at `pos`, turned and tipped; its bounding box follows the turn."""
    r = (_rot(yaw_deg, tilt_deg, tilt_axis) if quat is None else
         ik.quat_xyzw_to_matrix(np.asarray(quat, float) / np.linalg.norm(quat)))
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
    'outside': {'pos': (0.34, -0.061, 0.8675)},
    'inside': {'pos': (0.34, 0.08, 0.8675)},
    'driver': {'pos': (0.225, 0.035, 0.8075)},
}
GOAL_POSES = {
    'outside': {'pos': (0.29, -0.0175, 0.8675)},
    'inside': {'pos': (0.29, 0.0175, 0.8675)},
    'driver': {'pos': (0.225, 0.035, 0.8075)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_TURNS': '(2.0, 2.0)', 'TEST_GRIPPERS': 'None'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'driver', 'inside', 'outside'}
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
    ('untouched', START_POSES, {
    }, 0, {}),  # Both halves still hang on their holders.
    ('outside half through the latch only', START_POSES, {
        'outside': {'pos': (0.29, -0.0175, 0.8675)},
    }, 0.25, {}),  # The spindle and posts are through the latch with the rose on the door; nothing else.
    ('both halves on, no screw caught', GOAL_POSES, {
    }, 0.25, {'TEST_TURNS': '(0.0, 0.0)'}),  # Halves on but loose: only the outside half's fit counts.
    ('one screw caught', GOAL_POSES, {
    }, 0.5, {'TEST_TURNS': '(1.0, 0.0)'}),  # Both halves on and one screw on its thread.
    ('one screw snug', GOAL_POSES, {
    }, 0.75, {'TEST_TURNS': '(2.0, 0.5)'}),  # Level and seated, but only one screw snug.
    ('locked 0.8 degrees off level', GOAL_POSES, {
        'outside': {'pos': (0.29, -0.0175, 0.8675), 'tilt_deg': 0.8, 'tilt_axis': 'y', 'quat': (0.0, -0.006981260297961554, 0.0, 0.9999756307053947)},
        'inside': {'pos': (0.29, 0.0175, 0.8675), 'tilt_deg': 0.8, 'tilt_axis': 'y', 'quat': (0.0, -0.006981260297961554, 0.0, 0.9999756307053947)},
    }, 1, {'TEST_TURNS': '(2.0, 2.0)', 'TEST_GRIPPERS': 'None'}),  # Within the 1.0 degree full-credit band.
    ('locked drooping 2.2 degrees', GOAL_POSES, {
        'outside': {'pos': (0.29, -0.0175, 0.8675), 'tilt_deg': 2.2, 'tilt_axis': 'y', 'quat': (0.0, -0.019197442399689676, 0.0, 0.9998157121216442)},
        'inside': {'pos': (0.29, 0.0175, 0.8675), 'tilt_deg': 2.2, 'tilt_axis': 'y', 'quat': (0.0, -0.019197442399689676, 0.0, 0.9998157121216442)},
    }, 0.75, {'TEST_TURNS': '(2.0, 2.0)', 'TEST_GRIPPERS': 'None'}),  # Screwed up crooked, within 3 degrees: the unheld lever's droop in its posts' play.
    ('locked 3.5 degrees off level', GOAL_POSES, {
        'outside': {'pos': (0.29, -0.0175, 0.8675), 'tilt_deg': 3.5, 'tilt_axis': 'y'},
        'inside': {'pos': (0.29, 0.0175, 0.8675), 'tilt_deg': 3.5, 'tilt_axis': 'y'},
    }, 0.5, {'TEST_TURNS': '(2.0, 2.0)', 'TEST_GRIPPERS': 'None'}),  # Screwed up crooked beyond 3 degrees.
    ('outside rose 1 mm off the door', GOAL_POSES, {
        'outside': {'pos': (0.29, -0.0185, 0.8675)},
    }, 0.5, {'TEST_TURNS': '(2.0, 2.0)', 'TEST_GRIPPERS': 'None'}),  # Not seated: the rose's rim stands 1 mm off the face (the base too).
    ('outside half upside down, lever toward the latch edge', GOAL_POSES, {
        'outside': {'pos': (0.29, -0.0175, 0.8675), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'y', 'quat': (0.0, 1.0, 0.0, 6.123233995736766e-17)},
        'inside': {'pos': (0.29, 0.0175, 0.8675), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'y', 'quat': (0.0, 1.0, 0.0, 6.123233995736766e-17)},
    }, 0, {'TEST_TURNS': '(2.0, 2.0)', 'TEST_GRIPPERS': 'None'}),  # The lever must point away from the latch edge.
    ('outside half beside the bore, not through', GOAL_POSES, {
        'outside': {'pos': (0.25, -0.0575, 0.8675)},
    }, 0, {'TEST_TURNS': '(2.0, 2.0)', 'TEST_GRIPPERS': 'None'}),  # Near the door is not through the latch.
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
