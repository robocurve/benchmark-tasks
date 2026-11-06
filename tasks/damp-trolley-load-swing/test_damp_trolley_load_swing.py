"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest damp-trolley-load-swing    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("damp_trolley_load_swing.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 600


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
    'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': 0.0, 'hanger': -12.0}},
}
GOAL_POSES = {
    'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': -0.12, 'hanger': 0.0}},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == set()
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
    ('hanger 1 degrees', GOAL_POSES, {
        'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': -0.12, 'hanger': 1.0}},
    }, 1, {}),
    ('hanger -1.4 degrees', GOAL_POSES, {
        'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': -0.12, 'hanger': -1.4}},
    }, 1, {}),
    ('hanger 3 degrees', GOAL_POSES, {
        'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': -0.12, 'hanger': 3.0}},
    }, 0.5, {}),
    ('hanger -4.5 degrees', GOAL_POSES, {
        'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': -0.12, 'hanger': -4.5}},
    }, 0.5, {}),
    ('hanger 10 degrees', GOAL_POSES, {
        'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': -0.12, 'hanger': 10.0}},
    }, 0.25, {}),
    ('hanger -20 degrees', GOAL_POSES, {
        'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': -0.12, 'hanger': -20.0}},
    }, 0.25, {}),
    ('pointer offset -0.004 m', GOAL_POSES, {
        'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': -0.124, 'hanger': 0.0}},
    }, 1, {}),
    ('pointer offset 0.004 m', GOAL_POSES, {
        'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': -0.116, 'hanger': 0.0}},
    }, 1, {}),
    ('pointer offset 0.006 m', GOAL_POSES, {
        'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': -0.114, 'hanger': 0.0}},
    }, 0, {}),
    ('start', GOAL_POSES, {
        'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': 0.0, 'hanger': -12.0}},
    }, 0, {}),
    ('swing exceeds thirty degrees', GOAL_POSES, {
        'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': -0.12, 'hanger': 35.0}},
    }, 0, {}),
    ('weight against left post', GOAL_POSES, {
        'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': -0.17, 'hanger': 13.221511052}},
    }, 0, {}),  # Weight AABB touches the post at an end stop; below 30 degrees.
    ('weight against right post', GOAL_POSES, {
        'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': 0.05, 'hanger': -13.221511052}},
    }, 0, {}),  # Weight AABB touches the post at an end stop; below 30 degrees.
    ('twenty mm short', GOAL_POSES, {
        'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': -0.1, 'hanger': 0.0}},
    }, 0, {}),
    ('held at mark', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': '{"left": ((MARK_X, KNOB_TCP[1], KNOB_TCP[2]), 0.0), "right": ((0.1, -0.3, TOP + 0.20), 1.0)}'}),
    ('grab the weight', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': '{"left": ((MARK_X, ORIGIN[1], TOP + BOB[2]), 0.0), "right": ((0.1, -0.3, TOP + 0.20), 1.0)}'}),
    ('brace rod thirty mm away', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': '{"left": ((MARK_X, ORIGIN[1] + 0.034, TOP + HANGER[1].center[2]), 0.0), "right": ((0.1, -0.3, TOP + 0.20), 1.0)}'}),
    ('open palm end braces the rod from +x while the TCP is 71 mm away', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': '{"left": ((MARK_X + 0.075, ORIGIN[1], TOP + 0.1525), 1.0), "right": ((0.1, -0.3, TOP + 0.20), 1.0)}'}),  # The palm reaches 71 mm from the tool axis; the hand envelope (80 mm radius) touches the rod box.
    ('open palm end braces the rod from -x while the TCP is 71 mm away', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': '{"left": ((MARK_X - 0.075, ORIGIN[1], TOP + 0.1525), 1.0), "right": ((0.1, -0.3, TOP + 0.20), 1.0)}'}),  # Mirror of the +x brace; a hand on the load.
    ('palm brace while the hanger swings 3 degrees', GOAL_POSES, {
        'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': -0.12, 'hanger': 3.0}},
    }, 0, {'TEST_GRIPPERS': '{"left": ((MARK_X - 0.075, ORIGIN[1], TOP + 0.1525), 1.0), "right": ((0.1, -0.3, TOP + 0.20), 1.0)}'}),  # Any credit here would come from a hand on the load.
    ('hand axis 84 mm from the rod box', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': '{"left": ((MARK_X + 0.004 + 0.084, ORIGIN[1], TOP + 0.20), 1.0), "right": ((0.1, -0.3, TOP + 0.20), 1.0)}'}),  # Inside the 80 mm hand radius plus 5 mm clearance.
    ('hand axis 86 mm from the rod box', GOAL_POSES, {
    }, 1, {'TEST_GRIPPERS': '{"left": ((MARK_X + 0.004 + 0.086, ORIGIN[1], TOP + 0.20), 1.0), "right": ((0.1, -0.3, TOP + 0.20), 1.0)}'}),  # Just outside the 85 mm hand limit, released and still; full credit.
    ('weight 23.5 degrees toward the left post at the mark', GOAL_POSES, {
        'monorail': {'pos': (0.32, 0.12, 0.7475), 'joints': {'trolley': -0.12, 'hanger': 23.5}},
    }, 0.25, {}),  # The weight box clears the left post by about 2 mm; a legitimate swinging state at the mark.
    ('TCP 31 mm from trolley box', GOAL_POSES, {
    }, 1, {'TEST_GRIPPERS': '{"left": ((MARK_X, KNOB_TCP[1] + 0.010 + 0.031, KNOB_TCP[2]), 0.0), "right": ((0.1, -0.3, TOP + 0.20), 1.0)}'}),
    ('TCP 29 mm from trolley box', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': '{"left": ((MARK_X, KNOB_TCP[1] + 0.010 + 0.029, KNOB_TCP[2]), 0.0), "right": ((0.1, -0.3, TOP + 0.20), 1.0)}'}),
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
