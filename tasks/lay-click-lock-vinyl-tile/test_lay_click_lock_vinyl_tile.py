"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest lay-click-lock-vinyl-tile    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("lay_click_lock_vinyl_tile.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 1600


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
    'tile': {'pos': (0.3031, 0.01, 0.773)},
}
GOAL_POSES = {
    'tile': {'pos': (0.31, 0, 0.7475), 'tilt_deg': 9.262, 'tilt_axis': 'y', 'quat': (0.0, -0.08074131071287058, 0.0, 0.9967350905553429)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'tile'}
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
    ('untouched start (tongue on the lip, tipped on the batten)', START_POSES, {
    }, 0, {}),
    ('locked, end 2 mm off the spacer (end gap 11.5 mm)', GOAL_POSES, {
        'tile': {'pos': (0.31, 0.002, 0.7475), 'tilt_deg': 9.262, 'tilt_axis': 'y', 'quat': (0.0, -0.08074131071287058, 0.0, 0.9967350905553429)},
    }, 1, {}),
    ('locked but left 25 mm from the wall (not slid to the spacer)', GOAL_POSES, {
        'tile': {'pos': (0.31, 0.0155, 0.7475), 'tilt_deg': 9.262, 'tilt_axis': 'y', 'quat': (0.0, -0.08074131071287058, 0.0, 0.9967350905553429)},
    }, 0.75, {}),  # Locked along the whole edge, but the end gap is 25 mm (over 12, up to 40).
    ('racked, locked at the wall end only, the other end riding on the bump', GOAL_POSES, {
        'tile': {'pos': (0.307, -0.0015, 0.7479), 'yaw_deg': 0.57, 'tilt_deg': 9.398, 'tilt_axis': 'y', 'quat': (0.0016265053689867965, -0.08190812859343742, 0.005055958129203528, 0.9966257322776697)},
    }, 0.5, {}),  # One end has seam and lippage within the lock limits; the other end is 6 mm open and 1.4 mm high.
    ('lowered before the tongue was in, riding on the bump along its whole length', GOAL_POSES, {
        'tile': {'pos': (0.306, 0, 0.7482), 'tilt_deg': 9.532, 'tilt_axis': 'y', 'quat': (0.0, -0.08309027416744935, 0.0, 0.996542024371666)},
    }, 0, {}),  # Lying down against the row but raised 1.4 mm at the seam and 4 mm open; not locked anywhere.
    ('locked but the right gripper still pinches the near edge', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': "{'left': (np.array([0.10, 0.25, TOP + 0.15]), 1.0), 'right': (np.array([X_SEAM - TILE_D + 0.001, -0.18, TOP + 0.0025]), 0.0)}"}),
    ('tongue home but held tipped up 20 degrees', GOAL_POSES, {
        'tile': {'pos': (0.3173, 0, 0.7991), 'tilt_deg': 10.738, 'tilt_axis': 'y', 'quat': (0.0, 0.0935665633132155, 0.0, 0.9956130263459564)},
    }, 0, {}),
    ('hovering 5 mm above the locked pose', GOAL_POSES, {
        'tile': {'pos': (0.31, 0, 0.7525), 'tilt_deg': 9.262, 'tilt_axis': 'y', 'quat': (0.0, -0.08074131071287058, 0.0, 0.9967350905553429)},
    }, 0, {}),
    ("lying on top of the laid row's edge", GOAL_POSES, {
        'tile': {'pos': (0.36, 0, 0.7525), 'tilt_deg': 9.262, 'tilt_axis': 'y', 'quat': (0.0, -0.08074131071287058, 0.0, 0.9967350905553429)},
    }, 0, {}),
    ('lying flat 30 mm short of the row', GOAL_POSES, {
        'tile': {'pos': (0.28, 0, 0.7475), 'tilt_deg': 9.262, 'tilt_axis': 'y', 'quat': (0.0, -0.08074131071287058, 0.0, 0.9967350905553429)},
    }, 0, {}),
    ('locked but still sliding', GOAL_POSES, {
        'tile': {'pos': (0.31, 0, 0.7475), 'tilt_deg': 9.262, 'tilt_axis': 'y', 'vel': (0.0, 0.05, 0.0), 'quat': (0.0, -0.08074131071287058, 0.0, 0.9967350905553429)},
    }, 0, {}),
    ('dropped off the table earlier, then put back (sticky)', GOAL_POSES, {
    }, 0, {'TEST_STICKY': "('tile off the table',)"}),
    ('tile on the floor beside the table', START_POSES, {
        'tile': {'pos': (0.3, -0.8, 0.02)},
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
