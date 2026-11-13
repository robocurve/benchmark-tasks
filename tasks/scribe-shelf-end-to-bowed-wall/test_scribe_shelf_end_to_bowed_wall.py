"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest scribe-shelf-end-to-bowed-wall    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("scribe_shelf_end_to_bowed_wall.py"))
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
    'shelf': {'pos': (0.19, 0.058, 0.7485)},
    'compass': {'pos': (0.08, 0.11, 0.886)},
}
GOAL_POSES = {
    'shelf': {'pos': (0.19, 0.058, 0.7475)},
    'compass': {'pos': (0.08, 0.11, 0.885)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_STROKES': 'nominal_strokes(0)'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'compass', 'shelf'}
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
    ('line on target, compass still in the hand', GOAL_POSES, {
    }, 0.75, {'TEST_STROKES': 'nominal_strokes(0)', 'TEST_GRIPPERS': "{'left': (HANG_ORIGIN + TCP_LOCAL, 0.0), 'right': (np.array([0.20, -0.25, TOP + 0.20]), 1.0)}"}),
    ('line on target, hand still pressing the shelf', GOAL_POSES, {
    }, 0.75, {'TEST_STROKES': 'nominal_strokes(0)', 'TEST_GRIPPERS': "{'left': (np.array([0.13, 0.11, TOP + 0.15]), 1.0), 'right': (np.array([LAYOUT_FRONT_X[0] + HOLD_FROM_FRONT, HOLD_Y, SHELF_TOP + PAD_TIP_PAST_TCP - HOLD_PRESS]), 0.0)}"}),
    ('line on target, compass left lying on the shelf', GOAL_POSES, {
        'compass': {'pos': (0.2, 0, 0.7765), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0.75, {'TEST_STROKES': 'nominal_strokes(0)'}),
    ('line on target, compass still swinging on the stand', GOAL_POSES, {
        'compass': {'pos': (0.08, 0.11, 0.885), 'vel': (0.0, 0.05, 0.0)},
    }, 0.75, {'TEST_STROKES': 'nominal_strokes(0)'}),
    ("line 0.4 mm toward the board's end, full credit", GOAL_POSES, {
    }, 1, {'TEST_STROKES': 'nominal_strokes(0, -0.0004)'}),
    ('two passes 0.6 mm apart, read at their centre', GOAL_POSES, {
    }, 1, {'TEST_STROKES': 'nominal_strokes(0, -0.0003) + nominal_strokes(0, 0.0003)'}),
    ('line 0.8 mm off (shelf held 0.8 mm off the wall)', GOAL_POSES, {
    }, 0.5, {'TEST_STROKES': 'nominal_strokes(0, 0.0008)'}),
    ('line 1.5 mm off', GOAL_POSES, {
    }, 0.25, {'TEST_STROKES': 'nominal_strokes(0, 0.0015)'}),
    ('layout 3, its own line', GOAL_POSES, {
        'shelf': {'pos': (0.271, 0.0584, 0.7475)},
    }, 1, {'TEST_LAYOUT': '3', 'TEST_STROKES': 'nominal_strokes(3)'}),
    ('only the front half scribed', GOAL_POSES, {
    }, 0, {'TEST_STROKES': '[ideal_line(0, 0.0, 0.002, 0.070)]'}),
    ('line on target with a 1.9 mm skip between stations 57 and 67, still one line', GOAL_POSES, {
    }, 1, {'TEST_STROKES': 'broken_strokes(0, 0.0019)'}),
    ('line on target with a 2.1 mm skip between stations 57 and 67, broken', GOAL_POSES, {
    }, 0, {'TEST_STROKES': 'broken_strokes(0, 0.0021)'}),
    ('line on target starting 2.5 mm in from the front edge', GOAL_POSES, {
    }, 0, {'TEST_STROKES': '[ideal_line(0, 0.0, 0.0025, SHELF_D)]'}),
    ('line on target stopping 2.5 mm short of the back edge', GOAL_POSES, {
    }, 0, {'TEST_STROKES': '[ideal_line(0, 0.0, 0.0, SHELF_D - 0.0025)]'}),
    ('continuous line 2.5 mm off', GOAL_POSES, {
    }, 0, {'TEST_STROKES': 'nominal_strokes(0, 0.0025)'}),
    ('single touches on target at the 14 stations, nothing between', GOAL_POSES, {
    }, 0, {'TEST_STROKES': 'station_dots(0)'}),  # the touches are 7 to 10 mm apart, not one line; line credit needs no gap over 2 mm from edge to edge
    ('station touches on target joined by dashes 3 mm off the line', GOAL_POSES, {
    }, 0, {'TEST_STROKES': 'dashes_and_dots(0, 0.003)'}),  # the marks read at the stations are 3 mm from the rest of the line, so the line read is not continuous
    ('best straight line through the targets (ruler, no wall)', GOAL_POSES, {
    }, 0, {'TEST_STROKES': 'straight_strokes(0)'}),  # misses the wall's bend at the gate stations
    ("layout 1's line drawn on layout 0 (memorised path)", GOAL_POSES, {
    }, 0, {'TEST_STROKES': 'memorised_strokes(1, 0)'}),  # each seed lays the shelf against a different stretch of wall
    ("layout 0's line drawn on layout 3's shelf (memorised path)", GOAL_POSES, {
        'shelf': {'pos': (0.271, 0.0584, 0.7475)},
    }, 0, {'TEST_LAYOUT': '3', 'TEST_STROKES': 'memorised_strokes(0, 3)'}),  # each seed lays the shelf against a different stretch of wall
    ('two lines 1.5 mm apart (hedging)', GOAL_POSES, {
    }, 0, {'TEST_STROKES': 'nominal_strokes(0) + nominal_strokes(0, 0.0015)'}),  # crossings more than 1 mm apart leave a station unmarked
    ('scribbled band across the targets', GOAL_POSES, {
    }, 0, {'TEST_STROKES': '[ideal_line(0, b) for b in (-0.003, -0.0015, 0.0, 0.0015, 0.003)]'}),  # crossings more than 1 mm apart leave a station unmarked
    ('line on target, shelf then shoved 35 mm along the wall', GOAL_POSES, {
        'shelf': {'pos': (0.225, 0.058, 0.7475)},
    }, 0, {'TEST_STROKES': 'nominal_strokes(0)'}),  # moving the board more than 30 mm is a sticky failure
    ('line on target, shelf knocked off the table', GOAL_POSES, {
        'shelf': {'pos': (0.3, -0.4, 0.6475)},
    }, 0, {'TEST_STROKES': 'nominal_strokes(0)'}),  # sticky failure
    ('line on target, compass dropped off the table', GOAL_POSES, {
        'compass': {'pos': (0.05, 0.4, 0.6475)},
    }, 0, {'TEST_STROKES': 'nominal_strokes(0)'}),  # sticky failure
    ('no line, compass hung back', GOAL_POSES, {
    }, 0, {'TEST_STROKES': '[]'}),  # nothing scribed
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
