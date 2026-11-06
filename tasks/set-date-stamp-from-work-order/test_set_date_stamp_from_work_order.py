"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest set-date-stamp-from-work-order    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("set_date_stamp_from_work_order.py"))
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
    'stamp': {'pos': (0.275, -0.417, 0.7615), 'joints': {'day': 0.0, 'day_plunger': 0.0, 'month': 0.0, 'month_plunger': 0.0, 'face': 0.0}},
}
GOAL_POSES = {
    'stamp': {'pos': (0.275, -0.417, 0.7605), 'joints': {'day': 0.0, 'day_plunger': 0.0, 'month': 0.0, 'month_plunger': 0.0, 'face': 0.0}},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': '[nominal_event(0)]', 'TEST_INKED': 'True'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'stamp'}
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
    ('both bands set no impression', GOAL_POSES, {
        'stamp': {'pos': (0.275, -0.417, 0.7605), 'joints': {'day': 23.225806451612904, 'day_plunger': 0.0, 'month': 29.999999999999996, 'month_plunger': 0.0, 'face': 0.0}},
    }, 0.3, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': '[]', 'TEST_INKED': 'False'}),
    ('day band set only', GOAL_POSES, {
        'stamp': {'pos': (0.275, -0.417, 0.7605), 'joints': {'day': 23.225806451612904, 'day_plunger': 0.0, 'month': 0.0, 'month_plunger': 0.0, 'face': 0.0}},
    }, 0.15, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': '[]', 'TEST_INKED': 'False'}),
    ('month band set only', GOAL_POSES, {
        'stamp': {'pos': (0.275, -0.417, 0.7605), 'joints': {'day': 0.0, 'day_plunger': 0.0, 'month': 29.999999999999996, 'month_plunger': 0.0, 'face': 0.0}},
    }, 0.15, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': '[]', 'TEST_INKED': 'False'}),
    ('zero-month layout 1 day set so both bands correct', GOAL_POSES, {
        'stamp': {'pos': (0.275, -0.417, 0.7605), 'joints': {'day': 34.83870967741935, 'day_plunger': 0.0, 'month': 0.0, 'month_plunger': 0.0, 'face': 0.0}},
    }, 0.3, {'TEST_LAYOUT': 'LAYOUTS[1]', 'TEST_EVENTS': '[]', 'TEST_INKED': 'False'}),
    ('zero-month layout 14 day set so both bands correct', GOAL_POSES, {
        'stamp': {'pos': (0.275, -0.417, 0.7605), 'joints': {'day': -23.225806451612904, 'day_plunger': 0.0, 'month': 0.0, 'month_plunger': 0.0, 'face': 0.0}},
    }, 0.3, {'TEST_LAYOUT': 'LAYOUTS[14]', 'TEST_EVENTS': '[]', 'TEST_INKED': 'False'}),
    ('zero-month layout 1 day set but month knocked one detent', GOAL_POSES, {
        'stamp': {'pos': (0.275, -0.417, 0.7605), 'joints': {'day': 34.83870967741935, 'day_plunger': 0.0, 'month': 29.999999999999996, 'month_plunger': 0.0, 'face': 0.0}},
    }, 0.15, {'TEST_LAYOUT': 'LAYOUTS[1]', 'TEST_EVENTS': '[]', 'TEST_INKED': 'False'}),
    ('correct impression stamp lying on table', GOAL_POSES, {
        'stamp': {'pos': (0.225, -0.15, 0.8075), 'tilt_deg': 90, 'tilt_axis': 'y', 'joints': {'day': 0.0, 'day_plunger': 0.0, 'month': 0.0, 'month_plunger': 0.0, 'face': 0.0}},
    }, 0.7, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': '[nominal_event(0)]', 'TEST_INKED': 'True'}),
    ('correct impression seated but moving', GOAL_POSES, {
        'stamp': {'pos': (0.275, -0.417, 0.7605), 'vel': (0.2, 0.0, 0.0), 'joints': {'day': 0.0, 'day_plunger': 0.0, 'month': 0.0, 'month_plunger': 0.0, 'face': 0.0}},
    }, 0.9, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': '[nominal_event(0)]', 'TEST_INKED': 'True'}),
    ('correct impression seated released knob 2 degrees off detent', GOAL_POSES, {
        'stamp': {'pos': (0.275, -0.417, 0.7605), 'joints': {'day': 25.225806451612904, 'day_plunger': 0.0, 'month': 29.999999999999996, 'month_plunger': 0.0, 'face': 0.0}},
    }, 0.9, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': '[nominal_event(0)]', 'TEST_INKED': 'True'}),
    ('correct impression seated 6 mm off centre', GOAL_POSES, {
        'stamp': {'pos': (0.281, -0.417, 0.7605), 'joints': {'day': 23.225806451612904, 'day_plunger': 0.0, 'month': 29.999999999999996, 'month_plunger': 0.0, 'face': 0.0}},
    }, 1, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': '[nominal_event(0)]', 'TEST_INKED': 'True'}),
    ('wrong day', GOAL_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': 'cheat_cases()[0][1]', 'TEST_INKED': 'True'}),  # The wrong day is a permanent record error.
    ('wrong month', GOAL_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': 'cheat_cases()[1][1]', 'TEST_INKED': 'True'}),  # The wrong month is a permanent record error.
    ('start date then correct', GOAL_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': 'cheat_cases()[2][1]', 'TEST_INKED': 'True'}),  # A correction cannot erase the first wrong date.
    ('two correct impressions', GOAL_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': 'cheat_cases()[3][1]', 'TEST_INKED': 'True'}),  # Exactly one impression is allowed.
    ('glyph 3 mm from box line', GOAL_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': 'cheat_cases()[4][1]', 'TEST_INKED': 'True'}),  # The required line clearance is 5 mm.
    ('rotated 8 deg', GOAL_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': 'cheat_cases()[5][1]', 'TEST_INKED': 'True'}),  # Yaw exceeds 5 degrees.
    ('on table beside sheet', GOAL_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': 'cheat_cases()[6][1]', 'TEST_INKED': 'True'}),  # A table mark is permanent and off the work order.
    ('blank day 6 deg off detent', GOAL_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': 'cheat_cases()[7][1]', 'TEST_INKED': 'True'}),  # More than 4 degrees off detent makes a band blank.
    ('correct then later table press', GOAL_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': 'cheat_cases()[8][1]', 'TEST_INKED': 'True'}),  # A later table mark remains a violation after return.
    ('tilted 6 deg', GOAL_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': 'cheat_cases()[9][1]', 'TEST_INKED': 'True'}),  # Tilt exceeds 5 degrees.
    ('zero-month layout 1 day knocked 6 degrees so it reads blank', GOAL_POSES, {
        'stamp': {'pos': (0.275, -0.417, 0.7605), 'joints': {'day': 6.0, 'day_plunger': 0.0, 'month': 0.0, 'month_plunger': 0.0, 'face': 0.0}},
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[1]', 'TEST_EVENTS': '[]', 'TEST_INKED': 'False'}),  # The month needed no change, so it earns nothing until the day is right too.
    ('zero-month layout 7 day turned the wrong way', GOAL_POSES, {
        'stamp': {'pos': (0.275, -0.417, 0.7605), 'joints': {'day': -34.83870967741935, 'day_plunger': 0.0, 'month': 0.0, 'month_plunger': 0.0, 'face': 0.0}},
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[7]', 'TEST_EVENTS': '[]', 'TEST_INKED': 'False'}),  # A wrong day is not progress, and the unchanged month earns nothing on its own.
    ('zero-month layout 15 untouched start', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[15]', 'TEST_EVENTS': '[]', 'TEST_INKED': 'False'}),  # The untouched start scores zero in every layout.
    ('correct impression then smeared across the sheet', GOAL_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': '[nominal_event(0)]', 'TEST_INKED': 'True', 'TEST_SMEARED': 'True'}),  # Sliding or twisting the inked face after the impression smears the record.
    ('correct impression seated but still gripped at the handle', GOAL_POSES, {
    }, 0.9, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': '[nominal_event(0)]', 'TEST_INKED': 'True', 'TEST_GRIPPERS': "{'right': (np.r_[CRADLE_XY, SEAT_ROOT] + HANDLE, 0.0)}"}),  # Full score needs the stamp released, every TCP at least 30 mm from it.
    ('stamp pushed off the table with no impression', START_POSES, {
        'stamp': {'pos': (0.8, -0.8, 0.6475), 'joints': {'day': 0.0, 'day_plunger': 0.0, 'month': 0.0, 'month_plunger': 0.0, 'face': 0.0}},
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': '[]', 'TEST_INKED': 'False'}),  # Moving the stamp earns nothing without set bands or an impression.
    ('stamp toppled in the cradle with no impression', START_POSES, {
        'stamp': {'pos': (0.275, -0.417, 0.7785), 'tilt_deg': 90, 'tilt_axis': 'x', 'joints': {'day': 0.0, 'day_plunger': 0.0, 'month': 0.0, 'month_plunger': 0.0, 'face': 0.0}},
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_EVENTS': '[]', 'TEST_INKED': 'False'}),  # A toppled stamp with its bands at the start setting earns nothing.
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
