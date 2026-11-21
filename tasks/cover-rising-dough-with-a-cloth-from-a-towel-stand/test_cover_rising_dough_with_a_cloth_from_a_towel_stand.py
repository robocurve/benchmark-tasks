"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest cover-rising-dough-with-a-cloth-from-a-towel-stand    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("cover_rising_dough_with_a_cloth_from_a_towel_stand.py"))
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
    if hasattr(o, "extent"):
        return np.full(3, o.extent / 2)
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
    out = {o.name: place(o.name, o.pos if hasattr(o, "pos") else
                             (*o.xy, TASK.table_top + 0.001 + o.extent / 2),
                         math.degrees(getattr(o, "yaw", 0.0) or 0.0))
           for o in T.OBJECTS if not o.free}
    out.update({name: place(name, **p) for name, p in poses.items()})
    return out


START_POSES = {
    'bowl': {'pos': (0.15, 0, 0.7475)},
}
GOAL_POSES = {
    'bowl': {'pos': (0.15, 0, 0.7475)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_CLOTH': 'test_drape()', 'TEST_GRIPPERS': '{"left": (np.array([0.02, 0.14, TOP + 0.11]), 0.4), "right": (np.array([0.02, -0.14, TOP + 0.11]), 0.4)}'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'bowl'}
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
    ('untouched start, the cloth hanging on the stand', START_POSES, {
    }, 0, {}),
    ('laid 3 cm toward the robot (shortest overhang 2.7 cm)', GOAL_POSES, {
    }, 1, {'TEST_CLOTH': 'test_drape(0.03)'}),
    ('laid 4.5 cm off centre (shortest overhang 1.2 cm)', GOAL_POSES, {
    }, 0.75, {'TEST_CLOTH': 'test_drape(0.045)'}),
    ('laid 5.4 cm off centre (shortest overhang 3 mm)', GOAL_POSES, {
    }, 0.5, {'TEST_CLOTH': 'test_drape(0.054)'}),
    ('laid 6.5 cm off centre, 88 % of the rim covered', GOAL_POSES, {
    }, 0.5, {'TEST_CLOTH': 'test_drape(0.065)'}),
    ('laid 9 cm off centre, 68 % of the rim covered', GOAL_POSES, {
    }, 0.25, {'TEST_CLOTH': 'test_drape(0.09)'}),
    ('sagging 18 mm below the rim, clear of the dough', GOAL_POSES, {
    }, 0.75, {'TEST_CLOTH': 'test_drape(sag=0.020)'}),
    ('sagging onto the dough', GOAL_POSES, {
    }, 0.5, {'TEST_CLOTH': 'test_drape(sag=0.026)'}),
    ("a corner still over the stand's bar", GOAL_POSES, {
    }, 0.25, {'TEST_CLOTH': 'test_drape(corner_on_bar=True)'}),
    ('laid well but still held by the left hand', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': '{"left": (test_drape()[0], 0.0), "right": (np.array([0.02, -0.14, TOP + 0.11]), 0.4)}'}),  # the cloth must be let go
    ('cloth held 3 cm above the rim', GOAL_POSES, {
    }, 0, {'TEST_CLOTH': 'test_drape(lift=0.03)'}),  # hanging over the bowl is not covering it
    ('bowl knocked over under the cloth', GOAL_POSES, {
        'bowl': {'pos': (0.15, 0, 0.7475), 'tilt_deg': 30, 'tilt_axis': 'x'},
    }, 0, {}),  # the bowl tipped
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
