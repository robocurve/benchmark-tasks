"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest find-stud-with-magnetic-stud-finder    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("find_stud_with_magnetic_stud_finder.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 3600


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
    'finder': {'pos': (0.19, 0.3, 0.803)},
}
GOAL_POSES = {
    'finder': {'pos': (0.255, 0.19, 0.885)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'finder'}
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
    ('on the screw, still pinched by the left hand', GOAL_POSES, {
    }, 0.5, {'TEST_GRIPPERS': "{'left': (on_wall_origin(0.190, 0.1125) + TCP_IN_F, 0.0), 'right': (np.array([0.32, -0.305, TOP + 0.26]), 1.0)}"}),
    ('on the screw, let go, still sliding down', GOAL_POSES, {
        'finder': {'pos': (0.255, 0.19, 0.885), 'vel': (0.0, 0.0, -0.02)},
    }, 0.5, {}),
    ('held flat on the wall 40 mm left of the screw', GOAL_POSES, {
        'finder': {'pos': (0.255, 0.23, 0.8865)},
    }, 0.25, {'TEST_GRIPPERS': "{'left': (on_wall_origin(0.230, 0.114) + TCP_IN_F, 0.0), 'right': (np.array([0.32, -0.305, TOP + 0.26]), 1.0)}"}),
    ('held flat on the wall in the right row but 12 mm short of the screw', GOAL_POSES, {
        'finder': {'pos': (0.255, 0.178, 0.8865)},
    }, 0.25, {'TEST_GRIPPERS': "{'left': (on_wall_origin(0.178, 0.114) + TCP_IN_F, 0.0), 'right': (np.array([0.32, -0.305, TOP + 0.26]), 1.0)}"}),
    ("hanging on the right stud's screw", GOAL_POSES, {
        'finder': {'pos': (0.255, -0.2164, 0.885)},
    }, 1, {}),
    ('hanging by its upper magnet', GOAL_POSES, {
        'finder': {'pos': (0.255, 0.19, 0.835)},
    }, 1, {}),
    ('hanging 8 mm below the screw and 4 mm to one side', GOAL_POSES, {
        'finder': {'pos': (0.255, 0.194, 0.8785)},
    }, 1, {}),
    ('back pressed 1.5 mm into the paint over the screw', GOAL_POSES, {
        'finder': {'pos': (0.2565, 0.19, 0.885)},
    }, 1, {}),
    ("hung where layout 0's screw is, in a trial that drew layout 1 (a remembered spot)", GOAL_POSES, {
    }, 0, {'TEST_LAYOUT': '1'}),  # the screws move between layouts; nothing holds the finder there
    ('let go flat on the wall away from any screw', GOAL_POSES, {
        'finder': {'pos': (0.255, 0.26, 0.8865)},
    }, 0, {}),  # with no screw under a magnet the finder falls
    ("11 mm off the screw's axis, let go", GOAL_POSES, {
        'finder': {'pos': (0.255, 0.19, 0.8755)},
    }, 0, {}),  # outside the magnets' reach
    ('held 4 mm off the wall over the screw', GOAL_POSES, {
        'finder': {'pos': (0.251, 0.19, 0.885)},
    }, 0, {'TEST_GRIPPERS': "{'left': (on_wall_origin(0.190, 0.1125, 1, 0.004) + TCP_IN_F, 0.0), 'right': (np.array([0.32, -0.305, TOP + 0.26]), 1.0)}"}),  # lifted off the wall, not flat on it
    ('tilted 6 degrees on the wall over the screw', GOAL_POSES, {
        'finder': {'pos': (0.255, 0.19, 0.885), 'tilt_deg': 6, 'tilt_axis': 'y'},
    }, 0, {}),  # its top stands off the wall
    ('standing on the table with its back against the wall below the left screw', START_POSES, {
        'finder': {'pos': (0.255, 0.19, 0.797)},
    }, 0, {}),  # leaning on the wall, not hanging
    ("lying on the wall's cap above the left screw", START_POSES, {
        'finder': {'pos': (0.305, 0.19, 0.978), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0, {}),  # resting on top of the wall, not on its face
    ('back in its holder', START_POSES, {
    }, 0, {}),  # never placed
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
