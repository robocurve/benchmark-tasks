"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest fit-air-shroud-in-rack-server    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("fit_air_shroud_in_rack_server.py"))
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
    'shroud': {'pos': (-0.045, 0, 0.8343)},
}
GOAL_POSES = {
    'shroud': {'pos': (0.228, 0, 0.7485)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'shroud'}
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
    ("untouched start, resting on the drive cage's top", START_POSES, {
    }, 0, {}),
    ('seated with both tabs in, still pinched in both hands', GOAL_POSES, {
    }, 0.5, {'TEST_GRIPPERS': '{"left": (SEATED + GRIP_LOCAL["left"], 0.0), "right": (SEATED + GRIP_LOCAL["right"], 0.0)}'}),
    ('seated with both tabs in, hands opened but only 20 mm above the walls', GOAL_POSES, {
    }, 0.5, {'TEST_GRIPPERS': '{"left": (SEATED + GRIP_LOCAL["left"] + np.array([0, 0, 0.033]), 1.0), "right": (SEATED + GRIP_LOCAL["right"] + np.array([0, 0, 0.033]), 1.0)}'}),
    ('seated with both tabs in, hands 60 mm above the walls', GOAL_POSES, {
    }, 1, {'TEST_GRIPPERS': '{"left": (SEATED + GRIP_LOCAL["left"] + np.array([0, 0, 0.060]), 1.0), "right": (SEATED + GRIP_LOCAL["right"] + np.array([0, 0, 0.060]), 1.0)}'}),
    ('seated with both tabs in, still settling at 20 mm/s', GOAL_POSES, {
        'shroud': {'pos': (0.228, 0, 0.7485), 'vel': (0.0, 0.0, -0.02)},
    }, 0.5, {}),
    ('every end-wall corner 1.5 mm above the floor pan (a 2 mm gauge does not pass)', GOAL_POSES, {
        'shroud': {'pos': (0.228, 0, 0.75)},
    }, 1, {}),
    ('both tabs in, the back propped 6 mm up (resting on something under the back edge)', GOAL_POSES, {
        'shroud': {'pos': (0.2307, 0, 0.7517), 'tilt_deg': 2, 'tilt_axis': 'y', 'quat': (0.0, -0.01745240643728351, 0.0, 0.9998476951563913)},
    }, 0.25, {}),
    ('both tabs in, the back propped, still pinched in both hands', GOAL_POSES, {
        'shroud': {'pos': (0.2307, 0, 0.7517), 'tilt_deg': 2, 'tilt_axis': 'y', 'quat': (0.0, -0.01745240643728351, 0.0, 0.9998476951563913)},
    }, 0, {'TEST_GRIPPERS': '{"left": (tipped(2.0)[0] + GRIP_LOCAL["left"] + np.array([0, 0, 0.006]), 0.0), "right": (tipped(2.0)[0] + GRIP_LOCAL["right"] + np.array([0, 0, 0.006]), 0.0)}'}),
    ('lowered 4 mm short of the cage (tab tips 2 mm past its rear face)', GOAL_POSES, {
        'shroud': {'pos': (0.232, 0, 0.7485)},
    }, 0, {}),
    ('held in both hands, tilted, tabs in the slots, back 24 mm up', GOAL_POSES, {
        'shroud': {'pos': (0.238, 0, 0.7617), 'tilt_deg': 8, 'tilt_axis': 'y', 'quat': (0.0, -0.06975647374412529, 0.0, 0.9975640502598243)},
    }, 0, {'TEST_GRIPPERS': '{"left": (tipped(8.0)[0] + GRIP_LOCAL["left"], 0.0), "right": (tipped(8.0)[0] + GRIP_LOCAL["right"], 0.0)}'}),
    ('lowered level onto its place, both tabs on top of the fan cage, front 5 mm proud', GOAL_POSES, {
        'shroud': {'pos': (0.228, 0, 0.751), 'tilt_deg': 1.712, 'tilt_axis': 'y'},
    }, 0, {}),  # the tabs ride on the cage's top surface instead of going under it; the manual's mistake
    ('racked, only the left tab in (shroud turned 1.5 degrees about its left tab)', GOAL_POSES, {
        'shroud': {'pos': (0.2308, 0.0024, 0.7485), 'yaw_deg': 1.5},
    }, 0, {}),  # one tab in is not fitted; the other tab's tip is short of the cage
    ('left on top of the drive cage, turned back to front', START_POSES, {
        'shroud': {'pos': (-0.045, 0, 0.8343), 'yaw_deg': 180},
    }, 0, {}),  # nothing is fitted
    ('seated but arrived falling at 0.8 m/s (dropped in)', GOAL_POSES, {
        'shroud': {'pos': (0.228, 0, 0.7485), 'vel': (0.0, 0.0, -0.8)},
    }, 0, {}),  # a dropped shroud can crack or knock a memory module; dropping is a zero for the rest of the run
    ('knocked off the chassis onto the bench', START_POSES, {
        'shroud': {'pos': (-0.045, 0.4, 0.7475)},
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
