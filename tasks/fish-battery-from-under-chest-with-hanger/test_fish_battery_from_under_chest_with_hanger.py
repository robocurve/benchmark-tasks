"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest fish-battery-from-under-chest-with-hanger    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("fish_battery_from_under_chest_with_hanger.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 1800


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
    'chest': {'pos': (0.355, 0, 0.748)},
    'battery': {'pos': (0.485, 0.09, 0.7548), 'yaw_deg': 38},
    'hanger': {'pos': (0.1823, -0.0589, 0.7528), 'yaw_deg': -90},
}
GOAL_POSES = {
    'chest': {'pos': (0.355, 0, 0.748)},
    'battery': {'pos': (0.274, 0.301, 0.7548), 'yaw_deg': 80},
    'hanger': {'pos': (0.2619, 0.0597, 0.7522), 'yaw_deg': -94},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'hanger', 'battery', 'chest'}
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
    ('battery drawn 12 cm nearer, half under the front rail (not out)', START_POSES, {
        'battery': {'pos': (0.365, 0.17, 0.7548), 'yaw_deg': 70},
    }, 0.4, {}),
    ('battery drawn 6 cm nearer, still under the chest', START_POSES, {
        'battery': {'pos': (0.415, 0.14, 0.7548), 'yaw_deg': 50},
    }, 0.2, {}),
    ('battery drawn only 2 cm nearer', START_POSES, {
        'battery': {'pos': (0.475, 0.1, 0.7548), 'yaw_deg': 40},
    }, 0, {}),
    ("battery out, but the hand still pinches the hanger's bar", GOAL_POSES, {
    }, 0.7, {'TEST_GRIPPERS': "{'left': ((0.12, 0.42, TOP + 0.15), 1.0), 'right': ((GRIP_IN[0] + SWEEP_BACK[0], GRIP_IN[1] + SWEEP_BACK[1], TOP + 0.0155), 0.10)}"}),
    ('battery out and hanger let go, but the open hand rests 1 cm above the bar', GOAL_POSES, {
    }, 0.7, {'TEST_GRIPPERS': "{'left': ((0.12, 0.42, TOP + 0.15), 1.0), 'right': ((GRIP_IN[0] + SWEEP_BACK[0], GRIP_IN[1] + SWEEP_BACK[1], TOP + 0.025), 0.4)}"}),
    ('battery out, its back 1 mm in front of the rail face', GOAL_POSES, {
        'battery': {'pos': (0.3468, 0.2, 0.7548), 'yaw_deg': 90},
    }, 1, {}),
    ("battery's back 1 mm behind the rail face (not out)", GOAL_POSES, {
        'battery': {'pos': (0.3488, 0.2, 0.7548), 'yaw_deg': 90},
    }, 0.4, {}),
    ('battery pushed 13 cm deeper under the chest', START_POSES, {
        'battery': {'pos': (0.625, 0.01, 0.7548), 'yaw_deg': 38},
    }, 0, {}),
    ('battery pushed deep earlier, then brought out (sticky)', GOAL_POSES, {
    }, 0, {'TEST_STICKY': "('battery pushed deeper',)"}),  # 12 cm deeper than it lay the battery is past where the hanger held at its end can get behind it
    ("battery shoved out under the chest's left side", START_POSES, {
        'battery': {'pos': (0.505, 0.335, 0.7548), 'yaw_deg': 38},
    }, 0, {}),
    ('battery out, but the chest pushed 15 mm back', GOAL_POSES, {
        'chest': {'pos': (0.37, 0, 0.7475)},
    }, 0, {}),
    ('battery out, but the chest moved earlier and was put back (sticky)', GOAL_POSES, {
    }, 0, {'TEST_STICKY': "('chest moved',)"}),
    ('battery out but still rolling', GOAL_POSES, {
    }, 0, {'TEST_STILL': 'False'}),
    ('battery held 3 cm above the floor in front of the chest', GOAL_POSES, {
        'battery': {'pos': (0.274, 0.301, 0.7847), 'yaw_deg': 80},
    }, 0, {}),
    ('battery dropped off the table', GOAL_POSES, {
        'battery': {'pos': (0.2, 0.7, 0.6475), 'yaw_deg': 80},
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
