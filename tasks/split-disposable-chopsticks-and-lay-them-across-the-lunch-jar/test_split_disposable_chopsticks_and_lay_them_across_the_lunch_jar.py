"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest split-disposable-chopsticks-and-lay-them-across-the-lunch-jar    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("split_disposable_chopsticks_and_lay_them_across_the_lunch_jar.py"))
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
    'food_jar': {'pos': (0.255, 0, 0.7495)},
    'stick_left': {'pos': (0.255, 0, 0.8939)},
    'stick_right': {'pos': (0.255, 0, 0.8939)},
}
GOAL_POSES = {
    'food_jar': {'pos': (0.255, 0, 0.7495)},
    'stick_left': {'pos': (0.255, 0.002, 0.8939)},
    'stick_right': {'pos': (0.255, -0.002, 0.8939)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_GRIPPERS': '{"left": (PARK["left"], 0.012), "right": (PARK["right"], 0.012)}'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'food_jar', 'stick_left', 'stick_right'}
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
    ('untouched start, the pair whole on the lid', START_POSES, {
    }, 0, {}),
    ('split and laid ready with 9 mm between the tips', START_POSES, {
        'stick_left': {'pos': (0.255, 0.0035, 0.8939)},
        'stick_right': {'pos': (0.255, -0.0035, 0.8939)},
    }, 1, {'TEST_GRIPPERS': '{"left": (PARK["left"], 0.012), "right": (PARK["right"], 0.012)}'}),
    ('split and laid ready, both sticks turned 6 degrees on the lid', START_POSES, {
        'stick_left': {'pos': (0.255, 0.002, 0.8939), 'yaw_deg': 6},
        'stick_right': {'pos': (0.255, -0.002, 0.8939), 'yaw_deg': 6},
    }, 1, {'TEST_GRIPPERS': '{"left": (PARK["left"], 0.012), "right": (PARK["right"], 0.012)}'}),
    ('split, both sticks held 6 cm apart and 2 cm up', START_POSES, {
        'stick_left': {'pos': (0.255, 0.03, 0.9139)},
        'stick_right': {'pos': (0.255, -0.03, 0.9139)},
    }, 0.75, {'TEST_GRIPPERS': '{"left": (pinch_pose(tk.Pose(PAIR_POS), "left", pull=0.030, lift=0.020)[0], 0.0045), "right": (pinch_pose(tk.Pose(PAIR_POS), "right", pull=0.030, lift=0.020)[0], 0.0045)}'}),
    ('split and laid side by side but both hands still on the sticks', GOAL_POSES, {
    }, 0.75, {'TEST_GRIPPERS': '{"left": (pinch_pose(tk.Pose(PAIR_POS), "left", pull=LAY_OUT)[0], 0.0045), "right": (pinch_pose(tk.Pose(PAIR_POS), "right", pull=LAY_OUT)[0], 0.0045)}'}),
    ('split and laid on the lid with 30 mm between the tips', START_POSES, {
        'stick_left': {'pos': (0.255, 0.014, 0.8939)},
        'stick_right': {'pos': (0.255, -0.014, 0.8939)},
    }, 0.75, {'TEST_GRIPPERS': '{"left": (PARK["left"], 0.012), "right": (PARK["right"], 0.012)}'}),
    ('split and laid on the lid crossed at 20 degrees', START_POSES, {
        'stick_left': {'pos': (0.255, 0.002, 0.8939), 'yaw_deg': 10},
        'stick_right': {'pos': (0.255, -0.002, 0.8987), 'yaw_deg': -10},
    }, 0.75, {'TEST_GRIPPERS': '{"left": (PARK["left"], 0.012), "right": (PARK["right"], 0.012)}'}),
    ('split, one stick on the lid and the other on the mat', GOAL_POSES, {
        'stick_right': {'pos': (0.2, -0.14, 0.7519), 'yaw_deg': 30},
    }, 0.75, {'TEST_GRIPPERS': '{"left": (PARK["left"], 0.012), "right": (PARK["right"], 0.012)}'}),
    ('split, both sticks side by side on the mat beside the jar', START_POSES, {
        'stick_left': {'pos': (0.25, -0.15, 0.7519)},
        'stick_right': {'pos': (0.25, -0.155, 0.7519)},
    }, 0.75, {'TEST_GRIPPERS': '{"left": (PARK["left"], 0.012), "right": (PARK["right"], 0.012)}'}),
    ("split and laid with the tips together but one stick pushed 6 cm along, off balance on the lid's edge", GOAL_POSES, {
        'stick_right': {'pos': (0.165, -0.002, 0.8939)},
    }, 0.75, {'TEST_GRIPPERS': '{"left": (PARK["left"], 0.012), "right": (PARK["right"], 0.012)}'}),
    ('uneven split, laid ready', GOAL_POSES, {
    }, 0.5, {'TEST_GRIPPERS': '{"left": (PARK["left"], 0.012), "right": (PARK["right"], 0.012)}', 'TEST_UNEVEN': 'True'}),
    ('uneven split, sticks left on the mat', START_POSES, {
        'stick_left': {'pos': (0.25, -0.15, 0.7519)},
        'stick_right': {'pos': (0.25, -0.17, 0.7519), 'yaw_deg': 25},
    }, 0.25, {'TEST_GRIPPERS': '{"left": (PARK["left"], 0.012), "right": (PARK["right"], 0.012)}', 'TEST_UNEVEN': 'True'}),
    ('the whole pair turned 30 degrees on the lid without splitting (hands staggered along the pair)', START_POSES, {
        'stick_left': {'pos': (0.255, 0, 0.8939), 'yaw_deg': 30},
        'stick_right': {'pos': (0.255, 0, 0.8939), 'yaw_deg': 30},
    }, 0, {'TEST_GRIPPERS': '{"left": (PARK["left"], 0.012), "right": (PARK["right"], 0.012)}'}),  # the pair is still one piece
    ('the whole pair knocked off the jar onto the mat, unsplit', START_POSES, {
        'stick_left': {'pos': (0.25, -0.15, 0.7519)},
        'stick_right': {'pos': (0.25, -0.15, 0.7519)},
    }, 0, {'TEST_GRIPPERS': '{"left": (PARK["left"], 0.012), "right": (PARK["right"], 0.012)}'}),  # the pair is still one piece
    ('the whole pair lifted 5 cm and held, unsplit', START_POSES, {
        'stick_left': {'pos': (0.255, 0, 0.9439)},
        'stick_right': {'pos': (0.255, 0, 0.9439)},
    }, 0, {'TEST_GRIPPERS': '{"left": (pinch_pose(tk.Pose(PAIR_POS), "left", lift=0.05)[0], 0.0045), "right": (PARK["right"], 0.012)}'}),  # the pair is still one piece
    ('split, but one stick has fallen off the table', GOAL_POSES, {
        'stick_right': {'pos': (0.3, -0.45, 0.4475)},
    }, 0, {'TEST_GRIPPERS': '{"left": (PARK["left"], 0.012), "right": (PARK["right"], 0.012)}'}),  # a stick left the table
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
