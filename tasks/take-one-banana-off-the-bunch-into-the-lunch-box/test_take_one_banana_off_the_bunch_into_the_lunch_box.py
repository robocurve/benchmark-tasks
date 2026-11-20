"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest take-one-banana-off-the-bunch-into-the-lunch-box    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("take_one_banana_off_the_bunch_into_the_lunch_box.py"))
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
    'crown': {'pos': (0.13, 0.14, 0.7479)},
    'banana_1': {'pos': (0.095, 0.124, 0.7801), 'yaw_deg': -24},
    'banana_2': {'pos': (0.118, 0.124, 0.7801), 'yaw_deg': -5},
    'banana_3': {'pos': (0.14, 0.124, 0.7801), 'yaw_deg': 6},
    'banana_4': {'pos': (0.162, 0.124, 0.7801), 'yaw_deg': 18},
    'lunch_box': {'pos': (0.27, -0.215, 0.7479)},
    'lid': {'pos': (0.465, -0.215, 0.7479)},
}
GOAL_POSES = {
    'crown': {'pos': (0.13, 0.14, 0.7479)},
    'banana_1': {'pos': (0.2761, -0.1198, 0.7831)},
    'banana_2': {'pos': (0.118, 0.124, 0.7801), 'yaw_deg': -5},
    'banana_3': {'pos': (0.14, 0.124, 0.7801), 'yaw_deg': 6},
    'banana_4': {'pos': (0.162, 0.124, 0.7801), 'yaw_deg': 18},
    'lunch_box': {'pos': (0.27, -0.215, 0.7479)},
    'lid': {'pos': (0.465, -0.215, 0.7479)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'crown', 'banana_4', 'banana_2', 'banana_3', 'lunch_box', 'banana_1', 'lid'}
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
    ('untouched start, all four bananas on the bunch', START_POSES, {
    }, 0, {}),
    ('banana snapped off and still held 5 cm over its place', START_POSES, {
        'banana_1': {'pos': (0.095, 0.124, 0.8301), 'yaw_deg': -24},
    }, 0.25, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (grasp_plan()[0] + np.array([0.0, 0.0, 0.05]), 0.4)}'}),
    ('banana off and let go on the table, away from the box', START_POSES, {
        'banana_1': {'pos': (0.4, 0, 0.7801), 'yaw_deg': 180},
    }, 0.25, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ("banana resting across the box's side wall on the rim", GOAL_POSES, {
        'banana_1': {'pos': (0.3461, -0.1198, 0.8551)},
    }, 0.5, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ('banana in the box but still held', GOAL_POSES, {
    }, 0.5, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (np.array([BOX_XY[0], BOX_XY[1] - 0.05, TOP + 0.03]), 0.4)}'}),
    ('one banana boxed cleanly, crown dragged 8 mm', GOAL_POSES, {
    }, 1, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}', 'TEST_CROWN_START': '[CROWN.pos[0] - 0.008, CROWN.pos[1], CROWN.pos[2]]'}),
    ('one banana boxed cleanly, crown dragged 22 mm (the one-handed pull)', GOAL_POSES, {
    }, 0.75, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}', 'TEST_CROWN_START': '[CROWN.pos[0] + 0.016, CROWN.pos[1] + 0.015, CROWN.pos[2]]'}),
    ('one banana boxed cleanly, crown turned 15 degrees in place', GOAL_POSES, {
    }, 0.75, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}', 'TEST_CROWN_START': '[CROWN.pos[0], CROWN.pos[1], CROWN.pos[2], 15.0]'}),
    ('one banana boxed cleanly, crown turned 8 degrees in place', GOAL_POSES, {
    }, 1, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}', 'TEST_CROWN_START': '[CROWN.pos[0], CROWN.pos[1], CROWN.pos[2], -8.0]'}),
    ('banana in the box, crown dragged 45 mm', GOAL_POSES, {
    }, 0.5, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}', 'TEST_CROWN_START': '[CROWN.pos[0] - 0.045, CROWN.pos[1], CROWN.pos[2]]'}),
    ('banana in the box, crown dragged 80 mm', GOAL_POSES, {
    }, 0.25, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}', 'TEST_CROWN_START': '[CROWN.pos[0], CROWN.pos[1] + 0.080, CROWN.pos[2]]'}),
    ('banana in the box, squeezed at 42 N for 0.2 s (dent 5 to 8 mm)', GOAL_POSES, {
    }, 0.5, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}', 'TEST_SQUEEZE': '{"banana_1": 42.0}'}),
    ("banana in the box, squeezed at the gripper's 50 N limit (dent over 8 mm)", GOAL_POSES, {
    }, 0.25, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}', 'TEST_SQUEEZE': '{"banana_1": 50.0}'}),
    ('banana in the box, squeezed at 30 N (no dent)', GOAL_POSES, {
    }, 1, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}', 'TEST_SQUEEZE': '{"banana_1": 30.0}'}),
    ('two bananas off, one in the box and one on the table', GOAL_POSES, {
        'banana_2': {'pos': (0.4, 0, 0.7801), 'yaw_deg': 180},
    }, 0, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),  # only one banana may come off the bunch
    ('banana in the box, but the next banana on the bunch was dented', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}', 'TEST_SQUEEZE': '{"banana_2": 45.0}'}),  # a banana left on the bunch is damaged
    ('banana fallen off the table', START_POSES, {
        'banana_1': {'pos': (0.3, 0, 0.5475), 'yaw_deg': -24},
    }, 0, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),  # fruit off the table
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
