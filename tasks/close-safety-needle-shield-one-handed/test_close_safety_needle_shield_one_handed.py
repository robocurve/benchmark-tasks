"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest close-safety-needle-shield-one-handed    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("close_safety_needle_shield_one_handed.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 1400


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
    'syringe': {'pos': (0.2686, -0.0956, 0.7528), 'yaw_deg': -100, 'joints': {'body': 100.0, 'catch': 0.0}},
    'tray': {'pos': (0.13, -0.225, 0.7475)},
}
GOAL_POSES = {
    'syringe': {'pos': (0.13, -0.207, 0.7558), 'joints': {'body': 0.0, 'catch': 0.0}},
    'tray': {'pos': (0.13, -0.225, 0.7475)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'tray', 'syringe'}
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
    ('sheath locked over the needle, syringe left on the table', START_POSES, {
        'syringe': {'pos': (0.2686, -0.0956, 0.7528), 'yaw_deg': -100, 'joints': {'body': 0.0, 'catch': 0.0}},
    }, 0.4, {}),  # Locked and covered (0.4) but not in the sharps tray.
    ('bare syringe put in the tray, sheath still open', START_POSES, {
        'syringe': {'pos': (0.1886, -0.2626, 0.7558), 'yaw_deg': -100, 'joints': {'body': 100.0, 'catch': 0.0}},
    }, 0.15, {}),
    ('sheath swung to 30 degrees from closed, not locked, on the table', START_POSES, {
        'syringe': {'pos': (0.2686, -0.0956, 0.7528), 'yaw_deg': -100, 'joints': {'body': 29.999999999999996, 'catch': 0.0}},
    }, 0.1, {}),
    ('sheath resting on the needle with the catch still lifted (not clicked)', GOAL_POSES, {
        'syringe': {'pos': (0.13, -0.207, 0.7558), 'joints': {'body': 3.5, 'catch': 0.00044}},
    }, 0.15, {}),  # The catch has not dropped behind the needle, so the sheath is not locked; only the tray credit.
    ('locked syringe still held over the tray by the right gripper', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': "{'left': (np.array([0.10, 0.30, TOP + 0.20]), 1.0), 'right': (np.array([TRAY_POS[0], TRAY_POS[1], TOP + 0.03]), 0.1)}"}),
    ('locked syringe falling above the tray (not at rest)', GOAL_POSES, {
        'syringe': {'pos': (0.13, -0.207, 0.8475), 'vel': (0.0, 0.0, -0.3), 'joints': {'body': 0.0, 'catch': 0.0}},
    }, 0, {}),
    ('locked syringe lying across the tray rim', GOAL_POSES, {
        'syringe': {'pos': (0.18, -0.207, 0.8127), 'joints': {'body': 0.0, 'catch': 0.0}},
    }, 0.4, {}),  # Not inside the tray's outline and not below the rim, so only the lock credit.
    ('the other gripper came near the bare needle earlier (sticky)', GOAL_POSES, {
    }, 0, {'TEST_STICKY': "('free hand near the bare needle',)"}),
    ('the holding gripper came at the needle end earlier (sticky)', GOAL_POSES, {
    }, 0, {'TEST_STICKY': "('holding hand at the needle end',)"}),
    ('the bare needle touched the pad earlier (sticky)', GOAL_POSES, {
    }, 0, {'TEST_STICKY': "('bare needle touched the pad',)"}),
    ('the bare needle touched the tray wall earlier (sticky)', GOAL_POSES, {
    }, 0, {'TEST_STICKY': "('bare needle touched the tray wall',)"}),
    ('sheath pushed shut by a finger or off the table earlier (sticky)', GOAL_POSES, {
    }, 0, {'TEST_STICKY': "('sheath not locked on the table',)"}),
    ('syringe dropped off the table earlier, then put back (sticky)', GOAL_POSES, {
    }, 0, {'TEST_STICKY': "('syringe off the table',)"}),
    ('left gripper 60 mm from the bare needle tip', START_POSES, {
    }, 0, {'TEST_GRIPPERS': "{'left': (np.array([0.23, -0.115, TOP + 0.06]), 1.0), 'right': (np.array([0.23, -0.30, TOP + 0.20]), 1.0)}"}),
    ('bare needle tip pushed into the pad', START_POSES, {
        'syringe': {'pos': (0.2686, 0.1924, 0.7528), 'yaw_deg': -100, 'joints': {'body': 100.0, 'catch': 0.0}},
    }, 0, {}),
    ('syringe on the floor', START_POSES, {
        'syringe': {'pos': (0.3, -0.45, 0.02), 'yaw_deg': -100, 'joints': {'body': 100.0, 'catch': 0.0}},
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
