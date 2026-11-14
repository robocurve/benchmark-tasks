"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest close-cable-carrier-over-robot-cable    (in an environment with yam_arena installed)
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
from dataclasses import replace

TASK = load(Path(__file__).with_name("close_cable_carrier_over_robot_cable.py"))
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
    'carrier': {'pos': (0.17, 0.183, 0.7475), 'joints': {'bar01': 100.0, 'catch01': 0.0, 'bar02': 100.0, 'catch02': 0.0, 'bar03': 100.0, 'catch03': 0.0, 'bar04': 100.0, 'catch04': 0.0, 'bar05': 100.0, 'catch05': 0.0, 'bar06': 100.0, 'catch06': 0.0, 'bar07': 100.0, 'catch07': 0.0, 'bar08': 100.0, 'catch08': 0.0, 'bar09': 100.0, 'catch09': 0.0, 'bar10': 100.0, 'catch10': 0.0, 'bar11': 100.0, 'catch11': 0.0, 'bar12': 100.0, 'catch12': 0.0}},
    'grey': {'pos': (0.176, 0.283, 0.7543), 'joints': {'c00': 0.0, 'c01': -29.999999999999996, 'c02': 0.0, 'c03': 0.0, 'c04': 0.0, 'c05': 29.999999999999996, 'c06': 0.0, 'c07': 0.0, 'c08': 0.0, 'c09': 0.0, 'c10': 0.0, 'c11': 0.0, 'c12': 0.0, 'c13': 0.0, 'c14': 0.0, 'c15': 0.0, 'c16': 0.0, 'c17': 0.0, 'c18': 0.0, 'c19': 0.0, 'c20': 0.0, 'c21': 0.0, 'c22': 0.0, 'c23': 0.0, 'c24': 0.0, 'c25': 0.0, 'c26': 0.0, 'c27': 0.0}},
    'orange': {'pos': (0.164, 0.283, 0.7533), 'joints': {'c00': 0.0, 'c01': -29.999999999999996, 'c02': 0.0, 'c03': 0.0, 'c04': 0.0, 'c05': 0.0, 'c06': 0.0, 'c07': 29.999999999999996, 'c08': 0.0, 'c09': 0.0, 'c10': 0.0, 'c11': 0.0, 'c12': 0.0, 'c13': 0.0, 'c14': 0.0, 'c15': 0.0, 'c16': 0.0, 'c17': 0.0, 'c18': 0.0, 'c19': 0.0, 'c20': 0.0, 'c21': 0.0, 'c22': 0.0, 'c23': 0.0, 'c24': 0.0, 'c25': 0.0, 'c26': 0.0, 'c27': 0.0}},
}
GOAL_POSES = {
    'carrier': {'pos': (0.17, 0.183, 0.7475), 'joints': {'bar01': 0.0, 'catch01': 0.0, 'bar02': 0.0, 'catch02': 0.0, 'bar03': 0.0, 'catch03': 0.0, 'bar04': 0.0, 'catch04': 0.0, 'bar05': 0.0, 'catch05': 0.0, 'bar06': 0.0, 'catch06': 0.0, 'bar07': 0.0, 'catch07': 0.0, 'bar08': 0.0, 'catch08': 0.0, 'bar09': 0.0, 'catch09': 0.0, 'bar10': 0.0, 'catch10': 0.0, 'bar11': 0.0, 'catch11': 0.0, 'bar12': 0.0, 'catch12': 0.0}},
    'grey': {'pos': (0.176, 0.283, 0.7543), 'joints': {'c00': 0.0, 'c01': 0.0, 'c02': 0.0, 'c03': 0.0, 'c04': 0.0, 'c05': 0.0, 'c06': 0.0, 'c07': 0.0, 'c08': 0.0, 'c09': 0.0, 'c10': 0.0, 'c11': 0.0, 'c12': 0.0, 'c13': 0.0, 'c14': 0.0, 'c15': 0.0, 'c16': 0.0, 'c17': 0.0, 'c18': 0.0, 'c19': 0.0, 'c20': 0.0, 'c21': 0.0, 'c22': 0.0, 'c23': 0.0, 'c24': 0.0, 'c25': 0.0, 'c26': 0.0, 'c27': 0.0}},
    'orange': {'pos': (0.164, 0.283, 0.7533), 'joints': {'c00': 0.0, 'c01': 0.0, 'c02': 0.0, 'c03': 0.0, 'c04': 0.0, 'c05': 0.0, 'c06': 0.0, 'c07': 0.0, 'c08': 0.0, 'c09': 0.0, 'c10': 0.0, 'c11': 0.0, 'c12': 0.0, 'c13': 0.0, 'c14': 0.0, 'c15': 0.0, 'c16': 0.0, 'c17': 0.0, 'c18': 0.0, 'c19': 0.0, 'c20': 0.0, 'c21': 0.0, 'c22': 0.0, 'c23': 0.0, 'c24': 0.0, 'c25': 0.0, 'c26': 0.0, 'c27': 0.0}},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == set()
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
    ('both cables laid in order, every crossbar still open', START_POSES, {
        'grey': {'pos': (0.176, 0.283, 0.7543), 'joints': {'c00': 0.0, 'c01': 0.0, 'c02': 0.0, 'c03': 0.0, 'c04': 0.0, 'c05': 0.0, 'c06': 0.0, 'c07': 0.0, 'c08': 0.0, 'c09': 0.0, 'c10': 0.0, 'c11': 0.0, 'c12': 0.0, 'c13': 0.0, 'c14': 0.0, 'c15': 0.0, 'c16': 0.0, 'c17': 0.0, 'c18': 0.0, 'c19': 0.0, 'c20': 0.0, 'c21': 0.0, 'c22': 0.0, 'c23': 0.0, 'c24': 0.0, 'c25': 0.0, 'c26': 0.0, 'c27': 0.0}},
        'orange': {'pos': (0.164, 0.283, 0.7533), 'joints': {'c00': 0.0, 'c01': 0.0, 'c02': 0.0, 'c03': 0.0, 'c04': 0.0, 'c05': 0.0, 'c06': 0.0, 'c07': 0.0, 'c08': 0.0, 'c09': 0.0, 'c10': 0.0, 'c11': 0.0, 'c12': 0.0, 'c13': 0.0, 'c14': 0.0, 'c15': 0.0, 'c16': 0.0, 'c17': 0.0, 'c18': 0.0, 'c19': 0.0, 'c20': 0.0, 'c21': 0.0, 'c22': 0.0, 'c23': 0.0, 'c24': 0.0, 'c25': 0.0, 'c26': 0.0, 'c27': 0.0}},
    }, 0.2, {}),  # Cables inside every link without crossing (0.2); no link has a latched crossbar.
    ('both cables laid, crossbars of links 1-6 latched', START_POSES, {
        'carrier': {'pos': (0.17, 0.183, 0.7475), 'joints': {'bar01': 0.0, 'catch01': 0.0, 'bar02': 0.0, 'catch02': 0.0, 'bar03': 0.0, 'catch03': 0.0, 'bar04': 0.0, 'catch04': 0.0, 'bar05': 0.0, 'catch05': 0.0, 'bar06': 0.0, 'catch06': 0.0, 'bar07': 100.0, 'catch07': 0.0, 'bar08': 100.0, 'catch08': 0.0, 'bar09': 100.0, 'catch09': 0.0, 'bar10': 100.0, 'catch10': 0.0, 'bar11': 100.0, 'catch11': 0.0, 'bar12': 100.0, 'catch12': 0.0}},
        'grey': {'pos': (0.176, 0.283, 0.7543), 'joints': {'c00': 0.0, 'c01': 0.0, 'c02': 0.0, 'c03': 0.0, 'c04': 0.0, 'c05': 0.0, 'c06': 0.0, 'c07': 0.0, 'c08': 0.0, 'c09': 0.0, 'c10': 0.0, 'c11': 0.0, 'c12': 0.0, 'c13': 0.0, 'c14': 0.0, 'c15': 0.0, 'c16': 0.0, 'c17': 0.0, 'c18': 0.0, 'c19': 0.0, 'c20': 0.0, 'c21': 0.0, 'c22': 0.0, 'c23': 0.0, 'c24': 0.0, 'c25': 0.0, 'c26': 0.0, 'c27': 0.0}},
        'orange': {'pos': (0.164, 0.283, 0.7533), 'joints': {'c00': 0.0, 'c01': 0.0, 'c02': 0.0, 'c03': 0.0, 'c04': 0.0, 'c05': 0.0, 'c06': 0.0, 'c07': 0.0, 'c08': 0.0, 'c09': 0.0, 'c10': 0.0, 'c11': 0.0, 'c12': 0.0, 'c13': 0.0, 'c14': 0.0, 'c15': 0.0, 'c16': 0.0, 'c17': 0.0, 'c18': 0.0, 'c19': 0.0, 'c20': 0.0, 'c21': 0.0, 'c22': 0.0, 'c23': 0.0, 'c24': 0.0, 'c25': 0.0, 'c26': 0.0, 'c27': 0.0}},
    }, 0.5, {}),  # 6 x 0.05 for the covered links plus 0.2 for the uncrossed cables.
    ('all twelve latched but the orange cable crosses over the grey one', GOAL_POSES, {
        'orange': {'pos': (0.164, 0.283, 0.7533), 'joints': {'c00': 0.0, 'c01': 20.0, 'c02': 0.0, 'c03': -20.0, 'c04': 0.0, 'c05': 0.0, 'c06': 0.0, 'c07': 0.0, 'c08': 0.0, 'c09': 0.0, 'c10': 0.0, 'c11': 0.0, 'c12': 0.0, 'c13': 0.0, 'c14': 0.0, 'c15': 0.0, 'c16': 0.0, 'c17': 0.0, 'c18': 0.0, 'c19': 0.0, 'c20': 0.0, 'c21': 0.0, 'c22': 0.0, 'c23': 0.0, 'c24': 0.0, 'c25': 0.0, 'c26': 0.0, 'c27': 0.0}},
    }, 0.8, {}),  # Orange runs 1.7 mm farther from the robot than grey in every link (crossed), so the 0.2 order credit is lost.
    ('everything done but the crossbar of link 12 rests on its catch, not snapped', GOAL_POSES, {
        'carrier': {'pos': (0.17, 0.183, 0.7475), 'joints': {'bar01': 0.0, 'catch01': 0.0, 'bar02': 0.0, 'catch02': 0.0, 'bar03': 0.0, 'catch03': 0.0, 'bar04': 0.0, 'catch04': 0.0, 'bar05': 0.0, 'catch05': 0.0, 'bar06': 0.0, 'catch06': 0.0, 'bar07': 0.0, 'catch07': 0.0, 'bar08': 0.0, 'catch08': 0.0, 'bar09': 0.0, 'catch09': 0.0, 'bar10': 0.0, 'catch10': 0.0, 'bar11': 0.0, 'catch11': 0.0, 'bar12': 7.499999999999999, 'catch12': 0.0}},
    }, 0.75, {}),  # Link 12 earns nothing and the all-latched 0.2 is lost; 11 x 0.05 + 0.2.
    ('everything done but the catch of link 7 is held back off its crossbar', GOAL_POSES, {
        'carrier': {'pos': (0.17, 0.183, 0.7475), 'joints': {'bar01': 0.0, 'catch01': 0.0, 'bar02': 0.0, 'catch02': 0.0, 'bar03': 0.0, 'catch03': 0.0, 'bar04': 0.0, 'catch04': 0.0, 'bar05': 0.0, 'catch05': 0.0, 'bar06': 0.0, 'catch06': 0.0, 'bar07': 0.0, 'catch07': -0.0015, 'bar08': 0.0, 'catch08': 0.0, 'bar09': 0.0, 'catch09': 0.0, 'bar10': 0.0, 'catch10': 0.0, 'bar11': 0.0, 'catch11': 0.0, 'bar12': 0.0, 'catch12': 0.0}},
    }, 0.75, {}),
    ('both cables laid, crossbar 3 stuck 7 deg open on a cable, the rest open', GOAL_POSES, {
        'carrier': {'pos': (0.17, 0.183, 0.7475), 'joints': {'bar01': 100.0, 'catch01': 0.0, 'bar02': 100.0, 'catch02': 0.0, 'bar03': 7.0, 'catch03': 0.0, 'bar04': 100.0, 'catch04': 0.0, 'bar05': 100.0, 'catch05': 0.0, 'bar06': 100.0, 'catch06': 0.0, 'bar07': 100.0, 'catch07': 0.0, 'bar08': 100.0, 'catch08': 0.0, 'bar09': 100.0, 'catch09': 0.0, 'bar10': 100.0, 'catch10': 0.0, 'bar11': 100.0, 'catch11': 0.0, 'bar12': 100.0, 'catch12': 0.0}},
    }, 0.2, {}),  # No link is latched, so only the uncrossed-cables credit remains.
    ('full credit', GOAL_POSES, {
    }, 1, {}),
    ('crossbars all shut over an empty carrier, cables still on the table', START_POSES, {
        'carrier': {'pos': (0.17, 0.183, 0.7475), 'joints': {'bar01': 0.0, 'catch01': 0.0, 'bar02': 0.0, 'catch02': 0.0, 'bar03': 0.0, 'catch03': 0.0, 'bar04': 0.0, 'catch04': 0.0, 'bar05': 0.0, 'catch05': 0.0, 'bar06': 0.0, 'catch06': 0.0, 'bar07': 0.0, 'catch07': 0.0, 'bar08': 0.0, 'catch08': 0.0, 'bar09': 0.0, 'catch09': 0.0, 'bar10': 0.0, 'catch10': 0.0, 'bar11': 0.0, 'catch11': 0.0, 'bar12': 0.0, 'catch12': 0.0}},
    }, 0, {}),
    ('only the grey cable laid, all crossbars shut', GOAL_POSES, {
        'orange': {'pos': (0.164, 0.283, 0.7533), 'joints': {'c00': 0.0, 'c01': -29.999999999999996, 'c02': 0.0, 'c03': 0.0, 'c04': 0.0, 'c05': 0.0, 'c06': 0.0, 'c07': 29.999999999999996, 'c08': 0.0, 'c09': 0.0, 'c10': 0.0, 'c11': 0.0, 'c12': 0.0, 'c13': 0.0, 'c14': 0.0, 'c15': 0.0, 'c16': 0.0, 'c17': 0.0, 'c18': 0.0, 'c19': 0.0, 'c20': 0.0, 'c21': 0.0, 'c22': 0.0, 'c23': 0.0, 'c24': 0.0, 'c25': 0.0, 'c26': 0.0, 'c27': 0.0}},
    }, 0, {}),  # Every link needs both cables inside; the order and final credits need both cables too.
    ("finished, but the right gripper still pinches the grey cable's end", GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': "{'left': (np.array([0.08, 0.28, TOP + 0.16]), 1.0), 'right': (np.array([XC + 0.005, -0.29, TOP + 0.016]), 0.0)}"}),
    ('finished, but the left gripper still rests on crossbar 1', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': "{'left': (np.array([XC - 0.006, Y0 + link_y(1), TOP + 0.036]), 0.0), 'right': (np.array([0.08, -0.28, TOP + 0.16]), 1.0)}"}),
    ('grey cable pulled 8 mm through its tie-wrap', GOAL_POSES, {
        'grey': {'pos': (0.176, 0.283, 0.7543), 'joints': {'c00': -0.008, 'c01': 0.0, 'c02': 0.0, 'c03': 0.0, 'c04': 0.0, 'c05': 0.0, 'c06': 0.0, 'c07': 0.0, 'c08': 0.0, 'c09': 0.0, 'c10': 0.0, 'c11': 0.0, 'c12': 0.0, 'c13': 0.0, 'c14': 0.0, 'c15': 0.0, 'c16': 0.0, 'c17': 0.0, 'c18': 0.0, 'c19': 0.0, 'c20': 0.0, 'c21': 0.0, 'c22': 0.0, 'c23': 0.0, 'c24': 0.0, 'c25': 0.0, 'c26': 0.0, 'c27': 0.0}},
    }, 0, {}),
    ('a cable was pulled through its tie-wrap earlier, then pushed back (sticky)', GOAL_POSES, {
    }, 0, {'TEST_STICKY': "('cable pulled through its strain relief',)"}),
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
