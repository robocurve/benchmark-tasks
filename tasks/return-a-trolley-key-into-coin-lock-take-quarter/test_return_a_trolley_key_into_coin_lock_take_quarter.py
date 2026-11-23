"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest return-a-trolley-key-into-coin-lock-take-quarter    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("return_a_trolley_key_into_coin_lock_take_quarter.py"))
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
    'near_lock': {'pos': (0.15, -0.11, 0.8475), 'joints': {'pusher': 0.0, 'plunger': 0.0, 'ball': 0.0}},
    'front_lock': {'pos': (0.328, -0.11, 0.8475), 'joints': {'c01a': 0.0, 'c01': 0.0, 'c02a': 0.0, 'c02': 0.0, 'c03a': 0.0, 'c03': 0.0, 'c04a': 0.0, 'c04': 0.0, 'c05a': 0.0, 'c05': 0.0, 'c06a': 0.0, 'c06': 0.0, 'c07a': 0.0, 'c07': 0.0, 'c08a': 0.0, 'c08': 0.0, 'c09a': 0.0, 'c09': 0.0, 'c10a': 0.0, 'c10': 0.0, 'c11a': 0.0, 'c11': 0.0, 'c12a': 0.0, 'c12': 0.0, 'c13a': 0.0, 'c13': 0.0, 'key_z': 0.0, 'key_x': 0.0, 'key': 0.0}},
    'quarter': {'pos': (0.126, -0.13, 0.8869)},
    'mat': {'pos': (0.06, -0.04, 0.7485)},
}
GOAL_POSES = {
    'near_lock': {'pos': (0.15, -0.11, 0.8475), 'joints': {'pusher': -0.03, 'plunger': 0.006, 'ball': 0.0}},
    'front_lock': {'pos': (0.328, -0.11, 0.8475), 'joints': {'c01a': 70.291, 'c01': 36.507, 'c02a': 20.883, 'c02': 30.328, 'c03a': -0.839, 'c03': 18.439, 'c04a': -4.331, 'c04': 4.854, 'c05a': 0.11299999999999999, 'c05': -14.845000000000002, 'c06a': 6.869, 'c06': -45.095, 'c07a': -1.614, 'c07': -48.182, 'c08a': 0.649, 'c08': -35.95, 'c09a': -6.842, 'c09': -21.285, 'c10a': -17.705, 'c10': -6.531, 'c11a': -30.311, 'c11': 8.476, 'c12a': -36.932, 'c12': 27.6, 'c13a': -24.499, 'c13': 59.58599999999999, 'key_z': 94.815, 'key_x': -49.186, 'key': -68.659}},
    'quarter': {'pos': (0.06, -0.04, 0.7504), 'tilt_deg': 90, 'tilt_axis': 'x'},
    'mat': {'pos': (0.06, -0.04, 0.7485)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'mat', 'quarter'}
    assert TASK.docs and TASK.docs.strip()


def test_the_start_scores_nothing():
    assert T.score(START) == 0.0
    assert not T.success(START)


def test_the_goal_scores_full_and_succeeds():
    assert T.score(GOAL) == 1.0
    assert T.success(GOAL)

# (name, the layout it starts from, what's moved, the score it should get)
CASES = [
    ('untouched start: key on the bench, quarter locked in', START_POSES, {
    }, 0),
    ('key slid 8 mm into the slot, lock not reached', START_POSES, {
        'front_lock': {'pos': (0.328, -0.11, 0.8475), 'joints': {'c01a': 57.154, 'c01': 50.396, 'c02a': 8.897, 'c02': 29.904, 'c03a': -5.46, 'c03': 13.59, 'c04a': -4.543, 'c04': 0.6, 'c05a': 0.426, 'c05': -17.478, 'c06a': 3.764, 'c06': -48.784, 'c07a': -11.004, 'c07': -56.237, 'c08a': -2.981, 'c08': -34.787, 'c09a': -8.101, 'c09': -15.714, 'c10a': -18.427, 'c10': 0.048, 'c11a': -31.956999999999997, 'c11': 15.215000000000002, 'c12a': -40.831, 'c12': 35.986, 'c13a': -32.16, 'c13': 71.604, 'key_z': 110.644, 'key_x': -59.09700000000001, 'key': -60.452999999999996}},
    }, 0.25),
    ('key pushed to the stop but the pusher still in (lock not released)', START_POSES, {
        'near_lock': {'pos': (0.15, -0.11, 0.8475), 'joints': {'pusher': 0.0, 'plunger': 0.006, 'ball': 0.0}},
        'front_lock': {'pos': (0.328, -0.11, 0.8475), 'joints': {'c01a': 70.291, 'c01': 36.507, 'c02a': 20.883, 'c02': 30.328, 'c03a': -0.839, 'c03': 18.439, 'c04a': -4.331, 'c04': 4.854, 'c05a': 0.11299999999999999, 'c05': -14.845000000000002, 'c06a': 6.869, 'c06': -45.095, 'c07a': -1.614, 'c07': -48.182, 'c08a': 0.649, 'c08': -35.95, 'c09a': -6.842, 'c09': -21.285, 'c10a': -17.705, 'c10': -6.531, 'c11a': -30.311, 'c11': 8.476, 'c12a': -36.932, 'c12': 27.6, 'c13a': -24.499, 'c13': 59.58599999999999, 'key_z': 94.815, 'key_x': -49.186, 'key': -68.659}},
    }, 0.25),
    ('key home and locked, quarter still standing in the pusher', GOAL_POSES, {
        'quarter': {'pos': (0.096, -0.13, 0.8869), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.5),
    ('key locked, quarter drawn 30 mm up out of the pusher (still in the hand)', GOAL_POSES, {
        'quarter': {'pos': (0.096, -0.13, 0.9169), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.75),
    ('key locked, quarter dropped flat on the bench beside the mat', GOAL_POSES, {
        'quarter': {'pos': (0.14, -0.04, 0.7484), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.75),
    ('key locked, quarter standing on edge on the mat (resting on it counts)', GOAL_POSES, {
        'quarter': {'pos': (0.06, -0.04, 0.7616)},
    }, 1),
    ('key locked, quarter flat on the mat, mat nudged 20 mm', GOAL_POSES, {
        'quarter': {'pos': (0.08, -0.04, 0.7504), 'tilt_deg': 90, 'tilt_axis': 'x'},
        'mat': {'pos': (0.08, -0.04, 0.7485)},
    }, 1),
    ('key locked, quarter held 20 mm above the mat', GOAL_POSES, {
        'quarter': {'pos': (0.06, -0.04, 0.7816)},
    }, 0.75),
    ('quarter on the mat but the key never put in', START_POSES, {
        'near_lock': {'pos': (0.15, -0.11, 0.8475), 'joints': {'pusher': -0.03, 'plunger': 0.0, 'ball': 0.0}},
        'quarter': {'pos': (0.06, -0.04, 0.7504), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # the lock only gives the coin back for the key; without the key home nothing counts
]


@pytest.mark.parametrize(("name", "base", "moved", "expected"), CASES, ids=[c[0] for c in CASES])
def test_named_layouts(name, base, moved, expected):
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
