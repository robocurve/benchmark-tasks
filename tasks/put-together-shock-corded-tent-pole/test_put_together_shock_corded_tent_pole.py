"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest put-together-shock-corded-tent-pole    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("put_together_shock_corded_tent_pole.py"))
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
    'tube_a': {'pos': (0.2093, 0.0447, 0.7526), 'yaw_deg': 167.88},
    'tube_m': {'pos': (0.15, 0, 0.7526)},
    'tube_b': {'pos': (0.1597, 0.05, 0.7526), 'yaw_deg': 180},
    'cord_00': {'pos': (0.86, -0.6, 0.7498)},
    'cord_01': {'pos': (0.86, -0.588, 0.7498)},
    'cord_02': {'pos': (0.86, -0.576, 0.7498)},
    'cord_03': {'pos': (0.86, -0.564, 0.7498)},
}
GOAL_POSES = {
    'tube_a': {'pos': (0.15, -0.38, 0.7526)},
    'tube_m': {'pos': (0.15, 0, 0.7526)},
    'tube_b': {'pos': (0.15, 0.38, 0.7526)},
    'cord_00': {'pos': (0.86, -0.6, 0.7498)},
    'cord_01': {'pos': (0.86, -0.588, 0.7498)},
    'cord_02': {'pos': (0.86, -0.576, 0.7498)},
    'cord_03': {'pos': (0.86, -0.564, 0.7498)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'tube_a', 'cord_02', 'cord_00', 'cord_01', 'cord_03', 'tube_m', 'tube_b'}
    assert TASK.docs and TASK.docs.strip()


def test_the_start_scores_nothing():
    assert T.score(START) == 0.0
    assert not T.success(START)


def test_the_goal_scores_full_and_succeeds():
    assert T.score(GOAL) == 1.0
    assert T.success(GOAL)

# (name, the layout it starts from, what's moved, the score it should get)
CASES = [
    ('assembled but turned on the table (harmless)', GOAL_POSES, {
        'tube_a': {'pos': (0.2031, -0.3785, 0.7526), 'yaw_deg': 5, 'quat': (0.0, 0.0, 0.04361940091645234, 0.9990482209902032)},
        'tube_m': {'pos': (0.17, 0, 0.7526), 'yaw_deg': 5, 'quat': (0.0, 0.0, 0.04361940091645234, 0.9990482209902032)},
        'tube_b': {'pos': (0.1369, 0.3785, 0.7526), 'yaw_deg': 5, 'quat': (0.0, 0.0, 0.04361940091645234, 0.9990482209902032)},
    }, 1),
    ('right joint 3 mm short of home', GOAL_POSES, {
        'tube_a': {'pos': (0.15, -0.383, 0.7526)},
    }, 0.75),
    ('left joint 12 mm short of home', GOAL_POSES, {
        'tube_b': {'pos': (0.15, 0.392, 0.7526)},
    }, 0.5),
    ('right joint home, left section still folded back', START_POSES, {
        'tube_a': {'pos': (0.15, -0.38, 0.7526)},
    }, 0.25),
    ('left joint home, right section still folded', START_POSES, {
        'tube_b': {'pos': (0.15, 0.38, 0.7526)},
    }, 0.25),
    ('folded as it started (nothing done)', START_POSES, {
    }, 0),  # neither insert has entered its tube
    ('right section lined up but its insert 2 mm short of the mouth', START_POSES, {
        'tube_a': {'pos': (0.15, -0.432, 0.7526)},
    }, 0),  # lined up is not entered; neither insert is in a tube
    ('assembled pole pushed so its left end hangs off the table edge', GOAL_POSES, {
        'tube_a': {'pos': (0.15, -0.26, 0.7526)},
        'tube_m': {'pos': (0.15, 0.12, 0.7526)},
        'tube_b': {'pos': (0.15, 0.5, 0.7526)},
    }, 0),  # part of the pole is beyond the table's edge
    ('assembled pole fallen to the floor', GOAL_POSES, {
        'tube_a': {'pos': (0.15, -0.38, 0.05)},
        'tube_m': {'pos': (0.15, 0, 0.05)},
        'tube_b': {'pos': (0.15, 0.38, 0.05)},
    }, 0),  # the pole is off the table
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
