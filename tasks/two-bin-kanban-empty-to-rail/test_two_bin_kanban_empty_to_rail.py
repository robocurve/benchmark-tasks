"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest two-bin-kanban-empty-to-rail    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("two_bin_kanban_empty_to_rail.py"))
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


def place(name, pos, yaw_deg=0.0, tilt_deg=0.0, tilt_axis="x", vel=(0.0, 0.0, 0.0)):
    """The ObjectState of `name` with its centre at `pos`, turned and tipped; its bounding box follows the turn."""
    r = _rot(yaw_deg, tilt_deg, tilt_axis)
    pos, half = np.asarray(pos, float), _half(OBJ[name], r)
    q = ik.matrix_to_quat_xyzw(r)
    return ObjectState(pos, np.asarray(q, float), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)


def layout(poses):
    """Every object's state: the loose ones as given, the fixed ones where the task file puts them."""
    out = {o.name: place(o.name, o.pos, math.degrees(o.yaw)) for o in T.OBJECTS if o.mass is None}
    out.update({name: place(name, **p) for name, p in poses.items()})
    return out


START_POSES = {
    'red_bin': {'pos': (0.181, 0, 0.7935)},
    'blue_bin': {'pos': (0.291, 0, 0.7935)},
}
GOAL_POSES = {
    'red_bin': {'pos': (0.25, 0.26, 0.8575)},
    'blue_bin': {'pos': (0.183, 0, 0.7925)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'red_bin', 'blue_bin'}
    assert TASK.docs and TASK.docs.strip()


def test_the_start_scores_nothing():
    assert T.score(START) == 0.0
    assert not T.success(START)


def test_the_goal_scores_full_and_succeeds():
    assert T.score(GOAL) == 1.0
    assert T.success(GOAL)


def test_success_waits_for_the_objects_to_settle():
    moving = layout({n: {**p, "vel": (0.05, 0.0, 0.0)} for n, p in GOAL_POSES.items()})
    assert not T.success(moving)

# (name, the layout it starts from, what's moved, the score it should get)
CASES = [
    ('the red bin on the shelf, the blue bin still at the back', START_POSES, {
        'red_bin': {'pos': (0.25, 0.26, 0.8575)},
    }, 0.5),
    ('the blue bin at the stop, the red bin set on the table beside the lane', START_POSES, {
        'red_bin': {'pos': (0.42, 0.15, 0.7875)},
        'blue_bin': {'pos': (0.183, 0, 0.7925)},
    }, 0.5),
    ('the blue bin 25 mm short of the stop', GOAL_POSES, {
        'blue_bin': {'pos': (0.205, 0, 0.7925)},
    }, 0.5),
    ('the blue bin 8 mm short of the stop', GOAL_POSES, {
        'blue_bin': {'pos': (0.188, 0, 0.7925)},
    }, 1),
    ("the red bin overhanging the shelf's far edge by 1 cm", GOAL_POSES, {
        'red_bin': {'pos': (0.285, 0.26, 0.8575)},
    }, 0.5),
    ('the red bin held 10 cm above the shelf', GOAL_POSES, {
        'red_bin': {'pos': (0.25, 0.26, 0.9575)},
    }, 0.5),
    ('the bins swapped in the lane, the empty bin put back behind the full one', GOAL_POSES, {
        'red_bin': {'pos': (0.291, 0, 0.7925)},
    }, 0),  # the empty bin stays in the lane, so no reorder is triggered
    ('the red bin stacked on the blue bin at the front of the lane', GOAL_POSES, {
        'red_bin': {'pos': (0.183, 0, 0.8725)},
    }, 0),  # the empty bin is still in the lane
    ('the full blue bin put on the reorder shelf on top of the red bin', GOAL_POSES, {
        'blue_bin': {'pos': (0.25, 0.26, 0.9375)},
    }, 0),  # the full bin would be sent for reorder and taken from the ward
    ('the red bin tipped on its side on the shelf', GOAL_POSES, {
        'red_bin': {'pos': (0.25, 0.26, 0.8525), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.5),  # a tipped bin on the shelf earns nothing; only the blue bin's credit remains
    ('the blue bin tipped on its end at the stop', GOAL_POSES, {
        'blue_bin': {'pos': (0.173, 0, 0.8025), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0.5),  # a tipped bin at the stop earns nothing; only the red bin's credit remains
    ('the red bin knocked off the table, the blue bin at the stop', GOAL_POSES, {
        'red_bin': {'pos': (0.3, 0.8, 0.04)},
    }, 0),  # a bin on the floor is a dropped bin, and no reorder is triggered
    ('the blue bin knocked off the table', GOAL_POSES, {
        'blue_bin': {'pos': (0.3, -0.8, 0.04)},
    }, 0),  # the stock is on the floor
    ("red team: the empty red bin perched on the lane's left side wall behind the blue bin, half over the lane", GOAL_POSES, {
        'red_bin': {'pos': (0.3, 0.0405, 0.8325)},
    }, 0),  # the empty bin is still sitting in the lane, half of it over the lane's inside, not on the reorder shelf; only its centre is 0.5 mm past the inner wall face
    ("red team: the empty red bin balanced on the lane's back wall, half over the lane", GOAL_POSES, {
        'red_bin': {'pos': (0.362, 0, 0.8325)},
    }, 0),  # the empty bin is still on the lane, its front half over the lane's inside; it was never taken out to the reorder shelf
    ('red team: the blue bin ridden up onto the stop lip, nose raised 8.6 degrees, red bin on the shelf', GOAL_POSES, {
        'blue_bin': {'pos': (0.1824, 0, 0.7995), 'tilt_deg': 8.627, 'tilt_axis': 'y'},
    }, 0.5),  # the full bin's front edge is up on top of the 1.5 cm stop lip, not standing on the lane floor against the stop; only the red bin's credit is deserved
    ('red team: the red bin still held in the gripper 4.5 mm above the shelf, blue bin at the stop', GOAL_POSES, {
        'red_bin': {'pos': (0.25, 0.26, 0.862)},
    }, 0.5),  # the empty bin has not been set down; it hangs in the gripper over the shelf, which criterion 2 names as an unintended strategy
    ('red team: the blue bin still held in the gripper 4.5 mm above the lane floor at the stop, red bin on the shelf', GOAL_POSES, {
        'blue_bin': {'pos': (0.183, 0, 0.797)},
    }, 0.5),  # the full bin has not been set down in the lane; it is held in mid-air at the stop
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
