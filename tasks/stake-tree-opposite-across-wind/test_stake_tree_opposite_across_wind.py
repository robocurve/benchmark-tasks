"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest stake-tree-opposite-across-wind    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("stake_tree_opposite_across_wind.py"))
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
    'stake_a': {'pos': (0.22, -0.3, 0.8135)},
    'stake_b': {'pos': (0.14, -0.3, 0.8135)},
}
GOAL_POSES = {
    'stake_a': {'pos': (0.3619, -0.0919, 0.8075)},
    'stake_b': {'pos': (0.1781, 0.0919, 0.8075)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'stake_a', 'stake_b'}
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
    ('one stake placed, the other still in the tray', START_POSES, {
        'stake_a': {'pos': (0.3619, -0.0919, 0.8075)},
    }, 0.2),
    ('one stake placed, the other left at the hand-off spot', START_POSES, {
        'stake_a': {'pos': (0.3619, -0.0919, 0.8075)},
        'stake_b': {'pos': (0.1, 0, 0.8075)},
    }, 0.2),
    ('both stakes in the ring on the right side, 45 degrees apart', START_POSES, {
        'stake_a': {'pos': (0.3619, -0.0919, 0.8075)},
        'stake_b': {'pos': (0.27, -0.13, 0.8075)},
    }, 0.4),
    ('opposite each other but in line with the wind strip', START_POSES, {
        'stake_a': {'pos': (0.3619, 0.0919, 0.8075)},
        'stake_b': {'pos': (0.1781, -0.0919, 0.8075)},
    }, 0.6),
    ('the left stake 15 degrees off the opposite bearing', GOAL_POSES, {
        'stake_b': {'pos': (0.1574, 0.065, 0.8075)},
    }, 0.4),
    ('the left stake 8 degrees off the opposite bearing, still within tolerance', GOAL_POSES, {
        'stake_b': {'pos': (0.1662, 0.0782, 0.8075)},
    }, 1),
    ('both stakes at the inner edge of the ring, 11.5 cm out', START_POSES, {
        'stake_a': {'pos': (0.3513, -0.0813, 0.8075)},
        'stake_b': {'pos': (0.1887, 0.0813, 0.8075)},
    }, 1),
    ('the left stake 17 cm out, past the ring', GOAL_POSES, {
        'stake_b': {'pos': (0.1498, 0.1202, 0.8075)},
    }, 0.2),
    ('the left stake leaning 20 degrees', GOAL_POSES, {
        'stake_b': {'pos': (0.1781, 0.0919, 0.8107), 'tilt_deg': 20, 'tilt_axis': 'x'},
    }, 0.2),
    ('red team: the left stake standing upright but upside down (flipped end over end)', GOAL_POSES, {
        'stake_b': {'pos': (0.1781, 0.0919, 0.8075), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # the stake is a plain symmetric cylinder; flipped, it stands exactly as upright on the table as before, and the scoring text only asks for at most 10 degrees from vertical
    ('red team: both stakes upright but upside down in the correct spots', GOAL_POSES, {
        'stake_a': {'pos': (0.3619, -0.0919, 0.8075), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'y'},
        'stake_b': {'pos': (0.1781, 0.0919, 0.8075), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # the goal layout, with identical end-symmetric cylinders standing vertical; a level app reads 0 degrees
    ('red team: the left stake upside down and leaning 5 degrees', GOAL_POSES, {
        'stake_b': {'pos': (0.1781, 0.0919, 0.809), 'tilt_deg': 175, 'tilt_axis': 'x'},
    }, 1),  # 5 degrees from vertical, within the stated 10-degree limit
    ('red team: one stake placed, the other held in the gripper 10 cm above the root ball', GOAL_POSES, {
        'stake_b': {'pos': (0.22, 0.05, 0.9075)},
    }, 0.2),  # a stake carried in the air over the disc damages no roots; the root-ball penalty is meant for a stake standing on or in the root ball, yet the placed stake's credit is wiped
    ('a stake standing on the root ball', GOAL_POSES, {
        'stake_b': {'pos': (0.22, 0.05, 0.8175)},
    }, 0),  # a stake driven there would go through the roots
    ('a stake lying on its side on the root ball', GOAL_POSES, {
        'stake_b': {'pos': (0.22, 0.05, 0.7775), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # a stake dropped on the root ball rests on the roots; the root-ball rule covers any stake whose lowest point is within 1.5 cm of the table top
    ('a stake on the table touching the root-ball edge, 10 cm from the trunk', GOAL_POSES, {
        'stake_b': {'pos': (0.1993, 0.0707, 0.8075)},
    }, 0),  # its footprint overlaps the root ball
    ('the left stake knocked over, lying on the table pointing away from the tree', GOAL_POSES, {
        'stake_b': {'pos': (0.1781, 0.1519, 0.7675), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.2),  # a toppled stake supports nothing; only the standing one counts (its nearest point is 11.7 cm from the trunk's axis, clear of the root ball)
    ('the left stake knocked over towards the tree, one end lying on the root-ball rim', GOAL_POSES, {
        'stake_b': {'pos': (0.1781, 0.0919, 0.7775), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # its inner end, 7.9 cm from the trunk's axis, lies on the root ball
    ('the left stake pushed off the table', GOAL_POSES, {
        'stake_b': {'pos': (0.3, 0.8, 0.02), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0.2),  # only the stake still standing in place counts
    ('the left stake held 3 cm above its spot', GOAL_POSES, {
        'stake_b': {'pos': (0.1781, 0.0919, 0.8375)},
    }, 0.2),  # a stake in mid-air is not standing on the ground
    ('both stakes knocked over in the ring', GOAL_POSES, {
        'stake_a': {'pos': (0.3619, -0.0919, 0.7675), 'tilt_deg': 90, 'tilt_axis': 'y'},
        'stake_b': {'pos': (0.1781, 0.0919, 0.7675), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # nothing stands
    ('red team: one stake placed, the other lying flat across the top of the root-ball rim, centre 11.5 cm out', GOAL_POSES, {
        'stake_b': {'pos': (0.155, 0, 0.7775), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0),  # the stake lies on the root ball (its inner 3.5 cm rests on the disc's top, its lowest point 1 cm above the table), which the scoring text says the zero-overall rule covers ('it stands or lies on or against the root ball'), but its centre is 11.5 cm out so the rule doesn't fire and the placed stake keeps 0.2
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
