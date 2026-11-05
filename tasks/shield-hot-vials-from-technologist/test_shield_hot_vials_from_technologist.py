"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest shield-hot-vials-from-technologist    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("shield_hot_vials_from_technologist.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 1500


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
    'vial_left': {'pos': (0.25, 0.15, 0.7735)},
    'vial_right': {'pos': (0.25, -0.15, 0.7735)},
    'brick_1': {'pos': (0.2, 0.36, 0.7985)},
    'brick_2': {'pos': (0.3, 0.36, 0.7985)},
}
GOAL_POSES = {
    'vial_left': {'pos': (0.25, 0.15, 0.7735)},
    'vial_right': {'pos': (0.25, -0.15, 0.7735)},
    'brick_1': {'pos': (0.34, 0.12, 0.7875)},
    'brick_2': {'pos': (0.34, -0.12, 0.7875)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'vial_left', 'brick_2', 'brick_1', 'vial_right'}
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
    ('the left vial shielded 5 cm away, the other brick still in the rack', START_POSES, {
        'brick_1': {'pos': (0.34, 0.12, 0.7875)},
    }, 0.5),
    ('the right vial shielded 5 cm away, the other brick still in the rack', START_POSES, {
        'brick_2': {'pos': (0.34, -0.12, 0.7875)},
    }, 0.5),
    ("the left vial's brick turned 30 degrees, still across the line (gap about 2.5 cm)", START_POSES, {
        'brick_1': {'pos': (0.34, 0.12, 0.7875), 'yaw_deg': 30},
    }, 0.5),
    ("the left vial's brick 3 cm to the side of the line, still across it", START_POSES, {
        'brick_1': {'pos': (0.3495, 0.1485, 0.7875)},
    }, 0.5),
    ("a brick on the left vial's line but 18 cm from it", START_POSES, {
        'brick_1': {'pos': (0.47, 0.0767, 0.7875)},
    }, 0.25),
    ('the left vial shielded close, the right one only from 18 cm', START_POSES, {
        'brick_1': {'pos': (0.34, 0.12, 0.7875)},
        'brick_2': {'pos': (0.47, -0.0767, 0.7875)},
    }, 0.75),
    ('the second brick left on the transfer pad', GOAL_POSES, {
        'brick_2': {'pos': (0.15, 0, 0.7975)},
    }, 0.5),
    ('red team: both bricks flipped upside down, standing on their other 5 x 12 cm face at the demo spots', GOAL_POSES, {
        'brick_1': {'pos': (0.34, 0.12, 0.7875), 'tilt_deg': 180, 'tilt_axis': 'x'},
        'brick_2': {'pos': (0.34, -0.12, 0.7875), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # an upside-down brick stands on an identical 5 x 12 cm face, 8 cm tall; its 8 cm axis is vertical, so both vials are shielded
    ("red team: the left vial's brick flipped upside down, the right one upright", GOAL_POSES, {
        'brick_1': {'pos': (0.34, 0.12, 0.7875), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'y'},
    }, 1),  # the flipped brick is still standing upright on a 5 x 12 cm face across the line
    ("red team: the left vial's brick standing upside down with only 2 degrees of extra lean", START_POSES, {
        'brick_1': {'pos': (0.34, 0.12, 0.7875), 'tilt_deg': 178, 'tilt_axis': 'x'},
    }, 0.5),  # a brick on its other big face is standing, 2 degrees from vertical
    ("red team: a fallen brick yawed 45 degrees 6 cm from the right vial cancels the right vial's real shield", START_POSES, {
        'brick_1': {'pos': (0.1673, -0.2327, 0.7725), 'yaw_deg': 45, 'tilt_deg': 90, 'tilt_axis': 'y'},
        'brick_2': {'pos': (0.34, -0.12, 0.7875)},
    }, 0.5),  # the fallen brick's top-view outline is about 6 cm from the right vial (its bounding box corner is 2 mm away), so by the scoring text it does not touch the vial and the standing brick still shields it
    ("red team: a brick held just above the left vial's top, the other brick shielding it", START_POSES, {
        'brick_1': {'pos': (0.34, 0.12, 0.7875)},
        'brick_2': {'pos': (0.25, 0.15, 0.8405)},
    }, 0.5),  # by the scoring text a brick only touches when it is below the vial's top; this one is 3 mm above it, held by a gripper
    ('the left vial knocked over', GOAL_POSES, {
        'vial_left': {'pos': (0.25, 0.15, 0.7625), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # a tipped hot vial is a spill; nothing on the bench counts
    ('the right vial pushed 2 cm toward the robot', GOAL_POSES, {
        'vial_right': {'pos': (0.23, -0.15, 0.7725)},
    }, 0),  # the source has been moved
    ('a brick against the left vial, 2 mm from it, across its line', START_POSES, {
        'brick_1': {'pos': (0.292, 0.15, 0.7875)},
    }, 0),  # the brick touches the hot vial
    ('both bricks lying on their sides across the lines', START_POSES, {
        'brick_1': {'pos': (0.34, 0.12, 0.7725), 'tilt_deg': 90, 'tilt_axis': 'y'},
        'brick_2': {'pos': (0.34, -0.12, 0.8075), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # a brick lying flat (5 cm tall) or on its end is not standing shielding
    ('one brick across both lines next to the seat block', START_POSES, {
        'brick_1': {'pos': (0.595, 0, 0.7875)},
    }, 0),  # it crosses both lines, but 31 cm from each vial
    ("both bricks on the left vial's line", START_POSES, {
        'brick_1': {'pos': (0.34, 0.12, 0.7875)},
        'brick_2': {'pos': (0.47, 0.0767, 0.7875)},
    }, 0.5),  # a brick shields only one vial; the right vial is unshielded
    ('the second brick stacked on the first', START_POSES, {
        'brick_1': {'pos': (0.34, 0.12, 0.7875)},
        'brick_2': {'pos': (0.34, 0.12, 0.8675)},
    }, 0.5),  # the stacked brick is not on the table and shields nothing else
    ('a brick standing off the line beside the left vial', START_POSES, {
        'brick_1': {'pos': (0.34, 0.24, 0.7875)},
    }, 0),  # the vial still has a direct line to the seat
    ('both bricks pushed off the table', START_POSES, {
        'brick_1': {'pos': (0.3, 0.8, 0.04)},
        'brick_2': {'pos': (0.4, 0.8, 0.04)},
    }, 0),  # off the table, they shield nothing
    ("red team: a brick nicking the left vial's line with one corner by about 1 mm", START_POSES, {
        'brick_1': {'pos': (0.34, 0.187, 0.7875)},
    }, 0),  # almost the whole brick is off to the side of the line; it is not "across" the line and blocks nothing beyond the central ray
    ("a brick crossing the left vial's line but with its centre 5 cm to the side", START_POSES, {
        'brick_1': {'pos': (0.3558, 0.1674, 0.7875)},
    }, 0),  # most of the brick is off to the side; its centre is more than 4 cm from the line
    ('red team: the left vial lifted 8 mm off the table and held in a gripper, both vials shielded', GOAL_POSES, {
        'vial_left': {'pos': (0.25, 0.15, 0.7805)},
    }, 0),  # the instruction says the vials must not be touched or moved; a vial hanging 8 mm above the table can only be held in a gripper, i.e. it has been picked up, yet the 1 cm move tolerance (meant for nudges) lets it score
    ('red team: the right vial lifted 9 mm off the table and held, the left shielded only', START_POSES, {
        'vial_right': {'pos': (0.25, -0.15, 0.7815)},
        'brick_1': {'pos': (0.34, 0.12, 0.7875)},
    }, 0),  # a hot vial picked up off the bench has been handled; nothing should count
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
