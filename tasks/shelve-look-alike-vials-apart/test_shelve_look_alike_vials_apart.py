"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest shelve-look-alike-vials-apart    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("shelve_look_alike_vials_apart.py"))
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
    'red_carton': {'pos': (0.2, 0.3, 0.7785), 'yaw_deg': 90},
    'blue_carton': {'pos': (0.32, 0.3, 0.7785), 'yaw_deg': 90},
    'green_carton': {'pos': (0.2, -0.3, 0.7785), 'yaw_deg': 90},
    'yellow_carton': {'pos': (0.32, -0.3, 0.7785), 'yaw_deg': 90},
}
GOAL_POSES = {
    'red_carton': {'pos': (0.28, 0.13, 0.7815)},
    'blue_carton': {'pos': (0.28, 0.026, 0.7815)},
    'green_carton': {'pos': (0.28, -0.078, 0.7815), 'yaw_deg': 180},
    'yellow_carton': {'pos': (0.28, -0.13, 0.7815), 'yaw_deg': 180},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'yellow_carton', 'blue_carton', 'green_carton', 'red_carton'}
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
    ('the red carton shelved in pocket 1, facing the front', START_POSES, {
        'red_carton': {'pos': (0.28, 0.13, 0.7815)},
    }, 0.25),
    ('the red carton shelved in pocket 1 but turned sideways', START_POSES, {
        'red_carton': {'pos': (0.28, 0.13, 0.7815), 'yaw_deg': 90},
    }, 0.15),
    ('the red carton shelved facing the front within 15 degrees', START_POSES, {
        'red_carton': {'pos': (0.28, 0.13, 0.7815), 'yaw_deg': 15},
    }, 0.25),
    ('the red carton shelved but turned 30 degrees', START_POSES, {
        'red_carton': {'pos': (0.28, 0.13, 0.7815), 'yaw_deg': 30},
    }, 0.15),
    ('the red carton shelved facing the back (the name on the other wide face)', START_POSES, {
        'red_carton': {'pos': (0.28, 0.13, 0.7815), 'yaw_deg': 180},
    }, 0.25),
    ('the green and yellow cartons shelved facing the front', START_POSES, {
        'green_carton': {'pos': (0.28, -0.026, 0.7815)},
        'yellow_carton': {'pos': (0.28, -0.13, 0.7815)},
    }, 0.5),
    ('the look-alikes shelved at the two ends of the row', START_POSES, {
        'red_carton': {'pos': (0.28, 0.13, 0.7815)},
        'blue_carton': {'pos': (0.28, -0.13, 0.7815)},
    }, 0.5),
    ('everything shelved except the blue carton, left at its start', GOAL_POSES, {
        'blue_carton': {'pos': (0.32, 0.3, 0.7775), 'yaw_deg': 90},
    }, 0.75),
    ('all four shelved, the blue carton turned sideways', GOAL_POSES, {
        'blue_carton': {'pos': (0.28, 0.026, 0.7815), 'yaw_deg': 90},
    }, 0.9),
    ('all four shelved in pockets 1, 3, 5 and 6', GOAL_POSES, {
        'green_carton': {'pos': (0.28, -0.078, 0.7815)},
    }, 1),
    ('red team: blue carton held 15 cm up over pocket 2 on its way to pocket 3, the other three shelved', GOAL_POSES, {
        'blue_carton': {'pos': (0.28, 0.078, 0.9275), 'yaw_deg': 45},
    }, 0.75),  # the blue carton is in the gripper in transit, not on the shelf; red, green and yellow are properly shelved
    ('the look-alikes in neighbouring pockets 1 and 2', START_POSES, {
        'red_carton': {'pos': (0.28, 0.13, 0.7815)},
        'blue_carton': {'pos': (0.28, 0.078, 0.7815)},
    }, 0),  # side by side, one is picked for the other
    ('all four shelved facing the front, the look-alikes in pockets 1 and 2', GOAL_POSES, {
        'blue_carton': {'pos': (0.28, 0.078, 0.7815)},
    }, 0),  # the look-alikes are neighbours
    ('the look-alikes in pockets 3 and 4, across the middle of the row', GOAL_POSES, {
        'red_carton': {'pos': (0.28, 0.026, 0.7815)},
        'blue_carton': {'pos': (0.28, -0.026, 0.7815)},
        'green_carton': {'pos': (0.28, 0.13, 0.7815)},
    }, 0),  # the look-alikes are neighbours
    ('the blue carton lying across pocket 2, next to the red carton in pocket 1', START_POSES, {
        'red_carton': {'pos': (0.28, 0.13, 0.7815)},
        'blue_carton': {'pos': (0.28, 0.078, 0.7835), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # a toppled look-alike next to its pair is still side by side
    ('the blue carton standing on the green carton in pocket 2, next to the red carton in pocket 1', START_POSES, {
        'red_carton': {'pos': (0.28, 0.13, 0.7815)},
        'blue_carton': {'pos': (0.28, 0.078, 0.8415)},
        'green_carton': {'pos': (0.28, 0.078, 0.7815)},
    }, 0),  # the blue carton rests on the shelf next to its look-alike, 60 mm above the floor, under the 70 mm cut-off
    ('the blue carton held low over pocket 2, bottom 20 mm above the floor, next to the red carton in pocket 1', GOAL_POSES, {
        'blue_carton': {'pos': (0.28, 0.078, 0.8015)},
    }, 0),  # dipped into the neighbouring pocket below the 70 mm cut-off, it counts as side by side
    ('the red carton lying on its side over pocket 1', START_POSES, {
        'red_carton': {'pos': (0.28, 0.13, 0.7755), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0),  # toppled, not shelved
    ('the red carton leaning 20 degrees in pocket 1', START_POSES, {
        'red_carton': {'pos': (0.28, 0.13, 0.7865), 'tilt_deg': 20, 'tilt_axis': 'x'},
    }, 0),  # not upright
    ('the green carton pushed off the table', START_POSES, {
        'green_carton': {'pos': (0.3, -0.8, 0.012), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0),  # off the table is not shelved
    ('the green carton standing on top of the red carton in pocket 1', START_POSES, {
        'red_carton': {'pos': (0.28, 0.13, 0.7815)},
        'green_carton': {'pos': (0.28, 0.13, 0.8415)},
    }, 0.25),  # only the red carton is shelved; the green one is not on the pocket floor
    ('the yellow carton standing on the lips between pockets 5 and 6', START_POSES, {
        'yellow_carton': {'pos': (0.28, -0.104, 0.7935)},
    }, 0),  # on the rack, not in a pocket
    ('red team: blue standing on the green carton stacked on the yellow carton in pocket 2, red shelved in pocket 1', START_POSES, {
        'red_carton': {'pos': (0.28, 0.13, 0.7815)},
        'blue_carton': {'pos': (0.28, 0.078, 0.9015)},
        'green_carton': {'pos': (0.28, 0.078, 0.7815)},
        'yellow_carton': {'pos': (0.28, 0.078, 0.8415)},
    }, 0),  # the blue look-alike is left resting on the shelf in the pocket next to the red one, which the instruction says never to do; the 70 mm exemption is meant for cartons in the gripper
    ('red team: blue standing on the yellow carton balanced on the divider between pockets 2 and 3, red in pocket 1, green in pocket 4', START_POSES, {
        'red_carton': {'pos': (0.28, 0.13, 0.7815)},
        'blue_carton': {'pos': (0.28, 0.06, 0.8535)},
        'green_carton': {'pos': (0.28, -0.026, 0.7815)},
        'yellow_carton': {'pos': (0.28, 0.06, 0.7935)},
    }, 0),  # blue rests on the rack over pocket 2, next to red; a single carton standing on a divider lifts its bottom to 72 mm and escapes the rule
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
