"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest insert-box-into-size-run    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("insert_box_into_size_run.py"))
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
    'red_box': {'pos': (0.26, -0.091, 0.7985)},
    'blue_box': {'pos': (0.26, 0.25, 0.7965)},
    'green_box': {'pos': (0.26, -0.152, 0.7985)},
}
GOAL_POSES = {
    'red_box': {'pos': (0.26, -0.091, 0.7985)},
    'blue_box': {'pos': (0.2625, -0.1535, 0.7975)},
    'green_box': {'pos': (0.2625, -0.216, 0.7975)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'green_box', 'red_box', 'blue_box'}
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
    ('the green box moved right, a slot open, the blue box still on the mat', START_POSES, {
        'green_box': {'pos': (0.26, -0.216, 0.7975)},
    }, 0.1),
    ('the green box moved right but only 4 cm, too little for the blue box', START_POSES, {
        'green_box': {'pos': (0.26, -0.192, 0.7975)},
    }, 0),
    ('the blue box in order but 2 cm from the red box', GOAL_POSES, {
        'blue_box': {'pos': (0.26, -0.171, 0.7975)},
        'green_box': {'pos': (0.26, -0.236, 0.7975)},
    }, 0.8),
    ('the row in order and closed, but the blue box 1 cm forward of the back stop', GOAL_POSES, {
        'blue_box': {'pos': (0.25, -0.1535, 0.7975)},
    }, 0.8),
    ('the row in order and against the back, but 1.5 cm from the left end stop', GOAL_POSES, {
        'red_box': {'pos': (0.26, -0.106, 0.7975)},
        'blue_box': {'pos': (0.26, -0.1685, 0.7975)},
        'green_box': {'pos': (0.26, -0.231, 0.7975)},
    }, 0.8),
    ('the blue box in order, 2 cm from the red box and 1 cm forward', GOAL_POSES, {
        'blue_box': {'pos': (0.25, -0.171, 0.7975)},
        'green_box': {'pos': (0.26, -0.236, 0.7975)},
    }, 0.6),
    ('the blue box in the slot turned 15 degrees, the row closed', GOAL_POSES, {
        'blue_box': {'pos': (0.252, -0.1555, 0.7975), 'yaw_deg': 15},
        'green_box': {'pos': (0.26, -0.221, 0.7975)},
    }, 0.8),
    ('the blue box in the slot turned a quarter turn, a 4 cm end against the back stop, the row closed', GOAL_POSES, {
        'blue_box': {'pos': (0.25, -0.143, 0.7975), 'yaw_deg': 90},
        'green_box': {'pos': (0.26, -0.195, 0.7975)},
    }, 0.8),
    ('the finished row', GOAL_POSES, {
    }, 1),
    ('the blue box parked at the end of the run, right of the green box', START_POSES, {
        'blue_box': {'pos': (0.26, -0.217, 0.7975)},
    }, 0),  # the failure mode the task is about; the size order is broken
    ('the green and blue boxes swapped', GOAL_POSES, {
        'blue_box': {'pos': (0.26, -0.216, 0.7975)},
        'green_box': {'pos': (0.26, -0.1535, 0.7975)},
    }, 0),  # the run reads small, large, medium
    ('the blue box standing on top of the red and green boxes', START_POSES, {
        'blue_box': {'pos': (0.26, -0.1215, 0.8885)},
    }, 0),  # on top of the row is not in it
    ('the red box knocked over in the channel, the rest in order', GOAL_POSES, {
        'red_box': {'pos': (0.235, -0.091, 0.7725), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0),  # a knocked-over box breaks the run
    ('the blue box knocked over on its mat, the slot open', START_POSES, {
        'blue_box': {'pos': (0.26, 0.25, 0.7805), 'tilt_deg': 90, 'tilt_axis': 'x'},
        'green_box': {'pos': (0.26, -0.216, 0.7975)},
    }, 0),  # knocking a box over earns nothing
    ('the green box pushed off the table', START_POSES, {
        'green_box': {'pos': (0.26, -0.7, 0.045)},
    }, 0),  # a box off the table has left the run
    ('the blue box pushed off the table, the slot open', START_POSES, {
        'blue_box': {'pos': (0.26, 0.7, 0.045)},
        'green_box': {'pos': (0.26, -0.216, 0.7975)},
    }, 0),  # dropping the return on the floor earns nothing
    ('the blue box held in the air over the slot', GOAL_POSES, {
        'blue_box': {'pos': (0.26, -0.1535, 0.8275)},
    }, 0),  # left mid-air over the run, it isn't in it
    ('red team: red box yawed 9 degrees, only its near back corner within 5 mm of the stop', GOAL_POSES, {
        'red_box': {'pos': (0.2516, -0.095, 0.7975), 'yaw_deg': 9},
    }, 0.8),  # the back face runs from 5 mm to about 14 mm off the stop; the text says the back face is within 5 mm
    ("red team: blue box leaning back 5 degrees on the back stop's top edge and turned 9 degrees, far back corner about 1 cm off", GOAL_POSES, {
        'blue_box': {'pos': (0.2575, -0.1555, 0.7991), 'yaw_deg': 9, 'tilt_deg': 5, 'tilt_axis': 'y'},
        'green_box': {'pos': (0.26, -0.22, 0.7975)},
    }, 0.8),  # the box rests on its back bottom edge and leans on the stop's top edge (the real box clears the stop; only its axis-aligned bounds reach over it). Its near back corner is 3 mm off the stop at the floor, but its far back corner is 12 mm off at the floor and 10 mm at the stop's top, so not both back corners are within 5 mm. Because the top overhangs the stop, the nearest-point estimate goes negative and hides the 9-degree turn.
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
