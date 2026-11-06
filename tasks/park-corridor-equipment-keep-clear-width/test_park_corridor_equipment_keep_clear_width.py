"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest park-corridor-equipment-keep-clear-width    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("park_corridor_equipment_keep_clear_width.py"))
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
    'deep_cart_1': {'pos': (0.2, -0.3, 0.7785)},
    'deep_cart_2': {'pos': (0.2, -0.42, 0.7785)},
    'shallow_cart_1': {'pos': (0.2, 0.3, 0.7785)},
    'shallow_cart_2': {'pos': (0.2, 0.42, 0.7785)},
}
GOAL_POSES = {
    'deep_cart_1': {'pos': (0.359, 0.156, 0.7785)},
    'deep_cart_2': {'pos': (0.359, -0.15, 0.7785)},
    'shallow_cart_1': {'pos': (0.147, -0.156, 0.7785)},
    'shallow_cart_2': {'pos': (0.147, 0.15, 0.7785)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'deep_cart_2', 'shallow_cart_2', 'shallow_cart_1', 'deep_cart_1'}
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
    ('one green cart parked in the left near bay', START_POSES, {
        'shallow_cart_2': {'pos': (0.144, 0.15, 0.7785)},
    }, 0.25),
    ('each arm parks one of its own carts, no handover', START_POSES, {
        'deep_cart_2': {'pos': (0.356, -0.15, 0.7785)},
        'shallow_cart_2': {'pos': (0.144, 0.15, 0.7785)},
    }, 0.5),
    ('three carts parked, the fourth green cart still in its room', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.2, 0.3, 0.7775)},
    }, 0.75),
    ('only the two green carts parked, facing each other in the left half (192 mm clear)', START_POSES, {
        'shallow_cart_1': {'pos': (0.376, 0.15, 0.7785)},
        'shallow_cart_2': {'pos': (0.144, 0.15, 0.7785)},
    }, 0.5),
    ('a blue cart 9 mm off its wall still counts (147 mm clear)', GOAL_POSES, {
        'deep_cart_1': {'pos': (0.351, 0.15, 0.7785)},
    }, 1),
    ('a green cart turned 8 degrees against its wall still counts', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.147, -0.15, 0.7785), 'yaw_deg': 8},
    }, 1),
    ('a green cart upside down in its bay counts (the stand-in box has no top or bottom)', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.144, -0.15, 0.7785), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),
    ('a green cart left on the table behind the far wall, outside the corridor', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.47, -0.1, 0.7775)},
    }, 0.75),
    ('red team: both left-half carts turned 10 degrees the same way, offset along the wall, 148 mm actually clear', GOAL_POSES, {
        'deep_cart_1': {'pos': (0.3525, 0.16, 0.7785), 'yaw_deg': 10},
        'shallow_cart_2': {'pos': (0.1478, 0.137, 0.7785), 'yaw_deg': 10},
    }, 1),  # both carts are parked within the stated tolerances (turned 10 degrees, nearest point 2 mm off the wall) and their facing sides are parallel; the blue cart's deepest corner sits where the green cart is shallow, so the gap between them is 148 mm at every point along the corridor, above the 140 mm minimum; the scorer subtracts each cart's deepest reach wherever it is and reads 133 mm
    ('the naive layout, each arm parks its own pair (blue faces blue on the right, 112 mm clear)', GOAL_POSES, {
        'deep_cart_1': {'pos': (0.164, -0.15, 0.7785)},
        'shallow_cart_1': {'pos': (0.376, 0.15, 0.7785)},
    }, 0),  # a pinch point under 140 mm blocks every bed behind it
    ('only the two blue carts parked, face to face', START_POSES, {
        'deep_cart_1': {'pos': (0.164, -0.15, 0.7785)},
        'deep_cart_2': {'pos': (0.356, -0.15, 0.7785)},
    }, 0),  # 112 mm clear, a pinch point
    ('both blue carts turned 90 degrees so they stand 70 mm deep, face to face', START_POSES, {
        'deep_cart_1': {'pos': (0.159, -0.15, 0.7785), 'yaw_deg': 90},
        'deep_cart_2': {'pos': (0.361, -0.15, 0.7785), 'yaw_deg': 90},
    }, 0),  # turned carts are not parked along the wall, so they obstruct the corridor
    ('the last green cart left on the handover pad', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.26, 0, 0.7785)},
    }, 0),  # a cart staged mid-corridor is an obstruction
    ('the last green cart still held 5 mm above its bay', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.144, -0.15, 0.7835)},
    }, 0),  # not standing on the floor, so not parked; it is in the corridor
    ('the last green cart still held 1.5 mm above its bay', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.144, -0.15, 0.78)},
    }, 0),  # more than 1 mm above the floor sheet, so not set down; it is in the corridor
    ('left-half carts turned 10 degrees toward each other, 134 mm clear at the corners', GOAL_POSES, {
        'deep_cart_1': {'pos': (0.3525, 0.15, 0.7785), 'yaw_deg': 10},
        'shallow_cart_2': {'pos': (0.1478, 0.15, 0.7785), 'yaw_deg': 170},
    }, 0),  # each cart is within the 10 mm and 10 degree tolerances, but the blue cart's nearest corner faces the green cart's farthest side, leaving 134 mm at the narrowest point, a pinch
    ('the last green cart still held 20 mm above its bay', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.144, -0.15, 0.7985)},
    }, 0),  # not parked; it is in the corridor
    ('the last green cart still sliding at 0.2 m/s', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.144, -0.15, 0.7785), 'vel': (0.2, 0.0, 0.0)},
    }, 0),  # a moving cart is not parked
    ('a green cart lying on its side in its bay', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.144, -0.15, 0.7835), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # knocked over
    ('a green cart stood on its end in its bay', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.154, -0.15, 0.7685), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0),  # knocked over
    ('three parked, the fourth knocked over in its room', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.2, 0.3, 0.7825), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # a tipped cart is damaged equipment and a hazard
    ('three parked, the fourth pushed off the table', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.2, 0.7, 0.03)},
    }, 0),  # equipment knocked off the table
    ('a green cart reaching 10 mm into the red door zone', GOAL_POSES, {
        'shallow_cart_2': {'pos': (0.144, 0.125, 0.7785)},
    }, 0),  # blocks a door zone
    ('a green cart stacked on the blue cart in the right far bay', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.356, -0.15, 0.8385)},
    }, 0),  # not on the floor of a bay; it obstructs the corridor
    ('a green cart resting on an end stop', GOAL_POSES, {
        'shallow_cart_2': {'pos': (0.144, 0.19, 0.8025)},
    }, 0),  # not on the floor of a bay
    ('a green cart resting on top of the near wall', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.11, -0.15, 0.8275)},
    }, 0),  # not parked; it reaches into the corridor
    ('a green cart 15 mm off its wall', GOAL_POSES, {
        'shallow_cart_2': {'pos': (0.155, 0.15, 0.7785)},
    }, 0),  # not flush to the wall, so it is an obstruction
    ('a green cart turned 45 degrees in its bay', GOAL_POSES, {
        'shallow_cart_2': {'pos': (0.16, 0.15, 0.7785), 'yaw_deg': 45},
    }, 0),  # not along the wall
    ('the last green cart carried mid-corridor at time-out', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.26, -0.05, 0.8675)},
    }, 0),  # a cart in the corridor that is not parked is an obstruction
    ('red team: the last green cart still held 2.5 mm above its bay at time-out counts as parked', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.144, -0.15, 0.781)},
    }, 0),  # the scoring text says a cart still held in the corridor is an obstruction and zeroes the run; a cart hanging 2.5 mm off the floor is held
    ('red team: the last green cart still held 2 mm above its bay at time-out', GOAL_POSES, {
        'shallow_cart_1': {'pos': (0.144, -0.15, 0.7805)},
    }, 0),  # held off the floor, so still held; the text says that zeroes the run
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
