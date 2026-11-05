"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest mixed-pallet-level-layer-pairs    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("mixed_pallet_level_layer_pairs.py"))
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
    'blue_l': {'pos': (0.15, 0.38, 0.7685)},
    'green_l': {'pos': (0.25, 0.38, 0.7785)},
    'green_r': {'pos': (0.15, -0.3, 0.7785)},
    'blue_r': {'pos': (0.25, -0.3, 0.7685)},
    'orange_1': {'pos': (0.2, 0.165, 0.7635)},
    'orange_2': {'pos': (0.2, 0.27, 0.7635)},
}
GOAL_POSES = {
    'blue_l': {'pos': (0.19, 0.035, 0.7825), 'yaw_deg': -90},
    'green_l': {'pos': (0.29, 0.035, 0.7925), 'yaw_deg': -90},
    'green_r': {'pos': (0.29, -0.035, 0.7925), 'yaw_deg': 90},
    'blue_r': {'pos': (0.19, -0.035, 0.7825), 'yaw_deg': 90},
    'orange_1': {'pos': (0.19, 0, 0.8175), 'yaw_deg': -90},
    'orange_2': {'pos': (0.29, 0, 0.8375), 'yaw_deg': -90},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'green_l', 'orange_1', 'blue_r', 'blue_l', 'orange_2', 'green_r'}
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
    ('one blue carton upright on the deck', START_POSES, {
        'blue_l': {'pos': (0.19, 0.035, 0.7825)},
    }, 0.03),
    ('all four base cartons in the grid, the orange cartons not yet placed', START_POSES, {
        'blue_l': {'pos': (0.19, 0.035, 0.7825)},
        'green_l': {'pos': (0.29, 0.035, 0.7925)},
        'green_r': {'pos': (0.29, -0.035, 0.7925)},
        'blue_r': {'pos': (0.19, -0.035, 0.7825)},
    }, 0.12),
    ('one orange carton level on the blues, the other still at its start', GOAL_POSES, {
        'orange_2': {'pos': (0.2, 0.27, 0.7625), 'yaw_deg': -90},
    }, 0.56),
    ('the orange carton on the blues tilted 3 degrees', GOAL_POSES, {
        'orange_1': {'pos': (0.19, 0, 0.819), 'yaw_deg': 90, 'tilt_deg': 3, 'tilt_axis': 'x'},
    }, 0.78),
    ('the greens 25 mm apart under their orange carton', GOAL_POSES, {
        'green_r': {'pos': (0.29, -0.05, 0.7925), 'yaw_deg': 90},
    }, 0.56),
    ('the orange carton on the greens 18 mm past them along its length', GOAL_POSES, {
        'orange_2': {'pos': (0.29, 0.018, 0.8375), 'yaw_deg': 90},
    }, 0.56),
    ('the orange carton on the greens 12 mm off centre along its length still counts', GOAL_POSES, {
        'orange_2': {'pos': (0.29, 0.012, 0.8375), 'yaw_deg': 90},
    }, 1),
    ('orange carton tipped onto a green carton still at its start on the table keeps the credit on the deck', GOAL_POSES, {
        'green_r': {'pos': (0.15, -0.3, 0.7775), 'yaw_deg': 90},
        'orange_2': {'pos': (0.15, -0.3, 0.8336), 'yaw_deg': -90, 'tilt_deg': 10, 'tilt_axis': 'y'},
    }, 0.53),  # the zero rule is about a case on the layer; a green carton off the deck is not part of it
    ('red team: orange carton on the blues lying flat but upside down', GOAL_POSES, {
        'orange_1': {'pos': (0.19, 0, 0.8175), 'yaw_deg': 90, 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # an orange carton has no this-side-up marking; flat on its other big face it lies exactly as level across the two blues
    ('red team: orange carton on the greens lying flat but upside down', GOAL_POSES, {
        'orange_2': {'pos': (0.29, 0, 0.8375), 'yaw_deg': -90, 'tilt_deg': 180, 'tilt_axis': 'y'},
    }, 1),  # same as the blues case; the scorer reads 180 degrees as tipped and zeroes the whole layer
    ('red team: last orange carton carried tilted high above the greens at timeout', GOAL_POSES, {
        'orange_2': {'pos': (0.29, 0, 0.9425), 'yaw_deg': 90, 'tilt_deg': 10, 'tilt_axis': 'x'},
    }, 0.56),  # the bases and the first orange carton are done (0.56); the second is still in the gripper 9 cm above the layer, not lying over anything, yet the zero rule wipes the earned credit
    ('red team: last orange carton carried tilted above the blues at timeout', GOAL_POSES, {
        'orange_2': {'pos': (0.19, 0, 0.9525), 'yaw_deg': 90, 'tilt_deg': 20, 'tilt_axis': 'y'},
    }, 0.56),  # the carton in transit is 11 cm above the layer and resting on nothing; credit already earned should not drop to 0
    ('red team: green cube under its orange carton rolled a quarter turn in place', GOAL_POSES, {
        'green_l': {'pos': (0.29, 0.035, 0.7925), 'yaw_deg': -90, 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 1),  # the simulated green carton is a plain 6 cm cube with no arrows rendered, so this layout is pixel-identical to the goal; the policy cannot see which face is up
    ("each arm's own short and tall carton bridged, the orange cartons tilted", START_POSES, {
        'blue_l': {'pos': (0.19, 0.035, 0.7825)},
        'green_l': {'pos': (0.29, 0.035, 0.7925)},
        'green_r': {'pos': (0.29, -0.035, 0.7925)},
        'blue_r': {'pos': (0.19, -0.035, 0.7825)},
        'orange_1': {'pos': (0.24, 0.035, 0.8278), 'tilt_deg': 11, 'tilt_axis': 'y'},
        'orange_2': {'pos': (0.24, -0.035, 0.8278), 'tilt_deg': 11, 'tilt_axis': 'y'},
    }, 0),  # each orange carton rests on one edge across a short and a tall carton and would slide off in transit
    ('a blue carton turned on its side to stand in for a tall one', GOAL_POSES, {
        'blue_l': {'pos': (0.29, 0.025, 0.7925), 'yaw_deg': -90, 'tilt_deg': 90, 'tilt_axis': 'x'},
        'green_l': {'pos': (0.25, 0.38, 0.7775), 'yaw_deg': -90},
        'orange_1': {'pos': (0.2, 0.165, 0.7625), 'yaw_deg': -90},
    }, 0.06),  # a carton on its side is not upright, so it earns nothing and the orange carton on it earns nothing
    ('an orange carton tipped on its side across the greens', GOAL_POSES, {
        'orange_2': {'pos': (0.29, 0.02, 0.8525), 'yaw_deg': -90, 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # a tipped upper carton on the layer would slide off when the truck brakes
    ('a blue base carton toppled on its side on the deck', START_POSES, {
        'blue_l': {'pos': (0.19, 0.035, 0.7925), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # a blue carton on its side is 6 cm tall, not standing on a 6 x 6 cm face
    ('an orange carton tilted 10 degrees resting down on the blues', GOAL_POSES, {
        'orange_1': {'pos': (0.19, 0, 0.8287), 'yaw_deg': 90, 'tilt_deg': 10, 'tilt_axis': 'x'},
    }, 0),  # down on the layer and tilted past 5 degrees, it would slide off in transit
    ('two base cartons pushed off the table', START_POSES, {
        'blue_l': {'pos': (0.3, 0.8, 0.02)},
        'green_r': {'pos': (0.3, -0.8, 0.03)},
    }, 0),  # nothing is on the deck
    ('a blue carton stacked on a green one on the deck', START_POSES, {
        'blue_l': {'pos': (0.19, 0.035, 0.8425)},
        'green_l': {'pos': (0.19, 0.035, 0.7925)},
    }, 0.03),  # only the green stands on the deck; a stacked carton is not a base
    ('an orange carton lying flat on the deck', START_POSES, {
        'orange_1': {'pos': (0.24, 0, 0.7775)},
    }, 0),  # it rests on no base cartons
    ('red team: orange carton held level in the gripper 4 mm above the greens', GOAL_POSES, {
        'orange_2': {'pos': (0.29, 0, 0.8415), 'yaw_deg': 90},
    }, 0.56),  # it is still held, not laid on the greens; a carton the robot has not let go of does not form a layer
    ('red team: orange carton held still 1 mm above the greens', GOAL_POSES, {
        'orange_2': {'pos': (0.29, 0, 0.8385), 'yaw_deg': 90},
    }, 0.56),  # still in the gripper, not let go onto the greens; a held carton does not form a layer, yet score() is 1.0 and success() fires
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
