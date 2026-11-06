"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest split-parcels-under-tote-limit    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("split_parcels_under_tote_limit.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 1400


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
    'parcel_red': {'pos': (0.2, 0.035, 0.7835)},
    'parcel_blue': {'pos': (0.31, -0.075, 0.7835)},
    'parcel_green': {'pos': (0.2, -0.035, 0.7835)},
    'parcel_yellow': {'pos': (0.31, 0.075, 0.7785)},
    'parcel_orange': {'pos': (0.2, 0.105, 0.7785)},
}
GOAL_POSES = {
    'parcel_red': {'pos': (0.32, -0.335, 0.7815)},
    'parcel_blue': {'pos': (0.32, -0.25, 0.7815)},
    'parcel_green': {'pos': (0.32, 0.365, 0.7815)},
    'parcel_yellow': {'pos': (0.32, 0.24, 0.7765)},
    'parcel_orange': {'pos': (0.32, 0.3, 0.7765)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'parcel_red', 'parcel_orange', 'parcel_blue', 'parcel_green', 'parcel_yellow'}
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
    ('the yellow parcel packed in the left tote', START_POSES, {
        'parcel_yellow': {'pos': (0.32, 0.24, 0.7765)},
    }, 0.2),
    ('the red and blue parcels packed in the right tote', START_POSES, {
        'parcel_red': {'pos': (0.32, -0.335, 0.7815)},
        'parcel_blue': {'pos': (0.32, -0.25, 0.7815)},
    }, 0.4),
    ('four parcels packed under both limits, the green parcel left on the mat', GOAL_POSES, {
        'parcel_green': {'pos': (0.2, -0.035, 0.7825)},
    }, 0.8),
    ('the yellow parcel left tipped 45 degrees on an edge inside the left tote', GOAL_POSES, {
        'parcel_yellow': {'pos': (0.32, 0.24, 0.7869), 'tilt_deg': 45, 'tilt_axis': 'x'},
    }, 0.8),
    ('three parcels in a dead end (red and yellow in the left tote, green in the right), the rest on the mat', START_POSES, {
        'parcel_red': {'pos': (0.32, 0.365, 0.7815)},
        'parcel_green': {'pos': (0.32, -0.305, 0.7815)},
        'parcel_yellow': {'pos': (0.32, 0.24, 0.7765)},
    }, 0.3),
    ('the orange parcel stacked on the yellow one, above the rim', GOAL_POSES, {
        'parcel_orange': {'pos': (0.32, 0.24, 0.8265)},
    }, 0.8),
    ("red team: the green cube rolled a quarter turn onto its side, flat on the left tote's floor", GOAL_POSES, {
        'parcel_green': {'pos': (0.32, 0.365, 0.7815), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 1),  # an unmarked 6 cm cube resting flat on another face is square and fully inside the tote; a photo or tape measure cannot tell it from upright
    ("red team: the blue cube upside down, flat on the right tote's floor", GOAL_POSES, {
        'parcel_blue': {'pos': (0.32, -0.25, 0.7815), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # an unmarked cube flipped over is indistinguishable from upright and sits fully inside the tote
    ('red team: the orange cube on its side and the yellow upside down, both flat in the left tote', GOAL_POSES, {
        'parcel_yellow': {'pos': (0.32, 0.24, 0.7765), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'y'},
        'parcel_orange': {'pos': (0.32, 0.3, 0.7765), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 1),  # both cubes sit square and flat in the tote; the split is right
    ('each arm loads the parcels nearest its tote (red, yellow and orange in the left tote, 8 kg)', START_POSES, {
        'parcel_red': {'pos': (0.32, 0.365, 0.7815)},
        'parcel_blue': {'pos': (0.32, -0.25, 0.7815)},
        'parcel_green': {'pos': (0.32, -0.335, 0.7815)},
        'parcel_yellow': {'pos': (0.32, 0.24, 0.7765)},
        'parcel_orange': {'pos': (0.32, 0.3, 0.7765)},
    }, 0),  # the left tote holds 8 kg, over the 7 kg limit
    ("four parcels packed, the red one resting on the left tote's wall", GOAL_POSES, {
        'parcel_red': {'pos': (0.32, 0.4075, 0.8575)},
    }, 0),  # a parcel on the rim still loads the tote, which then holds 11 kg
    ('the red parcel knocked over on the mat', START_POSES, {
        'parcel_red': {'pos': (0.2, 0.035, 0.7875), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # knocking a parcel over packs nothing
    ('everything packed but the orange parcel pushed onto the floor', GOAL_POSES, {
        'parcel_orange': {'pos': (0.3, 0, 0.025)},
    }, 0),  # a parcel on the floor is a dropped shipment
    ('four parcels packed, the orange one pushed off the mat onto the table', GOAL_POSES, {
        'parcel_orange': {'pos': (0.2, 0.2, 0.7725)},
    }, 0.8),  # a parcel left out earns nothing but costs nothing else
    ('red team: nearest-tote loading with the orange parcel left on the mat (red and yellow left, blue and green right)', START_POSES, {
        'parcel_red': {'pos': (0.32, 0.365, 0.7815)},
        'parcel_blue': {'pos': (0.32, -0.25, 0.7815)},
        'parcel_green': {'pos': (0.32, -0.335, 0.7815)},
        'parcel_yellow': {'pos': (0.32, 0.24, 0.7765)},
    }, 0.4),  # any two parcels weigh at most 7 kg, so two per tote scores 0.8 with no weight reasoning at all; this is the nearest-tote mistake the context says the task catches, stopped one parcel short
    ('red team: two parcels per tote chosen by colour alone (red and orange left, blue and yellow right), green left out', START_POSES, {
        'parcel_red': {'pos': (0.32, 0.365, 0.7815)},
        'parcel_blue': {'pos': (0.32, -0.25, 0.7815)},
        'parcel_yellow': {'pos': (0.32, -0.335, 0.7765)},
        'parcel_orange': {'pos': (0.32, 0.275, 0.7765)},
    }, 0.4),  # a 2+2 split never exceeds 7 kg whatever the weights, so 0.8 is earned without using the stated weights
    ("red team: the red parcel still held in the gripper hovering 1 cm above the right tote's floor", GOAL_POSES, {
        'parcel_red': {'pos': (0.32, -0.335, 0.7915)},
    }, 0.8),  # a parcel still in the gripper's hand is not packed into the tote; score gives 1.0 while success is false
    ("red team: the yellow parcel held 2.5 cm above the left tote's floor, top just under the rim", GOAL_POSES, {
        'parcel_yellow': {'pos': (0.32, 0.24, 0.8015)},
    }, 0.8),  # held in mid-air inside the tote, not set down; scorer counts it packed
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
