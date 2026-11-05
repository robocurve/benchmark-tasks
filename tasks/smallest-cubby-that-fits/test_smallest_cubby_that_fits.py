"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest smallest-cubby-that-fits    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("smallest_cubby_that_fits.py"))
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
    'small_parcel_1': {'pos': (0.18, 0.36, 0.7815)},
    'small_parcel_2': {'pos': (0.32, 0.36, 0.7815)},
    'medium_parcel': {'pos': (0.18, 0.24, 0.784)},
    'large_parcel': {'pos': (0.33, 0.24, 0.794)},
}
GOAL_POSES = {
    'small_parcel_1': {'pos': (0.27, -0.3375, 0.7815)},
    'small_parcel_2': {'pos': (0.27, -0.3975, 0.7815)},
    'medium_parcel': {'pos': (0.27, -0.2675, 0.784)},
    'large_parcel': {'pos': (0.27, -0.1625, 0.794)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'small_parcel_1', 'medium_parcel', 'large_parcel', 'small_parcel_2'}
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
    ('the large parcel seated in the large compartment', START_POSES, {
        'large_parcel': {'pos': (0.27, -0.1625, 0.794)},
    }, 0.25),
    ('the medium parcel seated in the medium compartment', START_POSES, {
        'medium_parcel': {'pos': (0.27, -0.2675, 0.784)},
    }, 0.25),
    ('one small parcel seated in a small compartment', START_POSES, {
        'small_parcel_2': {'pos': (0.27, -0.3375, 0.7815)},
    }, 0.25),
    ('both small parcels seated in the small compartments', START_POSES, {
        'small_parcel_1': {'pos': (0.27, -0.3975, 0.7815)},
        'small_parcel_2': {'pos': (0.27, -0.3375, 0.7815)},
    }, 0.5),
    ('three seated, the large parcel left on the handover mat', GOAL_POSES, {
        'large_parcel': {'pos': (0.22, 0, 0.793)},
    }, 0.75),
    ('a small parcel leaning 20 degrees in its compartment', GOAL_POSES, {
        'small_parcel_1': {'pos': (0.27, -0.3375, 0.7874), 'tilt_deg': 20, 'tilt_axis': 'y'},
    }, 0.75),
    ('the medium parcel lying on a side face in its compartment', GOAL_POSES, {
        'medium_parcel': {'pos': (0.27, -0.2675, 0.7815), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 1),
    ('three seated, the last small parcel carried 2 mm above the seated large parcel at 5 cm/s', GOAL_POSES, {
        'small_parcel_2': {'pos': (0.27, -0.1625, 0.8685), 'vel': (0.0, -0.05, 0.0)},
    }, 0.75),
    ('red team: three seated, the last small parcel carried in the gripper 10 cm above the large compartment', GOAL_POSES, {
        'small_parcel_2': {'pos': (0.25, -0.2025, 0.8815)},
    }, 0.75),  # the parcel is in transit over the large compartment on the way to its small one; it is not stored there, and the three seated parcels are real progress
    ('red team: three seated, the last small parcel carried in the gripper 10 cm above the medium compartment', GOAL_POSES, {
        'small_parcel_2': {'pos': (0.25, -0.2675, 0.8815)},
    }, 0.75),  # the carry route from the handover mat to the outer small compartment crosses the medium compartment; nothing is misallocated
    ('red team: the medium parcel carried 10 cm above the large compartment, the large parcel seated', START_POSES, {
        'medium_parcel': {'pos': (0.245, -0.1425, 0.919)},
        'large_parcel': {'pos': (0.27, -0.1625, 0.794)},
    }, 0.25),  # the large parcel is seated; the medium one is merely passing overhead on its way to the medium compartment
    ('red team: a small parcel upside down, flat on its compartment floor', GOAL_POSES, {
        'small_parcel_1': {'pos': (0.27, -0.3375, 0.7815), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # a closed box flipped onto its top stands flat on the floor; a level app on it reads 0 degrees, as the README prescribes measuring
    ('red team: the large parcel upside down, flat on its compartment floor', GOAL_POSES, {
        'large_parcel': {'pos': (0.27, -0.1625, 0.794), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # same box footprint, same height, flat on the floor; visually and by level app indistinguishable from the goal
    ('red team: the medium parcel carried 6 mm above the seated large parcel while crossing the large compartment', START_POSES, {
        'medium_parcel': {'pos': (0.27, -0.1625, 0.875)},
        'large_parcel': {'pos': (0.27, -0.1625, 0.794)},
    }, 0.25),  # the medium parcel is held well above the 3 cm rim on its way to the medium compartment; the scoring text says a parcel carried above the rim is in no compartment, and the large parcel is seated
    ('red team: three seated, the last small parcel carried 6 mm above the seated large parcel', GOAL_POSES, {
        'small_parcel_2': {'pos': (0.27, -0.1625, 0.8725)},
    }, 0.75),  # the carry from the handover mat to the outer small compartment crosses the large one; the parcel is in the gripper, not stacked, and three parcels are seated
    ('red team: three seated, the last small parcel carried 6 mm above the seated small parcel in the inner small compartment', GOAL_POSES, {
        'small_parcel_2': {'pos': (0.27, -0.3375, 0.8475)},
    }, 0.75),  # the route to the outer small compartment passes over the inner one; a parcel hovering in the gripper does not share the compartment, so the seated small parcel keeps its credit
    ('a small parcel in the large compartment, the large parcel left on the mat', GOAL_POSES, {
        'small_parcel_1': {'pos': (0.27, -0.1625, 0.7815)},
        'large_parcel': {'pos': (0.33, 0.24, 0.793)},
    }, 0),  # the greedy mistake the task is about; the small parcel wastes the only slot the large parcel fits
    ('a small parcel in the medium compartment, the medium parcel in the large one', START_POSES, {
        'small_parcel_1': {'pos': (0.27, -0.2675, 0.7815)},
        'small_parcel_2': {'pos': (0.27, -0.3975, 0.7815)},
        'medium_parcel': {'pos': (0.27, -0.1625, 0.784)},
    }, 0),  # two wasted slots; the large parcel has nowhere to go
    ('both small parcels side by side in the large compartment', START_POSES, {
        'small_parcel_1': {'pos': (0.27, -0.1325, 0.7815)},
        'small_parcel_2': {'pos': (0.27, -0.1925, 0.7815)},
    }, 0),  # dumping parcels into the biggest compartment is the misallocation the rule forbids
    ('a small parcel stacked on the other in one small compartment', GOAL_POSES, {
        'small_parcel_2': {'pos': (0.27, -0.3375, 0.8415)},
    }, 0.5),  # one parcel per compartment; neither of the stacked parcels counts
    ('the medium parcel knocked over, balanced on an edge at 45 degrees in its compartment', GOAL_POSES, {
        'medium_parcel': {'pos': (0.27, -0.2675, 0.7957), 'tilt_deg': 45, 'tilt_axis': 'y'},
    }, 0.75),  # a toppled parcel that is not flat on a face is not seated
    ('a small parcel resting on top of the seated large parcel', GOAL_POSES, {
        'small_parcel_2': {'pos': (0.27, -0.1625, 0.8665)},
    }, 0),  # left on top of the large parcel, the small parcel is stored in the large compartment, a slot it does not need
    ('a small parcel let go 2 cm above the large compartment floor, the large parcel left on the mat', GOAL_POSES, {
        'small_parcel_2': {'pos': (0.27, -0.1625, 0.8015)},
        'large_parcel': {'pos': (0.33, 0.24, 0.793)},
    }, 0),  # below the rim the small parcel is in the large compartment, wasting it, even before it reaches the floor
    ('the large parcel pushed off the table', GOAL_POSES, {
        'large_parcel': {'pos': (0.3, 0.75, 0.0425)},
    }, 0),  # a dropped parcel is damaged
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
