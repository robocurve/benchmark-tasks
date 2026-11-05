"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest induct-parcels-single-file-gapped    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("induct_parcels_single_file_gapped.py"))
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
    'brown_parcel': {'pos': (0.205, 0.475, 0.7725)},
    'orange_parcel': {'pos': (0.21, 0.475, 0.816)},
    'red_parcel': {'pos': (0.3, 0.465, 0.7725), 'yaw_deg': 90},
    'purple_parcel': {'pos': (0.21, 0.565, 0.775), 'yaw_deg': -45},
}
GOAL_POSES = {
    'brown_parcel': {'pos': (0.3, 0.3, 0.7775), 'yaw_deg': -90},
    'orange_parcel': {'pos': (0.3, 0.07, 0.78), 'yaw_deg': -90},
    'red_parcel': {'pos': (0.3, -0.39, 0.7775), 'yaw_deg': 90},
    'purple_parcel': {'pos': (0.3, -0.16, 0.78), 'yaw_deg': -90},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'red_parcel', 'brown_parcel', 'orange_parcel', 'purple_parcel'}
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
    ('brown and orange cartons on the lane 12 cm apart', START_POSES, {
        'brown_parcel': {'pos': (0.3, 0.3, 0.7775), 'yaw_deg': 90},
        'orange_parcel': {'pos': (0.3, 0.07, 0.78), 'yaw_deg': 90},
    }, 0.25),
    ('brown and orange cartons on the lane 7 cm apart', START_POSES, {
        'brown_parcel': {'pos': (0.3, 0.3, 0.7775), 'yaw_deg': 90},
        'orange_parcel': {'pos': (0.3, 0.12, 0.78), 'yaw_deg': 90},
    }, 0.1),
    ('three cartons on the lane with full gaps, the red one still in the tote', START_POSES, {
        'brown_parcel': {'pos': (0.3, 0.3, 0.7775), 'yaw_deg': 90},
        'orange_parcel': {'pos': (0.3, 0.07, 0.78), 'yaw_deg': 90},
        'purple_parcel': {'pos': (0.3, -0.16, 0.78), 'yaw_deg': 90},
    }, 0.5),
    ('all four on the lane, the last gap only 6 cm', GOAL_POSES, {
        'red_parcel': {'pos': (0.3, -0.32, 0.7775), 'yaw_deg': 90},
    }, 0.6),
    ("all four on the lane, the red carton turned 30 degrees off the lane's length", GOAL_POSES, {
        'red_parcel': {'pos': (0.3, -0.39, 0.7775), 'yaw_deg': 60},
    }, 0.5),
    ('three cartons on the lane with full gaps, the red one on the floor', GOAL_POSES, {
        'red_parcel': {'pos': (0.3, 0.8, 0.02), 'yaw_deg': 90},
    }, 0.5),
    ('the red carton left on the staging pad', GOAL_POSES, {
        'red_parcel': {'pos': (0.15, 0, 0.7725), 'yaw_deg': 90},
    }, 0.5),
    ('red team: brown carton flipped upside down at its slot, otherwise the goal', GOAL_POSES, {
        'brown_parcel': {'pos': (0.3, 0.3, 0.7775), 'yaw_deg': 90, 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # an upside-down carton still lies flat on a largest face (the scoring text's own definition); the cartons have no labels, so which of the two largest faces is down is irrelevant
    ('red team: orange carton flipped upside down in the middle of the line, otherwise the goal', GOAL_POSES, {
        'orange_parcel': {'pos': (0.3, 0.07, 0.78), 'yaw_deg': -90, 'tilt_deg': 180, 'tilt_axis': 'y'},
    }, 1),  # flat on its largest face, long side along the lane, gaps unchanged; the text says nothing about which way up
    ('red team: three inducted with full gaps, red carton carried 20 cm above the lane over the purple one', GOAL_POSES, {
        'red_parcel': {'pos': (0.3, -0.16, 0.9775), 'yaw_deg': 90},
    }, 0.5),  # a carton still held in the gripper 20 cm up is not on the belt and cannot double-feed; the three laid cartons are the same progress as with red left in the tote
    ('red team: three inducted with full gaps, red carton carried 20 cm above the lane over the orange-purple gap', GOAL_POSES, {
        'red_parcel': {'pos': (0.3, -0.045, 0.9775), 'yaw_deg': 90},
    }, 0.5),  # a carton in transit high above the lane is not part of the line on the belt
    ('three inducted with full gaps, red carton held 3 cm above the lane between the orange and purple ones', GOAL_POSES, {
        'red_parcel': {'pos': (0.3, -0.045, 0.8075), 'yaw_deg': 90},
    }, 0.5),
    ('the orange carton held 1.5 cm above the brown one on the lane', GOAL_POSES, {
        'orange_parcel': {'pos': (0.3, 0.3, 0.835), 'yaw_deg': 90},
    }, 0),  # a carton hovering just over another counts as stacked on it (a double feed)
    ('brown and orange cartons side by side on the lane', GOAL_POSES, {
        'brown_parcel': {'pos': (0.265, 0.3, 0.7775), 'yaw_deg': 90},
        'orange_parcel': {'pos': (0.33, 0.3, 0.78), 'yaw_deg': 90},
    }, 0),  # two parcels side by side reach the scanner as one (a double feed)
    ('the orange carton stacked on the brown one on the lane', GOAL_POSES, {
        'orange_parcel': {'pos': (0.3, 0.3, 0.82), 'yaw_deg': 90},
    }, 0),  # a stacked pair is a double feed
    ('all four laid end to end, touching', START_POSES, {
        'brown_parcel': {'pos': (0.3, 0.3, 0.7775), 'yaw_deg': 90},
        'orange_parcel': {'pos': (0.3, 0.19, 0.78), 'yaw_deg': 90},
        'red_parcel': {'pos': (0.3, -0.005, 0.7775), 'yaw_deg': 90},
        'purple_parcel': {'pos': (0.3, 0.095, 0.78), 'yaw_deg': 90},
    }, 0),  # touching parcels reach the scanner as one item
    ('the brown carton toppled onto its side next to the orange one', START_POSES, {
        'brown_parcel': {'pos': (0.3, 0.3, 0.7875), 'tilt_deg': 90, 'tilt_axis': 'x'},
        'orange_parcel': {'pos': (0.3, 0.07, 0.78), 'yaw_deg': 90},
    }, 0),  # a parcel on its side and across the lane is not inducted, so the pair earns nothing
    ('all four pushed off the table onto the floor', START_POSES, {
        'brown_parcel': {'pos': (0.2, 0.8, 0.02)},
        'orange_parcel': {'pos': (0.3, 0.8, 0.0225)},
        'red_parcel': {'pos': (0.2, -0.8, 0.02), 'yaw_deg': 90},
        'purple_parcel': {'pos': (0.3, -0.8, 0.0225), 'yaw_deg': -45},
    }, 0),  # nothing is inducted
    ('all four over their places but still falling', GOAL_POSES, {
        'brown_parcel': {'pos': (0.3, 0.3, 0.7775), 'yaw_deg': -90, 'vel': (0.0, 0.0, -0.3)},
        'orange_parcel': {'pos': (0.3, 0.07, 0.78), 'yaw_deg': -90, 'vel': (0.0, 0.0, -0.3)},
        'red_parcel': {'pos': (0.3, -0.39, 0.7775), 'yaw_deg': 90, 'vel': (0.0, 0.0, -0.3)},
        'purple_parcel': {'pos': (0.3, -0.16, 0.78), 'yaw_deg': -90, 'vel': (0.0, 0.0, -0.3)},
    }, 0),  # a parcel counts only once it rests on the lane
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
