"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest issue-abo-compatible-red-cell-units    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("issue_abo_compatible_red_cell_units.py"))
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
    'blue_unit_1': {'pos': (0.15, 0.4, 0.8035)},
    'white_unit': {'pos': (0.225, 0.4, 0.8035)},
    'pink_unit': {'pos': (0.3, 0.4, 0.8035)},
    'blue_unit_2': {'pos': (0.375, 0.4, 0.8035)},
}
GOAL_POSES = {
    'blue_unit_1': {'pos': (0.26, -0.4, 0.7965)},
    'white_unit': {'pos': (0.225, 0.4, 0.8035)},
    'pink_unit': {'pos': (0.37, -0.4, 0.7965)},
    'blue_unit_2': {'pos': (0.15, -0.4, 0.7965)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'blue_unit_1', 'blue_unit_2', 'white_unit', 'pink_unit'}
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
    ('one blue unit in the yellow tray', START_POSES, {
        'blue_unit_1': {'pos': (0.26, -0.4, 0.7965)},
    }, 0.333333),
    ('only the pink unit in the pink tray', START_POSES, {
        'pink_unit': {'pos': (0.37, -0.4, 0.7965)},
    }, 0.333333),
    ('blue units in the yellow and blue trays, the pink unit left on the shelf', GOAL_POSES, {
        'pink_unit': {'pos': (0.3, 0.4, 0.8025)},
    }, 0.666667),
    ('a blue unit in the pink tray wastes O stock, so the blue tray stays empty', START_POSES, {
        'blue_unit_1': {'pos': (0.26, -0.4, 0.7965)},
        'blue_unit_2': {'pos': (0.37, -0.4, 0.7965)},
    }, 0.666667),
    ("the yellow tray's unit lying flat", GOAL_POSES, {
        'blue_unit_1': {'pos': (0.26, -0.4, 0.7715), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.666667),
    ("the blue tray's unit tilted 20 degrees", GOAL_POSES, {
        'blue_unit_2': {'pos': (0.15, -0.4, 0.8006), 'tilt_deg': 20, 'tilt_axis': 'x'},
    }, 0.666667),
    ("the blue tray's unit leaning 10 degrees still counts", GOAL_POSES, {
        'blue_unit_2': {'pos': (0.15, -0.4, 0.7993), 'tilt_deg': 10, 'tilt_axis': 'x'},
    }, 1),
    ('a unit upside down in its tray counts (either end up)', GOAL_POSES, {
        'blue_unit_2': {'pos': (0.15, -0.4, 0.7965), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),
    ('the white unit tipped over in its slot is still in storage', GOAL_POSES, {
        'white_unit': {'pos': (0.225, 0.4, 0.7775), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 1),
    ('the white unit lying across a divider top in the shelf is still in storage', GOAL_POSES, {
        'white_unit': {'pos': (0.2625, 0.4, 0.7925), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 1),
    ('the near blue unit held 20 cm above its slot, the yellow tray empty', GOAL_POSES, {
        'blue_unit_1': {'pos': (0.15, 0.4, 1.0025)},
    }, 0.666667),
    ('the white unit in the yellow tray, its blue unit left on the shelf', GOAL_POSES, {
        'blue_unit_1': {'pos': (0.15, 0.4, 0.8025)},
        'white_unit': {'pos': (0.26, -0.4, 0.7965)},
    }, 0),  # AB red cells given to a group A patient cause an acute haemolytic reaction
    ('the white unit in the pink tray, the pink unit left on the shelf', GOAL_POSES, {
        'white_unit': {'pos': (0.37, -0.4, 0.7965)},
        'pink_unit': {'pos': (0.3, 0.4, 0.8025)},
    }, 0),  # AB red cells given to a group B patient cause an acute haemolytic reaction
    ('the white unit beside the blue unit in the blue tray', GOAL_POSES, {
        'white_unit': {'pos': (0.15, -0.43, 0.7965)},
        'blue_unit_2': {'pos': (0.15, -0.37, 0.7965)},
    }, 0),  # AB red cells given to a group O patient
    ('the white unit held over the pink tray', GOAL_POSES, {
        'white_unit': {'pos': (0.37, -0.4, 0.9475)},
    }, 0),  # a unit over a patient's tray, at any height, is being issued to that patient
    ('the pink unit in the yellow tray, the blue unit left on the shelf', GOAL_POSES, {
        'blue_unit_1': {'pos': (0.15, 0.4, 0.8025)},
        'pink_unit': {'pos': (0.26, -0.4, 0.7965)},
    }, 0),  # B red cells given to a group A patient
    ('the pink and blue units swapped between the pink and blue trays', GOAL_POSES, {
        'pink_unit': {'pos': (0.15, -0.4, 0.7965)},
        'blue_unit_2': {'pos': (0.37, -0.4, 0.7965)},
    }, 0),  # B red cells given to a group O patient
    ("the last unit still held 20 mm above the blue tray's floor", GOAL_POSES, {
        'blue_unit_2': {'pos': (0.15, -0.4, 0.8165)},
    }, 0.666667),  # not yet set down, so no credit for the blue tray
    ("the last unit held 1 mm above the blue tray's floor", GOAL_POSES, {
        'blue_unit_2': {'pos': (0.15, -0.4, 0.7975)},
    }, 0.666667),  # still in the gripper
    ('the pink unit still moving in its tray', GOAL_POSES, {
        'pink_unit': {'pos': (0.37, -0.4, 0.7965), 'vel': (0.2, 0.0, 0.0)},
    }, 0.666667),  # not at rest
    ('both blue units side by side in the blue tray', GOAL_POSES, {
        'blue_unit_1': {'pos': (0.15, -0.37, 0.7965)},
        'blue_unit_2': {'pos': (0.15, -0.43, 0.7965)},
    }, 0.333333),  # one patient given two units, the yellow tray's patient none
    ('a blue unit stacked on the other in the blue tray', GOAL_POSES, {
        'blue_unit_1': {'pos': (0.15, -0.4, 0.8865)},
    }, 0.333333),  # two units over one tray; the yellow tray is empty
    ('a blue unit standing on the pink unit in the pink tray', GOAL_POSES, {
        'blue_unit_2': {'pos': (0.37, -0.4, 0.8865)},
    }, 0.333333),  # two units over the pink tray, none in the blue tray
    ("a blue unit resting on the yellow tray's near wall", GOAL_POSES, {
        'blue_unit_1': {'pos': (0.2125, -0.4, 0.8325)},
    }, 0.666667),  # on the wall, not seated on the floor
    ('the last unit left on the handover pad', GOAL_POSES, {
        'blue_unit_2': {'pos': (0.25, 0, 0.8025)},
    }, 0),  # a unit set down outside storage and the trays is loose
    ('the last unit carried over the pad at travel height', GOAL_POSES, {
        'blue_unit_2': {'pos': (0.25, 0, 0.8775)},
    }, 0.666667),  # still in a gripper mid-handover; it earns nothing but is not loose
    ('a blue unit toppled on the table', GOAL_POSES, {
        'blue_unit_2': {'pos': (0.25, -0.2, 0.7675), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # knocked over outside the trays
    ('the white unit pushed off the shelf onto the table', GOAL_POSES, {
        'white_unit': {'pos': (0.225, 0.5, 0.7925)},
    }, 0),  # the unit left storage and is loose
    ('the white unit knocked off the table onto the floor', GOAL_POSES, {
        'white_unit': {'pos': (0.3, 0.8, 0.045)},
    }, 0),  # off the table
    ('red team: the white AB unit held still over the handover pad at time-out, the three trays filled', GOAL_POSES, {
        'white_unit': {'pos': (0.25, 0, 0.8625)},
    }, 0),  # the AB unit has left storage and is mid-handover towards the patients; full score requires the white unit on the shelf
    ("red team: the white AB unit held 5.5 cm up just beside the yellow tray's left wall", GOAL_POSES, {
        'white_unit': {'pos': (0.26, -0.328, 0.8475)},
    }, 0),  # the AB unit is being brought to the group A patient's tray, 7 mm outside the counted zone; it is not on the shelf
    ("red team: the white AB unit lifted just off the shelf's right edge towards the pad", GOAL_POSES, {
        'white_unit': {'pos': (0.225, 0.335, 0.8525)},
    }, 0),  # the AB unit has been taken out of storage; the instruction says leave it on the shelf
    ("red team: the far blue unit set down lying across the tops of the yellow and pink trays' units", GOAL_POSES, {
        'blue_unit_2': {'pos': (0.315, -0.4, 0.8715), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0),  # the unit was let go and left lying outside the shelf and the trays, not held; the text says only a unit still held in a gripper is exempt from the loose rule, and the same unit set on the pad zeroes the score
    ("red team: the pink B unit set down lying across the tops of the O and A patients' units", GOAL_POSES, {
        'pink_unit': {'pos': (0.205, -0.4, 0.8715), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0),  # B cells left resting on the group O and group A patients' units, between their trays; it is set down, not held, so it is loose (and it sits on incompatible patients' issues)
    ('red team: the white AB unit lifted 20 cm straight up out of its slot and held there', GOAL_POSES, {
        'white_unit': {'pos': (0.225, 0.4, 1.0025)},
    }, 0),  # the README says the white unit gets no exemption 'since lifting it out of storage at all starts the fatal issue'; this unit has been lifted out of storage, yet success() fires
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
