"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest break-down-delivery-at-clean-line    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("break_down_delivery_at_clean_line.py"))
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
    'box_blue': {'pos': (0.235, 0.305, 0.781)},
    'box_green': {'pos': (0.3, 0.305, 0.781)},
    'box_orange': {'pos': (0.365, 0.305, 0.781)},
    'insert': {'pos': (0.3, 0.305, 0.8245)},
}
GOAL_POSES = {
    'box_blue': {'pos': (0.365, -0.305, 0.835)},
    'box_green': {'pos': (0.3, -0.305, 0.835)},
    'box_orange': {'pos': (0.235, -0.305, 0.835)},
    'insert': {'pos': (0.32, 0.145, 0.7645)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'box_blue', 'box_orange', 'insert', 'box_green'}
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
    ('the insert flat on the breakdown pad, the boxes still in the carton', START_POSES, {
        'insert': {'pos': (0.32, 0.145, 0.7645)},
    }, 0.1),
    ('the insert on the pad and the blue box on the shelf', START_POSES, {
        'box_blue': {'pos': (0.365, -0.305, 0.835)},
        'insert': {'pos': (0.32, 0.145, 0.7645)},
    }, 0.4),
    ('the orange box left on the pass-through', GOAL_POSES, {
        'box_orange': {'pos': (0.25, 0, 0.795)},
    }, 0.7),
    ('the insert set on the table on the dirty side, off the pad', GOAL_POSES, {
        'insert': {'pos': (0.12, 0.45, 0.7625)},
    }, 0.9),
    ('the insert on its side on the pad', GOAL_POSES, {
        'insert': {'pos': (0.32, 0.145, 0.7795), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.9),
    ('the green box set on the clean-side table instead of the shelf', GOAL_POSES, {
        'box_green': {'pos': (0.3, -0.15, 0.775)},
    }, 0.7),
    ('the blue box tilted 20 degrees against its neighbour on the shelf', GOAL_POSES, {
        'box_blue': {'pos': (0.365, -0.305, 0.8427), 'tilt_deg': 20, 'tilt_axis': 'x'},
    }, 0.7),
    ('red team: the insert flipped upside down, flat on the pad', GOAL_POSES, {
        'insert': {'pos': (0.32, 0.145, 0.7645), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # a rectangular block turned over is just as flat on the pad; the 15-degree flat rule measures the angle from its original up axis, so a flipped insert reads 180 degrees
    ('red team: the insert flipped end over end, flat on the pad', GOAL_POSES, {
        'insert': {'pos': (0.32, 0.145, 0.7645), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'y'},
    }, 1),  # same as above, turned over about its short axis; it lies flat on the pad
    ("red team: the insert yawed 45 degrees on the pad, clear of the pass-through's corner", GOAL_POSES, {
        'insert': {'pos': (0.39, 0.14, 0.7645), 'yaw_deg': -45},
    }, 1),  # the insert's real outline stays 2-3 cm clear of the pass-through (its nearest edge is the line x + y = 0.488, the pass-through corner is at x + y = 0.40); only its world-aligned bounding box overlaps the outline, so nothing has crossed, and its centre is on the pad, flat, so it counts as on the pad
    ("red team: the blue box yawed 45 degrees with its corner over the shelf's far edge", GOAL_POSES, {
        'box_blue': {'pos': (0.375, -0.305, 0.835), 'yaw_deg': 45},
    }, 0.7),  # turned 45 degrees, the box's corner reaches 3.9 cm from its centre, 0.4 cm past the shelf's far edge; the real outline is used, so it is not on the shelf
    ("red team: the blue box yawed 45 degrees just inside the shelf's far edge", GOAL_POSES, {
        'box_blue': {'pos': (0.37, -0.305, 0.835), 'yaw_deg': 45},
    }, 1),  # turned 45 degrees, the corner reaches 3.9 cm, 1 mm inside the far edge; the whole outline is on the shelf
    ('red team: the blue box upside down on the shelf', GOAL_POSES, {
        'box_blue': {'pos': (0.365, -0.305, 0.835), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # a cube in one colour looks identical upside down; a photo or a phone level can't tell it from upright
    ('the insert carried to the clean shelf, lying on the boxes', GOAL_POSES, {
        'insert': {'pos': (0.3, -0.305, 0.8775)},
    }, 0),  # brown corrugated cardboard in clean storage is the contamination the rule exists to stop
    ('the insert left on the pass-through', GOAL_POSES, {
        'insert': {'pos': (0.25, 0, 0.7825)},
    }, 0),  # the pass-through is the line itself; cardboard on it is handed into clean storage
    ('the insert on the dirty half of the pass-through', GOAL_POSES, {
        'insert': {'pos': (0.25, 0.04, 0.7825)},
    }, 0),  # still on the pass-through
    ('the insert knocked off the table on the clean side', GOAL_POSES, {
        'insert': {'pos': (0.3, -0.7, 0.015)},
    }, 0),  # cardboard past the line, on the clean-side floor
    ('a box tipped onto an edge on the shelf, 45 degrees', GOAL_POSES, {
        'box_blue': {'pos': (0.365, -0.305, 0.8464), 'tilt_deg': 45, 'tilt_axis': 'x'},
    }, 0.7),  # a box knocked onto its edge earns nothing for being on the shelf (a cube turned onto any face stands as well as upright, so only a box not lying on a face is toppled)
    ('the insert held in the air above the pass-through', GOAL_POSES, {
        'insert': {'pos': (0.25, 0.05, 0.8675)},
    }, 0),  # cardboard carried over the pass-through is cardboard at the line
    ('a box pushed off the table', GOAL_POSES, {
        'box_orange': {'pos': (0.3, -0.7, 0.0275)},
    }, 0.7),  # a box on the floor earns nothing
    ('a box held in the air above the shelf', GOAL_POSES, {
        'box_green': {'pos': (0.3, -0.305, 0.885)},
    }, 0.7),  # not set down on the shelf
    ('the boxes on the shelf with the insert still in the carton', START_POSES, {
        'box_blue': {'pos': (0.365, -0.305, 0.835)},
        'box_green': {'pos': (0.3, -0.305, 0.835)},
        'box_orange': {'pos': (0.235, -0.305, 0.835)},
        'insert': {'pos': (0.3, 0.305, 0.7675)},
    }, 0.9),  # the breakdown is unfinished, so no full score
    ('red team: the green box still in the gripper, moving, 8 mm above its slot at the time limit', GOAL_POSES, {
        'box_green': {'pos': (0.3, -0.305, 0.843), 'vel': (0.0, 0.0, 0.08)},
    }, 0.7),  # rebuttal: the finding is right that 1.0 was wrong, and score() now requires every credited object at rest (under 1 cm/s and 0.5 rad/s). But a box still being carried has not been set down, so it earns nothing for itself: 0.3 x 2 + 0.1 = 0.7, the same as the existing "box held in the air above the shelf" layout. Capping the total at 0.9 instead would give the moving box credit it has not earned.
    ("red team: the green box held still in the gripper 8 mm above its slot, the demo's release height", GOAL_POSES, {
        'box_green': {'pos': (0.3, -0.305, 0.843)},
    }, 0.7),  # the demo releases boxes 8 mm up; a policy that stops at the release point without opening the gripper has not put the box on the shelf
    ('red team: the green box held still in the gripper 9 mm above its slot on the shelf', GOAL_POSES, {
        'box_green': {'pos': (0.3, -0.305, 0.844)},
    }, 0.7),  # the box has not been set down; the gripper is holding it in the air above the shelf, and the README says a box in the air or still being carried earns nothing, yet a held-still box within the 1 cm height tolerance counts as on the shelf (and success fires if held 2.5 s)
    ('red team: the insert held still in the gripper 9 mm above the pad', GOAL_POSES, {
        'insert': {'pos': (0.32, 0.145, 0.7735)},
    }, 0.9),  # the insert has not been put on the pad; it is held in the air above it
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
