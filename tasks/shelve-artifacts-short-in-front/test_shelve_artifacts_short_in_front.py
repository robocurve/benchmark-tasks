"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest shelve-artifacts-short-in-front    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("shelve_artifacts_short_in_front.py"))
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
    'vase': {'pos': (0.2, 0.25, 0.8135)},
    'figurine': {'pos': (0.31, 0.25, 0.8035)},
    'bowl': {'pos': (0.185, 0.37, 0.7835)},
    'pot': {'pos': (0.28, 0.37, 0.7785)},
    'bead_box': {'pos': (0.36, 0.37, 0.7735)},
}
GOAL_POSES = {
    'vase': {'pos': (0.375, -0.23, 0.8275)},
    'figurine': {'pos': (0.375, -0.33, 0.8175)},
    'bowl': {'pos': (0.265, -0.22, 0.7975)},
    'pot': {'pos': (0.265, -0.31, 0.7925)},
    'bead_box': {'pos': (0.265, -0.39, 0.7875)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'pot', 'bead_box', 'bowl', 'figurine', 'vase'}
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
    ('the figurine shelved alone', START_POSES, {
        'figurine': {'pos': (0.375, -0.33, 0.8175)},
    }, 0.2),
    ('the back row shelved, vase and figurine', START_POSES, {
        'vase': {'pos': (0.375, -0.23, 0.8275)},
        'figurine': {'pos': (0.375, -0.33, 0.8175)},
    }, 0.4),
    ('the figurine shelved 1 cm from the front edge', START_POSES, {
        'figurine': {'pos': (0.235, -0.33, 0.8175)},
    }, 0.1),
    ('all five shelved, the bowl 1 cm from the front edge', GOAL_POSES, {
        'bowl': {'pos': (0.245, -0.22, 0.7975)},
    }, 0.9),
    ('all five shelved, the pot 1 cm from the bowl', GOAL_POSES, {
        'pot': {'pos': (0.265, -0.29, 0.7925)},
    }, 0.95),
    ('four shelved, the bead box left on the transfer pad', GOAL_POSES, {
        'bead_box': {'pos': (0.22, 0, 0.7725)},
    }, 0.8),
    ('red team: the vase held in the air above the front of the shelf, the other four shelved', GOAL_POSES, {
        'vase': {'pos': (0.23, -0.27, 0.9175)},
    }, 0.8),  # the vase is still being carried, not standing on the shelf, so it hides nothing yet; the four shelved artifacts are real progress and the scoring text defines being on the shelf by standing, lying or stacked
    ('red team: the bowl and pot side by side in the front row, the bowl 1.3 cm further forward, 1.5 cm apart', GOAL_POSES, {
        'bowl': {'pos': (0.257, -0.235, 0.7975)},
        'pot': {'pos': (0.27, -0.31, 0.7925)},
    }, 0.95),  # the bowl and pot stand side by side in the front row (their depth ranges overlap almost entirely), the pot fully visible beside the bowl; the only fault is the 1.5 cm gap, which the text prices at 0.05, yet a 1.3 cm stagger in depth zeroes all five artifacts' credit
    ('the figurine in front of the pot, in its lane', GOAL_POSES, {
        'figurine': {'pos': (0.265, -0.31, 0.8175)},
        'pot': {'pos': (0.375, -0.33, 0.7925)},
    }, 0),  # the pot is hidden from the aisle and can only be reached by lifting it over the figurine
    ('the bowl staggered in front of the pot, more than half of the pot behind it', GOAL_POSES, {
        'bowl': {'pos': (0.26, -0.25, 0.7975)},
        'pot': {'pos': (0.3, -0.31, 0.7925)},
    }, 0),  # the bowl's back face is 5 mm in front of the pot's centre, so more than half of the shorter pot stands behind the taller bowl in its lane and is hidden from the aisle
    ('the vase knocked flat in front of the bowl', GOAL_POSES, {
        'vase': {'pos': (0.27, -0.23, 0.7975), 'tilt_deg': 90, 'tilt_axis': 'y'},
        'bowl': {'pos': (0.375, -0.23, 0.7975)},
    }, 0),  # a fallen artifact still counts at its standing height, so knocking the vase over doesn't clear the bowl's lane
    ('the pot toppled on its side at its spot', GOAL_POSES, {
        'pot': {'pos': (0.265, -0.31, 0.7925), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.8),  # a toppled artifact earns nothing; the other four keep their credit
    ('the bead box pushed off the table', GOAL_POSES, {
        'bead_box': {'pos': (0.3, -0.72, 0.02)},
    }, 0.8),  # an artifact off the table earns nothing
    ("the bead box pushed off the shelf's front edge onto the table", GOAL_POSES, {
        'bead_box': {'pos': (0.15, -0.39, 0.7675)},
    }, 0.8),  # an artifact off the shelf earns nothing
    ('the bead box stacked on the bowl', GOAL_POSES, {
        'bead_box': {'pos': (0.265, -0.22, 0.8475)},
    }, 0.8),  # an artifact standing on another isn't on the shelf top and earns nothing
    ('red team: the pot stands directly in front of the bead box, hiding it', GOAL_POSES, {
        'pot': {'pos': (0.25, -0.395, 0.7925)},
        'bead_box': {'pos': (0.32, -0.395, 0.7875)},
    }, 0),  # the instruction says taller objects go behind shorter ones so none is hidden; the 5 cm pot (5 cm wide) stands squarely in front of the 4 cm bead box (4 cm wide), which cannot be seen from the aisle and must be reached past the pot
    ('red team: the bowl stands directly in front of the pot, hiding it', GOAL_POSES, {
        'figurine': {'pos': (0.375, -0.315, 0.8175)},
        'bowl': {'pos': (0.26, -0.39, 0.7975)},
        'pot': {'pos': (0.345, -0.39, 0.7925)},
        'bead_box': {'pos': (0.265, -0.31, 0.7875)},
    }, 0),  # the taller, wider bowl (6 cm tall, 7 cm across) stands in front of the pot (5 cm tall, 5 cm across) in the same lane, hiding it, against the instruction's taller-behind-shorter rule
    ('red team: the bead box still gripped 4 mm above its spot, the other four shelved', GOAL_POSES, {
        'bead_box': {'pos': (0.265, -0.39, 0.7915)},
    }, 0.8),  # the bead box is still held in the air and has not been set on the shelf, so only four artifacts are shelved
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
