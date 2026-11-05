"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest childproof-counter-hazards-back    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("childproof_counter_hazards_back.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 3000


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
    'knife': {'pos': (0.29, 0.27, 0.791)},
    'mug': {'pos': (0.21, 0.15, 0.8185)},
    'pills': {'pos': (0.36, 0.17, 0.8085)},
    'cup': {'pos': (0.29, -0.34, 0.8285)},
}
GOAL_POSES = {
    'knife': {'pos': (0.29, -0.39, 0.79)},
    'mug': {'pos': (0.29, -0.28, 0.8175)},
    'pills': {'pos': (0.29, -0.17, 0.8075)},
    'cup': {'pos': (0.29, 0.38, 0.8275)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'knife', 'mug', 'cup', 'pills'}
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
    ('the knife out of reach', START_POSES, {
        'knife': {'pos': (0.29, -0.39, 0.79)},
    }, 0.2),
    ('the knife and the mug out of reach', START_POSES, {
        'knife': {'pos': (0.29, -0.39, 0.79)},
        'mug': {'pos': (0.29, -0.28, 0.8175)},
        'cup': {'pos': (0.365, -0.34, 0.8275)},
    }, 0.4),
    ('all three hazards out of reach, the cup left at the far right', GOAL_POSES, {
        'cup': {'pos': (0.365, -0.34, 0.8275)},
    }, 0.6),
    ('the knife on its long edge right of the divider, still lying down', GOAL_POSES, {
        'knife': {'pos': (0.29, -0.39, 0.7925), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 1),
    ("the cup with its centre 140 mm from the toddler's end, the hazards out of reach", GOAL_POSES, {
        'cup': {'pos': (0.29, 0.31, 0.8275)},
    }, 1),
    ('the mug tilted 10 degrees against something, still counted upright', GOAL_POSES, {
        'mug': {'pos': (0.29, -0.28, 0.823), 'tilt_deg': 10, 'tilt_axis': 'x'},
    }, 1),
    ('red team: the knife lying flat but face-down (flipped over) right of the divider', GOAL_POSES, {
        'knife': {'pos': (0.29, -0.39, 0.79), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # a box flipped onto its other broad face still lies flat (a phone level app, the README's measuring tool, reads 0 degrees); the instruction only asks for the knife flat
    ('red team: the knife flipped end over end (180 about y), flat, right of the divider', GOAL_POSES, {
        'knife': {'pos': (0.29, -0.39, 0.79), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'y'},
    }, 1),  # same flat pose, turned over the other way; still lying flat out of reach
    ('the cup brought to the toddler, the hazards left where they were', START_POSES, {
        'cup': {'pos': (0.29, 0.38, 0.8275)},
    }, 0),  # the failure mode the task is about; the cup earns nothing while a hazard is within 550 mm of the toddler
    ("the cup at the toddler's end beside the knife, the mug and bottle out of reach", GOAL_POSES, {
        'knife': {'pos': (0.29, 0.27, 0.79)},
    }, 0.4),  # only the mug and bottle count; the cup earns nothing next to a knife the toddler can reach
    ('the mug toppled on its side right of the divider', GOAL_POSES, {
        'mug': {'pos': (0.29, -0.28, 0.8125), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.8),  # a toppled hot mug has spilled; it earns nothing
    ("the cup at the toddler's end, the hazards only in the middle band (300 to 550 mm away)", GOAL_POSES, {
        'knife': {'pos': (0.29, 0.04, 0.79)},
        'mug': {'pos': (0.205, -0.04, 0.8175)},
        'pills': {'pos': (0.375, 0.1, 0.8075)},
    }, 0),  # every hazard is still within the toddler's half-metre reach, so neither they nor the cup earn anything
    ('the knife standing on its end right of the divider', GOAL_POSES, {
        'knife': {'pos': (0.29, -0.39, 0.8525), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0.8),  # a knife standing on end is not lying down and can topple; it earns nothing
    ("the cup knocked over at the toddler's end", GOAL_POSES, {
        'cup': {'pos': (0.29, 0.38, 0.8075), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0.6),  # a toppled cup is not a drink the toddler can use
    ('the knife pushed off the right end onto the table', GOAL_POSES, {
        'knife': {'pos': (0.29, -0.55, 0.76)},
    }, 0),  # anything off the counter scores 0; a knife on the floor is within a toddler's reach
    ('the mug pushed off the front edge onto the table', GOAL_POSES, {
        'mug': {'pos': (0.1, -0.28, 0.7875)},
    }, 0),  # anything off the counter scores 0
    ('the cup stood on the stool', GOAL_POSES, {
        'cup': {'pos': (0.135, 0.37, 0.8575)},
    }, 0),  # off the counter; the stool is where the toddler stands
    ('the pill bottle set on top of the knife', GOAL_POSES, {
        'pills': {'pos': (0.29, -0.39, 0.8325)},
    }, 0.8),  # stacked on another object, not on the counter top
    ('the mug resting on the divider', GOAL_POSES, {
        'mug': {'pos': (0.29, -0.09, 0.8475)},
    }, 0.4),  # on the divider, not the counter top, and partly within reach, so the mug earns nothing and the cup earns nothing while it is there
    ('the mug held 5 cm above its spot', GOAL_POSES, {
        'mug': {'pos': (0.29, -0.28, 0.8675)},
    }, 0.8),  # an object left in mid-air earns nothing
    ('red team: the knife sliding fast toward the right end of the counter at the last step', GOAL_POSES, {
        'knife': {'pos': (0.29, -0.43, 0.79), 'vel': (0.0, -0.8, 0.0)},
    }, 0.8),  # the scoring text says each hazard earns credit when it rests on the counter; a knife sliding off the end at 0.8 m/s is not resting and is about to land on the floor
    ('red team: the mug sliding off the front edge at the last step', GOAL_POSES, {
        'mug': {'pos': (0.18, -0.28, 0.8175), 'vel': (-0.7, 0.0, 0.0)},
    }, 0.8),  # not resting; it is going over the front edge
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
