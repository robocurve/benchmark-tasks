"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest raised-floor-open-tiles-coned    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("raised_floor_open_tiles_coned.py"))
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
    'orange_tile_1': {'pos': (0.235, 0.07, 0.7635)},
    'orange_tile_2': {'pos': (0.375, -0.07, 0.7635)},
    'green_tile_1': {'pos': (0.375, 0.07, 0.7635)},
    'green_tile_2': {'pos': (0.235, -0.07, 0.7635)},
    'cone_1': {'pos': (0.25, -0.305, 0.7885)},
    'cone_2': {'pos': (0.35, -0.305, 0.7885)},
}
GOAL_POSES = {
    'orange_tile_1': {'pos': (0.26, 0.305, 0.7725)},
    'orange_tile_2': {'pos': (0.34, 0.305, 0.7725)},
    'green_tile_1': {'pos': (0.375, 0.07, 0.7635)},
    'green_tile_2': {'pos': (0.235, -0.07, 0.7635)},
    'cone_1': {'pos': (0.375, -0.07, 0.7875)},
    'cone_2': {'pos': (0.235, 0.07, 0.7875)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'orange_tile_1', 'cone_1', 'orange_tile_2', 'green_tile_1', 'green_tile_2', 'cone_2'}
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
    ('the near-left tile staged and its hole coned, the far-right tile untouched', START_POSES, {
        'orange_tile_1': {'pos': (0.26, 0.305, 0.7725)},
        'cone_2': {'pos': (0.235, 0.07, 0.7875)},
    }, 0.666667),
    ('the far-right tile staged and its hole coned, the near-left tile untouched', START_POSES, {
        'orange_tile_2': {'pos': (0.34, 0.305, 0.7725)},
        'cone_1': {'pos': (0.375, -0.07, 0.7875)},
    }, 0.666667),
    ('the near-left hole coned, its tile flat on the table off the pad and out of the aisle', START_POSES, {
        'orange_tile_1': {'pos': (0.45, 0.3, 0.7625)},
        'cone_2': {'pos': (0.235, 0.07, 0.7875)},
    }, 0.5),
    ('both holes coned, neither tile on the pad', START_POSES, {
        'orange_tile_1': {'pos': (0.45, 0.3, 0.7625)},
        'orange_tile_2': {'pos': (0.45, -0.3, 0.7625)},
        'cone_1': {'pos': (0.375, -0.07, 0.7875)},
        'cone_2': {'pos': (0.235, 0.07, 0.7875)},
    }, 0.666667),
    ('done, but one green tile nudged 15 mm inside its pocket', GOAL_POSES, {
        'green_tile_1': {'pos': (0.39, 0.07, 0.7625)},
    }, 0.833333),
    ('done with the two orange tiles stacked flat on the pad', GOAL_POSES, {
        'orange_tile_2': {'pos': (0.26, 0.305, 0.8025)},
    }, 1),
    ("done, but one cone 25 mm off its pocket's centre", GOAL_POSES, {
        'cone_1': {'pos': (0.4, -0.07, 0.7875)},
    }, 1),  # the cone stands upright on the pocket floor with its centre over the 12 cm square, so it is plainly in the hole
    ("done, but one cone 38 mm off its pocket's centre, against the pocket wall", GOAL_POSES, {
        'cone_1': {'pos': (0.337, -0.07, 0.7875)},
    }, 1),  # the walls keep a cone in the hole within 40 mm of the pocket's centre; it still guards it
    ('red team: done, but cone_2 stands upside down in the near-left hole', GOAL_POSES, {
        'cone_2': {'pos': (0.235, 0.07, 0.7875), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # the cone is a plain symmetric cylinder; end for end it stands exactly as upright (a level reads vertical) and guards the hole the same
    ('red team: done, both cones upside down in their holes', GOAL_POSES, {
        'cone_1': {'pos': (0.375, -0.07, 0.7875), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'y'},
        'cone_2': {'pos': (0.235, 0.07, 0.7875), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # both cylinders stand vertical and centred in the holes
    ('red team: done, but one orange tile lies flat on the pad face down', GOAL_POSES, {
        'orange_tile_1': {'pos': (0.26, 0.305, 0.7725), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # a uniform 7 x 7 x 3 block flipped over lies just as flat on the pad; nothing in the instruction or scoring text says face up
    ('red team: done, but a green tile sits flipped face down in its own pocket', GOAL_POSES, {
        'green_tile_1': {'pos': (0.375, 0.07, 0.7625), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'y'},
    }, 1),  # the flipped uniform block is flat, centred and closing its pocket; a level on it reads 0 degrees
    ('red team: one hole coned with its tile staged, the other orange tile untouched but upside down in its pocket', START_POSES, {
        'orange_tile_1': {'pos': (0.26, 0.305, 0.7725)},
        'orange_tile_2': {'pos': (0.375, -0.07, 0.7625), 'tilt_deg': 180, 'tilt_axis': 'x'},
        'cone_2': {'pos': (0.235, 0.07, 0.7875)},
    }, 0.666667),  # the far-right tile is flat in its pocket, closing it; same progress as the pinned 4/6 state
    ('both orange tiles staged, no cones', START_POSES, {
        'orange_tile_1': {'pos': (0.26, 0.305, 0.7725)},
        'orange_tile_2': {'pos': (0.34, 0.305, 0.7725)},
    }, 0),  # two open floor pockets with no guard, the failure the task is about
    ('both orange tiles staged, only one hole coned', START_POSES, {
        'orange_tile_1': {'pos': (0.26, 0.305, 0.7725)},
        'orange_tile_2': {'pos': (0.34, 0.305, 0.7725)},
        'cone_2': {'pos': (0.235, 0.07, 0.7875)},
    }, 0),  # the far-right pocket is open and unguarded
    ('the wrong tile pulled, staged and coned', START_POSES, {
        'green_tile_2': {'pos': (0.26, 0.305, 0.7725)},
        'cone_1': {'pos': (0.235, -0.07, 0.7875)},
    }, 0),  # an unmarked panel was pulled
    ('a green tile shoved half out onto the web', GOAL_POSES, {
        'green_tile_2': {'pos': (0.285, -0.07, 0.7925)},
    }, 0),  # an unmarked panel no longer seated loosens the floor and leaves a gap
    ('a cone toppled in its hole', GOAL_POSES, {
        'cone_2': {'pos': (0.235, 0.07, 0.7675), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # a fallen cone doesn't guard the hole, so the near-left pocket is open
    ('a cone standing upright on the table just beyond the frame, beside the far-right hole', GOAL_POSES, {
        'cone_1': {'pos': (0.48, -0.07, 0.7875)},
    }, 0),  # the cone's centre is outside the pocket's 12 cm square, so the far-right hole is open and unguarded
    ('a cone standing upright on the web beside the far-right hole', GOAL_POSES, {
        'cone_1': {'pos': (0.305, -0.07, 0.8175)},
    }, 0),  # the cone is on the web, outside the pocket's square and 3 cm above the pocket floor, so the hole is unguarded
    ("a cone standing upright on the frame's far rim beside the far-right hole", GOAL_POSES, {
        'cone_1': {'pos': (0.4425, -0.07, 0.8175)},
    }, 0),  # a cone on the rim is outside the pocket's square and not on the pocket floor
    ('a cone standing on the seated orange tile', START_POSES, {
        'cone_2': {'pos': (0.235, 0.07, 0.8175)},
    }, 0),  # nothing was lifted
    ('an orange tile knocked off the table, its hole coned', GOAL_POSES, {
        'orange_tile_1': {'pos': (0.45, 0.8, 0.015)},
    }, 0),  # a panel thrown off the table is not stored
    ('an orange tile left at the handover spot in the aisle, its hole coned', GOAL_POSES, {
        'orange_tile_1': {'pos': (0.115, 0, 0.7625)},
    }, 0),  # a loose panel lying in the aisle is a trip hazard
    ('an orange tile standing on edge on the pad', GOAL_POSES, {
        'orange_tile_2': {'pos': (0.34, 0.305, 0.7925), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.833333),  # a panel on edge is not stored flat, so it loses its staging point
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
