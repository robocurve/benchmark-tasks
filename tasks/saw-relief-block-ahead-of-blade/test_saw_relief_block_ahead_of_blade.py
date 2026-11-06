"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest saw-relief-block-ahead-of-blade    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("saw_relief_block_ahead_of_blade.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 1200


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
    'relief_block': {'pos': (0.22, -0.36, 0.7685)},
    'board': {'pos': (0.15, 0.14, 0.7935)},
}
GOAL_POSES = {
    'relief_block': {'pos': (0.265, -0.098, 0.7975)},
    'board': {'pos': (0.2885, 0.0735, 0.7925)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'relief_block', 'board'}
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
    ('the block set against the fence, the board still at its start', START_POSES, {
        'relief_block': {'pos': (0.265, -0.098, 0.7975)},
    }, 0.5),
    ('block and board both flush, no gaps (a 60 mm offcut)', GOAL_POSES, {
        'relief_block': {'pos': (0.265, -0.1, 0.7975)},
        'board': {'pos': (0.285, 0.07, 0.7925)},
    }, 1),
    ("the board's end 7 mm short of the block", GOAL_POSES, {
        'board': {'pos': (0.2885, 0.0775, 0.7925)},
    }, 0.5),
    ('the board 6 mm off the gauge face', GOAL_POSES, {
        'board': {'pos': (0.2915, 0.0735, 0.7925)},
    }, 0.5),
    ('the block resting on another face (a plain cube, any face down)', GOAL_POSES, {
        'relief_block': {'pos': (0.265, -0.098, 0.7975), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 1),
    ('the board upside down (either broad face up)', GOAL_POSES, {
        'board': {'pos': (0.2885, 0.0735, 0.7925), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'y'},
    }, 1),
    ('the board turned end for end', GOAL_POSES, {
        'board': {'pos': (0.2885, 0.0735, 0.7925), 'yaw_deg': 180},
    }, 1),
    ('the block turned 4 degrees on the fence (within 5)', GOAL_POSES, {
        'relief_block': {'pos': (0.265, -0.098, 0.7975), 'yaw_deg': 4},
    }, 1),
    ("the block too far toward the arms for the board's end to bear on it", GOAL_POSES, {
        'relief_block': {'pos': (0.23, -0.098, 0.7975)},
    }, 0.5),
    ("red team: the board turned 1 degree, its near edge touching the gauge's right corner, its end 3 mm from the block", GOAL_POSES, {
        'board': {'pos': (0.2848, 0.0754, 0.7925), 'yaw_deg': -1},
    }, 1),  # flat, 1 degree off left-right (the text allows 3), 0.5 mm from the gauge, 3 mm from the block, end overlapping the block 28 mm, 13 mm from the blade; only its right-end corner, 19 cm right of the gauge bar, lies 2.8 mm nearer the arms than the gauge face's plane
    ("red team: the board turned 2 degrees, its near edge touching the gauge's right corner, its end 3 mm from the block", GOAL_POSES, {
        'board': {'pos': (0.2842, 0.0758, 0.7925), 'yaw_deg': -2},
    }, 1),  # within the stated 3 degrees, touching the gauge and the block, 14 mm from the blade; the 1 mm gauge-face rule, applied to the whole board's world box, makes the stated 3 degree allowance unreachable in this direction
    ("red team: the board turned 3 degrees right end toward the arms, near edge 0.5 mm off the gauge's right corner, end 3 mm from the block", GOAL_POSES, {
        'board': {'pos': (0.2835, 0.0761, 0.7925), 'yaw_deg': -3},
    }, 1),  # flat, 3 degrees off left-right (the stated limit), the part opposite the bar in front of its face, 3 mm from the block, end overlapping the block 34 mm, 16 mm from the blade
    ('the block against the fence beside the blade', GOAL_POSES, {
        'relief_block': {'pos': (0.355, -0.098, 0.7975)},
    }, 0),  # the offcut would be trapped between the blade and the block and kicked back
    ('the block beside the blade, off the fence', GOAL_POSES, {
        'relief_block': {'pos': (0.355, -0.06, 0.7975)},
    }, 0),  # anything between the blade and the fence traps the offcut
    ('the block on the fence 15 mm before the blade', START_POSES, {
        'relief_block': {'pos': (0.285, -0.098, 0.7975)},
    }, 0),  # less than the 20 mm of relief the rule needs
    ('the block on the fence past the blade (outfeed side)', START_POSES, {
        'relief_block': {'pos': (0.415, -0.098, 0.7975)},
    }, 0),  # not ahead of the blade, so it is no relief block
    ('the block against the far side of the fence', START_POSES, {
        'relief_block': {'pos': (0.265, -0.161, 0.7975)},
    }, 0),  # on the wrong side of the fence, where the board can't reach it
    ('the block standing on top of the fence', START_POSES, {
        'relief_block': {'pos': (0.265, -0.13, 0.8375)},
    }, 0),  # not on the saw table
    ('the board butted against the fence, the block set ahead of it', START_POSES, {
        'relief_block': {'pos': (0.17, -0.098, 0.7975)},
        'board': {'pos': (0.2885, 0.031, 0.7925)},
    }, 0),  # the fence used as the length stop traps the offcut
    ('the board touching the blade', GOAL_POSES, {
        'board': {'pos': (0.294, 0.0735, 0.7925)},
    }, 0),  # the board must not touch the blade
    ('the block still held 10 mm above its spot', START_POSES, {
        'relief_block': {'pos': (0.265, -0.098, 0.8075)},
    }, 0),  # not set down
    ('the board still held 5 mm above its spot', GOAL_POSES, {
        'board': {'pos': (0.2885, 0.0735, 0.7975)},
    }, 0.5),  # the board is not set down; the block keeps its credit
    ('the block still moving', GOAL_POSES, {
        'relief_block': {'pos': (0.265, -0.098, 0.7975), 'vel': (0.2, 0.0, 0.0)},
    }, 0),  # not at rest, and the board's credit needs the block set
    ('the board still moving', GOAL_POSES, {
        'board': {'pos': (0.2885, 0.0735, 0.7925), 'vel': (0.0, 0.2, 0.0)},
    }, 0.5),  # not at rest
    ('the board standing on its edge against the gauge', GOAL_POSES, {
        'board': {'pos': (0.278, 0.0735, 0.8025), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0.5),  # not lying flat
    ('the board turned 4 degrees', GOAL_POSES, {
        'board': {'pos': (0.2885, 0.0735, 0.7925), 'yaw_deg': -4},
    }, 0.5),  # not square to the gauge (3 degrees at most)
    ('the board lying across the tops of the gauge and the block', GOAL_POSES, {
        'board': {'pos': (0.27, 0.0535, 0.8325)},
    }, 0.5),  # not on the saw table
    ('the board lying across the tops of the block and the fence, clear of the blade', GOAL_POSES, {
        'board': {'pos': (0.265, -0.05, 0.8325)},
    }, 0),  # its bottom is level with the fence's top, so the 10 mm fence rule still applies; the board on the fence is the trapped-offcut layout
    ('the block stacked on the board', GOAL_POSES, {
        'relief_block': {'pos': (0.2885, 0, 0.8275)},
    }, 0),  # the block is not on the saw table against the fence
    ('the block tipped onto an edge against the fence', START_POSES, {
        'relief_block': {'pos': (0.265, -0.0897, 0.8058), 'tilt_deg': 45, 'tilt_axis': 'x'},
    }, 0),  # tilted 45 degrees
    ('the block turned 45 degrees, a corner at the fence', START_POSES, {
        'relief_block': {'pos': (0.265, -0.0907, 0.7975), 'yaw_deg': 45},
    }, 0),  # not square to the fence
    ('the block knocked off the table', GOAL_POSES, {
        'relief_block': {'pos': (0.3, -0.75, 0.02)},
    }, 0),  # on the floor
    ('the board knocked off the table, the block set', GOAL_POSES, {
        'board': {'pos': (0.3, 0.75, 0.015)},
    }, 0.5),  # only the block is set
    ('red team: the board lying across the tops of the blade and the fence, over the beside-blade zone, the block set', GOAL_POSES, {
        'board': {'pos': (0.355, -0.05, 0.8325)},
    }, 0),  # the board rests on the blade (touching it) and lies over the beside-blade zone; the scoring text says the board touching the blade zeroes the score, but the 40 mm height gate lets a board sitting on the blade's top escape every zero rule
    ("red team: the board bridging the block's top, the blade's top and the fence, the block set", GOAL_POSES, {
        'board': {'pos': (0.3, -0.05, 0.8325)},
    }, 0),  # the board lies on the blade and partly over the beside-blade zone; touching the blade should zero the trial
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
