"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest pull-file-leave-out-card    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("pull_file_leave_out_card.py"))
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
    'file_yellow': {'pos': (0.26, 0.12, 0.8335)},
    'file_green': {'pos': (0.26, 0.06, 0.8335)},
    'file_blue': {'pos': (0.26, 0, 0.8335)},
    'file_orange': {'pos': (0.26, -0.06, 0.8335)},
    'out_card': {'pos': (0.25, 0.3, 0.8385)},
}
GOAL_POSES = {
    'file_yellow': {'pos': (0.26, 0.12, 0.8335)},
    'file_green': {'pos': (0.26, 0.06, 0.8335)},
    'file_blue': {'pos': (0.25, -0.3, 0.8325)},
    'file_orange': {'pos': (0.26, -0.06, 0.8335)},
    'out_card': {'pos': (0.26, 0, 0.8375)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'file_blue', 'file_yellow', 'file_orange', 'file_green', 'out_card'}
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
    ('out-card in the gap, blue file in the stand, the row untouched', START_POSES, {
        'file_blue': {'pos': (0.25, -0.3, 0.8325)},
        'out_card': {'pos': (0.26, 0, 0.8375)},
    }, 1),
    ('out-card in the gap, blue file put in the out-card holder, the row untouched', START_POSES, {
        'file_blue': {'pos': (0.25, 0.3, 0.8325)},
        'out_card': {'pos': (0.26, 0, 0.8375)},
    }, 0.6),
    ('out-card in the gap, blue file in the stand, green file pushed 2.5 cm left', GOAL_POSES, {
        'file_green': {'pos': (0.26, 0.085, 0.8325)},
    }, 0.8),
    ('out-card in the gap only, blue file in the holder, orange file pushed 2 cm back', START_POSES, {
        'file_blue': {'pos': (0.25, 0.3, 0.8325)},
        'file_orange': {'pos': (0.28, -0.06, 0.8325)},
        'out_card': {'pos': (0.26, 0, 0.8375)},
    }, 0.4),
    ('out-card 1 cm off the old centre still counts', GOAL_POSES, {
        'out_card': {'pos': (0.25, 0, 0.8375)},
    }, 1),
    ('red team: goal with the out-card upside down (identical-looking uniform block)', GOAL_POSES, {
        'out_card': {'pos': (0.26, 0, 0.8375), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # the card is a uniform block; flipped end over end it is geometrically identical and a photo cannot tell it apart from the goal
    ('red team: goal with the blue file upside down in the reader stand', GOAL_POSES, {
        'file_blue': {'pos': (0.25, -0.3, 0.8325), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # the file is a uniform block; upside down it stands in the slot exactly as in the goal and a photo cannot tell the difference
    ('red team: goal with the out-card stood on its short end in the gap (16 cm edge front to back, 12 cm tall)', GOAL_POSES, {
        'out_card': {'pos': (0.248, 0, 0.8175), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 1),  # the card stands upright on the board in the blue file's place, 2.5 cm thick across the row, just turned a quarter turn in its own plane; it marks the gap as well as the goal card and is not knocked over
    ('red team: goal with the blue file stood on its 12 cm side in the reader stand (15 cm edge along the slot)', GOAL_POSES, {
        'file_blue': {'pos': (0.25, -0.3, 0.8175), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0.6),  # the file was pulled and stands upright in the reader stand's slot on its base, rotated in its own plane; at worst it misses the 0.4 stand credit, but it is not knocked over, and the same file left in the wrong holder already earns 0.6
    ('red team: goal with the out-card standing 2 cm left of the old centre, alone in the 9 cm gap between green and orange', GOAL_POSES, {
        'out_card': {'pos': (0.26, 0.02, 0.8375)},
    }, 1),  # the card stands upright between the green and orange files, touching neither, overlapping the blue file's old footprint; the instruction asks for it in the blue file's place between green and orange
    ('blue file in the reader stand with no out-card in the gap', START_POSES, {
        'file_blue': {'pos': (0.25, -0.3, 0.8325)},
    }, 0),  # a file out with nothing marking its place is the lost record the task is about
    ("the row closed up, green file slid into the blue file's place and the card where green stood", GOAL_POSES, {
        'file_green': {'pos': (0.26, 0, 0.8325)},
        'out_card': {'pos': (0.26, 0.06, 0.8375)},
    }, 0),  # the out-card marks the wrong place, so the blue file would be re-filed in the wrong spot
    ('out-card leaning 25 degrees against the orange file', GOAL_POSES, {
        'out_card': {'pos': (0.26, -0.034, 0.8353), 'tilt_deg': 25, 'tilt_axis': 'x'},
    }, 0),  # a card that is not standing in the gap does not hold it open
    ('blue file shoved out of the front of the shelf onto the table, card in the gap', GOAL_POSES, {
        'file_blue': {'pos': (0.12, 0, 0.8225)},
    }, 0),  # pushing the file out with the card leaves it loose on the table, not handed to the reader
    ('orange file knocked over, lying flat on the table in front of the shelf', GOAL_POSES, {
        'file_orange': {'pos': (0.12, -0.06, 0.7625), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # a knocked-over neighbour has left the row
    ('blue file fallen off the table', GOAL_POSES, {
        'file_blue': {'pos': (0.3, -0.8, 0.075)},
    }, 0),  # the file is on the floor
    ('green file pulled to the reader stand instead, card in its place', START_POSES, {
        'file_green': {'pos': (0.25, -0.3, 0.8325)},
        'out_card': {'pos': (0.26, 0.06, 0.8375)},
    }, 0),  # the wrong file was pulled
    ('red team: blue file shoved 1.7 cm right along the board, out-card squeezed in beside it, neighbours untouched', START_POSES, {
        'file_blue': {'pos': (0.26, -0.017, 0.8325)},
        'out_card': {'pos': (0.26, 0.014, 0.8375)},
    }, 0),  # the blue file never left the shelf; the card is wedged in beside it, so nothing was pulled and the card marks no gap
    ('red team: blue file shoved 1.7 cm left along the board, out-card squeezed in to its right, neighbours untouched', START_POSES, {
        'file_blue': {'pos': (0.26, 0.017, 0.8325)},
        'out_card': {'pos': (0.26, -0.014, 0.8375)},
    }, 0),  # the blue file is still in the row on the shelf; squeezing the card in next to it is not pulling the file
    ('red team: blue and orange files shoved right against the end panel, out-card centred in the old place', START_POSES, {
        'file_blue': {'pos': (0.26, -0.046, 0.8325)},
        'file_orange': {'pos': (0.26, -0.079, 0.8325)},
        'out_card': {'pos': (0.26, 0, 0.8375)},
    }, 0),  # the blue file was pushed along the shelf instead of pulled, and a neighbour was moved to make room
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
