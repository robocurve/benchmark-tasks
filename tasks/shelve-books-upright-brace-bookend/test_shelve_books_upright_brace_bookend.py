"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest shelve-books-upright-brace-bookend    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("shelve_books_upright_brace_bookend.py"))
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
    'book_red': {'pos': (0.15, 0.275, 0.8135)},
    'book_green': {'pos': (0.15, 0.23, 0.8135)},
    'book_blue': {'pos': (0.15, 0.185, 0.8135)},
    'bookend': {'pos': (0.15, -0.22, 0.8085)},
}
GOAL_POSES = {
    'book_red': {'pos': (0.27, -0.011, 0.8225)},
    'book_green': {'pos': (0.27, 0.025, 0.8225)},
    'book_blue': {'pos': (0.27, 0.061, 0.8225)},
    'bookend': {'pos': (0.27, -0.048, 0.8225)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'book_blue', 'bookend', 'book_green', 'book_red'}
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
    ('the blue book against the end wall', START_POSES, {
        'book_blue': {'pos': (0.27, 0.061, 0.8225)},
    }, 0.2),
    ('one book upright on the board, away from the end wall', START_POSES, {
        'book_red': {'pos': (0.27, -0.15, 0.8225)},
    }, 0.1),
    ('all three books in the row, the bookend still at its start', START_POSES, {
        'book_red': {'pos': (0.27, -0.011, 0.8225)},
        'book_green': {'pos': (0.27, 0.025, 0.8225)},
        'book_blue': {'pos': (0.27, 0.061, 0.8225)},
    }, 0.6),
    ('the blue book against the end wall, the bookend upright on the board away from it', START_POSES, {
        'book_blue': {'pos': (0.27, 0.061, 0.8225)},
        'bookend': {'pos': (0.27, -0.15, 0.8225)},
    }, 0.4),
    ('the full row with the bookend 2 cm short of the last book', GOAL_POSES, {
        'bookend': {'pos': (0.27, -0.068, 0.8225)},
    }, 0.8),
    ('the third book 1 cm from the second, the bookend against it', GOAL_POSES, {
        'book_red': {'pos': (0.27, -0.021, 0.8225)},
        'bookend': {'pos': (0.27, -0.058, 0.8225)},
    }, 0.7),
    ('the third book upright but turned spine to the side', GOAL_POSES, {
        'book_red': {'pos': (0.27, -0.15, 0.8225), 'yaw_deg': 90},
    }, 0.6),
    ('red team: third book turned 9 degrees (inside the 10-degree spine allowance), corner touching the second book', GOAL_POSES, {
        'book_red': {'pos': (0.27, -0.0149, 0.8225), 'yaw_deg': 9},
        'bookend': {'pos': (0.27, -0.0579, 0.8225)},
    }, 1),  # the text counts a book within 10 degrees of front to back as shelved, and this one touches its neighbour (0 mm gap) with the bookend touching it; only the centre-based gap rule (which assumes zero yaw) drops it from the row
    ('red team: bookend turned 10 degrees, its corner touching the last book', GOAL_POSES, {
        'bookend': {'pos': (0.27, -0.054, 0.8225), 'yaw_deg': 10},
    }, 1),  # the bookend stands upright on the board touching the right side of the last book; the scoring text sets no yaw limit for the bookend
    ('red team: bookend turned 90 degrees, its 70 mm face flush against the last book', GOAL_POSES, {
        'bookend': {'pos': (0.27, -0.0632, 0.8225), 'yaw_deg': 90},
    }, 1),  # a bookend turned edge-on still stands flush against the last book and braces the row; the text never asks for a particular bookend orientation
    ('red team: third book still in the gripper 3 cm above the second, the trial ending mid-carry', GOAL_POSES, {
        'book_red': {'pos': (0.27, 0.025, 0.9725)},
    }, 0.6),  # two books are in the row and the bookend stands on the board; a book being carried over the row doesn't rest on anything, yet the stacking rule zeroes the whole trial
    ('red team: third book leaning 4.9 degrees (inside the 5-degree limit), touching the second book', GOAL_POSES, {
        'book_red': {'pos': (0.27, -0.0143, 0.8237), 'tilt_deg': 4.9, 'tilt_axis': 'x'},
        'bookend': {'pos': (0.27, -0.0565, 0.8225)},
    }, 1),  # the book counts as upright within the stated tilt limit and touches its neighbour, but the centre-based gap pushes it out of the row
    ('the bookend upright on an empty shelf', START_POSES, {
        'bookend': {'pos': (0.27, -0.15, 0.8225)},
    }, 0),  # a bookend with no books in the row braces nothing
    ('a book lying flat across the tops of the row', GOAL_POSES, {
        'book_red': {'pos': (0.27, 0.025, 0.8995), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # flat-stacked books are the damage the rule prevents
    ('a book lying flat on the board', GOAL_POSES, {
        'book_red': {'pos': (0.27, -0.15, 0.7795), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # a book lying on the shelf is the posture the rule forbids
    ('the middle book leaning 10 degrees', GOAL_POSES, {
        'book_green': {'pos': (0.27, 0.025, 0.8246), 'tilt_deg': 10, 'tilt_axis': 'x'},
    }, 0.5),  # a leaning book earns nothing and breaks the row, so the third book and the bookend get only their shelved credit
    ('the bookend knocked over on the board', GOAL_POSES, {
        'bookend': {'pos': (0.27, -0.098, 0.7825), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.6),  # a toppled bookend braces nothing
    ('the bookend stood on top of the row', GOAL_POSES, {
        'bookend': {'pos': (0.27, 0.025, 0.9425)},
    }, 0.6),  # a bookend on the books' tops braces nothing
    ('the third book pushed off the table', GOAL_POSES, {
        'book_red': {'pos': (0.5, 0.9, 0.06)},
    }, 0.6),  # a book on the floor isn't shelved, and the row is incomplete
    ("red team: middle book with its centre 1 mm inside the board's front edge, half of it hanging off", GOAL_POSES, {
        'book_green': {'pos': (0.226, 0.025, 0.8225)},
    }, 0.5),  # 37 of its 76 mm overhang the front edge and it's 44 mm out of line with its neighbours; that isn't shelving it in a row on the board
    ('red team: books pushed to the back wall, bookend turned edge-on standing in front of the last book', GOAL_POSES, {
        'book_red': {'pos': (0.296, -0.011, 0.8225)},
        'book_green': {'pos': (0.296, 0.025, 0.8225)},
        'book_blue': {'pos': (0.296, 0.061, 0.8225)},
        'bookend': {'pos': (0.236, -0.016, 0.8225), 'yaw_deg': 90},
    }, 0.8),  # the bookend stands in front of the last book's fore-edge, not against its right side; nothing stops the row's right end from leaning, yet it earns the full 0.4 bracing credit
    ('red team: books at the front edge, bookend turned edge-on hidden behind the last book', GOAL_POSES, {
        'book_red': {'pos': (0.254, -0.011, 0.8225)},
        'book_green': {'pos': (0.254, 0.025, 0.8225)},
        'book_blue': {'pos': (0.254, 0.061, 0.8225)},
        'bookend': {'pos': (0.314, -0.016, 0.8225), 'yaw_deg': 90},
    }, 0.8),  # the bookend stands between the last book and the back wall, behind it, not against its right side; it braces nothing sideways
    ("red team: bookend diagonally off the last book's front-right corner, not touching it", GOAL_POSES, {
        'book_red': {'pos': (0.296, -0.011, 0.8225)},
        'book_green': {'pos': (0.296, 0.025, 0.8225)},
        'book_blue': {'pos': (0.296, 0.061, 0.8225)},
        'bookend': {'pos': (0.236, -0.065, 0.8225), 'yaw_deg': 90},
    }, 0.8),  # the bookend's front-to-back extent (21.6-25.6 cm) and the book's (25.8-33.4 cm) don't overlap, so no face of the bookend touches the book; it is in front of and beside the row, bracing nothing
    ('red team: third book still held by the gripper 4 mm above the board in its spot, the bookend against it', GOAL_POSES, {
        'book_red': {'pos': (0.27, -0.011, 0.8265)},
    }, 0.6),  # rebuttal: agreed that the hanging book must not count, and it no longer does (bottom must be within 2 mm of the board). But then it earns 0, the row has only two books, and the bookend gets 0.2 for standing on the board, not 0.4: 0.2 + 0.2 + 0 + 0.2 = 0.6. 0.8 would need the hanging book to keep its 0.2, which the finding itself rules out.
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
