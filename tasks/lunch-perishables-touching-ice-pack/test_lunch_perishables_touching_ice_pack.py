"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest lunch-perishables-touching-ice-pack    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("lunch_perishables_touching_ice_pack.py"))
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
    'ice_pack': {'pos': (0.16, -0.33, 0.7655)},
    'crackers': {'pos': (0.3, -0.33, 0.7705)},
    'sandwich': {'pos': (0.34, 0.33, 0.768)},
    'yogurt': {'pos': (0.14, 0.33, 0.7805)},
    'apple': {'pos': (0.23, 0.33, 0.783)},
}
GOAL_POSES = {
    'ice_pack': {'pos': (0.26, -0.02, 0.7675)},
    'crackers': {'pos': (0.18, -0.11, 0.7725)},
    'sandwich': {'pos': (0.26, 0.0735, 0.77)},
    'yogurt': {'pos': (0.204, 0, 0.7825)},
    'apple': {'pos': (0.1455, 0.105, 0.785)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'ice_pack', 'sandwich', 'crackers', 'apple', 'yogurt'}
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
    ('the ice pack flat in the box, nothing else', START_POSES, {
        'ice_pack': {'pos': (0.26, -0.02, 0.7675)},
    }, 0.2),
    ('the ice pack and the sandwich against it', START_POSES, {
        'ice_pack': {'pos': (0.26, -0.02, 0.7675)},
        'sandwich': {'pos': (0.26, 0.077, 0.77)},
    }, 0.5),
    ('the ice pack and the yogurt against it', START_POSES, {
        'ice_pack': {'pos': (0.26, -0.02, 0.7675)},
        'yogurt': {'pos': (0.2005, 0, 0.7825)},
    }, 0.5),
    ('the crackers flat in the box, nothing else', START_POSES, {
        'crackers': {'pos': (0.18, -0.11, 0.7725)},
    }, 0.1),
    ('the apple in the box, nothing else', START_POSES, {
        'apple': {'pos': (0.1455, 0.105, 0.785)},
    }, 0.1),
    ('everything packed but the apple, still on its mat', GOAL_POSES, {
        'apple': {'pos': (0.23, 0.33, 0.782)},
    }, 0.9),
    ('the sandwich 4 mm from the ice pack still counts', GOAL_POSES, {
        'sandwich': {'pos': (0.26, 0.079, 0.77)},
    }, 1),
    ('the sandwich turned 20 degrees, one corner against the ice pack', GOAL_POSES, {
        'sandwich': {'pos': (0.26, 0.084, 0.77), 'yaw_deg': 20},
    }, 1),
    ('the ice pack upside down but flat', GOAL_POSES, {
        'ice_pack': {'pos': (0.26, -0.02, 0.7675), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),
    ('everything packed, the apple rolled 36 degrees on the box floor', GOAL_POSES, {
        'apple': {'pos': (0.1455, 0.105, 0.785), 'tilt_deg': 36, 'tilt_axis': 'y'},
    }, 1),
    ('red team: the sandwich exactly 5 mm from the pack', GOAL_POSES, {
        'sandwich': {'pos': (0.26, 0.08, 0.77)},
    }, 1),  # the scoring text says at most 5 mm, so exactly 5 mm counts
    ('red team: the yogurt exactly 5 mm from the pack', GOAL_POSES, {
        'yogurt': {'pos': (0.1975, 0, 0.7825)},
    }, 1),  # the scoring text says at most 5 mm, so exactly 5 mm counts
    ("red team: yogurt leaning 9 degrees on the near wall, 3.7 mm from the pack at the pack's mid-height", GOAL_POSES, {
        'ice_pack': {'pos': (0.2057, -0.02, 0.7675)},
        'sandwich': {'pos': (0.2057, 0.077, 0.77)},
        'yogurt': {'pos': (0.1412, 0, 0.7864), 'yaw_deg': 180, 'tilt_deg': 9, 'tilt_axis': 'y'},
        'apple': {'pos': (0.3, 0.13, 0.785)},
    }, 1),  # the text says gaps are measured horizontally at the ice pack's mid-height (1.5 cm above the floor); there the upright-enough (9 deg) cup's side is 3.7 mm from the pack (2.7 mm at its closest), but the code measures the cup's axis at its own centre 3.4 cm up, gets 7 mm, and the zero rule gives 0. (Encoding note, tilt_deg -9 lost its sign in the generated test, which then leaned the cup toward the pack instead; yaw_deg 180 with tilt_deg 9 is the same lean toward the near wall and survives the round trip.)
    ('the sandwich in the box 1 cm from the ice pack', GOAL_POSES, {
        'sandwich': {'pos': (0.26, 0.085, 0.77)},
    }, 0),  # a perishable packed away from the cold source warms up; this is the failure the task is about
    ('the yogurt in the box 3 cm from the ice pack', GOAL_POSES, {
        'yogurt': {'pos': (0.1725, 0, 0.7825)},
    }, 0),  # a perishable packed away from the cold source warms up
    ('both perishables in the box, the ice pack left on its mat', START_POSES, {
        'sandwich': {'pos': (0.26, 0.077, 0.77)},
        'yogurt': {'pos': (0.2005, 0, 0.7825)},
    }, 0),  # perishables packed with no cold source
    ('the sandwich stacked on the ice pack', GOAL_POSES, {
        'sandwich': {'pos': (0.26, -0.02, 0.8)},
    }, 0),  # items go in one layer on the floor; a sandwich on the pack isn't packed beside it
    ('the ice pack on top of the sandwich', GOAL_POSES, {
        'ice_pack': {'pos': (0.26, 0.077, 0.8025)},
    }, 0),  # the pack isn't flat on the floor, so neither perishable has its cold credit
    ('the ice pack standing on its long edge', GOAL_POSES, {
        'ice_pack': {'pos': (0.26, -0.02, 0.7825), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0),  # a pack on edge isn't lying flat, which voids both cold credits
    ('the yogurt tipped over beside the ice pack', GOAL_POSES, {
        'yogurt': {'pos': (0.2005, 0, 0.78), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # a tipped yogurt cup spills and isn't packed
    ('the crackers knocked onto their side', GOAL_POSES, {
        'crackers': {'pos': (0.18, -0.11, 0.7875), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.9),  # a toppled item earns nothing; the rest is still packed
    ('the ice pack pushed off the table', GOAL_POSES, {
        'ice_pack': {'pos': (0.3, -0.8, 0.015)},
    }, 0),  # without the pack in the box the perishables have no cold credit
    ('the crackers pushed off the table', GOAL_POSES, {
        'crackers': {'pos': (0.3, -0.8, 0.02)},
    }, 0.9),  # only what is packed counts
    ('the yogurt held in the air above its spot', GOAL_POSES, {
        'yogurt': {'pos': (0.2005, 0, 0.8525)},
    }, 0),  # an item left in mid-air isn't packed
    ('the apple rolled 36 degrees and still held 1 cm above the box floor', GOAL_POSES, {
        'apple': {'pos': (0.1455, 0.105, 0.795), 'tilt_deg': 36, 'tilt_axis': 'y'},
    }, 0.9),  # a held apple isn't on the floor, however it is turned; its height is judged from its centre, not its turned bounding box
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
