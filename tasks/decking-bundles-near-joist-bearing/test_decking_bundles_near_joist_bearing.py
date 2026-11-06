"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest decking-bundles-near-joist-bearing    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("decking_bundles_near_joist_bearing.py"))
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
    'bundle_a': {'pos': (0.19, 0.4, 0.7685)},
    'bundle_b': {'pos': (0.3, 0.4, 0.7685)},
}
GOAL_POSES = {
    'bundle_a': {'pos': (0.27, 0.12, 0.8175)},
    'bundle_b': {'pos': (0.27, -0.12, 0.8175)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'bundle_a', 'bundle_b'}
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
    ('only the left half landed, the other bundle still at its start', START_POSES, {
        'bundle_a': {'pos': (0.27, 0.12, 0.8175)},
    }, 0.5),
    ('only the right half landed, the other bundle still at its start', START_POSES, {
        'bundle_b': {'pos': (0.27, -0.12, 0.8175)},
    }, 0.5),
    ('the right bundle centred between two joists covers only two', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.16, 0.8175)},
    }, 0.75),
    ('the right bundle 55 mm from the girder face, half bearing credit', GOAL_POSES, {
        'bundle_b': {'pos': (0.315, -0.12, 0.8175)},
    }, 0.75),
    ('the right bundle 62.5 mm from the girder face, a quarter bearing credit', GOAL_POSES, {
        'bundle_b': {'pos': (0.3225, -0.12, 0.8175)},
    }, 0.625),
    ('the right bundle 69.9 mm from the girder face, almost no bearing credit, centre just outside the mid-span band', GOAL_POSES, {
        'bundle_b': {'pos': (0.3299, -0.12, 0.8175)},
    }, 0.501667),
    ('the left bundle across joists 1-3, the right one centred on the centre line over joists 3 and 4', GOAL_POSES, {
        'bundle_b': {'pos': (0.49, 0, 0.8175)},
    }, 0.5),
    ('the right bundle resting 0.3 mm above the joist tops', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.12, 0.8178)},
    }, 1),
    ('the left bundle against the back girder instead, also a bearing end', GOAL_POSES, {
        'bundle_a': {'pos': (0.49, 0.12, 0.8175)},
    }, 1),
    ('a bundle upside down, which the rule allows', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.12, 0.8175), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),
    ('the right bundle centred 20 mm off its joist, still on three', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.1, 0.8175)},
    }, 1),
    ('the right bundle landed at mid-span', GOAL_POSES, {
        'bundle_b': {'pos': (0.38, -0.12, 0.8175)},
    }, 0),  # a mid-span load can buckle joists not yet fully bridged and the bay collapses
    ('the right bundle stacked on the left one', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, 0.12, 0.8575)},
    }, 0),  # stacked bundles concentrate the load on three joists
    ('the right bundle stacked on the left one, turned 30 degrees', GOAL_POSES, {
        'bundle_b': {'pos': (0.28, 0.12, 0.8575), 'yaw_deg': 30},
    }, 0),  # stacked bundles concentrate the load on three joists
    ('the right bundle still held 10 mm above its spot', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.12, 0.8275)},
    }, 0.5),  # not landed
    ('the right bundle still held 5 mm above its spot', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.12, 0.8225)},
    }, 0.5),  # not landed
    ('the right bundle still sliding at 0.2 m/s', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.12, 0.8175), 'vel': (0.2, 0.0, 0.0)},
    }, 0.5),  # not at rest
    ('the right bundle on its side on the joists', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.12, 0.8275), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0.5),  # not lying flat
    ('the right bundle stood on its end on a joist', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.12, 0.9075), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.5),  # not lying flat
    ('the right bundle tilted 10 degrees on the joists', GOAL_POSES, {
        'bundle_b': {'pos': (0.28, -0.12, 0.8224), 'tilt_deg': 10, 'tilt_axis': 'y'},
    }, 0.5),  # not lying flat
    ('the right bundle pushed off the table', GOAL_POSES, {
        'bundle_b': {'pos': (0.3, -0.75, 0.02)},
    }, 0.5),  # on the floor
    ('the right bundle left on the handover mat at time-out', GOAL_POSES, {
        'bundle_b': {'pos': (0.13, 0, 0.7715)},
    }, 0.5),  # staging earns nothing
    ('both bundles on the left half', GOAL_POSES, {
        'bundle_b': {'pos': (0.49, 0.12, 0.8175)},
    }, 0.5),  # the right half is still bare; only the better bundle on a half counts
    ('the right bundle turned 20 degrees from square', GOAL_POSES, {
        'bundle_b': {'pos': (0.3, -0.12, 0.8175), 'yaw_deg': 20},
    }, 0.5),  # not square to the joists
    ('the right bundle lying along the joists', GOAL_POSES, {
        'bundle_b': {'pos': (0.38, -0.12, 0.8175), 'yaw_deg': 90},
    }, 0),  # lying along one joist with its centre at mid-span, the load the rule forbids
    ('the right bundle held still 0.8 mm above its spot', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.12, 0.8183)},
    }, 0.5),  # still in the gripper, not landed
    ('the right bundle hanging off the side of the bay on one joist', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.27, 0.8175)},
    }, 0.5),  # supported by one joist only
    ("the right bundle lying on the front girder's top", GOAL_POSES, {
        'bundle_b': {'pos': (0.24, -0.12, 0.8375)},
    }, 0.5),  # on the girder, not landed on the joists
    ('the right bundle on the table in front of the bay', GOAL_POSES, {
        'bundle_b': {'pos': (0.16, -0.3, 0.7675)},
    }, 0.5),  # not on the joists
    ('both bundles at their start', START_POSES, {
    }, 0),  # untouched
    ('the right bundle tilted 1 degree across its width, one long edge on the joists, the other held 1 mm up', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.12, 0.818), 'tilt_deg': 1, 'tilt_axis': 'y'},
    }, 0.5),  # one long edge is 1.05 mm above all three joists, past the 0.5 mm seat band; a released bundle drops flat, so it is still in the gripper
    ('the right bundle tilted 1 degree along its length, one end on joist 6, the other held up', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.12, 0.8191), 'tilt_deg': 1, 'tilt_axis': 'x'},
    }, 0.5),  # it sits only on joist 6 and hangs 1.6 mm above joist 5 and 3.0 mm above joist 4, still carried
    ('red team: the right bundle held still 2.5 mm above its spot by the gripper', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.12, 0.82)},
    }, 0.5),  # a bundle hanging 2.5 mm above the joists is still on the hook, not landed; the seat band reaches 3 mm above the joist tops, so a still grip counts as landed
    ('red team: the right bundle held still 1.5 mm above its spot by the gripper', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.12, 0.819)},
    }, 0.5),  # still held above the joists, not landed
    ('red team: the right bundle centred at mid-span, turned 13 degrees', GOAL_POSES, {
        'bundle_b': {'pos': (0.38, -0.12, 0.8175), 'yaw_deg': 13},
    }, 0),  # centred exactly at mid-span and within the task's own 15-degree squareness, it is a mid-span load; turning it 13 degrees widens its axis-aligned bounds past 100 mm so the 0-overall rule never fires
    ('red team: the right bundle centred at mid-span, turned 25 degrees', GOAL_POSES, {
        'bundle_b': {'pos': (0.38, -0.12, 0.8175), 'yaw_deg': 25},
    }, 0),  # centred at mid-span on three joists; a mid-span load buckles the joists however it is turned
    ('red team: the right bundle centred 25 mm from mid-span, square', GOAL_POSES, {
        'bundle_b': {'pos': (0.355, -0.12, 0.8175)},
    }, 0),  # its centre and most of its footprint lie in the middle 100 mm of the span (it is 95 mm, over 2 ft, from the girder); the instruction says never at mid-span, yet it adds credit
    ('red team: the right bundle lying along joist 5 at mid-span', GOAL_POSES, {
        'bundle_b': {'pos': (0.38, -0.12, 0.8175), 'yaw_deg': 90},
    }, 0),  # the whole bundle sits on one joist not yet fully bridged, across its mid-span, the worst load the rule forbids
    ('red team: the right bundle 90 mm from the front girder, 10 mm short of the mid-span band', GOAL_POSES, {
        'bundle_b': {'pos': (0.35, -0.12, 0.8175)},
    }, 0),  # rebuttal: with the mid-span rule now on the centre, as the same review asked, this bundle's centre is 120 mm from the girder face (30 mm from mid-span), inside the middle 100 mm of the span, so the whole bay scores 0; the bundle's own bearing credit already reaches 0 at a 70 mm gap, so there is no credit-then-cliff step any more
    ('red team: the left bundle across joists 2-4 and the right bundle across joists 4-6, joist 1 bare', GOAL_POSES, {
        'bundle_a': {'pos': (0.27, 0.016, 0.8175)},
        'bundle_b': {'pos': (0.49, -0.095, 0.8175)},
    }, 0.75),  # the instruction asks for one bundle across the left three joists and one across the right three; here the first bundle straddles the centre line on joists 2, 3 and 4 and the left three are not all loaded
    ('red team: the right bundle tilted 2 degrees along its length, touching only joist 6, its other end held up', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.12, 0.8206), 'tilt_deg': 2, 'tilt_axis': 'x'},
    }, 0.5),  # three level joists can't hold a bundle tilted 2 degrees along its length; it touches only the outer edge of joist 6 and hangs 3 mm above joist 5 and 5.6 mm above joist 4, so the gripper is still carrying it. The seat band only checks its lowest corner, so a still grip counts as landed on all three joists and success fires
    ('red team: the right bundle tilted 2.5 degrees across its width, back edge on the joists, front edge held up', GOAL_POSES, {
        'bundle_b': {'pos': (0.27, -0.12, 0.8188), 'tilt_deg': 2.5, 'tilt_axis': 'y'},
    }, 0.5),  # only one long edge touches the joists and the other is held 2.6 mm up; a released bundle drops flat, so a still bundle at this tilt is still in the gripper
    ('red team: the right bundle turned 45 degrees, corner at the girder, centre 99.5 mm from it, reaching past mid-span', GOAL_POSES, {
        'bundle_b': {'pos': (0.3295, -0.12, 0.8175), 'yaw_deg': 45},
    }, 0),  # its far corner reaches 428 mm, 48 mm past the joists' mid-point, so it puts load across mid-span on joists not yet fully bridged; only the centre-based test lets it off by half a millimetre
    ('red team: the right bundle turned 30 degrees, centre 90 mm from the girder, far end past mid-span', GOAL_POSES, {
        'bundle_b': {'pos': (0.32, -0.12, 0.8175), 'yaw_deg': 30},
    }, 0),  # its far corner reaches 401 mm, past the 380 mm mid-point of the span; a landing that loads mid-span and isn't within 1 ft of the bearing
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
