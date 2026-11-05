"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest bus-tub-glasses-rack-plates-stacked    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("bus_tub_glasses_rack_plates_stacked.py"))
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
    'glass_1': {'pos': (0.22, -0.145, 0.8035)},
    'glass_2': {'pos': (0.32, -0.145, 0.8035)},
    'plate_large': {'pos': (0.275, 0.135, 0.7685)},
    'plate_medium': {'pos': (0.355, 0.135, 0.7685)},
    'plate_small': {'pos': (0.2, 0.145, 0.7685)},
}
GOAL_POSES = {
    'glass_1': {'pos': (0.3425, 0.2625, 0.8025)},
    'glass_2': {'pos': (0.2575, 0.2625, 0.8025)},
    'plate_large': {'pos': (0.3, -0.305, 0.7665)},
    'plate_medium': {'pos': (0.3, -0.305, 0.7965)},
    'plate_small': {'pos': (0.3, -0.305, 0.8265)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'plate_small', 'glass_1', 'plate_large', 'plate_medium', 'glass_2'}
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
    ('one glass upright in a rack cell', START_POSES, {
        'glass_1': {'pos': (0.3425, 0.2625, 0.8025)},
    }, 0.2),
    ('both glasses upright in separate rack cells', START_POSES, {
        'glass_1': {'pos': (0.3425, 0.2625, 0.8025)},
        'glass_2': {'pos': (0.2575, 0.3475, 0.8025)},
    }, 0.4),
    ('the large plate alone on the tub floor', START_POSES, {
        'plate_large': {'pos': (0.3, -0.305, 0.7665)},
    }, 0.2),
    ('the large and medium plates stacked in the tub', START_POSES, {
        'plate_large': {'pos': (0.3, -0.305, 0.7665)},
        'plate_medium': {'pos': (0.3, -0.305, 0.7965)},
    }, 0.4),
    ('all three plates stacked in order, the glasses still on the tray', START_POSES, {
        'plate_large': {'pos': (0.3, -0.305, 0.7665)},
        'plate_medium': {'pos': (0.3, -0.305, 0.7965)},
        'plate_small': {'pos': (0.3, -0.305, 0.8265)},
    }, 0.6),
    ('the large and medium plates side by side on the tub floor', START_POSES, {
        'plate_large': {'pos': (0.25, -0.305, 0.7665)},
        'plate_medium': {'pos': (0.35, -0.305, 0.7665)},
    }, 0.2),
    ('the medium plate alone on the tub floor', START_POSES, {
        'plate_medium': {'pos': (0.3, -0.305, 0.7665)},
    }, 0),
    ('the small plate 3 cm off the centre of the medium plate', GOAL_POSES, {
        'plate_small': {'pos': (0.33, -0.305, 0.8265)},
    }, 0.8),
    ('a glass leaning 12 degrees in its cell', GOAL_POSES, {
        'glass_1': {'pos': (0.3425, 0.2625, 0.8077), 'tilt_deg': 12, 'tilt_axis': 'x'},
    }, 0.8),
    ('red team: small plate set on the tub floor diagonally beside the large-medium stack', GOAL_POSES, {
        'plate_small': {'pos': (0.347, -0.258, 0.7665)},
    }, 0.8),  # both glasses racked and large+medium stacked correctly; the small plate sits on the tub floor 6.6 cm from the stack's centre, so its disc is nowhere under the medium plate (their discs are 0.9 cm apart), yet the world-aligned bounding boxes overlap at the corner
    ('red team: plates only - small plate diagonally beside the large-medium stack, glasses on the tray', START_POSES, {
        'plate_large': {'pos': (0.3, -0.305, 0.7665)},
        'plate_medium': {'pos': (0.3, -0.305, 0.7965)},
        'plate_small': {'pos': (0.253, -0.352, 0.7665)},
    }, 0.4),  # large on the floor and medium on it earn 0.4; the small plate is beside the stack, not under the medium plate
    ('red team: medium plate carried in the air over the small plate still on the tray', GOAL_POSES, {
        'plate_medium': {'pos': (0.2, 0.145, 0.8825)},
        'plate_small': {'pos': (0.2, 0.145, 0.7675)},
    }, 0.6),  # glasses racked and the large plate on the tub floor (0.6); the medium plate is held 10 cm above the small plate in transit, not resting on it, so nothing is 'a larger plate resting on a smaller one'
    ('red team: large plate held 12 cm above the small plate on the tray at the time limit, glasses racked', GOAL_POSES, {
        'plate_large': {'pos': (0.2, 0.145, 0.9025)},
        'plate_medium': {'pos': (0.355, 0.135, 0.7675)},
        'plate_small': {'pos': (0.2, 0.145, 0.7675)},
    }, 0.4),  # both glasses racked; the large plate is in the gripper well above the small plate, not resting on it
    ('a glass in the tub beside the plate stack', GOAL_POSES, {
        'glass_2': {'pos': (0.375, -0.305, 0.8015)},
    }, 0),  # glassware dumped in the bus tub with the plates breaks
    ('a plate in an empty rack cell', GOAL_POSES, {
        'plate_small': {'pos': (0.2575, 0.3475, 0.7675)},
    }, 0),  # plates don't go in the glass rack
    ('the small plate buried under the large one', GOAL_POSES, {
        'plate_large': {'pos': (0.3, -0.305, 0.7965)},
        'plate_medium': {'pos': (0.3, -0.305, 0.8265)},
        'plate_small': {'pos': (0.3, -0.305, 0.7665)},
    }, 0),  # a small plate under a larger one rocks the stack and chips
    ('a glass knocked over on the tray', GOAL_POSES, {
        'glass_2': {'pos': (0.28, -0.1, 0.7825), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # a toppled glass breaks
    ('a glass lying on its side across the rack', GOAL_POSES, {
        'glass_2': {'pos': (0.3425, 0.3475, 0.8325), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0),  # a glass on its side isn't racked and can roll off
    ('a plate pushed off the table onto the floor', GOAL_POSES, {
        'plate_small': {'pos': (0.3, -0.8, 0.015)},
    }, 0),  # a dropped plate breaks
    ('the plates stacked in order but left on the tray', START_POSES, {
        'plate_large': {'pos': (0.28, 0.1, 0.7675)},
        'plate_medium': {'pos': (0.28, 0.1, 0.7975)},
        'plate_small': {'pos': (0.28, 0.1, 0.8275)},
    }, 0),  # the stack has to be in the tub
    ('red team: medium plate tilted 30 degrees leaning on the small plate on the tub floor, large plate on the floor beside them', GOAL_POSES, {
        'plate_large': {'pos': (0.365, -0.305, 0.7665)},
        'plate_medium': {'pos': (0.28, -0.305, 0.7807), 'tilt_deg': 30, 'tilt_axis': 'y'},
        'plate_small': {'pos': (0.2236, -0.305, 0.7665)},
    }, 0),  # the medium plate's raised side lies on the small plate's rim (1 cm of its outline over the small one) with its low edge on the floor; this is a larger plate resting on a smaller one, the breakage the zero-out exists for, yet the glasses and large plate keep 0.6
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
