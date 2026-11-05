"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest stack-height-three-times-base    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("stack_height_three_times_base.py"))
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
    'narrow': {'pos': (0.18, 0.14, 0.7935)},
    'cube_1': {'pos': (0.27, 0.14, 0.7735)},
    'cube_2': {'pos': (0.36, 0.14, 0.7735)},
    'wide_1': {'pos': (0.21, -0.14, 0.7685)},
    'wide_2': {'pos': (0.33, -0.14, 0.7685)},
}
GOAL_POSES = {
    'narrow': {'pos': (0.27, 0.305, 0.8725)},
    'cube_1': {'pos': (0.27, 0.305, 0.8025)},
    'cube_2': {'pos': (0.27, -0.305, 0.8825)},
    'wide_1': {'pos': (0.27, -0.305, 0.7975)},
    'wide_2': {'pos': (0.27, -0.305, 0.8375)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'cube_2', 'wide_2', 'narrow', 'wide_1', 'cube_1'}
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
    ('one red carton on the right pad', START_POSES, {
        'wide_1': {'pos': (0.27, -0.305, 0.7975)},
    }, 0.2),
    ('both red cartons stacked on the right pad', START_POSES, {
        'wide_1': {'pos': (0.27, -0.305, 0.7975)},
        'wide_2': {'pos': (0.27, -0.305, 0.8375)},
    }, 0.4),
    ('the right stack complete, red, red, green (13 cm on a 21 cm limit), the left pad empty', START_POSES, {
        'cube_2': {'pos': (0.27, -0.305, 0.8825)},
        'wide_1': {'pos': (0.27, -0.305, 0.7975)},
        'wide_2': {'pos': (0.27, -0.305, 0.8375)},
    }, 0.6),
    ('one green cube on the left pad', START_POSES, {
        'cube_1': {'pos': (0.27, 0.305, 0.8025)},
    }, 0.2),
    ('the left stack complete, green then blue (14 cm on a 15 cm limit)', START_POSES, {
        'narrow': {'pos': (0.27, 0.305, 0.8725)},
        'cube_1': {'pos': (0.27, 0.305, 0.8025)},
    }, 0.4),
    ('the blue carton standing alone on the left pad (9 cm on a 12 cm limit)', START_POSES, {
        'narrow': {'pos': (0.27, 0.305, 0.8225)},
    }, 0.2),
    ('the blue carton lying on its side on the left pad with a green cube on it (9 cm on a 12 cm limit)', START_POSES, {
        'narrow': {'pos': (0.27, 0.305, 0.7975), 'tilt_deg': 90, 'tilt_axis': 'x'},
        'cube_1': {'pos': (0.27, 0.305, 0.8425)},
    }, 0.4),
    ('the other legal split, blue alone on the left, red, red, green, green on the right (18 cm on 21 cm)', START_POSES, {
        'narrow': {'pos': (0.27, 0.305, 0.8225)},
        'cube_1': {'pos': (0.27, -0.305, 0.8825)},
        'cube_2': {'pos': (0.27, -0.305, 0.9325)},
        'wide_1': {'pos': (0.27, -0.305, 0.7975)},
        'wide_2': {'pos': (0.27, -0.305, 0.8375)},
    }, 1),
    ('the top cube on the right stack 2 cm off the centre of the carton below', GOAL_POSES, {
        'cube_2': {'pos': (0.29, -0.305, 0.8825)},
    }, 0.8),
    ('red team: a red carton standing on its 7 x 4 cm edge as the right base with a green cube on it (12 cm on a 12 cm limit)', START_POSES, {
        'cube_1': {'pos': (0.27, -0.305, 0.8725)},
        'wide_1': {'pos': (0.27, -0.305, 0.8125), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.4),  # exactly at the stated limit, which is "no more than"
    ('all five cartons in one tower on the right pad (27 cm on a 21 cm limit)', START_POSES, {
        'narrow': {'pos': (0.27, -0.305, 1.0025)},
        'cube_1': {'pos': (0.27, -0.305, 0.8825)},
        'cube_2': {'pos': (0.27, -0.305, 0.9325)},
        'wide_1': {'pos': (0.27, -0.305, 0.7975)},
        'wide_2': {'pos': (0.27, -0.305, 0.8375)},
    }, 0),  # the tower is over the limit for its base, the hazard the task is about
    ('the left-hand cartons all on the left pad, green, green, blue (19 cm on a 15 cm limit), the right stack fine', START_POSES, {
        'narrow': {'pos': (0.27, 0.305, 0.9225)},
        'cube_1': {'pos': (0.27, 0.305, 0.8025)},
        'cube_2': {'pos': (0.27, 0.305, 0.8525)},
        'wide_1': {'pos': (0.27, -0.305, 0.7975)},
        'wide_2': {'pos': (0.27, -0.305, 0.8375)},
    }, 0),  # one unsafe stack in the store is the hazard, whatever the other pad holds
    ('the blue carton standing at the bottom with a green cube on it (14 cm on a 12 cm limit)', START_POSES, {
        'narrow': {'pos': (0.27, 0.305, 0.8225)},
        'cube_1': {'pos': (0.27, 0.305, 0.8925)},
    }, 0),  # the narrow base allows only 12 cm
    ('green, green, blue stacked on the table where the left cartons start (19 cm on a 15 cm limit), reds on the right pad', START_POSES, {
        'narrow': {'pos': (0.27, 0.14, 0.8925)},
        'cube_2': {'pos': (0.27, 0.14, 0.8225)},
        'wide_1': {'pos': (0.27, -0.305, 0.7975)},
        'wide_2': {'pos': (0.27, -0.305, 0.8375)},
    }, 0),  # an over-limit tower built beside the pads is the same hazard as one on a pad
    ('review: green flat on the left pad, green tilted 20 degrees on it, blue tilted 20 degrees face to face on that (19.7 cm on a 15 cm limit), reds on the right', START_POSES, {
        'narrow': {'pos': (0.27, 0.2811, 0.9253), 'tilt_deg': 20, 'tilt_axis': 'x'},
        'cube_1': {'pos': (0.27, 0.305, 0.8025)},
        'cube_2': {'pos': (0.27, 0.305, 0.8595), 'tilt_deg': 20, 'tilt_axis': 'x'},
        'wide_1': {'pos': (0.27, -0.305, 0.7975)},
        'wide_2': {'pos': (0.27, -0.305, 0.8375)},
    }, 0),  # the tilted cartons' bottoms sit below the highest corner of the carton under them, so they fail the 5 mm seat test, but they still stand over the pad and count in its 19.7 cm height
    ('review: the same tilted green, green, blue tower (19.7 cm on a 15 cm limit) on the table, reds on the right', START_POSES, {
        'narrow': {'pos': (0.27, 0.1161, 0.8953), 'tilt_deg': 20, 'tilt_axis': 'x'},
        'cube_2': {'pos': (0.27, 0.14, 0.8295), 'tilt_deg': 20, 'tilt_axis': 'x'},
        'wide_1': {'pos': (0.27, -0.305, 0.7975)},
        'wide_2': {'pos': (0.27, -0.305, 0.8375)},
    }, 0),  # an over-limit tower beside the pads is the same hazard, tilted or not
    ('the blue carton held 2 cm above the green cube on the left pad (16 cm on a 15 cm limit), reds on the right', START_POSES, {
        'narrow': {'pos': (0.27, 0.305, 0.8925)},
        'cube_1': {'pos': (0.27, 0.305, 0.8025)},
        'wide_1': {'pos': (0.27, -0.305, 0.7975)},
        'wide_2': {'pos': (0.27, -0.305, 0.8375)},
    }, 0),  # a carton above the pad counts in the stack's height whether or not it rests there
    ('the blue carton knocked over on the table', START_POSES, {
        'narrow': {'pos': (0.18, 0.14, 0.7675), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # knocking a carton over puts nothing on a pad
    ('the top cube tipped 30 degrees on the right stack', GOAL_POSES, {
        'cube_2': {'pos': (0.27, -0.305, 0.8917), 'tilt_deg': 30, 'tilt_axis': 'x'},
    }, 0.8),  # a tipped carton is not counted; the four cartons stacked flat keep their credit
    ('a green cube pushed off the table onto the floor', GOAL_POSES, {
        'cube_2': {'pos': (0.3, -0.8, 0.025)},
    }, 0.8),  # the carton on the floor earns nothing; the four cartons still stacked keep their credit
    ('two green cubes side by side on the left pad', START_POSES, {
        'cube_1': {'pos': (0.27, 0.278, 0.8025)},
        'cube_2': {'pos': (0.27, 0.332, 0.8025)},
    }, 0),  # the instruction asks for one stack per pad
    ('the blue carton held in the air over the left pad', START_POSES, {
        'narrow': {'pos': (0.27, 0.305, 0.8775)},
    }, 0),  # a carton not standing on the pad is not stored
    ('red team: all five cartons in one 27 cm tower on the right pad, the upper four shifted 14 mm past the pad edge', START_POSES, {
        'narrow': {'pos': (0.322, -0.305, 1.0025)},
        'cube_1': {'pos': (0.322, -0.305, 0.8825)},
        'cube_2': {'pos': (0.322, -0.305, 0.9325)},
        'wide_1': {'pos': (0.308, -0.305, 0.7975)},
        'wide_2': {'pos': (0.322, -0.305, 0.8375)},
    }, 0),  # the 27 cm tower on a red base breaks the 21 cm limit; leaning it so the upper centres sit just past the pad outline doesn't make it safer (the stack's centre of mass, 48 mm from the pad centre, is still over the pad)
    ('red team: green, green, blue (19 cm on a 15 cm limit) on the left pad leaning 14 mm past the pad edge, reds on the right', START_POSES, {
        'narrow': {'pos': (0.322, 0.305, 0.9225)},
        'cube_1': {'pos': (0.308, 0.305, 0.8025)},
        'cube_2': {'pos': (0.322, 0.305, 0.8525)},
        'wide_1': {'pos': (0.27, -0.305, 0.7975)},
        'wide_2': {'pos': (0.27, -0.305, 0.8375)},
    }, 0),  # the left stack is 19 cm on a green cube (limit 15 cm), the exact hazard; offsetting the upper cartons past the outline only hides them from the height check
    ('red team: green, green, blue (19 cm) on the left pad with the bottom cube overhanging the pad edge, reds on the right', START_POSES, {
        'narrow': {'pos': (0.294, 0.305, 0.9225)},
        'cube_1': {'pos': (0.322, 0.305, 0.8025)},
        'cube_2': {'pos': (0.308, 0.305, 0.8525)},
        'wide_1': {'pos': (0.27, -0.305, 0.7975)},
        'wide_2': {'pos': (0.27, -0.305, 0.8375)},
    }, 0),  # a 19 cm stack stands on the left pad over a 5 cm base; the task's own rule is 0 overall for any unsafe stack in the store, but with the bottom centre 2 mm outside the outline the pad has no base and the height check is skipped
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
