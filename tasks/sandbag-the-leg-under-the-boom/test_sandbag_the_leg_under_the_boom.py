"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest sandbag-the-leg-under-the-boom    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("sandbag_the_leg_under_the_boom.py"))
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
    'bag_l': {'pos': (0.17, 0.12, 0.766)},
    'bag_r': {'pos': (0.17, -0.12, 0.766)},
}
GOAL_POSES = {
    'bag_l': {'pos': (0.3, 0.305, 0.785)},
    'bag_r': {'pos': (0.3, -0.305, 0.785), 'yaw_deg': 90},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'bag_l', 'bag_r'}
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
    ('the left bag lengthwise on the left boom leg', START_POSES, {
        'bag_l': {'pos': (0.3, 0.305, 0.785)},
    }, 0.5),
    ('the left bag crosswise on the left boom leg', START_POSES, {
        'bag_l': {'pos': (0.3, 0.305, 0.785), 'yaw_deg': 90},
    }, 0.35),
    ('the right bag lengthwise on the right boom leg', START_POSES, {
        'bag_r': {'pos': (0.3, -0.305, 0.785), 'yaw_deg': 90},
    }, 0.5),
    ('the right bag crosswise on the right boom leg', START_POSES, {
        'bag_r': {'pos': (0.3, -0.305, 0.785)},
    }, 0.35),
    ('both bags crosswise on their boom legs', START_POSES, {
        'bag_l': {'pos': (0.3, 0.305, 0.785), 'yaw_deg': 90},
        'bag_r': {'pos': (0.3, -0.305, 0.785)},
    }, 0.7),
    ('the left bag turned 15 degrees and 15 mm off the centre line, still within tolerance', GOAL_POSES, {
        'bag_l': {'pos': (0.32, 0.32, 0.785), 'yaw_deg': 15},
    }, 1),
    ('the left bag turned 30 degrees on its boom leg', GOAL_POSES, {
        'bag_l': {'pos': (0.3, 0.305, 0.785), 'yaw_deg': 30},
    }, 0.85),
    ('the left bag at the far end of its boom leg, 150 mm from the post', GOAL_POSES, {
        'bag_l': {'pos': (0.25, 0.305, 0.785)},
    }, 0.5),
    ('the right bag stacked on the left bag', GOAL_POSES, {
        'bag_r': {'pos': (0.3, 0.305, 0.82), 'yaw_deg': 90},
    }, 0.5),
    ('red team: the left bag upside down, flat and lengthwise on its boom leg', GOAL_POSES, {
        'bag_l': {'pos': (0.3, 0.305, 0.785), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # a rigid box flipped over is lying just as flat on the leg; it is the goal state
    ('red team: the right bag upside down, flat and lengthwise on its boom leg', GOAL_POSES, {
        'bag_r': {'pos': (0.3, -0.305, 0.785), 'yaw_deg': -90, 'tilt_deg': 180, 'tilt_axis': 'y'},
    }, 1),  # a rigid box flipped over is lying just as flat on the leg; it is the goal state
    ('red team: both bags upside down, flat and lengthwise on their boom legs', GOAL_POSES, {
        'bag_l': {'pos': (0.3, 0.305, 0.785), 'tilt_deg': 180, 'tilt_axis': 'x'},
        'bag_r': {'pos': (0.3, -0.305, 0.785), 'yaw_deg': 90, 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # both stands weighted correctly; which face is up is irrelevant for a uniform box
    ("the left bag on the left stand's wrong leg (heading 60 degrees)", GOAL_POSES, {
        'bag_l': {'pos': (0.45, 0.3916, 0.785), 'yaw_deg': 60},
    }, 0),  # set convention puts the bag on the leg under the boom; a bag on another leg is the rookie mistake
    ("the right bag on the right stand's wrong leg (heading 330 degrees)", GOAL_POSES, {
        'bag_r': {'pos': (0.3866, -0.455, 0.785), 'yaw_deg': -30},
    }, 0),  # set convention puts the bag on the leg under the boom; a bag on another leg is the rookie mistake
    ('the left bag wedged between two legs near the post', GOAL_POSES, {
        'bag_l': {'pos': (0.37, 0.357, 0.785), 'yaw_deg': 30},
    }, 0.5),  # bridging two legs is not weighting the boom leg; only the right stand counts
    ('the left bag toppled onto its side on its boom leg', GOAL_POSES, {
        'bag_l': {'pos': (0.3, 0.305, 0.7925), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.5),  # a bag resting on its 70 x 35 mm side face is not lying flat; only the right stand counts
    ('the left bag standing on its 50 x 35 mm end face on its boom leg', GOAL_POSES, {
        'bag_l': {'pos': (0.3, 0.305, 0.8025), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0.5),  # a bag standing on end is not lying flat; only the right stand counts
    ('the left bag lengthwise on its boom leg but only 55 mm out from the post', GOAL_POSES, {
        'bag_l': {'pos': (0.345, 0.305, 0.785)},
    }, 0.5),  # the instruction asks for the centre 7 to 13 cm out from the post; only the right stand counts
    ('the left bag pushed off the table onto the floor', START_POSES, {
        'bag_l': {'pos': (0.17, 0.12, 0.3)},
    }, 0),  # off the table is not on a leg
    ('the left bag held in the air above its boom leg', START_POSES, {
        'bag_l': {'pos': (0.3, 0.305, 0.835)},
    }, 0),  # a bag in the air weights nothing
    ('the left bag on the table beside its boom leg', START_POSES, {
        'bag_l': {'pos': (0.3, 0.255, 0.765)},
    }, 0),  # on the table, not on the leg
    ('red team: the left bag held 4.5 mm above its boom leg by the gripper', GOAL_POSES, {
        'bag_l': {'pos': (0.3, 0.305, 0.7895)},
    }, 0.5),  # the scoring text gives 0 to a bag in the air; a held bag puts no weight on the leg
    ('red team: both bags held 4.5 mm above their boom legs by the grippers', GOAL_POSES, {
        'bag_l': {'pos': (0.3, 0.305, 0.7895)},
        'bag_r': {'pos': (0.3, -0.305, 0.7895), 'yaw_deg': 90},
    }, 0),  # neither stand is weighted while both bags are still held in the air
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
