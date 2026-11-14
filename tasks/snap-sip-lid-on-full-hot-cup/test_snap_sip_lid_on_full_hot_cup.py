"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest snap-sip-lid-on-full-hot-cup    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.modules.pop("yam_parts", None)
import yam_parts
sys.modules.pop("yam_joints", None)
import yam_joints
sys.modules.pop("yam_mesh", None)
import yam_mesh
from dataclasses import replace

TASK = load(Path(__file__).with_name("snap_sip_lid_on_full_hot_cup.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 2800


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


def place(name, pos, yaw_deg=0.0, tilt_deg=0.0, tilt_axis="x", vel=(0.0, 0.0, 0.0), quat=None, joints=None):
    """The ObjectState of `name` with its centre at `pos`, turned and tipped; its bounding box follows the turn."""
    r = (_rot(yaw_deg, tilt_deg, tilt_axis) if quat is None else
         ik.quat_xyzw_to_matrix(np.asarray(quat, float) / np.linalg.norm(quat)))
    if isinstance(OBJ[name], yam_joints.Mechanism):
        q = {link.name: float((joints or {}).get(link.name, 0.0)) *
             (math.pi / 180 if isinstance(link.joint, yam_joints.Hinge) else 1.0)
             for link in OBJ[name].links}
        state = OBJ[name].offline_state(pos, math.radians(yaw_deg), q, R=r)
        return replace(state, lin_vel=np.asarray(vel, float))
    if isinstance(OBJ[name], yam_parts.Part):
        origin = np.asarray(pos, float)
        low, high = OBJ[name].aabb(origin, r)
        return yam_parts.PartState((low + high) / 2, np.asarray(ik.matrix_to_quat_xyzw(r), float),
                                   np.asarray(vel, float), np.zeros(3), low, high, origin)
    pos, half = np.asarray(pos, float), _half(OBJ[name], r)
    q = ik.matrix_to_quat_xyzw(r)
    return ObjectState(pos, np.asarray(q, float), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)


def layout(poses):
    """Every object's state: the loose ones as given, the fixed ones where the task file puts them."""
    out = {o.name: place(o.name, o.pos, math.degrees(o.yaw)) for o in T.OBJECTS if o.mass is None}
    out.update({name: place(name, **p) for name, p in poses.items()})
    return out


START_POSES = {
    'cup': {'pos': (0.12, 0, 0.7475), 'joints': {'panel_0': 0.0, 'panel_1': 0.0, 'panel_2': 0.0, 'panel_3': 0.0, 'panel_4': 0.0, 'panel_5': 0.0, 'panel_6': 0.0, 'panel_7': 0.0}},
    'lid': {'pos': (0.15, 0.165, 0.751), 'joints': {'sector_0': 0.0, 'sector_1': 0.0, 'sector_2': 0.0, 'sector_3': 0.0, 'sector_4': 0.0, 'sector_5': 0.0, 'sector_6': 0.0, 'sector_7': 0.0}},
}
GOAL_POSES = {
    'cup': {'pos': (0.12, 0, 0.7475), 'joints': {'panel_0': 0.0, 'panel_1': 0.0, 'panel_2': 0.0, 'panel_3': 0.0, 'panel_4': 0.0, 'panel_5': 0.0, 'panel_6': 0.0, 'panel_7': 0.0}},
    'lid': {'pos': (0.12, 0, 0.8821), 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0025}},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'lid', 'cup'}
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
    ('lid set on the cup, nothing pressed', START_POSES, {
        'lid': {'pos': (0.12, 0, 0.8821), 'joints': {'sector_0': 0.0, 'sector_1': 0.0, 'sector_2': 0.0, 'sector_3': 0.0, 'sector_4': 0.0, 'sector_5': 0.0, 'sector_6': 0.0, 'sector_7': 0.0}},
    }, 0.25),
    ('lid on, two opposite sectors seated', START_POSES, {
        'lid': {'pos': (0.12, 0, 0.8821), 'joints': {'sector_0': -0.0025, 'sector_1': 0.0, 'sector_2': 0.0, 'sector_3': 0.0, 'sector_4': -0.0025, 'sector_5': 0.0, 'sector_6': 0.0, 'sector_7': 0.0}},
    }, 0.25),
    ('lid on, three sectors seated', START_POSES, {
        'lid': {'pos': (0.12, 0, 0.8821), 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': 0.0, 'sector_4': 0.0, 'sector_5': 0.0, 'sector_6': 0.0, 'sector_7': 0.0}},
    }, 0.5),
    ('lid on, the left half (four) seated', START_POSES, {
        'lid': {'pos': (0.12, 0, 0.8821), 'joints': {'sector_0': 0.0, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': 0.0, 'sector_6': 0.0, 'sector_7': 0.0}},
    }, 0.5),
    ('lid on, five sectors seated', START_POSES, {
        'lid': {'pos': (0.12, 0, 0.8821), 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': 0.0, 'sector_6': 0.0, 'sector_7': 0.0}},
    }, 0.5),
    ('lid on, six sectors seated', START_POSES, {
        'lid': {'pos': (0.12, 0, 0.8821), 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': 0.0, 'sector_7': 0.0}},
    }, 0.75),
    ('lid on, seven seated and the last pressed only 1.5 mm', START_POSES, {
        'lid': {'pos': (0.12, 0, 0.8821), 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0015}},
    }, 0.75),
    ('lid on, all eight at the 0.8 mm seating tolerance', START_POSES, {
        'lid': {'pos': (0.12, 0, 0.8821), 'joints': {'sector_0': -0.0017, 'sector_1': -0.0017, 'sector_2': -0.0017, 'sector_3': -0.0017, 'sector_4': -0.0017, 'sector_5': -0.0017, 'sector_6': -0.0017, 'sector_7': -0.0017}},
    }, 1),
    ('all seated, lid turned 20 degrees on the cup', START_POSES, {
        'lid': {'pos': (0.12, 0, 0.8821), 'yaw_deg': 20, 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0025}},
    }, 1),
    ('all seated, lid 1 mm off centre (inside the skirt play)', START_POSES, {
        'lid': {'pos': (0.121, 0, 0.8821), 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0025}},
    }, 1),
    ('all seated, cup slid 20 mm with the lid (inside 30 mm)', START_POSES, {
        'cup': {'pos': (0.14, 0, 0.7475), 'joints': {'panel_0': 0.0, 'panel_1': 0.0, 'panel_2': 0.0, 'panel_3': 0.0, 'panel_4': 0.0, 'panel_5': 0.0, 'panel_6': 0.0, 'panel_7': 0.0}},
        'lid': {'pos': (0.14, 0, 0.8821), 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0025}},
    }, 1),
    ('all seated, cup turned 30 degrees where it stands', START_POSES, {
        'cup': {'pos': (0.12, 0, 0.7475), 'yaw_deg': 30, 'joints': {'panel_0': 0.0, 'panel_1': 0.0, 'panel_2': 0.0, 'panel_3': 0.0, 'panel_4': 0.0, 'panel_5': 0.0, 'panel_6': 0.0, 'panel_7': 0.0}},
        'lid': {'pos': (0.12, 0, 0.8821), 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0025}},
    }, 1),
    ('all seated, the wall held in 4 mm at one panel (a steadying hold)', START_POSES, {
        'cup': {'pos': (0.12, 0, 0.7475), 'joints': {'panel_0': 0.0, 'panel_1': 0.0, 'panel_2': -0.004, 'panel_3': 0.0, 'panel_4': 0.0, 'panel_5': 0.0, 'panel_6': 0.0, 'panel_7': 0.0}},
        'lid': {'pos': (0.12, 0, 0.8821), 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0025}},
    }, 1),
    ('all seated, but the cup tipped 20 degrees', START_POSES, {
        'cup': {'pos': (0.12, 0, 0.7475), 'tilt_deg': 20, 'tilt_axis': 'x', 'joints': {'panel_0': 0.0, 'panel_1': 0.0, 'panel_2': 0.0, 'panel_3': 0.0, 'panel_4': 0.0, 'panel_5': 0.0, 'panel_6': 0.0, 'panel_7': 0.0}},
        'lid': {'pos': (0.12, -0.046, 0.874), 'tilt_deg': 20, 'tilt_axis': 'x', 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0025}},
    }, 0),  # a full cup tipped that far spills
    ('all seated, but the cup slid 40 mm off its mark', START_POSES, {
        'cup': {'pos': (0.16, 0, 0.7475), 'joints': {'panel_0': 0.0, 'panel_1': 0.0, 'panel_2': 0.0, 'panel_3': 0.0, 'panel_4': 0.0, 'panel_5': 0.0, 'panel_6': 0.0, 'panel_7': 0.0}},
        'lid': {'pos': (0.16, 0, 0.8821), 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0025}},
    }, 0),  # the cup must be left where it stood
    ('all seated, but the cup lifted 15 mm off the counter', START_POSES, {
        'cup': {'pos': (0.12, 0, 0.7625), 'joints': {'panel_0': 0.0, 'panel_1': 0.0, 'panel_2': 0.0, 'panel_3': 0.0, 'panel_4': 0.0, 'panel_5': 0.0, 'panel_6': 0.0, 'panel_7': 0.0}},
        'lid': {'pos': (0.12, 0, 0.8971), 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0025}},
    }, 0),  # picking the cup up is not lidding it on the counter
    ('all seated, but the wall squeezed in 6 mm at a side panel', START_POSES, {
        'cup': {'pos': (0.12, 0, 0.7475), 'joints': {'panel_0': 0.0, 'panel_1': 0.0, 'panel_2': -0.006, 'panel_3': 0.0, 'panel_4': 0.0, 'panel_5': 0.0, 'panel_6': 0.0, 'panel_7': 0.0}},
        'lid': {'pos': (0.12, 0, 0.8821), 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0025}},
    }, 0),  # a squeezed paper cup dents and overflows
    ('all seated, but a diagonal panel pushed 9 mm along x (6.4 mm in)', START_POSES, {
        'cup': {'pos': (0.12, 0, 0.7475), 'joints': {'panel_0': 0.0, 'panel_1': 0.0, 'panel_2': 0.0, 'panel_3': 0.009, 'panel_4': 0.0, 'panel_5': 0.0, 'panel_6': 0.0, 'panel_7': 0.0}},
        'lid': {'pos': (0.12, 0, 0.8821), 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0025}},
    }, 0),  # a squeezed paper cup dents and overflows
    ('every sector pressed down with the lid still on the counter', START_POSES, {
        'lid': {'pos': (0.15, 0.165, 0.751), 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0025}},
    }, 0),  # the lid is not on the cup
    ('all seated, but the lid 5 mm off centre', START_POSES, {
        'lid': {'pos': (0.125, 0, 0.8821), 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0025}},
    }, 0),  # a lid that far off centre is riding on the lip, not snapped round it
    ('all seated, but the lid 2.5 mm high on the rim', START_POSES, {
        'lid': {'pos': (0.12, 0, 0.8846), 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0025}},
    }, 0),  # the lid is not down on the rim
    ('all seated, but the lid tilted 8 degrees on the rim', START_POSES, {
        'lid': {'pos': (0.12, 0, 0.8821), 'tilt_deg': 8, 'tilt_axis': 'y', 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0025}},
    }, 0),  # a crooked lid is not snapped on
    ('lid upside down on the cup, sectors pressed', START_POSES, {
        'lid': {'pos': (0.12, 0, 0.8946), 'tilt_deg': 180, 'tilt_axis': 'x', 'joints': {'sector_0': -0.0025, 'sector_1': -0.0025, 'sector_2': -0.0025, 'sector_3': -0.0025, 'sector_4': -0.0025, 'sector_5': -0.0025, 'sector_6': -0.0025, 'sector_7': -0.0025}},
    }, 0),  # upside down is not on
    ('lid on the counter beside the cup, nothing pressed', START_POSES, {
        'lid': {'pos': (0.12, -0.11, 0.751), 'joints': {'sector_0': 0.0, 'sector_1': 0.0, 'sector_2': 0.0, 'sector_3': 0.0, 'sector_4': 0.0, 'sector_5': 0.0, 'sector_6': 0.0, 'sector_7': 0.0}},
    }, 0),  # nothing done
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
