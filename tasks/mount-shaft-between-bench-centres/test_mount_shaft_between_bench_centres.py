"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest mount-shaft-between-bench-centres    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("mount_shaft_between_bench_centres.py"))
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
    'bench': {'pos': (0, 0, 0), 'joints': {'sleeve': 0.0, 'lever': 0.0}},
    'shaft': {'pos': (0.08, 0.052, 0.7788)},
}
GOAL_POSES = {
    'bench': {'pos': (0, 0, 0), 'joints': {'sleeve': -0.0015, 'lever': 0.0}},
    'shaft': {'pos': (0.18, 0.052, 0.8625)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'shaft'}
    assert TASK.docs and TASK.docs.strip()


def test_the_start_scores_nothing():
    assert T.score(START) == 0.0
    assert not T.success(START)


def test_the_goal_scores_full_and_succeeds(monkeypatch):
    module_values = {}
    for name, expr in GOAL_MODULE.items():
        module_values[name] = eval(expr, {**vars(T), **module_values, "math": math, "__builtins__": {}})
    for name, value in module_values.items():
        monkeypatch.setattr(T, name, value)
    assert T.score(GOAL) == 1.0
    assert T.success(GOAL)


def test_success_waits_for_the_objects_to_settle(monkeypatch):
    module_values = {}
    for name, expr in GOAL_MODULE.items():
        module_values[name] = eval(expr, {**vars(T), **module_values, "math": math, "__builtins__": {}})
    for name, value in module_values.items():
        monkeypatch.setattr(T, name, value)
    moving = layout({n: {**p, "vel": (0.05, 0.0, 0.0)} for n, p in GOAL_POSES.items()})
    assert not T.success(moving)

# (name, the layout it starts from, what's moved, the score it should get, module overrides)
CASES = [
    ('untouched start', START_POSES, {
    }, 0, {}),
    ('mounted end for end (the other centre hole on the fixed point)', GOAL_POSES, {
        'shaft': {'pos': (0.18, 0.052, 0.8625), 'yaw_deg': 180},
    }, 1, {}),  # Both ends are centre-drilled alike; which end goes on the fixed point does not matter.
    ('mounted, turned a quarter about its own axis', GOAL_POSES, {
        'shaft': {'pos': (0.18, 0.052, 0.8625), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 1, {}),  # Between centres the shaft is free to turn about its own axis; that is how it is checked.
    ('mounted but the left hand still holds the shaft', GOAL_POSES, {
    }, 0.75, {'TEST_GRIPPERS': '{"left": (np.array([BX, MOUNT_MID_Y, ZC + 0.003]), 0.0), "right": (np.array(PARK_R), 1.0)}'}),
    ('mounted but the right fingertips still on the knob', GOAL_POSES, {
    }, 0.75, {'TEST_GRIPPERS': '{"left": (np.array(PARK_L), 1.0), "right": (np.array(knob_tcp(0.0)), 0.0)}'}),
    ('mounted, still moving at 20 mm/s', GOAL_POSES, {
        'shaft': {'pos': (0.18, 0.052, 0.8625), 'vel': (0.0, 0.0, 0.02)},
    }, 0.25, {}),  # Both points seated but the shaft not still earns neither 1.0 nor 0.75; a seated point alone earns 0.25.
    ('left hole on the fixed point, lever held down, free end in line', GOAL_POSES, {
        'bench': {'pos': (0, 0, 0), 'joints': {'sleeve': -0.01, 'lever': 28.50743050935639}},
    }, 0.25, {'TEST_GRIPPERS': '{"left": (np.array([BX, MOUNT_MID_Y, ZC + 0.003]), 0.0), "right": (np.array(knob_tcp(LEVER_MAX)), 0.0)}'}),
    ('rammed in, the sleeve 4.5 mm back with the lever at rest, the left end 3 mm off its seat', GOAL_POSES, {
        'bench': {'pos': (0, 0, 0), 'joints': {'sleeve': -0.0045, 'lever': 0.0}},
        'shaft': {'pos': (0.18, 0.049, 0.8625)},
    }, 0.5, {}),  # Offline each state stands alone, so this is one point seated and the other in its hole short of the seat (0.5); on the GPU the sticky forcing cap also holds a rammed run to 0.5 at most (pinned in test_red_team.py with a timed sequence).
    ('shaft laid on the bench top between the tailstocks', START_POSES, {
        'shaft': {'pos': (0.18, 0.052, 0.7955)},
    }, 0, {}),
    ("shaft laid across the two tailstocks' V-prisms", START_POSES, {
        'shaft': {'pos': (0.18, 0.05, 0.9085)},
    }, 0, {}),
    ("shaft on the centre line 3 mm high (resting on the points' flanks, not in the seats)", GOAL_POSES, {
        'shaft': {'pos': (0.18, 0.052, 0.8655)},
    }, 0, {}),
    ('shaft back in its V-blocks with the lever left down', START_POSES, {
        'bench': {'pos': (0, 0, 0), 'joints': {'sleeve': -0.01, 'lever': 28.50743050935639}},
    }, 0, {}),
]


@pytest.mark.parametrize(("name", "base", "moved", "expected", "module"), CASES, ids=[c[0] for c in CASES])
def test_named_layouts(name, base, moved, expected, module, monkeypatch):
    module_values = {}
    for name, expr in module.items():
        module_values[name] = eval(expr, {**vars(T), **module_values, "math": math, "__builtins__": {}})
    for name, value in module_values.items():
        monkeypatch.setattr(T, name, value)
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
