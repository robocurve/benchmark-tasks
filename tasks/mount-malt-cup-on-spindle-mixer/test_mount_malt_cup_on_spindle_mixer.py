"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest mount-malt-cup-on-spindle-mixer    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("mount_malt_cup_on_spindle_mixer.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 3600


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
    'mixer': {'pos': (0.2771, -0.145, 0.7475), 'yaw_deg': 30, 'joints': {'spindle': 0.0, 'pin': 0.0}},
    'cup': {'pos': (0.3005, -0.4785, 0.7475), 'yaw_deg': -30},
}
GOAL_POSES = {
    'mixer': {'pos': (0.2771, -0.145, 0.7475), 'yaw_deg': 30, 'joints': {'spindle': 0.0, 'pin': 0.0047464791347594515}},
    'cup': {'pos': (0.2815, -0.1425, 0.9068), 'yaw_deg': 30, 'tilt_deg': 1.83, 'tilt_axis': 'y', 'quat': (0.004133103036016291, -0.01542495052332998, 0.2587860420638531, 0.9658026572719565)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'cup'}
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
    ('seated upright on the run step, pin held up 4 mm', GOAL_POSES, {
        'mixer': {'pos': (0.2771, -0.145, 0.7475), 'yaw_deg': 30, 'joints': {'spindle': 0.0, 'pin': 0.0040000000000000036}},
        'cup': {'pos': (0.2815, -0.1425, 0.9075), 'yaw_deg': 30},
    }, 1, {}),
    ('seated on the run step, cup turned a quarter about its axis', GOAL_POSES, {
        'cup': {'pos': (0.2815, -0.1425, 0.9068), 'yaw_deg': 120, 'tilt_deg': 1.83, 'tilt_axis': 'x', 'quat': (-0.007984541930403787, -0.013829632298623438, 0.8659149734647651, 0.49993624302520984)},
    }, 1, {}),  # The cup is round; which way it faces does not matter.
    ('seated on the run step but still in the hand', GOAL_POSES, {
    }, 0.5, {'TEST_GRIPPERS': '{"left": (np.array([0.10, 0.27, TOP + 0.16]), 1.0), "right": (np.array(mixer_xyz(SEATED_AXIS_X - 0.0113, 0, SEAT_RUN + 0.044)), 0.0)}'}),
    ('seated on the run step, still moving at 20 mm/s', GOAL_POSES, {
        'cup': {'pos': (0.2815, -0.1425, 0.9068), 'yaw_deg': 30, 'tilt_deg': 1.83, 'tilt_axis': 'y', 'vel': (0.0, 0.0, 0.02), 'quat': (0.004133103036016291, -0.01542495052332998, 0.2587860420638531, 0.9658026572719565)},
    }, 0.5, {}),
    ('hung on the standby step, pin down, released', GOAL_POSES, {
        'mixer': {'pos': (0.2771, -0.145, 0.7475), 'yaw_deg': 30, 'joints': {'spindle': 0.0, 'pin': 0.0}},
        'cup': {'pos': (0.2815, -0.1425, 0.8995), 'yaw_deg': 30},
    }, 0.5, {}),
    ('hung on the standby step, still in the hand', GOAL_POSES, {
        'mixer': {'pos': (0.2771, -0.145, 0.7475), 'yaw_deg': 30, 'joints': {'spindle': 0.0, 'pin': 0.0}},
        'cup': {'pos': (0.2815, -0.1425, 0.8995), 'yaw_deg': 30},
    }, 0.25, {'TEST_GRIPPERS': '{"left": (np.array([0.10, 0.27, TOP + 0.16]), 1.0), "right": (np.array(mixer_xyz(SEATED_AXIS_X - 0.0113, 0, SEAT_STANDBY + 0.044)), 0.0)}'}),
    ('held around the spindle, tipped 15 degrees, below the guide', START_POSES, {
        'cup': {'pos': (0.2403, -0.1663, 0.8825), 'yaw_deg': 30, 'tilt_deg': 15, 'tilt_axis': 'y'},
    }, 0.25, {'TEST_GRIPPERS': '{"left": (np.array([0.10, 0.27, TOP + 0.16]), 1.0), "right": (np.array(mixer_xyz(-0.042, 0, 0.1805)), 0.0)}'}),  # The spindle is inside the cup and the cup is off the counter, but it is not on the rest.
    ("on the run step 15 mm off the spindle's line", GOAL_POSES, {
        'mixer': {'pos': (0.2771, -0.145, 0.7475), 'yaw_deg': 30, 'joints': {'spindle': 0.0, 'pin': 0.0040000000000000036}},
        'cup': {'pos': (0.274, -0.1295, 0.9075), 'yaw_deg': 30},
    }, 0.25, {}),  # The bottom centre must be within 10 mm of the spindle's line across the rest; the spindle is still inside.
    ('carried 40 mm over the counter beside the mixer', START_POSES, {
        'cup': {'pos': (0.3005, -0.4785, 0.7875), 'yaw_deg': -30},
    }, 0, {'TEST_GRIPPERS': '{"left": (np.array([0.10, 0.27, TOP + 0.16]), 1.0), "right": (np.array([CUP_START[0] - 0.0113 * math.cos(START_YAW), CUP_START[1] - 0.0113 * math.sin(START_YAW), TOP + 0.084]), 0.0)}'}),
    ("stood upright on the mixer's base under the agitator", START_POSES, {
        'cup': {'pos': (0.2771, -0.145, 0.7725), 'yaw_deg': 30},
    }, 0, {}),  # The cup's top stays 2 mm under the agitator, so the spindle is not inside it.
    ('switch pin pushed up with the cup on the standby step', GOAL_POSES, {
        'mixer': {'pos': (0.2771, -0.145, 0.7475), 'yaw_deg': 30, 'joints': {'spindle': 0.0, 'pin': 0.004}},
        'cup': {'pos': (0.2815, -0.1425, 0.8995), 'yaw_deg': 30},
    }, 0.25, {}),  # The motor would run, but only the upper step counts as mounted; the spindle is inside the cup.
    ('seated with the pin down', GOAL_POSES, {
        'mixer': {'pos': (0.2771, -0.145, 0.7475), 'yaw_deg': 30, 'joints': {'spindle': 0.0, 'pin': 0.0}},
    }, 0.25, {}),  # Without the pin pushed in the motor is off; the spindle is inside the cup.
    ('tipped on its side on the counter', START_POSES, {
        'cup': {'pos': (0.3005, -0.4785, 0.7987), 'yaw_deg': -30, 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0, {}),  # The mix spills past 65 degrees.
    ('upside down on the run step', GOAL_POSES, {
        'cup': {'pos': (0.2815, -0.1425, 1.0881), 'yaw_deg': -150, 'tilt_deg': 180, 'tilt_axis': 'y', 'quat': (-0.25881904510252074, 0.9659258262890683, 1.5848095757158822e-17, 5.914589856893349e-17)},
    }, 0, {}),  # Spilled.
    ('dropped off the counter', START_POSES, {
        'cup': {'pos': (0.3005, -0.4785, 0.2475), 'yaw_deg': -30},
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
