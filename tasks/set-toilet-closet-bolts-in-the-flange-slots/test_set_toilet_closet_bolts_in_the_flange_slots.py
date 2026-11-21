"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest set-toilet-closet-bolts-in-the-flange-slots    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("set_toilet_closet_bolts_in_the_flange_slots.py"))
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
    'bolt_right': {'pos': (0.11, -0.13, 0.8074), 'yaw_deg': -22.154, 'quat': (0.0, 0.0, -0.19213118715002858, 0.9813692510591111), 'joints': {'retainer': 0.0}},
    'bolt_left': {'pos': (0.11, 0.13, 0.8074), 'yaw_deg': 157.846, 'quat': (0.0, 0.0, 0.9813692510591111, 0.1921311871500287), 'joints': {'retainer': 0.0}},
}
GOAL_POSES = {
    'bolt_right': {'pos': (0.18, -0.075, 0.807), 'joints': {'retainer': -0.050630000000000085}},
    'bolt_left': {'pos': (0.18, 0.075, 0.807), 'yaw_deg': 180, 'joints': {'retainer': -0.050630000000000085}},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'bolt_left', 'bolt_right'}
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
    ('untouched start, both bolts lying on their mats', START_POSES, {
    }, 0, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ('both set, 2.5 mm either side of the line', GOAL_POSES, {
        'bolt_right': {'pos': (0.1775, -0.075, 0.807), 'yaw_deg': -1.91, 'quat': (0.0, 0.0, -0.01666589507244508, 0.9998611143261019), 'joints': {'retainer': -0.050630000000000085}},
        'bolt_left': {'pos': (0.1775, 0.075, 0.807), 'yaw_deg': -178.09, 'quat': (0.0, 0.0, 0.9998611143261019, -0.01666589507244508), 'joints': {'retainer': -0.050630000000000085}},
    }, 1, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ('one bolt 4 mm off the line, the other set', GOAL_POSES, {
        'bolt_right': {'pos': (0.176, -0.0749, 0.807), 'yaw_deg': -3.056, 'quat': (0.0, 0.0, -0.026663506285210643, 0.9996444655139038), 'joints': {'retainer': -0.050630000000000085}},
    }, 0.75, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ('one retainer left at the tip, the other set', GOAL_POSES, {
        'bolt_right': {'pos': (0.18, -0.075, 0.807), 'joints': {'retainer': 0.0}},
    }, 0.75, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ('one retainer 3 mm short of the ring', GOAL_POSES, {
        'bolt_right': {'pos': (0.18, -0.075, 0.807), 'joints': {'retainer': -0.04763000000000008}},
    }, 0.75, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ('one bolt leaning 7 degrees', GOAL_POSES, {
        'bolt_right': {'pos': (0.18, -0.075, 0.807), 'tilt_deg': 7, 'tilt_axis': 'x', 'joints': {'retainer': -0.050630000000000085}},
    }, 0.75, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ('one bolt set, the other still on its mat', START_POSES, {
        'bolt_right': {'pos': (0.18, -0.075, 0.807), 'joints': {'retainer': -0.050630000000000085}},
    }, 0.5, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ('both slid to the line, retainers never pushed', GOAL_POSES, {
        'bolt_right': {'pos': (0.18, -0.075, 0.807), 'joints': {'retainer': 0.0}},
        'bolt_left': {'pos': (0.18, 0.075, 0.807), 'yaw_deg': 180, 'joints': {'retainer': 0.0}},
    }, 0.25, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ('both dropped in the wide ends, not slid', GOAL_POSES, {
        'bolt_right': {'pos': (0.1517, -0.0695, 0.807), 'yaw_deg': -22.154, 'quat': (0.0, 0.0, -0.19213118715002858, 0.9813692510591112), 'joints': {'retainer': 0.0}},
        'bolt_left': {'pos': (0.2083, 0.0695, 0.807), 'yaw_deg': 157.846, 'quat': (0.0, 0.0, 0.9813692510591111, 0.19213118715002866), 'joints': {'retainer': 0.0}},
    }, 0.25, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ('both set but the right one still pinched', GOAL_POSES, {
    }, 0.75, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (np.array([0.180, -0.075, RING_TOP + 0.066]), 0.2)}'}),
    ("both bolts stood upright on the ring's top at the line, heads not in the slots", START_POSES, {
        'bolt_right': {'pos': (0.18, -0.075, 0.812), 'joints': {'retainer': -0.050630000000000085}},
        'bolt_left': {'pos': (0.18, 0.075, 0.812), 'joints': {'retainer': -0.050630000000000085}},
    }, 0, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),  # neither head went down through a slot, so nothing holds the bolts
    ('both bolts stood on the plywood just outside the ring on the line', START_POSES, {
        'bolt_right': {'pos': (0.18, -0.12, 0.8039), 'joints': {'retainer': -0.050630000000000085}},
        'bolt_left': {'pos': (0.18, 0.12, 0.8039), 'joints': {'retainer': -0.050630000000000085}},
    }, 0, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),  # not in the flange at all
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
