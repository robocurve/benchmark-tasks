"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest change-shelf-edge-price-tickets    (in an environment with yam_arena installed)
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
from dataclasses import replace

TASK = load(Path(__file__).with_name("change_shelf_edge_price_tickets.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 4800


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
        q = {link.name: float((joints or {}).get(link.name, getattr(link.joint, "initial", 0.0))) *
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
    'strip': {'pos': (0.15, 0, 0.8675), 'joints': {'front': -0.22250788160420315}},
    'can_left': {'pos': (0.2981, 0.06, 0.9238)},
    'can_mid': {'pos': (0.2981, -0.04, 0.9238)},
    'can_right': {'pos': (0.2981, -0.14, 0.9238)},
    'ticket_left': {'pos': (0.1489, 0.06, 0.848)},
    'old_mid': {'pos': (0.1489, -0.04, 0.848)},
    'old_right': {'pos': (0.1489, -0.14, 0.848)},
    'new_mid': {'pos': (0.0878, -0.175, 0.771)},
    'new_right': {'pos': (0.0629, -0.175, 0.771)},
}
GOAL_POSES = {
    'strip': {'pos': (0.15, 0, 0.8675), 'joints': {'front': -0.22250788160420315}},
    'can_left': {'pos': (0.2981, 0.06, 0.9238)},
    'can_mid': {'pos': (0.2981, -0.04, 0.9238)},
    'can_right': {'pos': (0.2981, -0.14, 0.9238)},
    'ticket_left': {'pos': (0.1489, 0.06, 0.848)},
    'old_mid': {'pos': (0.035, -0.03, 0.7507), 'tilt_deg': 90, 'tilt_axis': 'y'},
    'old_right': {'pos': (0.065, -0.07, 0.7507), 'tilt_deg': 90, 'tilt_axis': 'y'},
    'new_mid': {'pos': (0.1489, -0.04, 0.848)},
    'new_right': {'pos': (0.1489, -0.14, 0.848)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'can_left', 'new_right', 'old_mid', 'can_mid', 'ticket_left', 'can_right', 'old_right', 'new_mid'}
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
    ('right product changed, middle product untouched', START_POSES, {
        'old_right': {'pos': (0.065, -0.07, 0.7507), 'tilt_deg': 90, 'tilt_axis': 'y'},
        'new_right': {'pos': (0.1489, -0.14, 0.848)},
    }, 0.5, {}),
    ('both old tickets in the waste tray, new tickets still in the stand', START_POSES, {
        'old_mid': {'pos': (0.035, -0.03, 0.7507), 'tilt_deg': 90, 'tilt_axis': 'y'},
        'old_right': {'pos': (0.065, -0.07, 0.7507), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0.5, {}),
    ('both old tickets out, only the right new ticket seated', GOAL_POSES, {
        'new_mid': {'pos': (0.0878, -0.175, 0.771)},
    }, 0.75, {}),
    ('new middle ticket seated but standing 5 mm proud of the channel floor', GOAL_POSES, {
        'new_mid': {'pos': (0.1489, -0.04, 0.853)},
    }, 0.75, {}),
    ('old right ticket dropped on the table beside the tray', GOAL_POSES, {
        'old_right': {'pos': (0.05, -0.17, 0.7477), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0.75, {}),
    ('new tickets swapped (each under the other product)', GOAL_POSES, {
        'new_mid': {'pos': (0.1489, -0.14, 0.848)},
        'new_right': {'pos': (0.1489, -0.04, 0.848)},
    }, 0.5, {}),  # Both old tickets are out (0.5), but neither product carries its own new ticket.
    ('new right ticket put in front of the old one, which was left in the strip', GOAL_POSES, {
        'old_right': {'pos': (0.1489, -0.14, 0.848)},
        'new_right': {'pos': (0.1482, -0.136, 0.848)},
    }, 0.5, {}),
    ('old middle ticket slid half out of the strip and left there', GOAL_POSES, {
        'old_mid': {'pos': (0.1489, -0.04, 0.873)},
    }, 0.5, {}),  # The old ticket is still in the strip, so neither the removal nor the new ticket counts for the middle can.
    ('new middle ticket seated with its printed face toward the shelf', GOAL_POSES, {
        'new_mid': {'pos': (0.1489, -0.04, 0.848), 'yaw_deg': 180},
    }, 0.75, {}),
    ('new right ticket leaning 10 degrees in the strip', GOAL_POSES, {
        'new_right': {'pos': (0.1469, -0.14, 0.848), 'tilt_deg': 10, 'tilt_axis': 'y'},
    }, 0.75, {}),
    ("new middle ticket 25 mm off its can's centre line", GOAL_POSES, {
        'new_mid': {'pos': (0.1489, -0.065, 0.848)},
    }, 0.75, {}),
    ('new middle ticket hovering 20 mm above its seat', GOAL_POSES, {
        'new_mid': {'pos': (0.1489, -0.04, 0.868)},
    }, 0.75, {}),
    ('clear front left open 10 degrees', GOAL_POSES, {
        'strip': {'pos': (0.15, 0, 0.8675), 'joints': {'front': -10.0}},
    }, 0, {}),  # Nothing is held in the strip; every ticket, including the untouched left one, is loose.
    ('untouched left ticket taken out and put in the waste tray', GOAL_POSES, {
        'ticket_left': {'pos': (0.05, -0.05, 0.7527), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0, {}),
    ('done, but the right hand still pinches the new middle ticket', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': "{'left': (np.array(PARK_L), 1.0), 'right': (np.array((SEAT_X, 0.0, Z_DECK + 0.010 + PAD)), 0.0)}"}),
    ('done, but the left fingertips still rest on the grip edge right above the new middle ticket', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': "{'left': (np.array((XF - 0.010, 0.0, Z_DECK + 0.010)), 0.0), 'right': (np.array(PARK_R), 1.0)}"}),
    ('done, but a ticket is still moving', GOAL_POSES, {
        'new_right': {'pos': (0.1489, -0.14, 0.848), 'vel': (0.0, 0.0, 0.05)},
    }, 0, {}),
    ('a can knocked over earlier and stood back up (sticky)', GOAL_POSES, {
    }, 0, {'TEST_STICKY': "('a can knocked over or off the shelf',)"}),
    ('middle can lying on its side on the shelf', GOAL_POSES, {
        'can_mid': {'pos': (0.2981, 0, 0.9056), 'tilt_deg': 90, 'tilt_axis': 'x'},
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
