"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest replace-server-system-coin-battery    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("replace_server_system_coin_battery.py"))
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
    'holder': {'pos': (0.15, 0, 0.7561), 'joints': {'latch': 0.0, 'contact': 0.0}},
    'old_cell': {'pos': (0.15, 0, 0.759)},
    'new_cell': {'pos': (0.15, 0.125, 0.7601)},
}
GOAL_POSES = {
    'holder': {'pos': (0.15, 0, 0.7561), 'joints': {'latch': 0.0, 'contact': 0.0}},
    'old_cell': {'pos': (0.15, -0.13, 0.7513)},
    'new_cell': {'pos': (0.15, 0, 0.759)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'old_cell', 'new_cell'}
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
    ('spent cell in the disposal tray, new cell still on its stand', START_POSES, {
        'holder': {'pos': (0.15, 0, 0.7561), 'joints': {'latch': 0.0, 'contact': 0.003}},
        'old_cell': {'pos': (0.15, -0.13, 0.7513)},
    }, 0.25, {}),
    ('new cell latched, spent cell set down on the bench beside the tray (not in it)', GOAL_POSES, {
        'old_cell': {'pos': (0.15, -0.2, 0.7493)},
    }, 0.75, {}),
    ('new cell under the hook but tilted up on the latch side, latch closed, spent cell in the tray', GOAL_POSES, {
        'new_cell': {'pos': (0.1492, 0, 0.761), 'tilt_deg': 12, 'tilt_axis': 'y', 'quat': (0.0, -0.10452846326765347, 0.0, 0.9945218953682734)},
    }, 0.5, {}),
    ('new cell latched but the left gripper still pinching it, spent cell in the tray', GOAL_POSES, {
    }, 0.25, {'TEST_GRIPPERS': "{'left': (np.array([HX, HY, SEATED_Z + .0117]), 0.2), 'right': (np.array([.11, -.22, TOP + .12]), 1.0)}"}),
    ('new cell lying flat on top of the hook and latch lips (not under them)', GOAL_POSES, {
        'new_cell': {'pos': (0.15, 0, 0.7631)},
    }, 0.25, {}),  # the lips are not over the cell, so nothing retains it and the contact is not made
    ('new cell seated and latched but "+" face down', GOAL_POSES, {
        'new_cell': {'pos': (0.15, 0, 0.759), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 0.25, {}),  # reversed polarity; only the disposal sub-goal counts
    ('new cell flat in the seat with the latch held open by the right gripper', GOAL_POSES, {
        'holder': {'pos': (0.15, 0, 0.7561), 'joints': {'latch': 0.0015, 'contact': 0.0}},
    }, 0.25, {'TEST_GRIPPERS': "{'left': (np.array([.11, .22, TOP + .12]), 1.0), 'right': (np.array([HX + .0128, HY, BOARD_TOP + .0198]), 0.0)}"}),  # the clip is not engaged and the cell is not released
    ('new cell hovering 5 mm above the seat', GOAL_POSES, {
        'new_cell': {'pos': (0.15, 0, 0.764)},
    }, 0.25, {}),  # not under the lips; only the disposal sub-goal counts
    ('spent cell left loose on the board, new cell latched', GOAL_POSES, {
        'old_cell': {'pos': (0.2, 0.04, 0.7579)},
    }, 0, {}),  # a loose lithium cell on a system board is a short-circuit hazard
    ('spent cell dropped into the chassis tray beside the board, new cell latched', GOAL_POSES, {
        'old_cell': {'pos': (0.255, 0.08, 0.7503)},
    }, 0, {}),  # a loose cell in the chassis is a short-circuit hazard
    ('a cell was dropped on the board earlier, then picked up; end state otherwise complete', GOAL_POSES, {
    }, 0, {'TEST_PAST_HAZARD': 'True'}),  # the hazard is sticky for the episode
    ('spent cell knocked off the table, new cell latched', GOAL_POSES, {
        'old_cell': {'pos': (0.15, -0.2, 0.5475)},
    }, 0, {}),  # dropped cell
    ('swapped - spent cell put back in the socket, new cell thrown in the tray', GOAL_POSES, {
        'old_cell': {'pos': (0.15, 0, 0.759)},
        'new_cell': {'pos': (0.15, -0.13, 0.7513)},
    }, 0, {}),  # the server still has the spent battery
    ('both cells in the disposal tray, socket empty', GOAL_POSES, {
        'holder': {'pos': (0.15, 0, 0.7561), 'joints': {'latch': 0.0, 'contact': 0.003}},
        'old_cell': {'pos': (0.138, -0.13, 0.7513)},
        'new_cell': {'pos': (0.162, -0.13, 0.7513)},
    }, 0.25, {}),  # no battery installed
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
