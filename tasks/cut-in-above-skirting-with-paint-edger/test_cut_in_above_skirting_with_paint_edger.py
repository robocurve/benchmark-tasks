"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest cut-in-above-skirting-with-paint-edger    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("cut_in_above_skirting_with_paint_edger.py"))
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
    'edger': {'pos': (0.237, 0.415, 0.809), 'joints': {'head': 0.0}},
}
GOAL_POSES = {
    'edger': {'pos': (0.237, 0.415, 0.8085), 'joints': {'head': 0.0}},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_PAINT': 'stroke_paint()'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'edger'}
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
    ('band cut in, edger still held above its rest', GOAL_POSES, {
        'edger': {'pos': (0.237, 0.415, 0.8385), 'joints': {'head': 0.0}},
    }, 0.75, {'TEST_PAINT': 'stroke_paint()', 'TEST_GRIPPERS': "{'left': (EDGER_HOME + TCP_IN_E + [0, 0, 0.030], 0.0), 'right': (np.array([0.20, -0.30, TOP + 0.25]), 1.0)}"}),
    ('band cut in, edger laid on its back on the table', GOAL_POSES, {
        'edger': {'pos': (0.15, 0.42, 0.7985), 'tilt_deg': 90, 'tilt_axis': 'y', 'quat': (0.0, -0.7071067811865475, 0.0, 0.7071067811865476), 'joints': {'head': 0.0}},
    }, 0.75, {'TEST_PAINT': 'stroke_paint()'}),
    ('band cut in, edger in its rest but still rocking', GOAL_POSES, {
        'edger': {'pos': (0.237, 0.415, 0.8085), 'vel': (0.0, 0.02, 0.0), 'joints': {'head': 0.0}},
    }, 0.75, {'TEST_PAINT': 'stroke_paint()'}),
    ('pad held 1.5 mm off the wall (pile still reaches)', GOAL_POSES, {
    }, 1, {'TEST_PAINT': 'stroke_paint(gap=0.0015)'}),
    ('pad tipped 1 degree, lower edge 1.7 mm off the wall', GOAL_POSES, {
    }, 1, {'TEST_PAINT': 'stroke_paint(pitch=1.0)'}),
    ('pad tipped 3 degrees the other way, upper edge off the wall', GOAL_POSES, {
    }, 1, {'TEST_PAINT': 'stroke_paint(pitch=-3.0)'}),
    ('pad set on the wall at three places instead of a stroke', GOAL_POSES, {
    }, 1, {'TEST_PAINT': 'dab_paint([0.250, 0.150, 0.050])'}),
    ('stroke stopped 20 mm short of the right tab', GOAL_POSES, {
    }, 0.5, {'TEST_PAINT': 'stroke_paint(0.250, 0.080)'}),
    ("stroke stopped at the stretch's middle", GOAL_POSES, {
    }, 0.25, {'TEST_PAINT': 'stroke_paint(0.250, 0.150)'}),
    ('wheels rode 1.6 mm high, bare line above the board', GOAL_POSES, {
    }, 0.25, {'TEST_PAINT': 'stroke_paint(lift=0.0016)'}),
    ('pad tipped 1.3 degrees, lower edge off the wall', GOAL_POSES, {
    }, 0.25, {'TEST_PAINT': 'stroke_paint(pitch=1.3)'}),
    ("good band but the pad dropped onto the board's top for 40 mm", GOAL_POSES, {
    }, 0.25, {'TEST_PAINT': 'stroke_paint() | board_smear(0.150)'}),
    ('edger never used, left in its rest', GOAL_POSES, {
    }, 0, {'TEST_PAINT': 'no_paint()'}),  # no paint on the wall
    ('pad pressed on the wall well above the board, wheels in the air', GOAL_POSES, {
    }, 0, {'TEST_PAINT': 'stroke_paint(lift=0.030)'}),  # less than 40 percent of the band and none of the cut line is painted
    ("pad run along the board's face instead of the wall", GOAL_POSES, {
    }, 0, {'TEST_PAINT': 'stroke_paint(gap=BOARD_T, lift=-0.050)'}),  # paint on the trim and none on the band
    ('band cut in, then the edger dropped pad-down on the table', GOAL_POSES, {
        'edger': {'pos': (0.15, 0.42, 0.7475), 'tilt_deg': 90, 'tilt_axis': 'y', 'joints': {'head': 0.0}},
    }, 0, {'TEST_PAINT': 'stroke_paint()'}),  # the pad on the table is a sticky failure
    ('band cut in, then the edger knocked off the table', GOAL_POSES, {
        'edger': {'pos': (0.3, 0.7, 0.4475), 'joints': {'head': 0.0}},
    }, 0, {'TEST_PAINT': 'stroke_paint()'}),  # sticky failure
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
