"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest repair-drain-pipe-with-slip-couplings    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("repair_drain_pipe_with_slip_couplings.py"))
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


def place(name, pos, yaw_deg=0.0, tilt_deg=0.0, tilt_axis="x", vel=(0.0, 0.0, 0.0), quat=None):
    """The ObjectState of `name` with its centre at `pos`, turned and tipped; its bounding box follows the turn."""
    r = (_rot(yaw_deg, tilt_deg, tilt_axis) if quat is None else
         ik.quat_xyzw_to_matrix(np.asarray(quat, float) / np.linalg.norm(quat)))
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
    'section': {'pos': (0.28, -0.02, 0.7785)},
    'coupling_a': {'pos': (0.12, 0.26, 0.782), 'yaw_deg': 90},
    'coupling_b': {'pos': (0.12, -0.26, 0.782), 'yaw_deg': -90},
    'small_coupling': {'pos': (0.12, 0.38, 0.776), 'yaw_deg': 90},
    'large_coupling': {'pos': (0.12, -0.367, 0.7885), 'yaw_deg': -90},
}
GOAL_POSES = {
    'section': {'pos': (0.2, 0, 0.817)},
    'coupling_a': {'pos': (0.2, 0.1685, 0.8165)},
    'coupling_b': {'pos': (0.2, -0.1685, 0.8165)},
    'small_coupling': {'pos': (0.12, 0.38, 0.776), 'yaw_deg': 90},
    'large_coupling': {'pos': (0.12, -0.367, 0.7885), 'yaw_deg': -90},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'small_coupling', 'coupling_b', 'section', 'coupling_a', 'large_coupling'}
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
    ('both sleeves parked on old stubs', START_POSES, {
        'coupling_a': {'pos': (0.2, 0.217, 0.8165)},
        'coupling_b': {'pos': (0.2, -0.217, 0.8165)},
    }, 0.25, {}),
    ('both sleeves fully on staged section', START_POSES, {
        'coupling_a': {'pos': (0.28, 0.105, 0.7785)},
        'coupling_b': {'pos': (0.28, -0.145, 0.7785)},
    }, 0.25, {}),
    ('parked sleeves and section on support', GOAL_POSES, {
        'coupling_a': {'pos': (0.2, 0.217, 0.8165)},
        'coupling_b': {'pos': (0.2, -0.217, 0.8165)},
    }, 0.5, {}),
    ('left joint only', GOAL_POSES, {
        'coupling_b': {'pos': (0.2, -0.217, 0.8165)},
    }, 0.5, {}),
    ('right joint only', GOAL_POSES, {
        'coupling_a': {'pos': (0.2, 0.217, 0.8165)},
    }, 0.5, {}),
    ('swapped matching couplings', GOAL_POSES, {
        'coupling_a': {'pos': (0.2, -0.1685, 0.8165)},
        'coupling_b': {'pos': (0.2, 0.1685, 0.8165)},
    }, 1, {}),
    ('sleeve nudged onto four mm tray floor', GOAL_POSES, {
        'coupling_a': {'pos': (0.12, 0.26, 0.7775), 'yaw_deg': 90},
    }, 0.5, {}),
    ('alternative order sleeves parked on supported section ends', GOAL_POSES, {
        'coupling_a': {'pos': (0.2, 0.125, 0.816)},
        'coupling_b': {'pos': (0.2, -0.125, 0.816)},
    }, 0.5, {}),
    ('alternative order completed repair', GOAL_POSES, {
    }, 1, {}),
    ('red team: left joint coupled, right sleeve set down on the bare table between its tray and the run', GOAL_POSES, {
        'coupling_b': {'pos': (0.12, -0.15, 0.7737), 'yaw_deg': -90},
    }, 0.5, {}),  # same progress as the author's 0.5 state with the sleeve on the 4 mm tray floor; setting an intact ABS sleeve down on the bench is not a failure, so 4 mm of tray floor should not swing the score from 0.5 to a permanent 0
    ('red team: section seated in line with both sleeves parked on the old stubs', GOAL_POSES, {
        'coupling_a': {'pos': (0.2, 0.217, 0.8165)},
        'coupling_b': {'pos': (0.2, -0.217, 0.8165)},
    }, 0.5, {}),  # two of the three subgoals are done (sleeves staged and the span seated in line); it earns the same 0.25 as staging the sleeves with the section untouched in its cradles
    ('section on support no sleeves on run', GOAL_POSES, {
        'coupling_a': {'pos': (0.12, 0.26, 0.782), 'yaw_deg': 90},
        'coupling_b': {'pos': (0.12, -0.26, 0.782), 'yaw_deg': -90},
    }, 0, {}),
    ('oversized decoy at left joint', GOAL_POSES, {
        'coupling_a': {'pos': (0.12, 0.26, 0.782), 'yaw_deg': 90},
        'large_coupling': {'pos': (0.2, 0.1685, 0.8117)},
    }, 0.5, {}),
    ('left sleeve only 20 mm on section', GOAL_POSES, {
        'coupling_a': {'pos': (0.2, 0.185, 0.8165)},
    }, 0.5, {}),
    ('sleeve stood on end', GOAL_POSES, {
        'coupling_a': {'pos': (0.2, 0.1685, 0.8175), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.5, {}),
    ('section six mm low', GOAL_POSES, {
        'section': {'pos': (0.2, 0, 0.8115)},
    }, 0.25, {}),
    ('section six degrees skewed', GOAL_POSES, {
        'section': {'pos': (0.2, 0, 0.817), 'yaw_deg': 6},
    }, 0.25, {}),
    ('section on top of stubs', GOAL_POSES, {
        'section': {'pos': (0.2, 0, 0.8495)},
    }, 0.25, {}),
    ('moving section', GOAL_POSES, {
        'section': {'pos': (0.2, 0, 0.817), 'vel': (0.2, 0.0, 0.0)},
    }, 0.75, {}),
    ('held section', GOAL_POSES, {
    }, 0.75, {'TEST_GRIPPERS': "{'left': (np.array([PIPE_X, 0, SUPPORT_AXIS_Z]), 0.), 'right': (np.array([.09,-.245,TOP+.16]),1.)}"}),
    ('section set down on the bench under the gap, sleeves still on the stubs', GOAL_POSES, {
        'section': {'pos': (0.2, 0, 0.7635)},
    }, 0.25, {}),  # the section is not in line, so only the staging credit remains (both sleeves overlap a stub by 36.5 mm); a bench set-down is not a failure
    ('sleeve set down on the bench below its joint', GOAL_POSES, {
        'coupling_a': {'pos': (0.2, 0.1685, 0.7735)},
    }, 0.5, {}),  # section in line with exactly one joint coupled; the loose sleeve on the bench earns nothing
    ('right sleeve fell off the bench', GOAL_POSES, {
        'coupling_b': {'pos': (0.4, -0.9, 0.3)},
    }, 0, {}),  # a part that falls off the bench zeroes the episode even with the left joint coupled
    ('section fell off the bench, sleeves parked', START_POSES, {
        'section': {'pos': (0.4, -0.9, 0.3)},
        'coupling_a': {'pos': (0.2, 0.217, 0.8165)},
        'coupling_b': {'pos': (0.2, -0.217, 0.8165)},
    }, 0, {}),  # staging credit is lost once the section has fallen off the bench
    ('all off table', GOAL_POSES, {
        'section': {'pos': (0.4, 1, 0.2)},
        'coupling_a': {'pos': (0.4, 1.2, 0.2)},
        'coupling_b': {'pos': (0.4, 1.4, 0.2)},
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
