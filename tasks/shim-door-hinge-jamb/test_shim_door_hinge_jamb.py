"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest shim-door-hinge-jamb    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("shim_door_hinge_jamb.py"))
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
    out = {o.name: place(o.name, o.pos, math.degrees(o.yaw)) for o in T.OBJECTS if o.mass is None or o.name in ('shim_left', 'shim_right')}
    out.update({name: place(name, **p) for name, p in poses.items()})
    return out


START_POSES = {
    'jamb': {'pos': (0.26, 0, 0.7475), 'joints': {'setter': 0.0, 'jamb': 0.0}},
    'shim_left': {'pos': (0.1868, 0.2234, 0.7871)},
    'shim_right': {'pos': (0.1932, -0.2234, 0.7871), 'yaw_deg': 180},
}
GOAL_POSES = {
    'jamb': {'pos': (0.26, 0, 0.7475), 'joints': {'setter': 0.001, 'jamb': 0.00015}},
    'shim_left': {'pos': (0.26, 0.0364, 0.8775)},
    'shim_right': {'pos': (0.2651, -0.0364, 0.8775), 'yaw_deg': 180},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_GAP': '.005', 'TEST_GRIPPERS': 'None'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == set()
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
    ('untouched holders', START_POSES, {
    }, 0, {}),  # Neither shim is at hinge height or inside the wall gap.
    ('goal at 4 mm', GOAL_POSES, {
        'jamb': {'pos': (0.26, 0, 0.7475), 'joints': {'setter': 0.0, 'jamb': 0.00015}},
        'shim_left': {'pos': (0.26, 0.0546, 0.8775)},
        'shim_right': {'pos': (0.2641, -0.0546, 0.8775), 'yaw_deg': 180},
    }, 1, {'TEST_GAP': '0.004', 'TEST_GRIPPERS': 'None'}),  # The gap is set per trial; the physical pack fits it.
    ('goal at 6.35 mm', GOAL_POSES, {
        'jamb': {'pos': (0.26, 0, 0.7475), 'joints': {'setter': 0.00235, 'jamb': 0.00015}},
        'shim_left': {'pos': (0.26, 0.0117, 0.8775)},
        'shim_right': {'pos': (0.2665, -0.0117, 0.8775), 'yaw_deg': 180},
    }, 1, {'TEST_GAP': '0.00635', 'TEST_GRIPPERS': 'None'}),  # The gap is set per trial; the physical pack fits it.
    ('bow 0.8 mm', GOAL_POSES, {
        'jamb': {'pos': (0.26, 0, 0.7475), 'joints': {'setter': 0.001, 'jamb': 0.0008}},
        'shim_left': {'pos': (0.26, 0.0245, 0.8775)},
        'shim_right': {'pos': (0.2658, -0.0245, 0.8775), 'yaw_deg': 180},
    }, 0.75, {'TEST_GAP': '.005', 'TEST_GRIPPERS': 'None'}),  # A snug pack bows the spring-backed jamb; the bow tier caps credit.
    ('bow 1.3 mm', GOAL_POSES, {
        'jamb': {'pos': (0.26, 0, 0.7475), 'joints': {'setter': 0.001, 'jamb': 0.0013}},
        'shim_left': {'pos': (0.26, 0.0154, 0.8775)},
        'shim_right': {'pos': (0.2663, -0.0154, 0.8775), 'yaw_deg': 180},
    }, 0.5, {'TEST_GAP': '.005', 'TEST_GRIPPERS': 'None'}),  # A snug pack bows the spring-backed jamb; the bow tier caps credit.
    ('bow 1.8 mm', GOAL_POSES, {
        'jamb': {'pos': (0.26, 0, 0.7475), 'joints': {'setter': 0.001, 'jamb': 0.0018}},
        'shim_left': {'pos': (0.26, 0.0062, 0.8775)},
        'shim_right': {'pos': (0.2668, -0.0062, 0.8775), 'yaw_deg': 180},
    }, 0.25, {'TEST_GAP': '.005', 'TEST_GRIPPERS': 'None'}),  # A snug pack bows the spring-backed jamb; the bow tier caps credit.
    ('loose pack 0.8 mm', GOAL_POSES, {
        'shim_left': {'pos': (0.26, 0.051, 0.8775)},
        'shim_right': {'pos': (0.2651, -0.051, 0.8775), 'yaw_deg': 180},
    }, 0.25, {'TEST_GAP': '.005', 'TEST_GRIPPERS': 'None'}),  # A pair in place but 0.8 mm short of the jamb does not carry it: not snug, so 0.25.
    ('loose pack 2 mm', GOAL_POSES, {
        'shim_left': {'pos': (0.26, 0.0729, 0.8775)},
        'shim_right': {'pos': (0.2651, -0.0729, 0.8775), 'yaw_deg': 180},
    }, 0, {'TEST_GAP': '.005', 'TEST_GRIPPERS': 'None'}),  # Centre-line clearance determines whether the opposed pair is snug or loose.
    ('one shim carrying', GOAL_POSES, {
        'shim_left': {'pos': (0.26, 0.0002, 0.8775)},
        'shim_right': {'pos': (0.1932, -0.2234, 0.7851), 'yaw_deg': 180},
    }, 0.25, {'TEST_GAP': '.005', 'TEST_GRIPPERS': 'None'}),  # One thin-end-leading shim touches both planes at the mouth, but there is no opposed pair.
    ('one shim loose', GOAL_POSES, {
        'shim_left': {'pos': (0.26, 0.0367, 0.8775)},
        'shim_right': {'pos': (0.1932, -0.2234, 0.7851), 'yaw_deg': 180},
    }, 0, {'TEST_GAP': '.005', 'TEST_GRIPPERS': 'None'}),  # A single shim with 1 mm remaining clearance does not carry the jamb.
    ('both shims from the same face', GOAL_POSES, {
        'shim_right': {'pos': (0.26, 0.0364, 0.8775)},
    }, 0, {'TEST_GAP': '.005', 'TEST_GRIPPERS': 'None'}),  # There must be one thin-end-leading shim from each face.
    ('butt first', GOAL_POSES, {
        'shim_left': {'pos': (0.265, 0.0484, 0.8775), 'yaw_deg': 180},
    }, 0, {'TEST_GAP': '.005', 'TEST_GRIPPERS': 'None'}),  # A butt inside the wall rather than outside its entry face is not a valid side.
    ('inclined face against stud', GOAL_POSES, {
        'shim_left': {'pos': (0.26, 0.0364, 0.8775), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 0, {'TEST_GAP': '.005', 'TEST_GRIPPERS': 'None'}),  # The shim normals must face into the pack, with the stud-side flat face at smaller x.
    ('overlap beside the middle', GOAL_POSES, {
        'shim_left': {'pos': (0.26, 0.0016, 0.8775)},
        'shim_right': {'pos': (0.2651, -0.1316, 0.8775), 'yaw_deg': 180},
    }, 0.25, {'TEST_GAP': '.005', 'TEST_GRIPPERS': 'None'}),  # Overlap -100 to -30 mm misses the middle. A alone carries at the mouth, earning only the single-shim tier.
    ('pack 30 mm above hinge', GOAL_POSES, {
        'shim_left': {'pos': (0.26, 0.0364, 0.9075)},
        'shim_right': {'pos': (0.2651, -0.0364, 0.9075), 'yaw_deg': 180},
    }, 0, {'TEST_GAP': '.005', 'TEST_GRIPPERS': 'None'}),  # The pack centre height must be within 25 mm of the hinge.
    ('overlap only 60 mm', GOAL_POSES, {
        'shim_left': {'pos': (0.26, 0.0716, 0.8775)},
        'shim_right': {'pos': (0.2651, -0.0716, 0.8775), 'yaw_deg': 180},
    }, 0, {'TEST_GAP': '.005', 'TEST_GRIPPERS': 'None'}),  # At least 70 mm of overlap is required.
    ('held snug goal', GOAL_POSES, {
    }, 0.25, {'TEST_GAP': '.005', 'TEST_GRIPPERS': "{'left': (pair_layout(.005)['shim_left'].origin, 0.0)}"}),  # Every TCP must clear both shims and the jamb by 30 mm.
    ('goal still moving', GOAL_POSES, {
        'shim_left': {'pos': (0.26, 0.0364, 0.8775), 'vel': (0.003, 0.0, 0.0)},
    }, 0.25, {'TEST_GAP': '.005', 'TEST_GRIPPERS': 'None'}),  # Offline velocity checks cap a moving pack at the release tier.
    ('both shims turned over snug', GOAL_POSES, {
        'shim_left': {'pos': (0.2651, 0.0364, 0.8775), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'x'},
        'shim_right': {'pos': (0.26, -0.0364, 0.8775), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1, {}),  # Both flat faces are exchanged; the opposed inclined faces still form a snug centred pack at 0.15 mm bow.
    ('memorised depth (pack 5.40 mm) in a 4.0 mm gap', GOAL_POSES, {
        'jamb': {'pos': (0.26, 0, 0.7475), 'joints': {'setter': 0.0, 'jamb': 0.0014}},
        'shim_left': {'pos': (0.26, 0.0318, 0.8775)},
        'shim_right': {'pos': (0.2654, -0.0318, 0.8775), 'yaw_deg': 180},
    }, 0.5, {'TEST_GAP': '0.004', 'TEST_GRIPPERS': 'None'}),  # A fixed pack sized for a 5.25 mm gap must bow a 4.0 mm trial's jamb 1.4 mm: 0.5, not 1.0.
    ('memorised depth (pack 5.40 mm) in a 6.35 mm gap', GOAL_POSES, {
        'jamb': {'pos': (0.26, 0, 0.7475), 'joints': {'setter': 0.00235, 'jamb': 0.0}},
        'shim_left': {'pos': (0.26, 0.0318, 0.8775)},
        'shim_right': {'pos': (0.2663, -0.0318, 0.8775), 'yaw_deg': 180},
    }, 0.25, {'TEST_GAP': '0.00635', 'TEST_GRIPPERS': 'None'}),  # The same fixed pack is 0.95 mm loose in a 6.35 mm trial: 0.25 (on the simulator it cannot even stay up).
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
