"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest clip-car-air-freshener-onto-vent-vane    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("clip_car_air_freshener_onto_vent_vane.py"))
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
    'vent': {'pos': (0.24, -0.12, 0.8775), 'joints': {'vane1': 0.0, 'vane2': 0.0, 'vane3': 0.0, 'vane4': 0.0}},
    'freshener': {'pos': (0.17, -0.06, 0.7695), 'joints': {'prong': 0.0}},
}
GOAL_POSES = {
    'vent': {'pos': (0.24, -0.12, 0.8775), 'joints': {'vane1': 0.0, 'vane2': 0.0, 'vane3': 0.0, 'vane4': 0.0}},
    'freshener': {'pos': (0.232, -0.12, 0.8848), 'joints': {'prong': 0.0004}},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'freshener'}
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
    ('home on the third vane, let go', START_POSES, {
        'freshener': {'pos': (0.232, -0.12, 0.8848), 'joints': {'prong': 0.0004}},
    }, 1, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('home on the bottom vane', START_POSES, {
        'freshener': {'pos': (0.232, -0.12, 0.8548), 'joints': {'prong': 0.0004}},
    }, 1, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('home on the top vane', START_POSES, {
        'freshener': {'pos': (0.232, -0.12, 0.8998), 'joints': {'prong': 0.0004}},
    }, 1, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('root gap 1.9 mm (still home)', START_POSES, {
        'freshener': {'pos': (0.2301, -0.12, 0.8848), 'joints': {'prong': 0.0004}},
    }, 1, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('engaged, root gap 4 mm (not home)', START_POSES, {
        'freshener': {'pos': (0.228, -0.12, 0.8848), 'joints': {'prong': 0.0004}},
    }, 0.5, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('engaged, root gap 11.5 mm', START_POSES, {
        'freshener': {'pos': (0.2205, -0.12, 0.8848), 'joints': {'prong': 0.0004}},
    }, 0.5, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('home, vane and clip turned 4 degrees', START_POSES, {
        'vent': {'pos': (0.24, -0.12, 0.8775), 'joints': {'vane1': 0.0, 'vane2': 0.0, 'vane3': 4.0, 'vane4': 0.0}},
        'freshener': {'pos': (0.232, -0.12, 0.886), 'tilt_deg': 4, 'tilt_axis': 'y', 'joints': {'prong': 0.0004}},
    }, 1, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('home, vane and clip turned 8 degrees (aim minor)', START_POSES, {
        'vent': {'pos': (0.24, -0.12, 0.8775), 'joints': {'vane1': 0.0, 'vane2': 0.0, 'vane3': 8.0, 'vane4': 0.0}},
        'freshener': {'pos': (0.2321, -0.12, 0.8871), 'tilt_deg': 8, 'tilt_axis': 'y', 'joints': {'prong': 0.0004}},
    }, 0.75, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('home, clip yawed 20 degrees (square minor)', START_POSES, {
        'freshener': {'pos': (0.2325, -0.1227, 0.8848), 'yaw_deg': 20, 'joints': {'prong': 0.0004}},
    }, 0.75, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('home, yawed 20 and the vane at 8 degrees (two minors)', START_POSES, {
        'vent': {'pos': (0.24, -0.12, 0.8775), 'joints': {'vane1': 0.0, 'vane2': 0.0, 'vane3': 8.0, 'vane4': 0.0}},
        'freshener': {'pos': (0.2326, -0.1227, 0.887), 'yaw_deg': 20.181, 'tilt_deg': 8, 'tilt_axis': 'y', 'quat': (0.012113084546138431, 0.06869671616600713, 0.17322517943366061, 0.9824088108221349), 'joints': {'prong': 0.0004}},
    }, 0.5, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('root gap 4 mm and the vane at 8 degrees', START_POSES, {
        'vent': {'pos': (0.24, -0.12, 0.8775), 'joints': {'vane1': 0.0, 'vane2': 0.0, 'vane3': 8.0, 'vane4': 0.0}},
        'freshener': {'pos': (0.2282, -0.12, 0.8877), 'tilt_deg': 8, 'tilt_axis': 'y', 'joints': {'prong': 0.0004}},
    }, 0.25, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('home, yawed 35 degrees (square major)', START_POSES, {
        'freshener': {'pos': (0.2334, -0.1246, 0.8848), 'yaw_deg': 35, 'joints': {'prong': 0.0004}},
    }, 0.25, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('home but still pinched', START_POSES, {
        'freshener': {'pos': (0.232, -0.12, 0.8848), 'joints': {'prong': 0.0004}},
    }, 0, {'TEST_GRIPPERS': '{"right": (np.array([FACE_X - 0.011, VENT_Y, TOP + VENT_Z + VANE_Z[2] + 0.0034]), 0.39)}'}),
    ('clip left standing on the mat', START_POSES, {
    }, 0, {}),
    ('clip lying face down on the mat', START_POSES, {
        'freshener': {'pos': (0.17, -0.06, 0.7585), 'tilt_deg': 90, 'tilt_axis': 'y', 'quat': (0.0, -0.7071067811865475, 0.0, 0.7071067811865476), 'joints': {'prong': 0.0}},
    }, 0, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('fork only 6 mm onto the vane (root gap 14 mm)', START_POSES, {
        'freshener': {'pos': (0.218, -0.12, 0.8848), 'joints': {'prong': 0.0004}},
    }, 0, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('vane on the upper prong, not between the prongs', START_POSES, {
        'freshener': {'pos': (0.232, -0.12, 0.8823), 'joints': {'prong': 0.0}},
    }, 0, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('vane under the lower prong', START_POSES, {
        'freshener': {'pos': (0.232, -0.12, 0.8882), 'joints': {'prong': 0.0}},
    }, 0, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('pushed into the gap between two vanes (no vane gripped)', START_POSES, {
        'freshener': {'pos': (0.232, -0.12, 0.8773), 'joints': {'prong': 0.0}},
    }, 0, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('home but the vent knocked 20 degrees with it', START_POSES, {
        'vent': {'pos': (0.24, -0.12, 0.8775), 'joints': {'vane1': 0.0, 'vane2': 0.0, 'vane3': 20.0, 'vane4': 0.0}},
        'freshener': {'pos': (0.2329, -0.12, 0.8905), 'tilt_deg': 20, 'tilt_axis': 'y', 'joints': {'prong': 0.0004}},
    }, 0, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('home but another vane knocked 16 degrees', START_POSES, {
        'vent': {'pos': (0.24, -0.12, 0.8775), 'joints': {'vane1': -16.0, 'vane2': 0.0, 'vane3': 0.0, 'vane4': 0.0}},
        'freshener': {'pos': (0.232, -0.12, 0.8848), 'joints': {'prong': 0.0004}},
    }, 0, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ("past the vane's end (fork off the blade)", START_POSES, {
        'freshener': {'pos': (0.232, -0.0675, 0.8848), 'joints': {'prong': 0.0}},
    }, 0, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}'}),
    ('home but it fails the 1 N hold check', START_POSES, {
        'freshener': {'pos': (0.232, -0.12, 0.8848), 'joints': {'prong': 0.0004}},
    }, 0, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0)}', 'TEST_HOLD': 'False'}),
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
