"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest slide-a-mic-into-its-desk-stand-clip-and-aim-it    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("slide_a_mic_into_its_desk_stand_clip_and_aim_it.py"))
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
    'stand': {'pos': (0.165, 0, 0.7477), 'yaw_deg': -20, 'joints': {'head': 0.0, 'jaw_l': 0.0, 'jaw_r': 0.0}},
    'mic': {'pos': (0.21, -0.1082, 0.7591), 'yaw_deg': -90},
}
GOAL_POSES = {
    'stand': {'pos': (0.165, 0, 0.7477), 'yaw_deg': -60, 'joints': {'head': 0.0, 'jaw_l': -0.023260073260073233, 'jaw_r': 0.023260073260073233}},
    'mic': {'pos': (0.147, 0.0312, 0.9407), 'yaw_deg': -60, 'tilt_deg': 6.013, 'tilt_axis': 'y', 'quat': (0.0262242127837904, 0.045421668930022245, -0.49931181706812267, 0.8648334359815256)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'stand', 'mic'}
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
    ('untouched start, the mic lying on the table', START_POSES, {
    }, 0, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ('seated with 8.5 mm out behind the clip, aimed', GOAL_POSES, {
        'mic': {'pos': (0.1517, 0.023, 0.9407), 'yaw_deg': -60, 'tilt_deg': 6.013, 'tilt_axis': 'y', 'quat': (0.0262242127837904, 0.045421668930022245, -0.49931181706812267, 0.8648334359815256)},
    }, 1, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ('in the clip but only 4 mm out behind it (not snug)', GOAL_POSES, {
        'mic': {'pos': (0.154, 0.0191, 0.9407), 'yaw_deg': -60, 'tilt_deg': 6.013, 'tilt_axis': 'y', 'quat': (0.0262242127837904, 0.045421668930022245, -0.49931181706812267, 0.8648334359815256)},
    }, 0.5, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ('seated but the stand never turned', GOAL_POSES, {
        'stand': {'pos': (0.165, 0, 0.7477), 'yaw_deg': -20, 'joints': {'head': 0.0, 'jaw_l': -0.023260073260073233, 'jaw_r': 0.023260073260073233}},
        'mic': {'pos': (0.1312, 0.0123, 0.9407), 'yaw_deg': -20, 'tilt_deg': 6.013, 'tilt_axis': 'y', 'quat': (0.009107573521310044, 0.0516516161322373, -0.1734091742428864, 0.9834522972386014)},
    }, 0.25, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ('seated, aimed 9 degrees off (miss 47 mm)', GOAL_POSES, {
        'stand': {'pos': (0.165, 0, 0.7477), 'yaw_deg': -51, 'joints': {'head': 0.0, 'jaw_l': -0.023260073260073233, 'jaw_r': 0.023260073260073233}},
        'mic': {'pos': (0.1423, 0.028, 0.9407), 'yaw_deg': -51, 'tilt_deg': 6.013, 'tilt_axis': 'y', 'quat': (0.022579629216967448, 0.04733917710461743, -0.42991855603068074, 0.9013429967753542)},
    }, 0.75, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ("seated backwards (grille toward the clip's back)", GOAL_POSES, {
        'mic': {'pos': (0.183, -0.0312, 0.9407), 'yaw_deg': 120, 'tilt_deg': 6.013, 'tilt_axis': 'y', 'quat': (-0.045421668930022245, 0.0262242127837904, 0.8648334359815256, 0.49931181706812267)},
    }, 0.25, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ('seated, the stand slid 35 mm off its spot', GOAL_POSES, {
        'stand': {'pos': (0.2, 0, 0.7477), 'yaw_deg': -60, 'joints': {'head': 0.0, 'jaw_l': -0.023260073260073233, 'jaw_r': 0.023260073260073233}},
        'mic': {'pos': (0.182, 0.0312, 0.9407), 'yaw_deg': -60, 'tilt_deg': 6.013, 'tilt_axis': 'y', 'quat': (0.0262242127837904, 0.045421668930022245, -0.49931181706812267, 0.8648334359815256)},
    }, 0.75, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (PARK["right"], 1.0)}'}),
    ('seated and aimed but the right hand still holds the mic', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': '{"left": (PARK["left"], 1.0), "right": (np.array(mic_seated(0.036)[0]) + 0.08 * np.array([math.cos(AIM_DEG * DEG), math.sin(AIM_DEG * DEG), 0.0]), 0.3)}'}),
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
