"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest put-a-binder-clip-on-a-stack-of-paper    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("put_a_binder_clip_on_a_stack_of_paper.py"))
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
    'paper_stack': {'pos': (0.2076, -0.025, 0.8229)},
    'binder_clip': {'pos': (0.15, -0.165, 0.749), 'yaw_deg': 90, 'joints': {'jaw_l': 0.0, 'handle_l': 0.0, 'jaw_r': 0.0, 'handle_r': 0.0}},
}
GOAL_POSES = {
    'paper_stack': {'pos': (0.2076, -0.025, 0.8229)},
    'binder_clip': {'pos': (0.2204, -0.109, 0.8759), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 5.781669148287533, 'handle_l': -164.47664321566774, 'jaw_r': -5.781669148287533, 'handle_r': 164.47664321566774}},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'paper_stack', 'binder_clip'}
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
    ('untouched start, the clip standing on the desk', START_POSES, {
    }, 0, {}),
    ('clip pushed fully on, both handles still standing up', GOAL_POSES, {
        'binder_clip': {'pos': (0.2204, -0.109, 0.8759), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 5.781669148287533, 'handle_l': 0.0, 'jaw_r': -5.781669148287533, 'handle_r': 0.0}},
    }, 0.5, {}),
    ('clip pushed fully on, only the front handle folded down', GOAL_POSES, {
        'binder_clip': {'pos': (0.2204, -0.109, 0.8759), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 5.781669148287533, 'handle_l': -164.47664321566774, 'jaw_r': -5.781669148287533, 'handle_r': 0.0}},
    }, 0.75, {}),
    ('clip let go 4 mm short of the back of the jaw, both handles folded', GOAL_POSES, {
        'binder_clip': {'pos': (0.2213, -0.109, 0.8798), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 5.781669148287533, 'handle_l': -164.47664321566774, 'jaw_r': -5.781669148287533, 'handle_r': 164.47664321566774}},
    }, 0.75, {}),
    ('clip let go 10 mm short of the back of the jaw (paper 9.5 mm in)', GOAL_POSES, {
        'binder_clip': {'pos': (0.2227, -0.109, 0.8856), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 5.781669148287533, 'handle_l': -164.47664321566774, 'jaw_r': -5.781669148287533, 'handle_r': 164.47664321566774}},
    }, 0.5, {}),
    ('clip let go 15 mm short of the back of the jaw (paper 4.5 mm in)', GOAL_POSES, {
        'binder_clip': {'pos': (0.2239, -0.109, 0.8905), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 5.781669148287533, 'handle_l': -164.47664321566774, 'jaw_r': -5.781669148287533, 'handle_r': 164.47664321566774}},
    }, 0.25, {}),
    ('paper 1.5 mm short of the back of the jaw still counts as fully on', GOAL_POSES, {
        'binder_clip': {'pos': (0.2207, -0.109, 0.8774), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 5.781669148287533, 'handle_l': -164.47664321566774, 'jaw_r': -5.781669148287533, 'handle_r': 164.47664321566774}},
    }, 1, {}),
    ('clip turned 5 degrees on the edge (one end deeper than the other)', GOAL_POSES, {
        'binder_clip': {'pos': (0.2204, -0.109, 0.8759), 'yaw_deg': 88.83, 'tilt_deg': 14.38, 'tilt_axis': 'x', 'quat': (0.11366168466438036, 0.05240213754212563, 0.6979119009153341, 0.7051624039758615), 'joints': {'jaw_l': 5.781669148287533, 'handle_l': -164.47664321566774, 'jaw_r': -5.781669148287533, 'handle_r': 164.47664321566774}},
    }, 0.75, {}),
    ('stack and clip pushed 4 mm back off the front wall', GOAL_POSES, {
        'paper_stack': {'pos': (0.2116, -0.025, 0.8229)},
        'binder_clip': {'pos': (0.2244, -0.109, 0.8759), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 5.781669148287533, 'handle_l': -164.47664321566774, 'jaw_r': -5.781669148287533, 'handle_r': 164.47664321566774}},
    }, 0.75, {}),
    ('stack and clip pushed 8 mm back off the front wall', GOAL_POSES, {
        'paper_stack': {'pos': (0.2156, -0.025, 0.8229)},
        'binder_clip': {'pos': (0.2284, -0.109, 0.8759), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 5.781669148287533, 'handle_l': -164.47664321566774, 'jaw_r': -5.781669148287533, 'handle_r': 164.47664321566774}},
    }, 0.5, {}),
    ('clip slid along the edge to the middle of the top edge', GOAL_POSES, {
        'binder_clip': {'pos': (0.2204, -0.025, 0.8759), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 5.781669148287533, 'handle_l': -164.47664321566774, 'jaw_r': -5.781669148287533, 'handle_r': 164.47664321566774}},
    }, 1, {}),
    ('done, but the right hand is still pinching the handles', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': '{"right": (np.array(GOAL_CLIP) + 0.040 * np.array(UP), 0.0), "left": (PARK["left"], 1.0)}'}),
    ('done, but the left hand is still holding the stack', GOAL_POSES, {
    }, 0, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0), "left": (np.array(STACK_POS) + 0.060 * np.array(UP) + np.array([0, 0.090, 0]), 0.0)}'}),
    ('done, both hands parked', GOAL_POSES, {
    }, 1, {'TEST_GRIPPERS': '{"right": (PARK["right"], 1.0), "left": (PARK["left"], 1.0)}'}),
    ('done, but the clip is still moving at 20 mm/s', GOAL_POSES, {
        'binder_clip': {'pos': (0.2204, -0.109, 0.8759), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'vel': (0.0, 0.0, -0.02), 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 5.781669148287533, 'handle_l': -164.47664321566774, 'jaw_r': -5.781669148287533, 'handle_r': 164.47664321566774}},
    }, 0, {}),
    ('clip let go with the paper only 2 mm into the jaw (on the rolled lips)', GOAL_POSES, {
        'binder_clip': {'pos': (0.2245, -0.109, 0.8929), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 5.781669148287533, 'handle_l': -164.47664321566774, 'jaw_r': -5.781669148287533, 'handle_r': 164.47664321566774}},
    }, 0, {}),  # the lips only touch the paper's top corners; it springs off
    ('clip shut on the paper with its jaw closed (the paper passed through a lip)', GOAL_POSES, {
        'binder_clip': {'pos': (0.2204, -0.109, 0.8759), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 0.0, 'handle_l': -164.47664321566774, 'jaw_r': 0.0, 'handle_r': 164.47664321566774}},
    }, 0, {}),  # a real jaw cannot be shut with paper between its lips
    ('clip shut beside the stack (on nothing), handles folded', GOAL_POSES, {
        'binder_clip': {'pos': (0.2087, -0.109, 0.8787), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 0.0, 'handle_l': -164.47664321566774, 'jaw_r': 0.0, 'handle_r': 164.47664321566774}},
    }, 0, {}),  # no paper between the lips
    ("clip hanging off the paper's end (its end 5 mm past the paper)", GOAL_POSES, {
        'binder_clip': {'pos': (0.2204, -0.119, 0.8759), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 5.781669148287533, 'handle_l': -164.47664321566774, 'jaw_r': -5.781669148287533, 'handle_r': 164.47664321566774}},
    }, 0, {}),  # only part of the jaw is on the paper
    ('clipped stack lifted out of the sorter and left on top of it', GOAL_POSES, {
        'paper_stack': {'pos': (0.2076, -0.025, 0.8829)},
        'binder_clip': {'pos': (0.2204, -0.109, 0.9359), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 5.781669148287533, 'handle_l': -164.47664321566774, 'jaw_r': -5.781669148287533, 'handle_r': 164.47664321566774}},
    }, 0, {}),  # the stack is clipped where it stands in the sorter
    ('stack knocked 15 mm back in its slot with the clip on it', GOAL_POSES, {
        'paper_stack': {'pos': (0.2226, -0.025, 0.8229)},
        'binder_clip': {'pos': (0.2354, -0.109, 0.8759), 'yaw_deg': 90, 'tilt_deg': 13.5, 'tilt_axis': 'x', 'quat': (0.08311101437304316, 0.08311101437304316, 0.7022054964822505, 0.7022054964822504), 'joints': {'jaw_l': 5.781669148287533, 'handle_l': -164.47664321566774, 'jaw_r': -5.781669148287533, 'handle_r': 164.47664321566774}},
    }, 0, {}),  # the stack no longer stands against its backstops
    ('clip dropped on the table', START_POSES, {
        'binder_clip': {'pos': (0.15, -0.2, 0.7495), 'joints': {'jaw_l': 0.0, 'handle_l': 0.0, 'jaw_r': 0.0, 'handle_r': 0.0}},
    }, 0, {}),  # the clip is not on the paper
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
