"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest clear-ferrous-items-off-mri-cart    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("clear_ferrous_items_off_mri_cart.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 3000


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
    'scissors': {'pos': (0.185, 0.17, 0.7508)},
    'penlight': {'pos': (0.32, 0.19, 0.7508)},
    'phone': {'pos': (0.25, -0.267, 0.7508)},
    'steth': {'pos': (0.25, 0.064, 0.7618), 'joints': {'a01': 0.0, 'a02': 0.0, 'a03': -14.999999999999998, 'a04': 0.0, 'a05': 0.0, 'a06': 0.0, 'a07': 0.0, 'a08': 0.0, 'a09': 14.999999999999998, 'a10': 0.0, 'b01': 0.0, 'b02': 0.0, 'b03': 0.0, 'b04': -18.6, 'b05': 0.0, 'b06': 0.0, 'b07': 0.0, 'b08': 0.0, 'b09': 0.0, 'b10': 18.6, 'chest': 0.0, 'headset': 0.0}},
    'pad': {'pos': (0.25, -0.235, 0.7641)},
    'wedge': {'pos': (0.25, -0.06, 0.7508)},
    'bottle': {'pos': (0.36, 0.05, 0.7508)},
}
GOAL_POSES = {
    'scissors': {'pos': (0.022, 0, 0.7508)},
    'penlight': {'pos': (0.092, 0.026, 0.7508)},
    'phone': {'pos': (0.055, -0.04, 0.7665)},
    'steth': {'pos': (0.085, -0.04, 0.8865), 'joints': {'a01': 20.0, 'a02': -36.0, 'a03': 0.0, 'a04': -36.0, 'a05': 0.0, 'a06': -36.0, 'a07': 0.0, 'a08': -36.0, 'a09': 0.0, 'a10': -36.0, 'b01': 20.0, 'b02': 36.0, 'b03': 0.0, 'b04': 36.0, 'b05': 0.0, 'b06': 36.0, 'b07': 0.0, 'b08': 36.0, 'b09': 0.0, 'b10': 36.0, 'chest': 0.0, 'headset': 0.0}},
    'pad': {'pos': (0.34, -0.235, 0.7508)},
    'wedge': {'pos': (0.25, -0.06, 0.7508)},
    'bottle': {'pos': (0.36, 0.05, 0.7508)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'phone', 'pad', 'bottle', 'steth', 'penlight', 'scissors', 'wedge'}
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
    ('only the scissors are in the bin', START_POSES, {
        'scissors': {'pos': (0.022, 0, 0.7508)},
    }, 0.2, {}),
    ('scissors and penlight in the bin', START_POSES, {
        'scissors': {'pos': (0.022, 0, 0.7508)},
        'penlight': {'pos': (0.092, 0.026, 0.7508)},
    }, 0.4, {}),
    ('scissors, penlight and phone in the bin, pad set aside, stethoscope still on the cart', START_POSES, {
        'scissors': {'pos': (0.022, 0, 0.7508)},
        'penlight': {'pos': (0.092, 0.026, 0.7508)},
        'phone': {'pos': (0.055, -0.04, 0.7665)},
        'pad': {'pos': (0.34, -0.235, 0.7508)},
    }, 0.6, {}),
    ('all four hazards in the bin but the bottle was knocked over on the tray', GOAL_POSES, {
        'bottle': {'pos': (0.36, 0.05, 0.7815), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.8, {}),  # Hazards 4 x 0.2; the last 0.2 needs every MR-safe item kept, with the bottle upright.
    ('full credit', GOAL_POSES, {
    }, 1, {}),
    ('everything swept into the bin, pad included', GOAL_POSES, {
        'pad': {'pos': (0.055, 0, 0.7805)},
    }, 0, {}),  # An MR-safe item in the bin is over-clearing; the trial scores 0.
    ('the wedge was knocked off the cart onto the table', GOAL_POSES, {
        'wedge': {'pos': (0.25, 0.42, 0.7478)},
    }, 0, {}),
    ('the bottle was knocked off the cart onto the table', GOAL_POSES, {
        'bottle': {'pos': (0.36, 0.45, 0.7478)},
    }, 0, {}),
    ('the pad was put in the bin and the stethoscope left on the cart', START_POSES, {
        'scissors': {'pos': (0.022, 0, 0.7508)},
        'penlight': {'pos': (0.092, 0.026, 0.7508)},
        'phone': {'pos': (0.055, -0.04, 0.7665)},
        'pad': {'pos': (0.055, 0.1, 0.7508)},
    }, 0, {}),  # Three hazards are in the bin, but an MR-safe item in the bin zeroes the trial.
    ('the phone is held hovering above the bin, not in it', GOAL_POSES, {
        'phone': {'pos': (0.055, -0.04, 0.8675)},
    }, 0.6, {}),  # The phone is above the rim, so it is not in the bin; the last 0.2 needs all four hazards.
    ('finished, but the right gripper still grips the pad on the tray', GOAL_POSES, {
    }, 0.8, {'TEST_GRIPPERS': "{'left': (np.array([0.20, 0.33, TOP + 0.133]), 1.0), 'right': (np.array([0.34, -0.235, TOP + 0.03]), 0.0)}"}),  # The MR-safe credit needs both grippers at least 3 cm from every MR-safe item.
    ('finished, but the left gripper still holds the scissors in the bin', GOAL_POSES, {
    }, 0.2, {'TEST_GRIPPERS': "{'left': (np.array([0.022, 0.02, TOP + 0.02]), 0.0), 'right': (np.array([0.20, -0.33, TOP + 0.133]), 1.0)}"}),  # The gripper is within 3 cm of the scissors, phone and stethoscope boxes, so only the penlight counts as released.
    ('the scissors were put on the table beside the bin instead of in it', GOAL_POSES, {
        'scissors': {'pos': (0.05, 0.25, 0.7478)},
    }, 0.6, {}),
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


def _run_steps(monkeypatch, frames):
    """Score a sequence of layouts as consecutive 20 Hz control steps of one episode; return the scores."""
    run = {"env": None, "t": 0.0, "step": 0, "episode": 9001}
    monkeypatch.setattr(T, "_RUN", run)
    monkeypatch.setattr(T, "TEST_GRIPPERS", {"left": (np.array([0.20, 0.33, T.TOP + 0.133]), 1.0),
                                             "right": (np.array([0.20, -0.33, T.TOP + 0.133]), 1.0)})
    scores = []
    for k, objects in enumerate(frames):
        run["step"], run["t"] = k + 1, k / 20.0
        scores.append(T.score(objects))
    return scores


def test_full_credit_needs_a_second_of_stillness_on_the_gpu_path(monkeypatch):
    scores = _run_steps(monkeypatch, [GOAL] * 30)
    assert scores[0] == 0.0 and scores[19] < 1.0 and scores[21] == 1.0


def test_an_item_creeping_1_mm_per_step_earns_nothing(monkeypatch):
    frames = []
    for k in range(40):
        poses = {**GOAL_POSES, "penlight": {"pos": (0.092, 0.026 + 0.001 * k, 0.7508)}}
        frames.append(layout(poses))
    scores = _run_steps(monkeypatch, frames)
    assert max(scores) == pytest.approx(0.6)


def test_a_penlight_spinning_slowly_about_its_own_axis_counts_as_at_rest(monkeypatch):
    frames = [layout({**GOAL_POSES, "penlight": {"pos": (0.092, 0.026, 0.7508), "tilt_deg": 1.0 * k,
                                                  "tilt_axis": "y"}}) for k in range(30)]
    assert _run_steps(monkeypatch, frames)[-1] == 1.0


def test_an_mr_safe_item_taken_back_out_of_the_bin_no_longer_counts_against_the_trial(monkeypatch):
    in_bin = layout({**GOAL_POSES, "pad": {"pos": (0.055, 0.10, 0.7508)}})
    scores = _run_steps(monkeypatch, [in_bin] * 5 + [GOAL] * 30)
    assert scores[4] == 0.0 and scores[-1] == 1.0
