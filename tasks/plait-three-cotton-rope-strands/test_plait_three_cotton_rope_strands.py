"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest plait-three-cotton-rope-strands    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("plait_three_cotton_rope_strands.py"))
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


def place(name, pos, yaw_deg=0.0, tilt_deg=0.0, tilt_axis="x", vel=(0.0, 0.0, 0.0)):
    """The ObjectState of `name` with its centre at `pos`, turned and tipped; its bounding box follows the turn."""
    r = _rot(yaw_deg, tilt_deg, tilt_axis)
    pos, half = np.asarray(pos, float), _half(OBJ[name], r)
    q = ik.matrix_to_quat_xyzw(r)
    return ObjectState(pos, np.asarray(q, float), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)


def layout(poses):
    """Every object's state: the loose ones as given, the fixed ones where the task file puts them."""
    out = {o.name: place(o.name, o.pos, math.degrees(o.yaw)) for o in T.OBJECTS if o.mass is None}
    out.update({name: place(name, **p) for name, p in poses.items()})
    return out


START_POSES = {
}
GOAL_POSES = {
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_ROPE': 'plait_word.example(plait_word.plait_schedule(6), NAMES, rest_z=TOP + 0.0055, over_z=TOP + 0.0155)'}


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

# (name, the layout it starts from, what's moved, the score it should get, module overrides)
CASES = [
    ('six crossings started on the left', START_POSES, {
    }, 1, {'TEST_ROPE': 'plait_word.example(plait_word.plait_schedule(6, 1), NAMES, rest_z=TOP + 0.0055, over_z=TOP + 0.0155)'}),
    ('seven crossings', START_POSES, {
    }, 1, {'TEST_ROPE': 'plait_word.example(plait_word.plait_schedule(7), NAMES, rest_z=TOP + 0.0055, over_z=TOP + 0.0155)'}),
    ('three crossings', START_POSES, {
    }, 0.5, {'TEST_ROPE': 'plait_word.example(plait_word.plait_schedule(3), NAMES, rest_z=TOP + 0.0055, over_z=TOP + 0.0155)'}),
    ('wrong side at the fourth crossing, so three count', START_POSES, {
    }, 0.5, {'TEST_ROPE': 'plait_word.example([(0, True), (1, True), (0, True), (0, True), (1, True), (0, True)], NAMES, rest_z=TOP + 0.0055, over_z=TOP + 0.0155)'}),
    ('three good crossings, then the right strand over the middle again and again (a twist)', START_POSES, {
    }, 0.5, {'TEST_ROPE': 'plait_word.example([(0, True), (1, True), (0, True), (0, True), (0, True), (0, True)], NAMES, rest_z=TOP + 0.0055, over_z=TOP + 0.0155)'}),  # counting stops where the sides stop alternating; the twist adds nothing
    ('outer strands passed under the middle (an inside-out braid)', START_POSES, {
    }, 0, {'TEST_ROPE': 'plait_word.example([(0, False), (1, False)] * 3, NAMES, rest_z=TOP + 0.0055, over_z=TOP + 0.0155)'}),  # the instruction asks for over; under-crossings never count
    ('six crossings with a gripper still on the rope', START_POSES, {
    }, 0, {'TEST_ROPE': 'plait_word.example(plait_word.plait_schedule(6), NAMES, rest_z=TOP + 0.0055, over_z=TOP + 0.0155)', 'TEST_GRIPPERS': "{'right': ((0.45, -0.005, TOP + 0.015), GRIP_DOWN), 'left': ((0.15, 0.29, TOP + 0.19), GRIP_DOWN)}"}),  # no photo while a hand holds the rope (offline there is no earlier photo to keep)
    ('three good crossings, then a strand laid across both others', START_POSES, {
    }, 0.5, {'TEST_ROPE': 'plait_word.example([(0, True), (1, True), (0, True), (1, False), (0, True), (1, True)], NAMES, rest_z=TOP + 0.0055, over_z=TOP + 0.0155)'}),  # at the fourth crossing the inner strand lies over the outer one; counting stops after three
    ('a strand laid over and straight back, then three crossings', START_POSES, {
    }, 0.5, {'TEST_ROPE': 'plait_word.example([(0, True), (0, False), (0, True), (1, True), (0, True)], NAMES, rest_z=TOP + 0.0055, over_z=TOP + 0.0155)'}),  # a crossing undone straight away is not a crossing
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
