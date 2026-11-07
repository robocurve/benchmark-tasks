"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest split-dual-power-across-pdus    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("split_dual_power_across_pdus.py"))
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
    'red_1': {'pos': (0.15, 0.03, 0.781)},
    'red_2': {'pos': (0.195, 0.03, 0.781)},
    'green_1': {'pos': (0.24, 0.03, 0.781)},
    'green_2': {'pos': (0.15, -0.03, 0.781)},
    'blue_1': {'pos': (0.195, -0.03, 0.781)},
    'blue_2': {'pos': (0.24, -0.03, 0.781)},
}
GOAL_POSES = {
    'red_1': {'pos': (0.24, 0.27, 0.785)},
    'red_2': {'pos': (0.24, -0.27, 0.785)},
    'green_1': {'pos': (0.36, 0.27, 0.785)},
    'green_2': {'pos': (0.3, -0.27, 0.785)},
    'blue_1': {'pos': (0.3, 0.27, 0.785)},
    'blue_2': {'pos': (0.36, -0.27, 0.785)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'red_1', 'red_2', 'blue_1', 'blue_2', 'green_2', 'green_1'}
    assert TASK.docs and TASK.docs.strip()


def test_the_start_scores_nothing():
    assert T.score(START) == 0.0
    assert not T.success(START)


def test_the_goal_scores_full_and_succeeds():
    assert T.score(GOAL) == 1.0
    assert T.success(GOAL)


def test_success_waits_for_the_objects_to_settle():
    moving = layout({n: {**p, "vel": (0.05, 0.0, 0.0)} for n, p in GOAL_POSES.items()})
    assert not T.success(moving)

# (name, the layout it starts from, what's moved, the score it should get)
CASES = [
    ('one red plug seated in PDU A', START_POSES, {
        'red_1': {'pos': (0.24, 0.27, 0.785)},
    }, 0.1),
    ('the red server split across both strips', START_POSES, {
        'red_1': {'pos': (0.24, 0.27, 0.785)},
        'red_2': {'pos': (0.24, -0.27, 0.785)},
    }, 0.3),
    ('red split and one green seated', START_POSES, {
        'red_1': {'pos': (0.24, 0.27, 0.785)},
        'red_2': {'pos': (0.24, -0.27, 0.785)},
        'green_1': {'pos': (0.36, 0.27, 0.785)},
    }, 0.4),
    ('PDU A filled with one of each colour, PDU B empty', START_POSES, {
        'red_1': {'pos': (0.24, 0.27, 0.785)},
        'green_1': {'pos': (0.36, 0.27, 0.785)},
        'blue_1': {'pos': (0.3, 0.27, 0.785)},
    }, 0.3),
    ('five plugs seated correctly, the last blue still on the tray', GOAL_POSES, {
        'blue_2': {'pos': (0.24, -0.03, 0.78)},
    }, 0.7),
    ('one plug standing 4 mm high in its socket still counts as seated', GOAL_POSES, {
        'red_1': {'pos': (0.24, 0.27, 0.789)},
    }, 1),
    ('one plug held 8 mm above its socket floor', GOAL_POSES, {
        'red_1': {'pos': (0.24, 0.27, 0.793)},
    }, 0.7),
    ('red team: all six plugs standing upside down in the correct sockets', GOAL_POSES, {
        'red_1': {'pos': (0.24, 0.27, 0.785), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'y'},
        'red_2': {'pos': (0.24, -0.27, 0.785), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'y'},
        'green_1': {'pos': (0.36, 0.27, 0.785), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'y'},
        'green_2': {'pos': (0.3, -0.27, 0.785), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'y'},
        'blue_1': {'pos': (0.3, 0.27, 0.785), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'y'},
        'blue_2': {'pos': (0.36, -0.27, 0.785), 'yaw_deg': 180, 'tilt_deg': 180, 'tilt_axis': 'y'},
    }, 1),  # visually the goal layout; every strip holds one red, one green, one blue, each fully down
    ('red team: one plug standing upside down (flipped end over end) in its socket', GOAL_POSES, {
        'red_1': {'pos': (0.24, 0.27, 0.785), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # the plug head is a uniform single-colour box with no marked top, so a flipped plug is physically and photographically identical to an upright one, fully down in its socket
    ('red team: an upside-down plug leaning 8 degrees in its socket', GOAL_POSES, {
        'green_2': {'pos': (0.3, -0.27, 0.7865), 'tilt_deg': 172, 'tilt_axis': 'x'},
    }, 1),  # identical to an upright plug leaning 8 degrees, which counts
    ('each arm plugs in its own column (both reds on PDU A, both blues on PDU B)', START_POSES, {
        'red_1': {'pos': (0.24, 0.27, 0.785)},
        'red_2': {'pos': (0.3, 0.27, 0.785)},
        'green_1': {'pos': (0.36, 0.27, 0.785)},
        'green_2': {'pos': (0.24, -0.27, 0.785)},
        'blue_1': {'pos': (0.3, -0.27, 0.785)},
        'blue_2': {'pos': (0.36, -0.27, 0.785)},
    }, 0),  # the red and blue servers each have both cords on one feed, so they lose power when it is cut
    ('both red plugs in PDU A, nothing else seated', START_POSES, {
        'red_1': {'pos': (0.24, 0.27, 0.785)},
        'red_2': {'pos': (0.3, 0.27, 0.785)},
    }, 0),  # the red server has no redundancy
    ("a plug lying across its socket's rim", GOAL_POSES, {
        'red_1': {'pos': (0.24, 0.27, 0.8), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.7),  # lying on the strip, not plugged in; the red server keeps only the other plug's 0.1
    ('a plug knocked over on the table', START_POSES, {
        'red_1': {'pos': (0.15, 0.12, 0.76), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0),  # knocking plugs over earns nothing
    ('a plug pushed off the table', GOAL_POSES, {
        'red_1': {'pos': (0.3, 0.8, 0.0225)},
    }, 0.7),  # on the floor, not plugged in
    ('a plug standing on top of a seated plug', START_POSES, {
        'red_1': {'pos': (0.24, 0.27, 0.785)},
        'red_2': {'pos': (0.24, 0.27, 0.83)},
    }, 0.1),  # only the lower plug is in the socket
]


@pytest.mark.parametrize(("name", "base", "moved", "expected"), CASES, ids=[c[0] for c in CASES])
def test_named_layouts(name, base, moved, expected):
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
