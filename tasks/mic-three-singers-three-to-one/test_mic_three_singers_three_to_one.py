"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest mic-three-singers-three-to-one    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("mic_three_singers_three_to_one.py"))
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
    'mic_red': {'pos': (0.14, 0.33, 0.7935)},
    'mic_green': {'pos': (0.2, 0.33, 0.7935)},
    'mic_blue': {'pos': (0.26, 0.33, 0.7935)},
}
GOAL_POSES = {
    'mic_red': {'pos': (0.28, 0.17, 0.7875), 'yaw_deg': 90},
    'mic_green': {'pos': (0.23, 0, 0.7875)},
    'mic_blue': {'pos': (0.28, -0.17, 0.7875), 'yaw_deg': 90},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'mic_blue', 'mic_red', 'mic_green'}
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
    ('the red mic placed, the others still in the case', START_POSES, {
        'mic_red': {'pos': (0.28, 0.17, 0.7875)},
    }, 0.1),
    ('red and green placed, keeping the 3:1 rule', START_POSES, {
        'mic_red': {'pos': (0.28, 0.17, 0.7875)},
        'mic_green': {'pos': (0.23, 0, 0.7875)},
    }, 0.5),
    ('green and blue placed, keeping the 3:1 rule', START_POSES, {
        'mic_green': {'pos': (0.23, 0, 0.7875)},
        'mic_blue': {'pos': (0.28, -0.17, 0.7875)},
    }, 0.5),
    ('red and blue placed, green still in the case', START_POSES, {
        'mic_red': {'pos': (0.28, 0.17, 0.7875)},
        'mic_blue': {'pos': (0.28, -0.17, 0.7875)},
    }, 0.3),
    ('another layout keeping the rule (red diagonal behind-left, green behind its singer)', START_POSES, {
        'mic_red': {'pos': (0.315, 0.155, 0.7875)},
        'mic_green': {'pos': (0.33, 0, 0.7875)},
        'mic_blue': {'pos': (0.28, -0.17, 0.7875)},
    }, 1),
    ('blue 5 cm in front of its singer (a spot the left arm reaches with its gripper low), green 5 cm behind its own', GOAL_POSES, {
        'mic_green': {'pos': (0.33, 0, 0.7875)},
        'mic_blue': {'pos': (0.23, -0.12, 0.7875), 'yaw_deg': 90},
    }, 1),
    ('green 58 mm in front of its singer, still keeping the rule', GOAL_POSES, {
        'mic_green': {'pos': (0.222, 0, 0.7875)},
    }, 1),
    ('red 65 mm from its singer (too far), green and blue placed', GOAL_POSES, {
        'mic_red': {'pos': (0.28, 0.185, 0.7875), 'yaw_deg': 90},
    }, 0.5),
    ('green turned about its axis, still placed', GOAL_POSES, {
        'mic_green': {'pos': (0.23, 0, 0.7875), 'yaw_deg': 45},
    }, 1),
    ('red and green placed, blue standing on the handover pad', GOAL_POSES, {
        'mic_blue': {'pos': (0.13, 0, 0.7885), 'yaw_deg': 90},
    }, 0.5),
    ('red team: blue stand held tilted 15 degrees in the right gripper above the pad, red and green placed', GOAL_POSES, {
        'mic_blue': {'pos': (0.13, 0, 0.8675), 'yaw_deg': 90, 'tilt_deg': 15, 'tilt_axis': 'x'},
    }, 0.5),  # a stand still in the gripper at time-out has not been knocked over; red and green placed and keeping the rule have earned 0.5
    ('red team: blue held in the right gripper mid-carry from the pad to its spot, red and green placed', GOAL_POSES, {
        'mic_blue': {'pos': (0.18, -0.06, 0.8725), 'yaw_deg': 90},
    }, 0.5),  # on the demo's handover route (pad to blue's spot passes within 15 cm of the green mic) the stand's bottom is 8.5 cm up, above the 8.2 cm carry height, so it counts as carried; it earns nothing and zeroes nothing, and red and green keep 0.5
    ('red team: green held in the left gripper mid-carry from the case past the placed red mic, blue in the case', START_POSES, {
        'mic_red': {'pos': (0.28, 0.17, 0.7875)},
        'mic_green': {'pos': (0.22, 0.22, 0.8725)},
    }, 0.1),  # green is being carried at travel height from the case to its spot, the direct route from the case passes the red mic; red is placed and has earned 0.1
    ("red team: blue held 5 mm past the pad's edge after lifting it off, red and green placed", GOAL_POSES, {
        'mic_blue': {'pos': (0.13, -0.04, 0.8725), 'yaw_deg': 90},
    }, 0.5),  # off the pad, blue is not stowed, but its bottom is 8.5 cm up, above the 8.2 cm carry height, so it counts as carried; it earns nothing and zeroes nothing, and red and green keep 0.5
    ('each mic straight in front of its own singer', START_POSES, {
        'mic_red': {'pos': (0.23, 0.12, 0.7875)},
        'mic_green': {'pos': (0.23, 0, 0.7875)},
        'mic_blue': {'pos': (0.23, -0.12, 0.7875)},
    }, 0),  # neighbouring mics 12 cm apart against 15 cm; the mix comb-filters (the failure the task is about)
    ('red placed in front of its singer, breaking the rule with green', GOAL_POSES, {
        'mic_red': {'pos': (0.23, 0.12, 0.7875), 'yaw_deg': 90},
    }, 0),  # one pair breaking the 3:1 rule loses the take
    ('the red stand knocked over at its spot', GOAL_POSES, {
        'mic_red': {'pos': (0.28, 0.19, 0.7625), 'yaw_deg': 90, 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # a knocked-over stand
    ('the red stand tipped 15 degrees, leaning', GOAL_POSES, {
        'mic_red': {'pos': (0.28, 0.17, 0.79), 'yaw_deg': 90, 'tilt_deg': 15, 'tilt_axis': 'x'},
    }, 0),  # a stand past 10 degrees counts as knocked over
    ('the red stand pushed off the table onto the floor', GOAL_POSES, {
        'mic_red': {'pos': (0.28, 0.8, 0.04), 'yaw_deg': 90},
    }, 0),  # a stand off the table
    ('the red stand balanced on top of its singer marker', GOAL_POSES, {
        'mic_red': {'pos': (0.28, 0.12, 0.8275), 'yaw_deg': 90},
    }, 0),  # not standing on the table, so red earns nothing, and as a live mic 13 cm from the green mic (against 3 x 5 = 15 cm) it breaks the 3:1 rule
    ('the red stand held in the air above its spot', GOAL_POSES, {
        'mic_red': {'pos': (0.28, 0.17, 0.8375), 'yaw_deg': 90},
    }, 0.5),  # a stand left mid-air earns nothing; green and blue keep theirs
    ('red and blue swapped', GOAL_POSES, {
        'mic_red': {'pos': (0.28, -0.17, 0.7875), 'yaw_deg': 90},
        'mic_blue': {'pos': (0.28, 0.17, 0.7875), 'yaw_deg': 90},
    }, 0.1),  # each mic must be by the singer of its own colour; only green counts
    ('all three mics left on the handover pad', START_POSES, {
        'mic_red': {'pos': (0.113, 0.017, 0.7885)},
        'mic_green': {'pos': (0.147, 0.017, 0.7885)},
        'mic_blue': {'pos': (0.13, -0.017, 0.7885)},
    }, 0),  # on the staging pad, far from every singer
    ("red team: all three straight in front of their singers, green pulled back to 6.1 cm so it isn't placed", START_POSES, {
        'mic_red': {'pos': (0.23, 0.12, 0.7875)},
        'mic_green': {'pos': (0.219, 0, 0.7875)},
        'mic_blue': {'pos': (0.23, -0.12, 0.7875)},
    }, 0),  # the comb-filtering layout the task is about; green, 1 mm past the 6 cm limit, is not placed, but it is a live mic 12.1 cm from the placed red and blue mics (against 3 x 5 = 15 cm), so the 3:1 rule is broken and the take scores 0
    ('red team: blue stand dumped 6 cm beside the placed green mic, outside its own 6 cm', GOAL_POSES, {
        'mic_blue': {'pos': (0.25, -0.055, 0.7875), 'yaw_deg': 90},
    }, 0),  # blue, 7.2 cm from its singer, is not placed, but it is a live mic 5.9 cm from the placed green mic (against 3 x 5 = 15 cm), so the 3:1 rule is broken and the take scores 0
    ('red team: red and blue placed, green at 6.1 cm right between red and green singers', START_POSES, {
        'mic_red': {'pos': (0.28, 0.17, 0.7875)},
        'mic_green': {'pos': (0.245, 0.05, 0.7875)},
        'mic_blue': {'pos': (0.28, -0.17, 0.7875)},
    }, 0),  # green, 6.1 cm from its singer, is not placed, but it is a live mic 12.5 cm from the placed red mic (against 3 x 5 = 15 cm), so the 3:1 rule is broken and the take scores 0
    ('red team: blue stand stacked on top of the placed green stand', GOAL_POSES, {
        'mic_blue': {'pos': (0.23, 0, 0.8675), 'yaw_deg': 90},
    }, 0),  # blue is not placed (its bottom is 8 cm above the table), and with its bottom under the 8.2 cm carry height it is a live mic 0 cm from the placed green mic (against 3 x 5 = 15 cm), so the 3:1 rule is broken and the take scores 0
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
