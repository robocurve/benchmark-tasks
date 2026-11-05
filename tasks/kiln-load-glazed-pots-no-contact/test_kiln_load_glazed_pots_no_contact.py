"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest kiln-load-glazed-pots-no-contact    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("kiln_load_glazed_pots_no_contact.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 2000


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
    'pot_green': {'pos': (0.18, 0.32, 0.7935)},
    'pot_blue': {'pos': (0.3, 0.32, 0.7885)},
    'pot_yellow': {'pos': (0.17, -0.32, 0.7835)},
    'pot_red': {'pos': (0.265, -0.32, 0.7785)},
    'pot_purple': {'pos': (0.355, -0.32, 0.771)},
}
GOAL_POSES = {
    'pot_green': {'pos': (0.2, 0.1, 0.7975)},
    'pot_blue': {'pos': (0.295, 0.095, 0.7925)},
    'pot_yellow': {'pos': (0.205, -0.1, 0.7875)},
    'pot_red': {'pos': (0.295, -0.095, 0.7825)},
    'pot_purple': {'pos': (0.205, 0, 0.775), 'yaw_deg': 90},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'pot_red', 'pot_purple', 'pot_green', 'pot_yellow', 'pot_blue'}
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
    ('the green pot loaded alone', START_POSES, {
        'pot_green': {'pos': (0.2, 0.1, 0.7975)},
    }, 0.2),
    ("the left arm's two pots loaded", START_POSES, {
        'pot_green': {'pos': (0.2, 0.1, 0.7975)},
        'pot_blue': {'pos': (0.295, 0.095, 0.7925)},
    }, 0.4),
    ("the right arm's three pots loaded", START_POSES, {
        'pot_yellow': {'pos': (0.205, -0.1, 0.7875)},
        'pot_red': {'pos': (0.295, -0.095, 0.7825)},
        'pot_purple': {'pos': (0.205, 0, 0.775)},
    }, 0.6),
    ('four pots loaded, purple still on its start mark', GOAL_POSES, {
        'pot_purple': {'pos': (0.355, -0.32, 0.77), 'yaw_deg': 90},
    }, 0.8),
    ('purple 10 mm from green, so both lose their credit', GOAL_POSES, {
        'pot_purple': {'pos': (0.205, 0.03, 0.775), 'yaw_deg': 90},
    }, 0.6),
    ('yellow 10 mm from the right wall', GOAL_POSES, {
        'pot_yellow': {'pos': (0.205, -0.125, 0.7875)},
    }, 0.8),
    ('red 14 mm from the back post', GOAL_POSES, {
        'pot_red': {'pos': (0.295, -0.0465, 0.7825)},
    }, 0.8),
    ('green in place but tilted 10 degrees', GOAL_POSES, {
        'pot_green': {'pos': (0.2, 0.1, 0.8029), 'tilt_deg': 10, 'tilt_axis': 'x'},
    }, 0.8),
    ("red team: red held 5 mm above the back post's top, nothing touching", GOAL_POSES, {
        'pot_red': {'pos': (0.31, 0, 0.8875)},
    }, 0.8),  # red is held with its bottom 5 mm above the back post's flat top, more than the 2 mm contact threshold, so nothing touches; red is off the shelf and earns nothing, and the other four pots keep their 0.2 each
    ('red team: purple toppled with its end 20 mm from upright green, green loses its credit', GOAL_POSES, {
        'pot_purple': {'pos': (0.205, 0.0225, 0.7775), 'yaw_deg': 90, 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.6),  # purple keeps the goal's 90-degree yaw, applied before the tilt, so it lies with its axis front to back and its curved side faces green; purple reaches y = 0.0225 + 0.025 = 0.0475 m and green's side is at y = 0.065 m, a 17.5 mm gap, under 20 mm, so green loses its credit and lying purple earns none; blue, yellow and red keep theirs. The next entry lies purple with its end toward green instead
    ('purple toppled with its end (not its side) 20 mm from upright green, green keeps its credit', GOAL_POSES, {
        'pot_purple': {'pos': (0.205, 0.0225, 0.7775), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.8),
    ('red team: green standing upside down in its target spot', GOAL_POSES, {
        'pot_green': {'pos': (0.2, 0.1, 0.7975), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 1),  # the pots are closed cylinders, the same at both ends, so an upside-down pot stands flat on the shelf and counts as upright; all five pots are loaded cleanly
    ('purple touching green', GOAL_POSES, {
        'pot_purple': {'pos': (0.205, 0.04, 0.775), 'yaw_deg': 90},
    }, 0),  # the glaze welds the two pots together, the failure the task is about
    ('yellow touching the front-right post', GOAL_POSES, {
        'pot_yellow': {'pos': (0.1783, -0.1267, 0.7875)},
    }, 0),  # the pot fuses to the shelf post
    ('red against the back wall', GOAL_POSES, {
        'pot_red': {'pos': (0.325, -0.095, 0.7825)},
    }, 0),  # the pot fuses to the kiln wall
    ('purple stacked on yellow inside the kiln', GOAL_POSES, {
        'pot_purple': {'pos': (0.205, -0.1, 0.845), 'yaw_deg': 90},
    }, 0),  # stacking glazed pots welds them together
    ('green knocked over, lying on its side in the kiln', GOAL_POSES, {
        'pot_green': {'pos': (0.2, 0.1, 0.7875), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.8),  # a toppled pot earns nothing; the other four still count
    ('purple pushed off the table onto the floor', GOAL_POSES, {
        'pot_purple': {'pos': (0.3, -0.8, 0.0235), 'yaw_deg': 90},
    }, 0.8),  # a pot off the table earns nothing
    ('green held 3 cm above its spot', GOAL_POSES, {
        'pot_green': {'pos': (0.2, 0.1, 0.8275)},
    }, 0.8),  # a pot in mid-air is not loaded
    ("green pushed against the outside of the kiln's left wall", GOAL_POSES, {
        'pot_green': {'pos': (0.25, 0.22, 0.7925)},
    }, 0.8),  # a pot outside the kiln earns nothing
    ('red team: green toppled lengthwise in the kiln with its end against the front-left post', GOAL_POSES, {
        'pot_green': {'pos': (0.205, 0.125, 0.7875), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0),  # green lies with its axis front to back and its front end's face at x = 0.16 m, flat against the side of the front-left post (centre x = 0.15 m, radius 10 mm); contact is measured between the actual surfaces, so this is glaze contact with a post and the whole score is 0
    ('red team: green toppled lengthwise in the kiln with its end against purple', GOAL_POSES, {
        'pot_green': {'pos': (0.205, 0.07, 0.7875), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # green lies with its axis side to side and its end at y = 0.07 - 0.045 = 0.025 m, exactly where purple's side is (purple's centre on the midline, radius 25 mm); two pots touching is glaze contact, so the whole score is 0
    ('red team: red leaning 20 degrees with its rim against the back post', GOAL_POSES, {
        'pot_red': {'pos': (0.31, -0.0437, 0.7892), 'tilt_deg': 20, 'tilt_axis': 'x'},
    }, 0),  # red leans 20 degrees toward the back post with its top rim against the post's side about 5 cm above the shelf; contact is measured between the actual surfaces, so this is glaze contact with a post and the whole score is 0
    ("red team: red sitting on the back wall's top with its centre just inside the kiln", GOAL_POSES, {
        'pot_red': {'pos': (0.349, -0.095, 0.8425)},
    }, 0),  # red's centre is 1 mm inside the back wall's inner face and its bottom stands on the wall's top edge; that is glaze contact with the wall, so the whole score is 0
    ("red team: red held 1 mm above the back wall's top, centre just inside the kiln", GOAL_POSES, {
        'pot_red': {'pos': (0.349, -0.095, 0.8435)},
    }, 0),  # red's bottom is 1 mm above the back wall's top edge, closer than the 2 mm contact threshold, so it counts as touching the wall and the whole score is 0
    ('red team: green toppled with its end 12 mm from upright purple, purple loses its credit', GOAL_POSES, {
        'pot_green': {'pos': (0.205, 0.082, 0.7875), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.6),  # green lies with its axis side to side and its end at y = 0.082 - 0.045 = 0.037 m, 12 mm from purple's side; that is not contact (over 2 mm) but is under the 20 mm gap, measured between the actual surfaces, so purple loses its credit and lying green earns none; blue, yellow and red keep theirs
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
