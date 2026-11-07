"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest hoist-bucket-onto-scaffold-deck    (in an environment with yam_arena installed)
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
from dataclasses import replace

TASK = load(Path(__file__).with_name("hoist_bucket_onto_scaffold_deck.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 1000


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
    'hoist_a': {'pos': (0.18, -0.03, 0.7475), 'joints': {'sheave': 0.0}},
    'rope': {'pos': (0.18, 0, 0.7845), 'joints': {}},
}
GOAL_POSES = {
    'hoist_a': {'pos': (0.18, -0.03, 0.7475), 'joints': {'sheave': 0.0}},
    'rope': {'pos': (0.18, 0.13, 0.9225), 'joints': {}},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {}


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
    ('start', START_POSES, {
    }, 0, {}),
    ('hoisted and hanging twenty mm above deck', START_POSES, {
        'rope': {'pos': (0.18, 0, 0.9425), 'joints': {}},
    }, 0.25, {}),
    ('landed at demo spot', GOAL_POSES, {
    }, 1, {}),
    ('landed five mm short of twenty mm margin', GOAL_POSES, {
        'rope': {'pos': (0.18, 0.095, 0.9225), 'joints': {}},
    }, 0.5, {}),
    ('landed while holding bail leg', GOAL_POSES, {
    }, 0.5, {'TEST_GRIPPERS': '{"left": ((0.146, 0.130, TOP + 0.230), 0.0), "right": ((0.1, -0.3, TOP + 0.2), 1.0)}'}),
    ('landed pail with a hand under the deck boards', GOAL_POSES, {
    }, 1, {'TEST_GRIPPERS': '{"left": ((0.180, 0.130, DECK_TOP - 0.050), 1.0), "right": ((0.1, -0.3, TOP + 0.2), 1.0)}'}),
    ('pail resting one mm into the boards', GOAL_POSES, {
        'rope': {'pos': (0.18, 0.13, 0.9215), 'joints': {}},
    }, 1, {}),
    ('landed with the load fall paid out slack', GOAL_POSES, {
    }, 1, {'TEST_LOAD_SLACK': '0.020'}),
    ("bail carried in an open gripper's throat, TCP 30 mm past the wire, rope slack", START_POSES, {
        'rope': {'pos': (0.18, 0, 0.8325), 'joints': {}},
    }, 0, {'TEST_LOAD_SLACK': '0.020', 'TEST_GRIPPERS': '{"left": ((0.180, 0.0, TOP + 0.085 + 0.075 - 0.030), 1.0), "right": ((0.1, -0.3, TOP + 0.2), 1.0)}'}),  # Bottom 50 mm above the table (inside the 10 mm over start to deck top minus 5 mm band) with the load fall 20 mm longer than its span, more than TAUT_EXTRA=8 mm, so the rope-load rule is a sticky zero after ROPE_LOAD_S=0.5 s. The TCP, 30 mm under the wire, is also 10 mm above the rim, inside HAND_CLEAR=12 mm.
    ('forearm lifting under the pail, TCP outside the footprint, rope slack', START_POSES, {
        'rope': {'pos': (0.18, 0, 0.8325), 'joints': {}},
    }, 0, {'TEST_LOAD_SLACK': '0.020', 'TEST_GRIPPERS': '{"left": ((0.180, 0.080, TOP + 0.030), 1.0), "right": ((0.1, -0.3, TOP + 0.2), 1.0)}'}),  # The TCP is 50 mm beside the pail and below it, so the 12 mm hand rule and the under-the-load rule miss it; the slack load fall in the raised band is the sticky zero.
    ('pail held by hand twenty mm over deck height, rope slack', START_POSES, {
        'rope': {'pos': (0.18, 0, 0.9425), 'joints': {}},
    }, 0, {'TEST_LOAD_SLACK': '0.020'}),  # The 0.25 tier needs the pail hanging on an attached, taut rope; a slack load fall is not carrying it.
    ('pail held two mm over the deck by its bail leg', GOAL_POSES, {
        'rope': {'pos': (0.18, 0.13, 0.9245), 'joints': {}},
    }, 0.25, {'TEST_GRIPPERS': '{"left": ((0.146, 0.130, TOP + 0.232), 0.0), "right": ((0.1, -0.3, TOP + 0.2), 1.0)}'}),  # Bottom 2 mm above the deck top is more than LANDED_ABOVE=1 mm, so the pail is not landed; it still hangs on the taut rope.
    ('pail tilted fifty degrees', GOAL_POSES, {
        'rope': {'pos': (0.18, 0.13, 0.9225), 'tilt_deg': 50, 'tilt_axis': 'y', 'joints': {}},
    }, 0, {}),
    ('hand under raised pail', START_POSES, {
        'rope': {'pos': (0.18, 0, 0.9425), 'joints': {}},
    }, 0, {'TEST_GRIPPERS': '{"left": ((0.180, 0.0, TOP + 0.120), 1.0), "right": ((0.1, -0.3, TOP + 0.2), 1.0)}'}),
    ('hand at pail raised below deck', START_POSES, {
        'rope': {'pos': (0.18, 0, 0.8325), 'joints': {}},
    }, 0, {'TEST_GRIPPERS': '{"left": ((0.180, 0.0, TOP + 0.100), 0.0), "right": ((0.1, -0.3, TOP + 0.2), 1.0)}'}),
    ('hand ten mm from raised pail is caught', START_POSES, {
        'rope': {'pos': (0.18, 0, 0.8325), 'joints': {}},
    }, 0, {'TEST_GRIPPERS': '{"left": ((0.180, 0.040, TOP + 0.085), 1.0), "right": ((0.1, -0.3, TOP + 0.2), 1.0)}'}),  # Inside HAND_CLEAR=12 mm; sticky handlift is checked across restoration by redteam_check.py.
    ('hand fourteen mm from raised pail is clear', START_POSES, {
        'rope': {'pos': (0.18, 0, 0.8325), 'joints': {}},
    }, 0, {'TEST_GRIPPERS': '{"left": ((0.180, 0.044, TOP + 0.085), 1.0), "right": ((0.1, -0.3, TOP + 0.2), 1.0)}'}),  # Outside HAND_CLEAR=12 mm; low pose itself scores zero. Redteam checks no sticky flag and later full credit.
    ('red team: pail hovering 4 mm over the deck with hands clear (rope tied off)', GOAL_POSES, {
        'rope': {'pos': (0.18, 0.13, 0.9265), 'joints': {}},
    }, 0.25, {}),  # Suspended 4 mm above the boards (only possible with the hauling line snagged or tied off); a suspended pail is not landed. Check result: 1.0 with success True, so success does not require the pail to rest on the deck. Fix: a one-sided band, e.g. bottom from 5 mm below to 1 mm above DECK_TOP.
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
