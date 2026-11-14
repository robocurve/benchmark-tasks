"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest calibrate-colorimeter-measure-sample    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("calibrate_colorimeter_measure_sample.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 5200


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
    'colorimeter': {'pos': (0.168, -0.005, 0.7475), 'joints': {'lid': 0.0, 'less': 0.0, 'more': 0.0, 'cal_button': 0.0}},
    'cuvette_a': {'pos': (0.19, -0.31, 0.7525)},
    'cuvette_b': {'pos': (0.19, -0.19, 0.7525)},
}
GOAL_POSES = {
    'colorimeter': {'pos': (0.168, -0.005, 0.7475), 'joints': {'lid': 0.0, 'less': 0.0, 'more': 0.0, 'cal_button': 0.0}},
    'cuvette_a': {'pos': (0.19, -0.31, 0.7515)},
    'cuvette_b': {'pos': (0.19, -0.19, 0.7515)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': 'nominal_instrument(0,True)'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'cuvette_b', 'cuvette_a'}
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
    ('reference stored only', GOAL_POSES, {
    }, 0.25, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': 'nominal_instrument(0,False)'}),
    ('recorded sample still in well', GOAL_POSES, {
        'cuvette_b': {'pos': (0.19, -0.045, 0.7775)},
    }, 0.5, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': 'nominal_instrument(0,True)'}),
    ('sample home blank upright on table', GOAL_POSES, {
        'cuvette_a': {'pos': (0.13, -0.15, 0.7475)},
    }, 0.75, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': 'nominal_instrument(0,True)'}),
    ('both home blank moving', GOAL_POSES, {
        'cuvette_a': {'pos': (0.19, -0.31, 0.7515), 'vel': (0.2, 0.0, 0.0)},
    }, 0.75, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': 'nominal_instrument(0,True)'}),
    ('swapped pockets', GOAL_POSES, {
        'cuvette_a': {'pos': (0.19, -0.19, 0.7515)},
        'cuvette_b': {'pos': (0.19, -0.31, 0.7515)},
    }, 1, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': 'nominal_instrument(0,True)'}),
    ('blank 2.5 mm off pocket centre', GOAL_POSES, {
        'cuvette_a': {'pos': (0.1925, -0.31, 0.7515)},
    }, 1, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': 'nominal_instrument(0,True)'}),
    ('red team: recorded, blank fully in its pocket leaning 7 degrees against the wall', GOAL_POSES, {
        'cuvette_a': {'pos': (0.19, -0.3088, 0.7523), 'tilt_deg': 7, 'tilt_axis': 'x'},
    }, 1, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': 'nominal_instrument(0,True)'}),  # The 15.5 mm pocket with 20 mm walls lets a fully inserted 12.5 mm body lean up to ~8.8 degrees; it is bottomed in the rack, released and still, so both cuvettes are returned.
    ('red team: recorded, sample fully in its pocket leaning 7 degrees against the wall', GOAL_POSES, {
        'cuvette_b': {'pos': (0.19, -0.1888, 0.7523), 'tilt_deg': 7, 'tilt_axis': 'x'},
    }, 1, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': 'nominal_instrument(0,True)'}),  # Sample bottomed in its pocket, leaning on the wall within the pocket's own geometric limit; it is in the rack.
    ('CAL on sample then reading sample', GOAL_POSES, {
        'cuvette_b': {'pos': (0.19, -0.045, 0.7775)},
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': "nominal_instrument(0,False,'sample')"}),  # Sample self-reference reads 0.00 rather than 0.12 AU.
    ('CAL on blank with ridged faces', GOAL_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': "nominal_instrument(0,False,'ridged_blank')"}),  # Wrong reference shifts the reading down by 0.60 AU.
    ('ridged blank CAL plus ridged sample', GOAL_POSES, {
        'colorimeter': {'pos': (0.168, -0.005, 0.7475), 'joints': {'lid': 0.06, 'less': 0.0, 'more': 0.0, 'cal_button': 0.0}},
        'cuvette_b': {'pos': (0.19, -0.045, 0.7775), 'yaw_deg': 90},
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': "nominal_instrument(0,False,'ridged_blank')"}),  # Measurement requires a clear-face sample and a clear-face blank reference throughout 1 s.
    ('CAL at starting wavelength then changing wavelength', GOAL_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[1]', 'TEST_INSTRUMENT': 'nominal_instrument(1,False,None)'}),  # Changing wavelength clears the reference.
    ('correct CAL sample read with ridged faces', GOAL_POSES, {
        'cuvette_b': {'pos': (0.19, -0.045, 0.7775), 'yaw_deg': 90},
    }, 0.25, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': 'nominal_instrument(0,False)'}),  # Ridged sample gives 0.72 AU; only reference credit remains.
    ('correct CAL sample read lid open', GOAL_POSES, {
        'cuvette_b': {'pos': (0.19, -0.045, 0.7775)},
    }, 0.25, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': 'nominal_instrument(0,False)'}),  # Open lid has no valid reading.
    ('correct CAL then CAL again on sample', GOAL_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': "nominal_instrument(0,False,'sample')"}),  # Sample CAL replaces the blank reference before the record.
    ('both returned without any record', GOAL_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': 'nominal_instrument(0,False,None)'}),  # Return tiers require a measurement record.
    ('record then cuvette lying on side in rack', GOAL_POSES, {
        'cuvette_a': {'pos': (0.19, -0.31, 0.7675), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.75, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': 'nominal_instrument(0,True)'}),  # Tipping over is not a loss; the blank is out of its pocket, so only the sample-returned tier applies.
    ('record then cuvette below table', GOAL_POSES, {
        'cuvette_a': {'pos': (0.19, -0.31, 0.7415)},
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': 'nominal_instrument(0,True)'}),  # Lowest bottom corner more than 5 mm below table is a sticky drop.
    ('record sample held in rack', GOAL_POSES, {
    }, 0.5, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': 'nominal_instrument(0,True)', 'TEST_GRIPPERS': "{'left': (np.r_[POCKETS[1],POCKET_FLOOR+.057],0.)}"}),  # Every TCP must be at least 30 mm from the box.
    ('untouched layout 0', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[0]', 'TEST_INSTRUMENT': "{'w': LAYOUTS[0][1], 'ref': None, 'record': False}"}),  # No reference and no measurement; zero presses is not progress.
    ('untouched layout 1', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[1]', 'TEST_INSTRUMENT': "{'w': LAYOUTS[1][1], 'ref': None, 'record': False}"}),  # No reference and no measurement; zero presses is not progress.
    ('untouched layout 2', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[2]', 'TEST_INSTRUMENT': "{'w': LAYOUTS[2][1], 'ref': None, 'record': False}"}),  # No reference and no measurement; zero presses is not progress.
    ('untouched layout 3', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[3]', 'TEST_INSTRUMENT': "{'w': LAYOUTS[3][1], 'ref': None, 'record': False}"}),  # No reference and no measurement; zero presses is not progress.
    ('untouched layout 4', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[4]', 'TEST_INSTRUMENT': "{'w': LAYOUTS[4][1], 'ref': None, 'record': False}"}),  # No reference and no measurement; zero presses is not progress.
    ('untouched layout 5', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[5]', 'TEST_INSTRUMENT': "{'w': LAYOUTS[5][1], 'ref': None, 'record': False}"}),  # No reference and no measurement; zero presses is not progress.
    ('untouched layout 6', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[6]', 'TEST_INSTRUMENT': "{'w': LAYOUTS[6][1], 'ref': None, 'record': False}"}),  # No reference and no measurement; zero presses is not progress.
    ('untouched layout 7', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[7]', 'TEST_INSTRUMENT': "{'w': LAYOUTS[7][1], 'ref': None, 'record': False}"}),  # No reference and no measurement; zero presses is not progress.
    ('untouched layout 8', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[8]', 'TEST_INSTRUMENT': "{'w': LAYOUTS[8][1], 'ref': None, 'record': False}"}),  # No reference and no measurement; zero presses is not progress.
    ('untouched layout 9', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[9]', 'TEST_INSTRUMENT': "{'w': LAYOUTS[9][1], 'ref': None, 'record': False}"}),  # No reference and no measurement; zero presses is not progress.
    ('untouched layout 10', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[10]', 'TEST_INSTRUMENT': "{'w': LAYOUTS[10][1], 'ref': None, 'record': False}"}),  # No reference and no measurement; zero presses is not progress.
    ('untouched layout 11', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[11]', 'TEST_INSTRUMENT': "{'w': LAYOUTS[11][1], 'ref': None, 'record': False}"}),  # No reference and no measurement; zero presses is not progress.
    ('untouched layout 12', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[12]', 'TEST_INSTRUMENT': "{'w': LAYOUTS[12][1], 'ref': None, 'record': False}"}),  # No reference and no measurement; zero presses is not progress.
    ('untouched layout 13', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[13]', 'TEST_INSTRUMENT': "{'w': LAYOUTS[13][1], 'ref': None, 'record': False}"}),  # No reference and no measurement; zero presses is not progress.
    ('untouched layout 14', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[14]', 'TEST_INSTRUMENT': "{'w': LAYOUTS[14][1], 'ref': None, 'record': False}"}),  # No reference and no measurement; zero presses is not progress.
    ('untouched layout 15', START_POSES, {
    }, 0, {'TEST_LAYOUT': 'LAYOUTS[15]', 'TEST_INSTRUMENT': "{'w': LAYOUTS[15][1], 'ref': None, 'record': False}"}),  # No reference and no measurement; zero presses is not progress.
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
