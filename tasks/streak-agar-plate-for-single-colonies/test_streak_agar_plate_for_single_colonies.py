"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest streak-agar-plate-for-single-colonies    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("streak_agar_plate_for_single_colonies.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 2600


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
    'dish': {'pos': (0.165, 0, 0.7475)},
    'lid': {'pos': (0.165, 0, 0.7535)},
    'loop': {'pos': (0.13, -0.2, 0.8557), 'joints': {'neck': 0.0}},
}
GOAL_POSES = {
    'dish': {'pos': (0.165, 0, 0.7475)},
    'lid': {'pos': (0.165, 0, 0.7535)},
    'loop': {'pos': (0.275, -0.15, 0.753), 'tilt_deg': 90, 'tilt_axis': 'x', 'joints': {'neck': 0.0}},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_TRACE': 'nominal_trace()'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'lid', 'loop', 'dish'}
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
    ('two quadrants streaked, plate closed, loop discarded', GOAL_POSES, {
    }, 0.8, {'TEST_TRACE': 'nominal_trace()[:2]'}),
    ('one quadrant streaked, plate closed, loop discarded', GOAL_POSES, {
    }, 0.6, {'TEST_TRACE': 'nominal_trace()[:1]'}),
    ('streaked the other way round the plate (clockwise), plate closed, loop discarded', GOAL_POSES, {
    }, 1, {'TEST_TRACE': 'nominal_trace(-1)'}),
    ('all quadrants streaked but the lid left off-centre on the rim', GOAL_POSES, {
        'lid': {'pos': (0.171, 0, 0.762), 'tilt_deg': 8, 'tilt_axis': 'y'},
    }, 0.8, {'TEST_TRACE': 'nominal_trace()'}),
    ('all quadrants streaked, loop still lying across the closed plate', GOAL_POSES, {
        'loop': {'pos': (0.165, 0, 0.7655), 'tilt_deg': 90, 'tilt_axis': 'x', 'joints': {'neck': 0.0}},
    }, 0.8, {'TEST_TRACE': 'nominal_trace()'}),
    ('all quadrants streaked, plate closed, loop still held in the right hand', GOAL_POSES, {
    }, 0.8, {'TEST_TRACE': 'nominal_trace()', 'TEST_GRIPPERS': "{'left': (np.array([.10, .30, 1.0]), 1.0), 'right': (np.array([TRAY_XY[0], TRAY_XY[1], TOP + .010]), 0.0)}"}),
    ('all quadrants streaked, loop discarded, left hand still resting on the lid', GOAL_POSES, {
    }, 0.8, {'TEST_TRACE': 'nominal_trace()', 'TEST_GRIPPERS': "{'left': (np.array([DISH_XY[0], DISH_XY[1] + LID_PINCH_Y, LID_GRASP_Z]), 0.0), 'right': (np.array([.10, -.30, 1.0]), 1.0)}"}),
    ('all quadrants streaked, lid still sliding on the plate', GOAL_POSES, {
        'lid': {'pos': (0.165, 0, 0.7535), 'vel': (0.05, 0.0, 0.0)},
    }, 0.8, {'TEST_TRACE': 'nominal_trace()'}),
    ('quadrant 4 streak runs back into the specimen quarter', GOAL_POSES, {
    }, 0.8, {'TEST_TRACE': 'nominal_trace()[:2] + [np.vstack([nominal_trace()[2], straight((.015, 290.), (.030, 340.))])]'}),
    ('quadrant 3 zigzag without first dragging through quadrant 2', GOAL_POSES, {
    }, 0.6, {'TEST_TRACE': '[nominal_trace()[0], densified(sector_plan(3)[4:]), nominal_trace()[2]]'}),
    ('nothing streaked, plate closed, loop discarded', GOAL_POSES, {
    }, 0, {'TEST_TRACE': '[]'}),  # doing the end steps without streaking earns nothing
    ('quadrant 2 zigzag that never touches the specimen', GOAL_POSES, {
    }, 0, {'TEST_TRACE': '[densified(sector_plan(2)[4:]), nominal_trace()[1], nominal_trace()[2]]'}),  # a streak that never picks up specimen dilutes nothing
    ('one long scribble over the whole plate', GOAL_POSES, {
    }, 0, {'TEST_TRACE': '[spiral()]'}),  # a spiral drags the specimen back through every quadrant
    ('streaks only inside the specimen quarter', GOAL_POSES, {
    }, 0, {'TEST_TRACE': '[straight((.030, -30.), (.030, 30.), (.020, -30.), (.020, 30.))]'}),
    ('lid set down on the bench beside the plate, all quadrants streaked', GOAL_POSES, {
        'lid': {'pos': (0.165, 0.12, 0.748)},
    }, 0, {'TEST_TRACE': 'nominal_trace()'}),  # the inside of the lid touched the bench (contamination)
    ('lid put on the bench and then back on the plate', GOAL_POSES, {
    }, 0, {'TEST_TRACE': 'nominal_trace()', 'TEST_EVER': "{'lid_on_bench': True}"}),  # sticky violation; restoring the lid does not undo the contact
    ('lid carried away from the plate and later replaced', GOAL_POSES, {
    }, 0, {'TEST_TRACE': 'nominal_trace()', 'TEST_EVER': "{'lid_off_plate': True}"}),  # the plate was left uncovered
    ('agar gouged during streaking', GOAL_POSES, {
    }, 0, {'TEST_TRACE': 'nominal_trace()', 'TEST_EVER': "{'gouge': True}"}),  # a torn agar surface ruins the plate
    ('plate knocked over on its side', GOAL_POSES, {
        'dish': {'pos': (0.165, 0, 0.7915), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0, {'TEST_TRACE': 'nominal_trace()'}),
    ('loop tip pressed below the agar surface', GOAL_POSES, {
        'loop': {'pos': (0.165, 0, 0.8485), 'joints': {'neck': 0.0}},
    }, 0, {'TEST_TRACE': 'nominal_trace()'}),  # the tip is in the agar, a gouge
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
