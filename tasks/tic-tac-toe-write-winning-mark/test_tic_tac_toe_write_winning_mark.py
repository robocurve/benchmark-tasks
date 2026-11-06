"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest tic-tac-toe-write-winning-mark    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("tic_tac_toe_write_winning_mark.py"))
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
    'pen': {'pos': (0.155, -0.31, 0.8955), 'joints': {'nib': 0.0}},
}
GOAL_POSES = {
    'pen': {'pos': (0.155, -0.31, 0.8945), 'joints': {'nib': 0.0}},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_BOARD': 'LAYOUTS[0]', 'TEST_STROKES': 'nominal_strokes(0)'}


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'pen'}
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
    ('right mark pen lying on table', GOAL_POSES, {
        'pen': {'pos': (0.195, -0.155, 0.7625), 'tilt_deg': 90, 'tilt_axis': 'y', 'joints': {'nib': 0.0}},
    }, 0.6, {'TEST_BOARD': 'LAYOUTS[0]', 'TEST_STROKES': 'nominal_strokes(0)'}),
    ('right mark pen hung moving 5 cm per second', GOAL_POSES, {
        'pen': {'pos': (0.155, -0.31, 0.8945), 'vel': (0.05, 0.0, 0.0), 'joints': {'nib': 0.0}},
    }, 0.8, {'TEST_BOARD': 'LAYOUTS[0]', 'TEST_STROKES': 'nominal_strokes(0)'}),
    ('right mark, pen hung 8 mm further out along the slot', GOAL_POSES, {
        'pen': {'pos': (0.155, -0.302, 0.8945), 'joints': {'nib': 0.0}},
    }, 1, {'TEST_BOARD': 'LAYOUTS[0]', 'TEST_STROKES': 'nominal_strokes(0)'}),
    ('right mark, pen hung past the prong tips', GOAL_POSES, {
        'pen': {'pos': (0.155, -0.269, 0.8945), 'joints': {'nib': 0.0}},
    }, 0.6, {'TEST_BOARD': 'LAYOUTS[0]', 'TEST_STROKES': 'nominal_strokes(0)'}),
    ('right mark, pen hung against the rear plate', GOAL_POSES, {
        'pen': {'pos': (0.155, -0.323, 0.8945), 'joints': {'nib': 0.0}},
    }, 1, {'TEST_BOARD': 'LAYOUTS[0]', 'TEST_STROKES': 'nominal_strokes(0)'}),
    ('right O traced twice (one bolder O in the photo)', GOAL_POSES, {
    }, 1, {'TEST_BOARD': 'LAYOUTS[0]', 'TEST_STROKES': 'nominal_strokes(0) * 2'}),
    ('right X traced twice (one bolder X in the photo)', GOAL_POSES, {
    }, 1, {'TEST_BOARD': 'LAYOUTS[8]', 'TEST_STROKES': 'nominal_strokes(8) * 2'}),
    ('regular pentagon reads as a hand-drawn O (corners 14 percent out)', GOAL_POSES, {
    }, 1, {'TEST_BOARD': 'LAYOUTS[0]', 'TEST_STROKES': 'polygon((0, 0), .017, 5)'}),
    ('right O plus one touch-down dab in the cell corner (4.4 mm of the 5 mm allowance)', GOAL_POSES, {
    }, 1, {'TEST_BOARD': 'LAYOUTS[0]', 'TEST_STROKES': 'nominal_strokes(0) + dabs([cell_center((0, 0)) + [.020, .020]])'}),
    ('red team: X with a 1.4 mm nib skip in one diagonal', GOAL_POSES, {
    }, 1, {'TEST_BOARD': 'LAYOUTS[8]', 'TEST_STROKES': '[np.linspace(cell_center((0,1))+np.array([-.015,-.015]), cell_center((0,1))+np.array([-.0005,-.0005]), 40), np.linspace(cell_center((0,1))+np.array([.0005,.0005]), cell_center((0,1))+np.array([.015,.015]), 40), np.linspace(cell_center((0,1))+np.array([-.015,.015]), cell_center((0,1))+np.array([.015,-.015]), 90)]'}),  # module: TEST_BOARD: LAYOUTS[8]; TEST_STROKES: "[np.linspace(cell_center((0,1))+np.array([-.015,-.015]), cell_center((0,1))+np.array([-.0005,-.0005]), 40), np.linspace(cell_center((0,1))+np.array([.0005,.0005]), cell_center((0,1))+np.array([.015,.015]), 40), np.linspace(cell_center((0,1))+np.array([-.015,.015]), cell_center((0,1))+np.array([.015,-.015]), 90)]". A one-step contact loss on the 120 N/m sprung nib leaves a 1.4 mm gap that the 2.2 mm-radius ink dots cover. The ink and the README's 'top photo and ruler' show a clean X, yet the stroke-count rule scores 0.
    ('red team: O with a 0.8 mm nib skip (two arcs)', GOAL_POSES, {
    }, 1, {'TEST_BOARD': 'LAYOUTS[0]', 'TEST_STROKES': '[cell_center((0,0)) + .015*np.column_stack((np.cos(np.radians(np.linspace(0,200,60))), np.sin(np.radians(np.linspace(0,200,60))))), cell_center((0,0)) + .015*np.column_stack((np.cos(np.radians(np.linspace(203,390,55))), np.sin(np.radians(np.linspace(203,390,55)))))]'}),  # module: TEST_BOARD: LAYOUTS[0]; TEST_STROKES: "[cell_center((0,0)) + .015*np.column_stack((np.cos(np.radians(np.linspace(0,200,60))), np.sin(np.radians(np.linspace(0,200,60))))), cell_center((0,0)) + .015*np.column_stack((np.cos(np.radians(np.linspace(203,390,55))), np.sin(np.radians(np.linspace(203,390,55)))))]". A sub-millimetre skip in the circle is invisible in the ink and in a photo, and the O is correct, but it scores 0 because O needs exactly one substantial stroke.
    ('red team: X filling the cell (40 mm square, inside the printed lines)', GOAL_POSES, {
    }, 1, {'TEST_BOARD': 'LAYOUTS[8]', 'TEST_STROKES': 'strokes_x((0,1), .040)'}),  # module: TEST_BOARD: LAYOUTS[8]; TEST_STROKES: "strokes_x((0,1), .040)". A 40 mm X sits 1.25 mm inside the 42.5 mm cell interior and touches no line, so it is a normal tic-tac-toe X. Its 56.6 mm strokes exceed the 52 mm cap, so it scores 0 as 'too large' while staying inside its own square.
    ('red team: O of radius 20 mm inside the cell', GOAL_POSES, {
    }, 1, {'TEST_BOARD': 'LAYOUTS[0]', 'TEST_STROKES': 'strokes_o((0,0), .020, 390.)'}),  # module: TEST_BOARD: LAYOUTS[0]; TEST_STROKES: "strokes_o((0,0), .020, 390.)". A 40 mm O stays inside the 42.5 mm cell interior and is a normal O, but it exceeds the 19 mm radius cap and scores 0.
    ('red team: correct X plus a 2 mm touch-down tick in the same cell', GOAL_POSES, {
    }, 1, {'TEST_BOARD': 'LAYOUTS[8]', 'TEST_STROKES': 'strokes_x((0,1)) + [np.linspace(cell_center((0,1))+np.array([-.017,-.012]), cell_center((0,1))+np.array([-.017,-.010]), 5)]'}),  # module: TEST_BOARD: LAYOUTS[8]; TEST_STROKES: "strokes_x((0,1)) + [np.linspace(cell_center((0,1))+np.array([-.017,-.012]), cell_center((0,1))+np.array([-.017,-.010]), 5)]". A 2 mm tick from the nib landing beside a clean X doesn't make the mark unclear to a reader, but it counts as a third substantial stroke and zeroes the score.
    ('red team: O drawn with a quarter-turn overlap (450 degrees)', GOAL_POSES, {
    }, 1, {'TEST_BOARD': 'LAYOUTS[0]', 'TEST_STROKES': 'strokes_o((0,0), .015, 450.)'}),  # module: TEST_BOARD: LAYOUTS[0]; TEST_STROKES: "strokes_o((0,0), .015, 450.)". An O that overlaps itself by 90 degrees looks the same as a single O in the photo but fails the 420-degree winding cap.
    ('wrong symbol in winning cell', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[0][1]]', 'TEST_STROKES': 'cheat_cases()[0][2]'}),  # Only the inferred side may be written.
    ('right symbol in another empty cell', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[1][1]]', 'TEST_STROKES': 'cheat_cases()[1][2]'}),  # Ink in another square is permanent.
    ('block instead of win', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[2][1]]', 'TEST_STROKES': 'cheat_cases()[2][2]'}),  # The winning move takes precedence over blocking elsewhere.
    ('scribble filling winning cell', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[3][1]]', 'TEST_STROKES': 'cheat_cases()[3][2]'}),  # A filled scribble is ambiguous.
    ('X over an existing O', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[4][1]]', 'TEST_STROKES': 'cheat_cases()[4][2]'}),  # Existing printed marks must not be overwritten.
    ('X and O together', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[5][1]]', 'TEST_STROKES': 'cheat_cases()[5][2]'}),  # Two symbols are unclear.
    ('right mark plus 5 mm stray', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[6][1]]', 'TEST_STROKES': 'cheat_cases()[6][2]'}),  # Permanent stray ink is a sticky violation even if the pen returns.
    ('square instead of O', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[7][1]]', 'TEST_STROKES': 'cheat_cases()[7][2]'}),  # A square's corners lie 22 percent outside its fitted circle, beyond the 18 percent band.
    ('6 mm mark', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[8][1]]', 'TEST_STROKES': 'cheat_cases()[8][2]'}),  # A 6 mm mark is too small.
    ('right mark 5 mm over grid', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[9][1]]', 'TEST_STROKES': 'cheat_cases()[9][2]'}),  # The glyph crosses the winning zone into a neighbouring square.
    ('pen hung no mark', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[10][1]]', 'TEST_STROKES': 'cheat_cases()[10][2]'}),  # Hanging alone does not complete the game.
    ('V instead of X', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[11][1]]', 'TEST_STROKES': 'cheat_cases()[11][2]'}),  # The strokes meet but do not cross; an X needs 8 mm of each line on both sides of the crossing.
    ('lambda instead of X', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[12][1]]', 'TEST_STROKES': 'cheat_cases()[12][2]'}),  # One line stops at the crossing.
    ('X arms crossing by only 5 mm', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[13][1]]', 'TEST_STROKES': 'cheat_cases()[13][2]'}),  # Each line must continue at least 8 mm past the crossing.
    ('triangle instead of O', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[14][1]]', 'TEST_STROKES': 'cheat_cases()[14][2]'}),  # A triangle's corners lie far outside the 18 percent band around its fitted circle.
    ('small square instead of O', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[15][1]]', 'TEST_STROKES': 'cheat_cases()[15][2]'}),  # At the 10 mm minimum radius the square's corners still put more than 5 mm of ink outside the band.
    ('dabbed O around the winning X', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[16][1]]', 'TEST_STROKES': 'cheat_cases()[16][2]'}),  # Each dab counts 4.4 mm of centreline, so the ring of dabs off the X lines exceeds the 5 mm allowance; X plus O is unclear.
    ('dabbed X over a printed O', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[17][1]]', 'TEST_STROKES': 'cheat_cases()[17][2]'}),  # Every dab outside the winning zone counts 4.4 mm, over the 1.0 mm outside limit.
    ('dotted 24 mm line into another square', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[18][1]]', 'TEST_STROKES': 'cheat_cases()[18][2]'}),  # The dots beyond the zone edge count 4.4 mm each.
    ('right mark plus one dab in another square', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[19][1]]', 'TEST_STROKES': 'cheat_cases()[19][2]'}),  # A single stationary touch outside the zone counts 4.4 mm, over the 1.0 mm limit.
    ('right mark plus two dabs off the glyph', GOAL_POSES, {
    }, 0, {'TEST_BOARD': 'LAYOUTS[cheat_cases()[20][1]]', 'TEST_STROKES': 'cheat_cases()[20][2]'}),  # Two dabs count 8.8 mm off the glyph, over the 5 mm allowance.
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
