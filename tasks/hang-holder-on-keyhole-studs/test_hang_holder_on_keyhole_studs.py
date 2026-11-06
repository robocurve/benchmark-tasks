"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest hang-holder-on-keyhole-studs    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("hang_holder_on_keyhole_studs.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 1200


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


def place(name, pos, yaw_deg=0.0, tilt_deg=0.0, tilt_axis="x", vel=(0.0, 0.0, 0.0), quat=None):
    """The ObjectState of `name` with its centre at `pos`, turned and tipped; its bounding box follows the turn."""
    r = (_rot(yaw_deg, tilt_deg, tilt_axis) if quat is None else
         ik.quat_xyzw_to_matrix(np.asarray(quat, float) / np.linalg.norm(quat)))
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
    'bracket': {'pos': (0.22, 0.2, 0.7975)},
}
GOAL_POSES = {
    'bracket': {'pos': (0.326, 0.18, 0.837)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'bracket'}
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
    ('x_play_front', GOAL_POSES, {
        'bracket': {'pos': (0.321, 0.18, 0.837)},
    }, 1),
    ('x_play_back', GOAL_POSES, {
        'bracket': {'pos': (0.331, 0.18, 0.837)},
    }, 1),
    ('face_pen_1mm', GOAL_POSES, {
        'bracket': {'pos': (0.327, 0.18, 0.837)},
    }, 1),
    ('actual_face_pen_0p1mm', GOAL_POSES, {
        'bracket': {'pos': (0.3321, 0.18, 0.837)},
    }, 1),
    ('actual_face_pen_1mm', GOAL_POSES, {
        'bracket': {'pos': (0.333, 0.18, 0.837)},
    }, 1),
    ('lifted and staged within 50 mm of the panel', START_POSES, {
        'bracket': {'pos': (0.3, 0.18, 0.8585)},
    }, 0.25),
    ('both heads passed through the entry windows', START_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.8585)},
    }, 0.5),
    ('both necks in the slots but not seated', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.842)},
    }, 0.5),
    ('both necks seated and still', GOAL_POSES, {
    }, 1),
    ('seated with 1 mm solver penetration into both slot ends', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.836)},
    }, 1),
    ('shanks 0.4 mm below the slot ends, within the bearing tolerance', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.8374)},
    }, 1),
    ('inserted and held 0.8 mm below bearing', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.8378)},
    }, 0.5),
    ('red team: inserted and halfway down from the entries to the slots', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.848)},
    }, 0.5),  # plate is behind both heads and the shanks are moving up from the entries into the slots; this is more progress than heads-in-entries, but it falls between the 8 mm entry band and the slot band
    ('red team: inserted and 1 mm into the lowering stroke past the entry band', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.8495)},
    }, 0.5),  # plate is behind both heads and the shanks are still inside the entry windows near the slot mouths
    ('red team: inserted riding 9 mm high in the entries', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.8675)},
    }, 0.5),  # plate is behind both heads; shanks are in the entries 3 mm above the entry bottom, and the plate can't get there without passing both heads
    ('red team: inserted through the entries 5 mm off centre sideways', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.185, 0.8585)},
    }, 0.5),  # the 24 mm entries let a 5 mm shank sit 9.5 mm off centre; the plate is behind both heads, so insertion is complete
    ('yawed_4deg_at_goal', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.837), 'yaw_deg': 4},
    }, 0.5),  # a yawed holder has not squared its paired slots for full credit
    ('face_pen_4mm', GOAL_POSES, {
        'bracket': {'pos': (0.336, 0.18, 0.837)},
    }, 0),  # four millimetres of face penetration exceeds the contact tolerance and puts the origin behind the panel face
    ('still held 5 mm above the locked height', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.842)},
    }, 0.5),  # insertion credit only; 5 mm exceeds the 2 mm lock tolerance
    ('still held 30 mm above the locked height', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.867)},
    }, 0.5),  # insertion credit only; the plate is behind both heads with each shank centre 1 mm above the 4.5 mm entry floor limit, so both shanks are in the entries, but neither bears on its slot end
    ('moving at 0.2 metres per second at the locked pose', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.837), 'vel': (0.2, 0.0, 0.0)},
    }, 0.5),  # insertion remains visible but the lock term requires resting
    ('upside down at the goal', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.837), 'tilt_deg': 180, 'tilt_axis': 'x'},
    }, 0),  # handle below plate; tilt exceeds 5 degrees
    ('lying on its side at the goal', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.837), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0),
    ('hung on just the right stud tilted 10 degrees', START_POSES, {
        'bracket': {'pos': (0.326, 0.1854, 0.8462), 'tilt_deg': 10, 'tilt_axis': 'x'},
    }, 0),  # one feature cannot replace two; the global tilt gate also rejects this
    ('half the plate hangs past the side of the panel', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.31, 0.837)},
    }, 0),  # the whole plate width must be within the panel width
    ('standing on top of the rest wall', START_POSES, {
        'bracket': {'pos': (0.2305, 0.2, 0.8425)},
    }, 0),  # stacked on a fixture, outside the staging region
    ('turned 45 degrees at the goal', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.837), 'yaw_deg': 45},
    }, 0),  # corners go behind the panel face; centre alignment alone is insufficient
    ('carried halfway at time out', START_POSES, {
        'bracket': {'pos': (0.281, 0.18, 0.8585)},
    }, 0),  # staging credit begins only within 50 mm of the panel face
    ('plate in front of the heads without insertion', GOAL_POSES, {
        'bracket': {'pos': (0.312, 0.18, 0.8585)},
    }, 0.25),  # lifted and close, but the plate lies in front of the head faces and outside the shank span
    ('plate immediately in front of the heads at locked height', GOAL_POSES, {
        'bracket': {'pos': (0.313, 0.18, 0.837)},
    }, 0.25),
    ('handle resting on both stud heads without insertion', GOAL_POSES, {
        'bracket': {'pos': (0.312, 0.18, 0.8335)},
    }, 0.25),  # the handle spans both heads but the plate stays in front of them; no shank occupies either slot
    ('bracket behind the panel', GOAL_POSES, {
        'bracket': {'pos': (0.349, 0.18, 0.837)},
    }, 0),
    ('upright on the table in front of the panel', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.7925)},
    }, 0),  # lowest point must be at least 20 mm above the table
    ('on the table away from the panel', START_POSES, {
        'bracket': {'pos': (0.3, -0.3, 0.7925)},
    }, 0),
    ('leaning on the panel from the table', START_POSES, {
        'bracket': {'pos': (0.3242, 0.18, 0.7922), 'tilt_deg': 10, 'tilt_axis': 'y'},
    }, 0),
    ('off the table', GOAL_POSES, {
        'bracket': {'pos': (-0.3, 0.18, 0.5)},
    }, 0),
    ('one stud aligned but shifted 100 mm sideways', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.28, 0.837)},
    }, 0),
    ('lifted too high above the panel', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.9285)},
    }, 0),
    ('red team: held 2 mm below the seat with neither shank bearing', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.839)},
    }, 0.5),  # both shanks sit 2 mm below the slots' closed ends, so nothing bears and only a still gripper holds it up; the docstring and README require the shanks to bear on the closed ends
    ('red team: held 1.5 mm below the seat with neither shank bearing', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.8385)},
    }, 0.5),  # same as above with a 1.5 mm gap; the seated pose can't be 1.5 mm off the bearing surfaces
    ('red team: rolled 2 degrees with only one shank bearing', GOAL_POSES, {
        'bracket': {'pos': (0.326, 0.18, 0.8387), 'tilt_deg': 2, 'tilt_axis': 'x'},
    }, 0.5),  # one shank bears on its slot end and the other is about 3.5 mm below its own; the holder is only stable like this while held, so it isn't hung on both studs
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
