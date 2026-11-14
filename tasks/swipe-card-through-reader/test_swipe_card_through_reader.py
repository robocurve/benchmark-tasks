"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest swipe-card-through-reader    (in an environment with yam_arena installed)
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

TASK = load(Path(__file__).with_name("swipe_card_through_reader.py"))
T = TASK.module
OBJ = {o.name: o for o in T.OBJECTS}
RUN_LIMIT_STEPS = 2400


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
    out = {o.name: place(o.name, o.pos, math.degrees(o.yaw)) for o in T.OBJECTS if o.mass is None or o.name in ('card',)}
    out.update({name: place(name, **p) for name, p in poses.items()})
    return out


START_POSES = {
    'reader': {'pos': (0.316, -0.3072, 0.7475), 'yaw_deg': -90, 'joints': {'head': 0.0}},
    'card': {'pos': (0.165, -0.405, 0.748)},
}
GOAL_POSES = {
    'reader': {'pos': (0.316, -0.3072, 0.7475), 'yaw_deg': -90, 'joints': {'head': 0.0}},
    'card': {'pos': (0.235, -0.41, 0.7785), 'tilt_deg': 90, 'tilt_axis': 'y'},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)

GOAL_MODULE = {'TEST_HISTORY': 'swipe_history("clean")'}


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
    ('clean swipe, card back in the stand but still pinched', GOAL_POSES, {
    }, 0.5, {'TEST_HISTORY': 'swipe_history("clean")', 'TEST_GRIPPERS': '{"left": (np.array([0.10, 0.27, TOP + 0.16]), 1.0), "right": (np.array([XS, YS, STAND_FLOOR_Z + TCP_ABOVE_EDGE]), 0.0)}'}),
    ('clean swipe, card back in the stand but still sliding at 20 mm/s', GOAL_POSES, {
        'card': {'pos': (0.235, -0.41, 0.7785), 'tilt_deg': 90, 'tilt_axis': 'y', 'vel': (0.0, 0.02, 0.0)},
    }, 0.5, {'TEST_HISTORY': 'swipe_history("clean")'}),
    ('clean swipe, card left standing in the reader slot', GOAL_POSES, {
        'card': {'pos': (0.3688, -0.3051, 0.7823), 'yaw_deg': 90, 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0.5, {'TEST_HISTORY': 'swipe_history("clean")'}),
    ('clean swipe, card standing on its edge on the counter beside the stand', GOAL_POSES, {
        'card': {'pos': (0.175, -0.41, 0.7745), 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0.5, {'TEST_HISTORY': 'swipe_history("clean")'}),  # Not back in its stand.
    ('clean swipe, card back in the stand facing the other way', GOAL_POSES, {
        'card': {'pos': (0.235, -0.41, 0.7785), 'yaw_deg': 180, 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 1, {'TEST_HISTORY': 'swipe_history("clean")'}),  # Any face toward the robot counts as back in the stand.
    ('clean swipe, card back in the stand stripe edge up', GOAL_POSES, {
        'card': {'pos': (0.235, -0.41, 0.7785), 'yaw_deg': 180, 'tilt_deg': 90, 'tilt_axis': 'y', 'quat': (0.0, -0.7071067811865476, 0.0, 0.7071067811865475)},
    }, 1, {'TEST_HISTORY': 'swipe_history("clean")'}),
    ('swiped toward -v from the far mouth', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': 'swipe_history("reverse")'}),  # The reader reads in either direction.
    ('track 0.8 mm off the head gap (card riding 0.8 mm above the floor)', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': 'swipe_history("high_0p8")'}),
    ('card tilted 0.4 deg along the track', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': 'swipe_history("tilted_small")'}),
    ('leading edge nudges the head short of the data, then one clean swipe', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': 'swipe_history("nudge")'}),  # Nothing encoded reached the head on the nudge, so the reader saw one swipe.
    ('pushed in and pulled back, then a clean swipe, card home', GOAL_POSES, {
    }, 0.5, {'TEST_HISTORY': 'swipe_history("retry")'}),  # Re-swipe rule, a good read after a failed swipe.
    ('pushed in and pulled back, then a clean swipe, card home but still pinched', GOAL_POSES, {
    }, 0.25, {'TEST_HISTORY': 'swipe_history("retry")', 'TEST_GRIPPERS': '{"left": (np.array([0.10, 0.27, TOP + 0.16]), 1.0), "right": (np.array([XS, YS, STAND_FLOOR_Z + TCP_ABOVE_EDGE]), 0.0)}'}),
    ('a clean swipe, then pushed back in until the head is 40 mm in and left there', GOAL_POSES, {
        'reader': {'pos': (0.316, -0.3072, 0.7475), 'yaw_deg': -90, 'joints': {'head': -0.00076}},
        'card': {'pos': (0.3132, -0.3051, 0.7823), 'yaw_deg': 90, 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0.25, {'TEST_HISTORY': '_chain(swipe_history("clean"), swipe_history("stuck_mid"))'}),  # The unfinished second swipe makes the read late; the card is not back in its stand.
    ('two good swipes, card home', GOAL_POSES, {
    }, 0.5, {'TEST_HISTORY': 'swipe_history("double")'}),  # Re-swipe rule, the card was read twice.
    ("red team: clean one-pass swipe stopped with the head on the card's blank last 4 mm, card left in the slot", GOAL_POSES, {
        'reader': {'pos': (0.316, -0.3072, 0.7475), 'yaw_deg': -90, 'joints': {'head': -0.00076}},
        'card': {'pos': (0.3548, -0.3051, 0.7823), 'yaw_deg': 90, 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0.5, {'TEST_HISTORY': '_chain(_rows(_profile([(0.3, SWIPE_FROM, SWIPE_FROM, "hold"), (0.55, SWIPE_FROM, CARD_L / 2 - 0.004, "linear")]), Q_TURNED))'}),  # One stroke at 0.23 m/s carried all the encoded data (to 78.67 mm) past the head, and the card stopped with the head on its blank trailing 4 mm (81.6 mm). The text says a clean read with the card left in the reader is 0.5. The code treats the unfinished contact run as no read at all, a rule the scoring text never states. [This case needs the proposal's module: TEST_HISTORY: '_chain(_rows(_profile([(0.3, SWIPE_FROM, SWIPE_FROM, "hold"), (0.55, SWIPE_FROM, CARD_L / 2 - 0.004, "linear")]), Q_TURNED))'. Checked: scores 0.0. The control, stopped 8 mm further with the head clear, scores 0.5.]
    ('red team: clean 0.2 m/s swipe that eases to a stop on the blank trailing strip, lifted out, card home', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': '_chain(_rows(_profile([(0.3, -0.0414 - CARD_L / 2, -0.0414 - CARD_L / 2, "hold"), (0.61, -0.0414 - CARD_L / 2, -0.0414 - CARD_L / 2 + 0.122, "linear"), (0.04, -0.0414 - CARD_L / 2 + 0.122, -0.0414 - CARD_L / 2 + 0.1244, "linear")]), Q_TURNED), _rows(_profile([(0.0, -0.0414 - CARD_L / 2 + 0.1244, -0.0414 - CARD_L / 2 + 0.1244, "hold")], lift=0.018), Q_TURNED, in_slot=False)[:1])'}),  # One stroke riding the floor at a steady 0.2 m/s; a sample lands with the head at 78.60 mm, 0.07 mm short of the last data bit, which passes the head 0.35 ms later at 0.2 m/s. The card then eases to a stop (4.4 mm in the next 50 ms) with the head on the blank trailing strip at 83.0 mm, is lifted out and stood back in the stand. The pair check averages 88 mm/s across the stop and calls the stroke too slow, so a clean read scores 0. [This case needs the proposal's module: TEST_HISTORY: '_chain(_rows(_profile([(0.3, -0.0414 - CARD_L / 2, -0.0414 - CARD_L / 2, "hold"), (0.61, -0.0414 - CARD_L / 2, -0.0414 - CARD_L / 2 + 0.122, "linear"), (0.04, -0.0414 - CARD_L / 2 + 0.122, -0.0414 - CARD_L / 2 + 0.1244, "linear")]), Q_TURNED), _rows(_profile([(0.0, -0.0414 - CARD_L / 2 + 0.1244, -0.0414 - CARD_L / 2 + 0.1244, "hold")], lift=0.018), Q_TURNED, in_slot=False)[:1])'. Checked: scores 0.0. The control, with the same motion sampled 0.2 mm later (sample at 78.80 mm), scores 1.0 with success.]
    ('red team: clean 0.2 m/s swipe that eases to a stop on the blank trailing strip, card left in the slot', GOAL_POSES, {
        'reader': {'pos': (0.316, -0.3072, 0.7475), 'yaw_deg': -90, 'joints': {'head': -0.00076}},
        'card': {'pos': (0.3562, -0.3051, 0.7823), 'yaw_deg': 90, 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0.5, {'TEST_HISTORY': '_chain(_rows(_profile([(0.3, -0.0414 - CARD_L / 2, -0.0414 - CARD_L / 2, "hold"), (0.61, -0.0414 - CARD_L / 2, -0.0414 - CARD_L / 2 + 0.122, "linear"), (0.04, -0.0414 - CARD_L / 2 + 0.122, -0.0414 - CARD_L / 2 + 0.1244, "linear")]), Q_TURNED))'}),  # The README promises that a card stopped with the head on its blank last 7 mm after one clean stroke is a clean read left in the reader (0.5). A gentle stop there (4 m/s2 from 0.2 m/s) scores 0 whenever a sample lands in the last 0.07 mm of data, because the stop is averaged into the speed of the pair that straddles 78.67 mm. [This case needs the proposal's module: TEST_HISTORY: '_chain(_rows(_profile([(0.3, -0.0414 - CARD_L / 2, -0.0414 - CARD_L / 2, "hold"), (0.61, -0.0414 - CARD_L / 2, -0.0414 - CARD_L / 2 + 0.122, "linear"), (0.04, -0.0414 - CARD_L / 2 + 0.122, -0.0414 - CARD_L / 2 + 0.1244, "linear")]), Q_TURNED))'. Checked: scores 0.0.]
    ('red team: clean 0.4 m/s swipe, card rises 2.3 mm only after its trailing end has passed the head, card home', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': '_chain(_rows(_profile([(0.3, SWIPE_FROM, SWIPE_FROM, "hold"), (0.35, SWIPE_FROM, SWIPE_FROM + 0.14, "linear")], z_of_y=lambda y: FLOOR_CARD_Z + 0.3 * np.maximum(0.0, y - CARD_L / 2)), Q_TURNED))'}),  # The card rides the floor at 0.4 m/s through all the data (last touched sample at 73.25 mm, on the floor) and starts to rise only once its trailing end is past the head (17 degree exit glide). The next sample has the head 7.65 mm beyond the card's end, so no part of the card is at the head, but the card is 2.3 mm up. The after-sample height check fails the read, and a clean swipe returned to the stand scores 0. [This case needs the proposal's module: TEST_HISTORY: '_chain(_rows(_profile([(0.3, SWIPE_FROM, SWIPE_FROM, "hold"), (0.35, SWIPE_FROM, SWIPE_FROM + 0.14, "linear")], z_of_y=lambda y: FLOOR_CARD_Z + 0.3 * np.maximum(0.0, y - CARD_L / 2)), Q_TURNED))'. Checked: scores 0.0. The control, with the identical path shifted 4.65 mm in sampling phase (after sample 3 mm past the card's end, 0.9 mm up), scores 1.0 with success.]
    ('red team: clean 0.4 m/s swipe glided down into the slot, on the floor before the head reaches the card, card home', GOAL_POSES, {
    }, 1, {'TEST_HISTORY': '_chain(_rows(_profile([(0.3, -0.05 - CARD_L / 2, -0.05 - CARD_L / 2, "hold"), (0.35, -0.05 - CARD_L / 2, 0.09 - CARD_L / 2, "linear")], z_of_y=lambda y: FLOOR_CARD_Z + 0.3 * np.maximum(0.0, -(y + CARD_L / 2))), Q_TURNED))'}),  # The card glides down along the slot (17 degrees), and its edge reaches the floor while its leading end is still short of the head; every touched sample rides the floor at 0.4 m/s through all the data. The untouched sample before has the head 10 mm short of the card's leading end with the card 3 mm up. The before-sample height check fails the read, so a clean swipe returned to the stand scores 0. [This case needs the proposal's module: TEST_HISTORY: '_chain(_rows(_profile([(0.3, -0.05 - CARD_L / 2, -0.05 - CARD_L / 2, "hold"), (0.35, -0.05 - CARD_L / 2, 0.09 - CARD_L / 2, "linear")], z_of_y=lambda y: FLOOR_CARD_Z + 0.3 * np.maximum(0.0, -(y + CARD_L / 2))), Q_TURNED))'. Checked: scores 0.0. The control, with the identical path shifted 7 mm in phase (before sample 3 mm short of the card, 0.9 mm up), scores 1.0 with success.]
    ('red team: clean swipe, card still pinched and held flat 19 mm above the counter', GOAL_POSES, {
        'card': {'pos': (0.2, -0.25, 0.7665)},
    }, 0.5, {'TEST_HISTORY': 'swipe_history("clean")', 'TEST_GRIPPERS': '{"left": (np.array([0.10, 0.27, TOP + 0.16]), 1.0), "right": (np.array([0.20, -0.215, TOP + 0.019]), 0.0)}'}),  # The card is pinched in the closed right gripper, not lying anywhere. The text calls a centre within 20 mm of the counter 'lying down', yet the low-centre rule ignores the grip and makes it a sticky drop. [This case needs the proposal's module: TEST_HISTORY: swipe_history("clean"); TEST_GRIPPERS: '{"left": (np.array([0.10, 0.27, TOP + 0.16]), 1.0), "right": (np.array([0.20, -0.215, TOP + 0.019]), 0.0)}'. Checked: scores 0.0.]
    ('clean swipe, then the card left lying flat on the counter', GOAL_POSES, {
        'card': {'pos': (0.15, -0.25, 0.7479)},
    }, 0.5, {'TEST_HISTORY': 'swipe_history("clean")'}),  # Not back in its stand. A card lying on the counter is where the task starts, not a drop.
    ('clean swipe, then the card lying flat on top of the reader', GOAL_POSES, {
        'card': {'pos': (0.316, -0.3072, 0.7783)},
    }, 0.5, {'TEST_HISTORY': 'swipe_history("clean")'}),  # Not back in its stand; lying on the reader is not a drop.
    ("card slid until its end overhangs the counter's edge, never swiped", START_POSES, {
        'card': {'pos': (0.165, -0.6432, 0.7479)},
    }, 0, {}),  # No read.
    ("clean swipe, card held by its end past the counter's edge, below the counter's surface", GOAL_POSES, {
        'card': {'pos': (0.165, -0.7, 0.7175)},
    }, 0.5, {'TEST_HISTORY': 'swipe_history("clean")', 'TEST_GRIPPERS': '{"left": (np.array([0.10, 0.27, TOP + 0.16]), 1.0), "right": (np.array([CARD_X0, EDGE_Y - 0.090, TOP - 0.030]), 0.0)}'}),  # Still in hand, so not dropped; not back in its stand.
    ("clean swipe, then the card let go past the counter's edge, falling", GOAL_POSES, {
        'card': {'pos': (0.165, -0.7, 0.7175)},
    }, 0, {'TEST_HISTORY': 'swipe_history("clean")'}),  # Let go past the edge with its centre 30 mm below the counter's surface: dropped.
    ('card back in the stand without any swipe', GOAL_POSES, {
    }, 0, {}),
    ('swiped with the stripe facing away from the head', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'swipe_history("wrong_side")'}),
    ('swiped stripe edge up', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'swipe_history("upside_down")'}),
    ('card riding 1.5 mm above the slot floor', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'swipe_history("high_1p5")'}),  # Track 2 misses the 1.5 mm read core.
    ('card tilted 3 deg along the track', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'swipe_history("tilted")'}),
    ('swiped at 0.12 m/s', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'swipe_history("slow")'}),  # Under the rated 6 in/s.
    ('swiped at 1.70 m/s', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'swipe_history("fast")'}),  # Over the rated 60 in/s.
    ('stopped 0.4 s with the head on the stripe, then finished the pass', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'swipe_history("stop")'}),
    ('pushed until the head is 40 mm in, then pulled back out', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'swipe_history("back")'}),
    ('card lifted out over the head mid-stripe while moving on, card home', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'swipe_history("lift_out")'}),  # The sample after the touched run has the card 30 mm off the floor while the data is still passing the head.
    ('card dropped onto the head mid-stripe from above, then slid on, card home', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'swipe_history("drop_in")'}),  # The sample before the touched run has the card 30 mm off the floor while the data is already passing the head.
    ('pushed until the head is 40 mm in and left there, still touching the head at the end', GOAL_POSES, {
        'reader': {'pos': (0.316, -0.3072, 0.7475), 'yaw_deg': -90, 'joints': {'head': -0.00076}},
        'card': {'pos': (0.3132, -0.3051, 0.7823), 'yaw_deg': 90, 'tilt_deg': 90, 'tilt_axis': 'y'},
    }, 0, {'TEST_HISTORY': 'swipe_history("stuck_mid")'}),  # An unfinished swipe that has not carried all the data past the head is a failed swipe; alone it is no read.
    ('the swipe motion 30 mm in front of the reader, nothing touches the head', GOAL_POSES, {
    }, 0, {'TEST_HISTORY': 'swipe_history("beside")'}),
    ('clean swipe, then the card on the floor', GOAL_POSES, {
        'card': {'pos': (0.3, -0.8, 0.01)},
    }, 0, {'TEST_HISTORY': 'swipe_history("clean")'}),  # Fell off the counter and let go: dropped.
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
