"""Offline checks of the task (no GPU, no Isaac Sim): the scorer on the start, the goal and named layouts, and
the scripted demo's reach and length.

    pytest pay-blackjack-beside-the-bet    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("pay_blackjack_beside_the_bet.py"))
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
    'bet_low': {'pos': (0.3, 0.14, 0.7648)},
    'bet_high': {'pos': (0.3, 0.14, 0.7951)},
    'bust': {'pos': (0.3, -0.14, 0.7648)},
    'bank_1': {'pos': (0.16, -0.2, 0.7628)},
    'bank_2': {'pos': (0.16, -0.262, 0.7628)},
    'bank_3': {'pos': (0.16, -0.324, 0.7628)},
}
GOAL_POSES = {
    'bet_low': {'pos': (0.3, 0.14, 0.7648)},
    'bet_high': {'pos': (0.3, 0.14, 0.7951)},
    'bust': {'pos': (0.16, -0.386, 0.7625)},
    'bank_1': {'pos': (0.3, 0.09, 0.7645)},
    'bank_2': {'pos': (0.3, 0.09, 0.7945)},
    'bank_3': {'pos': (0.3, 0.09, 0.8245)},
}
START, GOAL = layout(START_POSES), layout(GOAL_POSES)


def test_the_task_loads_with_its_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {'bank_1', 'bet_low', 'bank_2', 'bust', 'bet_high', 'bank_3'}
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
    ("paid and untouched, but B's chip left on circle B", GOAL_POSES, {
        'bust': {'pos': (0.3, -0.14, 0.7645)},
    }, 0.75),
    ('even money (2-chip payout), the third bank chip still in its well', GOAL_POSES, {
        'bank_3': {'pos': (0.16, -0.324, 0.7625)},
    }, 0.4),
    ("overpaid (4-chip payout, B's chip on top of it, rack empty)", GOAL_POSES, {
        'bust': {'pos': (0.3, 0.09, 0.8545)},
    }, 0.15),
    ("the bet's top chip nudged 4 mm on its base, payout and collect done", GOAL_POSES, {
        'bet_high': {'pos': (0.304, 0.14, 0.7945)},
    }, 1),
    ("one payout chip only, B's chip in well 1, the rest in the rack", START_POSES, {
        'bust': {'pos': (0.16, -0.2, 0.7625)},
        'bank_1': {'pos': (0.3, 0.09, 0.7645)},
    }, 0.4),
    ('the bet shoved 20 mm forward, payout and collect done', GOAL_POSES, {
        'bet_low': {'pos': (0.32, 0.14, 0.7645)},
        'bet_high': {'pos': (0.32, 0.14, 0.7945)},
    }, 0.85),
    ('the payout built in front of the bet instead of beside it toward the midline', GOAL_POSES, {
        'bank_1': {'pos': (0.35, 0.14, 0.7645)},
        'bank_2': {'pos': (0.35, 0.14, 0.7945)},
        'bank_3': {'pos': (0.35, 0.14, 0.8245)},
    }, 1),
    ("B's chip used in the payout and a bank chip in the rack (chips are identical)", GOAL_POSES, {
        'bust': {'pos': (0.3, 0.09, 0.8245)},
        'bank_3': {'pos': (0.16, -0.386, 0.7625)},
    }, 1),
    ("red team: B's chip racked in the empty well, nothing else done", START_POSES, {
        'bust': {'pos': (0.16, -0.386, 0.7625)},
    }, 0.25),  # collecting the busted bet into the rack is a completed subtask worth 0.25 by the scoring table, independent of the payout
    ("red team: the bet's top chip nudged 9 mm on its base, payout and collect done", GOAL_POSES, {
        'bet_high': {'pos': (0.309, 0.14, 0.7945)},
    }, 0.85),  # the bet is still a 2-chip stack (the top chip overlaps its base by 31 mm) and both chips are within the 10 mm 'untouched' tolerance; it deserves at least the 0.85 a 20 mm shove of the whole bet earns, not 0
    ("red team: payout touching both bet chips, the bet's chips spread 7 mm across the payout direction", GOAL_POSES, {
        'bet_low': {'pos': (0.2965, 0.14, 0.7645)},
        'bet_high': {'pos': (0.3035, 0.14, 0.7945)},
        'bank_1': {'pos': (0.3, 0.1001, 0.7645)},
        'bank_2': {'pos': (0.3, 0.1001, 0.7945)},
        'bank_3': {'pos': (0.3, 0.1001, 0.8245)},
    }, 1),  # the payout touches the bet with a 0 mm gap (allowed) and both bet chips are within 10 mm of A's centre; the mean of the bet's axes sits 39.9 mm from the payout only because the bet's chips are a little ragged
    ('one chip capped on the bet, the payout beside it', GOAL_POSES, {
        'bust': {'pos': (0.3, 0.14, 0.8245)},
    }, 0),  # a capped bet hides what the original wager was
    ('the whole payout capped on the bet (one 5-chip tower)', START_POSES, {
        'bust': {'pos': (0.16, -0.386, 0.7625)},
        'bank_1': {'pos': (0.3, 0.14, 0.8245)},
        'bank_2': {'pos': (0.3, 0.14, 0.8545)},
        'bank_3': {'pos': (0.3, 0.14, 0.8845)},
    }, 0),  # capping is the error the task is about
    ('a payout chip bridging the bet and the payout stack', GOAL_POSES, {
        'bust': {'pos': (0.3, 0.115, 0.8245)},
    }, 0),  # a chip resting on the bet caps it
    ("B's chip knocked off the table, nothing else done", START_POSES, {
        'bust': {'pos': (0.3, -0.8, 0.015)},
    }, 0),  # no payout stands beside the bet
    ("B's chip off the table, the payout done", GOAL_POSES, {
        'bust': {'pos': (0.3, -0.8, 0.015)},
    }, 0.75),  # B's bet was not collected into the rack
    ('the last payout chip still held 10 mm above its place', GOAL_POSES, {
        'bank_3': {'pos': (0.3, 0.09, 0.8345)},
    }, 0.15),  # a held chip is not paid, and is a loose chip, so the collect is lost too
    ('the last payout chip held 2 mm above its place', GOAL_POSES, {
        'bank_3': {'pos': (0.3, 0.09, 0.8265)},
    }, 0.15),  # a held chip is not paid, and is a loose chip, so the collect is lost too
    ('the last payout chip still moving at 0.2 m/s', GOAL_POSES, {
        'bank_3': {'pos': (0.3, 0.09, 0.8245), 'vel': (0.2, 0.0, 0.0)},
    }, 0.15),  # a moving chip is not paid, and is a loose chip, so the collect is lost too
    ('the last payout chip lying on its side on the stack', GOAL_POSES, {
        'bank_3': {'pos': (0.3, 0.09, 0.8295), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.15),  # a toppled chip is not a stack level, and is a loose chip, so the collect is lost too
    ("the bet's top chip knocked over beside it", GOAL_POSES, {
        'bet_high': {'pos': (0.255, 0.14, 0.7695), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0),  # the player's bet is broken (no 2-chip stack within 30 mm of A's centre), which zeroes the score
    ("the bet's top chip slid 16 mm off its base's axis, payout and collect done", GOAL_POSES, {
        'bet_high': {'pos': (0.316, 0.14, 0.7945)},
    }, 0),  # a top chip more than 15 mm off its base's axis no longer counts as stacked, so the bet is broken
    ('the bet taken into the rack, the payout beside its empty spot', GOAL_POSES, {
        'bet_low': {'pos': (0.16, -0.262, 0.7625)},
        'bet_high': {'pos': (0.16, -0.324, 0.7625)},
    }, 0),  # the winning player's stake was taken into the rack, which zeroes the score
    ('the payout 70 mm from the bet (a 30 mm gap)', GOAL_POSES, {
        'bank_1': {'pos': (0.3, 0.07, 0.7645)},
        'bank_2': {'pos': (0.3, 0.07, 0.7945)},
        'bank_3': {'pos': (0.3, 0.07, 0.8245)},
    }, 0.25),  # not beside the bet, so no payout and no untouched credit; B is racked and the three chips form one stack on circle A, so the collect's 0.25 stands
    ('every non-bet chip stood flat in its own stack on circle A, rack empty', START_POSES, {
        'bust': {'pos': (0.3, 0.09, 0.7645)},
        'bank_1': {'pos': (0.35, 0.14, 0.7645)},
        'bank_2': {'pos': (0.25, 0.14, 0.7645)},
        'bank_3': {'pos': (0.3, 0.19, 0.7645)},
    }, 0.15),  # four single chips are not one stack and nothing went into the rack; only the untouched bet counts
    ('the bet shoved 35 mm, payout and collect done', GOAL_POSES, {
        'bet_low': {'pos': (0.335, 0.14, 0.7645)},
        'bet_high': {'pos': (0.335, 0.14, 0.7945)},
    }, 0),  # the bet is more than 30 mm from its marked centre, so it no longer counts as the player's bet
    ('the payout split into a 2-chip and a 1-chip stack', GOAL_POSES, {
        'bank_3': {'pos': (0.35, 0.14, 0.7645)},
    }, 0.15),  # the payout must be one stack of 3; the chips outside the bet and the rack form two stacks, so the collect is lost too
    ('the last payout chip left on the handover square at time-out', GOAL_POSES, {
        'bank_3': {'pos': (0.22, 0, 0.7645)},
    }, 0.15),  # staging earns nothing; a chip on the square is neither paid nor racked
    ("B's chip left on the handover square", GOAL_POSES, {
        'bust': {'pos': (0.22, 0, 0.7645)},
    }, 0.75),  # B's bet is not in the rack
    ("B's chip stacked on a bank chip in a well (even-money payout)", GOAL_POSES, {
        'bust': {'pos': (0.16, -0.324, 0.7925)},
        'bank_3': {'pos': (0.16, -0.324, 0.7625)},
    }, 0.15),  # a chip on another chip is not seated in a well
    ("B's chip resting on the rack's far wall", GOAL_POSES, {
        'bust': {'pos': (0.191, -0.386, 0.7705)},
    }, 0.75),  # a chip on a wall is not seated in a well
    ("B's chip pushed to the edge of circle B", GOAL_POSES, {
        'bust': {'pos': (0.3, -0.05, 0.7645)},
    }, 0.75),  # still touching circle B, not collected
    ("red team: the bet's top chip slid beside the bet and B racked (from start)", START_POSES, {
        'bet_high': {'pos': (0.3, 0.09, 0.7645)},
        'bust': {'pos': (0.16, -0.386, 0.7625)},
    }, 0),  # the player's own stake was split, nothing paid; breaking the bet should not unlock the collect credit that racking B alone does not earn
    ('red team: every non-bet chip dumped on circle A, one flat beside the bet, rack empty', START_POSES, {
        'bust': {'pos': (0.3, 0.09, 0.7645)},
        'bank_1': {'pos': (0.36, 0.17, 0.7695), 'tilt_deg': 90, 'tilt_axis': 'y'},
        'bank_2': {'pos': (0.24, 0.17, 0.7695), 'tilt_deg': 90, 'tilt_axis': 'y'},
        'bank_3': {'pos': (0.3, 0.21, 0.7695), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.15),  # the house bank was dumped on the winner's circle and nothing went into the rack; only the untouched bet deserves credit
    ('red team: the bet shoved 28 mm away from the payout, a 48 mm gap between them', GOAL_POSES, {
        'bet_low': {'pos': (0.3, 0.168, 0.7645)},
        'bet_high': {'pos': (0.3, 0.168, 0.7945)},
    }, 0.25),  # the instruction says the payout must be at most 2 cm from the bet; here it is 48 mm away and the bet was moved, so only the collect counts
    ("red team: a chip sliding onto the bet's top at time-out, payout done", GOAL_POSES, {
        'bust': {'pos': (0.3, 0.14, 0.8245), 'vel': (0.03, 0.0, 0.0)},
    }, 0),  # a chip on top of the bet is capping whether or not it has stopped sliding
    ("red team: the third payout chip dropped loose on circle A's far side", GOAL_POSES, {
        'bank_3': {'pos': (0.3, 0.22, 0.7695), 'tilt_deg': 90, 'tilt_axis': 'x'},
    }, 0.15),  # same mistake as the author's 'last payout chip left on the handover square' (0.15); a loose chip should lose collect wherever it lies
    ('red team: every non-bet chip stacked in one 4-chip tower on circle A, 70 mm from the bet, rack empty', START_POSES, {
        'bust': {'pos': (0.3, 0.07, 0.7645)},
        'bank_1': {'pos': (0.3, 0.07, 0.7945)},
        'bank_2': {'pos': (0.3, 0.07, 0.8245)},
        'bank_3': {'pos': (0.3, 0.07, 0.8545)},
    }, 0),  # the rack is empty, so B's chip is on the winner's circle, not in the rack, whichever chip it is; nothing was paid beside the bet and nothing went into the rack, so the 'B collected' credit isn't earned
    ('red team: the 4-chip overpay beside the bet, rack empty', GOAL_POSES, {
        'bust': {'pos': (0.3, 0.09, 0.8545)},
    }, 0.15),  # with 4 chips on circle A and none in the rack, B's chip cannot be in the rack, so only the untouched bet counts
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
