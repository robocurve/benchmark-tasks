"""Offline checks for the sizing-board grading task (numpy + pytest only, no Isaac Sim).

Run with the yam-isaac-lab-arena clone (commit bd62771) importable, e.g.
    YAM_ARENA_PATH=~/yam-isaac-lab-arena pytest -q test_task.py
"""

import itertools
import os
import sys
from pathlib import Path

import numpy as np
import pytest

if os.environ.get("YAM_ARENA_PATH"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_PATH"]))

from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import Box, ObjectState, Sphere, load  # noqa: E402

TASK = load(Path(__file__).with_name("task.py"))
M = TASK.module
HZ = 20  # the run limit and the commands use control_hz=20
RUN_LIMIT_STEPS = 3600
PAD_HALF_WIDTH = 0.020  # pad extent across the closing direction (tip meshes)
PAD_OUTER = 0.057  # outer face of an open pad from the TCP along the closing direction
SMALL = [n for n in M.FRUIT_NAMES if M.GRADE[n] == "small"]
LARGE = [n for n in M.FRUIT_NAMES if M.GRADE[n] == "large"]


def state(pos, r, vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, dtype=float)
    q = np.array([0.0, 0.0, 0.0, 1.0])
    return ObjectState(pos, q, np.asarray(vel, dtype=float), np.zeros(3), pos - r, pos + r)


def fixed_states():
    out = {}
    for o in M.OBJECTS:
        if not o.free:
            p, h = np.asarray(o.pos), np.asarray(o.size) / 2
            out[o.name] = ObjectState(p, np.array([0, 0, 0, 1.0]), np.zeros(3), np.zeros(3), p - h, p + h)
    return out


def in_small_bin(i):
    """A spot on the small-grade bin's floor, spread so fruit do not overlap."""
    spots = [(-0.045, -0.045), (0.045, -0.045), (0.0, 0.045)]
    dx, dy = spots[i]
    return (M.SB_X + dx, M.SB_Y + dy, M.SB_FLOOR_TOP + M.R_SMALL)


def in_large_bin(i, r=M.R_LARGE):
    dx = (-0.04, 0.04, 0.0)[i]
    dy = (0.0, 0.0, 0.045)[i]
    return (M.LB_X + dx, M.LB_Y + dy, M.LB_FLOOR_TOP + r)


def goal_state():
    objects = fixed_states()
    for i, n in enumerate(SMALL):
        objects[n] = state(in_small_bin(i), M.RADIUS[n])
    for i, n in enumerate(LARGE):
        objects[n] = state(in_large_bin(i), M.RADIUS[n])
    return objects


def start_state():
    objects = fixed_states()
    for o in M.FRUIT:
        objects[o.name] = state((o.pos[0], o.pos[1], o.pos[2] - M.SPAWN_GAP), o.radius)
    return objects


def aabb(o):
    p = np.asarray(o.pos, dtype=float)
    h = np.asarray(o.size) / 2 if isinstance(o, Box) else np.full(3, o.radius)
    return p - h, p + h


# -- 1. the file loads ------------------------------------------------------------------------------------


def test_the_task_file_loads_with_its_hooks_and_docs():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.free_objects == M.FRUIT_NAMES
    assert TASK.docs.startswith("On the station table")
    assert all(isinstance(o, (Box, Sphere)) for o in TASK.objects)  # primitives only, no library assets
    assert len(SMALL) == 3 and len(LARGE) == 2


# -- 2. the demo is reachable -----------------------------------------------------------------------------


@pytest.mark.parametrize("side", ["left", "right"])
def test_the_demo_is_reachable_within_the_joint_limits(side):
    plan = TASK.demo()
    assert set(plan) == {"left", "right"}
    arm = ArmPlan(plan[side], station.BASES[side])
    steps = round(arm.duration * HZ) + 5
    actions = np.array([arm.action7((k + 1) / HZ)[0] for k in range(steps)])
    assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1
    assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)


def test_the_demo_fits_the_run_limit_and_both_arms_end_together():
    plan = TASK.demo()
    durations = [sum(w.duration for w in plan[s]) for s in ("left", "right")]
    assert durations[0] == pytest.approx(durations[1])
    assert durations[0] * HZ + HZ * TASK.success_hold_s < RUN_LIMIT_STEPS / 2


def sample(plan, side, dt=0.02):
    arm = ArmPlan(plan[side], station.BASES[side])
    ts = np.arange(0.0, arm.duration + dt, dt)
    return ts, [arm.target(t) for t in ts]


def test_the_arms_stay_apart_and_the_pads_never_dig_into_the_board():
    plan = TASK.demo()
    _, left = sample(plan, "left")
    _, right = sample(plan, "right")
    gap = min(np.linalg.norm(a[0] - b[0]) for a, b in zip(left, right, strict=True))
    assert gap > 0.18
    half = M.SB_OUT / 2 + PAD_OUTER
    for samples in (left, right):
        for pos, _, _, _ in samples:
            over_board = abs(pos[0] - M.SB_X) < M.SB_OUT / 2 + PAD_HALF_WIDTH and abs(pos[1] - M.SB_Y) < half
            if over_board:
                assert pos[2] - PAD_TIP_PAST_TCP > M.BOARD_TOP + 0.004


def test_every_fruit_is_let_go_over_the_centre_of_the_opening_high_enough_to_clear_it():
    left = TASK.demo()["left"]
    releases = [w for w in left if w.label == "let go over the opening"]
    assert len(releases) == 5
    for w in releases:
        assert w.finger == FINGER_OPEN
        assert np.hypot(w.pos[0] - M.SB_X, w.pos[1] - M.SB_Y) < 1e-9
        # the grip sits GRIP_ABOVE_CENTRE above the fruit's centre; its bottom must clear the board top
        assert w.pos[2] - M.GRIP_ABOVE_CENTRE - M.R_LARGE > M.BOARD_TOP + 0.005


# -- 3. goal state and partial credit ---------------------------------------------------------------------


def test_the_goal_state_succeeds_with_full_score_and_zero_distance():
    goal = goal_state()
    assert TASK.success(goal) and TASK.score(goal) == 1.0 and TASK.distance(goal) == 0.0


def test_the_start_state_scores_zero():
    s = start_state()
    assert not TASK.success(s) and TASK.score(s) == 0.0 and TASK.distance(s) > 0.5


@pytest.mark.parametrize("k", range(6))
def test_each_fruit_in_its_correct_bin_is_worth_0_2(k):
    objects = start_state()
    placed = goal_state()
    for n in M.FRUIT_NAMES[:k]:
        objects[n] = placed[n]
    assert TASK.score(objects) == pytest.approx(0.2 * k)
    assert TASK.success(objects) == (k == 5)


def test_a_fruit_still_moving_in_its_bin_earns_nothing_yet():
    objects = goal_state()
    n = SMALL[0]
    objects[n] = state(objects[n].pos, M.RADIUS[n], vel=(0.05, 0.0, 0.0))
    assert TASK.score(objects) == pytest.approx(0.8) and not TASK.success(objects)


def test_a_small_fruit_in_the_large_bin_costs_0_2():
    objects = goal_state()
    objects[SMALL[0]] = state(in_large_bin(2, M.R_SMALL), M.R_SMALL)
    # 4 correct, 1 wrong
    assert TASK.score(objects) == pytest.approx(0.6) and not TASK.success(objects)


@pytest.mark.parametrize("where", ["floor", "falling_off_the_edge"])
def test_a_fruit_off_the_table_scores_zero_and_caps_the_total_at_0_6(where):
    objects = goal_state()
    n = LARGE[0]
    pos = (0.3, 0.75, M.R_LARGE) if where == "floor" else (0.62, 0.0, M.TOP - 0.03)
    objects[n] = state(pos, M.R_LARGE, vel=(0, 0, 0) if where == "floor" else (0, 0, -1.0))
    assert TASK.score(objects) == pytest.approx(0.6) and not TASK.success(objects)
    for m in M.FRUIT_NAMES:  # every fruit dropped
        objects[m] = state((0.3, 0.75, M.R_LARGE), M.RADIUS[m])
    assert TASK.score(objects) == 0.0


def test_a_fruit_off_the_table_caps_but_does_not_raise_a_low_score():
    objects = start_state()
    objects[SMALL[0]] = goal_state()[SMALL[0]]
    objects[LARGE[0]] = state((0.3, 0.75, M.R_LARGE), M.R_LARGE)
    assert TASK.score(objects) == pytest.approx(0.2)


def test_all_scores_lie_in_0_1():
    goal, start = goal_state(), start_state()
    for mask in itertools.product((0, 1, 2, 3), repeat=5):
        objects = dict(start)
        for n, m in zip(M.FRUIT_NAMES, mask, strict=True):
            r = M.RADIUS[n]
            if m == 1:
                objects[n] = goal[n]
            elif m == 2:
                objects[n] = state((0.3, 0.75, r), r)
            elif m == 3 and M.GRADE[n] == "small":
                objects[n] = state(in_large_bin(2, r), r)
        s = TASK.score(objects)
        assert 0.0 <= s <= 1.0 and round(s / 0.2, 6).is_integer()


# -- 4. construct validity --------------------------------------------------------------------------------


def test_putting_every_fruit_in_the_large_bin_without_testing_scores_zero():
    objects = fixed_states()
    for i, n in enumerate(M.FRUIT_NAMES):
        x = M.LB_X + (-0.045, 0.0, 0.045, -0.045, 0.045)[i]
        y = M.LB_Y + (-0.045, 0.0, -0.045, 0.045, 0.045)[i]
        objects[n] = state((x, y, M.LB_FLOOR_TOP + M.RADIUS[n]), M.RADIUS[n])
    assert TASK.score(objects) == 0.0 and not TASK.success(objects)


def test_a_large_fruit_left_sitting_in_the_opening_earns_nothing():
    objects = goal_state()
    n = LARGE[0]
    objects[n] = state((M.SB_X, M.SB_Y, M.SEAT_Z), M.R_LARGE)
    assert TASK.score(objects) == pytest.approx(0.8) and not TASK.success(objects)
    objects[n] = state((M.SB_X - 0.06, M.SB_Y, M.BOARD_TOP + M.R_LARGE), M.R_LARGE)  # resting on the board
    assert TASK.score(objects) == pytest.approx(0.8)


def test_a_fruit_held_above_the_large_bin_or_on_its_rim_earns_nothing():
    objects = goal_state()
    n = LARGE[0]
    objects[n] = state((M.LB_X, M.LB_Y, M.LB_RIM + 0.03), M.R_LARGE)
    assert TASK.score(objects) == pytest.approx(0.8)
    objects[n] = state((M.LB_X - M.LB_IN / 2 - 0.005, M.LB_Y, M.LB_RIM + M.R_LARGE), M.R_LARGE)
    assert TASK.score(objects) == pytest.approx(0.8)


def test_a_large_fruit_cannot_pass_the_opening_but_a_small_one_can():
    assert 2 * M.R_LARGE > M.OPENING + 0.005  # 6 mm too big
    assert 2 * M.R_SMALL < M.OPENING - 0.005  # 6 mm to spare
    # the opening as left by the board's slabs
    near, far = (aabb(next(o for o in M.OBJECTS if o.name == f"board_{s}")) for s in ("near", "far"))
    left, right = (aabb(next(o for o in M.OBJECTS if o.name == f"board_{s}")) for s in ("left", "right"))
    assert far[0][0] - near[1][0] == pytest.approx(M.OPENING)
    assert left[0][1] - right[1][1] == pytest.approx(M.OPENING)


def test_the_board_closes_the_small_bin_so_the_opening_is_the_only_way_in():
    board = [aabb(o) for o in M.OBJECTS if o.name.startswith("board_")]
    walls = [aabb(o) for o in M.OBJECTS if o.name.startswith("small_bin_wall")]
    area = sum((hi[0] - lo[0]) * (hi[1] - lo[1]) for lo, hi in board)
    assert area + M.OPENING**2 == pytest.approx(M.SB_OUT**2)  # slabs + opening tile the bin's footprint
    for lo, hi in board:  # slabs sit flush on the walls and do not extend past the bin
        assert lo[2] == pytest.approx(M.BOARD_BOTTOM)
        assert lo[0] >= M.SB_X - M.SB_OUT / 2 - 1e-9 and hi[0] <= M.SB_X + M.SB_OUT / 2 + 1e-9
        assert lo[1] >= M.SB_Y - M.SB_OUT / 2 - 1e-9 and hi[1] <= M.SB_Y + M.SB_OUT / 2 + 1e-9
    for lo, hi in walls:
        assert hi[2] == pytest.approx(M.BOARD_BOTTOM) and lo[2] == pytest.approx(M.SB_FLOOR_TOP)
    # walls enclose the interior on all four sides, corner to corner
    xs = sorted({round(v, 6) for lo, hi in walls for v in (lo[0], hi[0])})
    ys = sorted({round(v, 6) for lo, hi in walls for v in (lo[1], hi[1])})
    assert xs[0] == pytest.approx(M.SB_X - M.SB_OUT / 2) and xs[-1] == pytest.approx(M.SB_X + M.SB_OUT / 2)
    assert ys[0] == pytest.approx(M.SB_Y - M.SB_OUT / 2) and ys[-1] == pytest.approx(M.SB_Y + M.SB_OUT / 2)


def test_two_small_fruit_stacked_under_the_opening_clear_the_board():
    # a third small fruit landing on two others never jams in the opening above them
    assert M.BOARD_BOTTOM - M.SB_FLOOR_TOP > 4 * M.R_SMALL + 0.005


def test_fruit_colour_and_names_do_not_give_the_grade_away():
    assert len({o.color for o in M.FRUIT}) == 1
    assert all("small" not in n and "large" not in n for n in M.FRUIT_NAMES)


# -- 5. start layout --------------------------------------------------------------------------------------


def test_no_two_objects_overlap_at_the_start():
    for a, b in itertools.combinations(M.OBJECTS, 2):
        (alo, ahi), (blo, bhi) = aabb(a), aabb(b)
        overlap = np.minimum(ahi, bhi) - np.maximum(alo, blo)
        assert overlap.min() <= 1e-9, (a.name, b.name)


def test_every_fruit_rests_on_the_tray_floor_inside_its_rim():
    lx, ly = M.TRAY_IN
    for o in M.FRUIT:
        assert o.pos[2] - o.radius - M.TRAY_FLOOR_TOP == pytest.approx(M.SPAWN_GAP)
        assert abs(o.pos[0] - M.TRAY_X) + o.radius < lx / 2
        # an open pad (outer face 57 mm off the grip) stays clear of the side rims
        assert abs(o.pos[1] - M.TRAY_Y) + PAD_OUTER < ly / 2


def test_neighbouring_fruit_leave_room_for_the_pads():
    for a, b in zip(M.FRUIT, M.FRUIT[1:]):
        gap = abs(b.pos[0] - a.pos[0])
        assert gap - max(a.radius, b.radius) > PAD_HALF_WIDTH + 0.01


def test_every_fruit_fits_the_gripper_with_a_centimetre_to_spare():
    assert M.GRIPPER_MAX_OPENING == pytest.approx(-2 * FINGER_OPEN, abs=1e-3)  # 94 mm pad gap
    for o in M.FRUIT:
        assert 2 * o.radius <= M.GRIPPER_MAX_OPENING - 0.010


def test_the_grip_heights_keep_the_pad_tips_off_the_floor_and_the_board():
    for o in M.FRUIT:
        tips = M.TRAY_FLOOR_TOP + o.radius + M.GRIP_ABOVE_CENTRE - PAD_TIP_PAST_TCP
        assert tips > M.TRAY_FLOOR_TOP + 0.005
        assert M.GRIP_ABOVE_CENTRE < PAD_TIP_PAST_TCP - 0.005  # pads still reach below the equator
    tips_at_seat = M.SEAT_Z + M.GRIP_ABOVE_CENTRE - PAD_TIP_PAST_TCP
    assert tips_at_seat > M.BOARD_TOP + 0.004
    assert M.LB_RELEASE_Z - PAD_TIP_PAST_TCP > M.LB_FLOOR_TOP + 0.01
    for x, y in M.LB_SPOTS:  # open pads stay inside the large bin's walls
        assert abs(y - M.LB_Y) + PAD_OUTER < M.LB_IN / 2
        assert abs(x - M.LB_X) + M.R_LARGE < M.LB_IN / 2
