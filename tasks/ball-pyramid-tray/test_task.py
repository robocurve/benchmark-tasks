"""Offline checks for the ball-pyramid tray task (numpy + pytest only, no Isaac Sim).

Run with the yam-isaac-lab-arena clone (commit bd62771) importable, e.g.
    YAM_ARENA_PATH=~/yam-isaac-lab-arena pytest -q test_task.py
"""

import itertools
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

if os.environ.get("YAM_ARENA_PATH"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_PATH"]))

from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import Box, ObjectState, Sphere, load  # noqa: E402

TASK = load(Path(__file__).with_name("task.py"))
M = TASK.module
HZ = 20
RUN_LIMIT_STEPS = 3600
PAD_HALF_WIDTH = 0.020  # pad extent across the closing direction
PAD_T = 0.010  # pad thickness along the closing direction
PAD_LEN = 0.045  # pad height above its tip that we model
OPEN_GAP_HALF = -FINGER_OPEN  # 46.95 mm from the TCP to an open pad's inner face
GRIP_SLOP = M.MAX_GRIP_OFFSET + M.TRACK_ERR  # how far a held ball can be from the commanded TCP along the closing axis
R = M.R


def state(pos, vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, dtype=float)
    return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - R, pos + R)


def fixed_states():
    out = {}
    for o in M.OBJECTS:
        if not o.free:
            p, h = np.asarray(o.pos), np.asarray(o.size) / 2
            out[o.name] = ObjectState(p, np.array([0, 0, 0, 1.0]), np.zeros(3), np.zeros(3), p - h, p + h)
    return out


def cornered(corner):
    sx, sy = M.CORNERS[corner]
    return np.array([M.TRAY_X + sx * M.CORNER, M.TRAY_Y + sy * M.CORNER, M.BOTTOM_Z])


SEAT = np.array([M.TRAY_X, M.TRAY_Y, M.BOTTOM_Z + M.TOP_RISE_CORNERS])
CORNER_NAMES = tuple(M.CORNERS)


def goal_state(top="ball_c"):
    objects = fixed_states()
    others = [n for n in M.BALL_NAMES if n != top]
    for n, c in zip(others, CORNER_NAMES, strict=True):
        objects[n] = state(cornered(c))
    objects[top] = state(SEAT)
    return objects


def start_state():
    objects = fixed_states()
    for o in M.BALLS:
        objects[o.name] = state((o.pos[0], o.pos[1], o.pos[2] - M.SPAWN_GAP))
    return objects


def aabb(o):
    p = np.asarray(o.pos, dtype=float)
    h = np.asarray(o.size) / 2 if isinstance(o, Box) else np.full(3, o.radius)
    return p - h, p + h


def axes(yaw):
    """(closing axis, across axis) of a straight-down grip at this yaw; yaw 0 closes along world y."""
    return np.array([-math.sin(yaw), math.cos(yaw), 0.0]), np.array([math.cos(yaw), math.sin(yaw), 0.0])


def pads(tcp, yaw, inner):
    """The two pads as (centre, closing axis, across axis, half extents, z range) with inner faces `inner` metres
    either side of the point `tcp` along the closing axis (inner may be a pair: +side, -side)."""
    c, v = axes(yaw)
    tip = tcp[2] - PAD_TIP_PAST_TCP
    out = []
    for sign, d in ((1, inner[0]), (-1, inner[1])):
        centre = np.asarray(tcp, float) + sign * (d + PAD_T / 2) * c
        out.append((centre, c, v, (PAD_T / 2, PAD_HALF_WIDTH), (tip, tip + PAD_LEN)))
    return out


def finger_block(tcp, yaw):
    """Closed fingers: one block 2 x PAD_T along the closing axis, 40 mm across."""
    c, v = axes(yaw)
    tip = tcp[2] - PAD_TIP_PAST_TCP
    return (np.asarray(tcp, float), c, v, (PAD_T, PAD_HALF_WIDTH), (tip, tip + PAD_LEN))


def gap_to_ball(box, centre):
    """Clearance (m) between a pad box and a ball; negative = overlap."""
    bc, c, v, (hc, hv), (z0, z1) = box
    d = np.asarray(centre, float) - bc
    lc = np.clip(d @ c, -hc, hc)
    lv = np.clip(d @ v, -hv, hv)
    near = bc + lc * c + lv * v
    near[2] = np.clip(centre[2], z0, z1)
    return float(np.linalg.norm(np.asarray(centre, float) - near) - R)


def releases(side, label):
    """(waypoint, corner) for every waypoint with this label in an arm's plan, with the corner it serves."""
    out, corner = [], None
    for w in TASK.demo()[side]:
        if w.label.startswith("carry ") and " corner" in w.label:
            corner = w.label.split(" the ")[1].replace(" corner", "")
        if w.label == label:
            out.append((w, corner))
    return out


ORDER = ("near_left", "far_right", "far_left", "near_right")  # the demo's order of placement
NEIGHBOURS_AT = {c: [cornered(p) for p in ORDER[:i]] for i, c in enumerate(ORDER)}  # balls already in place, cornered


# -- 1. the file loads ------------------------------------------------------------------------------------------


def test_the_task_file_loads_with_its_hooks_and_docs():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.free_objects == M.BALL_NAMES
    assert TASK.docs.startswith("On the station table")
    assert all(isinstance(o, (Box, Sphere)) for o in TASK.objects)
    assert all(isinstance(o, Sphere) for o in TASK.objects if o.free)
    assert len({(o.radius, o.mass, o.color) for o in M.BALLS}) == 1  # five identical balls


# -- 2. the demo is reachable -----------------------------------------------------------------------------------


@pytest.mark.parametrize("side", ["left", "right"])
def test_the_demo_is_reachable_within_the_joint_limits(side):
    plan = TASK.demo()
    assert set(plan) == {"left", "right"}
    arm = ArmPlan(plan[side], station.BASES[side])
    steps = round(arm.duration * HZ) + 5
    actions = np.array([arm.action7((k + 1) / HZ)[0] for k in range(steps)])
    assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1
    assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)


def test_the_demo_fits_the_run_limit_and_ends_with_a_still_hold():
    plan = TASK.demo()
    durations = [sum(w.duration for w in plan[s]) for s in ("left", "right")]
    assert durations[0] == pytest.approx(durations[1])
    assert durations[0] * HZ + 100 < RUN_LIMIT_STEPS / 2
    for s in ("left", "right"):
        assert plan[s][-1].label == "hold still" and plan[s][-1].duration >= 1.5
    t_release = sum(w.duration for w in itertools.takewhile(lambda w: w.label != "let go on top", plan["left"]))
    assert durations[0] - t_release >= M.HOLD_S + 1.5


def sample(plan, side, dt=0.02):
    arm = ArmPlan(plan[side], station.BASES[side])
    ts = np.arange(0.0, arm.duration + dt, dt)
    return [arm.target(t) for t in ts]


def test_the_arms_stay_apart():
    plan = TASK.demo()
    left, right = sample(plan, "left"), sample(plan, "right")
    gap = min(np.linalg.norm(a[0] - b[0]) for a, b in zip(left, right, strict=True))
    assert gap > 0.18


def test_only_one_arm_is_over_the_tray_at_a_time():
    plan = TASK.demo()
    half = M.TRAY_OUT / 2 + 0.06  # tray footprint plus an open pad's reach
    inside = {}
    for side in ("left", "right"):
        inside[side] = [abs(p[0] - M.TRAY_X) < half and abs(p[1] - M.TRAY_Y) < half for p, *_ in sample(plan, side)]
    assert not any(a and b for a, b in zip(inside["left"], inside["right"], strict=True))


def test_lowering_and_pushing_are_slow():
    for side in ("left", "right"):
        for w_prev, w in zip(TASK.demo()[side], TASK.demo()[side][1:]):
            dist = np.linalg.norm(np.subtract(w.pos, w_prev.pos))
            peak = 1.5 * dist / w.duration  # smoothstep peaks at 1.5 x the mean speed
            if w.label == "lower into the tray":
                assert dist / w.duration <= 0.036  # about 3.5 cm/s
            if w.label == "push into the corner":
                assert peak <= 0.025


# -- 3. the demo's geometry: the traps a centred-grip sweep would miss -------------------------------------------


def test_a_released_bottom_ball_lands_between_its_corner_and_its_cornered_neighbours():
    # u = distance from the tray centre along the corner's diagonal. Cornered neighbours are touched below
    # u = TOP_RISE_CORNERS; the rims are touched beyond u = DIAG.
    lo = M.RELEASE_U - GRIP_SLOP
    hi = M.RELEASE_U + GRIP_SLOP
    assert hi <= M.DIAG + 1e-9
    assert lo >= M.TOP_RISE_CORNERS + 0.005
    assert M.MAX_GRIP_OFFSET == pytest.approx(0.009, abs=1e-4)


@pytest.mark.parametrize("corner", ORDER)
def test_pads_clear_the_balls_already_in_the_tray_when_a_bottom_ball_is_let_go(corner):
    yaw = M.corner_yaw(corner)
    c, v = axes(yaw)
    tcp = np.array(M.on_diagonal(corner, M.RELEASE_U, M.RELEASE_Z))
    worst = np.inf
    for du in np.linspace(-GRIP_SLOP, GRIP_SLOP, 15):
        ball = tcp - M.GRIP_ABOVE_CENTRE * np.array([0, 0, 1]) + du * c
        # closed on the ball: inner faces touch its equator
        boxes = pads(ball + np.array([0, 0, M.GRIP_ABOVE_CENTRE]), yaw, (R, R))
        # opening: both pads move to the open stop around the (possibly shifted) TCP
        for dt in (-M.TRACK_ERR, 0.0, M.TRACK_ERR):
            boxes += pads(tcp + dt * c, yaw, (OPEN_GAP_HALF, OPEN_GAP_HALF))
        for nb in NEIGHBOURS_AT[corner]:
            worst = min(worst, min(gap_to_ball(b, nb) for b in boxes))
            assert np.linalg.norm(ball - nb) >= M.D + 0.003  # the falling ball misses its neighbours
    if NEIGHBOURS_AT[corner]:
        assert worst >= 0.008
    # wrist error across the closing axis too
    for dv in (-M.TRACK_ERR, M.TRACK_ERR):
        for b in pads(tcp + dv * v, yaw, (OPEN_GAP_HALF, OPEN_GAP_HALF)):
            for nb in NEIGHBOURS_AT[corner]:
                assert gap_to_ball(b, nb) >= 0.003


def test_pad_tips_clear_the_rim_and_the_floor_at_every_bottom_release():
    tips = M.RELEASE_Z - PAD_TIP_PAST_TCP
    assert tips - M.RIM_TOP >= 0.008
    assert M.GRIP_ABOVE_CENTRE < PAD_TIP_PAST_TCP - 0.005  # pads still reach below the equator
    assert M.RELEASE_Z - M.GRIP_ABOVE_CENTRE - R - M.FLOOR_TOP == pytest.approx(M.RELEASE_DROP)
    assert 0.003 <= M.RELEASE_DROP <= 0.010  # a short, proven drop


@pytest.mark.parametrize("corner", ORDER)
def test_the_corner_push_misses_the_other_balls_and_stops_short_of_the_rims(corner):
    yaw = M.corner_yaw(corner)
    own_innermost = np.array(M.on_diagonal(corner, M.RELEASE_U - GRIP_SLOP, M.BOTTOM_Z))
    for u in np.linspace(M.PUSH_START_U, M.PUSH_END_U, 31):
        block = finger_block(np.array(M.on_diagonal(corner, u, M.PUSH_Z)), yaw)
        for nb in NEIGHBOURS_AT[corner]:
            assert gap_to_ball(block, nb) >= 0.005
    # descends on the inner side of the ball wherever it landed
    start = finger_block(np.array(M.on_diagonal(corner, M.PUSH_START_U, M.PUSH_Z)), yaw)
    assert gap_to_ball(start, own_innermost) >= 0.003
    # ends 2 mm into a cornered ball: the soft wrist presses it gently against the rims while it holds (2 mm short
    # left the ball free to roll along a rim, GPU round 3)
    end = finger_block(np.array(M.on_diagonal(corner, M.PUSH_END_U, M.PUSH_Z)), yaw)
    assert gap_to_ball(end, cornered(corner)) == pytest.approx(-0.002, abs=1e-4)
    # at the push height the block spans the ball's equator and stays off the floor
    assert M.PUSH_Z - PAD_TIP_PAST_TCP < M.BOTTOM_Z < M.PUSH_Z
    assert M.PUSH_Z - PAD_TIP_PAST_TCP - M.FLOOR_TOP > 0.02


def test_the_top_ball_is_let_go_clear_of_the_four_with_its_pads_between_them():
    yaw = 0.0
    c, v = axes(yaw)
    tcp = np.array([M.TRAY_X, M.TRAY_Y, M.TOP_RELEASE_Z])
    bottoms = [cornered(k) for k in CORNER_NAMES]
    worst_pad = np.inf
    for du in np.linspace(-GRIP_SLOP, GRIP_SLOP, 15):
        for dv in (-M.TRACK_ERR, 0.0, M.TRACK_ERR):
            ball_tcp = tcp + du * c + dv * v
            boxes = pads(ball_tcp, yaw, (R, R)) + pads(tcp + dv * v, yaw, (OPEN_GAP_HALF, OPEN_GAP_HALF))
            for b in bottoms:
                worst_pad = min(worst_pad, min(gap_to_ball(x, b) for x in boxes))
            ball = ball_tcp - np.array([0, 0, M.GRIP_ABOVE_CENTRE])
            # wherever it is let go, it lands inside the pocket the four centres make and rolls to the seat
            assert max(abs(ball[0] - M.TRAY_X), abs(ball[1] - M.TRAY_Y)) < M.CORNER - 0.025
    assert worst_pad >= 0.005
    # the held ball clears the four for any grip offset up to the bound the open pads allow
    for du in np.linspace(-M.MAX_GRIP_OFFSET, M.MAX_GRIP_OFFSET, 19):
        ball = tcp - np.array([0, 0, M.GRIP_ABOVE_CENTRE]) + du * c
        assert all(np.linalg.norm(ball - b) > M.D + 0.001 for b in bottoms)
    assert M.TOP_RELEASE_Z - M.GRIP_ABOVE_CENTRE - SEAT[2] == pytest.approx(M.TOP_DROP)
    assert M.TOP_DROP <= 0.015


def test_carrying_and_clearing_heights_pass_over_the_bottom_balls():
    crowns = M.FLOOR_TOP + M.D
    assert M.TRAVEL_Z - M.GRIP_ABOVE_CENTRE - R - crowns >= 0.010  # a carried ball over a placed one
    assert M.CLEAR_Z - PAD_TIP_PAST_TCP - crowns >= 0.008  # open or closed pads over a placed ball
    assert M.TOP_RELEASE_Z + M.LIFT_AFTER_RELEASE - PAD_TIP_PAST_TCP > M.TOP_RELEASE_Z - PAD_TIP_PAST_TCP + 0.029


def test_the_grip_offset_bound_and_every_ball_fits_the_gripper_with_a_centimetre_to_spare():
    assert M.GRIPPER_MAX_OPENING == pytest.approx(-2 * FINGER_OPEN, abs=1e-4)
    assert FINGER_CLOSED == 0.0
    for o in M.BALLS:
        assert 2 * o.radius <= M.GRIPPER_MAX_OPENING - 0.010


def test_grasps_from_the_rack_keep_the_pads_off_the_board_and_the_neighbours():
    tips = M.SEAT_Z + M.GRIP_ABOVE_CENTRE - PAD_TIP_PAST_TCP
    assert tips - M.RACK_TOP >= 0.015
    for xs, y in ((M.RACK_L_X, M.RACK_L_Y), (M.RACK_R_X, M.RACK_R_Y)):
        for i, x in enumerate(xs):
            tcp = np.array([x, y, M.SEAT_Z + M.GRIP_ABOVE_CENTRE])
            for j, x2 in enumerate(xs):
                if i != j:
                    for b in pads(tcp, 0.0, (OPEN_GAP_HALF, OPEN_GAP_HALF)):
                        assert gap_to_ball(b, (x2, y, M.SEAT_Z)) >= 0.02


# -- 4. the geometry that makes the task what it is ------------------------------------------------------------


def test_the_top_ball_cannot_fall_through_and_only_four_fit_on_the_floor():
    assert M.SPREAD < M.FALL_THROUGH_SPREAD - 0.010  # 20.6 mm of play against 31.5 mm
    assert M.TOP_RISE_CORNERS > 0.030
    # Five points 76 mm apart cannot fit in the square the floor balls' centres can reach: the best spread of five
    # points in a square of side L is L / sqrt(2) apart.
    assert 2 * M.CORNER / math.sqrt(2) < M.D


def test_the_rim_holds_a_cornered_ball_under_the_top_ball():
    # The top ball pushes each bottom ball along the diagonal into two rims. Tipping over one rim's top edge would
    # need the horizontal push times its lever (R - WALL_H) to exceed the vertical load times EDGE_REACH.
    w = M.BALL_MASS * 9.81
    push_diag = w / 4 * M.DIAG / M.TOP_RISE_CORNERS
    push_one_rim = push_diag / math.sqrt(2)
    load = w + w / 4
    assert push_one_rim * (R - M.WALL_H) * 5 < load * M.EDGE_REACH  # a factor of 5 to spare
    assert M.WALL_H < R  # the rim is touched at its top edge, which sets CORNER
    assert M.CORNER == pytest.approx(M.TRAY_IN / 2 - M.EDGE_REACH)


def test_the_rim_is_low_enough_to_see_the_bottom_layer_but_confines_it():
    assert 0.015 <= M.WALL_H <= 0.030


# -- 5. scoring: goal, partial credit, every branch -------------------------------------------------------------


def test_the_goal_state_succeeds_with_full_score_and_zero_distance():
    g = goal_state()
    assert TASK.success(g) and TASK.score(g) == 1.0 and TASK.distance(g) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("top", M.BALL_NAMES)
def test_any_ball_can_be_the_top_one(top):
    g = goal_state(top)
    assert TASK.success(g) and TASK.score(g) == 1.0


def test_the_top_ball_on_four_touching_balls_also_counts():
    objects = fixed_states()
    half = M.D / 2
    for n, (sx, sy) in zip(M.BALL_NAMES[:4], ((-1, 1), (1, 1), (-1, -1), (1, -1)), strict=True):
        objects[n] = state((M.TRAY_X + sx * half, M.TRAY_Y + sy * half, M.BOTTOM_Z))
    objects["ball_e"] = state((M.TRAY_X, M.TRAY_Y, M.BOTTOM_Z + M.TOP_RISE_TIGHT))
    assert TASK.score(objects) == 1.0 and TASK.success(objects)


def test_the_start_state_scores_zero():
    s = start_state()
    assert TASK.score(s) == 0.0 and not TASK.success(s) and TASK.distance(s) > 0.5


@pytest.mark.parametrize("k", range(5))
def test_each_ball_on_the_tray_floor_is_worth_0_15(k):
    objects = start_state()
    g = goal_state()
    for n in [n for n in M.BALL_NAMES if n != "ball_c"][:k]:
        objects[n] = g[n]
    assert TASK.score(objects) == pytest.approx(0.15 * k)
    assert not TASK.success(objects)


def test_four_in_the_tray_and_one_left_on_the_table_scores_0_6():
    objects = goal_state()
    objects["ball_c"] = state((0.30, 0.15, M.TOP + R))
    assert TASK.score(objects) == pytest.approx(0.6) and not TASK.success(objects)
    objects["ball_c"] = start_state()["ball_c"]  # or never taken from the rack
    assert TASK.score(objects) == pytest.approx(0.6)


def test_five_balls_in_a_row_score_zero():
    objects = fixed_states()
    for i, n in enumerate(M.BALL_NAMES):
        objects[n] = state((0.12 + 0.08 * i, 0.12, M.TOP + R))  # in a row on the table in front of the tray
    assert TASK.score(objects) == 0.0 and not TASK.success(objects)
    for i, n in enumerate(M.BALL_NAMES):  # in a row across the tray's top, resting on the rims and each other
        objects[n] = state((M.TRAY_X, -0.16 + 0.08 * i, M.RIM_TOP + R))
    assert TASK.score(objects) == 0.0


def test_a_top_ball_lying_in_a_gap_at_the_edge_earns_nothing_for_the_top():
    objects = goal_state()
    # between the two far balls, against the far rim: raised, but 48 mm from the centre
    objects["ball_c"] = state((M.TRAY_X + M.CORNER, M.TRAY_Y, M.BOTTOM_Z + 0.04))
    assert TASK.score(objects) == pytest.approx(0.6) and not TASK.success(objects)
    objects["ball_c"] = state((M.TRAY_X + 0.016, M.TRAY_Y, SEAT[2]))  # 16 mm off centre
    assert TASK.score(objects) == pytest.approx(0.6)


def test_a_top_ball_sunk_into_the_layer_or_held_above_it_earns_nothing_for_the_top():
    objects = goal_state()
    objects["ball_c"] = state((M.TRAY_X, M.TRAY_Y, M.BOTTOM_Z + M.TOP_RISE_MIN - 0.002))
    assert TASK.score(objects) == pytest.approx(0.6)
    objects["ball_c"] = state((M.TRAY_X, M.TRAY_Y, M.BOTTOM_Z + M.TOP_RISE_MAX + 0.002))
    assert TASK.score(objects) == pytest.approx(0.6)


def test_a_top_ball_needs_a_complete_bottom_layer():
    objects = goal_state()
    objects["ball_a"] = state((0.30, 0.15, M.TOP + R))  # one bottom ball out on the table
    assert TASK.score(objects) == pytest.approx(0.45) and not TASK.success(objects)


def test_moving_balls_earn_nothing_until_they_settle():
    objects = goal_state()
    objects["ball_c"] = state(SEAT, vel=(0.02, 0.0, 0.0))  # top ball still rolling
    assert TASK.score(objects) == pytest.approx(0.6) and not TASK.success(objects)
    objects = goal_state()
    objects["ball_a"] = state(objects["ball_a"].pos, vel=(0.0, 0.02, 0.0))  # a bottom ball rolling
    assert TASK.score(objects) == pytest.approx(0.45) and not TASK.success(objects)


def test_a_ball_on_the_rim_or_outside_the_tray_is_not_on_the_floor():
    objects = goal_state()
    objects["ball_a"] = state((M.TRAY_X - M.TRAY_IN / 2 - 0.005, M.TRAY_Y + 0.03, M.RIM_TOP + 0.03))  # on the rim
    assert TASK.score(objects) == pytest.approx(0.45)
    objects["ball_a"] = state((M.TRAY_X - M.TRAY_OUT / 2 - R, M.TRAY_Y, M.TOP + R))  # just outside
    assert TASK.score(objects) == pytest.approx(0.45)
    objects["ball_a"] = state((0.3, 0.75, R))  # on the floor of the room
    assert TASK.score(objects) == pytest.approx(0.45)


def test_all_scores_lie_in_0_1_and_on_the_credit_grid():
    g, s = goal_state(), start_state()
    for mask in itertools.product((0, 1, 2), repeat=5):
        objects = dict(s)
        for n, m in zip(M.BALL_NAMES, mask, strict=True):
            if m == 1:
                objects[n] = g[n]
            elif m == 2:
                objects[n] = state((0.3, 0.75, R))
        x = TASK.score(objects)
        assert 0.0 <= x <= 1.0
        assert any(math.isclose(x, 0.15 * b + 0.4 * t) for b in range(5) for t in (0, 1))


def test_distance_shrinks_as_balls_go_in():
    objects = start_state()
    g = goal_state()
    last = TASK.distance(objects)
    for n in ("ball_a", "ball_b", "ball_d", "ball_e", "ball_c"):
        objects[n] = g[n]
        d = TASK.distance(objects)
        assert d < last
        last = d
    assert last == pytest.approx(0.0, abs=1e-9)


# -- 6. start layout --------------------------------------------------------------------------------------------


def test_no_two_objects_overlap_at_the_start():
    for a, b in itertools.combinations(M.OBJECTS, 2):
        (alo, ahi), (blo, bhi) = aabb(a), aabb(b)
        if isinstance(a, Sphere) or isinstance(b, Sphere):
            # a seated ball's box dips into its hole; compare against the true hole edges instead
            sph, other = (a, b) if isinstance(a, Sphere) else (b, a)
            if isinstance(other, Sphere):
                assert np.linalg.norm(np.subtract(sph.pos, other.pos)) > 2 * R
                continue
            lo, hi = aabb(other)
            near = np.clip(sph.pos, lo, hi)
            assert np.linalg.norm(np.subtract(sph.pos, near)) > R, (sph.name, other.name)
            continue
        overlap = np.minimum(ahi, bhi) - np.maximum(alo, blo)
        assert overlap.min() <= 1e-9, (a.name, b.name)


def test_every_ball_starts_seated_in_a_rack_hole_and_cannot_roll_away():
    holes = [(x, M.RACK_L_Y) for x in M.RACK_L_X] + [(x, M.RACK_R_Y) for x in M.RACK_R_X]
    assert sorted((o.pos[0], o.pos[1]) for o in M.BALLS) == sorted(holes)
    for o in M.BALLS:
        assert o.pos[2] == pytest.approx(M.SEAT_Z + M.SPAWN_GAP)
        assert o.pos[2] - M.SPAWN_GAP - R > M.TOP  # hangs in the hole clear of the table
    assert M.HOLE < 2 * R - 0.02  # sits on the four edges, well short of dropping through
    # the rack's pieces leave exactly the holes open
    for prefix, xs, y in (("rack_left", M.RACK_L_X, M.RACK_L_Y), ("rack_right", M.RACK_R_X, M.RACK_R_Y)):
        crosses = sorted((aabb(o) for o in M.OBJECTS if o.name.startswith(f"{prefix}_cross")), key=lambda b: b[0][0])
        gaps = [(b[0][0], a[1][0]) for a, b in zip(crosses, crosses[1:])]
        assert [round((lo + hi) / 2, 6) for hi, lo in gaps] == [round(x, 6) for x in xs]
        assert all(math.isclose(hi - lo, M.HOLE) for lo, hi in ((a[1][0], b[0][0]) for a, b in zip(crosses, crosses[1:])))
        sides = [aabb(o) for o in M.OBJECTS if o.name.startswith(f"{prefix}_side")]
        assert sorted(round(b[0][1], 6) for b in sides)[1] == pytest.approx(y + M.HOLE / 2)


def test_the_tray_rims_enclose_the_floor_corner_to_corner():
    rims = [aabb(o) for o in M.OBJECTS if o.name.startswith("tray_rim")]
    floor = aabb(next(o for o in M.OBJECTS if o.name == "tray_floor"))
    assert floor[1][2] == pytest.approx(M.FLOOR_TOP)
    for lo, hi in rims:
        assert lo[2] == pytest.approx(M.FLOOR_TOP) and hi[2] == pytest.approx(M.RIM_TOP)
    xs = sorted({round(v, 6) for lo, hi in rims for v in (lo[0], hi[0])})
    ys = sorted({round(v, 6) for lo, hi in rims for v in (lo[1], hi[1])})
    assert xs[0] == pytest.approx(M.TRAY_X - M.TRAY_OUT / 2) and xs[-1] == pytest.approx(M.TRAY_X + M.TRAY_OUT / 2)
    assert ys[0] == pytest.approx(M.TRAY_Y - M.TRAY_OUT / 2) and ys[-1] == pytest.approx(M.TRAY_Y + M.TRAY_OUT / 2)
    inner_x = sorted(v for lo, hi in rims for v in (lo[0], hi[0]) if abs(v - M.TRAY_X) < M.TRAY_OUT / 2 - 1e-6)
    assert min(abs(v - M.TRAY_X) for v in inner_x) == pytest.approx(M.TRAY_IN / 2)
