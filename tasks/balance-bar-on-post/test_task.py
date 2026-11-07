"""Offline checks for the balance-a-bar-on-a-post task (numpy + pytest only, no Isaac Sim).

Run with the yam-isaac-lab-arena clone (commit bd62771) importable, e.g.
    YAM_ARENA_PATH=~/yam-isaac-lab-arena pytest -q test_task.py
"""

import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

if os.environ.get("YAM_ARENA_PATH"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_PATH"]))

from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import FINGER_OPEN, GRIPPY_FRICTION, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import Box, ObjectState, load  # noqa: E402

HERE = Path(__file__).parent
TASK = load(HERE / "task.py")
M = TASK.module
META = json.loads((HERE / "task.json").read_text())
HZ = 20
RUN_LIMIT_STEPS = 2400
PAD_INNER_OPEN = 0.047  # inner face of an open pad from the TCP along the closing direction
PAD_OUTER = 0.057  # outer face of an open pad
PAD_HALF_WIDTH = 0.020  # pad extent across the closing direction

# Error budgets of the statics (see README "Balance"): where the bar's middle and the cubes may land.
E = M.CENTRE_ALONG  # the bar's middle off the post's axis along the bar: up to the scored 10 mm
D = 0.015  # each cube off its spot along the bar, either way (the grip offset runs along the bar: 12-20 mm)
W2 = M.POST_W / 2
BAR_MASS_RANGE = (0.140, 0.200)  # pine boards of this size weigh about this much


# -- states --------------------------------------------------------------------------------------------------


def quat_about_x(angle):
    return np.array([math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2)])


def state(pos, size, quat=(0.0, 0.0, 0.0, 1.0), vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, dtype=float)
    h = np.asarray(size, dtype=float) / 2
    return ObjectState(pos, np.asarray(quat, dtype=float), np.asarray(vel, dtype=float), np.zeros(3), pos - h, pos + h)


BAR_SIZE = (M.BAR_W, M.BAR_L, M.BAR_T)
CUBE_SIZE = (M.CUBE,) * 3


def start_state():
    return {o.name: state(np.asarray(o.pos) - (0, 0, M.SPAWN_GAP if o.free else 0.0), o.size) for o in M.OBJECTS}


def bar_on_post(e=0.0, across=0.0, angle=0.0, vel=(0.0, 0.0, 0.0)):
    """The bar on the post with its middle e along y (and `across` along x) off the post's axis, turned `angle`
    about x (a positive angle lowers its +y end)."""
    q = quat_about_x(-angle)
    centre = np.array([M.POST_X + across, M.POST_Y + e, M.POST_TOP + M.BAR_T / 2])
    return state(centre, BAR_SIZE, q, vel)


def on_bar(bar, d, x=0.0, level=0, vel=(0.0, 0.0, 0.0)):
    """A cube on the bar's top face at d along it from its middle (stacked `level` cubes high)."""
    r = M.rotation(bar.quat_xyzw)
    local = np.array([x, d, M.BAR_T / 2 + M.CUBE / 2 + level * M.CUBE])
    return state(np.asarray(bar.pos) + r @ local, CUBE_SIZE, bar.quat_xyzw, vel)


def scene(bar=None, left=None, right=None):
    s = start_state()
    if bar is not None:
        s["bar"] = bar
    if left is not None:
        s["block_left"] = left
    if right is not None:
        s["block_right"] = right
    return s


def goal_state(e=0.0, dl=0.0, dr=0.0):
    bar = bar_on_post(e)
    return scene(bar, on_bar(bar, M.SPOT_D + dl), on_bar(bar, -M.SPOT_D + dr))


# -- 1. the file and its metadata ----------------------------------------------------------------------------


def test_the_file_loads_with_its_hooks_and_docs():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.free_objects == ("bar", "block_left", "block_right")
    assert TASK.docs.startswith("On the station table")
    assert all(isinstance(o, Box) for o in TASK.objects)  # primitives only, no library assets
    assert not M.POST.free


def test_task_json_has_every_field_and_no_scores_yet():
    for key in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                "fidelity", "scores"):
        assert META[key], key
    # scores is filled after the Dream-1 run and media by the upload script
    assert META["scores"] == "not yet run" or META["scores"].startswith("Dream-1 run: "), META["scores"]
    assert META["run_limit"].startswith(f"{RUN_LIMIT_STEPS:,} steps")


def test_no_affiliation_appears_in_the_shipped_text():
    readme = (HERE / "README.md").read_text()
    for text in (json.dumps(META), TASK.docs, readme):
        low = text.lower()
        for word in ("university", "institute", "college", "our team", "our lab", "@"):
            assert word not in low, word


def test_the_readme_runs_the_exact_instruction():
    assert META["instruction"] in (HERE / "README.md").read_text()


# -- 2. the demo is reachable and well-formed ----------------------------------------------------------------

PLAN = TASK.demo()


@pytest.mark.parametrize("side", ["left", "right"])
def test_the_demo_is_reachable_within_the_joint_limits(side):
    arm = ArmPlan(PLAN[side], station.BASES[side])
    steps = round(arm.duration * HZ) + 5
    actions = np.array([arm.action7((i + 1) / HZ)[0] for i in range(steps)])
    assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1
    assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)


def test_the_demo_fits_the_run_limit_and_ends_with_both_arms_still_for_1_5_s():
    durations = [sum(w.duration for w in PLAN[s]) for s in ("left", "right")]
    assert durations[0] == pytest.approx(durations[1])
    assert durations[0] * HZ + HZ * TASK.success_hold_s + 100 < RUN_LIMIT_STEPS / 2
    for side in ("left", "right"):
        still, prev = 0.0, None
        for w in reversed(PLAN[side]):
            if prev is not None and (w.pos != prev.pos or w.finger != prev.finger):
                break
            still += w.duration
            prev = w
        assert still >= 1.5, side


def sample(side, dt=0.02):
    arm = ArmPlan(PLAN[side], station.BASES[side])
    ts = np.arange(0.0, arm.duration + dt, dt)
    return ts, [arm.target(t)[0] for t in ts]


def test_the_arms_stay_apart():
    _, left = sample("left")
    _, right = sample("right")
    assert min(np.linalg.norm(a - b) for a, b in zip(left, right, strict=True)) > 0.18


def test_every_release_is_followed_by_a_3_cm_lift_before_any_sideways_move():
    for side in ("left", "right"):
        wps = PLAN[side]
        for i, w in enumerate(wps):
            if not w.label.startswith("let go"):
                continue
            assert w.finger == FINGER_OPEN
            nxt = wps[i + 1]
            assert nxt.pos[:2] == pytest.approx(w.pos[:2])
            assert nxt.pos[2] - w.pos[2] >= 0.03 - 1e-9


def start_of(side, label):
    t = 0.0
    for w in PLAN[side]:
        if w.label == label:
            return t
        t += w.duration
    raise KeyError(label)


def test_both_cubes_come_down_and_are_let_go_at_the_same_moment():
    for step in ("carry over the {} end", "lower onto the {} end", "let go of the {} cube on the bar"):
        tl, tr = start_of("left", step.format("left")), start_of("right", step.format("right"))
        assert tl == pytest.approx(tr), step
    # from the carry on, the two arms' plans are mirror images in y with equal durations
    i = next(k for k, w in enumerate(PLAN["left"]) if w.label == "carry over the left end")
    j = next(k for k, w in enumerate(PLAN["right"]) if w.label == "carry over the right end")
    tail_l, tail_r = PLAN["left"][i:], PLAN["right"][j:]
    assert [w.duration for w in tail_l] == pytest.approx([w.duration for w in tail_r])
    assert [w.finger for w in tail_l] == [w.finger for w in tail_r]


def test_the_bar_is_on_the_post_and_the_hand_off_it_before_any_cube_is_over_the_bar():
    off = start_of("left", "lift off") + 0.8
    assert start_of("left", "carry over the left end") >= off
    assert start_of("right", "carry over the right end") >= off
    labels = [w.label for s in ("left", "right") for w in PLAN[s] if w.label.startswith("let go")]
    assert sorted(labels) == sorted(
        ["let go of the bar on the post", "let go of the left cube on the bar", "let go of the right cube on the bar"]
    )


def test_the_right_arm_holds_its_cube_clear_of_the_bar_while_the_bar_is_placed():
    # the held cube's near face and the bar's near end, along y, during the whole bar phase
    t_end = start_of("left", "lift off") + 0.8
    ts, right = sample("right")
    for t, p in zip(ts, right, strict=True):
        if t <= t_end and p[2] > M.CUBE_PICK_Z + 0.01:
            assert -p[1] - M.CUBE / 2 - (M.BAR_L / 2) > 0.03


# -- 3. goal state and partial credit ------------------------------------------------------------------------


def test_the_goal_state_succeeds_with_full_score_and_near_zero_distance():
    s = goal_state()
    assert TASK.success(s) and TASK.score(s) == 1.0
    assert TASK.distance(s) == pytest.approx(0.0, abs=1e-9)


def test_the_start_state_scores_zero_and_is_far_away():
    s = start_state()
    assert TASK.score(s) == 0.0 and not TASK.success(s)
    assert TASK.distance(s) > 0.3


@pytest.mark.parametrize("e", [-0.009, 0.0, 0.009])
@pytest.mark.parametrize("dl,dr", [(-0.024, 0.024), (0.024, -0.024), (0.0, 0.0), (-0.024, -0.024)])
def test_full_score_wherever_the_cubes_sit_on_the_ends(e, dl, dr):
    # the end zone is the outer 50 mm: the spot is 25 mm in, so +-24 mm about it stays inside
    assert TASK.score(goal_state(e, dl, dr)) == 1.0


def test_the_cubes_swapped_between_ends_still_score_full():
    bar = bar_on_post()
    s = scene(bar, on_bar(bar, -M.SPOT_D), on_bar(bar, M.SPOT_D))
    assert TASK.score(s) == 1.0


def test_the_bar_balanced_and_empty_scores_0_3():
    assert TASK.score(scene(bar_on_post())) == pytest.approx(0.3)


@pytest.mark.parametrize("d", [M.SPOT_D, -M.SPOT_D, 0.0, 0.06])
def test_exactly_one_cube_on_the_balanced_bar_scores_0_5(d):
    # an end cube alone can only be there while a hand holds the bar level: still partial progress
    bar = bar_on_post()
    assert TASK.score(scene(bar, left=on_bar(bar, d))) == pytest.approx(0.5)


def test_one_cube_on_each_half_but_not_on_the_ends_scores_0_5():
    bar = bar_on_post()
    assert TASK.score(scene(bar, on_bar(bar, 0.06), on_bar(bar, -0.06))) == pytest.approx(0.5)
    assert TASK.score(scene(bar, on_bar(bar, M.SPOT_D), on_bar(bar, -0.06))) == pytest.approx(0.5)


def test_both_ends_loaded_but_a_cube_still_moving_is_not_yet_full():
    bar = bar_on_post()
    s = scene(bar, on_bar(bar, M.SPOT_D, vel=(0.0, 0.02, 0.0)), on_bar(bar, -M.SPOT_D))
    assert TASK.score(s) == pytest.approx(0.5) and not TASK.success(s)


@pytest.mark.parametrize(
    "dl,dr,level",
    [(M.SPOT_D, M.SPOT_D - 0.045, 0), (-M.SPOT_D, -M.SPOT_D + 0.045, 0), (0.05, 0.005, 0), (-0.03, -0.03, 1),
     (M.SPOT_D, M.SPOT_D, 1)],
)
def test_both_cubes_on_the_same_half_score_zero(dl, dr, level):
    bar = bar_on_post()
    s = scene(bar, on_bar(bar, dl), on_bar(bar, dr, level=level))
    assert TASK.score(s) == 0.0 and not TASK.success(s)


@pytest.mark.parametrize("angle", [math.radians(6), -math.radians(6), math.radians(50)])
def test_a_tipped_bar_scores_zero_with_any_load(angle):
    bar = bar_on_post(angle=angle)
    for s in (scene(bar), scene(bar, on_bar(bar, M.SPOT_D), on_bar(bar, -M.SPOT_D))):
        assert TASK.score(s) == 0.0


def test_a_bar_tilted_under_5_deg_still_counts_as_level():
    bar = bar_on_post(angle=math.radians(4))
    assert TASK.score(scene(bar, on_bar(bar, M.SPOT_D), on_bar(bar, -M.SPOT_D))) == 1.0


@pytest.mark.parametrize("e,across", [(0.012, 0.0), (-0.012, 0.0), (0.0, 0.028)])
def test_a_bar_off_centre_on_the_post_scores_zero(e, across):
    bar = bar_on_post(e, across)
    assert TASK.score(scene(bar, on_bar(bar, M.SPOT_D), on_bar(bar, -M.SPOT_D))) == 0.0


def test_the_bar_still_moving_scores_zero():
    bar = bar_on_post(vel=(0.0, 0.0, 0.02))
    assert TASK.score(scene(bar, on_bar(bar, M.SPOT_D), on_bar(bar, -M.SPOT_D))) == 0.0


def test_building_the_load_on_the_table_instead_of_the_post_scores_zero():
    bar = state((M.POST_X - 0.10, 0.0, M.TOP + M.BAR_T / 2), BAR_SIZE)
    s = scene(bar, on_bar(bar, M.SPOT_D), on_bar(bar, -M.SPOT_D))
    assert TASK.score(s) == 0.0


def test_a_bar_lifted_off_the_post_or_pressed_into_it_scores_zero():
    for dz in (0.005, -0.004):
        bar = bar_on_post()
        bar = state(np.asarray(bar.pos) + (0, 0, dz), BAR_SIZE)
        assert TASK.score(scene(bar, on_bar(bar, M.SPOT_D), on_bar(bar, -M.SPOT_D))) == 0.0


def test_a_cube_held_above_its_end_does_not_count_as_on_the_end():
    bar = bar_on_post()
    hover = state(np.asarray(on_bar(bar, -M.SPOT_D).pos) + (0, 0, 0.006), CUBE_SIZE)
    s = scene(bar, on_bar(bar, M.SPOT_D), hover)
    assert TASK.score(s) == pytest.approx(0.5)


@pytest.mark.parametrize("what", ["bar", "block_left", "block_right"])
def test_anything_on_the_floor_scores_zero(what):
    s = goal_state()
    s[what] = state((0.3, 0.6, M.TOP - 0.3), s[what].high - s[what].low)
    assert TASK.score(s) == 0.0


def test_distance_shrinks_as_the_bar_and_cubes_near_their_places():
    bar = bar_on_post()
    half = scene(bar)
    assert TASK.distance(start_state()) > TASK.distance(half) > TASK.distance(goal_state())


# -- 4. statics: the derivation in README "Balance" ----------------------------------------------------------
# A rigid bar on a fixed flat post top stays put while the centre of mass of the bar and its load lies over the
# post top (|y_G| < w/2 along the bar). Bar middle at e, cubes of mass m at e + y_i.


def com(bar_mass, e, loads):
    return (bar_mass * e + sum(M.CUBE_MASS * (e + y) for y in loads)) / (bar_mass + M.CUBE_MASS * len(loads))


@pytest.mark.parametrize("bar_mass", BAR_MASS_RANGE + (M.BAR_MASS,))
def test_the_empty_bar_balances_with_a_wide_centring_tolerance(bar_mass):
    assert abs(com(bar_mass, E, [])) <= W2 - 0.015  # the scored 10 mm leaves 15 mm to the post's edge


@pytest.mark.parametrize("bar_mass", BAR_MASS_RANGE + (M.BAR_MASS,))
def test_a_cube_on_each_end_stays_balanced_in_the_worst_case(bar_mass):
    # bar 10 mm off one way, both cubes 15 mm off the same way: the centre of mass still 4 mm inside the edge
    worst = com(bar_mass, E, [M.SPOT_D + D, -M.SPOT_D + D])
    assert worst < W2 - 0.004
    nominal = com(bar_mass, 0.0, [M.SPOT_D, -M.SPOT_D])
    assert nominal == pytest.approx(0.0)


@pytest.mark.parametrize("bar_mass", BAR_MASS_RANGE + (M.BAR_MASS,))
def test_one_cube_anywhere_in_an_end_zone_tips_the_bar_wherever_it_balanced(bar_mass):
    inner = M.BAR_L / 2 - M.END_ZONE  # the end zone's inner edge, 100 mm from the middle
    # within the scored centring (bar 10 mm toward the other end): over 10 mm past the edge
    assert com(bar_mass, -E, [inner]) > W2 + 0.010
    # at the demo's spot, even for a bar that balanced anywhere on the post (middle up to w/2 the other way)
    assert com(bar_mass, -W2, [M.SPOT_D]) > W2
    # at the demo's spot with a 15 mm inward error: over 15 mm past the edge
    assert com(bar_mass, -E, [M.SPOT_D - D]) > W2 + 0.015


def test_two_cubes_on_one_end_tip_it_too():
    assert com(M.BAR_MASS, -E, [M.BAR_L / 2 - M.END_ZONE] * 2) > W2


def tipped_angle(e):
    """Angle at which the bar, tipping over the post's edge toward its +y end, rests its end on the table: the
    underside pivots on the edge at w/2, and the end's lower edge is L/2 + e - w/2 from it."""
    return math.asin(M.POST_H / (M.BAR_L / 2 + e - W2))


def test_loading_one_end_first_tips_the_bar_past_where_the_first_cube_can_stay():
    # a 40 mm cube topples off a slope steeper than 45 deg (and slides past atan(1.2) = 50 deg)
    topple = math.atan(1.0)
    assert min(tipped_angle(e) for e in (-E, 0.0, E)) > topple + math.radians(9)
    assert tipped_angle(W2 - 1e-6) > topple  # also for a bar that balanced anywhere on the post
    assert math.atan(GRIPPY_FRICTION[0]) < tipped_angle(E)  # it also slides at the scored centring
    # the end reaches the table before the bar turns past vertical: the post is shorter than the arm of the tip
    assert M.POST_H < M.BAR_L / 2 - W2 - E


def test_without_the_first_cube_falling_a_tipped_bar_would_come_back_level():
    # Why the tall post: on a short post (say 50 mm, 22 deg) a second cube on the raised end would bring the bar
    # back level with the first cube still on it, so loading one end at a time would work. Raised loads shift
    # toward the low side by h*sin(tilt); the loaded bar rights itself while
    # tan(tilt) < (2m + M)(w/2 - e) / (2m*h_cube + M*h_bar), h measured above the bar's underside.
    m, mb = M.CUBE_MASS, M.BAR_MASS
    h_cube, h_bar = M.BAR_T + M.CUBE / 2, M.BAR_T / 2
    rights = (2 * m + mb) * W2 / (2 * m * h_cube + mb * h_bar)
    short = math.asin(0.050 / (M.BAR_L / 2 - W2))
    assert math.tan(short) < rights  # short post: recoverable
    assert math.tan(tipped_angle(0.0)) > 1.0  # this post: the first cube is gone first
    # and even if it stayed on, a second cube on the raised end would not right the bar on this post
    for e in (-E, 0.0, E):
        rights_e = (2 * m + mb) * (W2 - e) / (2 * m * h_cube + mb * h_bar)
        assert math.tan(tipped_angle(e)) > 1.15 * rights_e, e


def test_the_one_end_first_check_differs_from_the_demo_only_in_the_right_arm():
    check = M.one_end_first_demo()
    assert [(w.duration, w.pos, w.finger, w.yaw) for w in check["left"]] == [
        (w.duration, w.pos, w.finger, w.yaw) for w in PLAN["left"]
    ]
    assert {w.pos for w in check["right"]} == {M.PARK["right"]}
    assert sum(w.duration for w in check["right"]) == pytest.approx(sum(w.duration for w in check["left"]))


# -- 5. geometry: grips, clearances, start layout ------------------------------------------------------------


def test_every_grasped_part_fits_the_gripper_with_a_centimetre_to_spare():
    assert M.BAR_W <= M.GRIPPER_MAX_OPENING - 0.010  # the bar is gripped across its 64 mm width
    assert M.CUBE <= M.GRIPPER_MAX_OPENING - 0.010


def test_pad_tips_clear_the_table_and_the_post_top():
    assert M.BAR_PICK_Z - PAD_TIP_PAST_TCP > M.TOP + 0.003
    assert M.CUBE_PICK_Z - PAD_TIP_PAST_TCP > M.TOP + 0.010
    # placing the bar: pad tips above the bar's underside, so they never reach the post top
    tips = M.BAR_RELEASE_Z - PAD_TIP_PAST_TCP
    assert tips - (M.POST_TOP + M.BAR_RELEASE_GAP) > 0.003
    # placing a cube: tips well above the bar's top
    assert M.CUBE_RELEASE_Z - PAD_TIP_PAST_TCP > M.BAR_TOP + 0.015


def test_carried_parts_clear_what_they_pass_over():
    assert M.BAR_CARRY_Z - M.BAR_GRIP_ABOVE_CENTRE - M.BAR_T / 2 >= M.POST_TOP + 0.015 - 1e-9
    assert M.CUBE_CARRY_Z - M.CUBE_GRIP_ABOVE_CENTRE - M.CUBE / 2 >= M.BAR_TOP + 0.015 - 1e-9
    # the left hand moves from the post to its cube with its open pads above the bar
    i = next(k for k, w in enumerate(PLAN["left"]) if w.label == "over the left cube")
    assert PLAN["left"][i - 1].pos[2] - PAD_TIP_PAST_TCP > M.BAR_TOP + 0.015


def test_open_pads_at_a_cube_miss_the_bar_end_and_the_post():
    # cube grips close along y: at the start spots the inner pad is beyond the bar's end and the post
    inner_pad = M.CUBE_Y0 - PAD_OUTER
    assert inner_pad > M.BAR_L / 2 + 0.010 and inner_pad > M.POST_W / 2
    # over the end spot the inner open pad is still over the bar, above it (checked above), not beside the post
    assert M.SPOT_D - PAD_OUTER > M.POST_W / 2


def test_the_place_aims_follow_the_grip_offset_direction():
    # the offset runs toward the gripper's -y; the aim shifts 6 mm toward +y
    assert M.aim(0.0) == pytest.approx((0.0, M.GRIP_AIM))
    assert M.aim(-math.pi / 2) == pytest.approx((M.GRIP_AIM, 0.0))
    for side in ("left", "right"):
        x, y = M.cube_place_xy(side)
        assert abs(abs(y - M.GRIP_AIM) - M.SPOT_D) < 1e-12


@pytest.mark.parametrize("offset", [-0.015, 0.0, 0.015])
def test_a_15_mm_grip_offset_along_the_bar_still_lands_on_the_ends_and_balances(offset):
    # both cube grips close along y, so both offsets run the same way in the world: model that
    bar = bar_on_post(0.005)
    yl, yr = M.cube_place_xy("left")[1] + offset, M.cube_place_xy("right")[1] + offset
    s = scene(bar, on_bar(bar, yl - 0.005), on_bar(bar, yr - 0.005))
    assert TASK.score(s) == 1.0
    assert abs(com(M.BAR_MASS, 0.005, [yl - 0.005, yr - 0.005])) < W2 - 0.008


def test_the_bar_lands_over_the_post_across_its_width_even_with_a_grip_offset():
    # the bar grip closes along x: an offset up to 15 mm (64 mm part) moves the bar across, where the post's
    # 25 mm half-width and the scored 25 mm tolerance both hold it
    px, _ = M.bar_place_xy()
    for off in (-0.015, 0.015):
        assert abs(px + off - M.POST_X) <= M.CENTRE_ACROSS
        assert abs(px + off - M.POST_X) < M.POST_W / 2


def aabb(o):
    p = np.asarray(o.pos, dtype=float)
    h = np.asarray(o.size) / 2
    return p - h, p + h


def test_no_two_objects_overlap_at_the_start():
    boxes = [(o.name, *aabb(o)) for o in M.OBJECTS]
    for i, (a, alo, ahi) in enumerate(boxes):
        for b, blo, bhi in boxes[i + 1:]:
            assert np.any(ahi <= blo) or np.any(bhi <= alo), (a, b)


def test_loose_objects_start_just_above_the_table():
    for o in M.OBJECTS:
        if o.free:
            assert o.pos[2] - o.size[2] / 2 - M.TOP == pytest.approx(M.SPAWN_GAP)


def test_the_bar_runs_along_y_with_one_end_in_front_of_each_arm():
    assert M.BAR.size[1] == max(M.BAR.size) and M.BAR.yaw == 0.0
    assert M.SPOT_D + M.CUBE / 2 <= M.BAR_L / 2  # the spots are on the bar
    assert M.BAR_L / 2 - M.SPOT_D <= M.END_ZONE - 0.020  # and 20 mm or more inside the scored end zone


def test_the_post_is_narrow_against_the_bar():
    assert 6 * M.POST_W <= M.BAR_L + 1e-12
    assert M.POST_W < M.BAR_W  # the bar covers the post top across its width
