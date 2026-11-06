"""Offline checks for move-by-stated-amounts (NumPy and pytest only, no Isaac Sim).

    YAM_ARENA_DIR=/path/to/yam-isaac-lab-arena python -m pytest test_task.py -q

YAM_ARENA_DIR is a checkout of the arena (tested at bd62771); it is not needed when yam_arena is installed.
"""

import itertools
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

for _candidate in (os.environ.get("YAM_ARENA_DIR"), "~/yam-isaac-lab-arena"):
    if _candidate and Path(_candidate).expanduser().is_dir():
        sys.path.insert(0, str(Path(_candidate).expanduser()))
        break

from yam_arena import YAM_URDF, ik, station  # noqa: E402
from yam_arena.analysis import box_signed_distance, link_hulls, world_hulls  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import TABLE_TOP, ObjectState, load  # noqa: E402
from yam_arena.urdf import axis_angle_matrix, forward_kinematics  # noqa: E402

HZ = 20.0  # the rate the task is run at (-E control_hz=20)
TASK_PATH = Path(__file__).resolve().parent / "task.py"
TASK = load(TASK_PATH)
T = TASK.module
TOP = TABLE_TOP["station"]
GRIPPER_MAX_GAP = 0.094  # pads 94 mm apart at FINGER_OPEN (arena tests/test_constants.py)
RUN_LIMIT_STEPS = 2400
NAMES = ("red_bar", "green_bar", "blue_bar")
Z = np.array([0.0, 0.0, 1.0])


# -- helpers ---------------------------------------------------------------------------------------------------------


def rot(yaw=0.0, tilt_deg=0.0, tilt_axis=(1.0, 0.0, 0.0)) -> np.ndarray:
    """Turn ``yaw`` about the vertical after tipping ``tilt_deg`` about a body axis."""
    tip = axis_angle_matrix(np.asarray(tilt_axis, float), math.radians(tilt_deg))
    return axis_angle_matrix(Z, yaw) @ tip


def box_state(pos, size, r=None, vel=(0.0, 0.0, 0.0)) -> ObjectState:
    """An ObjectState as the simulator reports it: world-aligned bounds of the turned box."""
    r = np.eye(3) if r is None else r
    pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    world = corners @ r.T + pos
    return ObjectState(pos, ik.matrix_to_quat_xyzw(r), np.asarray(vel, float), np.zeros(3), world.min(0), world.max(0))


def bar(name, dxy=(0.0, 0.0), turn_deg=0.0, z_low=None, tilt=0.0, tilt_axis=(1.0, 0.0, 0.0), vel=(0, 0, 0)):
    """A bar moved ``dxy`` (m) from its start and turned ``turn_deg`` about the vertical, with its bottom at
    ``z_low`` (default: on the table)."""
    b = T.BARS[name]
    r = rot(b.yaw + math.radians(turn_deg), tilt, tilt_axis)
    half_h = float(np.abs(r @ np.diag(np.asarray(b.size) / 2)).sum(axis=1)[2])
    z_low = TOP if z_low is None else z_low
    return box_state((b.pos[0] + dxy[0], b.pos[1] + dxy[1], z_low + half_h), b.size, r, vel)


def scene(**bars) -> dict:
    """Every bar settled on the table where it starts, unless given."""
    out = {n: bar(n) for n in NAMES}
    out.update(bars)
    return out


def goal(**overrides) -> dict:
    bars = dict(red_bar=bar("red_bar", (0.0, T.SLIDE)), blue_bar=bar("blue_bar", turn_deg=90.0))
    bars.update(overrides)
    return scene(**bars)


# -- loading and description ---------------------------------------------------------------------------------------


def test_the_task_file_loads_with_hooks_and_a_neutral_docstring():
    assert TASK.scene == "station" and TASK.score_over == "end" and TASK.success_hold_s == 1.0
    assert set(TASK.free_objects) == set(NAMES) and len(TASK.objects) == 3
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.docs.startswith("On the station table")
    for word in ("university", "team", "lab ", "dream"):
        assert word not in TASK.docs.lower()


def test_every_bar_is_a_loose_8_cm_square_stick_that_fits_the_gripper_with_room_to_spare():
    for b in T.BARS.values():
        assert b.size == (0.025, 0.080, 0.025) and b.mass == pytest.approx(0.025) and b.yaw == 0.0
    width = T.BAR_SIZE[0]  # gripped across its 25 mm width
    assert width + 0.010 <= GRIPPER_MAX_GAP - 0.010  # 1 cm margin under the 84 mm usable grip
    assert -FINGER_OPEN * 2 == pytest.approx(GRIPPER_MAX_GAP, abs=5e-4)
    approach_gap = -2 * T._finger(T.APPROACH_CLEARANCE)
    assert approach_gap == pytest.approx(0.045) and approach_gap < GRIPPER_MAX_GAP


# -- starting layout -----------------------------------------------------------------------------------------------


def test_bars_start_apart_on_the_table_and_far_enough_apart_for_fingers_and_the_slide():
    starts = scene()
    for a, b in itertools.combinations(NAMES, 2):
        gap = np.maximum(starts[a].low[:2] - starts[b].high[:2], starts[b].low[:2] - starts[a].high[:2]).max()
        # 20 mm of finger clearance plus the 5 cm slide, with room to spare
        assert gap >= 0.020 + T.SLIDE, (a, b, gap)
    tlo = np.array(station.TABLE_CENTER[:2]) - np.array(station.TABLE_SIZE[:2]) / 2
    thi = np.array(station.TABLE_CENTER[:2]) + np.array(station.TABLE_SIZE[:2]) / 2
    for b in T.BARS.values():
        st = box_state(b.pos, b.size, rot(b.yaw))
        assert np.all(st.low[:2] > tlo + 0.1) and np.all(st.high[:2] < thi - 0.1), b.name
        assert 0.0 < st.low[2] - TOP <= 0.002, b.name  # spawned just above the table, settles onto it
    # Red on the left (+y), green on the midline, blue on the right, all in one row.
    assert T.RED.pos[1] > T.GREEN.pos[1] == 0.0 > T.BLUE.pos[1]
    assert T.RED.pos[0] == T.GREEN.pos[0] == T.BLUE.pos[0] == T.ROW_X


def test_the_goal_footprints_stay_clear_of_the_green_bar():
    g = goal()
    for name in ("red_bar", "blue_bar"):
        gap = np.maximum(g[name].low[:2] - g["green_bar"].high[:2], g["green_bar"].low[:2] - g[name].high[:2]).max()
        assert gap > 0.05, name


def test_the_start_and_doing_nothing_score_zero():
    start = scene()
    assert TASK.score(start) == 0.0 and not TASK.success(start)
    assert TASK.distance(start) == pytest.approx(T.SLIDE + 0.1)  # red 5 cm off, blue not turned


# -- the demo --------------------------------------------------------------------------------------------------------


@pytest.fixture(scope="module")
def played():
    """Per arm: the plan, and per control step (time, joints, finger, TCP target pos, TCP target rotation, label)."""
    out = {}
    for side, waypoints in TASK.demo().items():
        arm = ArmPlan(waypoints, station.BASES[side])
        steps = []
        for k in range(round(arm.duration * HZ) + 5):
            t = (k + 1) / HZ
            pos, r, finger, label = arm.target(t)
            action, _ = arm.action7(t)
            steps.append((t, action[:6].copy(), finger, pos, r, label))
        out[side] = (arm, steps)
    return out


def test_the_demo_is_reachable_within_the_joint_limits_at_20_hz(played):
    assert set(played) == {"left", "right"}
    durations = set()
    for side, (arm, steps) in played.items():
        joints = np.array([s[1] for s in steps])
        assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1, side
        assert np.all(joints >= ik.LOWER) and np.all(joints <= ik.UPPER), side
        durations.add(arm.duration)
        assert arm.duration * HZ + 100 < RUN_LIMIT_STEPS / 4
    assert len(durations) == 1  # both arms finish together


def test_the_demo_ends_with_a_still_hold_of_at_least_one_and_a_half_seconds():
    for side, waypoints in TASK.demo().items():
        last, before = waypoints[-1], waypoints[-2]
        assert last.label == "hold still" and last.duration >= 1.5, side
        assert last.pos == before.pos and last.finger == before.finger and last.yaw == before.yaw, side


def test_the_demo_rises_three_centimetres_after_letting_go_before_moving_sideways():
    for side, waypoints in TASK.demo().items():
        labels = [w.label for w in waypoints]
        release, rise = waypoints[labels.index("release")], waypoints[labels.index("rise clear")]
        assert rise.pos[:2] == release.pos[:2] and rise.pos[2] - release.pos[2] >= 0.03 - 1e-9, side
        assert all(w.pos[:2] == release.pos[:2] for w in waypoints[labels.index("release") :]), side


def _arm_hulls(side, joints, finger):
    q = {f"joint{i + 1}": float(v) for i, v in enumerate(joints)}
    q.update(joint7=finger, joint8=finger)
    poses = forward_kinematics(YAM_URDF, q)
    base = np.eye(4)
    base[:3, 3] = station.BASES[side]
    return world_hulls({k: base @ v for k, v in poses.items() if k != "base"})  # the base is bolted to the table


def _hull_bar_clearance(hulls, state, size) -> float:
    r = T._rotation(state)
    best = np.inf
    for _, h in hulls:
        best = min(best, box_signed_distance(h.vertices, np.asarray(state.pos), r, np.asarray(size)).min())
    return float(best)


def test_neither_arm_comes_within_two_centimetres_of_a_bar_it_does_not_handle(played):
    """At every control step: no arm link within 20 mm of the green bar or of the other arm's bar (at its start or
    its goal), and the finger tips stay above the table."""
    assert set(link_hulls()) >= {"gripper", "tip_left", "tip_right"}
    g = goal()
    for side, (_, steps) in played.items():
        handled = next(n for n, s in T.DEMO_ARM.items() if s == side)
        others = [scene()["green_bar"]]
        for n in ("red_bar", "blue_bar"):
            if n != handled:
                others += [scene()[n], g[n]]
        worst = np.inf
        for _, joints, finger, _, _, label in steps:
            hulls = _arm_hulls(side, joints, finger)
            low = min(h.vertices[:, 2].min() for _, h in hulls)
            assert low > TOP + 0.003, (side, label, low - TOP)
            for st in others:
                worst = min(worst, _hull_bar_clearance(hulls, st, T.BAR_SIZE))
        assert worst > 0.020, (side, worst)


def _demo_result(name, offset=0.0, pick_err=(0.0, 0.0), place_err=(0.0, 0.0), yaw_err_deg=0.0):
    """Where the demo leaves a bar, under a simple grip model: closing the fingers pulls the bar ``offset`` off the
    TCP along the closing direction (the first GPU round measured 12-20 mm) and leaves it where it was along the
    other axis; the TCP is off by ``pick_err`` at the grip and ``place_err`` at the release (5 mm at rest)."""
    side = T.DEMO_ARM[name]
    pick_yaw, place_yaw = T.PICK_YAW[side], T.PLACE_YAW[side]
    start = np.asarray(T.BARS[name].pos[:2])
    closing = np.array([-math.sin(pick_yaw), math.cos(pick_yaw)])
    along = np.array([closing[1], -closing[0]])
    tcp_pick = start + np.asarray(pick_err)
    held = offset * closing + float(np.dot(start - tcp_pick, along)) * along  # bar centre minus TCP, at the grip
    turn = place_yaw - pick_yaw + math.radians(yaw_err_deg)
    c, s = math.cos(turn), math.sin(turn)
    final = T.goal_xy(name) + np.asarray(place_err) + np.array([[c, -s], [s, c]]) @ held
    return bar(name, final - start, math.degrees(turn))


def test_the_demo_plan_reaches_full_score_with_a_centred_grip():
    end = scene(red_bar=_demo_result("red_bar"), blue_bar=_demo_result("blue_bar"))
    assert TASK.score(end) == 1.0 and TASK.success(end)


def test_the_demo_still_scores_full_with_an_off_centre_grip_and_wrist_error():
    """The tolerances are wider than the robot's own error: +-15 mm grip offset along the closing direction, 5 mm of
    wrist error in x and y at both the grip and the release, and 5 deg of yaw slip, in every combination."""
    errs = [np.array(e) * 0.005 for e in itertools.product((-1, 1), repeat=2)]
    for name in ("red_bar", "blue_bar"):
        for offset, e1, e2, yaw in itertools.product((-0.015, 0.015), errs, errs, (-5.0, 5.0)):
            end = scene(**{name: _demo_result(name, offset, e1, e2, yaw)})
            part = T.red_score(end) if name == "red_bar" else T.blue_score(end)
            assert part == T.FULL, (name, offset, e1, e2, yaw)


def test_the_demo_moves_red_along_its_length_so_the_grip_offset_does_not_change_the_slide():
    # Fingers close along x for the red bar: the off-centre grip shifts it across the slide, not along it.
    r = _demo_result("red_bar", offset=0.015)
    dx, dy = T.offset(r, "red_bar")
    assert dy == pytest.approx(T.SLIDE) and abs(dx) == pytest.approx(0.015)


# -- scoring: full marks -------------------------------------------------------------------------------------------


def test_the_goal_succeeds_and_scores_one():
    assert TASK.success(goal()) and TASK.score(goal()) == 1.0
    assert TASK.distance(goal()) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("turn", [90.0, -90.0, 270.0, 85.0, -95.0, 81.0])
def test_a_quarter_turn_either_way_counts(turn):
    """+90 (anticlockwise from above) and -90 (clockwise) are both a quarter turn; so is 270, which leaves the same
    bar in the same place."""
    assert TASK.score(goal(blue_bar=bar("blue_bar", turn_deg=turn))) == 1.0


def test_a_bar_rolled_onto_another_long_face_still_lies_flat():
    rolled = goal(red_bar=bar("red_bar", (0.0, T.SLIDE), tilt=90.0, tilt_axis=(0.0, 1.0, 0.0)))
    assert T.lying_flat(rolled["red_bar"]) and TASK.score(rolled) == 1.0


def test_slight_imprecision_inside_the_tolerances_still_counts():
    sloppy = goal(
        red_bar=bar("red_bar", (0.024, 0.036), turn_deg=14.0),
        blue_bar=bar("blue_bar", (0.017, -0.017), turn_deg=-81.0),
        green_bar=bar("green_bar", (0.005, -0.006), turn_deg=7.0),
    )
    assert TASK.score(sloppy) == 1.0


def test_pushing_instead_of_lifting_scores_the_same():
    """Only the final state is scored, so a slid bar and a lifted-and-placed bar at the same spot score the same."""
    assert TASK.score(goal()) == TASK.score(scene(red_bar=_demo_result("red_bar"), blue_bar=_demo_result("blue_bar")))


# -- scoring: partial credit -----------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "red, blue, green, expected",
    [
        (dict(dxy=(0.0, 0.05)), None, None, 0.5),  # red done, blue untouched
        (None, dict(turn_deg=90.0), None, 0.5),  # blue done, red untouched
        (dict(dxy=(0.0, 0.07)), dict(turn_deg=90.0), None, 0.75),  # red 7 cm: 0.25
        (dict(dxy=(0.0, 0.03)), dict(turn_deg=90.0), None, 0.75),  # red 3 cm: 0.25
        (dict(dxy=(0.03, 0.05)), dict(turn_deg=90.0), None, 0.75),  # red drifted 3 cm along x: 0.25
        (dict(dxy=(0.0, 0.05), turn_deg=20.0), dict(turn_deg=90.0), None, 0.75),  # red turned 20 deg: 0.25
        (dict(dxy=(0.0, 0.05)), dict(turn_deg=70.0), None, 0.75),  # blue turned 70 deg: 0.25
        (dict(dxy=(0.0, 0.05)), dict(dxy=(0.03, 0.0), turn_deg=90.0), None, 0.75),  # blue moved 3 cm: 0.25
        (dict(dxy=(0.0, 0.07)), dict(turn_deg=70.0), None, 0.5),  # both 0.25
        (dict(dxy=(0.0, 0.05)), dict(turn_deg=90.0), dict(dxy=(0.012, 0.0)), 0.5),  # green bumped 12 mm: x0.5
        (dict(dxy=(0.0, 0.05)), dict(turn_deg=90.0), dict(turn_deg=15.0), 0.5),  # green turned 15 deg: x0.5
        (dict(dxy=(0.0, 0.07)), dict(turn_deg=90.0), dict(dxy=(0.0, 0.015)), 0.375),  # 0.75 x 0.5
    ],
)
def test_every_partial_credit_branch_returns_its_stated_value(red, blue, green, expected):
    bars = {}
    for name, spec in (("red_bar", red), ("blue_bar", blue), ("green_bar", green)):
        if spec is not None:
            bars[name] = bar(name, **spec)
    objects = scene(**bars)
    assert TASK.score(objects) == pytest.approx(expected)
    assert not TASK.success(objects)


@pytest.mark.parametrize(
    "red, blue",
    [
        (dict(dxy=(0.0, 0.08)), dict(turn_deg=90.0)),  # red 8 cm
        (dict(dxy=(0.0, 0.02)), dict(turn_deg=90.0)),  # red 2 cm
        (dict(dxy=(0.045, 0.05)), dict(turn_deg=90.0)),  # red drifted 4.5 cm along x
        (dict(dxy=(0.0, 0.05), turn_deg=45.0), dict(turn_deg=90.0)),  # red turned 45 deg
        (dict(dxy=(0.0, 0.05)), dict(turn_deg=60.0)),  # blue turned only 60 deg
        (dict(dxy=(0.0, 0.05)), dict(dxy=(0.0, 0.045), turn_deg=90.0)),  # blue turned but moved 4.5 cm
    ],
)
def test_outside_the_partial_band_a_bar_scores_zero(red, blue):
    objects = scene(red_bar=bar("red_bar", **red), blue_bar=bar("blue_bar", **blue))
    assert TASK.score(objects) == pytest.approx(0.5)


def test_a_bar_still_moving_does_not_count():
    moving = goal(red_bar=bar("red_bar", (0.0, T.SLIDE), vel=(0.0, 0.03, 0.0)))
    assert TASK.score(moving) == 0.5 and not TASK.success(moving)


# -- construct validity: shortcuts score nothing -------------------------------------------------------------------


def test_moving_every_bar_the_same_way_scores_zero():
    all_slid = scene(**{n: bar(n, (0.0, T.SLIDE)) for n in NAMES})
    assert TASK.score(all_slid) == 0.0
    all_turned = scene(**{n: bar(n, turn_deg=90.0) for n in NAMES})
    assert TASK.score(all_turned) == 0.0
    all_slid_and_turned = scene(**{n: bar(n, (0.0, T.SLIDE), turn_deg=90.0) for n in NAMES})
    assert TASK.score(all_slid_and_turned) == 0.0


def test_moving_the_wrong_bar_the_right_amount_scores_zero():
    # The green bar slid 5 cm instead of the red one.
    assert TASK.score(scene(green_bar=bar("green_bar", (0.0, T.SLIDE)))) == 0.0
    # The blue bar slid 5 cm and the red one turned: the actions swapped between the bars.
    swapped = scene(red_bar=bar("red_bar", turn_deg=90.0), blue_bar=bar("blue_bar", (0.0, T.SLIDE)))
    assert TASK.score(swapped) == 0.0
    # The green bar turned instead of the blue one (red done right): green breaks the constraint.
    assert TASK.score(scene(red_bar=bar("red_bar", (0.0, T.SLIDE)), green_bar=bar("green_bar", turn_deg=90.0))) == 0.0


def test_the_wrong_direction_scores_zero():
    right = goal(red_bar=bar("red_bar", (0.0, -T.SLIDE)))
    assert T.red_score(right) == 0.0
    forward = goal(red_bar=bar("red_bar", (T.SLIDE, 0.0)))
    assert T.red_score(forward) == 0.0


def test_a_half_turn_is_not_a_quarter_turn():
    assert T.blue_score(goal(blue_bar=bar("blue_bar", turn_deg=180.0))) == 0.0
    assert T.blue_score(goal(blue_bar=bar("blue_bar", turn_deg=45.0))) == 0.0


def test_the_green_bar_moved_or_knocked_over_zeroes_everything():
    for green in (
        bar("green_bar", (0.0, 0.025)),
        bar("green_bar", turn_deg=30.0),
        bar("green_bar", tilt=90.0, tilt_axis=(1.0, 0.0, 0.0)),  # stood on end
        bar("green_bar", tilt=30.0, tilt_axis=(1.0, 0.0, 0.0), z_low=TOP),  # propped up
        bar("green_bar", (0.0, 0.0), z_low=0.0),  # off the table
    ):
        assert TASK.score(goal(green_bar=green)) == 0.0


def test_bars_held_in_the_air_standing_on_end_or_off_the_table_score_zero():
    held = goal(red_bar=bar("red_bar", (0.0, T.SLIDE), z_low=TOP + 0.015), blue_bar=bar("blue_bar", turn_deg=90.0, z_low=TOP + 0.02))
    assert TASK.score(held) == 0.0
    on_end = goal(blue_bar=bar("blue_bar", tilt=90.0, tilt_axis=(1.0, 0.0, 0.0)))
    assert T.blue_score(on_end) == 0.0
    off = goal(red_bar=bar("red_bar", (0.0, T.SLIDE), z_low=0.0))
    assert T.red_score(off) == 0.0
    leaning = goal(red_bar=bar("red_bar", (0.0, T.SLIDE), tilt=20.0, tilt_axis=(1.0, 0.0, 0.0)))
    assert T.red_score(leaning) == 0.0


def test_stacking_a_bar_on_another_scores_zero_for_it():
    on_green = goal(red_bar=bar("red_bar", (0.0, -T.SPACING), z_low=TOP + T.BAR_SIZE[2]))
    assert T.red_score(on_green) == 0.0


def test_score_stays_in_range_and_success_matches_full_score_over_random_states():
    rng = np.random.default_rng(0)
    for _ in range(400):
        objects = scene(
            **{
                n: bar(n, rng.uniform(-0.08, 0.08, 2), rng.uniform(-180, 180), TOP + rng.choice([0.0, rng.uniform(0, 0.05)]), rng.uniform(0, 30))
                for n in NAMES
            }
        )
        s = TASK.score(objects)
        assert s in {0.0, 0.125, 0.25, 0.375, 0.5, 0.75, 1.0}
        assert TASK.success(objects) == (s == 1.0)
        assert TASK.distance(objects) >= 0.0
