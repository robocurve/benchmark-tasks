"""Offline checks for ood-test-scene (NumPy and pytest only, no Isaac Sim).

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
from yam_arena.analysis import link_hulls, world_hulls  # noqa: E402
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
USABLE_GAP = GRIPPER_MAX_GAP - 0.010  # 1 cm margin
RUN_LIMIT_STEPS = 2400
NAMES = T.NAMES
Z = np.array([0.0, 0.0, 1.0])
SIXTHS = {k / 6.0 for k in range(7)}


# -- helpers ---------------------------------------------------------------------------------------------------------


def rot(yaw=0.0, tilt_deg=0.0, tilt_axis=(1.0, 0.0, 0.0)) -> np.ndarray:
    tip = axis_angle_matrix(np.asarray(tilt_axis, float), math.radians(tilt_deg))
    return axis_angle_matrix(Z, yaw) @ tip


def cyl_state(name, xy, z_low=None, tilt=0.0, tilt_axis=(1.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0)) -> ObjectState:
    """An ObjectState for one of the cylinders with its centre over ``xy`` and its lowest point at ``z_low``
    (default: standing on the mat), as the simulator reports it: world-aligned bounds of the tilted cylinder."""
    r_mat = rot(0.0, tilt, tilt_axis)
    a = r_mat[:, 2]
    rad, half_h = T.radius(name), T.height(name) / 2
    half = np.abs(a) * half_h + rad * np.sqrt(np.clip(1.0 - a * a, 0.0, 1.0))
    z_low = T.MAT_TOP if z_low is None else z_low
    pos = np.array([xy[0], xy[1], z_low + half[2]])
    return ObjectState(pos, ik.matrix_to_quat_xyzw(r_mat), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)


def start_state(name) -> ObjectState:
    """Settled on its own outline."""
    return cyl_state(name, T.outline_xy(name), z_low=T.OUTLINE_TOP)


def scene(**placed) -> dict:
    """Every object settled on its own outline, unless given."""
    out = {n: start_state(n) for n in NAMES}
    out.update(placed)
    return out


def at(name, xy, **kw) -> ObjectState:
    return cyl_state(name, xy, **kw)


def goal(**overrides) -> dict:
    placed = {n: at(n, T.GOAL_XY[n]) for n in NAMES}
    placed.update(overrides)
    return scene(**placed)


def away(name, dist, direction=(-1.0, 0.0)) -> tuple[float, float]:
    """A point ``dist`` from the centre of the object's own outline, along ``direction``."""
    d = np.asarray(direction, float) / np.linalg.norm(direction)
    return tuple(T.outline_xy(name) + dist * d)


# -- loading and description ---------------------------------------------------------------------------------------


def test_the_task_file_loads_with_hooks_and_a_neutral_docstring():
    assert TASK.scene == "station" and TASK.score_over == "end" and TASK.success_hold_s == 1.0
    assert set(TASK.free_objects) == set(NAMES) and len(TASK.objects) == 7
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.docs.startswith("On the station table")
    for word in ("university", "team", "lab ", "dream"):
        assert word not in TASK.docs.lower()
    fixed = {o.name for o in TASK.objects if not o.free}
    assert fixed == {"mat", "red_tin_outline", "yellow_jar_outline", "blue_tub_outline"}


def test_every_object_is_an_upright_cylinder_that_fits_the_gripper_with_room_to_spare():
    for n in NAMES:
        width = 2 * T.radius(n)
        assert width + 2 * T.APPROACH_CLEARANCE <= USABLE_GAP, n  # opened 1 cm per side, under the 84 mm usable grip
        assert T._finger(n, T.APPROACH_CLEARANCE) >= FINGER_OPEN, n
        assert T.height(n) / width <= 1.6, n  # squat enough to stand when set down
    assert -FINGER_OPEN * 2 == pytest.approx(GRIPPER_MAX_GAP, abs=5e-4)


def test_the_colours_tell_objects_and_outlines_apart():
    colours = [np.array(T.SPEC[n][3]) for n in NAMES]
    for a, b in itertools.combinations(colours, 2):
        assert np.abs(a - b).sum() > 0.6
    mat = np.array(T.MAT_COLOR)
    for n in NAMES:  # every outline is far lighter than the dark mat
        assert np.array(T.SPEC[n][4]).mean() - mat.mean() > 0.4, n


# -- starting layout -----------------------------------------------------------------------------------------------


def _bbox_gap(a_low, a_high, b_low, b_high) -> float:
    return float(np.maximum(a_low[:2] - b_high[:2], b_low[:2] - a_high[:2]).max())


def test_the_mat_lies_on_the_table_and_the_outlines_on_the_mat():
    tlo = np.array(station.TABLE_CENTER[:2]) - np.array(station.TABLE_SIZE[:2]) / 2
    thi = np.array(station.TABLE_CENTER[:2]) + np.array(station.TABLE_SIZE[:2]) / 2
    assert T.MAT_X0 > tlo[0] + 0.1 and T.MAT_X1 < thi[0] - 0.1 and T.MAT_Y0 > tlo[1] + 0.1 and T.MAT_Y1 < thi[1] - 0.1
    assert T.MAT.pos[2] - T.MAT.size[2] / 2 == pytest.approx(TOP)
    for o in T.OUTLINES:
        x, y = o.pos[:2]
        assert o.pos[2] - o.height / 2 == pytest.approx(T.MAT_TOP)
        assert T.MAT_X0 + o.radius + 0.05 < x < T.MAT_X1 - o.radius - 0.05, o.name
        assert T.MAT_Y0 + o.radius + 0.05 < y < T.MAT_Y1 - o.radius - 0.05, o.name


def test_objects_start_standing_on_their_own_outlines_with_the_border_showing():
    for n in NAMES:
        item = T.ITEMS[n]
        assert item.pos[:2] == tuple(T.outline_xy(n))
        bottom = item.pos[2] - item.height / 2
        assert 0.0 < bottom - T.OUTLINE_TOP <= 0.002, n  # spawned just above its outline, settles onto it
        assert T.outline_radius(n) - item.radius == pytest.approx(0.010)  # a 1 cm ring shows round the base


def test_nothing_overlaps_at_the_start_and_outlines_are_well_apart():
    starts = {n: start_state(n) for n in NAMES}
    for a, b in itertools.combinations(NAMES, 2):
        assert _bbox_gap(starts[a].low, starts[a].high, starts[b].low, starts[b].high) > 0.04, (a, b)
        d = float(np.linalg.norm(T.outline_xy(a) - T.outline_xy(b)))
        assert d - T.outline_radius(a) - T.outline_radius(b) >= 0.05, (a, b)  # 5 cm of mat between outlines


def test_the_start_and_doing_nothing_score_zero():
    start = scene()
    assert TASK.score(start) == 0.0 and not TASK.success(start)
    for n in NAMES:
        assert T.covered(start, n) == [n]  # each object stands on its own outline
    assert TASK.distance(start) == pytest.approx(3 * (T.FULL_SHIFT + 0.05))


# -- the demo --------------------------------------------------------------------------------------------------------


@pytest.fixture(scope="module")
def played():
    """Per arm: the plan, and per control step (time, joints, finger, label)."""
    out = {}
    for side, waypoints in TASK.demo().items():
        arm = ArmPlan(waypoints, station.BASES[side])
        steps = []
        for k in range(round(arm.duration * HZ) + 5):
            t = (k + 1) / HZ
            _, _, finger, label = arm.target(t)
            action, _ = arm.action7(t)
            steps.append((t, action[:6].copy(), finger, label))
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
        assert before.label == "park" and last.pos[:2] == T.PARK_XY[side]


def test_the_demo_rises_three_centimetres_after_letting_go_before_moving_sideways():
    for side, waypoints in TASK.demo().items():
        for i, w in enumerate(waypoints):
            if w.label == "release":
                rise = waypoints[i + 1]
                assert rise.label == "rise clear" and rise.pos[:2] == w.pos[:2], side
                assert rise.pos[2] - w.pos[2] >= 0.03 - 1e-9, side
                assert waypoints[i + 2].pos[:2] == w.pos[:2], side  # straight up again before any sideways move


def test_the_demo_moves_every_object_once_by_the_arm_on_its_side():
    moved = [n for side in ("left", "right") for n in T.DEMO_ORDER[side]]
    assert sorted(moved) == sorted(NAMES)
    for n, side in T.DEMO_ARM.items():
        assert n in T.DEMO_ORDER[side]
        assert T.GOAL_XY[n][1] * (1 if side == "left" else -1) >= 0.0  # never in front of the other arm


def _arm_hulls(side, joints, finger):
    q = {f"joint{i + 1}": float(v) for i, v in enumerate(joints)}
    q.update(joint7=finger, joint8=finger)
    poses = forward_kinematics(YAM_URDF, q)
    base = np.eye(4)
    base[:3, 3] = station.BASES[side]
    return world_hulls({k: base @ v for k, v in poses.items() if k != "base"})


def _cyl_clearance(vertices, name, xy, z_low) -> float:
    """Smallest distance from any vertex to the upright cylinder standing at ``xy`` with its bottom at ``z_low``."""
    radial = np.linalg.norm(vertices[:, :2] - np.asarray(xy), axis=1) - T.radius(name)
    zc = z_low + T.height(name) / 2
    vertical = np.abs(vertices[:, 2] - zc) - T.height(name) / 2
    outside = np.hypot(np.maximum(radial, 0.0), np.maximum(vertical, 0.0))
    inside = np.minimum(np.maximum(radial, vertical), 0.0)
    return float((outside + inside).min())


def _where(name, t):
    """Where an object is at time t of the demo: "start" before its arm's move begins, "hand" during that move (the
    arm that moves it is exempt, as its pads straddle it), "goal" after the move."""
    side = T.DEMO_ARM[name]
    k = T.DEMO_ORDER[side].index(name)
    if t < k * T.MOVE_S:
        return "start"
    return "hand" if t <= (k + 1) * T.MOVE_S else "goal"


def test_no_arm_comes_within_two_centimetres_of_an_object_it_is_not_moving(played):
    """At every control step: no arm link within 20 mm of any object other than the one it is moving, where that
    object is at that moment (its start before its move, its goal after), and every link stays above the mat."""
    assert set(link_hulls()) >= {"gripper", "tip_left", "tip_right"}
    for side, (_, steps) in played.items():
        worst = np.inf
        for t, joints, finger, label in steps:
            hulls = _arm_hulls(side, joints, finger)
            verts = np.vstack([h.vertices for _, h in hulls])
            assert verts[:, 2].min() > T.MAT_TOP + 0.010, (side, label, verts[:, 2].min() - T.MAT_TOP)
            for n in NAMES:
                where = _where(n, t)
                if where == "start":
                    worst = min(worst, _cyl_clearance(verts, n, T.outline_xy(n), T.OUTLINE_TOP))
                elif where == "goal" or T.DEMO_ARM[n] != side:
                    worst = min(worst, _cyl_clearance(verts, n, T.GOAL_XY[n], T.MAT_TOP))
        assert worst > 0.020, (side, worst)


def _boxes(side, joints, finger):
    hulls = _arm_hulls(side, joints, finger)
    return np.array([h.vertices.min(0) for _, h in hulls]), np.array([h.vertices.max(0) for _, h in hulls])


def test_the_two_arms_stay_well_apart(played):
    """Distance between the two arms' link hulls, bounded from below by the distance between their bounding boxes."""
    (_, left), (_, right) = played["left"], played["right"]
    worst = np.inf
    for k in range(0, min(len(left), len(right)), 2):
        llo, lhi = _boxes("left", left[k][1], left[k][2])
        rlo, rhi = _boxes("right", right[k][1], right[k][2])
        gap = np.maximum(np.maximum(llo[:, None, :] - rhi[None, :, :], rlo[None, :, :] - lhi[:, None, :]), 0.0)
        worst = min(worst, float(np.linalg.norm(gap, axis=2).min()))
    assert worst > 0.05, worst


def _demo_result(name, offset=0.0, pick_err=(0.0, 0.0), place_err=(0.0, 0.0), release_push=0.0):
    """Where the demo leaves an object, under a simple grip model: closing the fingers (along y) pulls the object
    ``offset`` off the TCP along y and leaves it where it was along x; the TCP is off by ``pick_err`` at the grip and
    ``place_err`` at the release (5 mm at rest); opening the pads pushes it a further ``release_push`` along +y."""
    start = T.outline_xy(name)
    tcp_pick = start + np.asarray(pick_err)
    held = np.array([start[0] - tcp_pick[0], offset])  # object centre minus TCP while carried
    tcp_place = np.array([T.GOAL_XY[name][0], T.GOAL_XY[name][1] + T.GRIP_SHIFT_Y]) + np.asarray(place_err)
    return at(name, tcp_place + held + np.array([0.0, release_push]))


def test_the_demo_plan_reaches_full_score_with_a_centred_grip():
    end = scene(**{n: _demo_result(n) for n in NAMES})
    assert TASK.score(end) == 1.0 and TASK.success(end)


def test_the_demo_still_scores_full_with_an_off_centre_grip_and_wrist_error():
    """+-20 mm grip offset along the closing direction, 5 mm of wrist error in x and y at the grip and the release,
    and a 0-10 mm push toward +y as the pads open, in every combination, for every object."""
    errs = [np.array(e) * 0.005 for e in itertools.product((-1, 1), repeat=2)]
    for n in NAMES:
        worst_shift = np.inf
        for offset, e1, e2, push in itertools.product((-0.020, 0.020), errs, errs, (0.0, 0.010)):
            end = scene(**{n: _demo_result(n, offset, e1, e2, push)})
            assert T.object_sixths(end, n) == 2, (n, offset, e1, e2, push)
            worst_shift = min(worst_shift, T.shift(end, n))
        assert worst_shift >= T.FULL_SHIFT + 0.02, (n, worst_shift)  # at least 2 cm to spare


def test_the_demo_goals_keep_margins_to_the_mat_edges_and_every_outline():
    for n in NAMES:
        x, y = T.GOAL_XY[n]
        r = T.radius(n)
        edge = min(x - r - T.MAT_X0, T.MAT_X1 - r - x, y - r - T.MAT_Y0, T.MAT_Y1 - r - y)
        assert edge >= 0.020, (n, edge)
        for o in NAMES:
            gap = float(np.linalg.norm(np.asarray(T.GOAL_XY[n]) - T.outline_xy(o))) - r - T.outline_radius(o)
            assert gap >= 0.030, (n, o, gap)
    for a, b in itertools.combinations(NAMES, 2):
        gap = float(np.linalg.norm(np.subtract(T.GOAL_XY[a], T.GOAL_XY[b]))) - T.radius(a) - T.radius(b)
        assert gap >= 0.05, (a, b, gap)


# -- scoring: full marks -------------------------------------------------------------------------------------------


def test_the_goal_succeeds_and_scores_one():
    assert TASK.success(goal()) and TASK.score(goal()) == 1.0
    assert TASK.distance(goal()) == pytest.approx(0.0, abs=1e-9)


def test_other_layouts_also_score_full():
    """The demo's layout is one answer of many: behind the outlines, at the sides, or mixed."""
    layouts = [
        dict(red_tin=(0.42, 0.17), yellow_jar=(0.20, 0.0), blue_tub=(0.42, -0.17)),
        dict(red_tin=(0.18, 0.05), yellow_jar=(0.43, 0.12), blue_tub=(0.18, -0.05)),
        dict(red_tin=(0.19, 0.19), yellow_jar=(0.43, -0.12), blue_tub=(0.17, -0.17)),
    ]
    for layout in layouts:
        objects = scene(**{n: at(n, xy) for n, xy in layout.items()})
        assert TASK.score(objects) == 1.0 and TASK.success(objects), layout


def test_either_flat_end_down_counts_as_upright_and_slight_tilt_is_tolerated():
    flipped = goal(red_tin=at("red_tin", T.GOAL_XY["red_tin"], tilt=180.0))
    assert TASK.score(flipped) == 1.0
    leaning = goal(blue_tub=at("blue_tub", T.GOAL_XY["blue_tub"], tilt=8.0))
    assert TASK.score(leaning) == 1.0


def test_exactly_ten_centimetres_counts():
    objects = goal(yellow_jar=at("yellow_jar", away("yellow_jar", 0.1001)))
    assert T.object_sixths(objects, "yellow_jar") == 2


# -- scoring: partial credit -----------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "placed, expected",
    [
        (dict(red_tin=T.GOAL_XY["red_tin"]), 2 / 6),  # one object done, the others untouched
        (dict(red_tin=T.GOAL_XY["red_tin"], blue_tub=T.GOAL_XY["blue_tub"]), 4 / 6),  # two done
        (dict(red_tin=away("red_tin", 0.08), yellow_jar=T.GOAL_XY["yellow_jar"], blue_tub=T.GOAL_XY["blue_tub"]), 5 / 6),
        (dict(red_tin=away("red_tin", 0.07), yellow_jar=away("yellow_jar", 0.06), blue_tub=away("blue_tub", 0.09)), 3 / 6),
        (dict(yellow_jar=away("yellow_jar", 0.055)), 1 / 6),  # jar 5.5 cm toward the arms, clear of its outline
        (dict(red_tin=away("red_tin", 0.07), yellow_jar=T.GOAL_XY["yellow_jar"]), 3 / 6),
    ],
)
def test_every_partial_credit_branch_returns_its_stated_value(placed, expected):
    objects = scene(**{n: at(n, xy) for n, xy in placed.items()})
    assert TASK.score(objects) == pytest.approx(expected)
    assert not TASK.success(objects)


def test_partial_band_needs_the_object_clear_of_its_own_outline():
    """Between 5 and 10 cm the object still has to leave its own outline uncovered: the red tin's base overlaps
    its outline until its centre is 6.4 cm away (27 mm + 37 mm), so 6 cm scores 0 and 6.5 cm scores 1/6."""
    assert T.object_sixths(scene(red_tin=at("red_tin", away("red_tin", 0.060))), "red_tin") == 0
    assert T.object_sixths(scene(red_tin=at("red_tin", away("red_tin", 0.065))), "red_tin") == 1


def test_a_moving_object_does_not_count():
    moving = goal(red_tin=at("red_tin", T.GOAL_XY["red_tin"], vel=(0.0, 0.03, 0.0)))
    assert TASK.score(moving) == pytest.approx(4 / 6) and not TASK.success(moving)


# -- construct validity: shortcuts score nothing -------------------------------------------------------------------


def test_moving_objects_under_five_centimetres_scores_zero():
    nudged = scene(**{n: at(n, away(n, 0.045)) for n in NAMES})
    assert TASK.score(nudged) == 0.0


def test_swapping_objects_onto_each_others_outlines_scores_zero():
    """Every object ends far from its own outline, but on another one: the training layout with labels swapped."""
    swap = scene(
        red_tin=at("red_tin", T.outline_xy("blue_tub")),
        blue_tub=at("blue_tub", T.outline_xy("yellow_jar")),
        yellow_jar=at("yellow_jar", T.outline_xy("red_tin")),
    )
    for n in NAMES:
        assert T.shift(swap, n) >= T.FULL_SHIFT and T.covered(swap, n)
    assert TASK.score(swap) == 0.0


def test_partly_covering_another_outline_scores_zero_for_that_object():
    # The yellow jar 12 cm from its own outline, but its base over the edge of the red tin's (empty) outline.
    xy = (T.outline_xy("red_tin")[0] - 0.04, T.outline_xy("red_tin")[1] - 0.04)
    objects = goal(yellow_jar=at("yellow_jar", xy))
    assert T.shift(objects, "yellow_jar") >= T.FULL_SHIFT and T.covered(objects, "yellow_jar") == ["red_tin"]
    assert TASK.score(objects) == pytest.approx(4 / 6)


def test_putting_objects_off_the_mat_scores_zero():
    off_mat = scene(
        red_tin=at("red_tin", (0.30, 0.30), z_low=TOP),
        yellow_jar=at("yellow_jar", (0.52, 0.0), z_low=TOP),
        blue_tub=at("blue_tub", (0.30, -0.30), z_low=TOP),
    )
    assert TASK.score(off_mat) == 0.0
    # Overhanging the mat's edge (centre 1 cm inside it) also fails.
    overhang = goal(red_tin=at("red_tin", (0.20, T.MAT_Y1 - 0.010)))
    assert T.object_sixths(overhang, "red_tin") == 0


def test_knocking_objects_over_scores_zero_for_them():
    lying = goal(
        red_tin=at("red_tin", T.GOAL_XY["red_tin"], tilt=90.0),
        blue_tub=at("blue_tub", T.GOAL_XY["blue_tub"], tilt=90.0, tilt_axis=(0.0, 1.0, 0.0)),
    )
    assert TASK.score(lying) == pytest.approx(2 / 6)
    tipped = goal(yellow_jar=at("yellow_jar", T.GOAL_XY["yellow_jar"], tilt=25.0))
    assert TASK.score(tipped) == pytest.approx(4 / 6)


def test_stacking_or_holding_an_object_scores_zero_for_it():
    red_xy = T.GOAL_XY["red_tin"]
    stacked = goal(yellow_jar=at("yellow_jar", red_xy, z_low=T.MAT_TOP + T.height("red_tin")))
    assert T.object_sixths(stacked, "yellow_jar") == 0
    held = goal(blue_tub=at("blue_tub", T.GOAL_XY["blue_tub"], z_low=T.MAT_TOP + 0.02))
    assert T.object_sixths(held, "blue_tub") == 0


def test_an_object_on_the_floor_zeroes_everything():
    fallen = goal(yellow_jar=at("yellow_jar", (0.60, 0.0), z_low=station.FLOOR_Z, tilt=90.0))
    assert TASK.score(fallen) == 0.0 and not TASK.success(fallen)


def test_score_stays_in_sixths_and_success_matches_full_score_over_random_states():
    rng = np.random.default_rng(0)
    for _ in range(500):
        placed = {}
        for n in NAMES:
            xy = rng.uniform((0.05, -0.32), (0.55, 0.32))
            z_low = rng.choice([T.MAT_TOP, T.OUTLINE_TOP, TOP, T.MAT_TOP + rng.uniform(0.0, 0.08), station.FLOOR_Z])
            placed[n] = at(n, xy, z_low=z_low, tilt=rng.choice([0.0, rng.uniform(0, 90)]))
        objects = scene(**placed)
        s = TASK.score(objects)
        assert any(s == pytest.approx(v) for v in SIXTHS)
        assert TASK.success(objects) == (s == 1.0)
        assert TASK.distance(objects) >= 0.0
