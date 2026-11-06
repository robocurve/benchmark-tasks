"""Offline checks for ring-around-post (NumPy and pytest only, no Isaac Sim).

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
from yam_arena.analysis import box_signed_distance, hull_set_distance, world_hulls  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Cylinder, ObjectState, load  # noqa: E402
from yam_arena.urdf import axis_angle_matrix, forward_kinematics  # noqa: E402

HZ = 20.0  # the rate the task is run at (-E control_hz=20)
TASK_PATH = Path(__file__).resolve().parent / "task.py"
TASK = load(TASK_PATH)
T = TASK.module
TOP = TABLE_TOP["station"]
GRIPPER_MAX_GAP = 0.094  # pads 94 mm apart at FINGER_OPEN (arena tests/test_constants.py)
RUN_LIMIT_STEPS = 3600
NAMES = tuple(T.BLOCKS)
CUBE_SIZE = (T.CUBE, T.CUBE, T.CUBE)
Z = np.array([0.0, 0.0, 1.0])
POST = np.asarray(T.POST_XY, dtype=float)


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


def cube(xy, yaw_deg=0.0, z_low=None, tilt=0.0, tilt_axis=(1.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0)) -> ObjectState:
    """A cube with its centre over ``xy`` and its bottom at ``z_low`` (default: on the table)."""
    r = rot(math.radians(yaw_deg), tilt, tilt_axis)
    half_h = float(np.abs(r @ np.diag(np.asarray(CUBE_SIZE) / 2)).sum(axis=1)[2])
    z_low = TOP if z_low is None else z_low
    return box_state((xy[0], xy[1], z_low + half_h), CUBE_SIZE, r, vel)


def polar(radius_m, angle_deg) -> np.ndarray:
    a = math.radians(angle_deg)
    return POST + radius_m * np.array([math.cos(a), math.sin(a)])


def post_state() -> ObjectState:
    p = T.POST
    lo = np.array([p.pos[0] - p.radius, p.pos[1] - p.radius, TOP])
    hi = np.array([p.pos[0] + p.radius, p.pos[1] + p.radius, TOP + p.height])
    return ObjectState(np.asarray(p.pos, float), np.array([0.0, 0.0, 0.0, 1.0]), np.zeros(3), np.zeros(3), lo, hi)


def start_state(name) -> ObjectState:
    b = T.BLOCKS[name]
    return cube(b.pos[:2])


def scene(**blocks) -> dict:
    """The post, and every block settled where it starts in the row unless given."""
    out = {"post": post_state()}
    out.update({n: start_state(n) for n in NAMES})
    out.update(blocks)
    return out


def ring(angles, radius_m=T.RING_R, names=NAMES, **kw) -> dict:
    """Blocks at ``angles`` (deg) round the post, ``radius_m`` from its axis; blocks without an angle stay in the
    row."""
    return {n: cube(polar(radius_m, a), **kw) for n, a in zip(names, angles)}


HEXAGON = (30.0, 90.0, 150.0, 210.0, 270.0, 330.0)


def goal(**overrides) -> dict:
    blocks = ring(HEXAGON)
    blocks.update(overrides)
    return scene(**blocks)


# -- loading and description ---------------------------------------------------------------------------------------


def test_the_task_file_loads_with_hooks_and_a_neutral_docstring():
    assert TASK.scene == "station" and TASK.score_over == "end" and TASK.success_hold_s == 1.0
    assert set(TASK.free_objects) == set(NAMES) and len(NAMES) == 6 and len(TASK.objects) == 7
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.docs.startswith("On the station table")
    for word in ("university", "team", "lab ", "dream"):
        assert word not in TASK.docs.lower()


def test_the_post_is_a_fixed_30_mm_dowel_and_every_block_a_loose_30_mm_cube_that_fits_the_gripper():
    assert isinstance(T.POST, Cylinder) and not T.POST.free
    assert T.POST.radius == pytest.approx(0.015) and T.POST.height == pytest.approx(0.060)
    assert T.POST.pos[2] == pytest.approx(TOP + T.POST.height / 2)  # stands on the table
    for b in T.BLOCKS.values():
        assert b.size == (0.030, 0.030, 0.030) and b.mass == pytest.approx(0.014) and b.yaw == 0.0 and b.free
        assert b.color == T.BLUE  # six identical blocks: any block may go anywhere
    assert T.CUBE + 0.010 <= GRIPPER_MAX_GAP - 0.010  # 1 cm margin under the 84 mm usable grip
    assert -FINGER_OPEN * 2 == pytest.approx(GRIPPER_MAX_GAP, abs=5e-4)
    approach_gap = -2 * T._finger(T.APPROACH_CLEARANCE)
    assert approach_gap == pytest.approx(0.054) and approach_gap < GRIPPER_MAX_GAP  # 12 mm a side (GPU round 3)


# -- starting layout -----------------------------------------------------------------------------------------------


def test_blocks_start_apart_in_a_row_on_the_table_far_from_the_ring():
    starts = scene()
    for a, b in itertools.combinations(NAMES, 2):
        gap = np.maximum(starts[a].low[:2] - starts[b].high[:2], starts[b].low[:2] - starts[a].high[:2]).max()
        assert gap >= 0.050 - 1e-9, (a, b, gap)  # room for a pad (about 8 mm thick) plus 8 mm clearance, twice over
    tlo = np.array(station.TABLE_CENTER[:2]) - np.array(station.TABLE_SIZE[:2]) / 2
    thi = np.array(station.TABLE_CENTER[:2]) + np.array(station.TABLE_SIZE[:2]) / 2
    for name in NAMES:
        b = T.BLOCKS[name]
        st = box_state(b.pos, b.size)
        assert np.all(st.low[:2] > tlo + 0.1) and np.all(st.high[:2] < thi - 0.1), name
        assert 0.0 < st.low[2] - TOP <= 0.002, name  # spawned just above the table, settles onto it
        # every start is outside the ring band, so a block left in the row can never count
        assert T.radius(starts[name]) > T.RING_R + T.RING_TOL + 0.04, name
    assert all(b.pos[0] == T.STAGE_X for b in T.BLOCKS.values())
    # three on each side of the midline, so each arm has its own three
    assert sum(b.pos[1] > 0 for b in T.BLOCKS.values()) == 3 and min(abs(b.pos[1]) for b in T.BLOCKS.values()) >= 0.12


def test_the_post_stands_on_the_midline_where_both_halves_of_the_ring_are_in_reach():
    assert T.POST_XY == (0.30, 0.0)
    st = post_state()
    for name in NAMES:
        s = start_state(name)
        gap = np.maximum(s.low[:2] - st.high[:2], st.low[:2] - s.high[:2]).max()
        assert gap > 0.10, name


def test_the_start_and_doing_nothing_score_zero():
    start = scene()
    assert TASK.score(start) == 0.0 and not TASK.success(start)
    assert T.ring_gaps(start) == {}
    assert TASK.distance(start) > 0.4


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
    assert len(durations) == 1 and durations.pop() == pytest.approx(38.2)  # both arms finish together


def test_each_arm_serves_its_own_half_of_the_ring_and_the_plan_covers_every_block_once():
    seen = []
    for side, steps in T.DEMO_PLAN.items():
        for name, angle, pick_yaw, place_yaw in steps:
            seen.append(name)
            target = T.slot_xy(angle)
            # the left arm works only on its own side (y > 0), the right arm on the other
            assert (target[1] > 0.04) == (side == "left") and (T.BLOCKS[name].pos[1] > 0) == (side == "left")
            # at the pick the pads meet two faces of a square cube; over the slot they close along the radius
            assert pick_yaw % 90.0 == 0.0 and abs(place_yaw - pick_yaw) <= 45.0
            radial = (target - POST) / np.linalg.norm(target - POST)
            assert abs(float(np.dot(T.closing_direction(math.radians(place_yaw)), radial))) == pytest.approx(1.0)
    assert sorted(seen) == sorted(NAMES)
    slots = sorted(a for steps in T.DEMO_PLAN.values() for _, a, _, _ in steps)
    assert slots == sorted(HEXAGON)


def test_the_demo_ends_parked_clear_of_the_ring_with_a_still_hold_of_at_least_one_and_a_half_seconds():
    for side, waypoints in TASK.demo().items():
        last, before = waypoints[-1], waypoints[-2]
        assert last.label == "hold still" and last.duration >= 1.5, side
        assert last.pos == before.pos and last.finger == before.finger and last.yaw == before.yaw, side
        assert np.linalg.norm(np.asarray(last.pos[:2]) - POST) > T.RING_R + 0.08, side


def test_the_demo_rises_three_centimetres_after_every_release_before_moving_sideways():
    for side, waypoints in TASK.demo().items():
        labels = [w.label for w in waypoints]
        releases = [i for i, lab in enumerate(labels) if lab.endswith(": release")]
        assert len(releases) == 3, side
        for i in releases:
            release, rise, up = waypoints[i], waypoints[i + 1], waypoints[i + 2]
            assert rise.label.endswith("rise clear") and rise.pos[:2] == release.pos[:2]
            assert rise.pos[2] - release.pos[2] >= 0.03 - 1e-9 and up.pos[:2] == release.pos[:2]
            # let go 3 mm above the table: the block's bottom is GRIP_Z - 20 mm below the TCP
            assert release.pos[2] - (T.GRIP_Z - TOP) == pytest.approx(TOP + T.RELEASE_GAP)


def _arm_hulls(side, joints, finger):
    q = {f"joint{i + 1}": float(v) for i, v in enumerate(joints)}
    q.update(joint7=finger, joint8=finger)
    poses = forward_kinematics(YAM_URDF, q)
    base = np.eye(4)
    base[:3, 3] = station.BASES[side]
    return world_hulls({k: base @ v for k, v in poses.items() if k != "base"})  # the base is bolted to the table


def _clearance_to_box(hulls, center, size, r=np.eye(3)) -> float:
    return float(min(box_signed_distance(h.vertices, np.asarray(center), r, np.asarray(size)).min() for _, h in hulls))


def _block_timeline():
    """For each block: (time it leaves the table at the start of its lift, time it is let go at its slot)."""
    out = {}
    for side, waypoints in TASK.demo().items():
        t = 0.0
        for w in waypoints:
            name, _, phase = w.label.partition(": ")
            if phase == "lift":
                out.setdefault(name, [None, None])[0] = t
            t += w.duration
            if phase == "release":
                out[name][1] = t
    return out


def _handling(label):
    """The block an arm is working on right now (from descending onto it to going straight back up from it after
    the release, where the pad tips start 1 cm over its top), or None."""
    name, _, phase = label.partition(": ")
    return name if phase in {"descend", "close", "lift", "carry", "lower", "release", "rise clear", "up"} else None


def test_neither_arm_touches_the_post_the_table_another_block_or_the_other_arm(played):
    """At every control step: no arm link inside 20 mm of the post (its bounding box), the finger tips above the
    table, no link within 15 mm of any block the arm is not working on (in the row before its pick, at its slot
    after its release), and the two arms at least 5 cm apart."""
    timeline = _block_timeline()
    goal_xy = {name: T.slot_xy(a) for steps in T.DEMO_PLAN.values() for name, a, _, _ in steps}
    post_c = np.array([*T.POST_XY, TOP + T.POST_HEIGHT / 2])
    post_size = (2 * T.POST_RADIUS, 2 * T.POST_RADIUS, T.POST_HEIGHT)
    (_, left), (_, right) = played["left"], played["right"]
    worst = dict(post=np.inf, block=np.inf, arms=np.inf, table=np.inf)
    for k, (l_step, r_step) in enumerate(zip(left, right)):
        hulls = {}
        for side, (t, joints, finger, label) in (("left", l_step), ("right", r_step)):
            h = hulls[side] = _arm_hulls(side, joints, finger)
            worst["table"] = min(worst["table"], min(x.vertices[:, 2].min() for _, x in h) - TOP)
            worst["post"] = min(worst["post"], _clearance_to_box(h, post_c, post_size))
            if k % 2:  # blocks every other step (10 Hz) to keep the test quick
                continue
            mine = _handling(label)
            for name in NAMES:
                lift, let_go = timeline[name]
                if name == mine or lift <= t < let_go:
                    continue
                xy = T.BLOCKS[name].pos[:2] if t < lift else goal_xy[name]
                worst["block"] = min(worst["block"], _clearance_to_box(h, (*xy, TOP + T.CUBE / 2), CUBE_SIZE))
        if k % 4 == 0:
            worst["arms"] = min(worst["arms"], hull_set_distance(hulls["left"], hulls["right"])[0])
    assert worst["table"] > 0.004, worst
    assert worst["post"] > 0.020, worst
    assert worst["block"] > 0.015, worst
    assert worst["arms"] > 0.050, worst


def _demo_result(name, angle, pick_yaw_deg, place_yaw_deg, offset=0.0, pick_err=(0.0, 0.0), place_err=(0.0, 0.0)):
    """Where the demo leaves a block (xy), under a simple grip model: closing the fingers pulls the block ``offset``
    off the TCP along the closing direction and leaves it where it was along the other axis; the TCP is off by
    ``pick_err`` at the grip and ``place_err`` at the release; turning the grip turns the held offset with it."""
    y0, y1 = math.radians(pick_yaw_deg), math.radians(place_yaw_deg)
    start = np.asarray(T.BLOCKS[name].pos[:2])
    closing = T.closing_direction(y0)
    along = np.array([closing[1], -closing[0]])
    held = offset * closing + float(np.dot(start - (start + np.asarray(pick_err)), along)) * along
    c, s = math.cos(y1 - y0), math.sin(y1 - y0)
    return T.slot_xy(angle) + np.asarray(place_err) + np.array([[c, -s], [s, c]]) @ held


def _demo_steps():
    return [step for steps in T.DEMO_PLAN.values() for step in steps]


def test_the_demo_plan_reaches_full_score_with_a_centred_grip():
    end = scene(**{s[0]: cube(_demo_result(*s)) for s in _demo_steps()})
    assert TASK.score(end) == 1.0 and TASK.success(end)


def test_the_demo_still_scores_full_with_an_off_centre_grip_and_wrist_error():
    """Every combination of a +-15 mm off-centre grip along the closing direction and up to 3 mm of TCP error in x
    and y at the grip and at the release, chosen separately for every block (the worst case is adversarial: each
    block's error and its neighbours' errors pull its gaps apart). Radius: each block alone; gaps: each pair of
    neighbours over all of both blocks' combinations."""
    errs = [np.array(e) * 0.003 for e in itertools.product((-1, 0, 1), repeat=2)]
    finals = {}
    for step in _demo_steps():
        pts = [_demo_result(*step, off, e1, e2) for off in (-0.015, 0.015) for e1 in errs for e2 in errs]
        finals[step[0]] = (step[1], np.array(pts))
    worst_r = max(float(np.abs(np.linalg.norm(p - POST, axis=1) - T.RING_R).max()) for _, p in finals.values())
    assert worst_r <= T.RING_TOL, worst_r
    order = [n for _, n in sorted((a, n) for n, (a, _) in finals.items())]
    worst_gap = 0.0
    for a, b in zip(order, order[1:] + order[:1]):
        ang_a = np.degrees(np.arctan2(*(finals[a][1] - POST).T[::-1]))
        ang_b = np.degrees(np.arctan2(*(finals[b][1] - POST).T[::-1]))
        gaps = (ang_b[None, :] - ang_a[:, None]) % 360.0
        worst_gap = max(worst_gap, float(np.abs(gaps - T.GAP_DEG).max()))
    assert worst_gap <= T.GAP_TOL_DEG, worst_gap
    # one corner of the sweep run through the scorer itself
    end = scene(**{s[0]: cube(_demo_result(*s, 0.015, (0.003, -0.003), (-0.003, 0.003))) for s in _demo_steps()})
    assert TASK.score(end) == 1.0


def test_the_grip_offset_moves_a_block_along_the_radius_not_round_the_ring():
    for step in _demo_steps():
        centred, shifted = _demo_result(*step), _demo_result(*step, offset=0.015)
        assert abs(np.linalg.norm(shifted - POST) - np.linalg.norm(centred - POST)) == pytest.approx(0.015, abs=1e-9)
        a0 = math.atan2(*(centred - POST)[::-1])
        a1 = math.atan2(*(shifted - POST)[::-1])
        assert abs(a1 - a0) < 1e-9, step[0]


# -- scoring: full marks -------------------------------------------------------------------------------------------


def test_the_goal_succeeds_and_scores_one():
    assert TASK.success(goal()) and TASK.score(goal()) == 1.0
    assert TASK.distance(goal()) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("turn", [0.0, 17.0, 30.0, 45.0, -73.0, 180.0])
def test_any_rotation_of_the_ring_and_any_order_of_the_blocks_counts(turn):
    """The instruction fixes no starting angle and the blocks are identical, so any rotation and any assignment of
    blocks to places scores the same."""
    angles = [a + turn for a in HEXAGON]
    shuffled = (NAMES[3], NAMES[0], NAMES[5], NAMES[1], NAMES[4], NAMES[2])
    assert TASK.score(scene(**ring(angles, names=shuffled))) == 1.0


def test_slight_imprecision_inside_the_tolerances_still_counts():
    # radii 81-119 mm, gaps 51-69 deg, blocks turned any way about the vertical
    angles = (0.0, 51.0, 115.0, 175.0, 240.0, 299.0)
    radii = (0.081, 0.119, 0.100, 0.090, 0.110, 0.095)
    blocks = {n: cube(polar(r, a), yaw_deg=13.0 * i) for i, (n, a, r) in enumerate(zip(NAMES, angles, radii))}
    objects = scene(**blocks)
    assert TASK.score(objects) == 1.0 and TASK.success(objects)


def test_a_block_on_another_face_still_counts():
    tipped = goal(block_1=cube(polar(T.RING_R, HEXAGON[0]), tilt=90.0, tilt_axis=(1.0, 0.0, 0.0)))
    assert T.flat_on_table(tipped["block_1"]) and TASK.score(tipped) == 1.0


# -- scoring: partial credit -----------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "angles, expected",
    [
        (HEXAGON[:5], 3),  # five in place, one left in the row: the two beside the empty slot face a 120 deg gap
        (HEXAGON[:4], 2),  # four in a run
        (HEXAGON[:3], 1),  # three in a run: only the middle one has a neighbour 60 deg away on both sides
        (HEXAGON[:2], 0),  # two: each has 300 deg on one side
        ((30.0, 90.0, 210.0, 270.0), 0),  # four, two pairs facing each other: every block has a 120 deg gap
        ((0.0, 75.0, 120.0, 180.0, 240.0, 300.0), 3),  # one block 15 deg off: it and both neighbours lose credit
        ((0.0, 60.0, 120.0, 180.0, 240.0, 285.0), 3),
    ],
)
def test_every_partial_credit_branch_returns_its_stated_value(angles, expected):
    objects = scene(**ring(angles))
    assert len(T.placed(objects)) == expected
    assert TASK.score(objects) == pytest.approx(expected / 6)
    assert not TASK.success(objects)


@pytest.mark.parametrize("bad_radius", [0.079, 0.121, 0.150, 0.060])
def test_a_block_outside_the_radius_band_drops_out_of_the_ring(bad_radius):
    """The block is left out entirely: its two neighbours now face a 120 deg gap, so three blocks remain placed."""
    objects = goal(block_1=cube(polar(bad_radius, HEXAGON[0])))
    assert "block_1" not in T.ring_gaps(objects)
    assert TASK.score(objects) == pytest.approx(3 / 6)


def test_a_block_still_moving_does_not_count():
    moving = goal(block_2=cube(polar(T.RING_R, HEXAGON[1]), vel=(0.0, 0.03, 0.0)))
    assert TASK.score(moving) == pytest.approx(3 / 6) and not TASK.success(moving)


def test_distance_shrinks_as_the_ring_takes_shape():
    d = [TASK.distance(scene(**ring(HEXAGON[:k]))) for k in range(7)]
    assert all(a > b for a, b in zip(d, d[1:])), d


# -- construct validity: shortcuts score nothing -------------------------------------------------------------------


@pytest.mark.parametrize("centre_angle", [0.0, 45.0, 90.0, 200.0])
def test_a_pile_of_blocks_at_the_right_distance_scores_zero(centre_angle):
    """Six blocks pushed together into a 2 x 3 heap whose centre is 10 cm from the post: several sit inside the
    radius band, but neighbours are a few degrees apart."""
    c = polar(T.RING_R, centre_angle)
    offsets = [np.array([dx, dy]) for dx in (-0.015, 0.015) for dy in (-0.031, 0.0, 0.031)]
    objects = scene(**{n: cube(c + o) for n, o in zip(NAMES, offsets)})
    assert len(T.ring_gaps(objects)) >= 2
    assert TASK.score(objects) == 0.0


def test_a_stacked_tower_at_a_slot_counts_only_its_bottom_block():
    tower = {n: cube(polar(T.RING_R, 30.0), z_low=TOP + i * T.CUBE) for i, n in enumerate(NAMES)}
    assert TASK.score(scene(**tower)) == 0.0
    # the full ring with an extra block stacked on one of them: the ring still has six blocks, so stacking the
    # sixth instead of placing it loses the credit it would have earned
    five_and_one_on_top = goal(block_6=cube(polar(T.RING_R, HEXAGON[0]), z_low=TOP + T.CUBE))
    assert TASK.score(five_and_one_on_top) == pytest.approx(3 / 6)


@pytest.mark.parametrize("dist_mm", [0, 40, 70, 82, 86.6, 90, 100, 110])
@pytest.mark.parametrize("spacing_mm", [32, 40, 50, 60, 70])
def test_a_straight_row_of_six_scores_zero(dist_mm, spacing_mm):
    """Six blocks in a straight line passing ``dist_mm`` from the post's axis (a line through the post goes round
    it: the two middle blocks sit either side), with ``spacing_mm`` between centres."""
    foot = POST + np.array([-dist_mm / 1000, 0.0])
    s = (np.arange(6) - 2.5) * spacing_mm / 1000
    if dist_mm < 30:
        s = s + np.sign(s) * 0.03  # step round the post
    blocks = {n: cube(foot + np.array([0.0, v])) for n, v in zip(NAMES, s)}
    assert TASK.score(scene(**blocks)) == 0.0


def test_no_three_blocks_in_a_straight_line_can_earn_anything():
    """Exhaustive over straight lines passing 0-125 mm from the post's axis, with blocks at least 30 mm apart along
    the line on a 3 mm grid. A placed block needs both its neighbours round the post 50-70 deg away; in any set of
    blocks, a placed block and its two neighbours alone still give it those same two gaps, so if no collinear
    triple can do it, no straight row of any length can."""
    lo, hi = T.RING_R - T.RING_TOL, T.RING_R + T.RING_TOL
    best = 0
    for d in np.arange(0.0, hi + 0.006, 0.002):
        s = np.arange(-0.15, 0.1501, 0.003)
        r = np.hypot(d, s)
        s = s[(r >= lo) & (r <= hi)]
        if len(s) < 3:
            continue
        ang = np.degrees(np.arctan2(s, -d))
        idx = np.array(list(itertools.combinations(range(len(s)), 3)))
        ss = s[idx]
        apart = (np.diff(np.sort(ss, axis=1), axis=1) >= 0.030 - 1e-9).all(axis=1)
        a = np.sort(ang[idx][apart], axis=1)
        if not len(a):
            continue
        gaps = np.stack([a[:, 1] - a[:, 0], a[:, 2] - a[:, 1], 360.0 - (a[:, 2] - a[:, 0])], axis=1)
        ok = np.abs(gaps - T.GAP_DEG) <= T.GAP_TOL_DEG
        # block k's gaps are gaps[k-1] (before) and gaps[k] (after)
        placed = ok & np.roll(ok, 1, axis=1)
        best = max(best, int(placed.sum(axis=1).max()))
    assert best == 0


@pytest.mark.parametrize("radius_m", [0.030, 0.031, 0.050, 0.150, 0.200])
def test_an_even_ring_at_the_wrong_distance_scores_zero(radius_m):
    """Including a ring with every block pressed against the post (centres 30 mm from its axis)."""
    if radius_m < 0.035:  # six 30 mm cubes cannot all touch a 30 mm post; any that do sit 30-31 mm out
        objects = scene(**ring(HEXAGON[:3], radius_m=radius_m))
    else:
        objects = scene(**ring(HEXAGON, radius_m=radius_m))
    assert TASK.score(objects) == 0.0


def test_a_ring_of_five_or_of_seven_spacings_scores_zero():
    pentagon = scene(**ring([72.0 * k for k in range(5)]))  # five blocks evenly round: 72 deg gaps
    assert TASK.score(pentagon) == 0.0
    tight = scene(**ring([45.0 * k for k in range(6)]))  # six blocks 45 deg apart, bunched on one side
    assert len(T.placed(tight)) == 0


def test_blocks_held_up_standing_on_an_edge_or_off_the_table_score_zero():
    held = scene(**ring(HEXAGON, z_low=TOP + 0.02))
    assert TASK.score(held) == 0.0
    on_edge = goal(block_1=cube(polar(T.RING_R, HEXAGON[0]), tilt=45.0, tilt_axis=(1.0, 0.0, 0.0)))
    assert not T.flat_on_table(on_edge["block_1"]) and TASK.score(on_edge) == pytest.approx(3 / 6)
    floor = scene(**ring(HEXAGON, z_low=0.0))
    assert TASK.score(floor) == 0.0
