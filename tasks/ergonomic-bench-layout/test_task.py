"""Offline checks for the ergonomic-bench-layout task (numpy + pytest; no Isaac Sim).

Point YAM_ARENA at a clone of yam-isaac-lab-arena (bd62771) if `yam_arena` is not installed:

    YAM_ARENA=~/yam-isaac-lab-arena python -m pytest -q test_task.py
"""

import functools
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

_ARENA = Path(os.environ.get("YAM_ARENA", "~/yam-isaac-lab-arena")).expanduser()
if _ARENA.is_dir():
    sys.path.insert(0, str(_ARENA))

from yam_arena import YAM_URDF, ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN, PAD_TIP_PAST_TCP, READY  # noqa: E402
from yam_arena.demo import ArmPlan, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, ObjectState, Sphere, load  # noqa: E402
from yam_arena.urdf import forward_kinematics, quat_xyzw_to_matrix  # noqa: E402

HERE = Path(__file__).resolve().parent
TASK = load(HERE / "task.py")
M = TASK.module
HZ = 20.0
TOP = TABLE_TOP["station"]
BINS = M.BIN_NAMES  # heaviest first
LARGE, MEDIUM, SMALL = BINS


# -- helpers --------------------------------------------------------------------------------------------------------


def quat(yaw=0.0, tilt_deg=0.0):
    """Yaw about z, then (optionally) a tilt about the world x axis."""
    qz = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    h = math.radians(tilt_deg) / 2
    qx = np.array([math.sin(h), 0.0, 0.0, math.cos(h)])
    x1, y1, z1, w1 = qx
    x2, y2, z2, w2 = qz
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def half_extent(o) -> np.ndarray:
    if isinstance(o, Box):
        h = np.asarray(o.size, float) / 2
        if abs(math.sin(o.yaw)) > 0.5:
            h = h[[1, 0, 2]]
        return h
    if isinstance(o, Cylinder):
        return np.array([o.radius, o.radius, o.height / 2])
    return np.array([o.radius] * 3)


def bin_state(name, xy, bottom=None, yaw=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
    """Bin ``name`` with its bottom centre at (x, y, bottom), turned by ``yaw`` (0: width along x), upright unless
    tilted. ``bottom`` defaults to the pad's top."""
    w, l, h = M.BIN_DIMS[name]
    half = np.array([w, l, h]) / 2
    if abs(math.sin(yaw)) > 0.5:
        half = half[[1, 0, 2]]
    if tilt_deg:  # lying on its long side: height and width swap roles across y
        half = np.array([half[0], half[2], half[1]])
    b = M.ZONE_TOP if bottom is None else bottom
    c = np.array([xy[0], xy[1], b + half[2]])
    return ObjectState(c, quat(yaw, tilt_deg), np.asarray(vel, float), np.zeros(3), c - half, c + half)


def spec_boxes(include_bins=True):
    out = []
    for o in M.OBJECTS:
        if not include_bins and o.name in BINS:
            continue
        p, h = np.asarray(o.pos, float), half_extent(o)
        out.append((o.name, p - h, p + h))
    return out


def fixed_states():
    return {n: ObjectState((lo + hi) / 2, np.array([0, 0, 0, 1.0]), np.zeros(3), np.zeros(3), lo, hi)
            for n, lo, hi in spec_boxes(include_bins=False)}


def start_states():
    return {n: bin_state(n, M.START_XY[n], TOP, M.START[n][1]) for n in BINS}


def world(**bins):
    out = fixed_states()
    out.update(start_states())
    out.update(bins)
    return out


def goal(**extra):
    s = world(**{n: bin_state(n, M.GOAL[n]) for n in BINS})
    s.update(extra)
    return s


ROW_X = {n: M.GOAL[n][0] for n in BINS}  # nearest the worker first
Y0 = M.WORKER_Y


# -- 1. the file loads ----------------------------------------------------------------------------------------------


def test_loads_with_three_loose_bins_and_hooks():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == set(BINS)
    for hook in (TASK.success, TASK.distance, TASK.score, TASK.demo):
        assert callable(hook)
    assert TASK.docs and "worker" in TASK.docs


def test_bins_are_one_material_so_bigger_is_heavier():
    objs = {o.name: o for o in M.BINS}
    assert len({o.color for o in objs.values()}) == 1
    dens = {n: o.mass / float(np.prod(o.size)) for n, o in objs.items()}
    assert max(dens.values()) / min(dens.values()) < 1.01  # the same fill (masses rounded to the gram)
    masses = [objs[n].mass for n in BINS]
    assert masses == sorted(masses, reverse=True) and masses[0] <= 0.30  # round 1 held 370 g by a 2.5 cm band
    lengths = [M.BIN_DIMS[n][1] for n in BINS]
    for a, b in zip(lengths, lengths[1:]):
        assert a - b >= 0.035  # top-camera rule: size differences of 3-4 cm or more
    for i in range(3):
        dims = [M.BIN_DIMS[n][i] for n in BINS]
        assert dims == sorted(dims, reverse=True)  # bigger in every dimension: no size cue contradicts another


# -- 2. the demo is reachable and ends still ------------------------------------------------------------------------


@functools.lru_cache(maxsize=None)
def _played(side="right"):
    plan = TASK.demo()[side]
    arm = ArmPlan(plan, station.BASES[side])
    steps = round(arm.duration * HZ) + 5
    acts, qs, labels = [], [], []
    for k in range(steps):
        a, label = arm.action7((k + 1) / HZ)
        acts.append(a)
        qs.append(arm.q.copy())
        labels.append(label)
    return arm, np.array(acts), np.array(qs), tuple(labels)


def _check_reach(waypoints, side="right"):
    arm = ArmPlan(waypoints, station.BASES[side])
    acts = np.array([arm.action7((k + 1) / HZ)[0] for k in range(round(arm.duration * HZ) + 5)])
    assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1
    assert np.all(acts[:, :6] >= ik.LOWER) and np.all(acts[:, :6] <= ik.UPPER)


def test_demo_is_reachable_within_joint_limits_at_20_hz():
    assert set(TASK.demo()) == {"right"}
    arm, acts, _, _ = _played()
    assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1
    assert np.all(acts[:, :6] >= ik.LOWER) and np.all(acts[:, :6] <= ik.UPPER)


def test_demo_ends_with_a_still_hold_of_at_least_1_5_s():
    plan = TASK.demo()["right"]
    last, before = plan[-1], plan[-2]
    assert last.duration >= 1.5 and tuple(last.pos) == tuple(before.pos) and last.finger == before.finger
    assert last.yaw == before.yaw


def test_demo_fits_the_run_limit_and_the_readme_max_steps():
    plan = TASK.demo()["right"]
    steps = sum(w.duration for w in plan) * HZ
    meta = json.loads((HERE / "task.json").read_text())
    run_limit = int(meta["run_limit"].split()[0].replace(",", ""))
    assert steps < run_limit / 2, (steps, run_limit)
    readme = (HERE / "README.md").read_text()
    assert f"--max-steps {round(steps) + 100}" in readme


def test_releases_rise_at_least_3_cm_before_moving_sideways():
    plan = TASK.demo()["right"]
    n = 0
    for i, w in enumerate(plan):
        if w.label == "release":
            up = plan[i + 1]
            assert up.label == "retreat" and up.pos[:2] == w.pos[:2] and up.pos[2] - w.pos[2] >= 0.03
            n += 1
    assert n == 3


@pytest.mark.parametrize("name", BINS)
def test_every_bin_can_be_gripped_straight_down_where_it_starts(name):
    x, y = M.START_XY[name]
    grip = TOP + M.REST_GAP + M.GRIP_ABOVE_BOTTOM[name]
    yaw = M.GRIP_YAW[name]
    _check_reach([Waypoint(1.5, (x, y, M.TRAVEL_Z), FINGER_OPEN, "", yaw),
                  Waypoint(1.5, (x, y, grip), FINGER_OPEN, "", yaw)])


@pytest.mark.parametrize("name", BINS)
def test_every_bin_can_be_set_down_on_any_row(name):
    # Any bin may end on any row (a wrong order must be possible to build, or the order credit means nothing).
    g = M.GRIP_ABOVE_BOTTOM[name]
    for x in ROW_X.values():
        _check_reach([Waypoint(1.5, (x, Y0, M.TRAVEL_Z), FINGER_OPEN, "", M.PLACE_YAW),
                      Waypoint(1.5, (x, Y0, M.ZONE_TOP + M.RELEASE_ABOVE + g), FINGER_OPEN, "", M.PLACE_YAW)])


# -- 2b. kinematic sweep: fingers, carriage and the carried bin clear everything -----------------------------------

# The open gripper in the TCP frame (x = finger closing axis, z = approach = world down), from the URDF's
# tip_left/tip_right meshes, as in the earlier tasks: (z range, half width along y, x range of the -x finger
# for opening d). The +x finger mirrors it; the carriage above spans the full width.
FINGER_SECTIONS = [
    ((-0.014, 0.0128), 0.0137, lambda d: (-d - 0.0064, -d)),
    ((-0.034, -0.014), 0.0200, lambda d: (-d - 0.0100, -d - 0.0065)),
    ((-0.054, -0.034), 0.0330, lambda d: (-d - 0.0228, -d - 0.0024)),
]
CARRIAGE = ((-0.081, -0.054), 0.034, (-0.071, 0.071))
MARGIN = 0.003
HELD = ("lift", "carry", "lower")


def _grid(lo, hi, n=4):
    axes = [np.linspace(a, b, n) for a, b in zip(lo, hi, strict=True)]
    return np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)


def _gripper_points(d):
    pts = []
    for (z0, z1), hw, xr in FINGER_SECTIONS:
        x0, x1 = xr(d)
        pts.append(_grid((x0, -hw, z0), (x1, hw, z1)))
        pts.append(_grid((-x1, -hw, z0), (-x0, hw, z1)))
    (z0, z1), hw, (x0, x1) = CARRIAGE
    pts.append(_grid((x0, -hw, z0), (x1, hw, z1), 6))
    return np.concatenate(pts)


def _bin_points(lo, hi):
    return _grid(lo, hi, 6)


def _hits(points, boxes, margin):
    return {name for name, lo, hi in boxes if np.any(np.all((points > lo - margin) & (points < hi + margin), axis=1))}


def _sweep(offset=0.0):
    """Play the demo kinematically. The held bin sits ``offset`` off the TCP along the finger closing axis (sim:
    11-20 mm toward one finger). Returns (collisions, final bin boxes {name: (lo, hi)})."""
    plan = TASK.demo()["right"]
    arm, _, qs, labels = _played()
    base = station.BASES["right"]
    ends = np.cumsum([w.duration for w in plan])
    fixed = [b for b in spec_boxes(include_bins=False)]
    boxes = {n: (lo, hi) for n, lo, hi in spec_boxes() if n in BINS}
    held, placed, local, local_shift = None, None, None, None
    collisions = []
    for k, (q, label) in enumerate(zip(qs, labels, strict=True)):
        t = (k + 1) / HZ
        i = min(int(np.searchsorted(ends, t, side="right")), len(plan) - 1)
        name = BINS[i // 8] if i < 24 else None
        tcp, rot = ik.tcp_pose(q, base)
        _, _, finger, _ = arm.target(t)
        d = abs(finger)
        if name and label == "close" and held is None:
            held = name
            lo, hi = boxes[name]
            local = (_bin_points(lo, hi) - tcp) @ rot  # the bin's points in the TCP frame, fixed while it is held
            local_shift = local + [offset, 0.0, 0.0]  # where the grip offset shows: from the carry on
        if held and label in ("close", "release", *HELD):
            d = max(d, M.BIN_DIMS[held][0] / 2)  # the pads stop on the bin
        if label == "to the bin":
            placed = None
        # The pads open around the bin just let go and rise straight up; the sim's asymmetric pads are not
        # modelled, so that bin is left out of the finger check until the next move starts.
        others = [(n, *boxes[n]) for n in BINS if n not in (held, placed)]
        hit = _hits(tcp + _gripper_points(d) @ rot.T, fixed + others, MARGIN)
        if held and label in HELD:
            pts = tcp + (local if label == "lift" else local_shift) @ rot.T
            above_pad = [b for b in fixed if b[0] == "reach_pad"] if label == "lower" else []
            hit |= {f"carried {held} vs {h}" for h in _hits(pts, [b for b in fixed if b not in above_pad] + others,
                                                          MARGIN)}
        if hit:
            collisions.append((round(t, 2), label, sorted(hit)))
        if held and label in HELD:
            tcp_release, rot_release = tcp, rot
        if held and label == "retreat":  # the pads have opened: the bin stays where it was let go
            pts = tcp_release + local_shift @ rot_release.T
            lo, hi = pts.min(0), pts.max(0)
            drop = lo[2] - M.ZONE_TOP
            boxes[held] = (lo - [0, 0, drop], hi - [0, 0, drop])
            held, placed = None, held
    return collisions, boxes


@pytest.mark.parametrize("offset", [0.0, -0.015, 0.015, -0.020, 0.020])
def test_demo_sweep_clears_the_worker_the_pad_and_the_standing_bins(offset):
    collisions, _ = _sweep(offset)
    assert collisions == [], collisions[:10]


@pytest.mark.parametrize("offset", [0.0, -0.015, 0.015, -0.020, 0.020])
def test_demo_ends_at_full_score_with_room_to_spare(offset):
    _, boxes = _sweep(offset)
    s = world()
    for n in BINS:
        lo, hi = boxes[n]
        c = (lo + hi) / 2
        s[n] = ObjectState(c, np.array([0, 0, 0, 1.0]), np.zeros(3), np.zeros(3), lo, hi)
    assert TASK.score(s) == 1.0 and TASK.success(s)
    for a, b in M.PAIRS:
        assert M.gap(s[a], s[b]) >= M.SPACING + 0.015, (a, b, M.gap(s[a], s[b]))
        assert M.body_distance(s[b]) - M.body_distance(s[a]) >= M.ORDER_MIN_GAP + 0.05


def test_mixed_grip_offsets_of_10_mm_still_leave_the_spacing():
    # Offsets have the same sign in sim; even opposite 10 mm offsets on neighbours keep 20 mm between bins.
    s = goal(**{n: bin_state(n, (M.GOAL[n][0] + (0.01 if i % 2 else -0.01), Y0)) for i, n in enumerate(BINS)})
    assert TASK.score(s) == 1.0


def test_carry_height_clears_every_standing_bin():
    for n in BINS:
        bottom = M.TRAVEL_Z - M.GRIP_ABOVE_BOTTOM[n]
        tallest = max(M.BIN_DIMS[m][2] for m in BINS if m != n)
        assert bottom - (M.ZONE_TOP + tallest) >= 0.01, n
        assert M.TRAVEL_Z - PAD_TIP_PAST_TCP - (M.ZONE_TOP + tallest) >= 0.03, n


def test_grip_heights_keep_tops_under_the_carriage_and_tips_off_the_table():
    for n in BINS:
        g, h = M.GRIP_ABOVE_BOTTOM[n], M.BIN_DIMS[n][2]
        assert g - PAD_TIP_PAST_TCP >= 0.015, n  # fingertips at least 15 mm above the table or pad
        assert h - g <= -CARRIAGE[0][1] - 0.015, n  # bin top 15 mm or more under the finger carriage
        assert PAD_TIP_PAST_TCP + min(h - g, -FINGER_SECTIONS[-1][0][0]) >= 0.025, n  # a 25 mm or taller pad band


def test_everything_gripped_fits_the_gripper_with_a_centimetre_to_spare():
    max_open = 2 * abs(FINGER_OPEN)  # 93.9 mm between the pads, fully open
    usable = 0.084  # usable grip taken as 84 mm (pad geometry), as in the earlier tasks
    for n in BINS:
        w = M.BIN_DIMS[n][0]
        assert w + 0.01 <= usable <= max_open, n
        assert abs(M._finger(w, M.APPROACH_CLEARANCE)) - w / 2 >= 0.01, n
        assert abs(M._finger(w, M.LOOSEN)) > w / 2, n


# -- 3. scoring: goal, partial credit, every branch -----------------------------------------------------------------


def test_starting_layout_scores_zero():
    s = world()
    assert TASK.score(s) == 0.0 and not TASK.success(s)
    assert TASK.distance(s) > 0.3


def test_goal_succeeds_with_full_score_and_zero_distance():
    s = goal()
    assert TASK.score(s) == 1.0 and TASK.success(s) and TASK.distance(s) == 0.0


def test_all_three_on_the_pad_in_reverse_order_earns_0_55():
    rev = dict(zip(BINS, reversed(list(ROW_X.values())), strict=True))
    s = goal(**{n: bin_state(n, (rev[n], Y0)) for n in BINS})
    assert all(M.on_pad(s[n]) for n in BINS)
    assert TASK.score(s) == pytest.approx(0.55) and not TASK.success(s)


@pytest.mark.parametrize("a,b", [(LARGE, MEDIUM), (MEDIUM, SMALL), (LARGE, SMALL)])
def test_one_swapped_pair_costs_credit(a, b):
    s = goal(**{a: bin_state(a, M.GOAL[b]), b: bin_state(b, M.GOAL[a])})
    want = {(LARGE, MEDIUM): 0.85, (MEDIUM, SMALL): 0.85, (LARGE, SMALL): 0.55}[(a, b)]
    assert TASK.score(s) == pytest.approx(want) and not TASK.success(s)


def test_each_bin_on_the_pad_earns_0_15_and_each_ordered_pair_0_15():
    assert TASK.score(world(**{LARGE: bin_state(LARGE, M.GOAL[LARGE])})) == pytest.approx(0.15)
    assert TASK.score(world(**{n: bin_state(n, M.GOAL[n]) for n in (LARGE, SMALL)})) == pytest.approx(0.45)
    assert TASK.score(world(**{n: bin_state(n, M.GOAL[n]) for n in (MEDIUM, SMALL)})) == pytest.approx(0.45)


def test_crammed_together_in_order_misses_only_the_spacing_credit():
    xs = {LARGE: 0.40, MEDIUM: 0.341, SMALL: 0.29}  # 0-1 mm apart
    s = goal(**{n: bin_state(n, (xs[n], Y0)) for n in BINS})
    assert all(M.on_pad(s[n]) for n in BINS)
    assert TASK.score(s) == pytest.approx(0.90) and not TASK.success(s)


def test_19_mm_apart_misses_the_spacing_credit_and_21_mm_earns_it():
    for gap, want in ((0.019, 0.90), (0.021, 1.0)):
        xl = 0.43
        xm = xl - M.BIN_DIMS[LARGE][0] / 2 - gap - M.BIN_DIMS[MEDIUM][0] / 2
        xs = xm - M.BIN_DIMS[MEDIUM][0] / 2 - 0.04 - M.BIN_DIMS[SMALL][0] / 2
        s = goal(**{LARGE: bin_state(LARGE, (xl, Y0)), MEDIUM: bin_state(MEDIUM, (xm, Y0)),
                    SMALL: bin_state(SMALL, (xs, Y0))})
        assert TASK.score(s) == pytest.approx(want), gap


def test_side_by_side_at_equal_distance_is_not_an_order():
    # Two bins mirrored about the worker's midline, the same distance from the chest: neither is nearer.
    s = goal(**{MEDIUM: bin_state(MEDIUM, (0.30, Y0 + 0.075), yaw=math.pi / 2),
                SMALL: bin_state(SMALL, (0.30, Y0 - 0.075), yaw=math.pi / 2),
                LARGE: bin_state(LARGE, (0.43, Y0))})
    assert all(M.on_pad(s[n]) for n in BINS)
    assert not M.ordered(s, MEDIUM, SMALL)
    assert TASK.score(s) < 1.0


def test_order_is_by_distance_to_the_chest_not_by_a_table_axis():
    # A row along the worker's chest line (y) instead of toward the worker: right order only if radially so.
    s = goal(**{LARGE: bin_state(LARGE, (0.44, Y0), yaw=0.0), MEDIUM: bin_state(MEDIUM, (0.30, Y0 + 0.06)),
                SMALL: bin_state(SMALL, (0.22, Y0 - 0.07))})
    d = [M.body_distance(s[n]) for n in BINS]
    assert d == sorted(d) and TASK.score(s) == 1.0


def test_a_stack_of_bins_scores_zero():
    base = M.GOAL[LARGE]
    s = goal(**{LARGE: bin_state(LARGE, base),
                MEDIUM: bin_state(MEDIUM, base, M.ZONE_TOP + M.BIN_DIMS[LARGE][2]),
                SMALL: bin_state(SMALL, base, M.ZONE_TOP + M.BIN_DIMS[LARGE][2] + M.BIN_DIMS[MEDIUM][2])})
    assert TASK.score(s) == 0.0 and not TASK.success(s)


def test_one_bin_stacked_on_another_anywhere_scores_zero():
    # The other two in place on the pad; the small one on top of the large one.
    s = goal(**{SMALL: bin_state(SMALL, M.GOAL[LARGE], M.ZONE_TOP + M.BIN_DIMS[LARGE][2])})
    assert TASK.score(s) == 0.0
    # stacked off the pad, at the start spots: still 0
    s = world(**{SMALL: bin_state(SMALL, M.START_XY[MEDIUM], TOP + M.BIN_DIMS[MEDIUM][2])})
    assert TASK.score(s) == 0.0


def test_a_bin_on_the_floor_scores_zero():
    floor = ObjectState(np.array([0.3, -0.8, 0.03]), quat(0, 90), np.zeros(3), np.zeros(3),
                        np.array([0.27, -0.89, 0.0]), np.array([0.33, -0.71, 0.06]))
    for n in BINS:
        assert TASK.score(goal(**{n: floor})) == 0.0, n


def test_a_tipped_bin_on_the_pad_earns_nothing_for_itself():
    s = goal(**{SMALL: bin_state(SMALL, M.GOAL[SMALL], tilt_deg=90)})
    assert not M.on_pad(s[SMALL])
    assert TASK.score(s) == pytest.approx(0.15 * 2 + 0.15)  # two placed, one ordered pair, no spacing credit
    assert M.on_pad(goal(**{SMALL: bin_state(SMALL, M.GOAL[SMALL], tilt_deg=15)})[SMALL])


def test_held_over_the_pad_is_not_on_it():
    s = goal(**{SMALL: bin_state(SMALL, M.GOAL[SMALL], M.ZONE_TOP + 0.03)})
    assert not M.on_pad(s[SMALL]) and TASK.score(s) < 1.0


def test_the_inside_test_is_the_centre_over_the_pad():
    hx = M.BIN_DIMS[SMALL][0] / 2
    inside = goal(**{SMALL: bin_state(SMALL, (M.ZONE_X0 + 0.002, Y0))})  # centre just inside the near edge
    outside = goal(**{SMALL: bin_state(SMALL, (M.ZONE_X0 - 0.002, Y0), TOP)})  # centre just off it
    assert M.on_pad(inside[SMALL]) and not M.on_pad(outside[SMALL])
    assert inside[SMALL].low[0] < M.ZONE_X0 - hx + 0.01
    assert M.on_pad(goal(**{SMALL: bin_state(SMALL, (0.235, M.ZONE_Y1 - 0.002), yaw=math.pi / 2)})[SMALL])
    assert not M.on_pad(goal(**{SMALL: bin_state(SMALL, (0.235, M.ZONE_Y1 + 0.002), TOP, yaw=math.pi / 2)})[SMALL])


def test_moving_bins_score_but_do_not_succeed():
    s = goal(**{SMALL: bin_state(SMALL, M.GOAL[SMALL], vel=(0.05, 0, 0))})
    assert TASK.score(s) == 1.0 and not TASK.success(s)


def test_distance_shrinks_toward_the_goal():
    d0 = TASK.distance(world())
    d1 = TASK.distance(world(**{LARGE: bin_state(LARGE, M.GOAL[LARGE])}))
    d2 = TASK.distance(world(**{n: bin_state(n, M.GOAL[n]) for n in (LARGE, MEDIUM)}))
    assert d0 > d1 > d2 > TASK.distance(goal()) == 0.0


def test_score_stays_in_0_1_for_random_layouts():
    rng = np.random.default_rng(0)
    for _ in range(300):
        bins = {n: bin_state(n, (rng.uniform(0.1, 0.5), rng.uniform(-0.4, 0.1)), yaw=rng.choice([0, math.pi / 2]))
                for n in BINS}
        v = TASK.score(world(**bins))
        assert 0.0 <= v <= 1.0


# -- 4. layout: no overlaps, supports, the worker and the pad ------------------------------------------------------


def test_objects_do_not_overlap_at_the_start():
    boxes = spec_boxes()
    for i, (a, alo, ahi) in enumerate(boxes):
        for b, blo, bhi in boxes[i + 1:]:
            assert np.any(np.minimum(ahi, bhi) - np.maximum(alo, blo) <= 1e-9), (a, b)


def test_bins_start_on_the_table_off_the_pad_with_room():
    s = start_states()
    for n in BINS:
        lo, hi = s[n].low, s[n].high
        assert abs(lo[2] - TOP) < 1e-9 and not M.on_pad(s[n]), n
        clear = max(M.ZONE_X0 - hi[0], lo[0] - M.ZONE_X1, M.ZONE_Y0 - hi[1], lo[1] - M.ZONE_Y1)
        assert clear >= 0.02, (n, clear)
    for a, b in M.PAIRS:
        assert M.gap(s[a], s[b]) >= 0.03, (a, b)
    # bins rest on the table: the spawn spec sits 1 mm above it
    for o in M.BINS:
        assert o.pos[2] - half_extent(o)[2] == pytest.approx(TOP + M.REST_GAP)


def test_the_worker_sits_beyond_the_far_edge_with_forearms_on_the_bench():
    objs = {o.name: o for o in M.WORKER}
    _, lo, hi = next(b for b in spec_boxes() if b[0] == "worker_torso")
    assert lo[0] > M.TABLE_FAR_X + 0.02 and hi[2] <= TOP + 0.35
    for side in ("left", "right"):
        _, flo, fhi = next(b for b in spec_boxes() if b[0] == f"worker_forearm_{side}")
        assert flo[2] == pytest.approx(TOP) and fhi[0] < lo[0] and flo[0] - M.ZONE_X1 >= 0.04
    stool = objs["worker_stool"]
    assert stool.pos[2] - stool.height / 2 == pytest.approx(M.FLOOR_Z)
    assert all(o.mass is None for o in M.WORKER) and M.ZONE.mass is None
    # the pad lies between the worker's hands and the arms, centred on the worker
    assert (M.ZONE_Y0 + M.ZONE_Y1) / 2 == pytest.approx(M.WORKER_Y)
    assert M.BODY_XY[0] - M.ZONE_X1 >= 0.40


def test_the_order_is_unambiguous_on_the_goal_rows():
    d = [M.body_distance(goal()[n]) for n in BINS]
    assert d[1] - d[0] >= 0.08 and d[2] - d[1] >= 0.08


# -- 5. the arm and the worker; what the top camera sees -----------------------------------------------------------

CHAIN = ["base", "link1", "link2", "link3", "link4", "link5", "gripper"]


def _skeleton(q, side="right"):
    fk = forward_kinematics(YAM_URDF, {n: float(v) for n, v in zip(ARM_JOINTS, q, strict=True)})
    b = np.asarray(station.BASES[side])
    pts = [fk[k][:3, 3] + b for k in CHAIN]
    g = fk["gripper"]
    pts.append(g[:3, :3] @ np.array([0.0, 0.0, 0.147]) + g[:3, 3] + b)  # past the fingertips
    return list(zip(pts[:-1], pts[1:], strict=True))


def _seg_dist(p1, q1, p2, q2, n=40):
    s = np.linspace(0, 1, n)[:, None]
    a, b = p1 + s * (q1 - p1), p2 + s * (q2 - p2)
    return float(np.min(np.linalg.norm(a[:, None] - b[None], axis=2)))


def _box_dist(p, lo, hi):
    return float(np.linalg.norm(np.maximum(0.0, np.maximum(lo - p, p - hi))))


def test_the_arm_stays_10_cm_from_the_worker_all_run():
    _, _, qs, _ = _played()
    worker = [b for b in spec_boxes() if b[0].startswith("worker_")]
    worst = 9.0
    for q in qs[::3]:
        for a, b in _skeleton(q):
            for s in np.linspace(0, 1, 8):
                p = a + s * (b - a)
                worst = min(worst, *(_box_dist(p, lo, hi) for _, lo, hi in worker))
    assert worst >= 0.10, worst


def _project(point, cam=None):
    cam = cam or station.top_camera()
    rot = quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw))
    v = rot.T @ (np.asarray(point, float) - np.asarray(cam.pos))  # OpenGL: looking along -z, +y up
    lens = cam.lens
    return (lens.cx + lens.fx * v[0] / -v[2]) / lens.width, (lens.cy - lens.fy * v[1] / -v[2]) / lens.height, -v[2]


def _framed(point):
    u, v, depth = _project(point)
    return depth > 0 and 0.0 <= u <= 1.0 and 0.0 <= v <= 1.0


def test_top_camera_frames_the_pad_every_bin_start_and_goal_and_the_forearms():
    names = {"reach_pad", "worker_forearm_left", "worker_forearm_right", *BINS}
    for n, lo, hi in spec_boxes():
        if n in names:
            for corner in _grid(lo, hi, 2):
                assert _framed(corner), (n, corner)
    g = goal()
    for n in BINS:
        for corner in _grid(g[n].low, g[n].high, 2):
            assert _framed(corner), (n, corner)


def test_the_head_and_shoulders_are_above_the_top_frame():
    # Recorded in the README: the top view shows the forearms and the lower chest, not the shoulders or head.
    head = next(o for o in M.WORKER if isinstance(o, Sphere))
    assert not _framed(head.pos)
    _, lo, hi = next(b for b in spec_boxes() if b[0] == "worker_torso")
    assert not _framed((lo[0], M.WORKER_Y, hi[2]))
    assert _framed((lo[0], M.WORKER_Y, TOP + 0.05))  # the chest just above the bench is in view


def _hidden(point, segs, radius=0.05):
    cam = np.asarray(station.top_camera().pos)
    return min(_seg_dist(cam, np.asarray(point, float), a, b) for a, b in segs) < radius


def _scored_points(states):
    pts = [(x, y, M.ZONE_TOP) for x in np.linspace(M.ZONE_X0, M.ZONE_X1, 6)
           for y in np.linspace(M.ZONE_Y0, M.ZONE_Y1, 6)]
    for n in BINS:
        lo, hi = states[n].low, states[n].high
        pts += [tuple(p) for p in _grid((lo[0], lo[1], hi[2]), (hi[0], hi[1], hi[2]), 3)]  # bin tops
    return pts


@pytest.mark.parametrize("frame", ["first", "last"])
def test_neither_arm_hides_the_pad_or_a_bin_in_the_first_or_last_top_frame(frame):
    _, _, qs, _ = _played()
    right = np.asarray(READY) if frame == "first" else qs[-1]
    segs = _skeleton(right, "right") + _skeleton(np.asarray(READY), "left")
    states = start_states() if frame == "first" else goal()
    hidden = [p for p in _scored_points(states) if _hidden(p, segs)]
    assert hidden == [], hidden[:5]


def test_the_bins_differ_by_several_pixels_in_the_top_camera():
    # lengths 180 / 140 / 100 mm on the goal rows; at 320 x 176 each step must be at least 4 px
    s = goal()
    px = {}
    for n in BINS:
        lo, hi = s[n].low, s[n].high
        x = (lo[0] + hi[0]) / 2
        u0, v0, _ = _project((x, lo[1], hi[2]))
        u1, v1, _ = _project((x, hi[1], hi[2]))
        px[n] = math.hypot((u1 - u0) * 320, (v1 - v0) * 176)
    assert px[LARGE] - px[MEDIUM] >= 4.0 and px[MEDIUM] - px[SMALL] >= 4.0, px


# -- task.json and the README ---------------------------------------------------------------------------------------

COLOURS = ("green", "blue", "grey", "gray", "black", "white", "red", "yellow", "orange", "tan", "skin")
BANNED = ("university", "laboratory", "our team", "sprint", "institute")


def test_task_json_has_every_field_and_the_instruction_names_no_colour():
    meta = json.loads((HERE / "task.json").read_text())
    for key in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                "fidelity", "scores"):
        assert meta.get(key), key
    # scores is filled after the Dream-1 run and media by the upload script
    assert meta["scores"] == "not yet run" or meta["scores"].startswith("Dream-1 run: "), meta["scores"]
    assert meta["category"] == "Impact on jobs"
    words = set(meta["instruction"].lower().replace(",", " ").replace(".", " ").replace(";", " ").split())
    assert not words & set(COLOURS), words & set(COLOURS)
    assert "heaviest" in meta["instruction"].lower() and "lightest" in meta["instruction"].lower()
    assert "closed boxes" in meta["fidelity"]


def test_no_names_of_people_or_institutions():
    text = (json.dumps(json.loads((HERE / "task.json").read_text())) + TASK.docs + (HERE / "README.md").read_text())
    for word in BANNED:
        assert word not in text.lower(), word
