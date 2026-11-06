"""Offline checks for re-lid-the-tins (NumPy and pytest only, no Isaac Sim).

    YAM_ARENA_DIR=/path/to/yam-isaac-lab-arena python -m pytest test_task.py -q

YAM_ARENA_DIR is a checkout of the arena (tested at bd62771); it is not needed when yam_arena is installed.
"""

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
from yam_arena.analysis import hull_set_distance, world_hulls  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import TABLE_TOP, ObjectState, load  # noqa: E402
from yam_arena.urdf import axis_angle_matrix, forward_kinematics  # noqa: E402

HZ = 20.0  # the rate the task is run at (-E control_hz=20)
TASK_PATH = Path(__file__).resolve().parent / "task.py"
TASK = load(TASK_PATH)
T = TASK.module
TOP = TABLE_TOP["station"]
GRIPPER_MAX_GAP = 2 * -FINGER_OPEN  # 93.9 mm between the pads at FINGER_OPEN (arena tests/test_constants.py)
RUN_LIMIT_STEPS = 3600
DEMO_MAX_STEPS = 1100  # the README's --max-steps for the scripted demo
TRACKING = 0.005  # TCP error at rest under load, measured in sim on other tasks (IMPLEMENTATION.md)


# -- helpers ---------------------------------------------------------------------------------------------------


def rot(tilt_deg=0.0, tilt_axis=(1.0, 0.0, 0.0), yaw=0.0) -> np.ndarray:
    tip = axis_angle_matrix(np.asarray(tilt_axis, float), math.radians(tilt_deg))
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw) @ tip


def cyl_state(pos, radius, height, r=None, vel=(0.0, 0.0, 0.0)) -> ObjectState:
    """An ObjectState as the simulator reports it: world-aligned bounds of the (possibly tilted) cylinder."""
    r = np.eye(3) if r is None else r
    axis = r[:, 2]
    half = radius * np.sqrt(np.clip(1.0 - axis**2, 0.0, 1.0)) + height / 2 * np.abs(axis)
    pos = np.asarray(pos, float)
    return ObjectState(pos, ik.matrix_to_quat_xyzw(r), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)


def lid(name, tin=None, xy=None, z_low=None, dxy=(0.0, 0.0), tilt=0.0, tilt_axis=(1.0, 0.0, 0.0), vel=(0, 0, 0)):
    """A lid lying on ``tin`` (centred, plus ``dxy``) or at ``xy`` with its bounding-box bottom at ``z_low``."""
    radius, height = T.LID_SPEC[name][0], T.LID_H
    r = rot(tilt, tilt_axis)
    st = cyl_state((0.0, 0.0, 0.0), radius, height, r)
    if tin is not None:
        xy = T.tin_axis(tin) if xy is None else np.asarray(xy, float)
        z_low = T.TIN_TOP if z_low is None else z_low
    xy = np.asarray(xy, float) + np.asarray(dxy, float)
    return cyl_state((xy[0], xy[1], z_low - st.low[2]), radius, height, r, vel)


def scene(**lids) -> dict:
    """Every object's state: tins where they stand, lids at their start unless given."""
    out = {}
    for o in TASK.objects:
        out[o.name] = cyl_state(o.pos, o.radius, o.height)
    out.update(lids)
    return out


def goal(**override):
    lids = {l: lid(l, tin=t) for t, l in T.MATCH.items()}
    lids.update(override)
    return scene(**lids)


def tin_of(lid_name):
    return next(t for t, l in T.MATCH.items() if l == lid_name)


def table_spot(k):
    """Free table spots in front of the tins, one per lid."""
    return (0.16, (0.15, 0.0, -0.15)[k])


def disc_overlap(r, d):
    """Fraction of a disc of radius r covered by an equal disc whose centre is d away."""
    u = d / (2 * r)
    if u >= 1:
        return 0.0
    return (2 / math.pi) * (math.acos(u) - u * math.sqrt(1 - u * u))


def cyl_signed_distance(points, center, radius, height):
    """Signed distance from points to an upright cylinder (m; negative inside)."""
    p = np.asarray(points) - np.asarray(center)
    dr = np.hypot(p[:, 0], p[:, 1]) - radius
    dz = np.abs(p[:, 2]) - height / 2
    outside = np.hypot(np.maximum(dr, 0), np.maximum(dz, 0))
    return outside + np.minimum(np.maximum(dr, dz), 0.0)


def cyl_surface(center, radius, height, n=24, m=5):
    """Points on an upright cylinder's side, top and bottom."""
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    pts = []
    for z in np.linspace(-height / 2, height / 2, m):
        pts += [(radius * math.cos(t), radius * math.sin(t), z) for t in a]
    for z in (-height / 2, height / 2):
        for rr in (0.0, radius / 2):
            pts += [(rr * math.cos(t), rr * math.sin(t), z) for t in a]
    return np.asarray(pts) + np.asarray(center)


def hull_cyl_clearance(hulls, center, radius, height):
    """Clearance between arm hulls and an upright cylinder: hull vertices vs the cylinder, cylinder samples vs hulls."""
    center = np.asarray(center, float)
    lo = center - np.array([radius, radius, height / 2])
    hi = center + np.array([radius, radius, height / 2])
    pts = cyl_surface(center, radius, height)
    best = np.inf
    for _, h in hulls:
        vlo, vhi = h.vertices.min(0), h.vertices.max(0)
        if np.maximum(lo - vhi, vlo - hi).max() > 0.03:
            continue
        best = min(best, cyl_signed_distance(h.vertices, center, radius, height).min())
        best = min(best, (pts @ h.planes[:, :3].T - h.planes[:, 3]).max(axis=1).min())
    return float(best)


def arm_hulls(side, joints, finger):
    q = {f"joint{i + 1}": float(v) for i, v in enumerate(joints)}
    q.update(joint7=finger, joint8=finger)
    poses = forward_kinematics(YAM_URDF, q)
    base = np.eye(4)
    base[:3, 3] = station.BASES[side]
    return world_hulls({k: base @ v for k, v in poses.items() if k != "base"})


# -- the demo, played at 20 Hz, with the lids followed through it ------------------------------------------------


def held_per_waypoint(waypoints):
    """The lid in the hand at each waypoint: from its "close" to the end of its "release", else None."""
    out, current, holding = [], None, None
    for w in waypoints:
        if w.label.startswith("over "):
            current = w.label.split()[1]
        if w.label == "close":
            holding = current
        out.append(holding)
        if w.label == "release":
            holding = None
    return out


def waypoint_index(waypoints, t):
    """The waypoint ArmPlan.target is in at time t (the same walk, so labels and indices agree)."""
    for i, w in enumerate(waypoints):
        if t < w.duration:
            return i
        t -= w.duration
    return len(waypoints)


@pytest.fixture(scope="module")
def played():
    """Per arm: the plan, and per control step (time, joints, finger, TCP target, label, lid in hand or None)."""
    out = {}
    for side, waypoints in TASK.demo().items():
        arm = ArmPlan(waypoints, station.BASES[side])
        held = held_per_waypoint(waypoints)
        steps = []
        for k in range(round(arm.duration * HZ) + 5):
            t = (k + 1) / HZ
            pos, _, finger, label = arm.target(t)
            action, _ = arm.action7(t)
            i = waypoint_index(waypoints, t)
            steps.append((t, action[:6].copy(), finger, np.asarray(pos), label, held[i] if i < len(held) else None))
        out[side] = (arm, steps)
    return out


def releases(played):
    """Every release in time order: (time, side, lid, TCP position at the release)."""
    out = []
    for side, (_, steps) in played.items():
        prev = None
        for t, _, _, pos, label, held in steps:
            if label == "release" and prev != "release":
                out.append((t, side, held, pos))
            prev = label
    return sorted(out, key=lambda e: e[0])


def landing(lid_name, tcp, shift_y=0.0):
    """Where a lid let go at ``tcp`` ends up: on the tin under it, or on the table, offset ``shift_y`` from the TCP."""
    x, y = tcp[0], tcp[1] + shift_y
    for tin, (r, ty) in T.TIN_SPEC.items():
        if math.hypot(x - T.TIN_X, y - ty) < r:
            return lid(lid_name, xy=(x, y), z_low=T.TIN_TOP)
    return lid(lid_name, xy=(x, y), z_low=TOP)


def demo_end_state(played, grip_offset=0.0, tracking=(0.0, 0.0)):
    """The lids where the demo's releases put them, if each lid sits ``grip_offset`` in y from the TCP while held
    (negative: toward -y, as seen in sim) and the TCP is off by ``tracking`` at each release."""
    lids = {}
    for _, _, name, tcp in releases(played):
        p = np.array(tcp) + np.array([tracking[0], tracking[1], 0.0])
        lids[name] = landing(name, p, grip_offset)
    return scene(**lids)


# -- loading and description -------------------------------------------------------------------------------------


def test_the_task_file_loads_with_hooks_and_a_neutral_docstring():
    assert TASK.scene == "station" and TASK.score_over == "end" and TASK.success_hold_s == 1.0
    assert set(TASK.free_objects) == {"lid_a", "lid_b", "lid_c"}
    assert {o.name for o in TASK.objects if not o.free} == {"tin_left", "tin_middle", "tin_right"}
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.docs.startswith("On the station table")
    for word in ("university", "team", "lab ", "dream"):
        assert word not in TASK.docs.lower()


def test_lids_share_one_colour_one_thickness_and_names_do_not_give_the_size_away():
    colours = {o.color for o in T.LIDS.values()}
    assert len(colours) == 1 and colours.isdisjoint({o.color for o in T.TINS.values()})
    assert {o.height for o in T.LIDS.values()} == {T.LID_H}
    radii = [T.LID_SPEC[n][0] for n in sorted(T.LIDS)]
    assert radii != sorted(radii) and radii != sorted(radii, reverse=True)  # lid_a, lid_b, lid_c: not by size


# -- the fit the task rests on -------------------------------------------------------------------------------------


def test_each_tin_takes_the_one_lid_of_its_own_diameter():
    for tin, lid_name in T.MATCH.items():
        assert T.LID_SPEC[lid_name][0] == pytest.approx(T.TIN_SPEC[tin][0])
        others = [l for l in T.LIDS if l != lid_name]
        assert all(abs(T.LID_SPEC[l][0] - T.TIN_SPEC[tin][0]) >= 0.010 - 1e-9 for l in others)
    assert sorted(T.MATCH.values()) == sorted(T.LIDS)


def test_every_lid_starts_on_a_wrong_tin_in_one_three_way_cycle_with_no_free_tin():
    start_tin = {name: spec[2] for name, spec in T.LID_SPEC.items()}
    assert sorted(start_tin.values()) == sorted(T.TINS)  # one lid per tin: nowhere free to put a lid
    for name, tin in start_tin.items():
        assert T.MATCH[tin] != name
    # Following "the lid on this tin belongs on tin X" visits all three tins: a single cycle, not a swap.
    seen, tin = [], "tin_left"
    for _ in range(3):
        seen.append(tin)
        tin = tin_of(next(n for n, t in start_tin.items() if t == tin))
    assert tin == "tin_left" and len(set(seen)) == 3
    # Both kinds of misfit are on the table: a lid larger than its tin and a lid smaller than its tin.
    diff = [T.LID_SPEC[n][0] - T.TIN_SPEC[t][0] for n, t in start_tin.items()]
    assert max(diff) > 0 > min(diff)


def test_diameter_steps_are_at_least_two_centimetres():
    d = sorted(2 * r for r, _ in T.TIN_SPEC.values())
    assert d == pytest.approx([0.040, 0.060, 0.080])
    assert all(b - a >= 0.020 - 1e-9 for a, b in zip(d, d[1:]))


def test_the_largest_lid_is_a_centimetre_under_the_gripper_opening():
    assert GRIPPER_MAX_GAP == pytest.approx(0.0939, abs=1e-4)
    largest = 2 * max(r for r, _, _ in T.LID_SPEC.values())
    assert largest + 0.010 <= GRIPPER_MAX_GAP
    assert T.approach_finger("lid_a") == FINGER_OPEN  # fully open for the 80 mm lid: 6.95 mm each side
    for name, (r, _, _) in T.LID_SPEC.items():
        clearance = -T.approach_finger(name) - r
        assert 0.0069 <= clearance <= T.APPROACH_CLEARANCE + 1e-9, name


def test_the_lids_are_thick_enough_to_grip_with_the_pad_tips_above_the_tin_top():
    """The pads hold the lid's side with their tips 7 mm above its underside, so they never reach down to the tin
    top it sits on (a lid of its own size has the tin wall right under the pads), and the TCP is inside the lid."""
    tip_above_bottom = T.GRIP_ABOVE_BOTTOM - PAD_TIP_PAST_TCP
    assert tip_above_bottom >= 0.006
    assert T.GRIP_ABOVE_BOTTOM < T.LID_H  # the TCP between the pads is below the lid top
    band = T.LID_H - tip_above_bottom
    assert band >= 0.015  # 18 mm of pad on the lid's side
    # Even if the lid slips 5 mm down in the pads (seen in sim), the tips stay above its underside.
    assert tip_above_bottom - 0.005 > 0.0


def test_the_centre_tolerance_means_the_lid_visibly_covers_its_tin():
    for tin, (r, _) in T.TIN_SPEC.items():
        tol = T.centre_tol(tin)
        assert tol == pytest.approx(0.010)
        assert disc_overlap(r, tol) >= 2 / 3  # at least two-thirds of the tin top under the lid
        assert tol <= r / 2  # the lid's centre of mass stays well inside the rim: it rests flat
    assert disc_overlap(0.020, 0.010) == pytest.approx(0.685, abs=0.001)
    assert disc_overlap(0.040, 0.010) == pytest.approx(0.841, abs=0.001)


# -- starting layout ---------------------------------------------------------------------------------------------


def _bounds(o):
    st = cyl_state(o.pos, o.radius, o.height)
    return st.low, st.high


def test_nothing_overlaps_at_the_start_and_everything_is_on_the_table():
    objects = list(TASK.objects)
    for i, a in enumerate(objects):
        for b in objects[i + 1 :]:
            d = math.hypot(a.pos[0] - b.pos[0], a.pos[1] - b.pos[1])
            la, ha = _bounds(a)
            lb, hb = _bounds(b)
            vertical = min(ha[2], hb[2]) - max(la[2], lb[2])
            assert d >= a.radius + b.radius or vertical <= 1e-9, (a.name, b.name)
    tlo = np.array(station.TABLE_CENTER[:2]) - np.array(station.TABLE_SIZE[:2]) / 2
    thi = np.array(station.TABLE_CENTER[:2]) + np.array(station.TABLE_SIZE[:2]) / 2
    for o in TASK.objects:
        lo, hi = _bounds(o)
        assert np.all(lo[:2] > tlo) and np.all(hi[:2] < thi), o.name
    for o in T.TINS.values():
        assert _bounds(o)[0][2] == pytest.approx(TOP, abs=1e-9)


def test_every_lid_rests_on_its_starting_tin_with_its_centre_over_the_tin_top():
    for name, (r, _, tin) in T.LID_SPEC.items():
        o = T.LIDS[name]
        assert 0.0 < _bounds(o)[0][2] - T.TIN_TOP <= 0.002
        assert math.hypot(o.pos[0] - T.TIN_X, o.pos[1] - T.TIN_SPEC[tin][1]) == pytest.approx(0.0)


def test_lids_on_neighbouring_tins_leave_room_for_the_pads():
    """Pads are at most 10 mm thick in their lowest 33 mm; open around any lid they stay 2 cm off its neighbours."""
    ys = {name: T.LIDS[name].pos[1] for name in T.LIDS}
    for name, (r, _, _) in T.LID_SPEC.items():
        pad_outer = -T.approach_finger(name) + 0.010
        for other, (r2, _, _) in T.LID_SPEC.items():
            if other != name:
                assert abs(ys[name] - ys[other]) - pad_outer - r2 >= 0.020, (name, other)


def test_the_start_scores_zero_and_is_far_from_the_goal():
    start = scene()
    assert TASK.score(start) == 0.0 and not TASK.success(start)
    assert TASK.distance(start) > 0.3


# -- the demo -------------------------------------------------------------------------------------------------------


def test_the_demo_is_reachable_within_the_joint_limits_at_20_hz(played):
    assert set(played) == {"left", "right"}
    for side, (arm, steps) in played.items():
        joints = np.array([s[1] for s in steps])
        assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1, side
        assert np.all(joints >= ik.LOWER) and np.all(joints <= ik.UPPER), side


def test_the_demo_fits_the_step_budgets_and_both_arms_end_together_holding_still(played):
    durations = [arm.duration for arm, _ in played.values()]
    assert max(durations) - min(durations) < 1e-9  # both arms finish together
    duration = max(durations)
    assert duration * HZ + 100 <= DEMO_MAX_STEPS  # plan plus the ~60-70 steps of overhead seen in sim
    assert DEMO_MAX_STEPS < RUN_LIMIT_STEPS / 3
    last_release = max(t for t, _, _, _ in releases(played))
    for side, (arm, _) in played.items():
        assert arm.waypoints[-1].label == "hold" and arm.waypoints[-1].duration >= 1.5, side
    assert duration - last_release >= 1.5 + 1.0


def test_four_moves_park_the_80_mm_lid_on_the_table_once(played):
    rel = releases(played)
    assert [r[2] for r in rel] == ["lid_a", "lid_c", "lid_b", "lid_a"] or [r[2] for r in rel] == [
        "lid_a",
        "lid_b",
        "lid_c",
        "lid_a",
    ]
    first = rel[0]
    assert first[2] == "lid_a" and landing("lid_a", first[3]).low[2] == pytest.approx(TOP)


def test_every_lid_is_put_down_only_where_nothing_else_lies(played):
    """Replay the releases in order: each lid lands on a free tin top or a free table spot, never on another lid."""
    where = {name: T.LIDS[name].pos[:2] for name in T.LIDS}
    lifted = set()
    events = []
    for side, (_, steps) in played.items():
        prev = None
        for t, _, _, pos, label, held in steps:
            if label == "close" and prev != "close":
                events.append((t, "pick", held, pos))
            if label == "release" and prev != "release":
                events.append((t, "place", held, pos))
            prev = label
    for _, kind, name, pos in sorted(events, key=lambda e: e[0]):
        if kind == "pick":
            assert math.hypot(*(np.asarray(where[name]) - pos[:2])) < 0.004, name  # the pick is over the lid
            lifted.add(name)
            where[name] = None
            continue
        lifted.discard(name)
        for other, xy in where.items():
            if other != name and xy is not None:
                gap = math.hypot(*(np.asarray(xy) - pos[:2])) - T.LID_SPEC[name][0] - T.LID_SPEC[other][0]
                assert gap > 0.03, (name, other)
        where[name] = (pos[0], pos[1] - T.GRIP_SHIFT_Y)  # centred, after the pads' expected offset
    assert not lifted


def test_the_demo_ends_with_every_lid_seated_on_its_own_tin(played):
    end = demo_end_state(played, grip_offset=-T.GRIP_SHIFT_Y)
    assert TASK.score(end) == 1.0 and TASK.success(end)


@pytest.mark.parametrize("grip_offset", [0.0, -T.APPROACH_CLEARANCE / 2, -T.APPROACH_CLEARANCE])
@pytest.mark.parametrize("tracking", [(0, 0), (TRACKING, 0), (-TRACKING, 0), (0, TRACKING), (0, -TRACKING)])
def test_the_demo_still_scores_one_with_the_grip_offset_and_tracking_error_seen_in_sim(played, grip_offset, tracking):
    """The pads hold a lid between 0 and APPROACH_CLEARANCE toward -y of the TCP (one finger leads; the lid can move
    no further than the other pad), and the TCP may be 5 mm off at rest. Every place still seats its lid."""
    end = demo_end_state(played, grip_offset=grip_offset, tracking=tracking)
    assert TASK.score(end) == 1.0


def test_the_arms_keep_apart_and_clear_the_tins_the_lids_and_the_table(played):
    """At every other control step: the two arms at least 3 cm apart; no arm link within 4 mm of a tin, or of a
    lid it is not holding (each lid where the plan has put it by then; the lid just let go is skipped while the
    open pads rise beside it); every link above the table."""
    l_steps, r_steps = played["left"][1], played["right"][1]
    n = min(len(l_steps), len(r_steps))
    where = {name: np.array(T.LIDS[name].pos) for name in T.LIDS}
    just_released = {"left": None, "right": None}
    worst_arms, worst_tin = np.inf, np.inf
    for k in range(n):
        for side, steps in (("left", l_steps), ("right", r_steps)):
            held, prev_held = steps[k][5], steps[k - 1][5] if k else None
            if held is not None:
                where[held] = None
            elif prev_held is not None:
                where[prev_held] = landing(prev_held, steps[k - 1][3], -T.GRIP_SHIFT_Y).pos
                just_released[side] = prev_held
            if steps[k][4] not in ("release", "lift off", "up"):
                just_released[side] = None
        if k % 2:
            continue
        hulls = {side: arm_hulls(side, steps[k][1], steps[k][2]) for side, steps in (("left", l_steps), ("right", r_steps))}
        worst_arms = min(worst_arms, hull_set_distance(hulls["left"], hulls["right"])[0])
        for side, steps in (("left", l_steps), ("right", r_steps)):
            label, held = steps[k][4], steps[k][5]
            for tin in T.TINS.values():
                worst_tin = min(worst_tin, hull_cyl_clearance(hulls[side], tin.pos, tin.radius, tin.height))
            for name, p in where.items():
                if p is None or name == held or name == just_released[side]:
                    continue
                c = hull_cyl_clearance(hulls[side], p, T.LID_SPEC[name][0], T.LID_H)
                assert c > 0.004, (side, steps[k][0], label, name, c)
            low = min(h.vertices[:, 2].min() for _, h in hulls[side])
            assert low > TOP + 0.005, (side, label)
    assert worst_arms > 0.03, worst_arms
    assert worst_tin > 0.004, worst_tin


def test_a_carried_lid_passes_3_cm_above_everything_on_the_table(played):
    """Carried at travel height, a lid's underside is 3 cm above the tallest thing on the table (a lid on a tin),
    so a lid held off-centre in the pads by any amount cannot catch a neighbour."""
    tallest = T.TIN_TOP + T.LID_H
    for side, (_, steps) in played.items():
        for t, _, _, pos, label, held in steps:
            if held is not None and label.startswith("carry"):
                assert pos[2] - T.GRIP_ABOVE_BOTTOM - tallest >= 0.030, (side, held, t)


def test_the_pads_never_reach_below_a_lid_they_hold(played):
    """While a lid is in the hand its underside is 7 mm below the pad tips, so the pads cannot touch the tin top or
    the table the lid is set on."""
    for side, (_, steps) in played.items():
        for t, _, _, pos, label, held in steps:
            if held is not None:
                tips = pos[2] - PAD_TIP_PAST_TCP
                assert tips - (pos[2] - T.GRIP_ABOVE_BOTTOM) >= 0.006, (side, t)


def test_the_right_arm_picks_the_parked_lid_where_the_left_arm_left_it(played):
    rel = releases(played)
    park = rel[0][3]
    _, steps = played["right"]
    picks = [s[3] for i, s in enumerate(steps) if s[4] == "close" and steps[i - 1][4] != "close" and s[5] == "lid_a"]
    assert len(picks) == 1
    lid_centre = np.array([park[0], park[1] - T.GRIP_SHIFT_Y])
    # The lid lies within +-GRIP_SHIFT_Y of PARK_XY; the pick aims at PARK_XY, inside the 6.95 mm open clearance.
    assert np.allclose(picks[0][:2], T.PARK_XY, atol=1e-9)
    assert abs(lid_centre[1] - T.PARK_XY[1]) <= T.GRIP_SHIFT_Y + 1e-9
    assert T.GRIP_SHIFT_Y + TRACKING / 2 < -FINGER_OPEN - T.LID_SPEC["lid_a"][0]


def test_lifts_after_a_release_go_straight_up_3_cm_first(played):
    for side, (arm, _) in played.items():
        wps = arm.waypoints
        for i, w in enumerate(wps):
            if w.label == "release":
                up = wps[i + 1]
                assert up.label == "lift off"
                assert up.pos[:2] == w.pos[:2] and up.pos[2] - w.pos[2] == pytest.approx(T.LIFT_AFTER_RELEASE)


# -- scoring ---------------------------------------------------------------------------------------------------------


def test_the_goal_succeeds_scores_one_and_has_zero_distance():
    assert TASK.success(goal()) and TASK.score(goal()) == 1.0
    assert TASK.distance(goal()) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("right", [("tin_left",), ("tin_middle",), ("tin_right",), ("tin_left", "tin_middle"), ("tin_middle", "tin_right"), ("tin_left", "tin_right")])
def test_each_tin_with_its_own_lid_is_worth_a_third(right):
    lids = {}
    for k, (tin, name) in enumerate(T.MATCH.items()):
        lids[name] = lid(name, tin=tin) if tin in right else lid(name, xy=table_spot(k), z_low=TOP)
    objects = scene(**lids)
    assert TASK.score(objects) == pytest.approx(len(right) / 3)
    assert not TASK.success(objects)


@pytest.mark.parametrize("tin", ["tin_left", "tin_middle", "tin_right"])
def test_the_centre_tolerance_edge(tin):
    tol = T.centre_tol(tin)
    name = T.MATCH[tin]
    for direction in ((1, 0), (0, 1), (-0.6, -0.8)):
        dx = np.asarray(direction) * (tol - 0.001)
        assert T.tin_score(goal(**{name: lid(name, tin=tin, dxy=dx)}), tin) == pytest.approx(1 / 3)
        dx = np.asarray(direction) * (tol + 0.001)
        assert T.tin_score(goal(**{name: lid(name, tin=tin, dxy=dx)}), tin) == 0.0


def test_a_smaller_lid_on_a_larger_tin_fails_and_a_larger_lid_on_a_smaller_tin_fails():
    # Smaller: the 40 mm lid centred on the 60 mm tin, and the 60 mm lid centred on the 80 mm tin.
    smaller = scene(lid_b=lid("lid_b", tin="tin_middle"), lid_c=lid("lid_c", tin="tin_right"), lid_a=lid("lid_a", xy=table_spot(0), z_low=TOP))
    assert T.tin_score(smaller, "tin_middle") == 0.0 and T.tin_score(smaller, "tin_right") == 0.0
    # Larger: the 80 mm lid centred on the 40 mm tin, and on the 60 mm tin.
    for tin in ("tin_left", "tin_middle"):
        larger = scene(lid_a=lid("lid_a", tin=tin), lid_b=lid("lid_b", xy=table_spot(1), z_low=TOP), lid_c=lid("lid_c", xy=table_spot(2), z_low=TOP))
        assert TASK.score(larger) == 0.0
    # Every lid on every wrong tin, centred and flat: zero.
    for perm in [("lid_a", "lid_b", "lid_c"), ("lid_c", "lid_a", "lid_b"), ("lid_b", "lid_a", "lid_c"), ("lid_a", "lid_c", "lid_b"), ("lid_c", "lid_b", "lid_a")]:
        objects = scene(**{l: lid(l, tin=t) for t, l in zip(T.TINS, perm)})
        wrong = [t for t, l in zip(T.TINS, perm) if T.MATCH[t] != l]
        assert all(T.tin_score(objects, t) == 0.0 for t in wrong), perm


def test_the_start_cycle_turned_once_more_is_still_all_wrong():
    # Each lid moved one tin along the row the other way: every tin again carries a lid it does not fit.
    objects = scene(lid_a=lid("lid_a", tin="tin_middle"), lid_b=lid("lid_b", tin="tin_right"), lid_c=lid("lid_c", tin="tin_left"))
    assert TASK.score(objects) == 0.0


def test_a_lid_tilted_or_moving_on_its_tin_does_not_count_but_upside_down_does():
    tin, name = "tin_right", "lid_a"
    assert T.tin_score(goal(**{name: lid(name, tin=tin, tilt=4.0)}), tin) == pytest.approx(1 / 3)
    assert T.tin_score(goal(**{name: lid(name, tin=tin, tilt=6.0)}), tin) == 0.0
    assert T.tin_score(goal(**{name: lid(name, tin=tin, tilt=90.0)}), tin) == 0.0  # standing on edge
    assert T.tin_score(goal(**{name: lid(name, tin=tin, vel=(0.0, 0.02, 0.0))}), tin) == 0.0
    assert T.tin_score(goal(**{name: lid(name, tin=tin, tilt=180.0)}), tin) == pytest.approx(1 / 3)


# -- construct validity: shortcuts score nothing ---------------------------------------------------------------------


def test_a_second_lid_stacked_on_a_correct_one_zeroes_that_tin():
    for tin, own in T.MATCH.items():
        for other in T.LIDS:
            if other == own:
                continue
            stacked = goal(**{other: lid(other, tin=tin, z_low=T.TIN_TOP + T.LID_H)})
            assert T.tin_score(stacked, tin) == 0.0, (tin, other)
            assert not TASK.success(stacked)
    # All three lids stacked on one tin, its own lid at the bottom: nothing.
    pile = scene(
        lid_a=lid("lid_a", tin="tin_right"),
        lid_c=lid("lid_c", tin="tin_right", z_low=T.TIN_TOP + T.LID_H),
        lid_b=lid("lid_b", tin="tin_right", z_low=T.TIN_TOP + 2 * T.LID_H),
    )
    assert TASK.score(pile) == 0.0


def test_the_right_lid_on_top_of_a_wrong_lid_earns_nothing():
    objects = goal(
        lid_c=lid("lid_c", tin="tin_left"),  # the 60 mm lid on the 40 mm tin
        lid_b=lid("lid_b", tin="tin_left", z_low=T.TIN_TOP + T.LID_H),  # its own lid on top of it
    )
    assert T.tin_score(objects, "tin_left") == 0.0
    assert TASK.score(objects) == pytest.approx(1 / 3)  # only the 80 mm tin is right


def test_lids_left_on_the_table_held_in_the_air_or_knocked_off_score_zero():
    on_table = scene(**{l: lid(l, xy=table_spot(k), z_low=TOP) for k, l in enumerate(sorted(T.LIDS))})
    assert TASK.score(on_table) == 0.0
    held = scene(**{l: lid(l, tin=t, z_low=T.TIN_TOP + 0.02) for t, l in T.MATCH.items()})
    assert TASK.score(held) == 0.0
    off = scene(**{l: lid(l, xy=(0.3, y), z_low=0.0) for l, y in zip(sorted(T.LIDS), (0.2, 0.0, -0.2))})
    assert TASK.score(off) == 0.0 and TASK.distance(off) > 2.0


def test_a_lid_leaning_against_its_tin_from_the_table_scores_zero():
    leaning = goal(lid_a=lid("lid_a", xy=(T.TIN_X - 0.06, T.TIN_SPEC["tin_right"][1]), z_low=TOP, tilt=40.0, tilt_axis=(0, 1, 0)))
    assert T.tin_score(leaning, "tin_right") == 0.0


def test_score_stays_in_thirds_over_random_states_and_success_means_one():
    rng = np.random.default_rng(0)
    for _ in range(400):
        lids = {}
        for name in T.LIDS:
            if rng.random() < 0.5:
                tin = rng.choice(list(T.TINS))
                lids[name] = lid(name, tin=tin, dxy=rng.normal(0, 0.008, 2), tilt=abs(rng.normal(0, 3)))
            else:
                lids[name] = lid(name, xy=rng.uniform((0.1, -0.3), (0.45, 0.3)), z_low=TOP + rng.uniform(0, 0.1), tilt=rng.uniform(0, 90))
        objects = scene(**lids)
        s = TASK.score(objects)
        assert any(s == pytest.approx(k / 3) for k in range(4)), s
        assert TASK.success(objects) == (s == pytest.approx(1.0))
