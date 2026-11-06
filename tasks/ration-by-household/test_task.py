"""Offline checks for the ration-by-household task (numpy + pytest; no Isaac Sim).

Point YAM_ARENA at a clone of yam-isaac-lab-arena (bd62771) if `yam_arena` is not installed:

    YAM_ARENA=~/yam-isaac-lab-arena python -m pytest -q test_task.py
"""

import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

_ARENA = Path(os.environ.get("YAM_ARENA", os.environ.get("YAM_ARENA_DIR", "~/yam-isaac-lab-arena"))).expanduser()
if _ARENA.is_dir():
    sys.path.insert(0, str(_ARENA))

from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import ArmPlan, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, ObjectState, load  # noqa: E402

HERE = Path(__file__).resolve().parent
TASK = load(HERE / "task.py")
M = TASK.module
NAMES = M.PACK_NAMES
HZ = 20.0
TOP = TABLE_TOP["station"]
SIDES = ("left", "right")


# -- helpers --------------------------------------------------------------------------------------------------------


def quat_tilt(deg: float, axis=(1.0, 0.0, 0.0)) -> np.ndarray:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    h = math.radians(deg) / 2
    return np.array([*(a * math.sin(h)), math.cos(h)])


def pack(xy, bottom, on_side=False, vel=(0.0, 0.0, 0.0)):
    """A pack with its bottom centre at (x, y, bottom); ``on_side`` stands it on its 70 x 35 face."""
    half = np.array([M.PACK_X, M.PACK_H, M.PACK_Y]) / 2 if on_side else np.array([M.PACK_X, M.PACK_Y, M.PACK_H]) / 2
    centre = np.array([xy[0], xy[1], bottom + half[2]])
    q = quat_tilt(90.0) if on_side else np.array([0, 0, 0, 1.0])
    return ObjectState(centre, q, np.asarray(vel, float), np.zeros(3), centre - half, centre + half)


FLOOR_STATE = ObjectState(np.array([0.5, 0.8, 0.02]), np.array([0, 0, 0, 1.0]), np.zeros(3), np.zeros(3),
                          np.array([0.46, 0.77, 0.0]), np.array([0.54, 0.83, 0.035]))


def fixed_states():
    out = {}
    for o in M.OBJECTS:
        if o.free:
            continue
        p = np.asarray(o.pos)
        h = np.asarray(o.size) / 2 if isinstance(o, Box) else np.array([o.radius, o.radius, o.height / 2])
        out[o.name] = ObjectState(p, np.array([0, 0, 0, 1.0]), np.zeros(3), np.zeros(3), p - h, p + h)
    return out


def start_states():
    return {n: pack(xy, TOP) for n, xy in zip(NAMES, M.SUPPLY, strict=True)}


def mat_spots(h, n):
    """``n`` (xy, bottom) spots on household ``h``'s mat: four flat in a 2 x 2 grid, then layers of four on top."""
    x0, x1, y0, y1 = M.mat_bounds(h)
    grid = [(x, (y0 + y1) / 2 + dy) for x in (x0 + 0.050, x0 + 0.140) for dy in (-0.050, 0.050)]
    return [(grid[k % 4], M.MAT_TOP + M.PACK_H * (k // 4)) for k in range(n)]


def world(**packs):
    out = fixed_states()
    out.update(start_states())
    out.update(packs)
    return out


def share(left, middle, right, **extra):
    """``left``, ``middle``, ``right`` packs on those mats (taken from the supply in name order), the rest left in
    the supply, plus overrides."""
    moved, it = {}, iter(NAMES)
    for h, n in zip(M.HOUSEHOLDS, (left, middle, right), strict=True):
        for xy, bottom in mat_spots(h, n):
            moved[next(it)] = pack(xy, bottom)
    moved.update(extra)
    return world(**moved)


def spec_boxes():
    out = []
    for o in M.OBJECTS:
        p = np.asarray(o.pos)
        h = np.asarray(o.size) / 2 if isinstance(o, Box) else np.array([o.radius, o.radius, o.height / 2])
        out.append((o.name, p - h, p + h))
    return out


# -- 1. the file loads ----------------------------------------------------------------------------------------------


def test_loads_with_twelve_identical_packs_and_hooks():
    assert TASK.scene == "station" and TASK.table_top == TOP
    assert set(TASK.free_objects) == set(NAMES) and len(NAMES) == 12
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.score_over == "end" and TASK.success_hold_s == 1.0
    assert TASK.docs.startswith("On the station table")
    loose = [o for o in M.OBJECTS if o.free]
    assert all(isinstance(o, Box) for o in loose)
    assert len({(o.size, o.mass, o.color) for o in loose}) == 1  # nothing tells the packs apart


def test_households_differ_only_in_their_pegs():
    mats = [o for o in M.OBJECTS if o.name.startswith("mat_")]
    cards = [o for o in M.OBJECTS if o.name.startswith("card_")]
    assert len({(tuple(np.round(o.size, 6)), o.color) for o in mats}) == 1
    assert len({(tuple(np.round(o.size, 6)), o.color) for o in cards}) == 1
    pegs = {h: [o for o in M.OBJECTS if o.name.startswith(f"peg_{h}_")] for h in M.HOUSEHOLDS}
    assert {h: len(p) for h, p in pegs.items()} == {"left": 3, "middle": 1, "right": 2} == M.PEOPLE
    assert M.TARGET == {"left": 6, "middle": 2, "right": 4} and sum(M.TARGET.values()) == len(NAMES)
    # pegs stand on their own household's card, 30 mm of white between neighbours
    for h, ps in pegs.items():
        x0, x1, y0, y1 = M.mat_bounds(h)
        for p in ps:
            assert y0 + p.radius <= p.pos[1] <= y1 - p.radius
        ys = sorted(p.pos[1] for p in ps)
        assert all(b - a - 2 * M.PEG_R >= 0.03 - 1e-9 for a, b in zip(ys, ys[1:]))


# -- 2. the demo is reachable and ends still -----------------------------------------------------------------------


def _play(waypoints, side):
    arm = ArmPlan(waypoints, station.BASES[side])
    steps = round(arm.duration * HZ) + 5
    acts = np.array([arm.action7((k + 1) / HZ)[0] for k in range(steps)])
    return arm, acts


@pytest.mark.parametrize("side", SIDES)
def test_demo_is_reachable_within_joint_limits_at_20_hz(side):
    plan = TASK.demo()
    assert set(plan) == set(SIDES)
    arm, acts = _play(plan[side], side)
    assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1, (arm.worst_ik_mm, arm.worst_ik_deg)
    assert np.all(acts[:, :6] >= ik.LOWER) and np.all(acts[:, :6] <= ik.UPPER)


@pytest.mark.parametrize("side", SIDES)
def test_demo_ends_with_a_still_hold_of_at_least_1_5_s(side):
    plan = TASK.demo()[side]
    last, before = plan[-1], plan[-2]
    assert last.duration >= 1.5 and tuple(last.pos) == tuple(before.pos) and last.finger == before.finger


def test_both_arms_finish_together_and_the_demo_fits_the_run_limit():
    plans = TASK.demo()
    d = {s: sum(w.duration for w in plans[s]) for s in SIDES}
    assert d["left"] == pytest.approx(d["right"])
    run_limit = int(json.loads((HERE / "task.json").read_text())["run_limit"].split()[0].replace(",", ""))
    assert d["left"] * HZ < run_limit / 3, (d, run_limit)


@pytest.mark.parametrize("side", SIDES)
def test_demo_lifts_three_cm_before_moving_away_after_every_release(side):
    plan = TASK.demo()[side]
    for i, w in enumerate(plan):
        if w.label == "release":
            nxt = plan[i + 1]
            assert nxt.pos[:2] == w.pos[:2] and nxt.pos[2] - w.pos[2] >= 0.03 - 1e-9


@pytest.mark.parametrize("cell", range(12))
def test_every_pack_can_be_gripped_straight_down_by_its_own_arm(cell):
    x, y = M.SUPPLY[cell]
    side = "left" if y > 0 else "right"
    grip = TOP + M.REST_GAP + M.GRIP_ABOVE_BOTTOM
    wps = [Waypoint(1.5, (x, y, M.TRAVEL_Z), FINGER_OPEN), Waypoint(1.5, (x, y, grip), FINGER_OPEN)]
    arm, acts = _play(wps, side)
    assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1
    assert np.all(acts[:, :6] >= ik.LOWER) and np.all(acts[:, :6] <= ik.UPPER)


# -- 2b. kinematic sweep: fingers, carriage and the carried pack clear everything ---------------------------------

# The open gripper in the TCP frame (x = finger closing axis, z = approach = world down), from the URDF's
# tip_left/tip_right meshes, as in the first-round tasks: (z range, half width along y, x range of the -x finger
# for opening d). The +x finger mirrors it; the carriage above spans the full width.
FINGER_SECTIONS = [
    ((-0.014, 0.0128), 0.0137, lambda d: (-d - 0.0064, -d)),
    ((-0.034, -0.014), 0.0200, lambda d: (-d - 0.0100, -d - 0.0065)),
    ((-0.054, -0.034), 0.0330, lambda d: (-d - 0.0228, -d - 0.0024)),
]
CARRIAGE = ((-0.081, -0.054), 0.034, (-0.071, 0.071))
MARGIN = 0.002
HALF = np.array([M.PACK_X, M.PACK_Y, M.PACK_H]) / 2


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


def _pack_box(xy, bottom):
    c = np.array([xy[0], xy[1], bottom + HALF[2]])
    return c - HALF, c + HALF


def _hits(points, boxes, margin):
    return {name for name, lo, hi in boxes if np.any(np.all((points > lo - margin) & (points < hi + margin), axis=1))}


def _sweep(grip_offset_y=0.0):
    """Play both arms kinematically; a held pack sits ``grip_offset_y`` from the TCP (sim: about -12 to -20 mm).
    Returns (collisions, final pack bottoms {name: (x, y, bottom)}, closest horizontal TCP distance between arms)."""
    plans = TASK.demo()
    arms = {s: ArmPlan(plans[s], station.BASES[s]) for s in SIDES}
    ends = {s: np.cumsum([w.duration for w in plans[s]]) for s in SIDES}
    moves = {s: M.demo_moves(s) for s in SIDES}
    fixed = [(n, lo, hi) for n, lo, hi in spec_boxes() if n not in NAMES]
    packs = {o.name: (o.pos[0], o.pos[1], o.pos[2] - M.PACK_H / 2) for o in M.PACKS}
    held = {s: None for s in SIDES}
    done = {s: set() for s in SIDES}
    released = {s: set() for s in SIDES}
    collisions, closest = [], math.inf
    duration = max(a.duration for a in arms.values())
    for k in range(round(duration * HZ) + 5):
        t = (k + 1) / HZ
        tcps = {}
        for s in SIDES:
            arm = arms[s]
            _, label = arm.action7(t)
            i = int(np.searchsorted(ends[s], t, side="right"))
            move = i // M.MOVE_LEN if i < len(moves[s]) * M.MOVE_LEN else None
            if move is not None and move not in done[s]:
                (px, py), _, _ = moves[s][move]
                held[s] = min(packs, key=lambda n: math.hypot(packs[n][0] - px, packs[n][1] - py))
                assert math.hypot(packs[held[s]][0] - px, packs[held[s]][1] - py) < 1e-6
                done[s].add(move)
            tcp, rot = ik.tcp_pose(arm.q, station.BASES[s])
            tcps[s] = tcp
            _, _, finger, _ = arm.target(t)
            carrying = move is not None and label in ("lift", "carry", "lower")
            d = abs(finger)
            if move is not None and label in ("close", "lift", "carry", "lower"):
                d = max(d, M.PACK_Y / 2)  # the pads stop on the pack
            others = [(n, *_pack_box(p[:2], p[2])) for n, p in packs.items() if n != held[s] or move is None]
            hit = _hits(tcp + _gripper_points(d) @ rot.T, fixed + others, MARGIN)
            if carrying:
                bottom = tcp + [0, grip_offset_y, -M.GRIP_ABOVE_BOTTOM]
                lo, hi = _pack_box(bottom[:2], bottom[2])
                pts = _grid(lo, hi, 5)
                hit |= {f"carried {held[s]} vs {h}" for h in _hits(pts, fixed + others, MARGIN)}
            if hit:
                collisions.append((s, round(t, 2), label, sorted(hit)))
            if move is not None and label == "release" and move not in released[s]:
                _, (qx, qy), support = moves[s][move]
                packs[held[s]] = (qx, qy + grip_offset_y, support)
                released[s].add(move)
        closest = min(closest, math.hypot(*(tcps["left"][:2] - tcps["right"][:2])))
    return collisions, packs, closest


@pytest.fixture(scope="module", params=[0.0, -0.015, -0.020])
def sweep(request):
    return request.param, _sweep(request.param)


def test_demo_sweep_clears_mats_cards_pegs_and_every_pack(sweep):
    _, (collisions, _, _) = sweep
    assert collisions == [], collisions[:10]


def test_the_two_grippers_never_come_within_16_cm(sweep):
    # The carriages reach 71 mm to each side of the TCP along y, so 16 cm keeps them 18 mm apart.
    _, (_, _, closest) = sweep
    assert closest >= 0.16, closest


def test_demo_ends_with_two_packs_per_person(sweep):
    offset, (_, packs, _) = sweep
    states = world(**{n: pack((x, y), b) for n, (x, y, b) in packs.items()})
    assert M.counts(states) == M.TARGET
    assert TASK.score(states) == 1.0 and TASK.success(states)
    for n, (x, y, b) in packs.items():
        h = next(h for h in M.HOUSEHOLDS if M.on_mat(states[n], h))
        x0, x1, y0, y1 = M.mat_bounds(h)
        # every pack lies wholly on its mat, even with the grip offset
        assert x0 <= x - M.PACK_X / 2 and x + M.PACK_X / 2 <= x1, n
        assert y0 <= y - M.PACK_Y / 2 and y + M.PACK_Y / 2 <= y1, (n, offset)


def test_stacks_line_up_because_the_offset_repeats():
    # Both packs of a stack are gripped and placed the same way, so a steady offset shifts them together; a 9 mm
    # difference (the spread seen in sim) still leaves the top pack's centre 18 mm inside the lower pack's edge.
    assert M.PACK_Y / 2 - 0.009 >= 0.018


def test_travel_height_clears_stacks_and_the_open_fingers_clear_the_supply():
    stack_top = M.MAT_TOP + 2 * M.PACK_H
    assert M.TRAVEL_Z - M.GRIP_ABOVE_BOTTOM > stack_top + 0.03  # carried pack's bottom over a two-pack stack
    assert M.TRAVEL_Z - PAD_TIP_PAST_TCP > stack_top + 0.03  # empty hand over a stack
    assert M.TRAVEL_Z - PAD_TIP_PAST_TCP > M.CARD_TOP + M.PEG_H + 0.03  # and over the pegs
    # at the grip, the pad tips stay above the table
    assert M.GRIP_ABOVE_BOTTOM - PAD_TIP_PAST_TCP >= 0.004


# -- 3. scoring: the goal and every partial-credit branch -----------------------------------------------------------


def test_starting_layout_scores_zero():
    s = world()
    assert M.counts(s) == {"left": 0, "middle": 0, "right": 0}
    assert TASK.score(s) == 0.0 and not TASK.success(s)
    assert TASK.distance(s) == pytest.approx(0.6)


def test_goal_succeeds_with_full_score_and_zero_distance():
    s = share(6, 2, 4)
    assert TASK.success(s) and TASK.score(s) == 1.0 and TASK.distance(s) == pytest.approx(0.0)


def test_flat_stacked_or_on_edge_packs_all_count():
    # flat: the six on the left mat are in one layer; stacked three high, and a pack standing on its side
    s = share(6, 2, 4)
    (xy0, _), (xy1, _) = mat_spots("middle", 2)
    s[NAMES[6]] = pack(xy0, M.MAT_TOP, on_side=True)
    s[NAMES[7]] = pack(xy0, M.MAT_TOP + M.PACK_Y)  # on top of the one on its side: lowest point 55 mm up
    assert M.counts(s) == M.TARGET and TASK.score(s) == 1.0
    left = {n: pack(mat_spots("left", 1)[0][0], M.MAT_TOP + M.PACK_H * k) for k, n in enumerate(NAMES[:3])}
    s = share(6, 2, 4, **left)  # three of the left household's six in one stack
    assert M.counts(s)["left"] == 6 and TASK.score(s) == 1.0


@pytest.mark.parametrize(
    "c, expected",
    [
        ((6, 2, 4), 1.0),
        ((5, 2, 4), 5 / 6),  # one left in the supply
        ((6, 1, 4), 5 / 6),
        ((6, 2, 3), 5 / 6),
        ((7, 2, 3), 2 / 3),  # one too many for one household, one too few for another
        ((5, 3, 4), 2 / 3),
        ((7, 1, 4), 2 / 3),
        ((5, 2, 3), 2 / 3),
        ((5, 1, 3), 1 / 2),  # three left in the supply
        ((7, 1, 3), 1 / 2),
        ((6, 1, 5), 2 / 3),
        ((8, 1, 3), 1 / 3),
        ((3, 1, 2), 1 / 6),  # one per person, six left in the supply: only the middle is one off
        ((6, 0, 4), 2 / 3),  # the middle household forgotten
        ((9, 0, 3), 1 / 6),
        ((4, 2, 3), 1 / 2),
    ],
)
def test_per_household_credit(c, expected):
    s = share(*c)
    assert M.counts(s) == dict(zip(M.HOUSEHOLDS, c, strict=True))
    assert M.ordered_by_size(M.counts(s))
    assert TASK.score(s) == pytest.approx(expected)
    assert TASK.success(s) == (c == (6, 2, 4))


def test_household_credit_table():
    assert [M.household_credit(n, 4) for n in range(8)] == pytest.approx([0, 0, 0, 1 / 6, 1 / 3, 1 / 6, 0, 0])


def test_moving_packs_score_but_do_not_succeed():
    s = share(6, 2, 4)
    s[NAMES[0]] = pack(mat_spots("left", 1)[0][0], M.MAT_TOP, vel=(0.05, 0, 0))
    assert TASK.score(s) == 1.0 and not TASK.success(s)


def test_distance_shrinks_toward_the_goal():
    path = [(0, 0, 0), (2, 0, 0), (2, 0, 2), (4, 2, 2), (6, 2, 4)]
    d = [TASK.distance(share(*c)) for c in path]
    assert all(a > b for a, b in zip(d, d[1:])) and d[-1] == pytest.approx(0.0)


# -- 4. construct validity: shortcuts and the wrong rule score 0 ----------------------------------------------------


@pytest.mark.parametrize(
    "c, why",
    [
        ((4, 4, 4), "an equal split ignores household size"),
        ((3, 3, 3), "an equal split of nine"),
        ((6, 4, 2), "middle and right households swapped"),
        ((4, 2, 6), "left and right swapped"),
        ((2, 6, 4), "the single person given the most"),
        ((12, 0, 0), "everything to the largest household"),
        ((6, 3, 3), "one person given as much as two"),
        ((4, 2, 4), "three people given no more than two"),
        ((0, 0, 0), "nothing handed out"),
        ((1, 1, 1), "one pack per household"),
    ],
)
def test_allocations_not_ordered_by_household_size_score_zero(c, why):
    s = share(*c)
    assert not M.ordered_by_size(M.counts(s)), why
    assert TASK.score(s) == 0.0 and not TASK.success(s), why


def test_any_pack_on_the_floor_zeroes_the_score():
    for name in (NAMES[0], NAMES[11]):  # one from a mat, one from the supply
        s = share(6, 2, 3, **{name: FLOOR_STATE})
        assert TASK.score(s) == 0.0 and not TASK.success(s)


def test_packs_on_the_card_between_mats_or_held_above_do_not_count():
    on_card = pack(((M.CARD_X0 + M.CARD_X1) / 2, M.MAT_Y["right"] + 0.06), M.CARD_TOP)
    between = pack((0.34, 0.15), TOP)
    held = pack(mat_spots("right", 4)[3][0], M.MAT_TOP + M.NEAR_MAT + 0.005)
    for bad in (on_card, between, held):
        s = share(6, 2, 4, **{NAMES[11]: bad})  # the right household's fourth pack is elsewhere
        assert M.counts(s) == {"left": 6, "middle": 2, "right": 3}, bad
        assert TASK.score(s) == pytest.approx(5 / 6) and not TASK.success(s)


def test_pushing_the_supply_back_onto_the_mats_does_not_reach_the_share():
    # Shoving each arm's six packs straight back (+x) by 15 cm lands them on whatever mat is behind them: the
    # left six partly on the left mat, the right six partly on the right mat, none on the middle mat.
    pushed = {n: pack((x + 0.15, y), TOP if not (M.MAT_X0 <= x + 0.15) else M.MAT_TOP)
              for n, (x, y) in zip(NAMES, M.SUPPLY, strict=True)}
    s = world(**pushed)
    c = M.counts(s)
    assert c["middle"] == 0 and TASK.score(s) <= 1 / 6 and not TASK.success(s)


# -- 5. layout: no overlaps, supports, clearances, gripper fit ------------------------------------------------------


def test_objects_do_not_overlap_at_the_start():
    boxes = spec_boxes()
    for i, (a, alo, ahi) in enumerate(boxes):
        for b, blo, bhi in boxes[i + 1 :]:
            overlap = np.minimum(ahi, bhi) - np.maximum(alo, blo)
            assert np.any(overlap <= 1e-9), (a, b, overlap)


def test_everything_rests_on_the_table_or_a_fixture():
    fixed = [(n, lo, hi) for n, lo, hi in spec_boxes() if n not in NAMES]
    for name, lo, hi in fixed:
        centre = (lo + hi) / 2
        under = [h[2] for n, l, h in fixed if n != name and np.all(l[:2] <= centre[:2]) and np.all(centre[:2] <= h[:2])]
        assert abs(lo[2] - TOP) < 1e-9 or any(abs(lo[2] - u) < 1e-9 for u in under), name
    for o in M.PACKS:
        bottom = o.pos[2] - o.size[2] / 2
        under = [hi[2] for n, lo, hi in fixed if lo[0] <= o.pos[0] <= hi[0] and lo[1] <= o.pos[1] <= hi[1]]
        assert not under and 0.0 <= bottom - TOP <= 0.002, o.name  # on the bare table, off every mat


def test_no_pack_starts_on_a_mat_and_mats_are_far_apart():
    s = world()
    assert all(not M.on_mat(s[n], h) for n in NAMES for h in M.HOUSEHOLDS)
    ys = sorted(M.MAT_Y.values())
    assert all(b - a - M.MAT >= 0.1 - 1e-9 for a, b in zip(ys, ys[1:]))
    # the supply's back row stops before the mats begin
    assert max(M.SUPPLY_ROW_X) + M.PACK_X / 2 < M.MAT_X0 - 0.01


def test_twenty_mm_of_free_space_where_the_fingers_go_in():
    # Fingers close along world y (yaw 0). The open finger's lower sections reach 57 mm from the grip line; from
    # there to the next pack across must be at least 20 mm. Front-to-back neighbours sit outside the fingers' y band.
    finger_outer = abs(FINGER_OPEN) + 0.0100
    for (x, y) in M.SUPPLY:
        for (u, v) in M.SUPPLY:
            if (u, v) == (x, y):
                continue
            if abs(u - x) < M.PACK_X / 2 + 0.033:  # x ranges overlap with the finger's 33 mm half width
                if abs(v - y) > 0.01:
                    assert abs(v - y) - M.PACK_Y / 2 - finger_outer >= 0.02 - 1e-9, ((x, y), (u, v))
    assert M.SUPPLY_ROW_X[1] - M.SUPPLY_ROW_X[0] - M.PACK_X >= 0.015 - 1e-9


def test_packs_fit_the_gripper_with_a_centimetre_to_spare():
    max_open = 2 * abs(FINGER_OPEN)  # 93.9 mm between the pads, fully open
    assert M.PACK_Y + 0.01 <= max_open and M.PACK_Y <= 0.084 - 0.01
    # the held pack's top stays well under the finger carriage (54 mm above the TCP)
    assert M.PACK_H - M.GRIP_ABOVE_BOTTOM < 0.054 - 0.02


def test_each_mat_can_hold_more_than_its_share():
    # Over-giving must be possible, or the count would not be the policy's choice. Six fit flat (three of 55 mm one
    # way, two of 70 mm the other) and stacks hold more.
    assert 3 * M.PACK_Y + 0.02 <= M.MAT and 2 * M.PACK_X + 0.02 <= M.MAT


def test_demo_uses_every_pack_once_and_builds_one_two_pack_stack_per_person():
    picks = [p for s in SIDES for p, _ in M.PLAN[s]]
    assert sorted(picks) == sorted(M.SUPPLY)
    stacks = [st for s in SIDES for _, st in M.PLAN[s]]
    for h in M.HOUSEHOLDS:
        assert len(M.STACKS[h]) == M.PEOPLE[h]
        assert all(stacks.count((h, k)) == M.PER_PERSON for k in range(M.PEOPLE[h]))


# -- task.json ------------------------------------------------------------------------------------------------------


def test_task_json_has_every_field_and_no_names():
    meta = json.loads((HERE / "task.json").read_text())
    for key in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                "fidelity", "scores"):
        assert meta.get(key), key
    # scores is filled after the Dream-1 run and media by the upload script
    assert meta["scores"] == "not yet run" or meta["scores"].startswith("Dream-1 run: "), meta["scores"]
    assert meta["category"] == "Societal relevance"
    assert "twelve ration packs" in meta["instruction"] and "per person" in meta["instruction"]
    text = json.dumps(meta).lower() + TASK.docs.lower() + (HERE / "README.md").read_text().lower()
    for word in ("robocurve sprint", "our team"):
        assert word not in text, word
