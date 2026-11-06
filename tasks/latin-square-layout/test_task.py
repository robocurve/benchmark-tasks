"""Offline checks for latin-square-layout (NumPy and pytest only, no Isaac Sim).

    YAM_ARENA_DIR=/path/to/yam-isaac-lab-arena python -m pytest test_task.py -q

YAM_ARENA_DIR is a checkout of the arena (tested at bd62771); it is not needed when yam_arena is installed.
"""

import functools
import itertools
import json
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
from yam_arena.analysis import box_signed_distance, hull_set_distance, link_hulls, world_hulls  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN, PAD_TIP_PAST_TCP, READY  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, ObjectState, load  # noqa: E402
from yam_arena.urdf import axis_angle_matrix, forward_kinematics, quat_xyzw_to_matrix  # noqa: E402

HZ = 20.0  # the rate the task is run at (-E control_hz=20)
HERE = Path(__file__).resolve().parent
TASK = load(HERE / "task.py")
T = TASK.module
TOP = TABLE_TOP["station"]
GRIPPER_MAX_GAP = 0.094  # pads 94 mm apart at FINGER_OPEN (arena constants.py)
RUN_LIMIT_STEPS = 2400
Z = np.array([0.0, 0.0, 1.0])
SIDES = ("left", "right")


# -- helpers ---------------------------------------------------------------------------------------------------------


def rot(yaw=0.0, tilt_deg=0.0, tilt_axis=(1.0, 0.0, 0.0)) -> np.ndarray:
    tip = axis_angle_matrix(np.asarray(tilt_axis, float), math.radians(tilt_deg))
    return axis_angle_matrix(Z, yaw) @ tip


def box_state(pos, size, r=None, vel=(0.0, 0.0, 0.0)) -> ObjectState:
    """An ObjectState as the simulator reports it: world-aligned bounds of the turned box."""
    r = np.eye(3) if r is None else r
    pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    world = corners @ r.T + pos
    return ObjectState(pos, ik.matrix_to_quat_xyzw(r), np.asarray(vel, float), np.zeros(3), world.min(0), world.max(0))


def block(name, xy, z_low=None, yaw=0.0, tilt=0.0, vel=(0.0, 0.0, 0.0)) -> ObjectState:
    """Block ``name`` with its centre over ``xy``, its bottom at ``z_low`` (default: on the board)."""
    size = T.block_size(name)
    r = rot(yaw, tilt)
    half_h = float(np.abs(r @ np.diag(np.asarray(size) / 2)).sum(axis=1)[2])
    z_low = T.BOARD_TOP if z_low is None else z_low
    return box_state((xy[0], xy[1], z_low + half_h), size, r, vel)


def fixed_state(obj: Box) -> ObjectState:
    return box_state(obj.pos, obj.size, rot(obj.yaw))


def start(**overrides) -> dict:
    """The starting layout: fixed parts where they are, loose blocks settled on the table at their staging spots."""
    out = {o.name: fixed_state(o) for o in T.OBJECTS if o.mass is None}
    out.update({n: block(n, T.STAGING[n], TOP, yaw=T.STAGE_YAW) for n in T.LOOSE})
    out.update(overrides)
    return out


def in_cell(name, cell, dxy=(0.0, 0.0), **kw) -> ObjectState:
    x, y = T.cell_xy(cell)
    return block(name, (x + dxy[0], y + dxy[1]), **kw)


def goal(**overrides) -> dict:
    out = start()
    out.update({n: in_cell(n, T.GOAL_CELL[n]) for n in T.LOOSE})
    out.update(overrides)
    return out


def with_layout(placement: dict) -> dict:
    """Start state with each loose block in ``placement`` (name -> cell) seated in that square."""
    out = start()
    out.update({n: in_cell(n, c) for n, c in placement.items()})
    return out


def aabb(o):
    if isinstance(o, Box):
        st = box_state(o.pos, o.size, rot(o.yaw))
        return st.low, st.high
    raise TypeError(o)


# -- loading and description ---------------------------------------------------------------------------------------


def test_the_task_file_loads_with_hooks_and_a_neutral_docstring():
    assert TASK.scene == "station" and TASK.score_over == "end" and TASK.success_hold_s == 1.0
    assert set(TASK.free_objects) == set(T.LOOSE) and len(T.LOOSE) == 6
    assert len(T.BLOCKS) == 9 and len(T.FIXED_BLOCKS) == 3
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.docs.startswith("On the station table")
    assert all(isinstance(o, Box) for o in T.OBJECTS)  # primitives only, no library assets


def test_one_block_of_every_colour_and_length():
    combos = {T.attributes(n) for n in T.BLOCKS}
    assert combos == set(itertools.product(T.COLOURS, T.LENGTHS))
    for n, b in T.BLOCKS.items():
        assert b.size == T.block_size(n) and b.yaw == (T.STAGE_YAW if n in T.LOOSE else 0.0)
        assert b.color == T.COLOURS[T.attributes(n)[0]]


def test_every_loose_block_is_light_pine_and_fits_the_gripper_with_room_to_spare():
    for n in T.LOOSE:
        b = T.BLOCKS[n]
        assert b.mass == pytest.approx(500.0 * np.prod(b.size))
        assert 0.010 <= b.mass <= 0.040
    assert -FINGER_OPEN * 2 == pytest.approx(GRIPPER_MAX_GAP, abs=5e-4)
    assert T.SECTION + 0.010 <= GRIPPER_MAX_GAP - 0.010  # gripped across 25 mm; 1 cm margin under 84 mm usable
    approach_gap = -2 * T._finger(T.APPROACH_CLEARANCE)
    assert approach_gap == pytest.approx(0.045) and approach_gap < GRIPPER_MAX_GAP
    for b in T.FIXED_BLOCKS:
        assert b.mass is None


# -- the puzzle ------------------------------------------------------------------------------------------------------


def _grid_score(grid) -> int:
    return sum(T.line_ok(grid, line) for line in T.LINES)


@functools.cache
def _completions():
    """Every way to put the six loose blocks in the six empty squares, with the number of lines it satisfies."""
    free = [c for c in T.CELLS if c not in T.GIVENS]
    out = []
    for perm in itertools.permutations(T.LOOSE):
        grid = dict(T.GIVENS)
        grid.update(zip(free, perm, strict=True))
        out.append((grid, _grid_score(grid)))
    return out


def test_the_givens_are_part_of_the_solution_and_the_solution_is_the_only_one():
    for c, n in T.GIVENS.items():
        assert T.SOLUTION[c] == n
    assert sorted(T.SOLUTION.values()) == sorted(T.BLOCKS)
    assert _grid_score(T.SOLUTION) == 6
    full = [g for g, s in _completions() if s == 6]
    assert full == [T.SOLUTION]


def test_no_line_is_complete_at_the_start():
    for line in T.LINES:
        assert sum(c in T.GIVENS for c in line) <= 2


def test_a_random_fill_or_a_colour_only_pattern_earns_little():
    """Construct validity in numbers: filling the six squares at random averages 0.10; a fill that gets every
    colour right but ignores length (the one-attribute skill) averages 0.28 and can score 0; so can length alone."""
    scores = [s / 6 for _, s in _completions()]
    assert np.mean(scores) == pytest.approx(0.1028, abs=1e-3)

    def colours_ok(g):
        return all(len({T.attributes(g[c])[0] for c in line}) == 3 for line in T.LINES)

    def lengths_ok(g):
        return all(len({T.attributes(g[c])[1] for c in line}) == 3 for line in T.LINES)

    colour_only = [s / 6 for g, s in _completions() if colours_ok(g)]
    length_only = [s / 6 for g, s in _completions() if lengths_ok(g)]
    assert len(colour_only) == len(length_only) == 12
    assert np.mean(colour_only) == pytest.approx(0.278, abs=2e-3) and min(colour_only) == 0.0
    assert np.mean(length_only) == pytest.approx(0.278, abs=2e-3) and min(length_only) == 0.0


# -- starting layout -----------------------------------------------------------------------------------------------


def test_nothing_overlaps_at_the_start():
    s = start()
    names = [o.name for o in T.OBJECTS if not o.name.startswith(("board", "tape"))]
    for a, b in itertools.combinations(names, 2):
        gap = np.maximum(s[a].low - s[b].high, s[b].low - s[a].high).max()
        assert gap > 0.0, (a, b)
    board = s["board"]
    for n in T.LOOSE:  # staged blocks clear of the board edge by at least 4 cm
        gap = np.maximum(s[n].low[:2] - board.high[:2], board.low[:2] - s[n].high[:2]).max()
        assert gap >= 0.04, (n, gap)
    for a, b in itertools.combinations(T.LOOSE, 2):  # staged end to end, at least 25 mm apart
        gap = np.maximum(s[a].low[:2] - s[b].high[:2], s[b].low[:2] - s[a].high[:2]).max()
        assert gap >= 0.025, (a, b, gap)


def test_loose_blocks_rest_on_the_table_and_fixed_blocks_on_the_board():
    tlo = np.array(station.TABLE_CENTER[:2]) - np.array(station.TABLE_SIZE[:2]) / 2
    thi = np.array(station.TABLE_CENTER[:2]) + np.array(station.TABLE_SIZE[:2]) / 2
    for o in T.OBJECTS:
        lo, hi = aabb(o)
        assert np.all(lo[:2] > tlo + 0.1) and np.all(hi[:2] < thi - 0.1), o.name
    for n in T.LOOSE:
        lo, _ = aabb(T.BLOCKS[n])
        assert 0.0 < lo[2] - TOP <= 0.002, n
    for b in T.FIXED_BLOCKS:
        lo, _ = aabb(b)
        assert lo[2] == pytest.approx(T.BOARD_TOP)
    lo, hi = aabb(T.OBJECTS[0])
    assert T.OBJECTS[0].name == "board" and lo[2] == pytest.approx(TOP) and hi[2] == pytest.approx(T.BOARD_TOP)


def test_every_block_fits_inside_its_square_with_room():
    for c in T.CELLS:
        size = T.block_size(T.SOLUTION[c])
        assert T.PITCH_X / 2 - size[0] / 2 - T.TAPE_W / 2 >= 0.020  # 24.5 mm front and back
        assert T.PITCH_Y / 2 - size[1] / 2 - T.TAPE_W / 2 >= 0.015  # 17 mm at each end of a long block
    assert T.cell_of(fixed_state(T.BLOCKS["blue_short"])) == (1, 1)


def test_the_start_scores_zero():
    s = start()
    assert TASK.score(s) == 0.0 and not TASK.success(s)
    assert TASK.distance(s) > 6 * 0.05


# -- the demo --------------------------------------------------------------------------------------------------------


@functools.cache
def _played():
    """Per arm: the plan and per control step (time, joints, finger, TCP target, label)."""
    out = {}
    for side, waypoints in TASK.demo().items():
        arm = ArmPlan(waypoints, station.BASES[side])
        steps = []
        for k in range(round(arm.duration * HZ) + 5):
            t = (k + 1) / HZ
            pos, _, finger, label = arm.target(t)
            action, _ = arm.action7(t)
            steps.append((t, action[:6].copy(), finger, pos, label))
        out[side] = (arm, steps)
    return out


def test_the_demo_is_reachable_within_the_joint_limits_at_20_hz():
    played = _played()
    assert set(played) == set(SIDES)
    durations = set()
    for side, (arm, steps) in played.items():
        joints = np.array([s[1] for s in steps])
        assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1, side
        assert np.all(joints >= ik.LOWER) and np.all(joints <= ik.UPPER), side
        durations.add(arm.duration)
        assert arm.duration * HZ + 100 < RUN_LIMIT_STEPS / 2
    assert len(durations) == 1


def test_the_demo_moves_each_loose_block_once_to_its_square():
    moved = [n for side in SIDES for n in T.DEMO[side]]
    assert sorted(moved) == sorted(T.LOOSE)
    for side, names in T.DEMO.items():
        for n in names:  # each arm picks from its own side and places in its half or the middle column
            assert math.copysign(1, T.STAGING[n][1]) == (1 if side == "left" else -1)
            col = T.GOAL_CELL[n][1]
            assert col in ((0, 1) if side == "left" else (1, 2))
    assert T.GOAL_CELL[T.DEMO["left"][0]][1] == 1 and T.GOAL_CELL[T.DEMO["right"][-1]][1] == 1  # middle: first, last


def test_the_demo_ends_with_a_still_hold_and_rises_after_every_release():
    for side, waypoints in TASK.demo().items():
        last, before = waypoints[-1], waypoints[-2]
        assert last.label == "hold still" and last.duration >= 1.5, side
        assert last.pos == before.pos and last.finger == before.finger and last.yaw == before.yaw, side
        for i, w in enumerate(waypoints):
            if w.label == "release":
                rise = waypoints[i + 1]
                assert rise.label == "rise clear" and rise.pos[:2] == w.pos[:2]
                assert rise.pos[2] - w.pos[2] >= 0.03 - 1e-9


def test_grip_and_carry_heights():
    assert T.TOP + T.GRIP_ABOVE_BOTTOM - PAD_TIP_PAST_TCP >= T.TOP + 0.005  # pad tips 6 mm above the table
    assert T.GRIP_ABOVE_BOTTOM + PAD_TIP_PAST_TCP <= T.SECTION + 0.010  # pads hold the block's upper part
    carried_bottom = T.CARRY_Z - T.GRIP_ABOVE_BOTTOM
    assert carried_bottom >= T.BOARD_TOP + T.SECTION + 0.020  # 2 cm over any block standing on the board
    assert T.HOVER_Z - PAD_TIP_PAST_TCP >= T.BOARD_TOP + T.SECTION + 0.050


def _arm_hulls(side, joints, finger):
    q = {f"joint{i + 1}": float(v) for i, v in enumerate(joints)}
    q.update(joint7=finger, joint8=finger)
    poses = forward_kinematics(YAM_URDF, q)
    base = np.eye(4)
    base[:3, 3] = station.BASES[side]
    return world_hulls({k: base @ v for k, v in poses.items() if k != "base"})


def _event_times(side):
    """Per block this arm handles: (time its grip starts closing, time it is released)."""
    t, out, current = 0.0, {}, None
    names = iter(T.DEMO[side])
    for w in TASK.demo()[side]:
        if w.label == "to the block":
            current = next(names)
        if w.label == "close":
            out.setdefault(current, [t, None])
        t += w.duration
        if w.label == "release":
            out[current][1] = t
    return out


def _states_at(t):
    """Where each block is at time t of the demo (staged until picked, in its square once released)."""
    s = start()
    for side in SIDES:
        for n, (_, released) in _event_times(side).items():
            if t >= released:
                s[n] = in_cell(n, T.GOAL_CELL[n])
    return s


def test_neither_arm_comes_within_15_mm_of_a_block_it_is_not_handling():
    """At every control step (every other one, for speed): every arm link at least 15 mm from every block except
    the one this arm is picking, carrying or has just let go of; the pad tips stay above the table and board."""
    assert set(link_hulls()) >= {"gripper", "tip_left", "tip_right"}
    for side, (_, steps) in _played().items():
        events = _event_times(side)
        worst = np.inf
        for t, joints, finger, _, label in steps[::2]:
            hulls = _arm_hulls(side, joints, finger)
            low = min(h.vertices[:, 2].min() for _, h in hulls)
            assert low > TOP + 0.004, (side, label, low - TOP)
            states = _states_at(t)
            busy = {n for n, (a, b) in events.items() if a - 1.6 <= t <= b + 2.1}  # descend .. back up to hover
            for n in T.BLOCKS:
                if n in busy:
                    continue
                st = states[n]
                r = quat_xyzw_to_matrix(np.asarray(st.quat_xyzw))
                for _, h in hulls:
                    d = box_signed_distance(h.vertices, np.asarray(st.pos), r, np.asarray(T.block_size(n))).min()
                    worst = min(worst, float(d))
        assert worst > 0.015, (side, worst)


def test_the_arms_stay_apart():
    (_, left), (_, right) = _played()["left"], _played()["right"]
    worst = np.inf
    for a, b in zip(left[::4], right[::4], strict=True):
        d, _ = hull_set_distance(_arm_hulls("left", a[1], a[2]), _arm_hulls("right", b[1], b[2]))
        worst = min(worst, d)
    assert worst > 0.05, worst


def _demo_result(name, offset=0.0, pick_err=(0.0, 0.0), place_err=(0.0, 0.0), yaw_err_deg=0.0):
    """Where the demo leaves a block under a simple grip model: closing the pads pulls it ``offset`` off the TCP
    along the closing direction, whatever the TCP's error that way; along the block it stays where it lay, so the
    TCP's error that way carries over. The wrist then turns the held block with it; the TCP is off by
    ``place_err`` at the release, and the block may slip ``yaw_err_deg`` in the pads."""
    side = next(s for s in SIDES if name in T.DEMO[s])
    pick_yaw, place_yaw = T.PICK_YAW[side], T.PLACE_YAW[side]
    closing = np.array([-math.sin(pick_yaw), math.cos(pick_yaw)])
    along = np.array([closing[1], -closing[0]])
    held = offset * closing - float(np.dot(np.asarray(pick_err), along)) * along  # block centre minus TCP
    turn = place_yaw - pick_yaw
    c, s = math.cos(turn), math.sin(turn)
    final = np.asarray(T.cell_xy(T.GOAL_CELL[name])) + np.asarray(place_err) + np.array([[c, -s], [s, c]]) @ held
    return block(name, final, yaw=T.STAGE_YAW + turn + math.radians(yaw_err_deg))


def test_the_demo_plan_reaches_full_score_with_a_centred_grip():
    end = goal(**{n: _demo_result(n) for n in T.LOOSE})
    assert TASK.score(end) == 1.0 and TASK.success(end) and TASK.distance(end) == pytest.approx(0.0, abs=1e-9)


def test_the_demo_still_scores_full_with_an_off_centre_grip_and_wrist_error():
    """+-15 mm grip offset along the closing direction, 5 mm of wrist error at both grip and release plus 10 mm of
    settling after the pads open, and 5 deg of slip, every combination, all six blocks at once."""
    errs = [np.array(e) * 0.005 for e in itertools.product((-1, 1), repeat=2)]
    settles = [np.array(e) * 0.010 for e in ((1, 0), (-1, 0), (0, 1), (0, -1))]
    for offset, e1, e2, settle, slip in itertools.product((-0.015, 0.015), errs, errs, settles, (-5.0, 5.0)):
        end = goal(**{n: _demo_result(n, offset, e1, e2 + settle, slip) for n in T.LOOSE})
        assert TASK.score(end) == 1.0, (offset, e1, e2, settle, slip)


# -- scoring: full marks -------------------------------------------------------------------------------------------


def test_the_goal_succeeds_and_scores_one():
    g = goal()
    assert TASK.success(g) and TASK.score(g) == 1.0
    assert TASK.distance(g) == pytest.approx(0.0, abs=1e-9)


def test_orientation_inside_a_square_does_not_matter():
    turned = goal(red_long=in_cell("red_long", (2, 1), yaw=math.pi / 2),
                  yellow_short=in_cell("yellow_short", (2, 2), yaw=0.4),
                  yellow_medium=in_cell("yellow_medium", (0, 1), tilt=90.0))  # stood on end
    assert TASK.score(turned) == 1.0


def test_off_centre_inside_the_tape_still_counts():
    sloppy = goal(blue_medium=in_cell("blue_medium", (2, 0), (0.035, -0.07)),
                  red_medium=in_cell("red_medium", (1, 2), (-0.035, 0.07)))
    assert TASK.score(sloppy) == 1.0


# -- scoring: partial credit -----------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "placement, expected_lines",
    [
        ({}, 0),  # nothing moved
        ({n: T.GOAL_CELL[n] for n in T.DEMO["left"]}, 2),  # left column and near row
        ({n: T.GOAL_CELL[n] for n in T.DEMO["right"]}, 1),  # right column only
        ({n: T.GOAL_CELL[n] for n in T.LOOSE if n != "yellow_short"}, 4),  # far row and right column open
        ({**{n: T.GOAL_CELL[n] for n in T.LOOSE}, "red_long": (2, 0), "blue_medium": (2, 1)}, 4),  # cols 0, 1 fail
        ({**{n: T.GOAL_CELL[n] for n in T.LOOSE}, "yellow_medium": (1, 2), "red_medium": (0, 1)}, 2),
        ({**{n: T.GOAL_CELL[n] for n in T.LOOSE}, "yellow_short": (0, 1), "yellow_medium": (2, 2)}, 2),  # lengths
    ],
)
def test_every_line_is_worth_one_sixth(placement, expected_lines):
    objects = with_layout(placement)
    assert TASK.score(objects) == pytest.approx(expected_lines / 6)
    assert TASK.success(objects) == (expected_lines == 6)


def test_a_block_still_moving_or_held_does_not_count():
    moving = goal(yellow_short=in_cell("yellow_short", (2, 2), vel=(0.0, 0.03, 0.0)))
    assert TASK.score(moving) == pytest.approx(4 / 6) and not TASK.success(moving)
    held = goal(yellow_short=in_cell("yellow_short", (2, 2), z_low=T.BOARD_TOP + 0.02))
    assert TASK.score(held) == pytest.approx(4 / 6)


def test_a_block_on_the_tape_line_belongs_to_the_square_its_centre_is_over():
    x0, _ = T.cell_xy((2, 2))
    just_in = goal(yellow_short=block("yellow_short", (x0, T.GRID_Y[0] + 0.001)))
    just_out = goal(yellow_short=block("yellow_short", (x0, T.GRID_Y[0] - 0.001)))
    assert TASK.score(just_in) == 1.0 and TASK.score(just_out) == pytest.approx(4 / 6)


# -- construct validity: shortcuts score nothing -------------------------------------------------------------------


def test_piling_everything_into_one_square_scores_zero():
    x, y = T.cell_xy((2, 1))
    pile = start(**{n: block(n, (x, y), T.BOARD_TOP + k * T.SECTION) for k, n in enumerate(T.LOOSE)})
    assert TASK.score(pile) == 0.0


def test_a_second_block_in_a_square_voids_that_square():
    # yellow_short in its square, and the spare-looking red_long also squeezed into it: the square counts as empty
    objects = goal(red_long=in_cell("red_long", (2, 2), (0.0, 0.03)), yellow_short=in_cell("yellow_short", (2, 2), (0.0, -0.05)))
    assert TASK.score(objects) == pytest.approx(3 / 6)  # row 2, column 1 (red_long gone) and column 2 lost


def test_covering_a_fixed_block_voids_its_square():
    objects = goal(yellow_medium=block("yellow_medium", T.cell_xy((1, 1)), T.BOARD_TOP + T.SECTION))
    # stacked on blue_short: yellow_medium is not seated, (0, 1) is empty -> row 0 and column 1 fail
    assert TASK.score(objects) == pytest.approx(4 / 6)
    beside = goal(yellow_medium=in_cell("yellow_medium", (1, 1), (0.03, 0.0)))  # on the board in the given's square
    assert TASK.score(beside) == pytest.approx(3 / 6)  # (0, 1) empty and (1, 1) void: rows 0, 1 and column 1 lose


def test_a_colour_only_or_length_only_layout_can_score_zero():
    """A layout that satisfies every colour rule but ignores length (the one-attribute, pattern-continuing skill)
    scores 0 here, and so does one that gets every length right but ignores colour."""
    colour_only = {"yellow_long": (0, 1), "yellow_short": (1, 0), "red_medium": (1, 2),
                   "blue_medium": (2, 0), "red_long": (2, 1), "yellow_medium": (2, 2)}
    length_only = {"blue_medium": (0, 1), "yellow_long": (1, 0), "yellow_medium": (1, 2),
                   "red_medium": (2, 0), "red_long": (2, 1), "yellow_short": (2, 2)}
    for placement in (colour_only, length_only):
        grid = dict(T.GIVENS)
        grid.update({c: n for n, c in placement.items()})
        assert sorted(grid) == sorted(T.CELLS)
        assert TASK.score(with_layout(placement)) == 0.0
    for placement, attr in ((colour_only, 0), (length_only, 1)):
        grid = {**T.GIVENS, **{c: n for n, c in placement.items()}}
        for line in T.LINES:
            assert len({T.attributes(grid[c])[attr] for c in line}) == 3


def test_blocks_knocked_off_the_board_or_the_table_score_nothing():
    off = goal(**{n: block(n, (T.cell_xy(T.GOAL_CELL[n])[0], 0.5), TOP) for n in T.LOOSE})
    assert TASK.score(off) == 0.0
    floor = goal(**{n: block(n, T.cell_xy(T.GOAL_CELL[n]), 0.0) for n in T.LOOSE})
    assert TASK.score(floor) == 0.0


def test_score_stays_in_range_and_success_matches_full_score_over_random_states():
    rng = np.random.default_rng(0)
    lo = np.array([T.GRID_X[0] - 0.05, T.GRID_Y[0] - 0.05])
    hi = np.array([T.GRID_X[1] + 0.05, T.GRID_Y[1] + 0.05])
    for _ in range(400):
        objects = goal(**{
            n: block(n, rng.uniform(lo, hi), T.BOARD_TOP + rng.choice([0.0, rng.uniform(0, 0.05)]), rng.uniform(-3, 3))
            for n in T.LOOSE if rng.random() < 0.5
        })
        s = TASK.score(objects)
        assert any(s == pytest.approx(k / 6) for k in range(7))
        assert TASK.success(objects) == (s == 1.0)
        assert TASK.distance(objects) >= 0.0


# -- the top camera ------------------------------------------------------------------------------------------------


def _project(point, cam=None, width=320, height=176):
    cam = cam or station.top_camera()
    lens = cam.lens.resized(width, height)
    r = quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw))
    v = r.T @ (np.asarray(point, float) - np.asarray(cam.pos))  # OpenGL: looking along -z, +y up
    return np.array([lens.cx + lens.fx * v[0] / -v[2], lens.cy - lens.fy * v[1] / -v[2]]), -v[2]


def test_the_top_camera_frames_the_board_and_every_staged_block():
    s = start()
    for n in ("board", *T.LOOSE):
        for corner in itertools.product(*zip(s[n].low, s[n].high, strict=True)):
            (u, v), depth = _project(corner)
            assert depth > 0 and 0 <= u <= 320 and 0 <= v <= 176, (n, corner, u, v)


def test_the_three_lengths_differ_by_several_pixels_in_the_top_camera():
    """4 cm steps, measured along the block's top edge in every square and staging spot; 2 cm was invisible on
    the GPU rounds, so each step must be at least 4 px at 320 x 176."""
    spots = [T.cell_xy(c) for c in T.CELLS] + list(T.STAGING.values())
    for x, y in spots:
        z = T.BOARD_TOP + T.SECTION
        px = {}
        for length, ln in T.LENGTHS.items():
            a, _ = _project((x, y - ln / 2, z))
            b, _ = _project((x, y + ln / 2, z))
            px[length] = float(np.linalg.norm(b - a))
        assert px["medium"] - px["short"] >= 4.0 and px["long"] - px["medium"] >= 4.0, (x, y, px)


CHAIN = ["base", "link1", "link2", "link3", "link4", "link5", "gripper"]


def _skeleton(q, side):
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


def _hidden(point, segs, radius=0.05):
    cam = np.asarray(station.top_camera().pos)
    return min(_seg_dist(cam, np.asarray(point, float), a, b) for a, b in segs) < radius


@pytest.mark.parametrize("frame", ["first", "last"])
def test_neither_arm_hides_a_square_or_a_staged_block_in_the_first_or_last_top_frame(frame):
    """Rays from the camera to the top of every block (and every square's centre) pass at least 5 cm from both
    arms' link chains, with the arms at the ready pose (first frame) or parked (last frame)."""
    if frame == "first":
        segs = _skeleton(np.asarray(READY), "left") + _skeleton(np.asarray(READY), "right")
        states = start()
    else:
        segs = _skeleton(_played()["left"][1][-1][1], "left") + _skeleton(_played()["right"][1][-1][1], "right")
        states = goal()
    points = [(*T.cell_xy(c), T.BOARD_TOP) for c in T.CELLS]
    for n in T.BLOCKS:
        lo, hi = states[n].low, states[n].high
        points += [(x, y, hi[2]) for x in (lo[0], hi[0]) for y in (lo[1], hi[1])]
    hidden = [p for p in points if _hidden(p, segs)]
    assert hidden == [], hidden[:5]


# -- task.json and the README ---------------------------------------------------------------------------------------

BANNED = ("university", "laboratory", "our team", "sprint", "institute")


def test_task_json_has_every_field_and_states_the_rules():
    meta = json.loads((HERE / "task.json").read_text())
    for key in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                "fidelity", "scores"):
        assert meta.get(key), key
    # scores is filled after the Dream-1 run and media by the upload script
    assert meta["scores"] == "not yet run" or meta["scores"].startswith("Dream-1 run: "), meta["scores"]
    assert meta["category"] == "Impact on AI timelines"
    assert meta["run_limit"].startswith("2,400 steps")
    text = meta["instruction"].lower()
    for word in ("row", "column", "red", "yellow", "blue", "short", "medium", "long"):
        assert word in text, word


def test_no_identifying_names_in_the_folder():
    for f in ("task.py", "task.json", "README.md"):
        body = (HERE / f).read_text().lower()
        for word in BANNED:
            assert word not in body, (f, word)
