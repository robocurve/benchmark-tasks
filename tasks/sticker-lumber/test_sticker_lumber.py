"""Checks of sticker_lumber.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_sticker_lumber.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("sticker_lumber.py"))
M = TASK.module
A, B = M.STICKERS
NEAR, FAR = M.BOLSTER_X
LEFT_BOARD_Y = M.LOWER_Y[1]


def piece(size, x, y, bottom_z, yaw_deg=0.0, tilt_deg=0.0, moving=False):
    """A box of ``size`` with its centre at x, y, turned ``yaw_deg`` about the vertical then tilted ``tilt_deg`` about
    its own long axis, its lowest corner at ``bottom_z``. low/high are built as Arena 0.3.1 builds them."""
    s, t = math.radians(yaw_deg), math.radians(tilt_deg)
    rz = np.array([[math.cos(s), -math.sin(s), 0], [math.sin(s), math.cos(s), 0], [0, 0, 1]])
    rx = np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]])
    rot = rz @ rx
    half = np.array(size) / 2
    corners = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * half @ rot.T
    centre = np.array([x, y, bottom_z - corners[:, 2].min()])
    world = corners + centre
    qz = np.array([0, 0, math.sin(s / 2), math.cos(s / 2)])
    qx = np.array([math.sin(t / 2), 0, 0, math.cos(t / 2)])
    q = _qmul(qz, qx)
    return ObjectState(pos=centre, quat_xyzw=q, lin_vel=np.array([0.05, 0, 0]) if moving else np.zeros(3),
                       ang_vel=np.zeros(3), low=world.min(axis=0), high=world.max(axis=0))


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def sticker_on_course(beam_x, dx=0.0, dy=0.0, yaw_deg=90.0, **kw):
    """A sticker across the course over ``beam_x`` (yaw 90: running left to right)."""
    return piece(M.STICKER, beam_x + dx, M.PILE_Y + dy, M.COURSE_Z, yaw_deg, **kw)


def sticker_at_start(name):
    return piece(M.STICKER, *M.STICKER_START[name], M.TOP, math.degrees(M.STICKER_START_YAW))


def board_on_stickers(dx=0.0, dy=0.0, yaw_deg=0.0, y=LEFT_BOARD_Y, **kw):
    return piece(M.BOARD, M.BOARD_X + dx, y + dy, M.COURSE_Z + M.STICKER[2], yaw_deg, **kw)


def board_at_start():
    return piece(M.BOARD, *M.TOP_BOARD_START, M.TOP)


def scene(a=None, b=None, board=None):
    return {A: a or sticker_at_start(A), B: b or sticker_at_start(B), M.TOP_BOARD: board or board_at_start()}


def done(**kw):
    objects = scene(sticker_on_course(NEAR), sticker_on_course(FAR), board_on_stickers())
    objects.update(kw)
    return objects


def test_the_start_scores_zero():
    assert M.score(scene()) == 0.0 and not M.success(scene())


def test_the_finished_pile_is_success():
    assert M.success(done()) and M.score(done()) == 1.0


def test_either_sticker_may_go_on_either_beam_and_the_board_over_either_tan_board():
    objects = scene(sticker_on_course(FAR), sticker_on_course(NEAR), board_on_stickers(y=M.LOWER_Y[0]))
    assert M.success(objects)


def test_a_board_laid_straight_on_the_course_earns_nothing():
    """Skipping the stickers: the board is on the course, 2 cm lower than it should be."""
    board = piece(M.BOARD, M.BOARD_X, LEFT_BOARD_Y, M.COURSE_Z)
    assert M.score(scene(board=board)) == 0.0


def test_stickers_on_the_table_or_on_the_beam_ends_earn_nothing():
    on_table = piece(M.STICKER, NEAR, 0.06, M.TOP, 90.0)
    on_beam_end = piece(M.STICKER, NEAR, M.PILE_Y + 0.15, M.TOP + M.BOLSTER[2], 90.0)  # off the course, past the end
    assert M.score(scene(on_table, on_beam_end)) == 0.0


@pytest.mark.parametrize(("dx_mm", "expected"), [(4, 0.25), (6, 0.125), (14, 0.125), (16, 0.0)])
def test_alignment_limits(dx_mm, expected):
    assert M.score(scene(a=sticker_on_course(NEAR, dx=dx_mm / 1000))) == expected


def test_a_sticker_at_an_angle_is_judged_by_both_ends():
    """Centre right over the beam, but turned 5 degrees: its ends are 7 mm off, so half credit only."""
    assert M.score(scene(a=sticker_on_course(NEAR, yaw_deg=95.0))) == 0.125
    assert M.score(scene(a=sticker_on_course(NEAR, yaw_deg=91.0))) == 0.25  # ends 1.4 mm off


def test_two_stickers_over_one_beam_count_once():
    assert M.score(scene(sticker_on_course(NEAR), sticker_on_course(NEAR, dx=0.003))) == 0.25


def test_a_sticker_must_bear_on_both_boards():
    """Slid 2.5 cm along its length it still bears 4.5 cm on the near board; slid 6 cm it bears only 1 cm."""
    assert M.score(scene(a=sticker_on_course(NEAR, dy=0.025))) == 0.25
    assert M.score(scene(a=sticker_on_course(NEAR, dy=0.06))) == 0.0


def test_pieces_on_edge_do_not_count_but_upside_down_ones_do():
    on_side = sticker_on_course(NEAR, tilt_deg=90.0)  # square in section: rolled onto another face, it is the same
    assert M.score(scene(a=on_side)) == 0.25
    board_on_edge = board_on_stickers(tilt_deg=90.0)
    assert M.score(done(**{M.TOP_BOARD: board_on_edge})) == 0.5
    upside_down = done(**{A: sticker_on_course(NEAR, tilt_deg=180.0), M.TOP_BOARD: board_on_stickers(tilt_deg=180.0)})
    assert M.score(upside_down) == 1.0
    tipped = board_on_stickers(tilt_deg=15.0)
    assert M.score(done(**{M.TOP_BOARD: tipped})) == 0.5


@pytest.mark.parametrize(("change", "placed"), [
    ({"dx": 0.009}, True), ({"dx": 0.011}, False),
    ({"dy": 0.009}, True), ({"dy": 0.011}, False),
    ({"yaw_deg": 4.0}, True), ({"yaw_deg": 6.0}, False),
    ({"yaw_deg": 180.0}, True), ({"yaw_deg": -176.0}, True),  # end for end looks the same
])
def test_board_limits(change, placed):
    assert M.score(done(**{M.TOP_BOARD: board_on_stickers(**change)})) == (1.0 if placed else 0.5)


def test_a_board_over_the_gap_between_the_tan_boards_is_not_placed():
    assert M.score(done(**{M.TOP_BOARD: board_on_stickers(y=M.PILE_Y)})) == 0.5


def test_the_board_needs_both_stickers_across_the_course():
    """Stickers laid along a tan board (not across the course), with the yellow board resting on them, earn nothing;
    so do stickers across the course but 3 cm off the beams."""
    along = [piece(M.STICKER, M.BOARD_X + dx, LEFT_BOARD_Y, M.COURSE_Z, 0.0) for dx in (-0.05, 0.05)]
    assert M.score(scene(*along, board_on_stickers())) == 0.0
    off_beams = [sticker_on_course(x) for x in (NEAR + 0.03, FAR - 0.03)]
    assert M.score(scene(*off_beams, board_on_stickers())) == 0.0
    one = scene(sticker_on_course(NEAR), sticker_at_start(B), board_on_stickers())
    assert M.score(one) == 0.25


@pytest.mark.parametrize(("xs", "expected"), [
    ((0.25, 0.27), 0.0),  # both between the beams: the pile sags (review of 6219965)
    ((0.22, 0.30), 0.0),  # still between
    ((NEAR, NEAR + 0.003), 0.25),  # both over one beam: the other span is unsupported
    ((NEAR + 0.014, FAR - 0.014), 0.75),  # each within 15 mm of its own beam: half credit each, board counts
    ((NEAR, FAR - 0.016), 0.25),  # one 16 mm off: no board points
    ((NEAR, FAR), 1.0),
])
def test_the_board_counts_only_on_stickers_over_both_beams(xs, expected):
    stickers = [sticker_on_course(x) for x in xs]
    assert M.score(scene(*stickers, board_on_stickers())) == expected


def test_still_moving_is_not_yet_success():
    objects = done(**{M.TOP_BOARD: board_on_stickers(moving=True)})
    assert M.score(objects) == 1.0 and not M.success(objects)


def test_the_demo_ends_with_everything_in_place():
    places = [w.pos[:2] for w in M.demo()["left"] if w.label.endswith("let go")]
    assert places == [pytest.approx((NEAR, M.PILE_Y)), pytest.approx((FAR, M.PILE_Y)),
                      pytest.approx((M.BOARD_X, LEFT_BOARD_Y))]


def test_the_demo_is_within_reach():
    plan = ArmPlan(M.demo()["left"], station.BASES["left"])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
