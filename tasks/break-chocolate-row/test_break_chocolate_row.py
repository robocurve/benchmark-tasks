"""Scoring and demo checks for break-chocolate-row that run without the simulator:
python -m pytest -q test_break_chocolate_row.py"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

import break_chocolate_row as M

HERE = Path(__file__).resolve().parent
Z3 = np.zeros(3)


def pads(x, y, tips_z) -> M.Pads:
    """A gripper pointing straight down, its pad tips at ``tips_z``."""
    return M.Pads(back=np.array([x, y, tips_z + M.PAD_TIP_PAST_TCP + M.PAD_BACK]), tips=np.array([x, y, tips_z]))


AWAY = {"left_gripper": pads(0.22, 0.25, M.TOP + 0.15), "right_gripper": pads(0.22, -0.30, M.TOP + 0.15)}


def quat_about_x(deg: float) -> np.ndarray:
    h = math.radians(deg) / 2
    return np.array([math.sin(h), 0.0, 0.0, math.cos(h)])


def state(pos, quat=(0, 0, 0, 1.0), size=M.PIECE, moving=False) -> ObjectState:
    pos = np.asarray(pos, float)
    half = np.asarray(size, float) / 2
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.array([0.1, 0, 0]) if moving else Z3,
                       ang_vel=Z3, low=pos - half, high=pos + half)


def plate():
    c = np.array([*M.PLATE_XY, M.TOP + M.PLATE_H / 2])
    return state(c, size=(2 * M.PLATE_R, 2 * M.PLATE_R, M.PLATE_H))


def bar(moves=None, moving=(), **grippers) -> dict:
    """The bar at rest, with some pieces moved: ``moves`` maps a piece's name to (shift in m, tilt about x in degrees).
    Pieces moved by the same shift and tilt stay joined to each other."""
    moves = moves or {}
    out = {"plate": plate()}
    for n in M.NAMES:
        shift, tilt = moves.get(n, ((0.0, 0.0, 0.0), 0.0))
        out[n] = state(M.CENTRES[n] + np.asarray(shift, float), quat_about_x(tilt), moving=n in moving)
    return {**out, **AWAY, **grippers}


def row(i):
    return [M.name(i, k) for k in range(M.COLS)]


def rows(*idx):
    return [n for i in idx for n in row(i)]


def moved(names, shift=(0.0, -0.03, 0.0), tilt=0.0):
    return {n: (shift, tilt) for n in names}


# -- the task file and the bar ---------------------------------------------------------------------------------------

def test_the_task_file_loads():
    task = load(HERE / "break_chocolate_row.py")
    assert task.names == ("plate", *M.NAMES)
    assert [o.name for o in task.objects if getattr(o, "author_joints", False)] == [M.NAMES[-1]]


def test_the_bar_is_a_standard_bar_on_the_plate():
    assert len(M.NAMES) == 12 and M.REAL_PIECE_KG * 12 == pytest.approx(0.043) and M.PIECE_KG == pytest.approx(10 * M.REAL_PIECE_KG)
    xs, ys = [c[0] for c in M.CENTRES.values()], [c[1] for c in M.CENTRES.values()]
    assert max(ys) - min(ys) + M.PIECE[1] == pytest.approx(M.BAR_L)
    assert max(xs) - min(xs) + M.PIECE[0] == pytest.approx(M.BAR_W)
    corners = [np.hypot(x - M.PLATE_XY[0], y - M.PLATE_XY[1]) for x in (min(xs), max(xs)) for y in (min(ys), max(ys))]
    assert max(corners) < M.PLATE_R - 0.04


def test_every_neighbouring_pair_is_joined_once():
    assert len(M.PAIRS) == (M.ROWS - 1) * M.COLS + M.ROWS * (M.COLS - 1) == 17
    assert len({frozenset(p[:2]) for p in M.PAIRS}) == 17


@pytest.mark.parametrize("between", [0, 1, 2])
def test_each_groove_across_the_bar_is_as_strong_as_the_planning_value(between):
    joints = [p for p in M.PAIRS if p[0] in row(between) and p[1] in row(between + 1)]
    assert len(joints) == M.COLS
    assert sum(M.STRENGTH_PER_M * length for *_, length in joints) == pytest.approx(M.GROOVE_TORQUE)


def test_the_long_grooves_have_the_same_strength_per_metre():
    long = [p for p in M.PAIRS if p[0][:-1] == p[1][:-1]]
    assert len(long) == M.ROWS * (M.COLS - 1)
    assert {length for *_, length in long} == {M.PIECE[1]}


# -- scoring --------------------------------------------------------------------------------------------------------

def test_the_whole_bar_scores_0():
    s = bar()
    assert len(M.parts(s)) == 1 and M.score(s) == 0.0 and not M.success(s)


@pytest.mark.parametrize("end", [0, 3])
def test_an_end_row_off_and_set_down_succeeds(end):
    s = bar(moved(row(end), shift=(0, 0.03 if end else -0.03, 0)))
    assert [len(p) for p in M.parts(s)] == [3, 9]
    assert M.success(s) and M.score(s) == M.COMPLETE_SCORE


def test_the_right_break_with_a_hand_still_on_the_row_scores_0_75():
    p = M.CENTRES[M.name(0, 1)] + np.array([0, -0.03, 0])
    s = bar(moved(row(0)), right_gripper=pads(p[0], p[1], p[2] - 0.002))
    assert not M.hands_off(s) and M.score(s) == 0.75 and not M.success(s)


def test_the_right_break_still_moving_scores_0_75():
    s = bar(moved(row(0)), moving=row(0))
    assert M.score(s) == 0.75


def test_a_lifted_row_tilted_up_counts_as_off():
    s = bar(moved(row(0), shift=(0, 0, 0.004), tilt=-20.0))
    assert [len(p) for p in M.parts(s)] == [3, 9]


def test_an_end_row_off_with_the_rest_broken_scores_0_5():
    s = bar({**moved(row(0)), **moved(rows(3), shift=(0, 0.03, 0))})
    assert sorted(len(p) for p in M.parts(s)) == [3, 3, 6] and M.score(s) == 0.5


def test_broken_in_the_middle_scores_0_25():
    s = bar(moved(rows(0, 1)))
    assert [len(p) for p in M.parts(s)] == [6, 6] and M.score(s) == 0.25


def test_a_column_off_along_the_length_scores_0_25():
    col = [M.name(i, 0) for i in range(M.ROWS)]
    s = bar(moved(col, shift=(-0.02, 0, 0)))
    assert [len(p) for p in M.parts(s)] == [4, 8] and M.score(s) == 0.25


def test_one_square_off_the_end_row_scores_0_25():
    s = bar(moved([M.name(0, 0)], shift=(-0.02, -0.02, 0)))
    assert [len(p) for p in M.parts(s)] == [1, 11] and M.score(s) == 0.25


def test_any_piece_off_the_plate_scores_0():
    s = bar(moved(row(0), shift=(0, -0.2, -M.PLATE_H)))
    assert M.score(s) == 0.0 and not M.success(s)


def test_joined_tolerates_settling_but_not_a_visible_gap():
    assert len(M.parts(bar(moved(row(0), shift=(0, -0.001, 0))))) == 1
    assert len(M.parts(bar(moved(row(0), shift=(0, -0.002, 0))))) == 2
    still = (0.0, 0.0, 0.0)
    assert len(M.parts(bar(moved(row(0), shift=still, tilt=2.0)))) == 1  # each piece turned about its own middle
    assert len(M.parts(bar(moved(row(0), shift=still, tilt=6.0)))) == 2


def test_the_whole_bar_moved_together_is_still_one_part():
    s = bar(moved(M.NAMES, shift=(0.01, 0.01, 0.0)))
    assert len(M.parts(s)) == 1


# -- the demo -------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("plan", ["demo", "ends"])
@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_plans_are_within_reach(monkeypatch, plan, arm):
    if plan == "ends":
        monkeypatch.setenv("CHOC_DEMO", "ends")
    p = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, p.duration, 0.05):
        p.action7(t)
    assert p.worst_ik_mm < 0.5 and p.worst_ik_deg < 0.1, (p.worst_ik_mm, p.worst_ik_deg)


def tips_of(w) -> np.ndarray:
    return np.asarray(w.pos, float) + M.PAD_TIP_PAST_TCP * np.asarray(w.yaw, float)[:, 2]


def test_the_demo_holds_the_second_row_beside_the_groove_and_lifts_the_end_row():
    d = M.demo()
    hold = next(w for w in d["left"] if w.label.startswith("pinch"))
    pinch = next(w for w in d["right"] if w.label.startswith("pinch"))
    lift = [w for w in d["right"] if w.label.startswith("bend it up")][-1]
    # the groove is on the midline; the left fingertips just inside row 1, the right ones on the end row
    assert M.GROOVE_Y == pytest.approx(0.0)
    assert M.GROOVE_Y + 0.006 < tips_of(hold)[1] < M.GROOVE_Y + 0.013
    assert tips_of(pinch)[1] == pytest.approx(M.END_ROW_Y)
    for w in (hold, pinch):
        assert tips_of(w)[2] == pytest.approx(M.PLATE_TOP + M.TIPS_UP)
    # the last bend turns the pinch about the groove's line: the fingertips stay as far from the line, and rise
    r0 = np.linalg.norm((tips_of(pinch) - M.GROOVE_LINE)[1:])
    assert np.linalg.norm((tips_of(lift) - M.GROOVE_LINE)[1:]) == pytest.approx(r0)
    assert tips_of(lift)[2] > tips_of(pinch)[2] + 0.001  # 5 degrees about the groove raises them 1.6 mm
    # the left hand holds until the right hand has finished bending the row up
    def until(plan, label):
        labels = [w.label for w in plan]
        return sum(w.duration for w in plan[: labels.index(label)])
    last_bend = [w.label for w in d["right"] if w.label.startswith("bend it up")][-1]
    assert until(d["left"], "let go") >= until(d["right"], last_bend) + d["right"][[w.label for w in d["right"]].index(last_bend)].duration


def test_the_hands_lean_apart_so_the_arms_keep_clear():
    d = M.demo()
    hold = next(w for w in d["left"] if w.label.startswith("pinch"))
    pinch = next(w for w in d["right"] if w.label.startswith("pinch"))

    def body(w):  # a point 12 cm up the hand from its TCP
        return np.asarray(w.pos, float) - 0.12 * np.asarray(w.yaw, float)[:, 2]

    assert body(hold)[1] - body(pinch)[1] > 0.15
    # both still close their fingers along x, across the bar's width: the hand's closing axis is the one that lies
    # along world y for the plain downward grip (which closes along y)
    plain = M.yam_demo.grip_rotation(0.0)
    closing = int(np.argmax(np.abs(plain[1, :])))
    for w in (hold, pinch):
        assert abs(np.asarray(w.yaw, float)[0, closing]) == pytest.approx(1.0)


def test_the_demo_ends_with_both_hands_far_from_the_bar():
    d = M.demo()
    s = bar(moved(row(0)))
    for arm in ("left", "right"):
        x, y, z = d[arm][-1].pos
        s[f"{arm}_gripper"] = pads(x, y, z - M.PAD_TIP_PAST_TCP)
    assert M.hands_off(s)


# -- the joint record from the simulator ----------------------------------------------------------------------------

def between(i, j):
    return {f"{M.name(i, k)}__{M.name(j, k)}" for k in range(M.COLS)}


def test_the_broken_joints_decide_the_parts_not_the_poses():
    whole_looking = bar()  # every piece still where it started
    whole_looking["bar_joints"] = M.BarJoints(frozenset(between(0, 1)))
    assert [len(p) for p in M.parts(whole_looking)] == [3, 9] and M.success(whole_looking)
    whole_looking["bar_joints"] = M.BarJoints(frozenset(between(0, 1) | between(2, 3)))
    assert sorted(len(p) for p in M.parts(whole_looking)) == [3, 3, 6] and M.score(whole_looking) == 0.5


def test_an_intact_joint_record_keeps_the_bar_whole_even_if_a_piece_has_shifted():
    s = bar(moved([M.name(0, 0)], shift=(0, -0.003, 0)))
    s["bar_joints"] = M.BarJoints()
    assert len(M.parts(s)) == 1 and M.score(s) == 0.0


def test_one_square_broken_off_the_end_row_is_not_an_end_row():
    s = bar()
    s["bar_joints"] = M.BarJoints(frozenset({f"{M.name(0, 0)}__{M.name(1, 0)}", f"{M.name(0, 0)}__{M.name(0, 1)}"}))
    assert [len(p) for p in M.parts(s)] == [1, 11] and M.score(s) == 0.25


# -- the grooves: hinges that break past a bend ---------------------------------------------------------------------

def test_each_groove_hinges_about_its_own_line():
    assert M.groove_axis(M.name(0, 1), M.name(1, 1)) == "X"  # between rows: the groove runs across the bar
    assert M.groove_axis(M.name(2, 0), M.name(2, 1)) == "Y"  # between the pieces of a row: along the bar


@pytest.mark.parametrize("length", [M.PIECE[0], M.PIECE[1]])
def test_a_joint_takes_its_share_of_the_groove_strength_at_the_breaking_bend(length):
    assert M.joint_stiffness(length) * math.radians(M.BEND_BREAK_DEG) == pytest.approx(M.STRENGTH_PER_M * length)


def test_the_bend_is_measured_about_the_groove():
    s = bar(moved(row(0), shift=(0, 0, 0), tilt=2.0))  # the end row turned about x
    assert abs(M.bend_deg(s, M.name(0, 1), M.name(1, 1))) == pytest.approx(2.0)
    assert M.bend_deg(s, M.name(1, 0), M.name(1, 1)) == pytest.approx(0.0, abs=1e-9)
    assert abs(M.bend_deg(s, M.name(0, 0), M.name(0, 1))) == pytest.approx(0.0, abs=1e-9)  # both turned alike
