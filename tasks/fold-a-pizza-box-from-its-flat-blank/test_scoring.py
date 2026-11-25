"""Offline checks of the pizza box task: the blank's geometry, the scoring rule and the task file. No simulator.

    python -m pytest -q test_scoring.py
"""
import itertools
import json
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import boxkit as K  # noqa: E402
import task as T  # noqa: E402

LOCK = {n: 90.0 for n in ("wall_l", "wall_r", "tab_l", "tab_r", "front", "cap", "leaf")}
CLOSED = {**LOCK, "back": 90.0, "lid": 90.0, "flap_l": 95.0, "flap_r": 95.0, "flap_f": 95.0}
FAR = [np.array([0.0, 0.0, 0.5]), np.array([0.0, 0.3, 0.5])]


def folds(d):
    f = {n: 0.0 for n in K.ORDER[1:]}
    f.update(d)
    return f


def measure(d, hands=FAR):
    f = folds(d)
    return T.measure(f, K.fk(f), hands)


def score(d, hands=FAR, still=True, on_board=True):
    return T.score_from(measure(d, hands), still, on_board)


def extents(d):
    P = K.fk(folds(d))
    out = {}
    for n in K.ORDER:
        bs = []
        for c, sz in K.BOXES[n]:
            h = np.array(sz) / 2
            pts = np.array([np.array(c) + h * np.array(s) for s in itertools.product((-1, 1), repeat=3)])
            w = K.panel_points(n, P[n], pts)
            bs.append((w.min(0), w.max(0)))
        out[n] = bs
    return out


def overlaps(d):
    X = extents(d)
    bad = []
    for a, b in itertools.combinations(K.ORDER, 2):
        if K.PANELS[a][0] == b or K.PANELS[b][0] == a:
            continue
        for l1, h1 in X[a]:
            for l2, h2 in X[b]:
                if (np.minimum(h1, h2) - np.maximum(l1, l2) > 1e-5).all():
                    bad.append((a, b))
    return bad


# -- the blank ---------------------------------------------------------------------------------------------------------
def test_listed_sizes():
    assert K.S == pytest.approx(0.254) and K.H == pytest.approx(0.0508) and K.T == pytest.approx(0.003175)


def test_thirteen_panels_twelve_creases():
    assert len(K.ORDER) == 13 and len(K.CREASES) == 12
    for n in K.ORDER[1:]:
        assert K.PANELS[n][0] in K.ORDER and K.PANELS[n][4] in K.CREASE


@pytest.mark.parametrize("state", [{}, LOCK, CLOSED])
def test_panels_do_not_overlap(state):
    assert overlaps(state) == []


def test_blank_weighs_about_120_g():
    m = K.masses()
    assert 0.105 < sum(m.values()) < 0.135
    assert min(m.values()) >= K.MIN_MASS


def test_flat_blank_size():
    X = extents({})
    lo = np.min([b[0] for bs in X.values() for b in bs], 0)
    hi = np.max([b[1] for bs in X.values() for b in bs], 0)
    assert 0.69 < hi[0] - lo[0] < 0.71 and 0.350 < hi[1] - lo[1] < 0.360 and hi[2] - lo[2] == pytest.approx(K.T)


def test_feet_go_through_the_slots_when_locked():
    X = extents(LOCK)
    for lo, hi in X["leaf"][1:]:
        assert K.SLOT_X[0] < lo[0] - K.XF and hi[0] - K.XF < K.SLOT_X[1]
        assert K.SLOT_Y[0] < min(abs(lo[1]), abs(hi[1])) and max(abs(lo[1]), abs(hi[1])) < K.SLOT_Y[1]
        assert lo[2] < -K.T / 2                       # the foot's end is below the base


def test_tab_sits_in_its_pocket_when_locked():
    X = extents(LOCK)
    front, leaf, cap = X["front"][0], X["leaf"][0], X["cap"][0]
    for n in ("tab_l", "tab_r"):
        lo, hi = X[n][0]
        assert front[1][0] < lo[0] and hi[0] < leaf[0][0]          # between the front wall and the inner leaf
        assert lo[2] > K.T / 2 and hi[2] < cap[0][2]               # above the base, under the cap


def test_lid_and_flaps_are_inside_when_closed():
    X = extents(CLOSED)
    wall_in = K.YS - K.T / 2
    for n in ("lid", "flap_l", "flap_r", "flap_f"):
        lo, hi = X[n][0]
        assert -wall_in < lo[1] and hi[1] < wall_in
    assert X["flap_f"][0][0][0] > X["leaf"][0][1][0]               # the front flap is behind the inner leaf
    assert X["lid"][0][1][2] <= K.H + K.T / 2 + 1e-6              # the lid's top is level with the walls' tops


def test_creases_rest_part_open():
    k, fs, fd, damp = K.CREASE["main"]
    assert 60.0 < np.degrees(fs / k) < 85.0            # a wall let go rests short of upright
    for kind in ("tab", "roll", "leaf", "flap"):
        k, fs, fd, damp = K.CREASE[kind]
        assert np.degrees(fs / k) > 110.0             # the short perforated creases stay where they are put


def test_fk_matches_extents_for_a_single_wall():
    P = K.fk(folds({"wall_l": 90.0}))
    top = K.panel_points("wall_l", P["wall_l"], (0.0, K.YS + K.H, 0.0))[0]
    assert top == pytest.approx([0.0, K.YS, K.H], abs=1e-9)


def test_box_distance():
    P = K.fk(folds({}))
    assert K.box_distance((0.0, 0.0, 0.05), "base", P["base"]) == pytest.approx(0.05 - K.T / 2)
    assert K.box_distance((0.0, 0.0, 0.0), "base", P["base"]) == 0.0


# -- the rule ----------------------------------------------------------------------------------------------------------
def test_flat_blank_scores_nothing():
    assert score({}) == 0.0


def test_walls_up_without_the_lock_score_nothing():
    assert score({"wall_l": 66.0, "wall_r": 66.0, "tab_l": 90.0, "tab_r": 90.0}) == 0.0
    assert score({"wall_l": 90.0, "wall_r": 90.0, "tab_l": 90.0, "tab_r": 90.0, "front": 90.0}) == 0.0


def test_rolled_over_one_tab_is_a_quarter():
    assert score({**LOCK, "tab_r": 0.0}) == 0.25
    m = measure({**LOCK, "tab_r": 0.0})
    assert m["tabs_under"] == [True, False] and not m["locked"]


def test_leaf_down_but_feet_not_in_is_a_quarter():
    d = {**LOCK, "cap": 60.0, "leaf": 120.0}
    assert measure(d)["feet_in"] == [False, False]
    assert score(d) == 0.25


def test_tabs_outside_the_roll_over_score_nothing():
    d = {**LOCK, "tab_l": 0.0, "tab_r": 0.0}
    assert measure(d)["feet_in"] == [True, True]
    assert score(d) == 0.0


def test_locked_is_a_half():
    m = measure(LOCK)
    assert m["locked"] and m["feet_in"] == [True, True] and m["tabs_under"] == [True, True] and m["standing"]
    assert score(LOCK) == 0.5


def test_a_wall_out_of_plumb_is_not_locked():
    assert score({**LOCK, "wall_l": 90.0 - T.PLUMB - 1.0}) == 0.25
    assert score({**LOCK, "wall_l": 90.0 - T.PLUMB + 1.0}) == 0.5


def test_a_gripper_on_the_walls_keeps_it_at_a_quarter():
    P = K.fk(folds(LOCK))
    on_wall = K.panel_points("front", P["front"], (K.XF - 0.03, 0.0, 0.0))[0] + np.array([-0.01, 0.0, 0.0])
    assert score(LOCK, hands=[on_wall, FAR[1]]) == 0.25


def test_lid_down_with_a_flap_outside_is_three_quarters():
    d = {**CLOSED, "lid": 86.0, "flap_r": 0.0}
    m = measure(d)
    assert m["lid_down"] and not m["lid_shut"] and m["flaps_in"] == [True, False, True]
    assert score(d) == 0.75


def test_lid_open_stays_at_a_half():
    assert score({**CLOSED, "lid": 60.0}) == 0.5


def test_closed_box_scores_one():
    m = measure(CLOSED)
    assert m["lid_shut"] and m["flaps_in"] == [True, True, True] and m["released"]
    assert score(CLOSED) == 1.0


def test_closed_but_not_let_go_or_moving_is_three_quarters():
    P = K.fk(folds(CLOSED))
    on_lid = K.panel_points("lid", P["lid"], (K.XB + K.H + 0.1, 0.0, 0.0))[0] + np.array([0.0, 0.0, 0.02])
    assert score(CLOSED, hands=[on_lid, FAR[1]]) == 0.75
    assert score(CLOSED, still=False) == 0.75


def test_back_wall_leaning_keeps_the_lid_from_counting_as_shut():
    assert not measure({**CLOSED, "back": 90.0 - T.BACK_PLUMB - 2.0, "lid": 90.0 + T.BACK_PLUMB + 2.0})["lid_shut"]


@pytest.mark.parametrize("wall", ["wall_l", "wall_r", "front"])
def test_a_wall_bent_back_or_folded_in_is_not_locked(wall):
    assert score({**CLOSED, wall: -35.0}) == 0.0
    assert score({**CLOSED, wall: 127.0}) == 0.0


def test_a_score_takes_a_set_only_past_upright():
    assert K.crease_set(0.0) == K.crease_set(60.0) == K.crease_set(90.0) == 0.0
    assert K.crease_set(98.0) == pytest.approx(8.0) and K.crease_set(125.0) == pytest.approx(35.0)
    assert K.crease_set(400.0) == K.SET_MAX


def test_a_prebroken_wall_rests_about_upright_and_one_folded_to_upright_does_not():
    k, fs, fd, damp = K.CREASE["main"]
    back = np.degrees(fd / k)                           # how far the spring carries a wall back before friction holds it
    assert abs(K.crease_set(125.0) + back - 90.0) < 15.0
    assert K.crease_set(98.0) + back < 90.0 - 10.0


def test_off_the_board_voids_the_trial():
    assert score(CLOSED, on_board=False) == 0.0


def test_sliding_or_turning_the_box_costs_nothing():
    f = folds(CLOSED)
    P = K.fk(f)
    c, s = np.cos(0.6), np.sin(0.6)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    shift = np.array([0.07, -0.05, 0.0])
    moved = {n: (Rz @ R, Rz @ p + shift) for n, (R, p) in P.items()}
    assert T.score_from(T.measure(f, moved, [h + 1.0 for h in FAR])) == 1.0


def test_scores_are_the_five_tiers_only():
    seen = {score(d) for d in ({}, {**LOCK, "tab_r": 0.0}, LOCK, {**CLOSED, "lid": 86.0, "flap_r": 0.0}, CLOSED)}
    assert seen == {0.0, 0.25, 0.5, 0.75, 1.0}


# -- the task file -----------------------------------------------------------------------------------------------------
def test_layouts():
    assert T.LOT in ("A", "B")
    assert T.BOARD == (0.610, 0.184, 0.038)
    assert T.blank_pos()[2] == pytest.approx(T.TOP + T.BOARD[2] + K.T / 2)
    assert T.board_pos()[0] - T.BOARD[0] / 2 == pytest.approx(T.CENTRE[0] + K.XF + T.BOARD_BACK)


def test_blank_starts_flat_and_scores_zero():
    assert T.START == {}
    assert T.score(None) == 0.0 and not T.success(None)


def test_task_json():
    path = HERE / "task.json"
    if not path.exists():
        pytest.skip("task.json not written yet")
    j = json.loads(path.read_text())
    for key in ("title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"):
        assert j.get(key), key
    assert j["instruction"] == T.INSTRUCTION
    assert len(j["title"]) <= 120 and len(j["fidelity"]) <= 500 and len(j.get("context", "")) <= 1000 and len(j["scores"]) <= 300
    assert j["category"] in ("Impact on jobs", "Societal relatability")
