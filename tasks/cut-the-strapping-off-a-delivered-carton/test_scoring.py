"""Offline checks of the strap geometry, the cut rule, the scoring rule and the demo plan (no simulator).

    PYTHONPATH=<folder holding yam_arena> python -m pytest -q test_scoring.py
"""

from __future__ import annotations

import json
import math
import pathlib

import numpy as np
import pytest

import task as T
from strapkit import Cut, Cutter, StrapState

HERE = pathlib.Path(__file__).resolve().parent
RIG = T.RIG


# -- the straps -------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("s", [0, 1])
def test_strap_is_a_closed_loop_just_outside_the_carton(s):
    links = RIG.links(s)
    assert len(links) == RIG.n_link == 24
    total = 0.0
    for k, (c, R, length) in enumerate(links):
        nc, nR, nl = links[(k + 1) % len(links)]
        far, near = c + R[:, 2] * length / 2, nc - nR[:, 2] * nl / 2
        assert np.allclose(far, near, atol=1e-9)                      # each link ends where the next begins
        assert np.allclose(R.T @ R, np.eye(3), atol=1e-12) and np.isclose(np.linalg.det(R), 1.0)
        assert abs(c[0] - T.STRAP_X[s]) < 1e-12 and np.allclose(R[:, 0], (1, 0, 0))
        out = c + R[:, 1] * 0.01                                      # the link's y axis points out of the carton
        assert np.linalg.norm(out[1:] - np.asarray(T.CARTON_C)[1:]) > np.linalg.norm(c[1:] - np.asarray(T.CARTON_C)[1:])
        total += length
    assert math.isclose(total, 2 * (T.CARTON[1] + T.CARTON[2]) + 4 * T.STRAP_T, abs_tol=1e-9)
    tops = [c for c, _, _ in links[: RIG.n_top]]
    assert all(math.isclose(c[2], T.ZT + T.STRAP_T / 2, abs_tol=1e-9) for c in tops)


def test_the_sprung_slide_is_under_the_carton_and_cannot_be_cut():
    j = RIG.slide_joint
    below = RIG.links(0)[j][0][2]
    assert below < T.CARTON_C[2] - T.CARTON[2] / 2
    assert j not in RIG.top_joints()


def test_straps_clear_the_boards_and_the_table():
    for (c, size, _) in T.BOARDS:
        for x in T.STRAP_X:
            assert abs(x - c[0]) - size[0] / 2 - T.STRAP_W / 2 > 0.005
    assert T.CARTON_C[2] - T.CARTON[2] / 2 - T.STRAP_T > T.TOP + 0.015


# -- the cutter's shape -------------------------------------------------------------------------------------------------
def test_cutter_matches_the_real_tools_envelope_and_channel():
    lo = np.min([np.asarray(c) - np.asarray(s) / 2 for c, s, _ in T.TOOL_BOXES] + [np.min(T.TOE, axis=0)], axis=0)
    hi = np.max([np.asarray(c) + np.asarray(s) / 2 for c, s, _ in T.TOOL_BOXES] + [np.max(T.TOE, axis=0)], axis=0)
    length, thick, height = hi - lo
    assert 0.150 <= length <= 0.158 and math.isclose(thick, 0.013) and 0.050 <= height <= 0.056
    sole_top = max(c[2] + s[2] / 2 for c, s, _ in T.TOOL_BOXES[3:])
    jaw_under = T.TOOL_BOXES[1][0][2] - T.TOOL_BOXES[1][1][2] / 2
    assert math.isclose(jaw_under - sole_top, 0.004, abs_tol=1e-6)    # the channel: 4 mm cutting depth
    assert sole_top < T.BLADE_C[2] - T.BLADE[2] / 2 and T.BLADE_C[2] + T.BLADE[2] / 2 < jaw_under
    assert T.STRAP_T < jaw_under - sole_top                            # the strap fits the channel
    tip = [p for p in T.TOE if p[0] == 0.0]
    assert max(p[2] for p in tip) <= 0.0005                            # the toe's tip is thinner than the strap


def test_blade_is_out_of_reach_of_a_flat_face():
    """A flat surface under the sole cannot reach the blade: it sits above the sole's top, behind the toe."""
    assert T.BLADE_C[2] - T.BLADE[2] / 2 > 0.002 and T.BLADE_C[0] > 0.022


# -- the cut rule ---------------------------------------------------------------------------------------------------------
def forces(strap=None, link=None, n=0.0):
    f = np.zeros((2, RIG.n_link))
    if strap is not None:
        f[strap, link] = n
    return f


def test_cut_needs_the_force_for_a_tenth_of_a_second():
    c = Cutter(RIG)
    for _ in range(T.CUT_HOLD - 1):
        assert c.step(forces(1, 4, T.CUT_N)) == []
    assert c.step(forces(1, 4, T.CUT_N)) == [(1, 4)]


def test_a_lighter_or_interrupted_press_does_not_cut():
    c = Cutter(RIG)
    for _ in range(200):
        assert c.step(forces(0, 3, T.CUT_N - 0.01)) == []
    for _ in range(5):
        for _ in range(T.CUT_HOLD - 1):
            assert c.step(forces(0, 3, T.CUT_N + 5)) == []
        assert c.step(forces()) == []
    assert c.peak >= T.CUT_N + 5


def test_cut_ends_are_followed_for_half_a_second():
    c = Cutter(RIG)
    pos = np.zeros((2, RIG.n_link, 3))
    c.part(0, 5, (0, 0, 0), 1.0, 6.0, 0.05, [pos[0, 4], pos[0, 5]])
    for i in range(RIG.watch_steps):
        pos[0, 5, 1] += 0.001                                         # one end travels 1 mm a step, the other stays
        c.watch(pos, 1 / 240)
        assert c.cuts[0].watched == (i == RIG.watch_steps - 1)
    assert math.isclose(c.cuts[0].end_move[1], 0.001 * RIG.watch_steps, rel_tol=1e-6) and c.cuts[0].end_move[0] == 0.0
    assert math.isclose(c.cuts[0].end_speed[1], 0.24, rel_tol=1e-6)


def test_carton_is_marked_only_by_a_sustained_press():
    c = Cutter(RIG)
    for _ in range(RIG.dig_hold - 1):
        c.carton(T.DIG_N + 5)
    c.carton(0.0)
    assert not c.dug and c.peak_carton >= T.DIG_N + 5                # a short knock does not count
    for _ in range(RIG.dig_hold):
        c.carton(T.DIG_N)
    assert c.dug
    c.carton(0.0)
    assert c.dug                                                      # and it stays marked


def test_hold_is_the_least_press_over_the_cuts_tenth_of_a_second():
    c = Cutter(RIG)
    spot = [np.array([0.3, 0.07, 0.84]), np.array([0.43, 0.07, 0.84])]
    assert c.held("left_robot", 1, (0.43, -0.02, 0.84)) == (0.0, 9.0)     # nothing logged yet
    for i in range(T.CUT_HOLD):
        c.pressed({"left_robot": (np.array([0.0, 3.0 if i else 0.4]), spot), "right_robot": (np.zeros(2), spot)})
    n, m = c.held("left_robot", 1, (0.43, -0.02, 0.84))
    assert math.isclose(n, 0.4) and math.isclose(m, 0.09)
    c.pressed({"left_robot": (np.array([0.0, 3.0]), spot), "right_robot": (np.zeros(2), spot)})
    assert math.isclose(c.held("left_robot", 1, (0.43, -0.02, 0.84))[0], 3.0)   # the light step has left the window
    assert c.held("right_robot", 1, (0.43, -0.02, 0.84))[0] == 0.0


# -- the scoring rule -----------------------------------------------------------------------------------------------------
def state(cuts=(), tool=None, tool_R=None, tool_v=0.0, carton=None, carton_R=None, dug=False, tcp=None, speed=0.0):
    carton = np.asarray(T.CARTON_C if carton is None else carton, float)
    tool = np.asarray(T.TOOL0 if tool is None else tool, float)
    far = {"left_robot": np.array([0.3, 0.3, T.TOP + 0.25]), "right_robot": np.array([0.3, -0.3, T.TOP + 0.25])}
    z = np.zeros(3)
    return StrapState(pos=carton, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=z, ang_vel=z, low=z, high=z,
                      carton_pos=carton, carton_R=np.eye(3) if carton_R is None else carton_R, carton_vel=np.zeros(6),
                      tool_pos=tool, tool_R=T.HOLD_R if tool_R is None else tool_R,
                      tool_vel=np.r_[tool_v, 0, 0, 0, 0, 0], link_pos=(), link_speed=speed, tcp=tcp or far,
                      cuts=tuple(cuts), dug=dug)


def cut(strap, held=True, watched=True, held_n=None, held_m=None):
    n = (2.5 if held else 0.0) if held_n is None else held_n
    m = (0.09 if held else 9.0) if held_m is None else held_m
    return Cut(strap, 4, np.array([T.STRAP_X[strap], T.Y_CUT, T.ZT]), 10.0, 6.0, 0.06, n, m, (0.004, 0.07), (0.1, 2.0), watched)


@pytest.fixture(autouse=True)
def fresh():
    T._reset_state()


def score(**kw) -> float:
    return T.score({"strapped": state(**kw)})


def test_nothing_done_scores_zero():
    assert score() == 0.0


def test_cutter_lifted_is_a_quarter_and_stays_a_quarter():
    up = np.asarray(T.TOOL0) + (0, 0, T.LIFT_M + 0.001)
    assert score(tool=np.asarray(T.TOOL0) + (0, 0, T.LIFT_M - 0.002)) == 0.0
    assert score(tool=up) == 0.25
    assert score() == 0.25                                            # put back down without cutting anything


def test_one_strap_is_half_and_two_are_three_quarters_until_everything_is_finished():
    assert score(cuts=[cut(1)]) == 0.5
    held = {"left_robot": np.array([0.3, 0.3, T.TOP + 0.25]), "right_robot": np.asarray(T.PICK)}
    assert score(cuts=[cut(1), cut(0)], tcp=held) == 0.75                                   # a gripper still at the cutter
    assert score(cuts=[cut(1), cut(0)], tool=np.asarray(T.TOOL0) + (0, 0, 0.10)) == 0.75      # cutter not put down
    assert score(cuts=[cut(1), cut(0)], tool_v=0.05) == 0.75                                # cutter still moving
    near = {"left_robot": np.asarray(T.CARTON_C) + (0, 0, T.CARTON[2] / 2 + 0.02), "right_robot": np.array([0.3, -0.3, 1.0])}
    assert score(cuts=[cut(1), cut(0)], tcp=near) == 0.75                                   # a gripper 2 cm from the carton
    assert score(cuts=[cut(1), cut(0)]) == 1.0
    assert T.success({"strapped": state(cuts=[cut(1), cut(0)])})


def test_two_cuts_in_one_strap_count_once():
    assert score(cuts=[cut(1), cut(1)]) == 0.5


def test_a_strap_cut_with_nobody_pressing_on_it_voids_the_trial():
    assert score(cuts=[cut(1, held=False)]) == 0.0
    assert score(cuts=[cut(1), cut(0, held=False)]) == 0.0
    assert score(cuts=[cut(1, held_n=T.HOLD_N - 0.05)]) == 0.0         # too light a touch
    assert score(cuts=[cut(1, held_n=T.HOLD_N)]) == 0.5
    assert score(cuts=[cut(1, held_m=T.HOLD_M + 0.01)]) == 0.0         # pressing, but too far from the cut
    assert score(cuts=[cut(1, held_m=T.HOLD_M)]) == 0.5


def test_marking_or_dragging_or_tipping_the_carton_voids_the_trial():
    assert score(cuts=[cut(1), cut(0)], dug=True) == 0.0
    assert score(cuts=[cut(1), cut(0)], carton=np.asarray(T.CARTON_C) + (T.SLID_M + 0.002, 0, 0)) == 0.0
    assert score(cuts=[cut(1), cut(0)], carton=np.asarray(T.CARTON_C) + (T.SLID_M - 0.002, 0, 0)) == 1.0
    a = math.radians(12)
    tip = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    assert score(cuts=[cut(1), cut(0)], carton_R=tip) == 0.0


def test_box_gap():
    assert T.box_gap((0, 0, 0), (0, 0, 0), np.eye(3), (1, 1, 1)) == 0.0
    assert math.isclose(T.box_gap((1.0, 0, 0), (0, 0, 0), np.eye(3), (1, 1, 1)), 0.5)


# -- the demo plan --------------------------------------------------------------------------------------------------------
def test_plan_arms_finish_together_and_grip_within_the_real_grippers_force():
    p = T.plan()
    assert math.isclose(sum(w[0] for w in p["left"]), sum(w[0] for w in p["right"]), abs_tol=1e-6)
    assert sum(w[0] for w in p["right"]) * 20 <= 700
    gaps = {-2 * w[3] for w in p["right"] if w[3] not in (T.OPEN,)}
    assert gaps == {T.HOLD_GAP}
    assert 5000.0 * (T.T_THICK - T.HOLD_GAP) / 2 <= 40.0 + 1e-9       # N a finger at the gripper's 5 N/mm


def test_plan_draws_the_blade_past_each_strap_with_the_other_hand_on_it():
    p = T.plan()
    pulls = [i for i, w in enumerate(p["right"]) if w[4].startswith("draw")]
    assert len(pulls) == 2
    for i, s in zip(pulls, (1, 0)):
        start = np.asarray(p["right"][i - 1][1]) - T.USE_R @ T.GRIP_T
        end = np.asarray(p["right"][i][1]) - T.USE_R @ T.GRIP_T
        assert start[0] > T.STRAP_X[s] + T.STRAP_W / 2                 # the toe's tip starts beyond the strap
        assert end[0] + T.BLADE_C[0] < T.STRAP_X[s] - T.STRAP_W / 2     # and the blade ends past it
        assert math.isclose(start[1], T.Y_CUT) and math.isclose(end[1], T.Y_CUT)
    press = [w for w in p["left"] if w[4].startswith("hold it")]
    assert len(press) == 2 and abs(T.Y_HOLD - T.Y_CUT) <= 0.10


def test_no_hold_plan_leaves_the_left_arm_still():
    assert set(T.plan(hold=False)) == {"right"}


def test_control_plans():
    assert not any(w[4].startswith("draw") for w in T.plan(stop_after=0)["right"])       # pick up and put back
    assert sum(w[4].startswith("draw") for w in T.plan(stop_after=1)["right"]) == 1
    keep = T.plan(keep=True)["right"]
    assert keep[-1][3] == T.GRIP and keep[-1][4].startswith("keep")                      # never lets go


def test_in_use_attitude_is_sole_down_head_away():
    assert np.allclose(T.USE_R[:, 0], (1, 0, 0)) and T.USE_R[2, 2] > 0.98


# -- the written task -------------------------------------------------------------------------------------------------------
def test_task_json_matches_the_code():
    path = HERE / "task.json"
    if not path.exists():
        pytest.skip("task.json not written yet")
    j = json.loads(path.read_text())
    assert j["instruction"] == T.INSTRUCTION
    for key in ("title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"):
        assert j.get(key)
    assert len(j["title"]) <= 120 and len(j["fidelity"]) <= 500 and len(j.get("context", "")) <= 1000 and len(j["scores"]) <= 300
    assert j["category"] == "Impact on jobs"
    for needle in ("1 N", "12 cm", "15 N", "3 cm", "5 cm"):
        assert needle in j["scoring"]
    assert T.INSTRUCTION in (HERE / "README.md").read_text()
