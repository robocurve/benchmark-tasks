"""Offline checks of task.py (no GPU, no Isaac Sim): the ring mechanism's kinematics and snap, every scoring outcome
from fabricated poses (including attempts to score without doing the task), the runtime's ending, the layout, the
demo's reach, and that the task's text states the numbers the code uses.

    python -m pytest -p no:cacheprovider test_scoring.py
"""

from __future__ import annotations

import importlib
import json
import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

T = importlib.import_module("task")
D = importlib.import_module("demo")

FAR = np.array([[0.0, 0.40, 1.10], [0.0, -0.40, 1.10]])          # grippers well away from everything
NEAR = np.array([[T.XC, T.YM, T.ZC + 0.03]])                       # a gripper point 3 cm over the middle ring
SHUT = T.plate_poses(0.0)
OPEN = T.plate_poses(-T.OPEN_DEG)
START = (np.array(T.CARD_START), T.CARD_START_R)


def ring_centre(plates):
    """The right ring halves' circle centre (world)."""
    pr = plates[1]
    return pr[0] + pr[1] @ (np.array([T.XC, T.YM, T.ZC]) - T.H0)


def card_on_rings(plates, psi_deg, shift=(0.0, 0.0, 0.0)):
    """The card lying along polar angle psi about the right halves' centre with its holes' centres on the wire's
    centre line (the right halves through all three holes), moved by ``shift`` (m, world)."""
    r = D.card_frame(math.radians(psi_deg))
    pos = ring_centre(plates) + (T.RC - T.HOLE_Y) * r[:, 1] + np.asarray(shift)
    return pos, r


class Episode:
    """Feeds fabricated steps to the task as the runtime does: observe() (the simulator hook), then success() and
    score(); keeps the runtime's own 1 s success count."""

    def __init__(self):
        T._reset_state()
        self.plates = SHUT
        self.card = START
        self.grippers = FAR
        self.runtime_held = 0
        self.ended = False
        self.log = []

    def step(self, plates=None, card=None, grippers=None, n=1):
        if plates is not None:
            self.plates = plates
        if card is not None:
            self.card = card
        if grippers is not None:
            self.grippers = grippers
        s = None
        for _ in range(n):
            T.observe(*self.plates, self.card, self.grippers)
            ok = T.success(None)
            s = T.score(None)
            assert not ok or s == 1.0, "success() true while score() is below 1.0"
            self.runtime_held = self.runtime_held + 1 if ok else 0
            self.ended = self.runtime_held >= T.HOLD_STEPS
            self.log.append((s, ok, self.ended))
            if self.ended:
                break
        return s


def filed(ep, psi=185.0):
    """Open the rings, put the card on the right halves, shut the rings with it on."""
    ep.step(plates=OPEN)
    ep.step(card=card_on_rings(OPEN, psi))
    return ep.step(plates=SHUT, card=card_on_rings(SHUT, psi), grippers=NEAR)


# -- the mechanism ---------------------------------------------------------------------------------------------------

def test_rings_shut_at_the_start_and_open_22_mm():
    assert T.tip_gaps(T.wire_points(*SHUT)) == pytest.approx([0.0] * 3, abs=1e-9)
    gaps = T.tip_gaps(T.wire_points(*OPEN))
    assert all(0.021 < g < 0.023 for g in gaps)
    # the three rings stay coaxial: the plates are rigid
    assert len({round(g, 9) for g in gaps}) == 1


def test_over_centre_the_spring_holds_the_rings_shut_or_open():
    """The cover's spring pushes the right plate's outer edge toward the left one: its energy -F y is highest between
    shut and open, so both ends are stable (an over-centre toggle) and the force at the tips peaks at OPEN_FORCE."""
    turns = np.linspace(0.0, -T.OPEN_DEG, 321)
    energy, gaps = [], []
    for b in turns:
        pl, pr = T.plate_poses(b)
        edge = pr[0] + pr[1] @ (T.EDGE_R - T.H0)
        assert edge[2] == pytest.approx(T.EDGE_R[2], abs=1e-9)
        energy.append(-T.SPRING_FORCE * edge[1])
        gaps.append(T.tip_gaps(T.wire_points(pl, pr))[0])
    energy, gaps = np.array(energy), np.array(gaps)
    peak = int(np.argmax(energy))
    assert 0 < peak < len(turns) - 1
    assert turns[peak] == pytest.approx(-T.SHUT_DEG, abs=0.2)          # the plates flat: the centre point
    force = np.diff(energy) / np.diff(gaps)                            # N at the tips, pulling them apart
    assert force.max() == pytest.approx(T.OPEN_FORCE, rel=0.03)         # to open
    assert -force.min() == pytest.approx(T.OPEN_FORCE, rel=0.10)        # to shut again (13 N)
    assert force[0] > 0 and force[-1] < 0                               # pushed shut near shut, open near open


def test_spring_force_hardly_changes_over_the_travel():
    pl, pr = T.plate_poses(-T.SHUT_DEG)
    travel = abs((pr[0] + pr[1] @ (T.EDGE_R - T.H0))[1] - T.EDGE_R[1])
    assert travel < 0.0005 and T.SPRING_K * travel < 0.01 * T.SPRING_FORCE


# -- scoring ---------------------------------------------------------------------------------------------------------

def test_start_scores_zero():
    ep = Episode()
    assert ep.step(n=30) == 0.0
    assert T.rings_through_holes(*START, T.wire_points(*SHUT)) == [False] * 3


def test_rings_part_open_score_nothing():
    ep = Episode()
    assert ep.step(plates=T.plate_poses(-8.0), n=5) == 0.0     # at the centre point: 8 mm apart
    assert ep.step(plates=SHUT) == 0.0


def test_rings_open_score_a_quarter_and_keep_it():
    ep = Episode()
    assert ep.step(plates=OPEN) == 0.25
    assert ep.step(plates=SHUT, n=30) == 0.25


def test_card_on_all_three_open_rings_scores_half():
    ep = Episode()
    ep.step(plates=OPEN)
    for psi in (120.0, 150.0, 185.0):
        assert T.rings_through_holes(*card_on_rings(OPEN, psi), T.wire_points(*OPEN)) == [True] * 3
        assert ep.step(card=card_on_rings(OPEN, psi)) == 0.5
    assert ep.step(card=START, plates=SHUT, n=30) == 0.5       # taken off again: kept


def test_card_on_two_rings_or_beside_them_earns_nothing_more():
    ep = Episode()
    ep.step(plates=OPEN)
    two = card_on_rings(OPEN, 185.0, shift=(0.108, 0.0, 0.0))     # moved one hole along: on two rings only
    assert T.rings_through_holes(*two, T.wire_points(*OPEN)).count(True) == 2
    assert ep.step(card=two) == 0.25
    beside = card_on_rings(OPEN, 185.0, shift=(0.012, 0.0, 0.0))  # the wire would cross 12 mm from every hole
    assert T.rings_through_holes(*beside, T.wire_points(*OPEN)) == [False] * 3
    assert ep.step(card=beside) == 0.25


def test_card_lying_on_the_shut_rings_or_hooked_beyond_the_tips_earns_nothing():
    ep = Episode()
    over = (np.array([T.XC, T.YM - T.CARD_W / 2 + 0.03, T.ZC + T.RC + T.WIRE_R + T.CARD_T]), np.eye(3))
    assert ep.step(card=over, n=5) == 0.0
    # open rings, the card's plane just beyond the right tips (not yet on them)
    ep.step(plates=OPEN)
    c, psi_tip, _ = D.open_geometry(T)
    r = D.card_frame(psi_tip)
    beyond = (c + (T.RC - T.HOLE_Y) * r[:, 1] - 0.004 * r[:, 2], r)
    assert T.rings_through_holes(*beyond, T.wire_points(*OPEN)) == [False] * 3
    assert ep.step(card=beyond) == 0.25


def test_filed_with_a_hand_on_it_scores_three_quarters_never_full():
    ep = Episode()
    assert filed(ep) == 0.75
    assert ep.step(n=60) == 0.75
    assert not ep.ended


def test_filed_and_let_go_scores_full_after_a_second_and_the_episode_ends_at_full():
    ep = Episode()
    filed(ep)
    ep.step(grippers=FAR, n=200)
    scores = [s for s, _, _ in ep.log]
    first = scores.index(1.0)
    assert all(s == 0.75 for s in scores[3:first])
    assert first >= 3 + T.HOLD_STEPS                      # still for 1 s and then held for 1 s
    assert ep.ended and ep.log[-1] == (1.0, True, True)


def test_episode_can_only_end_at_full_score():
    for card, plates in ((START, SHUT), (START, OPEN), (card_on_rings(OPEN, 185.0), OPEN)):
        if True:
            ep = Episode()
            ep.step(plates=plates, card=card, n=200)
            assert not ep.ended and all(not ok for _, ok, _ in ep.log)


def test_card_caught_between_the_tips_never_counts_as_shut():
    ep = Episode()
    ep.step(plates=OPEN)
    ep.step(card=card_on_rings(OPEN, 185.0))
    caught = T.plate_poses(-3.0)                           # the tips held 3 mm apart
    assert ep.step(plates=caught, card=card_on_rings(caught, 185.0), n=60) == 0.5


def test_moving_card_never_scores_full():
    ep = Episode()
    filed(ep)
    for k in range(100):
        ep.step(card=card_on_rings(SHUT, 185.0 + 3.0 * math.sin(k / 3.0)), grippers=FAR)
    assert max(s for s, _, _ in ep.log) == 0.75


def test_gripper_within_2_cm_of_the_rings_never_scores_full():
    ep = Episode()
    filed(ep)
    tip = T.wire_points(*SHUT)[(0, 1)][-1]
    assert ep.step(grippers=np.array([tip + [0.0, 0.0, 0.019]]), n=80) == 0.75
    assert ep.step(grippers=np.array([tip + [0.0, 0.0, 0.025]]), n=80) == 1.0


# -- layout ------------------------------------------------------------------------------------------------------------

def test_layout():
    # the card's holes match the rings: 108 mm apart, 7.1 mm holes on 4 mm wire
    assert np.diff(T.RING_X) == pytest.approx([0.108, 0.108]) and np.diff(T.HOLE_X) == pytest.approx([0.108, 0.108])
    assert 2 * T.HOLE_R - 2 * T.WIRE_R > 0.002
    # real sizes and masses
    assert (T.CARD_L, T.CARD_W) == (0.2794, 0.2159) and T.CARD_MASS == pytest.approx(0.199 * T.CARD_L * T.CARD_W,
                                                                                         rel=0.02)
    assert T.RING_ID == 0.0254 and T.COVER_L == 0.292 and T.SPINE_W == 0.036
    # the card stands in the step file's front slot on its punched edge, leaning on the first divider's top edge,
    # its top above it; the step file is off the binder
    assert T.holes_world(*START)[:, 2].max() < T.CARD_START[2]
    binder_right = T.YM + T.SPINE_GAP - T.COVER_W
    assert T.SORTER_FRONT < binder_right - 0.005
    back = T.CARD_BOTTOM + T.CARD_T / 2 * T.CARD_BACK
    s_div = (T.TOP + T.DIV_H[0] - T.CARD_BOTTOM[2]) / math.cos(math.radians(T.LEAN_DEG))
    assert (back + s_div * T.CARD_UP)[1] - (T.DIV1_Y + T.WIRE_T / 2) == pytest.approx(0.0003, abs=1e-6)
    assert T.CARD_W - s_div > 0.045
    # filed, the card lies on the back cover, inside it
    corners = T.card_corners(*card_on_rings(SHUT, 185.0))
    assert corners[:, 1].min() > binder_right and np.all(np.abs(corners[:, 0] - T.XC) < T.COVER_L / 2)
    # the card's collision pieces leave the holes open: none comes within the hole's radius of a hole's centre
    for s in T.card_shapes():
        if not s.collide:
            continue
        turn = 2 * math.atan2(s.quat[2], s.quat[3])          # the piece's turn about the card's normal
        c, si = math.cos(turn), math.sin(turn)
        for h in T.HOLE_X:
            rel = np.array([h - s.centre[0], T.HOLE_Y - s.centre[1]])
            loc = np.abs(np.array([c * rel[0] + si * rel[1], -si * rel[0] + c * rel[1]]))
            d = np.linalg.norm(np.maximum(loc - np.array(s.size[:2]) / 2, 0.0))
            assert d >= T.HOLE_R - 1e-6, (s, h)


# -- the demo -----------------------------------------------------------------------------------------------------------

def test_demo_reaches_its_working_moves_and_fits_the_run_limit():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    plan = D.offline_plan(T)
    assert plan["steps"] + 3 * T.HOLD_STEPS + 60 < 1800
    for arm in ("left", "right"):
        q = np.array(READY, float)
        for pos, rot, label in plan[arm]:
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), q, base_pos=station.BASES[arm], iters=200)
            if label.startswith("over the"):            # approach hovers: within 2 cm is enough
                assert pe < 0.02, (arm, label, pe)
                continue
            assert pe < 0.002 and re < math.radians(1.0), (arm, label, pe, re)


def test_demo_reports_the_real_score():
    src = (HERE / "demo.py").read_text()
    assert '1.0 if success' not in src and 'info.get("score"' in src
    # the script reads the simulator's state only to print calibration numbers
    for line in src.splitlines():
        if "ctx.states()" in line and "def " not in line:
            assert line.startswith("        ") and ("st = " in line or "s = " in line or "s, sl, sr" in line), line


# -- the text states what the code does ---------------------------------------------------------------------------------

def test_task_json_states_the_numbers():
    tj = json.loads((HERE / "task.json").read_text())
    limits = {"title": 120, "context": 1000, "scores": 300, "fidelity": 500, "setup": 4000, "scoring": 4000,
              "instruction": 1000, "run_limit": 200}
    for k, n in limits.items():
        assert len(tj[k]) <= n, k
    assert len(", ".join(tj["keywords"])) <= 200
    assert tj["instruction"] == T.INSTRUCTION
    sc = tj["scoring"]
    for s in ("1.0", "0.75", "0.5", "0.25", "10 mm", "1 mm", "2 cm", "20 control steps", "all three"):
        assert s in sc, s
    fid = tj["fidelity"]
    for s in ("12 N", "rigid", "0.5 mm", "estimate", "22 mm"):
        assert s in fid, s
    assert T.OPEN_GAP == 0.010 and T.SHUT_GAP == 0.001 and T.CLEAR == 0.02 and T.STILL == 0.001
    assert T.HOLD_STEPS == 20 and T.OPEN_FORCE == 12.0
    text = json.dumps(tj) + (HERE / "README.md").read_text()
    for bad in ("/Users/", "/home/", "/tmp/", "@"):
        assert bad not in text, bad
