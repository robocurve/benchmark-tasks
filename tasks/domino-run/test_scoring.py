"""Offline checks of task.py (no GPU, no Isaac Sim): every scoring outcome from fabricated domino poses and robot
distances (including attempts to score without doing the task), the robot-distance geometry, the runtime's ending, the
layout, the demo's reach, and that the task's text states the numbers the code uses.

    PYTHONDONTWRITEBYTECODE=1 python -m pytest -p no:cacheprovider test_scoring.py
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
from domkit import rotation  # noqa: E402

T = importlib.import_module("task")

I3 = np.eye(3)
FAR = [T.FAR] * T.N


def _ry(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def _rz(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def flat(i):
    return (np.array(T.START[i]), rotation(T.FLAT_XYZW))


def standing(k, dx=0.0, dy=0.0, r=I3):
    """A domino standing on tick k (0 = nearest the arms), on its short end."""
    r = np.asarray(r, float)
    a = r[:, 2] * (1 if r[2, 2] > 0 else -1)
    base = np.array([T.TICK_X[k] + dx, T.RUN_Y + dy, T.TOP])
    return (base + a * T.DOM_L / 2, r)


def leaning(k, deg, back=False):
    """The domino on tick k turned ``deg`` forward (away from the arms) about the bottom edge it falls over."""
    s = -1 if back else 1
    r = _ry(s * deg)
    pivot = np.array([T.TICK_X[k] + s * T.DOM_T / 2, T.RUN_Y, T.TOP])
    return (pivot + r @ np.array([-s * T.DOM_T / 2, 0.0, T.DOM_L / 2]), r)


def all_standing():
    """Domino i on tick i."""
    return [standing(k) for k in range(T.N)]


def chain_fallen():
    """How the run lies when it has fallen: each leaning on the next at about 76 deg, the last flat."""
    return [leaning(k, 76.0) for k in range(T.N - 1)] + [leaning(T.N - 1, 90.0)]


class Episode:
    """Feeds fabricated steps to the task as the runtime does: observe() (the simulator hook), then success() and
    score(); keeps the runtime's own 1 s success count."""

    def __init__(self):
        T._reset_state()
        self.doms = [flat(i) for i in range(T.N)]
        self.dist = list(FAR)
        self.runtime_held = 0
        self.ended = False
        self.log = []

    def step(self, doms=None, dist=None, n=1):
        if doms is not None:
            self.doms = list(doms)
        if dist is not None:
            self.dist = list(dist)
        s = None
        for _ in range(n):
            T.observe(self.doms, self.dist)
            ok = T.success(None)
            s = T.score(None)
            assert not ok or s == 1.0, "success() true while score() is below 1.0"
            self.runtime_held = self.runtime_held + 1 if ok else 0
            self.ended = self.runtime_held >= T.HOLD_STEPS
            self.log.append((s, ok, self.ended))
            if self.ended:
                break
        return s

    def set_up(self):
        """All six stand on their ticks, untouched, for 0.5 s."""
        return self.step(doms=all_standing(), dist=FAR, n=T.SET_STEPS + 2)

    def fall(self, upto=T.N, touch_first=True, frames=8):
        """Tip the first (touching it), then the dominoes fall one after another, up to ``upto`` of them."""
        doms = list(self.doms)
        if touch_first:
            d = list(FAR)
            d[0] = 0.0
            doms[0] = leaning(0, 5.0)
            self.step(doms=doms, dist=d, n=3)
        for k in range(upto):
            for f in range(1, frames + 1):
                end = 90.0 if k == T.N - 1 else 76.0
                doms[k] = leaning(k, end * f / frames)
                self.step(doms=doms, dist=FAR)
        return self.step(doms=doms, dist=FAR)


def two(i, d):
    out = list(FAR)
    out[i] = d
    return out


# -- partial credit for standing them up -------------------------------------------------------------------------------

def test_start_scores_zero():
    ep = Episode()
    assert ep.step(n=40) == 0.0
    assert all(t is None for t in T._EP["info"]["ticks"])


def test_three_standing_on_their_own_score_a_quarter_after_half_a_second():
    ep = Episode()
    doms = [standing(5), standing(4), standing(3), flat(3), flat(4), flat(5)]
    assert ep.step(doms=doms, n=T.SET_STEPS - 1) == 0.0      # not yet still for 0.5 s
    assert ep.step(n=2) == 0.25
    assert ep.step(doms=[flat(i) for i in range(T.N)], n=30) == 0.25   # kept once earned


def test_two_standing_score_nothing():
    ep = Episode()
    doms = [standing(5), standing(4), flat(2), flat(3), flat(4), flat(5)]
    assert ep.step(doms=doms, n=40) == 0.0


def test_standing_but_held_by_the_robot_does_not_count():
    ep = Episode()
    doms = [standing(5), standing(4), standing(3), flat(3), flat(4), flat(5)]
    assert ep.step(doms=doms, dist=two(2, 0.0005), n=40) == 0.0


def test_all_six_standing_on_their_ticks_score_half():
    ep = Episode()
    assert ep.set_up() == 0.5
    assert T._EP["info"]["set"]


def test_off_the_tick_twisted_or_off_the_line_does_not_count():
    for bad in (standing(2, dx=0.008), standing(2, dy=0.010), standing(2, r=_rz(20.0)),
                standing(2, dx=0.015)):   # halfway between ticks
        ep = Episode()
        doms = all_standing()
        doms[2] = bad
        assert ep.step(doms=doms, n=30) == 0.25
    ep = Episode()
    doms = all_standing()
    doms[2] = standing(2, dx=0.005, dy=0.007, r=_rz(12.0))   # within the tolerances
    assert ep.step(doms=doms, n=30) == 0.5


def test_upside_down_counts_but_on_its_long_edge_or_on_its_face_does_not():
    ep = Episode()
    doms = all_standing()
    doms[1] = standing(1, r=_ry(180.0))
    assert ep.step(doms=doms, n=30) == 0.5
    on_edge = _ry(90.0) @ np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])   # long side along x, 24 mm tall
    for r, h in ((on_edge, T.DOM_W / 2), (_ry(90.0), T.DOM_T / 2)):
        ep = Episode()
        doms = all_standing()
        doms[1] = (np.array([T.TICK_X[1], T.RUN_Y, T.TOP + h]), r)
        assert ep.step(doms=doms, n=30) == 0.25


def test_standing_on_another_domino_does_not_count():
    ep = Episode()
    doms = all_standing()
    p, r = doms[3]
    doms[3] = (p + np.array([0, 0, T.DOM_T]), r)
    assert ep.step(doms=doms, n=30) == 0.25


def test_leaning_dominoes_are_not_standing():
    ep = Episode()
    doms = all_standing()
    doms[4] = leaning(4, 12.0)
    assert ep.step(doms=doms, n=30) == 0.25


# -- the run ---------------------------------------------------------------------------------------------------------

def test_full_run_scores_full_only_after_a_second_and_the_episode_ends_at_full():
    ep = Episode()
    ep.set_up()
    assert ep.fall() == 0.75
    s = ep.step(n=T.HOLD_STEPS - 2)
    assert s < 1.0 or ep.log[-1][1]
    ep.step(n=200)
    assert ep.ended and ep.log[-1] == (1.0, True, True)
    assert all(s == 1.0 for s, ok, _ in ep.log if ok)


def test_episode_can_only_end_at_full_score():
    ep = Episode()
    ep.set_up()
    ep.fall(upto=4)
    ep.step(n=300)
    assert not ep.ended and T.score(None) == 0.75


def test_chain_that_stops_early_scores_three_quarters():
    for upto in (2, 3, 5):
        ep = Episode()
        ep.set_up()
        assert ep.fall(upto=upto) == 0.75
        assert ep.step(n=60) == 0.75


def test_only_the_first_falling_scores_half():
    ep = Episode()
    ep.set_up()
    assert ep.fall(upto=1) == 0.5
    assert ep.step(n=60) == 0.5


def test_touching_any_other_domino_after_the_set_fouls_the_run():
    ep = Episode()
    ep.set_up()
    ep.step(dist=two(3, 0.0), n=1)       # a brush against the fourth, even without moving it
    assert T._EP["info"]["fouled"]
    assert ep.fall() == 0.5
    assert ep.step(n=100) == 0.5


def test_knocking_them_down_one_by_one_by_hand_scores_half():
    ep = Episode()
    ep.set_up()
    doms = list(ep.doms)
    for k in range(T.N):
        for f in range(1, 9):
            doms[k] = leaning(k, (90.0 if k == T.N - 1 else 76.0) * f / 8)
            ep.step(doms=doms, dist=two(k, 0.0))
    assert ep.step(dist=FAR, n=100) == 0.5


def test_touching_the_first_after_the_second_has_started_fouls():
    ep = Episode()
    ep.set_up()
    doms = list(ep.doms)
    doms[0] = leaning(0, 30.0)
    doms[1] = leaning(1, 20.0)
    ep.step(doms=doms, dist=two(0, 0.0))    # still pushing the first while the second falls (a bulldozer)
    assert T._EP["info"]["fouled"]
    ep2 = Episode()
    ep2.set_up()
    doms = list(ep2.doms)
    doms[0] = leaning(0, 30.0)
    doms[1] = leaning(1, 10.0)
    ep2.step(doms=doms, dist=two(0, 0.0))   # the second only nudged so far: allowed
    assert not T._EP["info"]["fouled"]


def test_out_of_order_fouls():
    ep = Episode()
    ep.set_up()
    doms = list(ep.doms)
    doms[2] = leaning(2, 20.0)                # the third falls first (with nothing touching it)
    ep.step(doms=doms)
    assert T._EP["info"]["fouled"]
    assert ep.fall() == 0.5


def test_falling_back_toward_the_arms_does_not_count():
    ep = Episode()
    ep.set_up()
    doms = list(ep.doms)
    doms[0] = leaning(0, 90.0, back=True)
    doms[1] = leaning(1, 90.0, back=True)
    assert ep.step(doms=doms, n=40) == 0.5


def test_hands_near_the_fallen_run_never_score_full():
    ep = Episode()
    ep.set_up()
    ep.fall()
    assert ep.step(dist=[0.015] + [T.FAR] * 5, n=200) == 0.75
    assert ep.step(dist=FAR, n=200) == 1.0       # drawn back: it counts once hands are off


def test_last_one_not_lying_flat_or_run_out_of_the_lane_never_scores_full():
    ep = Episode()
    ep.set_up()
    ep.fall()
    doms = chain_fallen()
    doms[5] = leaning(5, 70.0)
    assert ep.step(doms=doms, n=200) == 0.75
    ep = Episode()
    ep.set_up()
    ep.fall()
    doms = chain_fallen()
    p, r = doms[5]
    doms[5] = (p + np.array([0.0, 0.025, 0.0]), r)     # slid sideways over the tape
    assert ep.step(doms=doms, n=200) == 0.75


def test_still_moving_never_scores_full():
    ep = Episode()
    ep.set_up()
    ep.fall()
    for k in range(200):
        doms = chain_fallen()
        p, r = doms[5]
        doms[5] = (p + np.array([0.003 * math.sin(k / 2), 0, 0]), r)
        ep.step(doms=doms)
    assert T.score(None) == 0.75


def test_laying_them_out_already_fallen_scores_nothing():
    ep = Episode()
    assert ep.step(doms=chain_fallen(), n=300) == 0.0


def test_a_fresh_set_after_a_foul_can_still_score_full():
    ep = Episode()
    ep.set_up()
    ep.step(dist=two(4, 0.0))
    assert T._EP["info"]["fouled"]
    ep.set_up()
    ep.fall()
    ep.step(n=200)
    assert ep.ended and T.score(None) == 1.0


def test_a_domino_that_topples_early_can_be_stood_again():
    ep = Episode()
    doms = all_standing()
    doms[3] = leaning(3, 90.0)                  # knocked over while setting up
    ep.step(doms=doms, n=20)
    assert T.score(None) == 0.25
    ep.set_up()
    ep.fall()
    ep.step(n=200)
    assert ep.ended


# -- geometry ------------------------------------------------------------------------------------------------------------

def _box_hull(centre, size):
    c, h = np.asarray(centre, float), np.asarray(size, float) / 2
    verts = np.array([c + h * np.array([a, b, d]) for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)])
    planes = []
    for i in range(3):
        for s in (-1, 1):
            n = np.zeros(3)
            n[i] = s
            planes.append([*n, n @ c + h[i]])
    return verts, np.array(planes)


def test_robot_distance():
    p, r = standing(2)
    face = T.TICK_X[2] - T.DOM_T / 2
    fingertip = (0.006, 0.010, 0.010)
    for gap in (0.0, 0.0005, 0.005, 0.015, 0.025):
        hull = _box_hull((face - gap - 0.003, 0.0, T.TOP + 0.040), fingertip)
        assert T.robot_distance(p, r, [hull]) == pytest.approx(gap, abs=2e-4)
    # a fingertip pressed 2 mm into the domino's face; one level with its top edge; one far away
    assert T.robot_distance(p, r, [_box_hull((face + 0.001, 0, T.TOP + 0.03), fingertip)]) < 0
    assert T.robot_distance(p, r, [_box_hull((T.TICK_X[2], 0, T.TOP + T.DOM_L + 0.005), fingertip)]) \
        == pytest.approx(0.0, abs=2e-4)
    assert T.robot_distance(p, r, [_box_hull((0.0, 0.3, 1.2), fingertip)]) == T.FAR
    # a large hull whose vertices are all far from the domino but whose face cuts through it still counts as touching
    slab = _box_hull((T.TICK_X[2], 0.0, T.TOP + 0.03), (0.002, 0.30, 0.30))
    assert T.robot_distance(p, r, [slab]) < 0


def test_surface_points_cover_the_domino():
    pts = T.SURFACE
    h = np.array([T.DOM_T, T.DOM_W, T.DOM_L]) / 2
    assert np.allclose(np.abs(pts).max(0), h)
    assert np.all(np.isclose(np.abs(pts), h).any(axis=1))      # all on the surface


def test_tilt_and_heights():
    assert T.tilt_deg(I3) == 0 and T.tilt_deg(_ry(180)) == pytest.approx(0, abs=1e-6)
    assert T.top_height(*standing(0)) == pytest.approx(T.DOM_L)
    # a standing domino tips over past atan(7.5 / 48) = 8.9 deg, so "upright within 10 deg" is "standing on its own"
    assert math.degrees(math.atan(T.DOM_T / T.DOM_L)) < T.UPRIGHT_DEG
    # leaning on an upright neighbour (stopped run): its top 46 mm up, so not "fallen"
    lean = math.asin((T.SPACING - T.DOM_T) / T.DOM_L)
    assert T.top_height(*leaning(0, math.degrees(lean))) > T.DOWN_TOP + 0.01
    # resting on a fallen neighbour (about 76 deg) and lying flat: "fallen"; flat counts as lying flat
    assert T.top_height(*leaning(0, 76.0)) < T.DOWN_TOP - 0.005
    assert T.top_height(*leaning(0, 90.0)) == pytest.approx(T.DOM_T) and T.DOM_T < T.FLAT_TOP
    assert T.fallen_forward(*leaning(0, 76.0), 1.0) and not T.fallen_forward(*leaning(0, 76.0, back=True), 1.0)


# -- the layout ------------------------------------------------------------------------------------------------------

def test_layout():
    assert (T.DOM_T, T.DOM_W, T.DOM_L, T.DOM_MASS) == (0.0075, 0.024, 0.048, 0.010)
    assert np.allclose(np.diff(T.TICK_X), T.SPACING) and T.SPACING == 0.030
    # spacing between 0.5 and 0.75 of the height: each falling domino reaches the next
    assert 0.5 * T.DOM_L <= T.SPACING <= 0.75 * T.DOM_L
    # the ticks inside the lane; the run, fallen, inside it
    assert T.LANE_X0 < T.TICK_X[0] - 0.02 and T.TICK_X[-1] + T.DOM_L + 0.02 < T.LANE_X1
    assert all(T.in_lane(p, r) for p, r in chain_fallen())
    assert all(T.in_lane(p, r) for p, r in all_standing())
    # the flat dominoes clear of the lane and of each other (room for open fingers), on the table
    for i in range(T.N):
        c = T.corners(*flat(i))
        assert np.abs(c[:, 1]).min() > T.LANE_HALF + T.TAPE_W + 0.07
        assert c[:, 2].min() >= T.TOP - 1e-9
    ys = sorted(T.FLAT_Y)
    assert min(np.diff(ys)) - T.DOM_W >= 0.025
    assert [T.tick_of(*s) for s in all_standing()] == list(range(T.N))
    assert T.OBJECTS[0].kinematic and all(o.mass == T.DOM_MASS for o in T.OBJECTS[1:])


# -- the demo ------------------------------------------------------------------------------------------------------------

def test_demo_reaches_every_move_and_fits_the_run_limit():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    D = importlib.import_module("demo")
    plan = D.offline_plan(T)
    assert plan["steps"] + 4 * T.HOLD_STEPS + 60 < 2400
    for arm in ("left", "right"):
        q = np.array(READY, float)
        for pos, rot, label in plan[arm]:
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), q, base_pos=station.BASES[arm], iters=200)
            assert pe < 0.001 and re < math.radians(0.5), (arm, label, pe, re)


def test_demo_grasps_across_the_width_not_the_faces():
    D = importlib.import_module("demo")
    pick, place = D.rotations()
    r_h, _ = D.in_hand_planned(T)
    # in the hand, the domino's width (its y) lies along the fingers' closing line (the TCP's x)
    assert abs((r_h @ np.array([0, 1.0, 0]))[0]) == pytest.approx(1.0)
    # placed with this grip, the domino stands upright, faces square to the lane
    assert np.allclose(place @ r_h, I3, atol=1e-9)


def test_demo_reports_the_real_score():
    src = (HERE / "demo.py").read_text()
    assert '1.0 if success' not in src and 'info.get("score"' in src
    body = src.split("def script", 1)[1].split("def offline_plan", 1)[0]
    assert body.count("ctx.states()") == 1 and "if CALIBRATE:\n            s = ctx.states()" in body


# -- the text states what the code does ------------------------------------------------------------------------------

def test_task_json_states_the_numbers():
    tj = json.loads((HERE / "task.json").read_text())
    limits = {"title": 120, "context": 1000, "scores": 300, "fidelity": 500, "setup": 4000, "scoring": 4000,
              "instruction": 1000, "run_limit": 200}
    for k, n in limits.items():
        assert len(tj[k]) <= n, k
    assert len(json.dumps(tj["keywords"])) <= 200
    assert tj["instruction"] == T.INSTRUCTION
    sc = tj["scoring"]
    for s in ("1.0", "0.75", "0.5", "0.25", "10 degrees", "6 mm", "8 mm", "15 degrees", "3 mm", "30 mm", "10 mm",
              "2 cm", "1 mm", "1 s", "0.5 s"):
        assert s in sc, s
    assert (T.UPRIGHT_DEG, T.ON_TICK, T.OFF_LINE, T.SQUARE_DEG, T.ON_TABLE) == (10.0, 0.006, 0.008, 15.0, 0.003)
    assert (T.START_DEG, T.DOWN_TOP, T.FLAT_TOP, T.CLEAR, T.STILL) == (15.0, 0.030, 0.010, 0.020, 0.001)
    assert (T.SET_STEPS, T.HOLD_STEPS, T.TOUCH) == (10, 20, 0.001)
    st = tj["setup"]
    for s in ("48 x 24 x 7.5 mm", "10 g", "30 mm", "60 mm"):
        assert s in st, s
    text = json.dumps(tj) + (HERE / "README.md").read_text()
    for bad in ("/Users", "/home/", "/tmp", "@"):
        assert bad not in text, bad
