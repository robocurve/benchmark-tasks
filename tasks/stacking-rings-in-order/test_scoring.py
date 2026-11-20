"""Offline checks of task.py (no GPU, no Isaac Sim): every scoring outcome from fabricated poses (including attempts
to score without doing the task), the runtime's ending, the toy's geometry (the taper really stops a ring that goes on
out of order), the layout, the demo's reach, and that the task's text states the numbers the code uses.

    python -m pytest -p no:cacheprovider test_scoring.py
"""

from __future__ import annotations

import importlib
import json
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

T = importlib.import_module("task")

I3 = np.eye(3)
FAR = np.array([[0.0, 0.40, 1.10], [0.0, -0.40, 1.10]])        # grippers well away from everything
BASE = np.array([T.BASE_XY[0], T.BASE_XY[1], T.TOP + T.BASE_H])
START = [(np.array([x, y, T.TOP + T.RING_H / 2]), I3) for x, y in T.RING_START]


def _rot_x(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def on_post(i, bottom=None, base=(BASE, I3), dx=0.0):
    """Ring i on the post with its bottom ``bottom`` above the base's top face (default: its own seat)."""
    bp, br = base
    b = T.seat(i) if bottom is None else bottom
    return (bp + br @ np.array([dx, 0.0, b + T.RING_H / 2]), br)


class Episode:
    """Feeds fabricated steps to the task as the runtime does: observe() (the simulator hook), then success() and
    score(); keeps the runtime's own 1 s success count."""

    def __init__(self):
        T._reset_state()
        self.base = (BASE.copy(), I3)
        self.rings = list(START)
        self.grippers = FAR
        self.runtime_held = 0
        self.ended = False
        self.log = []

    def step(self, rings=None, base=None, grippers=None, n=1):
        if rings:
            for i, pose in rings.items():
                self.rings[i] = pose
        if base is not None:
            self.base = base
        if grippers is not None:
            self.grippers = grippers
        out = None
        for _ in range(n):
            T.observe(*self.base, self.rings, self.grippers)
            ok = T.success(None)
            s = T.score(None)
            assert not ok or s == 1.0, "success() true while score() is below 1.0"
            self.runtime_held = self.runtime_held + 1 if ok else 0
            self.ended = self.runtime_held >= T.HOLD_STEPS
            self.log.append((s, ok, self.ended))
            out = s
            if self.ended:
                break
        return out


def all_stacked():
    return {i: on_post(i) for i in range(T.N_RINGS)}


# -- the start and partial credit ------------------------------------------------------------------------------------

def test_start_scores_zero():
    ep = Episode()
    assert ep.step(n=50) == 0.0 and not ep.ended


def test_each_ring_in_order_from_the_bottom_scores_015():
    for k in range(T.N_RINGS):
        ep = Episode()
        assert ep.step(rings={i: on_post(i) for i in range(k)}, n=5) == round(0.15 * k, 4)


def test_all_five_in_order_while_still_held_scores_075():
    ep = Episode()
    holding = np.array([on_post(4)[0] + np.array([0.0, T.RING_OD[4] / 2 + 0.003, 0.0])])   # a finger 3 mm off ring 4
    assert ep.step(rings=all_stacked(), grippers=holding, n=100) == 0.6     # the held ring does not count
    near = np.array([on_post(4)[0] + np.array([0.0, T.RING_OD[4] / 2 + 0.010, 0.0])])      # 1 cm off: counts
    assert ep.step(grippers=near, n=100) == 0.75 and not ep.ended


def test_rings_out_of_order_earn_only_the_ordered_part():
    ep = Episode()
    # the largest on the base, then the yellow (third) where it stops on the bare post above it: only the largest
    # counts, and the orange cannot go on below the yellow
    rings = {0: on_post(0), 2: on_post(2, bottom=T.stop_height(2))}
    assert ep.step(rings=rings, n=5) == 0.15
    ep = Episode()
    # smaller ring first: the orange stopped part way down the bare post, the red on top of it -> nothing in order
    rings = {1: on_post(1, bottom=T.stop_height(1)), 0: on_post(0, bottom=T.stop_height(1) + T.RING_H)}
    assert ep.step(rings=rings, n=5) == 0.0


def test_swapped_rings_are_not_counted():
    ep = Episode()
    rings = all_stacked()
    rings[2], rings[3] = on_post(2, bottom=T.seat(3)), on_post(3, bottom=T.seat(2))
    assert ep.step(rings=rings, n=60) == 0.3


def test_a_ring_beside_the_post_at_the_right_height_does_not_count():
    ep = Episode()
    beside = on_post(0, dx=T.RING_ID[0] / 2 + 0.001)     # the post's axis outside its hole (it is not on the post)
    assert ep.step(rings={0: beside}, n=5) == 0.0
    inside = on_post(0, dx=T.RING_ID[0] / 2 - 0.001)
    assert ep.step(rings={0: inside}, n=5) == 0.15


def test_a_ring_held_up_in_the_air_at_its_place_does_not_count():
    ep = Episode()
    held = on_post(0)
    finger = np.array([held[0] + np.array([0.0, T.RING_OD[0] / 2 + 0.002, 0.0])])
    assert ep.step(rings={0: held}, grippers=finger, n=5) == 0.0


def test_gap_or_tilt_beyond_the_limits_does_not_count():
    ep = Episode()
    assert ep.step(rings={0: on_post(0, bottom=0.006)}, n=5) == 0.0       # 6 mm above the base
    assert ep.step(rings={0: on_post(0, bottom=0.004)}, n=5) == 0.15      # 4 mm: counts
    tilted = (on_post(0)[0], _rot_x(11))
    assert ep.step(rings={0: tilted}, n=5) == 0.0


def test_toy_knocked_over_scores_nothing():
    ep = Episode()
    tipped = (BASE.copy(), _rot_x(12))
    rings = {i: on_post(i, base=tipped) for i in range(T.N_RINGS)}
    assert ep.step(base=tipped, rings=rings, n=60) == 0.0 and not ep.ended
    lean = (BASE.copy(), _rot_x(8))      # rocked 8 degrees: still upright enough, everything counts
    rings = {i: on_post(i, base=lean) for i in range(T.N_RINGS)}
    ep2 = Episode()
    assert ep2.step(base=lean, rings=rings, n=10) == 0.75


# -- 1.0 and the runtime's ending -------------------------------------------------------------------------------------

def test_full_stack_left_alone_scores_full_after_a_second_and_the_episode_ends_at_full():
    ep = Episode()
    ep.step(rings=all_stacked())
    first = len(ep.log)
    ep.step(n=200)
    scores = [s for s, _, _ in ep.log[first - 1:]]
    full = 2 * T.HOLD_STEPS - 1      # 21 samples still (20 steps), then held for 20 steps
    assert scores[:full] == [0.75] * full and scores[full] == 1.0
    assert ep.ended and ep.log[-1][0] == 1.0
    assert all(s == 1.0 for s, ok, _ in ep.log if ok)


def test_episode_can_only_end_at_full_score():
    ep = Episode()
    ep.step(rings=all_stacked(), n=25)
    moved = on_post(4)
    ep.step(rings={4: (moved[0] + np.array([0.003, 0, 0]), I3)})    # a 3 mm nudge: the still count starts again
    ep.step(n=200)
    for s, ok, ended in ep.log:
        assert (not ok) or s == 1.0
        assert (not ended) or s == 1.0


def test_hand_near_the_toy_never_scores_full():
    ep = Episode()
    near = np.array([BASE + np.array([T.BASE_R + 0.01, 0.0, -0.02])])   # 1 cm from the base, steadying it
    assert ep.step(rings=all_stacked(), grippers=near, n=150) == 0.75 and not ep.ended


def test_still_rocking_never_scores_full():
    ep = Episode()
    for k in range(150):
        lean = (BASE.copy(), _rot_x(3 * math.sin(k / 3)))
        rings = {i: on_post(i, base=lean) for i in range(T.N_RINGS)}
        ep.step(base=lean, rings=rings)
    assert ep.log[-1][0] == 0.75 and not ep.ended


def test_score_is_taken_at_the_end_not_the_best():
    ep = Episode()
    ep.step(rings={0: on_post(0), 1: on_post(1)}, n=5)
    assert ep.step(rings={1: START[1]}, n=5) == 0.15      # the orange taken off again
    assert T.SCORE_OVER == "end"


# -- the toy -----------------------------------------------------------------------------------------------------------

def test_the_taper_stops_a_ring_that_goes_on_out_of_order():
    for i in range(1, T.N_RINGS):
        stop = T.stop_height(i)
        # alone on the post, a ring stops at least 10 mm short of its own place ...
        assert stop <= T.seat(i) - 0.010, i
        # ... and above where the next larger one would have to go, so that one cannot get under it
        assert stop > T.seat(i - 1) - T.SEAT_TOL, i
        # each larger ring is too big to pass through it: its hole is smaller than every larger ring's outside
        assert all(T.RING_ID[i] < T.RING_OD[j] for j in range(i))
    assert T.stop_height(0) == 0.0          # the largest goes all the way down to the base
    for i in range(T.N_RINGS):              # correctly stacked, each ring clears the post where it sits
        for z in np.arange(T.seat(i), T.seat(i) + T.RING_H, 0.001):
            k = int(z / T.POST_STEP + 1e-9)
            if k < T.N_SECTIONS:
                assert T.section_d(k) < T.RING_ID[i], (i, z)
    # the smallest ring's hole clears the ball on top of the post
    assert T.RING_ID[-1] > 2 * T.TIP_R + 0.005
    # all but the smallest ring are wider than the hand opens (94 mm): they have to be taken by the wall
    assert min(T.RING_OD[:4]) > 0.094


def test_real_sizes_and_layout():
    inch = 0.0254
    assert all(abs(a - b * inch) < 0.0005 for a, b in zip(T.RING_OD, (4.875, 4.5, 4.125, 3.75, 3.5)))   # sourced
    assert abs(T.BASE_H + T.POST_TOP - 7.76 * inch) < 0.002            # the toy stands 197 mm tall (sourced)
    assert abs(T.BASE_H - 1.0 * inch) < 0.001                          # the base 1 in tall (sourced)
    assert abs(T.BASE_MASS + sum(T.RING_MASS) - 8.3 * 0.02835) < 0.001  # 8.3 oz in all (sourced)
    assert T.RING_H == 0.030 and T.DOME_H == 0.012
    # the base's centre of mass is below the rocking dome's centre of curvature, with all rings on: it rights itself
    masses = [T.BASE_MASS] + list(T.RING_MASS)
    zs = [T.BASE_COM_Z] + [T.seat(i) + T.RING_H / 2 for i in range(T.N_RINGS)]
    com = sum(m * z for m, z in zip(masses, zs)) / sum(masses)
    assert com < -T.BASE_H + T.ROCK_R - 0.02
    # rings apart from each other and from the toy, on the table within the arms' reach
    pts = [np.array(p) for p in T.RING_START]
    rad = [od / 2 for od in T.RING_OD]
    for i in range(T.N_RINGS):
        assert np.linalg.norm(pts[i] - np.array(T.BASE_XY)) > rad[i] + T.BASE_R + 0.02
        for j in range(i):
            assert np.linalg.norm(pts[i] - pts[j]) > rad[i] + rad[j] + 0.02
        assert 0.10 < pts[i][0] < 0.45 and abs(pts[i][1]) < 0.25
    # mixed: the rings are not laid out in size order along either table axis
    for ax in (0, 1):
        order = list(np.argsort([p[ax] for p in pts]))
        assert order != sorted(order) and order != sorted(order, reverse=True)


def test_rocker_is_the_drawn_dome():
    toy, rocker = T.OBJECTS[0], T.OBJECTS[1]
    assert rocker.weld_to == "stacker" and rocker.ignore_robots
    assert set(rocker.ignore) == {f"ring_{i}" for i in range(T.N_RINGS)}
    # the sphere's bottom is the dome's bottom, its radius the dome's radius of curvature
    assert abs((rocker.pos[2] - T.ROCK_R) - (toy.pos[2] - T.BASE_H)) < 1e-9
    assert rocker.shapes[0].radius == T.ROCK_R and rocker.weld_at == (0.0, 0.0, T.ROCKER_Z)
    # the dome drawn on the base lies on that sphere
    c = np.array([0.0, 0.0, T.ROCKER_Z])
    for p in T._dome().points[:-1]:
        assert abs(np.linalg.norm(np.array(p) - c) - T.ROCK_R) < 1e-9
    # the base's rim (125 mm across, 12 mm above the bottom) reaches the table only past 21 degrees of tilt
    for deg in np.arange(0.0, 21.0, 0.5):
        a = math.radians(deg)
        rim = np.array([T.BASE_R, -T.BASE_H + T.DOME_H]) - np.array([0.0, T.ROCKER_Z])   # from the centre
        z = -math.sin(a) * rim[0] + math.cos(a) * rim[1]
        assert z > -T.ROCK_R
    # the whole toy's centre of mass stays at BASE_COM_Z
    rest = T.BASE_MASS - T.ROCKER_MASS
    com = (toy.com[2] * rest + T.ROCKER_MASS * T.ROCKER_Z) / T.BASE_MASS
    assert abs(com - T.BASE_COM_Z) < 1e-9 and toy.mass == rest


# -- the demo -----------------------------------------------------------------------------------------------------------

def test_demo_reaches_every_move_and_fits_the_run_limit():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    D = importlib.import_module("demo")
    plan = D.offline_plan(T)
    assert plan["steps"] + 3 * T.HOLD_STEPS + 60 < 2400
    for arm in ("left", "right"):
        q = np.array(READY, float)
        for pos, rot, label in plan[arm]:
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), q, base_pos=station.BASES[arm], iters=200)
            assert pe < 0.001 and re < math.radians(0.5), (arm, label, pe, re)


def test_demo_reports_the_real_score():
    src = (HERE / "demo.py").read_text()
    assert '1.0 if success' not in src and 'info.get("score"' in src
    assert src.count("ctx.states()") == 1 and "if CALIBRATE:\n            yield from wait(0.5)\n            s = ctx.states()" in src


# -- the text states what the code does ---------------------------------------------------------------------------------

def test_task_json_states_the_numbers():
    tj = json.loads((HERE / "task.json").read_text())
    limits = {"title": 120, "context": 1000, "scores": 300, "fidelity": 500, "setup": 4000, "scoring": 4000,
              "instruction": 1000, "run_limit": 200}
    for k, n in limits.items():
        assert len(tj[k]) <= n, k
    assert len(json.dumps(tj["keywords"])) <= 200
    assert tj["instruction"] == T.INSTRUCTION
    assert tj["category"] == "Societal relatability"
    sc = tj["scoring"]
    for s in ("1.0", "0.75", "0.15", "5 mm", "10 degrees", "2 cm", "2 mm", "20 control steps", "1 s"):
        assert s in sc, s
    st = tj["setup"]
    for s in ("124", "114", "105", "95", "89", "30 mm", "60 mm", "32 mm", "196 mm", "169 mm", "235 g"):
        assert s in st, s
    assert T.SEAT_TOL == 0.005 and T.TILT_DEG == 10.0 and T.TOUCH == 0.005 and T.CLEAR == 0.02
    assert T.STILL == 0.002 and T.HOLD_STEPS == 20 and T.PER_RING == 0.15 and T.ALL_FIVE_HELD == 0.75
    text = json.dumps(tj) + (HERE / "README.md").read_text()
    for bad in ("/Users", "/home/", "/tmp", "@"):
        assert bad not in text, bad
