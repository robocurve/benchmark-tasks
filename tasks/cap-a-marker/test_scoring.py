"""Offline checks for the cap-a-marker task (no simulator): the parts' sizes, the snap's geometry and its click force,
every scoring outcome and attempts to score without doing the task, success() only at score 1.0, the demo's reach,
and task.json.

    PYTHONDONTWRITEBYTECODE=1 python -m pytest -p no:cacheprovider test_scoring.py
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task as T  # noqa: E402
from markerkit import rotation  # noqa: E402

R_LYING = rotation(T.BARREL_XYZW)          # the marker lying across the table, tip toward -y
FAR = np.array([[0.0, 0.5, 1.5]])          # gripper points far away


def _rx(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def _lying_marker():
    """The capped marker lying on the table: it rests on the cap (18 mm) and its back end (16.5 mm)."""
    shoulder = np.array([0.30, 0.0, T.TOP + T.CAP_R])
    return shoulder, R_LYING


def _cap_at(shoulder, r_barrel, gap, offset=(0.0, 0.0), tilt_deg=0.0):
    a = r_barrel[:, 2]
    pos = shoulder + gap * a + r_barrel[:, 0] * offset[0] + r_barrel[:, 1] * offset[1]
    return pos, r_barrel @ _rx(tilt_deg)


def _run(steps, bp, br, cp, cr, pts=FAR):
    s = 0.0
    for _ in range(steps):
        s = T.observe(bp, br, cp, cr, pts)
    return s


@pytest.fixture(autouse=True)
def fresh():
    T._reset_state()
    yield


# -- the parts ------------------------------------------------------------------------------------------------------

def test_sizes_match_the_listed_marker():
    assert T.BODY_L + T.CAP_L == pytest.approx(0.1222, abs=0.0005)   # listed 4.81 in long capped
    assert 2 * T.CAP_R + T.CLIP_T == pytest.approx(0.0191, abs=0.0005)  # listed 0.75 in across (cap and clip)
    total = sum(o.mass for o in T.OBJECTS)
    assert total == pytest.approx(0.0091, abs=0.0005)            # listed 0.02 lb
    assert T.TIP_Z < T.CAP_L - T.END_T                           # the nib fits inside the cap


def test_snap_geometry():
    assert T.BORE_R > T.RIDGE_R > T.NOSE_R                       # the bore clears the ridge; the ridge stands proud
    assert T.PAD_IN < T.RIDGE_R                                  # a pad at rest stands inside the ridge ...
    assert T.PAD_IN + T.PAD_TRAVEL > T.RIDGE_R                   # ... and can be pushed out past it
    assert T.PAD_IN > T.NOSE_R                                   # it clears the nose itself
    assert T.PAD_Z + T.PAD_R < T.RIDGE_Z[0]                      # home, the pads sit behind the ridge
    assert T.PAD_Z - T.PAD_R > 0                                 # inside the mouth
    assert T.BORE_R > T.POST_R > T.PAD_IN                        # posted, the pads grip the back end ...
    assert T.POST_DEFLECT < T.PAD_TRAVEL                         # ... within their travel
    for s in T.PADS:
        assert s.preload() == pytest.approx(T.PAD_PRELOAD)


def unclicked_gap():
    """Where a cap pushed on without force stops: its pads against the ridge's front corner."""
    d0 = T.PAD_R - (T.RIDGE_R - T.PAD_IN)
    return T.RIDGE_Z[1] + math.sqrt(T.PAD_R ** 2 - d0 ** 2) - T.PAD_Z


def test_an_unclicked_cap_stops_short_of_home_but_on_the_ridge():
    g = unclicked_gap()
    assert T.GAP_SEATED + 0.002 < g < T.GAP_RIDGE                # about 3 mm: 0.5, never 0.75


def click_force(mu: float) -> float:
    """Quasi-static push (N) needed to drive the three pads over the ridge's front corner (a 1 mm round pad on a square
    corner), the highest along the way."""
    worst = 0.0
    lift = T.RIDGE_R - T.PAD_IN
    for i in range(101):
        dl = lift * i / 100
        d = T.PAD_R - lift + dl
        s = d / T.PAD_R
        c = math.sqrt(max(0.0, 1 - s * s))
        radial = T.PAD_PRELOAD + T.PAD_K * dl
        den = s - mu * c
        if den <= 0:
            return math.inf
        worst = max(worst, len(T.PADS) * radial * (c + mu * s) / den)
    return worst


def test_posted_cap_pull_off_is_about_one_newton():
    radial = T.PAD_PRELOAD + T.PAD_K * T.POST_DEFLECT
    pull = len(T.PADS) * radial * 0.35                           # static friction
    assert 0.8 <= pull <= 2.0


def test_click_force_is_several_newtons():
    assert 4.0 <= click_force(0.3) <= 8.0                         # the friction the parts are given (0.3 dynamic)
    assert click_force(0.0) < click_force(0.3)


# -- the fit -------------------------------------------------------------------------------------------------------

def test_fit_levels():
    sh, r = _lying_marker()
    for gap, level in ((0.0, 0.8), (0.0004, 0.8), (0.0007, 0.6), (unclicked_gap(), 0.6), (0.0039, 0.6),
                       (0.0045, 0.4), (0.0119, 0.4), (0.0125, 0.0), (0.05, 0.0)):
        f = T.cap_fit(sh, r, *_cap_at(sh, r, gap))
        assert T.fit_level(f) == level, gap


def test_off_axis_tilted_reversed_or_posted_cap_earns_nothing():
    sh, r = _lying_marker()
    assert T.fit_level(T.cap_fit(sh, r, *_cap_at(sh, r, 0.0, offset=(0.0025, 0.0)))) == 0.0
    assert T.fit_level(T.cap_fit(sh, r, *_cap_at(sh, r, 0.0, tilt_deg=12.0))) == 0.0
    reversed_pos = sh + T.CAP_L * r[:, 2]                         # closed end at the shoulder, mouth outward
    assert T.fit_level(T.cap_fit(sh, r, reversed_pos, r @ _rx(180))) == 0.0
    # posted: over the back end, its mouth against the back shoulder, pointing back along the barrel
    posted = sh - (T.BODY_L - T.POST_L) * r[:, 2]
    assert T.fit_level(T.cap_fit(sh, r, posted, r @ _rx(180))) == 0.0
    assert T.fit_level(T.cap_fit(sh, r, posted, r)) == 0.0        # (and the other way round)


# -- the score over an episode -------------------------------------------------------------------------------------

def test_full_score_needs_a_second_of_seated_lying_let_go_still():
    sh, r = _lying_marker()
    cp, cr = _cap_at(sh, r, 0.0002)
    scores = [T.observe(sh, r, cp, cr, FAR) for _ in range(60)]
    first = scores.index(1.0)
    # 'still' needs 21 snapshots, then 20 steps of holding
    assert first == T.HOLD_STEPS + T.HOLD_STEPS - 1
    assert all(s == 0.8 for s in scores[:first])
    assert all(s == 1.0 for s in scores[first:])


def test_success_only_when_score_is_one():
    sh, r = _lying_marker()
    cp, cr = _cap_at(sh, r, 0.0002)
    for _ in range(60):
        s = T.observe(sh, r, cp, cr, FAR)
        assert T.success({}) == (T.score({}) == 1.0) == (s == 1.0)


def test_hand_near_marker_blocks_full_score():
    sh, r = _lying_marker()
    cp, cr = _cap_at(sh, r, 0.0)
    near = (sh + np.array([0.0, 0.0, T.CAP_R + 0.015]))[None, :]
    assert _run(80, sh, r, cp, cr, near) == 0.8
    assert T.success({}) is False


def test_held_in_the_air_blocks_full_score():
    sh, r = _lying_marker()
    sh = sh + np.array([0.0, 0.0, 0.05])
    cp, cr = _cap_at(sh, r, 0.0)
    assert _run(80, sh, r, cp, cr) == 0.8


def test_standing_marker_blocks_full_score():
    sh = np.array([0.30, 0.0, T.TOP + T.BODY_L])
    r = _rx(0.0)                                                 # tip up
    cp, cr = _cap_at(sh, r, 0.0)
    assert _run(80, sh, r, cp, cr) == 0.8


def test_moving_blocks_full_score():
    sh, r = _lying_marker()
    for k in range(80):
        p = sh + np.array([0.002 * k, 0.0, 0.0])                  # rolling 2 mm a step
        cp, cr = _cap_at(p, r, 0.0)
        T.observe(p, r, cp, cr, FAR)
    assert T.score({}) == 0.8


def test_best_partial_is_kept_and_full_score_is_lost_if_it_comes_off():
    sh, r = _lying_marker()
    assert _run(5, sh, r, *_cap_at(sh, r, 0.008)) == 0.4
    assert _run(5, sh, r, *_cap_at(sh, r, 0.003)) == 0.6
    assert _run(5, sh, r, *_cap_at(sh, r, 0.03)) == 0.6          # pulled off again: the best stays
    assert _run(60, sh, r, *_cap_at(sh, r, 0.0)) == 1.0
    assert _run(1, sh, r, *_cap_at(sh, r, 0.03)) == 0.8          # pulled off after full score: back to the best fit


def test_nothing_done_scores_zero():
    b = np.array(T.BARREL_START)
    c = np.array(T.CAP_START)
    assert not T.off_back_end(b, rotation(T.BARREL_XYZW), c, rotation(T.CAP_XYZW))   # it starts posted
    assert _run(60, b, rotation(T.BARREL_XYZW), c, rotation(T.CAP_XYZW)) == 0.0
    assert T.success({}) is False


def test_pulling_the_cap_off_earns_the_first_rung_only():
    b, rb = np.array(T.BARREL_START), rotation(T.BARREL_XYZW)
    rc = rotation(T.CAP_XYZW)
    c0 = np.array(T.CAP_START)
    a = rc[:, 2]
    assert _run(3, b, rb, c0 + 0.008 * a, rc) == 0.0             # pulled 8 mm: the back end still 2 mm in
    assert _run(3, b, rb, c0 + 0.0151 * a, rc) == 0.2            # pulled 15 mm: the back end 5 mm clear
    assert _run(60, b, rb, c0, rc) == 0.2                        # posted back again: the rung reached stays, no more
    T._reset_state()
    assert _run(3, b, rb, c0 + np.array([0.0, 0.0, 0.03]), rc) == 0.2   # lifted off sideways


def test_exploit_posted_cap_lying_let_go_scores_zero():
    sh, r = _lying_marker()
    posted = sh - (T.BODY_L - T.POST_L) * r[:, 2]
    assert _run(80, sh, r, posted, r @ _rx(180)) == 0.0


def test_exploit_cap_laid_beside_the_tip_earns_only_the_removal_rung():
    sh, r = _lying_marker()
    cp, cr = _cap_at(sh, r, 0.0, offset=(0.0, 0.020))             # touching end to end but beside the axis
    assert _run(80, sh, r, cp, cr) == 0.2


def test_lying_check():
    sh, r = _lying_marker()
    cp, cr = _cap_at(sh, r, 0.0)
    assert T.lying(sh, r, cp, cr)
    assert not T.lying(sh + np.array([0, 0, 0.004]), r, *_cap_at(sh + np.array([0, 0, 0.004]), r, 0.0))
    r2 = R_LYING @ _rx(-15.0)
    assert not T.lying(sh, r2, *_cap_at(sh, r2, 0.0))


def test_start_layout_posted_and_lying():
    b, c = np.array(T.BARREL_START), np.array(T.CAP_START)
    rb, rc = rotation(T.BARREL_XYZW), rotation(T.CAP_XYZW)
    assert c[2] - T.CAP_R == pytest.approx(T.TOP, abs=0.0005)
    assert rb[:, 2] @ rc[:, 2] == pytest.approx(-1.0)              # the cap points back along the barrel
    assert c - b == pytest.approx(-(T.BODY_L - T.POST_L) * rb[:, 2])   # its mouth against the post's step
    assert T.clip_low(c, rc) > c[2]                                # clip up


def test_lying_on_its_clip_counts():
    sh, r = _lying_marker()                  # in this frame the clip (the cap's -y side) points down
    # resting on the clip and the back end: tipped so both touch the table
    tilt = math.atan2(T.CAP_R + T.CLIP_T - T.BODY_R, T.BODY_L + T.CLIP_Z[1])
    rt = _rx(math.degrees(tilt) * np.sign(r[2, 2] if abs(r[2, 2]) > 0.5 else 1.0)) @ r
    if (rt[:, 2] @ np.array([0, 0, 1.0])) < 0:      # the cap's end must be the raised one
        rt = _rx(-2 * math.degrees(tilt)) @ rt
    sh2 = sh.copy()
    for _ in range(50):                      # settle: lowest clip corner on the table
        cp2, cr2 = _cap_at(sh2, rt, 0.0)
        sh2[2] -= T.clip_low(cp2, cr2) - T.TOP
    cp2, cr2 = _cap_at(sh2, rt, 0.0)
    assert T.clip_low(cp2, cr2) == pytest.approx(T.TOP, abs=0.0003)
    assert T.lying(sh2, rt, cp2, cr2)
    up = r @ np.diag([-1.0, -1.0, 1.0])      # turned half round its axis: clip up, lying on the cap itself
    cp3, cr3 = _cap_at(sh, up, 0.0)
    assert T.clip_low(cp3, cr3) > T.TOP + 0.015
    assert T.lying(sh, up, cp3, cr3)


# -- the demo -------------------------------------------------------------------------------------------------------

def test_demo_moves_are_reachable():
    import demo
    from yam_arena import ik, station
    from yam_arena.constants import READY

    plan = demo.offline_plan(T)
    assert plan["steps"] * 1.0 / 20 < 100                         # well inside the run limit
    for side in ("left", "right"):
        q = np.array(READY, float)
        for pos, rot, label in plan[side]:
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), q, base_pos=station.BASES[side], iters=200)
            assert pe < 0.003 and math.degrees(re) < 1.0, (side, label, pe, re)


# -- task.json --------------------------------------------------------------------------------------------------------

LIMITS = {"title": 120, "context": 1000, "scores": 300, "fidelity": 500, "setup": 4000, "scoring": 4000,
          "instruction": 1000, "run_limit": 200}


def test_task_json():
    j = json.loads((HERE / "task.json").read_text())
    for k in ("title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"):
        assert j.get(k), k
    for k, n in LIMITS.items():
        assert len(j[k]) <= n, (k, len(j[k]))
    assert len(", ".join(j["keywords"])) <= 200
    assert j["instruction"] == T.INSTRUCTION
    assert j["category"] == "Societal relatability"
    text = json.dumps(j)
    for needle in ("0.5 mm", "4 mm", "12 mm", "5 mm", "10 degrees", "2 cm", "1 s", "122 mm", "19 mm", "EXPO"):
        assert needle in text, needle
    for bad in ("/Users/", "/home/", "@"):
        assert bad not in text, bad
