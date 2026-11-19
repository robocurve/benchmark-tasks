# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for hose-quick-connect, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); the tests feed it synthetic
samples. measure() is checked on stand-in link states built from poses.
"""

import math
import pathlib
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402,F401
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control


def sample(**kw):
    """A sample of the finished state: nipple latched, collar forward, plug still, hands clear."""
    m = {"collar": 0.0, "pawl": 0.0015, "fx": -0.002, "tip": 0.026, "tilt": 5.0, "lat": 0.002, "speed": 0.0,
         "engaged": True, "pawl_x": 0.008, "pawl_r": 0.0058,
         "engaged": True, "pawl_x": 0.008, "pawl_r": 0.0058, "clear": {"left": 0.17, "right": 0.10}, "tcp_speed": {"left": 0.0, "right": 0.0}}
    m.update(kw)
    return m


def inserting(**kw):
    """Mid-insertion: collar held back 8.5 mm, nipple 15 mm in, a hand on each part."""
    base = dict(collar=0.0085, pawl=0.004, fx=-0.013, tip=0.015, tilt=2.0, lat=0.001, speed=0.02,
                engaged=False, pawl_x=0.020, pawl_r=0.0078,
                clear={"left": 0.0, "right": 0.0}, tcp_speed={"left": 0.0, "right": 0.01})
    base.update(kw)
    return sample(**base)


def feed(samples, rec=None, t0=0.0):
    rec = rec or T.Record()
    t = t0
    for m in samples:
        t += DT
        rec.update(m, t)
    return rec


def secs(m, s):
    return [m] * int(round(s / DT))


def demo_trace():
    out = secs(sample(collar=0.0, fx=-0.040, tip=-0.012, speed=0.0, clear={"left": 0.2, "right": 0.0},
                      tcp_speed={"left": 0.1, "right": 0.1}), 3.0)
    out += secs(inserting(fx=-0.040, tip=-0.012), 1.0)  # collar back, nipple not yet in
    out += secs(inserting(), 1.0)
    out += secs(inserting(fx=-0.001, tip=0.027, engaged=True), 1.0)  # pawl dropped into the groove, collar still back
    out += secs(sample(collar=0.004, fx=-0.001, tip=0.027, clear={"left": 0.0, "right": 0.0}), 0.5)  # collar closing
    out += secs(sample(fx=-0.001, tip=0.027, clear={"left": 0.05, "right": 0.0}, tcp_speed={"left": 0.1, "right": 0.0}), 0.5)
    out += secs(sample(fx=-0.002, tip=0.026), 2.0)  # both let go, still
    return out


def score(rec):
    return T.ladder(rec)[0]


def test_demo_trace_scores_one():
    r = feed(demo_trace())
    assert score(r) == 1.0, T.ladder(r)


def test_no_steps_scores_zero():
    assert score(T.Record()) == 0.0


def test_doing_nothing_scores_zero():
    r = feed(secs(sample(collar=0.0, fx=-0.07, tip=-0.04, tilt=18.0, lat=0.085), 10))
    assert score(r) == 0.0


def test_pushing_against_the_closed_collar_scores_zero():
    r = feed(secs(inserting(collar=0.0, tip=0.0059), 5) + secs(sample(fx=-0.02, tip=0.006, engaged=False), 3))
    assert score(r) == 0.0


def test_collar_pulled_but_no_nipple_scores_zero():
    r = feed(secs(inserting(tip=-0.012, fx=-0.040), 5) + secs(sample(fx=-0.07, tip=-0.04, engaged=False), 3))
    assert score(r) == 0.0


def test_positions_without_the_pawl_in_the_groove_score_zero():
    """The old full score (flange at the mouth, collar forward, still, hands clear) with the pawl on the shaft scores 0."""
    for kw in ({"engaged": False}, {"pawl_r": 0.0078, "engaged": False}, {"pawl_x": 0.0125, "engaged": False}):
        r = feed(demo_trace()[:-40] + secs(sample(**kw), 3))
        assert score(r) == 0.0 or score(r) == 0.5, kw
    r = feed(secs(sample(engaged=False, fx=-0.002), 6))
    assert score(r) == 0.0, "nothing ever engaged: a fitting merely sitting at the mouth earns nothing"


def test_passing_through_the_groove_for_less_than_half_a_second_scores_zero():
    r = feed(secs(inserting(fx=-0.001, tip=0.027, engaged=True), 0.45) + secs(sample(fx=-0.02, engaged=False), 4))
    assert score(r) == 0.0
    r = feed(secs(inserting(fx=-0.001, tip=0.027, engaged=True), 0.55) + secs(sample(fx=-0.02, engaged=False), 4))
    assert score(r) == 0.5


def test_engagement_is_the_only_gate_on_the_lock():
    r = feed(secs(inserting(fx=-0.001, tip=0.027, engaged=False), 3) + secs(sample(engaged=False), 3))
    assert score(r) == 0.0
    r = feed(secs(inserting(fx=-0.001, tip=0.027, engaged=True), 3))
    assert score(r) == 0.5


def test_engaged_with_collar_still_back_is_half():
    r = feed(secs(inserting(), 1) + secs(inserting(fx=-0.001, tip=0.027, engaged=True), 3))
    assert score(r) == 0.5


def test_locked_collar_forward_but_hands_on_is_three_quarters():
    r = feed(demo_trace()[:80] + secs(inserting(), 1) + secs(inserting(fx=-0.001, tip=0.027, engaged=True), 1)
             + secs(sample(clear={"left": 0.0, "right": 0.0}), 3))
    assert score(r) == 0.75


def test_flange_gap_3_0_seated_3_1_not():
    base = demo_trace()[:-40]
    r = feed(base + secs(sample(fx=-0.0030, tip=0.025), 2))
    assert score(r) == 1.0
    r = feed(base + secs(sample(fx=-0.0031, tip=0.025), 2))
    assert score(r) == 0.5


def test_tilt_12_0_seated_12_1_not():
    base = demo_trace()[:-40]
    assert score(feed(base + secs(sample(tilt=12.0), 2))) == 1.0
    assert score(feed(base + secs(sample(tilt=12.1), 2))) < 1.0


def test_lateral_4_0_seated_4_1_not():
    base = demo_trace()[:-40]
    assert score(feed(base + secs(sample(lat=0.0040), 2))) == 1.0
    assert score(feed(base + secs(sample(lat=0.0041), 2))) < 1.0


def test_collar_forward_1_0_mm_ok_1_1_not():
    base = demo_trace()[:-40]
    assert score(feed(base + secs(sample(collar=0.0010), 2))) == 1.0
    assert score(feed(base + secs(sample(collar=0.0011), 2))) < 1.0


def test_pad_29_mm_restarts_the_release_timer_30_does_not():
    base = demo_trace()[:-40]
    assert score(feed(base + secs(sample(clear={"left": 0.030, "right": 0.030}), 2))) == 1.0
    assert score(feed(base + secs(sample(clear={"left": 0.029, "right": 0.030}), 2))) == 0.75


def test_hand_6_mm_per_s_is_not_still_5_is():
    base = demo_trace()[:-40]
    assert score(feed(base + secs(sample(tcp_speed={"left": 0.0049, "right": 0.0}), 2))) == 1.0
    assert score(feed(base + secs(sample(tcp_speed={"left": 0.0060, "right": 0.0}), 2))) == 0.75


def test_plug_still_3_mm_per_s():
    base = demo_trace()[:-40]
    assert score(feed(base + secs(sample(speed=0.0029), 2))) == 1.0
    assert score(feed(base + secs(sample(speed=0.0031), 2))) == 0.75


def test_release_needs_a_full_second():
    base = demo_trace()[:-40]
    assert score(feed(base + secs(sample(), 0.95))) == 0.75
    assert score(feed(base + secs(sample(), 1.05))) == 1.0


def test_rungs_are_never_lost_by_pulling_the_plug_out_again():
    r = feed(demo_trace() + secs(sample(fx=-0.06, tip=-0.03), 3))
    assert score(r) == 0.5, "engaged once, then pulled out: the rung stays at 0.5, never 0.75 or 1.0"


def test_record_is_clean_after_a_reset():
    r = feed(demo_trace())
    r.reset()
    assert score(r) == 0.0 and r.H.evers == {}


def test_measure_on_stand_in_states():
    def pose(x, y, z, yaw=0.0):
        P = np.eye(4)
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        P[:2, :2] = [[c, -s], [s, c]]
        P[:3, 3] = [x, y, z]
        return P

    tap = SimpleNamespace(poses={"body": pose(0.15, 0.0, 0.7475)}, q={"slide": 0.0085, "pawl_slide": 0.0021})
    hose = SimpleNamespace(poses={"plug": pose(0.15 - 0.012, 0.002, 0.7475 + T.AZ, yaw=4.0)})
    m, _ = T.measure(tap, hose)
    assert abs(m["collar"] - 0.0085) < 1e-9 and abs(m["pawl"] - 0.0021) < 1e-9
    assert abs(m["fx"] + 0.012) < 1e-9
    assert abs(m["tip"] - (-0.012 + 0.028 * math.cos(math.radians(4.0)))) < 1e-9
    assert abs(m["tilt"] - 4.0) < 1e-6 and abs(m["lat"] - 0.002) < 1e-9
    assert m["engaged"] is False  # no pawl pose given


def test_measure_engagement_from_the_pawl_pose():
    def at(x, z, name):
        P = np.eye(4)
        P[:3, 3] = [x, 0.0, z]
        return P

    body = at(0.15, 0.7475, "b")
    plug = at(0.15 - 0.002, 0.7475 + T.AZ, "p")  # flange 2 mm in front of the mouth, on the bore axis

    def engaged(pawl_centre_z):
        tap = SimpleNamespace(poses={"body": body, "pawl": at(0.15 + 0.008, pawl_centre_z, "w")},
                              q={"slide": 0.0, "pawl_slide": 0.0})
        return T.measure(tap, SimpleNamespace(poses={"plug": plug}))[0]

    m = engaged(0.7475 + T.AZ + 0.0085)  # tip 3.5 mm above the axis, in the groove (8 + 2 = 10 mm behind the flange face)
    assert m["engaged"] and abs(m["pawl_r"] - 0.0035) < 1e-9 and abs(m["pawl_x"] - 0.010) < 1e-9
    assert not engaged(0.7475 + T.AZ + 0.0125)["engaged"]  # pawl riding on the shaft: tip 7.5 mm out
    assert engaged(0.7475 + T.AZ + 0.0119)["engaged"] and not engaged(0.7475 + T.AZ + 0.0121)["engaged"]  # 6.9 vs 7.1 mm
    late = SimpleNamespace(poses={"body": body, "pawl": at(0.15 + 0.008, 0.7475 + T.AZ + 0.0085, "w")},
                           q={"slide": 0.0, "pawl_slide": 0.0})
    assert not T.measure(late, SimpleNamespace(poses={"plug": at(0.15 - 0.0045, 0.7475 + T.AZ, "p")}))[0]["engaged"]


def test_plug_speed_from_pose_change():
    def pose(x):
        P = np.eye(4)
        P[:3, 3] = [x, 0.0, 0.7475 + T.AZ]
        return P

    tap = SimpleNamespace(poses={"body": np.eye(4)}, q={"slide": 0.0, "pawl_slide": 0.0})
    tap.poses["body"][:3, 3] = [0.0, 0.0, 0.7475]
    hose = SimpleNamespace(poses={"plug": pose(0.0003)})
    m, _ = T.measure(tap, hose, prev=pose(0.0), dt=0.05)
    assert abs(m["speed"] - 0.006) < 1e-9
