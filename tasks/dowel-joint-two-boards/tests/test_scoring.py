# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for dowel-joint-two-boards, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer reads the final state only (the last seam measurement plus the current release run); no earlier state earns
credit. Most tests feed a Record (a linkkit History fed every control step) synthetic seam measurements. The measure() tests build real board poses.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402,F401
import task as T  # noqa: E402

DT = 0.05
START_A = np.array([0.36, -0.33, 0.7475])
START_B = np.array([0.36, 0.33, 0.7475])
END_A = np.array([0.36, -0.075, 0.7475])
END_B = np.array([0.36, 0.075, 0.7475])


def sample(gap=0.0, gap_l=None, gap_r=None, dx=0.0, dz=0.0, yaw=0.0, pos_a=END_A, pos_b=END_B, clear=0.06,
           tcp_speed=0.0, speed=0.0, tilt=0.0):
    return {"gap_l": gap if gap_l is None else gap_l, "gap_r": gap if gap_r is None else gap_r, "dx": dx, "dz": dz,
            "yaw": yaw,
            "tilt_a": tilt, "tilt_b": tilt, "pos_a": np.array(pos_a, float), "pos_b": np.array(pos_b, float),
            "clear": {"left": clear, "right": clear}, "tcp_speed": {"left": tcp_speed, "right": tcp_speed},
            "speed": speed}


def feed(samples, rec=None, t0=0.0):
    rec = rec or T.Record()
    t = t0
    for m in samples:
        t += DT
        rec.update(m, t)
    return rec


def secs(m, s):
    return [m] * int(round(s / DT))


def demo_trace(final=None):
    """Both boards at their start, slid in, dowels entering evenly, seated, then both hands withdrawn for 2 s."""
    out = secs(sample(gap=0.51, pos_a=START_A, pos_b=START_B, clear=0.0, tcp_speed=0.05), 2.0)
    for g in (0.1, 0.05, 0.019, 0.01, 0.005, 0.0):
        out += secs(sample(gap=g, clear=0.0, tcp_speed=0.01, pos_a=END_A - [0, 0.0 + g / 2, 0],
                           pos_b=END_B + [0, g / 2, 0]), 0.5)
    out += secs(sample(clear=0.0, tcp_speed=0.0), 1.0)
    out += secs(final or sample(clear=0.06), 2.0)
    return out


def test_demo_trace_scores_one():
    rec = feed(demo_trace())
    assert T.ladder(rec)[0] == 1.0, T.ladder(rec)


def test_empty_is_zero():
    assert T.ladder(T.Record())[0] == 0.0


def test_nothing_moved_is_zero():
    rec = feed(secs(sample(gap=0.51, pos_a=START_A, pos_b=START_B, clear=0.2), 20.0))
    assert T.ladder(rec)[0] == 0.0


def test_a_board_pushed_toward_a_static_one_but_not_in_scores_zero():
    """left_only / right_only in the simulator: one board is carried across and stops short (reach), nothing enters."""
    pos_b = np.array([0.36, -0.13, 0.7475])
    rec = feed(secs(sample(gap=0.03, pos_a=START_A, pos_b=pos_b), 5.0))
    assert T.ladder(rec)[0] == 0.0


def test_ladder_rungs():
    both = dict(pos_a=END_A, pos_b=END_B)
    assert T.ladder(feed(secs(sample(gap=0.019, **both), 1.0)))[0] == 0.25
    assert T.ladder(feed(secs(sample(gap=0.010, **both), 1.0)))[0] == 0.5
    assert T.ladder(feed(secs(sample(gap=0.0, clear=0.0, **both), 1.0)))[0] == 0.75


def test_tips_in_limit_19_mm():
    assert T.ladder(feed(secs(sample(gap=0.019), 1.0)))[0] == 0.25
    assert T.ladder(feed(secs(sample(gap=0.0191), 1.0)))[0] == 0.0


def test_tip_rung_needs_the_final_state_not_an_earlier_one():
    """Tips in at some point, then pulled back out: the end state (gap 60 mm) earns nothing."""
    out = secs(sample(gap=0.015), 1.0) + secs(sample(gap=0.060), 1.0)
    assert T.ladder(feed(out))[0] == 0.0
    out = secs(sample(gap=0.005), 1.0) + secs(sample(gap=0.015), 1.0)  # half in earlier, only tips in at the end
    assert T.ladder(feed(out))[0] == 0.25


def test_half_in_limit_10_mm():
    assert T.ladder(feed(secs(sample(gap=0.0100), 1.0)))[0] == 0.5
    assert T.ladder(feed(secs(sample(gap=0.0101), 1.0)))[0] == 0.25


def test_lower_gap_bound_overlapping_boards_score_zero():
    """A negative gap means the seam ends overlap: it cannot score on any rung."""
    for g in (-0.003, -0.02, -0.075, -0.15):
        assert T.ladder(feed(secs(sample(gap=g), 1.0)))[0] == 0.0, g
        assert T.ladder(feed(demo_trace(sample(gap=g))))[0] == 0.0, g
    assert T.ladder(feed(secs(sample(gap_l=0.015, gap_r=-0.004), 1.0)))[0] == 0.0  # one corner overlapped


def test_lower_gap_bound_edge_is_minus_2_mm():
    assert T.ladder(feed(demo_trace(sample(gap=-0.002))))[0] == 1.0
    assert T.ladder(feed(demo_trace(sample(gap=-0.0021))))[0] == 0.0


def test_stacked_boards_score_zero_on_every_rung():
    """One board lying on the other: 18 mm of relative height at any gap, including a seam gap of 0 and 15 mm."""
    for g in (0.0, 0.004, 0.009, 0.015, 0.019, -0.05):
        assert T.ladder(feed(secs(sample(gap=g, dz=0.018), 2.0)))[0] == 0.0, g
    assert T.ladder(feed(demo_trace(sample(dz=0.018))))[0] == 0.0
    assert T.ladder(feed(demo_trace(sample(dz=-0.018))))[0] == 0.0


def test_partial_rungs_need_the_boards_level_within_2_mm():
    for g, rung in ((0.015, 0.25), (0.008, 0.5)):
        assert T.ladder(feed(secs(sample(gap=g, dz=0.002), 1.0)))[0] == rung
        assert T.ladder(feed(secs(sample(gap=g, dz=0.0021), 1.0)))[0] == 0.0
    assert T.ladder(feed(demo_trace(sample(dz=0.002))))[0] == 1.0
    assert T.ladder(feed(demo_trace(sample(dz=0.0021))))[0] < 1.0


def test_partial_rungs_need_flat_boards():
    assert T.ladder(feed(secs(sample(gap=0.015, tilt=3.5), 1.0)))[0] == 0.0
    assert T.ladder(feed(secs(sample(gap=0.008, tilt=3.5), 1.0)))[0] == 0.0
    assert T.ladder(feed(secs(sample(gap=0.015, tilt=3.0), 1.0)))[0] == 0.25


def test_partial_rungs_need_the_boards_square_and_flush():
    assert T.ladder(feed(secs(sample(gap=0.008, dx=0.0011), 1.0)))[0] == 0.0
    assert T.ladder(feed(secs(sample(gap=0.008, yaw=2.1), 1.0)))[0] == 0.0
    assert T.ladder(feed(secs(sample(gap=0.008, dx=0.001, yaw=2.0), 1.0)))[0] == 0.5


def test_seated_limit_half_mm():
    assert T.ladder(feed(demo_trace(sample(gap=0.0005))))[0] == 1.0
    assert T.ladder(feed(demo_trace(sample(gap=0.0006))))[0] < 1.0


def test_flush_limit_1_mm():
    assert T.ladder(feed(demo_trace(sample(dx=0.0010))))[0] == 1.0
    assert T.ladder(feed(demo_trace(sample(dx=0.0011))))[0] < 1.0


def test_a_wedge_that_was_backed_out_and_seated_still_scores_1():
    """One end first (end gaps 2 mm apart with a tip in), backed off and pressed again square: the final state counts."""
    out = secs(sample(gap_l=0.015, gap_r=0.017), 1.0)
    assert T.ladder(feed(out + demo_trace()))[0] == 1.0


def test_a_joint_left_wedged_is_not_seated():
    rec = feed(secs(sample(gap_l=0.0195, gap_r=0.0245), 2.0))
    assert T.ladder(rec)[0] == 0.0
    rec = feed(secs(sample(gap_l=0.015, gap_r=0.017), 2.0))
    assert T.ladder(rec)[0] < 0.75


def test_a_seat_that_was_pulled_apart_again_keeps_nothing():
    """Seated and released earlier, then pulled apart and left at 15 mm: only the final state is scored."""
    out = demo_trace() + secs(sample(gap=0.015, clear=0.06), 2.0)
    assert T.ladder(feed(out))[0] == 0.25
    out = demo_trace() + secs(sample(gap=0.060, clear=0.06), 2.0)
    assert T.ladder(feed(out))[0] == 0.0


def test_release_gate_30_mm():
    ok = feed(demo_trace(sample(clear=0.030)))
    bad = feed(demo_trace(sample(clear=0.0299)))
    assert T.ladder(ok)[0] == 1.0
    assert T.ladder(bad)[0] == 0.75


def test_release_needs_one_second():
    out = demo_trace()[:-int(2.0 / DT)]
    rec = feed(out + secs(sample(clear=0.06), 0.9))
    assert T.ladder(rec)[0] == 0.75
    rec = feed(out + secs(sample(clear=0.06), 1.05))
    assert T.ladder(rec)[0] == 1.0


def test_a_moving_hand_or_board_blocks_the_release():
    assert T.ladder(feed(demo_trace(sample(tcp_speed=0.02))))[0] == 0.75
    assert T.ladder(feed(demo_trace(sample(speed=0.01))))[0] == 0.75


def test_tilted_board_is_not_seated():
    assert T.ladder(feed(demo_trace(sample(tilt=3.5))))[0] < 1.0


def test_record_is_clean_after_reset():
    rec = feed(secs(sample(gap_l=0.015, gap_r=0.030), 1.0))
    assert T.ladder(rec)[0] == 0.0
    rec.reset()
    assert T.ladder(rec)[0] == 0.0
    rec = feed(demo_trace(), rec)
    assert T.ladder(rec)[0] == 1.0


def pose(x, y, deg=0.0):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    P = np.eye(4)
    P[:2, :2] = [[c, -s], [s, c]]
    P[:3, 3] = (x, y, 0.7475)
    return P


def test_measure_seated_and_apart():
    m = T.measure(pose(0.36, -0.075), pose(0.36, 0.075))
    assert abs(m["gap_l"]) < 1e-9 and abs(m["gap_r"]) < 1e-9 and abs(m["dx"]) < 1e-9 and abs(m["yaw"]) < 1e-9
    m = T.measure(pose(0.36, -0.255 - 0.075), pose(0.36, 0.255 + 0.075))
    assert abs(m["gap_l"] - 0.51) < 1e-9 and abs(m["gap_r"] - 0.51) < 1e-9


def test_measure_yaw_makes_the_end_gaps_differ():
    m = T.measure(pose(0.36, -0.075), pose(0.36, 0.075, 3.0))
    assert abs(m["yaw"] - 3.0) < 1e-6
    assert abs(abs(m["gap_l"] - m["gap_r"]) - 0.080 * math.sin(math.radians(3.0))) < 2e-5


def test_measure_sideways_offset():
    m = T.measure(pose(0.36, -0.075), pose(0.363, 0.075))
    assert abs(m["dx"] - 0.003) < 1e-9


def test_measure_height_difference_and_stacking():
    PB = pose(0.36, 0.075)
    PB[2, 3] += 0.018  # the dark board lies on the pale one
    assert abs(T.measure(pose(0.36, -0.075), PB)["dz"] - 0.018) < 1e-9
    assert abs(T.measure(pose(0.36, -0.075), pose(0.36, 0.075))["dz"]) < 1e-9
    stacked = T.measure(pose(0.36, 0.0), PB)  # seam faces 'touching' but stacked and overlapping
    rec = feed(secs(stacked | {"clear": {}, "tcp_speed": {}, "speed": 0.0}, 2.0))
    assert T.ladder(rec)[0] == 0.0


def test_measure_overlapping_boards_have_negative_gaps():
    m = T.measure(pose(0.36, -0.05), pose(0.36, 0.05))  # centres 100 mm apart: 50 mm overlap
    assert abs(m["gap_l"] + 0.050) < 1e-9 and abs(m["gap_r"] + 0.050) < 1e-9
    rec = feed(secs(m | {"clear": {}, "tcp_speed": {}, "speed": 0.0}, 2.0))
    assert T.ladder(rec)[0] == 0.0
