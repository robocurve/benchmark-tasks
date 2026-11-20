# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for gantry-square-stops, pure Python (no simulator). Run from the task folder: pytest -q tests"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

DT = 0.05


def sample(gl=0.0, gr=0.0, speed=0.0, lift=0.0, clear=0.08, tcp=0.0, spread=0.0, dy=0.0, yaw=0.0, lift_l=None,
           lift_r=None):
    """One step. Gaps in mm; ``spread`` makes the corner gaps differ (gmax = g + spread, gmin = g - spread)."""
    return {"gmax": {"left": gl + spread, "right": gr + spread}, "gmin": {"left": gl - spread, "right": gr - spread},
            "lift": {"left": lift if lift_l is None else lift_l, "right": lift if lift_r is None else lift_r},
            "dy": {"left": dy, "right": dy}, "xf": {"left": T.STOP_X - gl * T.MM, "right": T.STOP_X - gr * T.MM},
            "speed": speed, "yaw": yaw, "clear": {"left": clear, "right": clear},
            "tcp_speed": {"left": tcp, "right": tcp}}


def run(samples, n_each=1):
    rec = T.Record()
    t = 0.0
    for m in samples:
        for _ in range(n_each):
            t += DT
            rec.update(m, t)
    return rec


def demo_trace(end=None, hold=30):
    out = [sample(40.1, 42.8), sample(20.0, 20.4, speed=0.012), sample(5.0, 5.0, speed=0.008)]
    return out + [end or sample()] * hold


def score(samples):
    return T.ladder(run(samples))[0]


def pose(x, y_yaw_deg=0.0, z=T.TOP, y=0.0):
    c, s = math.cos(math.radians(y_yaw_deg)), math.sin(math.radians(y_yaw_deg))
    P = np.eye(4)
    P[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    P[:3, 3] = (x, y, z)
    return P


def test_demo_shaped_trace_scores_one():
    assert score(demo_trace()) == 1.0


def test_nothing_scores_zero():
    assert score([sample(40.1, 42.8)] * 40) == 0.0


def test_one_end_only_scores_zero():
    assert score([sample(40.1, 42.8), sample(39.0, 41.0)] * 20) == 0.0


def test_gap_is_the_worst_end():
    assert score(demo_trace(sample(0.0, 0.06))) == 0.5
    assert score(demo_trace(sample(0.05, 0.05))) == 1.0


def test_feeler_limit_is_0_05_mm():
    assert score(demo_trace(sample(0.049, 0.0))) == 1.0
    assert score(demo_trace(sample(0.051, 0.0))) == 0.5


def test_near_rung_is_5_mm():
    assert score(demo_trace(sample(4.9, 4.9))) == 0.5
    assert score(demo_trace(sample(5.1, 5.1))) == 0.25


def test_quarter_needs_both_ends_25_mm_slid():
    assert score([sample(40.0, 42.0), sample(14.0, 16.0), sample(30.0, 30.0)]) == 0.25
    assert score([sample(40.0, 42.0), sample(14.0, 30.0), sample(30.0, 30.0)]) == 0.0


def test_fast_transient_is_not_a_permanent_zero():
    trace = [sample(40.1, 42.8), sample(10.0, 10.0, speed=0.5)] + demo_trace()
    assert score(trace) == 1.0


def test_brief_lift_with_no_travel_is_not_a_permanent_zero():
    trace = [sample(40.1, 42.8), sample(40.1, 42.8, lift=5.0), sample(40.1, 42.8)] + demo_trace()[1:]
    assert score(trace) == 1.0


def test_unseated_end_is_not_full():
    assert score(demo_trace(sample(lift=0.6))) == 0.5
    assert score(demo_trace(sample(lift=0.5))) == 1.0


def test_one_sided_lift_is_not_full():
    assert score(demo_trace(sample(lift_l=0.7, lift_r=0.0))) == 0.5
    assert score(demo_trace(sample(lift_l=0.0, lift_r=0.7))) == 0.5


def test_beam_must_be_still_one_second():
    moving = sample(speed=0.004)
    assert score([sample(40.1, 42.8)] + [moving] * 30) == 0.75
    short = [sample(40.1, 42.8)] + [moving] * 30 + [sample()] * 19
    assert score(short) == 0.75


def test_release_needs_30_mm_and_still():
    assert score(demo_trace(sample(clear=0.029))) == 0.75
    assert score(demo_trace(sample(clear=0.030))) == 1.0
    assert score(demo_trace(sample(tcp=0.006))) == 0.75
    assert score([sample(40.1, 42.8)] + [sample()] * 18) == 0.75


# -- off-rail states ---------------------------------------------------------------------------------------------
def test_lift_carry_cheat_scores_zero():
    """Beam lifted off the rails, carried 40 mm and set down on the stops: no sliding, no credit."""
    trace = [sample(40.1, 42.8)] + [sample(40.1, 42.8, lift=30.0)] * 5 + [sample(20.0, 20.4, lift=30.0)] * 5
    trace += [sample(0.0, 0.0, lift=30.0)] * 5 + [sample()] * 40
    assert score(trace) == 0.0


def test_lift_carry_of_one_end_scores_zero():
    trace = [sample(40.1, 42.8)] + [sample(20.0, 42.8, lift_l=30.0)] * 5 + [sample(0.0, 42.8, lift_l=30.0)] * 5
    trace += [sample(0.0, 0.0)] * 40
    assert score(trace) == 0.0


def test_lifted_carry_of_part_of_the_way_is_capped():
    """Slid 30 mm, then lifted and carried the rest: partial slide credit only, never above 0.25."""
    trace = [sample(40.1, 42.8), sample(25.0, 27.0), sample(10.0, 12.0)]
    trace += [sample(10.0, 12.0, lift=30.0), sample(0.0, 0.0, lift=30.0)] + [sample()] * 40
    assert score(trace) == 0.25


def test_end_off_the_rail_is_not_credited_above_quarter():
    assert score(demo_trace(sample(lift=5.0))) == 0.25
    assert score(demo_trace(sample(dy=1.0))) == 0.25
    assert score(demo_trace(sample(yaw=0.9))) == 0.25


def test_engagement_limits():
    assert T.engaged(sample(lift=1.0), "left") and not T.engaged(sample(lift=1.01), "left")
    assert T.engaged(sample(dy=0.6), "right") and not T.engaged(sample(dy=-0.61), "right")
    assert T.engaged(sample(yaw=-0.45), "left") and not T.engaged(sample(yaw=-0.71), "left")
    assert not T.engaged(sample(gl=300.0), "left")  # behind the rear end of the rail


# -- beyond-stop and face-to-face states -------------------------------------------------------------------------
def test_beyond_the_stop_is_not_full():
    assert score(demo_trace(sample(-0.09, 0.0))) == 1.0
    assert score(demo_trace(sample(-0.11, 0.0))) == 0.5
    assert score(demo_trace(sample(0.0, -0.4))) == 0.5
    assert score(demo_trace(sample(-3.0, 0.0))) == 0.25  # through the stop: off the rail


def test_face_to_face_needs_every_corner():
    assert score(demo_trace(sample(0.0, 0.0, spread=0.04))) == 1.0
    assert score(demo_trace(sample(0.0, 0.0, spread=0.2))) == 0.5  # one corner on the stop, the other 0.4 mm off


def test_signed_gap_cannot_hide_a_far_corner():
    """Mean gap zero but corners +0.3 and -0.3: not squared."""
    assert score(demo_trace(sample(0.0, 0.0, spread=0.3))) == 0.5


def test_reset_clears_the_record():
    rec = run([sample(10.0, 10.0, speed=0.5), sample(9.0, 9.0)])
    assert rec.last is not None and rec.slid["left"] > 0
    rec.reset()
    assert rec.last is None and not rec.H.evers and T.ladder(rec)[0] == 0.0 and rec.slid == {"left": 0.0, "right": 0.0}


# -- measurement from a beam pose --------------------------------------------------------------------------------
def test_measure_pose_start_gaps():
    m = T.measure_pose(pose(T.XC0, T.RACK_DEG))
    assert abs(m["gmax"]["left"] - 40.1) < 0.2 and abs(m["gmax"]["right"] - 42.8) < 0.2
    assert abs(m["gmin"]["left"] - 39.9) < 0.2
    assert abs(m["yaw"] - T.RACK_DEG) < 1e-6 and max(m["lift"].values()) < 1e-6
    assert all(T.engaged(m, s) for s in T.SIDES)


def test_measure_pose_square_at_stops():
    m = T.measure_pose(pose(T.STOP_X - T.BLOCK_L / 2))
    assert abs(m["gmax"]["left"]) < 1e-6 and abs(m["gmin"]["right"]) < 1e-6


def test_measure_pose_racked_at_a_stop_reads_the_feeler_gap():
    m = T.measure_pose(pose(T.STOP_X - T.BLOCK_L / 2 - 0.0005, 0.01))
    assert m["gmax"]["left"] > 0.4 and m["gmax"]["right"] > 0.4


def test_measure_pose_past_the_stop_is_negative():
    m = T.measure_pose(pose(T.STOP_X - T.BLOCK_L / 2 + 0.003))
    assert m["gmax"]["left"] < -2.9 and not T.engaged(m, "left")


def test_measure_pose_height_is_two_sided():
    up = T.measure_pose(pose(T.XC0, z=T.TOP + 0.0008))
    down = T.measure_pose(pose(T.XC0, z=T.TOP - 0.0008))
    assert abs(up["lift"]["left"] - 0.8) < 1e-6 and abs(down["lift"]["left"] - 0.8) < 1e-6
    assert T.measure_pose(pose(T.XC0, z=T.TOP + 0.02))["lift"]["left"] > T.ENGAGE_LIFT_MM


def test_measure_pose_lateral_offset():
    m = T.measure_pose(pose(T.XC0, y=0.001))
    assert abs(m["dy"]["left"] - 1.0) < 1e-6 and not T.engaged(m, "left")


def test_speed_from_pose_change():
    a, b = pose(0.20), pose(0.2004)
    assert abs(T.measure_pose(b, a, 0.05)["speed"] - 0.008) < 1e-9


def test_carriage_is_captured_in_the_scene():
    """The lips sit between the rail neck and the rail head: no straight lift-off path, 0.25 mm on every face."""
    assert abs(T.LIP_Z0 + T.LIP_H - (T.FOOT_H + T.NECK_H - T.FIT)) < 1e-12  # 0.25 mm under the head
    assert abs(T.LIP_Z0 - (T.FOOT_H + T.FIT)) < 1e-12  # 0.25 mm over the foot
    assert abs(T.LIP_TIP - T.NECK_W / 2 - T.FIT) < 1e-12 and T.LIP_TIP < T.RAIL_W / 2  # tip inside the head's edge
    assert abs(T.CHANNEL_W / 2 - T.RAIL_W / 2 - T.FIT) < 1e-12
