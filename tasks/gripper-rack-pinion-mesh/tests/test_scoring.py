# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for gripper-rack-pinion-mesh, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); the tests feed it
synthetic samples shaped like measure() output.
"""

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control


def sample(sN=20.0, sF=20.0, **kw):
    """A sample of the finished state: far rack seated, both jaws open `s` mm, hands clear."""
    m = {
        "sN": sN, "sF": sF, "F": (T.FAR_X, T.YC0 - sF * T.MM, T.Z0), "N_z": T.Z0, "N_x": T.NEAR_X, "N_tilt": 0.0, "N_yaw": 0.0,
        "tilt": 0.0, "yaw_err": 0.0, "clear_F": {"left": 0.08, "right": 0.06},
        "clear": {"left": 0.08, "right": 0.06}, "tcp_speed": {"left": 0.0, "right": 0.0},
    }
    m.update(kw)
    return m


def feed(samples, rec=None, t0=0.0):
    rec = rec or T.Record()
    t = t0
    for m in samples:
        t += DT
        rec.update(m, t)
    return rec


def pre():
    """A coupled pull earlier in the run: both racks seated, the near rack alone driven 20 mm open, the far rack following."""
    return [sample(float(k), float(k)) for k in range(0, 21)]


def feed_c(samples):
    return feed(pre() + samples)


def secs(m, s):
    return [m] * int(round(s / DT))


def demo_trace():
    """Rest, the near rack closed, the far rack lowered, the jaws pulled open, hands withdrawn, 3 s still."""
    lying = sample(0.0, -30.0, F=(0.172, 0.058, T.Z0), clear={"left": 0.2, "right": 0.2})
    out = secs(lying, 2.0)
    out += secs(sample(1.0, 1.0, clear={"left": 0.0, "right": 0.0}, tcp_speed={"left": 0.0, "right": 0.01}), 1.0)
    out += [sample(1.0 + k, 1.0 + k - 1.0, clear={"left": 0.1, "right": 0.0}) for k in range(0, 21)]
    out += secs(sample(21.0, 20.0, clear={"left": 0.1, "right": 0.0}), 0.5)
    out += secs(sample(21.0, 20.0), 3.0)
    return out


def test_demo_trace_scores_one():
    s, why = T.ladder(feed(demo_trace()))
    assert s == 1.0, why


def test_far_rack_not_seated_scores_zero():
    assert T.ladder(feed(secs(sample(0.0, 0.0, F=(0.172, 0.058, T.Z0)), 3.0)))[0] == 0.0


@pytest.mark.parametrize("dz,ok", [(0.0015, True), (0.0016, False)])
def test_seat_height(dz, ok):
    m = sample(20.0, 20.0, F=(T.FAR_X, T.YC0 - 0.020, T.Z0 + dz))
    assert (T.ladder(feed_c(secs(m, 3.0)))[0] > 0.0) == ok


@pytest.mark.parametrize("dx,ok", [(0.0019, True), (0.0021, False)])
def test_seat_sideways(dx, ok):
    m = sample(20.0, 20.0, F=(T.FAR_X + dx, T.YC0 - 0.020, T.Z0))
    assert (T.ladder(feed_c(secs(m, 3.0)))[0] > 0.0) == ok


@pytest.mark.parametrize("tilt,ok", [(3.0, True), (3.1, False)])
def test_seat_tilt(tilt, ok):
    assert (T.ladder(feed_c(secs(sample(tilt=tilt), 3.0)))[0] > 0.0) == ok


def test_one_tooth_off_is_scored_by_the_final_state_only():
    rec = feed(secs(sample(0.0, 9.42), 1.0))  # a tooth off for 1 s: offset 4.71 mm
    assert T.ladder(rec)[0] == 0.0  # not coupled yet
    assert T.ladder(feed(demo_trace(), rec, t0=1.0))[0] == 1.0  # undone and redone: the final state counts


def test_one_tooth_off_pulled_open_is_a_quarter():
    out = [sample(1.0 + k, 1.0 + k - 9.42) for k in range(0, 24)] + secs(sample(24.0, 14.6), 3.0)
    assert T.ladder(feed(out))[0] == 0.25


def test_near_rack_lifted_out_and_set_beside_the_housing_scores_zero():
    # far rack seated, jaws 20 mm each, but the near rack lies on the table 48 mm to the side, sN reading 20 mm
    beside = sample(20.0, 20.0, N_x=T.NEAR_X - 0.048, N_z=T.Z0 - 0.004)
    s, why = T.ladder(feed(demo_trace()[:-90] + secs(beside, 3.0)))
    assert s == 0.0 and "near rack" in why


@pytest.mark.parametrize("dz,ok", [(0.0015, True), (0.0016, False)])
def test_near_rack_seat_height(dz, ok):
    m = sample(N_z=T.Z0 + dz)  # right place, riding up out of its channel
    assert (T.ladder(feed(demo_trace()[:-60] + secs(m, 3.0)))[0] > 0.0) == ok


@pytest.mark.parametrize("dx,ok", [(0.0019, True), (0.0021, False)])
def test_near_rack_sideways(dx, ok):
    m = sample(N_x=T.NEAR_X + dx)
    assert (T.ladder(feed(demo_trace()[:-60] + secs(m, 3.0)))[0] > 0.0) == ok


@pytest.mark.parametrize("tilt,ok", [(3.0, True), (3.1, False)])
def test_near_rack_tilt(tilt, ok):
    m = sample(N_tilt=tilt)
    assert (T.ladder(feed(demo_trace()[:-60] + secs(m, 3.0)))[0] > 0.0) == ok


def test_far_rack_sunk_below_the_floor_is_not_seated():
    m = sample(F=(T.FAR_X, T.YC0 - 0.020, T.Z0 - 0.0016))
    assert T.ladder(feed(secs(m, 3.0)))[0] == 0.0


def test_independently_positioned_racks_score_zero():
    # both racks seated and both 20 mm open, but each was dragged by its own hand: a pad within 5 mm of the far rack
    hand = {"left": 0.002, "right": 0.06}
    out = [sample(1.0 + k, 1.0 + k, clear_F=hand) for k in range(0, 20)] + secs(sample(21.0, 21.0), 3.0)
    s, why = T.ladder(feed(out))
    assert s == 0.0 and "drive each other" in why


def test_racks_moved_one_after_the_other_score_zero():
    out = [sample(1.0 + k, 1.0) for k in range(0, 20)] + [sample(21.0, 1.0 + k) for k in range(0, 20)]
    out += secs(sample(21.0, 21.0), 3.0)
    assert T.ladder(feed(out))[0] == 0.0


def test_coupling_threshold():
    def run(mm):
        out = [sample(1.0 + k, 1.0 + k) for k in range(0, int(mm) + 1)] + secs(sample(1.0 + mm, 1.0 + mm), 3.0)
        return T.ladder(feed(out))
    assert run(18)[0] == 1.0
    assert run(10)[0] == 0.0


def test_coupling_must_happen_with_both_racks_seated():
    # the near rack was pulled out of its channel during the pull; later put back
    out = [sample(1.0 + k, 1.0 + k, N_z=T.Z0 + 0.01) for k in range(0, 20)] + secs(sample(21.0, 21.0), 3.0)
    assert T.ladder(feed(out))[0] == 0.0


def test_symmetry_rung():
    # offset 3.0 mm: symmetric enough for the open rungs; offset 3.2 mm: not symmetric, no coupling issue
    assert T.ladder(feed_c(secs(sample(26.0, 20.0), 3.0)))[0] > 0.25
    assert T.ladder(feed_c(secs(sample(26.4, 20.0), 3.0)))[0] == 0.25


def test_open_15_each():
    assert T.ladder(feed_c(secs(sample(15.0, 15.0), 3.0)))[0] == 1.0
    s, why = T.ladder(feed_c(secs(sample(15.0, 14.9), 3.0)))
    assert s == 0.5 and "15 mm" in why


def test_closed_jaws_are_half():
    assert T.ladder(feed_c(secs(sample(1.0, 1.0), 3.0)))[0] == 0.5


def test_release_distance_and_time():
    near = sample(clear={"left": 0.029, "right": 0.08})
    s, why = T.ladder(feed_c(secs(near, 3.0)))
    assert s == 0.75 and "30 mm" in why
    s, _ = T.ladder(feed_c(secs(sample(clear={"left": 0.030, "right": 0.08}), 3.0)))
    assert s == 1.0
    s, _ = T.ladder(feed_c(secs(sample(), 0.9)))  # 0.9 s clear is not 1 s
    assert s == 0.75


def test_hand_still_for_release():
    moving = sample(tcp_speed={"left": 0.006, "right": 0.0})
    assert T.ladder(feed_c(secs(moving, 3.0)))[0] == 0.75


def test_racks_must_be_still():
    sl = [sample(20.0 + 0.5 * k, 20.0 + 0.5 * k) for k in range(60)]  # 10 mm/s
    assert T.ladder(feed_c(sl))[0] == 0.75


def test_rack_off_the_bench_at_the_end_is_not_seated():
    rec = feed(demo_trace())
    rec = feed(secs(sample(F=(T.FAR_X, T.YC0 - 0.02, T.TOP - 0.02)), 0.2), rec, t0=20.0)
    assert T.ladder(rec)[0] == 0.0


def test_reset_clears_the_record():
    rec = feed(demo_trace())
    assert rec.coupled > 15.0
    rec.reset()
    assert T.ladder(rec)[0] == 0.0 and rec.coupled == 0.0
    assert T.ladder(feed(demo_trace(), rec))[0] == 1.0


def test_offset_formula():
    assert T.offset_mm({"sN": 10.0, "sF": 4.0}) == 3.0
    assert T.offset_mm({"sN": 4.0, "sF": 10.0}) == -3.0
