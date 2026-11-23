# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for cabinet-hinge-adjust, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); most tests feed it
synthetic samples at 20 Hz. The measure() tests build door poses from the hinge's kinematics.
"""

import math
import pathlib
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402
import task as T  # noqa: E402

DT = 0.05


def sample(**kw):
    """The finished state: door closed, reveals 4.45 / 4.55 mm, flush, still, driver in its rack, hands clear."""
    m = {
        "psi": 0.0, "gR": 4.45, "gL": 4.55, "proud": 0.0, "sag": 0.0, "door_speed": 0.0,
        "th": {"hs": -0.9, "hd": 0.0, "hc": 0.0}, "engaged": False,
        "clear": {"left": 0.09, "right": 0.17}, "tcp_speed": {"left": 0.0, "right": 0.0},
    }
    m.update(kw)
    return m


def start(**kw):
    """The start state: door open, side cam 39.3 deg off neutral, reveals 6.4 / 2.6 mm."""
    base = dict(psi=90.0, gR=6.4, gL=2.6, th={"hs": 39.3, "hd": 0.0, "hc": 0.0})
    base.update(kw)
    return sample(**base)


def run(samples, start_t=0.0):
    rec = T.Record()
    rec.reset()
    t = start_t
    for m in samples:
        t += DT
        rec.update(m, t)
    return rec


def score(samples):
    return T.ladder(run(samples))[0]


def hold(m, seconds):
    return [m] * int(round(seconds / DT))


def test_finished_state_scores_one():
    assert score(hold(start(), 1) + hold(sample(), 2.0)) == 1.0


def test_no_steps():
    assert T.ladder(T.Record())[0] == 0.0


def test_start_state_scores_zero():
    assert score(hold(start(), 3.0)) == 0.0


def test_side_cam_ladder_edge():
    # 39.3 - 10 = 29.3 deg: at or below earns 0.25, above does not
    assert score(hold(start(th={"hs": 29.29, "hd": 0.0, "hc": 0.0}), 1)) == 0.25
    assert score(hold(start(th={"hs": 29.31, "hd": 0.0, "hc": 0.0}), 1)) == 0.0
    assert score(hold(start(th={"hs": -29.29, "hd": 0.0, "hc": 0.0}), 1)) == 0.25   # either side of neutral


def test_side_cam_overturned_never_scores_high():
    # an overturned cam closes with uneven reveals: never above 0.25 and never 1.0
    assert score(hold(sample(gR=1.5, gL=7.5, th={"hs": -85.0, "hd": 0.0, "hc": 0.0}), 3)) <= 0.25


def test_even_limit():
    assert T.even(sample(gR=4.0, gL=5.0))          # difference 1.0
    assert not T.even(sample(gR=3.95, gL=5.05))    # difference 1.1


def test_band_limits():
    assert T.even(sample(gR=3.0, gL=3.5))
    assert not T.even(sample(gR=2.9, gL=3.4))
    assert T.even(sample(gR=5.5, gL=6.0))
    assert not T.even(sample(gR=5.6, gL=6.1))


def test_closed_limit():
    assert T.closed(sample(psi=3.0))
    assert not T.closed(sample(psi=3.1))
    assert not T.even(sample(psi=3.1))


def test_flush_limit():
    assert T.flush(sample(proud=0.6))
    assert T.flush(sample(proud=-0.6))
    assert not T.flush(sample(proud=0.7))
    # even reveals but proud by 0.7 mm: closed with even reveals at some time, not flush at the end
    assert score(hold(sample(proud=0.7), 3.0)) == 0.5


def test_sag_gate_any_time():
    run_ = hold(start(), 1) + hold(sample(sag=1.2), 1) + hold(sample(), 3.0)
    assert score(run_) == 0.0
    assert "gate" in T.ladder(run(run_))[1]


def test_sag_just_under_gate():
    assert score(hold(sample(sag=1.1), 1) + hold(sample(), 3.0)) == 1.0


def test_sag_gate_while_door_open():
    # the slip is read from the hinge line, so it counts at any door angle
    assert score(hold(start(sag=1.3), 1) + hold(sample(), 3.0)) == 0.0


def test_release_distance_edge():
    assert score(hold(sample(clear={"left": 0.029, "right": 0.17}), 3.0)) == 0.75
    assert score(hold(sample(clear={"left": 0.030, "right": 0.17}), 3.0)) == 1.0


def test_release_one_pad_step_restarts_timer():
    near = sample(clear={"left": 0.02, "right": 0.17})
    assert score(hold(sample(), 0.9) + [near] + hold(sample(), 0.9)) == 0.75
    assert score(hold(sample(), 0.9) + [near] + hold(sample(), 1.1)) == 1.0


def test_release_hand_speed_edge():
    assert score(hold(sample(tcp_speed={"left": 0.006, "right": 0.0}), 3.0)) == 0.75
    assert score(hold(sample(tcp_speed={"left": 0.004, "right": 0.0}), 3.0)) == 1.0


def test_release_needs_one_second():
    near = sample(clear={"left": 0.02, "right": 0.17})
    assert score(hold(near, 1.5) + hold(sample(), 0.95)) == 0.75
    assert score(hold(near, 1.5) + hold(sample(), 1.1)) == 1.0
    assert score(hold(sample(), 0.95)) == 0.5      # the door itself must also be still for 1 s


def test_driver_left_in_slot_blocks_release():
    assert score(hold(sample(engaged=True), 3.0)) == 0.75


def test_door_must_be_steady():
    # reveals even and flush but the door still moving at 4 mm/s: not an end state yet
    assert score(hold(sample(door_speed=0.004), 3.0)) == 0.5
    assert score(hold(sample(door_speed=0.002), 3.0)) == 1.0


def test_closed_then_reopened_scores_half():
    run_ = hold(start(), 1) + hold(sample(), 1.0) + hold(start(gR=4.45, gL=4.55, th={"hs": -0.9, "hd": 0.0, "hc": 0.0}), 2)
    assert score(run_) == 0.5


def test_uneven_close_scores_by_cam_only():
    uneven = sample(gR=6.4, gL=2.6, th={"hs": 39.3, "hd": 0.0, "hc": 0.0})
    assert score(hold(uneven, 3.0)) == 0.0
    # side cam turned 25 deg toward neutral but the reveals are still uneven: only the cam rung
    mid = sample(gR=5.4, gL=3.6 - 0.8, th={"hs": 25.0, "hd": 0.0, "hc": 0.0})
    assert score(hold(mid, 3.0)) == 0.25


def test_depth_cam_alone_does_not_score():
    m = start(th={"hs": 39.3, "hd": 50.0, "hc": 0.0}, proud=1.5)
    assert score(hold(m, 2.0) + hold(sample(gR=6.4, gL=2.6, proud=1.5, th={"hs": 39.3, "hd": 50.0, "hc": 0.0}), 3.0)) == 0.0


def test_clamp_first_then_correct_is_gated():
    # clamp loosened 60 deg: the arm slips 2.5 mm (the sag latch); later work cannot undo it
    loose = sample(sag=2.5)
    assert score(hold(start(), 1) + hold(loose, 3.0) + hold(sample(sag=2.5), 3.0)) == 0.0


def test_record_cleared_by_reset():
    rec = run(hold(sample(sag=2.0), 2.0))
    assert "slipped" in rec.H.evers
    rec.reset()
    for _ in range(40):
        rec.update(sample(), None)
    assert "slipped" not in rec.H.evers
    assert rec.last is not None


# -- measure() on door poses built from the hinge's kinematics -----------------------------------------------------------


def door_pose(psi_deg, d=0.0, dx=0.0, dz=0.0):
    """World pose of the door link for opening angle psi, lateral offset d, proud dx and sag dz (m)."""
    p = math.radians(psi_deg)
    R = np.array([[math.cos(p), 0, -math.sin(p)], [0, 1, 0], [math.sin(p), 0, math.cos(p)]])
    pivot = np.array([T.F - T.BOARD - dx, d, T.TOP + T.HH - dz])
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = pivot + R @ np.array([T.BOARD / 2, 0.0, T.DH / 2])
    return M


def stub_cab(psi, d, dx=0.0, dz=0.0, th=0.0):
    poses = {"door": door_pose(psi, d, dx, dz)}
    for n in ("hs", "hd", "hc"):
        M = np.eye(4)
        M[:3, :3] = lk.axis_rotation((0, 0, 1), math.radians(th if n == "hs" else 0.0))
        M[:3, 3] = (T.HEAD_X[n], T.HEAD_Y, T.TOP + T.HEAD_Z)
        poses[n] = M
    return SimpleNamespace(poses=poses, lin_vels={"door": np.zeros(3)}, low=np.array([-1.0, -1.0, 0.0]),
                           high=np.array([1.0, 1.0, 2.0]))


def stub_driver():
    M = np.eye(4)
    M[:3, 3] = (T.DRV_X, T.DRV_Y, T.TOP + 0.0125)
    return SimpleNamespace(poses={"drv": M}, low=np.array([T.DRV_X - 0.02, T.DRV_Y - 0.02, T.TOP]),
                           high=np.array([T.DRV_X + 0.02, T.DRV_Y + 0.02, T.TOP + 0.1]))


def far_tools():
    rot = np.eye(3)
    return {s: lk.Tool(s, np.array([0.3, 0.3 if s == "left" else -0.3, T.TOP + 0.3]), rot, 0.094, 0.0)
            for s in ("left", "right")}


@pytest.mark.parametrize("psi", [0.0, 30.0, 90.0])
def test_reveals_are_read_at_any_door_angle(psi):
    m = T.measure(stub_cab(psi, 0.0019), stub_driver(), far_tools())
    assert m["psi"] == pytest.approx(psi, abs=1e-6)
    assert m["gR"] == pytest.approx(6.4, abs=1e-6)     # 4.5 + 1.9
    assert m["gL"] == pytest.approx(2.6, abs=1e-6)     # 4.5 - 1.9
    assert m["proud"] == pytest.approx(0.0, abs=1e-6)
    assert m["sag"] == pytest.approx(0.0, abs=1e-6)


def test_measure_proud_and_sag():
    m = T.measure(stub_cab(0.0, 0.0, dx=0.0007, dz=0.0013), stub_driver(), far_tools())
    assert m["proud"] == pytest.approx(0.7, abs=1e-6)
    assert m["sag"] == pytest.approx(1.3, abs=1e-6)
    assert m["gR"] == pytest.approx(4.5, abs=1e-6)
    assert m["gL"] == pytest.approx(4.5, abs=1e-6)


def test_cam_geometry_zero_is_centred():
    # the side cam is an eccentric: neutral (0 deg) centres the door; 39.3 deg moves it 1.9 mm
    assert T.A_SIDE * math.sin(math.radians(0.0)) == 0.0
    assert 1000 * T.A_SIDE * math.sin(math.radians(T.SIDE0_DEG)) == pytest.approx(1.9, abs=0.01)
    # reveal difference 1.0 mm is reached at +-9.6 deg of cam angle
    assert 2000 * T.A_SIDE * math.sin(math.radians(9.6)) == pytest.approx(1.0, abs=0.01)


def test_measure_engaged_flag():
    cab = stub_cab(90.0, 0.0, th=0.0)
    drv = stub_driver()
    h = cab.poses["hs"][:3, 3]
    drv.poses["drv"][:3, 3] = (h[0], h[1], h[2])            # tip at the head centre: in the slot
    assert T.measure(cab, drv, far_tools())["engaged"]
    drv.poses["drv"][:3, 3] = (h[0], h[1] + 0.01, h[2])     # 10 mm across the slot, clear of the heads
    assert not T.measure(cab, drv, far_tools())["engaged"]
    drv.poses["drv"][:3, 3] = (h[0], h[1], h[2] + 0.004)    # above the head
    assert not T.measure(cab, drv, far_tools())["engaged"]
