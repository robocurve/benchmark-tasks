# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for servo-horn-zero-spline, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run record (a linkkit History fed every control step) plus ladder(); the tests feed it
synthetic samples, and measure() with real 4x4 poses.
"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402
import task as T  # noqa: E402

DT = 0.05


def sample(**kw):
    """The finished state: horn seated at index 0, hands clear and still."""
    m = {"off": 0.2, "hz": 0.1, "tilt": 0.5, "yaw": 0.3, "d": 0.3, "k": 0, "seat": 1.5, "held": False,
         "h_speed": 0.0, "h_spin": 0.0, "clear": {"left": 0.08, "right": 0.06}, "tcp_speed": {"left": 0.0, "right": 0.0}}
    m.update(kw)
    m["d"] = kw.get("d", (m["yaw"] + T.PITCH / 2) % T.PITCH - T.PITCH / 2)
    m["k"] = kw.get("k", int(round(m["yaw"] / T.PITCH)))
    return m


def run(samples):
    H = lk.History()
    for i, m in enumerate(samples):
        H._tick(i * DT)
        T.record(H, m)
    return H


def carried(**kw):
    return sample(seat=0.0, hz=1.5, held=True, **kw)


def demo_trace(n_end=40):
    return [carried(yaw=0.2)] * 10 + [sample(seat=0.7, hz=0.8)] * 3 + [sample()] * n_end


def test_demo_shape_scores_one():
    assert T.ladder(run(demo_trace()))[0] == 1.0


def test_nothing_is_zero():
    H = run([sample(seat=0.0, hz=-40.0, off=150.0, yaw=68.0, k=5)] * 50)
    assert T.ladder(H)[0] == 0.0


def test_wrong_tooth_at_the_end_scores_zero():
    tr = [carried(yaw=14.4)] * 5 + [sample(yaw=14.4)] * 40
    assert T.ladder(run(tr))[0] == 0.0
    assert T.ladder(run(tr))[1]["wrong_index_now"]


def test_wrong_tooth_then_pulled_off_and_refitted_scores_one():
    tr = ([sample(seat=0.0, hz=1.5, held=True, yaw=14.4)] * 3 + [sample(yaw=14.4)] * 5
          + [sample(seat=0.0, hz=30.0, held=True, yaw=14.4)] * 5 + [carried(yaw=0.2)] * 10
          + [sample(seat=0.7, hz=0.8)] * 3 + [sample()] * 40)
    sc, info = T.ladder(run(tr))
    assert sc == 1.0 and not info["wrong_index_now"]


def test_wrong_tooth_then_pulled_off_and_dropped_is_not_one():
    tr = [carried(yaw=14.4)] * 3 + [sample(yaw=14.4)] * 5 + [sample(seat=0.0, hz=-40.0, off=150.0, yaw=30.0, held=False)] * 40
    assert T.ladder(run(tr))[0] == 0.0


def test_refit_leaving_it_on_another_tooth_again_is_zero():
    tr = [carried(yaw=14.4)] * 3 + [sample(yaw=14.4)] * 5 + [sample(seat=0.0, hz=30.0, held=True)] * 5 + [sample(yaw=28.8)] * 40
    assert T.ladder(run(tr))[0] == 0.0


def test_seat_window_edge():
    ok = run([carried(yaw=1.9)] * 3 + [sample(yaw=1.9)] * 40)
    bad = run([carried(yaw=1.9)] * 3 + [sample(yaw=2.1)] * 40)
    assert T.ladder(ok)[0] == 1.0
    assert T.ladder(bad)[0] == 0.0  # seat at 2.1 deg off the mark is not index 0 within the window


def test_aligned_window_is_two_degrees_of_any_index():
    base = dict(off=0.2, tilt=1.0, hz=1.5)
    assert T.aligned(T.measure(*_poses(yaw=14.4 + 1.9, **base)))
    assert not T.aligned(T.measure(*_poses(yaw=14.4 + 2.1, **base)))
    assert T.aligned(T.measure(*_poses(yaw=-1.9, **base)))
    assert not T.aligned(T.measure(*_poses(yaw=68.0, **base)))  # where the horn lies: 4 deg from index 5
    assert T.aligned(T.measure(*_poses(yaw=72.0, **base)))


def test_aligned_needs_bore_on_shaft_and_flat():
    assert T.aligned(T.measure(*_poses(yaw=0.0, off=0.7, tilt=3.0)))
    assert not T.aligned(T.measure(*_poses(yaw=0.0, off=0.9, tilt=0.0)))
    assert not T.aligned(T.measure(*_poses(yaw=0.0, off=0.0, tilt=4.5)))


def test_press_where_it_lands_unseated_scores_zero():
    # horn lying on the crests at 68 deg, never seated
    H = run([carried(yaw=68.0, k=5)] * 60)
    assert T.ladder(H)[0] == 0.0


def test_carried_rung_needs_a_hold():
    dropped = run([sample(seat=0.0, hz=1.5, held=False, yaw=0.5)] * 30)
    assert T.ladder(dropped)[0] == 0.0
    held = run([carried(yaw=0.5)] * 3)
    assert T.ladder(held)[0] == 0.25


def test_carried_rung_needs_the_mark_within_3_deg():
    assert T.ladder(run([carried(yaw=2.9)] * 3))[0] == 0.25
    assert T.ladder(run([carried(yaw=3.1)] * 3))[0] == 0.0


def test_seated_but_still_held_is_half_or_three_quarters():
    tr = [carried(yaw=0.2)] * 5 + [sample(held=True, clear={"left": 0.08, "right": 0.0})] * 40
    assert T.ladder(run(tr))[0] == 0.75
    tr2 = [carried(yaw=0.2)] * 5 + [sample(hz=0.9, tilt=0.5)] * 40  # sitting 0.9 mm proud at the end
    assert T.ladder(run(tr2))[0] == 0.5


def test_end_limits():
    for kw, want in (({"hz": 0.5}, 1.0), ({"hz": 0.6}, 0.5), ({"tilt": 2.0}, 1.0), ({"tilt": 2.1}, 0.5),
                     ({"off": 0.8}, 1.0), ({"off": 0.9}, 0.25), ({"seat": 1.3}, 1.0), ({"seat": 1.2}, 0.25)):
        tr = [carried(yaw=0.2)] * 5 + [sample(**kw)] * 40
        assert T.ladder(run(tr))[0] == want, kw


def test_release_gate_30mm_and_5mms():
    def end(**kw):
        return [carried(yaw=0.2)] * 5 + [sample(**kw)] * 40

    assert T.ladder(run(end(clear={"left": 0.08, "right": 0.030})))[0] == 1.0
    assert T.ladder(run(end(clear={"left": 0.08, "right": 0.029})))[0] == 0.75
    assert T.ladder(run(end(tcp_speed={"left": 0.0, "right": 0.0049})))[0] == 1.0
    assert T.ladder(run(end(tcp_speed={"left": 0.0, "right": 0.0051})))[0] == 0.75


def test_one_second_of_release_is_not_enough():
    tr = [carried(yaw=0.2)] * 5 + [sample(clear={"left": 0.08, "right": 0.0})] * 40 + [sample()] * 19
    assert T.ladder(run(tr))[0] == 0.75  # 0.95 s clear
    assert T.ladder(run(tr + [sample()] * 2))[0] == 1.0


def test_horn_must_be_still_for_the_last_second():
    tr = [carried(yaw=0.2)] * 5 + [sample()] * 40 + [sample(h_speed=0.003)]
    assert T.ladder(run(tr))[0] == 0.75
    tr = [carried(yaw=0.2)] * 5 + [sample()] * 40 + [sample(h_spin=2.5)]
    assert T.ladder(run(tr))[0] == 0.75


def test_unseating_after_the_seat_loses_the_end_rung():
    tr = [carried(yaw=0.2)] * 5 + [sample()] * 5 + [sample(seat=0.0, hz=3.0, yaw=40.0, k=3)] * 40
    assert T.ladder(run(tr))[0] == 0.25  # seated state is gone; only the carried rung remains


def test_record_is_clean_after_reset():
    H = run([carried(yaw=14.4)] * 3 + [sample(yaw=14.4)] * 3)
    assert T.ladder(H)[0] == 0.0
    H.reset()
    assert "last" not in H.values and not H.evers
    assert T.ladder(H)[0] == 0.0


def _poses(yaw, off=0.0, tilt=0.0, hz=0.0, q=0.0):
    sp = np.eye(4)
    sp[:3, 3] = (0.25, -0.05, 0.7475)
    a, b = math.radians(yaw), math.radians(tilt)
    rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    rx = np.array([[1, 0, 0], [0, math.cos(b), -math.sin(b)], [0, math.sin(b), math.cos(b)]])
    hp = np.eye(4)
    hp[:3, :3] = rz @ rx
    hp[:3, 3] = sp[:3, 3] + (off / 1000, 0.0, T.Z_BOSS + hz / 1000)
    return hp, sp, q


def test_measure_from_poses():
    m = T.measure(*_poses(yaw=-7.0, off=0.5, tilt=1.0, hz=1.5, q=0.0013))
    assert m["off"] == pytest.approx(0.5, abs=1e-6)
    assert m["hz"] == pytest.approx(1.5, abs=1e-6)
    assert m["tilt"] == pytest.approx(1.0, abs=1e-6)
    assert m["yaw"] == pytest.approx(-7.0, abs=0.05)
    assert m["k"] == 0 and m["d"] == pytest.approx(-7.0, abs=0.05)
    assert m["seat"] == pytest.approx(1.3, abs=1e-6)
    assert T.measure(*_poses(yaw=7.3))["k"] == 1
    assert T.measure(*_poses(yaw=144.0))["k"] == 10


def test_spline_geometry():
    assert T.PITCH == pytest.approx(14.4)
    # the horn's initial azimuth sits off every index by more than the window
    d = (T.HORN_YAW0 + T.PITCH / 2) % T.PITCH - T.PITCH / 2
    assert abs(d) > T.WINDOW_DEG
    assert T.SEAT_SPRING * T.SEAT_TRAVEL == pytest.approx(3.6)
