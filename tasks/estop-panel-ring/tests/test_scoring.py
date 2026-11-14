# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Scorer edge cases for estop-panel-ring, pure Python (no simulator). Run from the task folder: pytest -q tests

The scorer is a whole-run Record (a linkkit History fed every control step) plus ladder(); the tests feed it synthetic
samples shaped like measure() output, and check the panel hole and ring geometry arithmetic.
"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import linkkit as lk  # noqa: E402,F401
import task as T  # noqa: E402

DT = 0.05  # 20 Hz control


def sample(**kw):
    """A finished state: button seated with the tab in the notch, ring snug, grippers clear and still."""
    m = {
        "y": 0.0024, "dx": 0.0, "dz": -0.0004, "z_abs": 0.867, "tab_err": 0.6, "tilt": 4.0, "spin": 172.0,
        "slide": 0.0057, "btn_speed": 0.0016, "ring_speed": 0.0034,
        "clear": {"left": 0.047, "right": 0.085}, "tcp_speed": {"left": 0.0, "right": 0.0},
    }
    m.update(kw)
    return m


def run(samples, t0=0.0):
    rec = T.Record()
    for i, m in enumerate(samples):
        rec.update(m, t0 + i * DT)
    return rec


def steps(m, seconds):
    return [m] * int(round(seconds / DT) + 1)


def held(**kw):
    """Mid-task: the left hand still holds the button."""
    return sample(clear={"left": 0.0, "right": 0.085}, **kw)


def test_demo_shaped_run_scores_one():
    rec = run(steps(held(), 1.0) + steps(sample(), 1.5))
    assert T.ladder(rec)[0] == 1.0


def test_nothing_scores_zero():
    start = sample(y=0.135, dx=0.02, dz=-0.1, tab_err=100.0, tilt=0.0, spin=0.0, slide=0.0)
    assert T.ladder(run(steps(start, 2.0)))[0] == 0.0


def test_no_steps():
    assert T.ladder(T.Record())[0] == 0.0


def test_insert_rungs():
    ins = held(y=0.006, spin=0.0, slide=0.0)
    assert T.ladder(run(steps(ins, 0.5)))[0] == 0.25
    # one step further out is not 8 mm in
    assert T.ladder(run(steps(held(y=0.0061, spin=0.0, slide=0.0), 0.5)))[0] == 0.0
    # tab 12 deg off is still in the notch, 12.1 is not
    assert T.ladder(run(steps(held(y=0.004, tab_err=12.0, spin=0.0, slide=0.0), 0.5)))[0] == 0.25
    assert T.ladder(run(steps(held(y=0.004, tab_err=12.1, spin=0.0, slide=0.0), 0.5)))[0] == 0.0
    # lateral 3 mm ok, 3.1 not
    assert T.ladder(run(steps(held(y=0.004, dx=0.003, spin=0.0, slide=0.0), 0.5)))[0] == 0.25
    assert T.ladder(run(steps(held(y=0.004, dx=0.0031, spin=0.0, slide=0.0), 0.5)))[0] == 0.0
    # tilt 8 deg ok for the insert rung, 8.1 not
    assert T.ladder(run(steps(held(y=0.004, tilt=8.0, spin=0.0, slide=0.0), 0.5)))[0] == 0.25
    assert T.ladder(run(steps(held(y=0.004, tilt=8.1, spin=0.0, slide=0.0), 0.5)))[0] == 0.0


def test_seated_rung_and_edges():
    assert T.ladder(run(steps(held(y=0.0030, spin=0.0, slide=0.0), 0.5)))[0] == 0.5
    assert T.ladder(run(steps(held(y=0.0031, spin=0.0, slide=0.0), 0.5)))[0] == 0.25
    assert T.ladder(run(steps(held(tilt=6.0, spin=0.0, slide=0.0), 0.5)))[0] == 0.5
    assert T.ladder(run(steps(held(tilt=6.1, spin=0.0, slide=0.0), 0.5)))[0] == 0.25


def test_ring_rungs():
    seat = held(spin=0.0, slide=0.0)
    on = held(spin=90.0, slide=0.003)
    rec = run(steps(seat, 0.5) + steps(on, 0.5))
    assert T.ladder(rec)[0] == 0.75  # turned 90 deg, still in progress at the end
    under = held(spin=89.9, slide=0.003)
    assert T.ladder(run(steps(seat, 0.5) + steps(under, 0.5)))[0] == 0.5
    # turning with the button not seated never counts as ring_on (the thread would not engage)
    loose = held(y=0.006, spin=120.0, slide=0.004)
    assert T.ladder(run(steps(loose, 0.5)))[0] == 0.25


def test_snug_limits():
    base = steps(held(), 0.5)
    assert T.ladder(run(base + steps(sample(spin=165.0), 1.5)))[0] == 1.0
    assert T.ladder(run(base + steps(sample(spin=164.9), 1.5)))[0] == 0.75
    assert T.ladder(run(base + steps(sample(spin=185.0), 1.5)))[0] == 1.0
    assert T.ladder(run(base + steps(sample(spin=185.1), 1.5)))[0] == 0.75
    # ring face 0.5 mm from the panel passes, 0.6 mm does not
    assert T.ladder(run(base + steps(sample(slide=0.0055), 1.5)))[0] == 1.0
    assert T.ladder(run(base + steps(sample(slide=0.0054), 1.5)))[0] == 0.75


def test_release_and_still_timers():
    base = steps(held(), 0.5)
    # 0.95 s of release is not enough, 1.0 s is
    assert T.ladder(run(base + [sample()] * 19))[0] == 0.75
    assert T.ladder(run(base + [sample()] * 20))[0] == 1.0
    # one pad 29 mm away restarts the timer; 30 mm does not
    near = sample(clear={"left": 0.029, "right": 0.085})
    ok = sample(clear={"left": 0.030, "right": 0.085})
    assert T.ladder(run(base + [sample()] * 10 + [near] + [sample()] * 19))[0] == 0.75
    assert T.ladder(run(base + steps(ok, 1.5)))[0] == 1.0
    # a moving hand (tcp 10 mm/s) is not still
    assert T.ladder(run(base + steps(sample(tcp_speed={"left": 0.0, "right": 0.01}), 1.5)))[0] == 0.75
    # a button at 3 mm/s is not still, 2.9 mm/s is
    assert T.ladder(run(base + steps(sample(btn_speed=0.003), 1.5)))[0] == 0.75
    assert T.ladder(run(base + steps(sample(btn_speed=0.0029), 1.5)))[0] == 1.0
    assert T.ladder(run(base + steps(sample(ring_speed=0.005), 1.5)))[0] == 0.75


def test_dropped_button_gates_to_zero():
    fall = sample(z_abs=T.DROP_Z - 0.001)
    rec = run(steps(held(), 0.5) + [fall] + steps(sample(), 2.0))
    assert T.ladder(rec)[0] == 0.0
    assert "fell off the table" in T.ladder(rec)[1]


def test_record_is_clean_after_reset():
    rec = run(steps(held(), 0.5) + [sample(z_abs=0.0)])
    assert T.ladder(rec)[0] == 0.0
    rec.reset()
    assert T.ladder(rec)[0] == 0.0 and rec.H.evers == {}
    rec.update(sample(), 0.0)
    assert "dropped" not in rec.H.evers


def test_engagement_rule():
    assert T.engaged(sample(y=0.0035, tilt=8.0))
    assert not T.engaged(sample(y=0.0036))
    assert not T.engaged(sample(tilt=8.1))
    assert not T.engaged(sample(tab_err=12.1))
    assert not T.engaged(sample(dz=0.0031))


def test_tab_angle_from_rotation():
    assert T.tab_angle(np.eye(3)) == pytest.approx(T.TAB0_DEG)
    # rolling the button -TAB0 about +y puts the tab straight up
    c, s = math.cos(math.radians(-T.TAB0_DEG)), math.sin(math.radians(-T.TAB0_DEG))
    Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    assert T.tab_angle(Ry) == pytest.approx(0.0, abs=1e-9)
    assert T.wrap_deg(190.0) == pytest.approx(-170.0)
    assert T.wrap_deg(-190.0) == pytest.approx(170.0)


def _in_boxes(x, z, boxes):
    return any(abs(x - b.pos[0]) <= b.dims[0] / 2 + 1e-9 and abs(z - b.pos[2]) <= b.dims[2] / 2 + 1e-9 for b in boxes)


def test_panel_hole_is_round_and_clear():
    boxes = [b for b in T.plate_boxes() if abs(b.dims[1] - T.PT) < 1e-9]
    r_body = T.BARREL_R
    for deg in range(0, 360, 5):
        x, z = r_body * math.cos(math.radians(deg)), r_body * math.sin(math.radians(deg))
        assert not _in_boxes(x, z, boxes), f"barrel at {deg} deg is inside the plate"
    # the plate is solid 0.8 mm outside the 22.5 mm hole (the stepped hole is a little larger than a circle), away from the notch
    for deg in range(0, 360, 5):
        if abs(deg - 90) < 14:
            continue
        x, z = 0.01205 * math.cos(math.radians(deg)), 0.01205 * math.sin(math.radians(deg))
        assert _in_boxes(x, z, boxes), f"gap in the plate at {deg} deg"


def test_tab_fits_notch_with_margin():
    boxes = [b for b in T.plate_boxes() if abs(b.dims[1] - T.PT) < 1e-9]
    # tab up: its edges at x = +-1.4 mm run free through the notch (+-2.5 mm) from r 11.25 to 12.9 mm
    for x in (-0.0014, 0.0, 0.0014):
        for z in (0.0113, 0.0121, 0.0129):
            assert not _in_boxes(x, z, boxes)
    # a tab 90 deg round (pointing +x) meets the plate
    assert _in_boxes(0.0123, 0.0, boxes)
    # the notch closes above 13.5 mm
    assert _in_boxes(0.0, 0.0140, boxes)
    # tolerated roll: the tab edge reaches the notch wall at about asin(1.1 mm / 12.9 mm) = 4.9 deg of margin
    assert math.degrees(math.asin((T.NOTCH_W / 2 - T.TAB_W / 2) / 0.0129)) == pytest.approx(4.9, abs=0.1)


def test_thread_kinematics():
    lead_per_deg = T.LEAD / 360.0
    assert 180.0 * lead_per_deg == pytest.approx(T.TRAVEL)  # a half turn takes the ring from 6 mm out to the panel
    assert T.SPIN_MAX_DEG > 180.0 > T.RING_SNUG_DEG
    # the ring never reaches the panel before the snug angle band: 165 deg leaves 0.5 mm
    assert (180.0 - T.RING_SNUG_DEG) * lead_per_deg == pytest.approx(0.0005)


def test_ring_is_a_hex_with_clear_bore():
    boxes = T.ring_boxes()
    assert len(boxes) == 6
    flats = sorted({round(abs(b.pos[0]), 6) for b in boxes if abs(b.pos[2]) < 1e-9})
    assert flats == [pytest.approx(0.0131, abs=1e-6)]
    inner = 0.0131 - 0.0038 / 2
    assert inner - T.BARREL_R >= 0.0002  # 0.2 mm clearance round the barrel
    outer = 0.0131 + 0.0038 / 2
    assert outer == pytest.approx(0.015)  # 30 mm across flats


def test_demo_variants_listed():
    assert set(T.VARIANTS) >= {"", "nothing", "no_roll", "right_only", "drop_in_front"}
