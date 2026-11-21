"""Offline tests of the scorer and the hook-and-loop rules on hand-built states (no simulator).

Run: python -m pytest test_shoescore.py   (or python test_shoescore.py)
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import shoescore as ss  # noqa: E402
from shoekit import FastenRule, engage_ok, pair_load  # noqa: E402

L = 0.0056
S0 = 25.5 * L  # arclength at the loop at the start


def frame(t, s=S0 - 0.022, gap=0.008, ov=0.050, leg=0.0, robot=0.2):
    return ss.Frame(t=t, s_ring=(s,), gap=(gap,), overlap=(ov,), leg_off=leg, robot_dist=robot)


def run(frames, pull=True):
    sc = ss.Scorer(n_straps=1)
    sc.update(frame(0.0, s=S0, gap=0.040, ov=0.0, robot=0.01))
    for f in frames:
        sc.update(f)
        if sc.want_pull:
            sc.pull_result([pull])
            sc.update(f)
    return sc


def hold(**kw):
    return [frame(0.05 * k + 1.0, **kw) for k in range(30)]


def near(sc, v):
    return abs(sc.score - v) < 1e-9


def test_full():
    assert near(run(hold()), 1.0)


def test_not_held_one_second():
    assert run([frame(1.0 + 0.05 * k) for k in range(10)]).score == 0.0


def test_robot_touching_at_end():
    assert run(hold() + [frame(3.0, robot=0.01)]).score == 0.0


def test_drawn_only():
    assert near(run(hold(gap=0.020, ov=0.0)), 0.25)


def test_pressed_but_not_snug():
    # hook patch pressed down over 50 mm, but the strap not drawn far enough to close the upper
    assert near(run(hold(gap=0.020, ov=0.050)), 0.25)


def test_snug_not_pressed():
    assert near(run(hold(ov=0.0)), 0.5)


def test_short_overlap():
    assert near(run(hold(ov=0.0336)), 0.5)


def test_pull_fails():
    assert near(run(hold(), pull=False), 0.75)


def test_not_drawn_but_closed():
    # gap closed by squeezing the upper while the strap was not drawn: nothing
    assert run(hold(s=S0 - 0.004, ov=0.0)).score == 0.0


def test_leg_off_mark_held():
    sc = run([frame(0.5, leg=0.06)] + hold())
    assert sc.score == 0.0 and "leg" in sc.zero


def test_leg_within_mark():
    assert near(run([frame(0.5, leg=0.045)] + hold()), 1.0)


def test_strap_out_of_loop_held():
    sc = run([frame(0.5 + 0.05 * k, s=None) for k in range(6)] + hold())
    assert sc.score == 0.0 and "loop" in sc.zero


def test_one_missed_reading_is_not_out_of_loop():
    assert near(run([frame(0.5, s=None)] + hold()), 1.0)


def test_pull_redone_after_touch():
    sc = run(hold())
    sc.update(frame(3.0, robot=0.01))
    assert sc.pull is None


def test_ring_crossing():
    loc = np.array([[0, 0.02, 0], [0, 0.01, 0], [0, -0.0028, 0.001], [0, -0.01, 0.008]])
    s = ss.ring_crossing(loc, L, 0.016, 0.006)
    assert s is not None and abs(s - (1.5 + 0.01 / 0.0128) * L) < 1e-9
    loc_out = loc.copy()
    loc_out[:, 0] = 0.03  # beside the loop, not through it
    assert ss.ring_crossing(loc_out, L, 0.016, 0.006) is None


def test_overlap_is_fastened_span():
    assert abs(ss.overlap_m(((30, 12), (30, 13), (31, 12)), L) - 2 * L) < 1e-12
    assert abs(ss.overlap_m(((26, 16), (28, 14), (33, 10)), L) - 8 * L) < 1e-12
    assert ss.overlap_m((), L) == 0.0


def test_engage_rules():
    r = FastenRule()
    up, down = np.array((0, 0, 1.0)), np.array((0, 0, -1.0))
    pl, ph = np.zeros(3), np.array((0, 0, 0.0018))
    assert engage_ok(r, 2.5, down, up, ph, pl)  # pressed, faces opposed, hook on the loop's face
    assert not engage_ok(r, 1.5, down, up, ph, pl)  # laid on, not pressed
    assert not engage_ok(r, 5.0, up, up, ph, pl)  # back of the strap on the loop
    assert not engage_ok(r, 5.0, down, up, -ph, pl)  # hook under the loop plate
    tilt = np.array((math.sin(math.radians(60)), 0, -math.cos(math.radians(60))))
    assert not engage_ok(r, 5.0, tilt, up, ph, pl)  # edge-on


def test_pair_load():
    R = np.eye(3)
    peel, shear = pair_load(3000.0, R, np.zeros(3), np.array((0.0, 0.0, 0.0018 + 0.001)), np.array((0, 0, 0.0018)))
    assert abs(peel - 3.0) < 1e-9 and shear < 1e-9
    peel, shear = pair_load(3000.0, R, np.zeros(3), np.array((0.004, 0.0, 0.0018)), np.array((0, 0, 0.0018)))
    assert abs(shear - 12.0) < 1e-9 and abs(peel) < 1e-9


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            try:
                fn()
                print("pass", name)
            except AssertionError:
                fails += 1
                print("FAIL", name)
    sys.exit(1 if fails else 0)
