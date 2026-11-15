"""Offline checks of task.py (no GPU, no Isaac Sim): every scoring outcome from fabricated joint readings, and the
mechanism's geometry (the sag blocks the bolt, the keeper takes it once lifted, the handle's notch).

    python -m pytest test_scoring.py
"""

from __future__ import annotations

import importlib
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from gatekit import PartState, link_poses  # noqa: E402

T = importlib.import_module("task")

LEVEL = -T.SAG_ANGLE            # the gate lifted so the bolt is level with the keeper
DOWN = -math.pi / 2             # the handle turned down into its notch


def _objects(leaf=0.0, bolt=0.0, handle=0.0):
    p = np.array(T.ROOT)
    return {"gate": PartState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                              low=p, high=p, joints={"leaf": leaf, "bolt": bolt, "handle": handle})}


def _run(frames):
    T._reset_state()
    out = (0.0, False)
    for objs in frames:
        ok = T.success(objs)          # the runtime's order: success() then score() on the same states
        out = (T.score(objs), ok)
    return out


def test_start_scores_zero_and_the_bolt_sits_low():
    r = T.readings(_objects()["gate"])
    assert -0.0070 < r["dz"] < -0.0060 and abs(r["into"] + 0.018) < 1e-4
    assert _run([_objects()] * 30) == (0.0, False)


def test_lifting_to_line_up():
    assert _run([_objects(leaf=LEVEL)])[0] == 0.25
    assert _run([_objects(leaf=LEVEL), _objects()])[0] == 0.25                        # sticky
    low = -T.SAG_ANGLE * (T.SAG - T.LINED_UP - 0.0005) / T.SAG
    assert _run([_objects(leaf=low)])[0] == 0.0                                      # 3.5 mm short
    assert _run([_objects(leaf=-T.LIFT_ANGLE)])[0] == 0.0                            # lifted too high (+8.6 mm)


def test_into_the_keeper():
    assert _run([_objects(leaf=LEVEL, bolt=-0.024)])[0] == 0.5                        # 6 mm in
    assert _run([_objects(leaf=LEVEL, bolt=-0.022)])[0] == 0.25                       # 4 mm in
    assert _run([_objects(leaf=LEVEL, bolt=-0.044)] * 30)[0] == 0.75                  # home, handle up
    assert _run([_objects(leaf=LEVEL, bolt=-0.044), _objects()])[0] == 0.75           # sticky


def test_bolted_succeeds_after_one_second_still():
    done = _objects(leaf=LEVEL, bolt=-T.THROW, handle=DOWN)
    assert _run([done] * 19) == (0.75, False)
    assert _run([done] * 20) == (1.0, True)
    assert _run([_objects(leaf=LEVEL, bolt=-T.THROW)] * 25 + [done] * 19) == (0.75, False)
    assert _run([_objects(leaf=LEVEL, bolt=-T.THROW, handle=math.radians(-65))] * 30)[0] == 0.75
    assert _run([_objects(leaf=LEVEL, bolt=-T.THROW, handle=math.radians(-75))] * 20) == (1.0, True)
    # the gate let down onto the bolt (resting on the keeper's lower strap) still counts
    rest = -T.SAG_ANGLE * (T.SAG - 0.003) / T.SAG
    assert _run([_objects(leaf=rest, bolt=-T.THROW, handle=DOWN)] * 20) == (1.0, True)


def test_moving_bolt_is_not_success():
    frames = [_objects(leaf=LEVEL, bolt=-T.THROW + (0.002 if k % 2 else 0.0), handle=DOWN) for k in range(40)]
    assert _run(frames) == (0.75, False)


def test_score_never_one_before_success():
    done = _objects(leaf=LEVEL, bolt=-T.THROW, handle=DOWN)
    T._reset_state()
    for k in range(25):
        ok = T.success(done)
        s = T.score(done)
        assert (s == 1.0) == ok


# -- geometry ---------------------------------------------------------------------------------------------------------

def _keeper():
    boxes = T._post_boxes()[2:5]
    return [(np.array(c) + np.array([0, 0, T.TOP]), np.array(s)) for c, s, _ in boxes]


def _rod_hits_keeper(leaf, bolt):
    """Points on the bolt's end face and its side, against the keeper's straps (world)."""
    p, r = link_poses(T.GATE, {"leaf": leaf, "bolt": bolt})["bolt"]
    centre = T.ROOT + p
    pts = []
    for y in np.linspace(-T.ROD_LEN / 2, -T.ROD_LEN / 2 + 0.03, 7):
        for a in np.linspace(0, 2 * math.pi, 37):
            for rad in (T.ROD_R, T.ROD_R * 0.5):
                pts.append(centre + r @ np.array([rad * math.cos(a), y, rad * math.sin(a)]))
    for c, s in _keeper():
        lo, hi = c - s / 2, c + s / 2
        if any(((q > lo + 1e-6) & (q < hi - 1e-6)).all() for q in pts):
            return True
    return False


def test_sagged_bolt_butts_the_keeper_and_a_lifted_one_goes_in():
    assert _rod_hits_keeper(0.0, -0.025)                     # sagging: 7 mm into the lower strap's path
    assert not _rod_hits_keeper(LEVEL, -T.THROW)             # level: all the way in
    for mm in (-2.5, 2.5):                                   # within the 3 mm play
        a = -T.SAG_ANGLE * (T.SAG + mm / 1000) / T.SAG
        assert not _rod_hits_keeper(a, -T.THROW)
    for mm in (-3.5, 3.5):
        a = -T.SAG_ANGLE * (T.SAG + mm / 1000) / T.SAG
        assert _rod_hits_keeper(a, -T.THROW)
    assert _rod_hits_keeper(-T.LIFT_ANGLE, -T.THROW)        # lifted to the stop: too high
    assert not _rod_hits_keeper(0.0, 0.0)                    # drawn back it is clear of the keeper


def test_notch_only_at_the_end_of_travel():
    lips = T._gate_boxes()[6:8]
    notch_lo = lips[0][0][1] + lips[0][1][1] / 2
    notch_hi = lips[1][0][1] - lips[1][1][1] / 2
    assert abs((notch_lo + notch_hi) / 2 - T.NOTCH_Y) < 1e-9
    assert abs(notch_hi - notch_lo - 2 * T.STEM_R - 0.002) < 1e-9   # 1 mm play each side of the stem
    # drawn back the stem sits over the lip; thrown it sits in the notch
    rear = lips[1]
    assert rear[0][1] - rear[1][1] / 2 < T.STEM_Y - T.STEM_R and rear[0][1] + rear[1][1] / 2 > T.STEM_Y + T.STEM_R
    # the lip spans the stem's path when turned down (x within 4 mm of the bolt's axis)
    x0, x1 = lips[0][0][0] - lips[0][1][0] / 2, lips[0][0][0] + lips[0][1][0] / 2
    assert x0 < T.AXIS_X - T.STEM_R and x1 > T.AXIS_X + T.STEM_R
    assert lips[0][0][2] + lips[0][1][2] / 2 <= T.BOLT_H - T.ROD_R - 0.0004


def test_the_handle_turned_down_clears_the_plate_and_table():
    knob_back = T.AXIS_X + T.KNOB_R
    assert knob_back < T.GATE_FRONT_X - T.PLATE_T
    assert T.BOLT_H - T.SAG - 0.038 > 0.02


def test_gate_clears_the_post():
    assert abs(T.GATE_EDGE_Y - T.POST[3] - 0.010) < 1e-9
