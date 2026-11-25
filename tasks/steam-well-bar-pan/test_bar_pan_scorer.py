"""Teleport unit tests for w50_bar_pan_scorer.BarPanTracker (numpy only; run: python test_bar_pan_scorer.py)."""
from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import w50_bar_pan_scorer as SC  # noqa: E402

WC, RIM, ST = (0.48, 0.0), 0.8261, 0.5975
I3 = np.eye(3)
BAR_START = (np.array([0.2130, 0.0, ST + 0.0005]), I3)
BAR_SEAT = (np.array([0.48051, -0.00066, RIM]), I3)
PAN_START = (np.array([0.2130, 0.4675, ST + 0.0005]), I3)
# native readback of the seated pan (Brev/5090 trace, far channel): leans 2.97 deg, inner flange on the bar
PAN_SEAT = (np.array([0.5634456, 0.1323579, 0.7688715]),
            np.array([[0.99865975, -7.5e-07, 0.05175624], [-4.4e-08, 1.0, 1.53e-05], [-0.05175624, -1.53e-05, 0.99865975]]))


def _Ry(d):
    a = math.radians(d)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


PAN_FALLEN = (np.array([0.50, 0.13, RIM - 0.15875 + 0.004]), _Ry(14.0))
OPEN = {"L": [-46.9, -46.9], "R": [-46.9, -46.9]}
LOADED = {"L": [-5.2, -5.0], "R": [-5.1, -5.8]}
FIX = {"stand": (np.zeros(3), I3), "warmer": (np.array([0.0, 0.0, ST]), I3)}


def pad(p):
    c = np.asarray(p, float)
    return c + 0.004 * np.array([[0, 0, 0], [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], float)


PARK = (pad((0.22, +0.32, 0.92)), pad((0.22, -0.32, 0.92)))


def bar_grip_pads(bp):
    x, y, z = bp
    return pad((x + 0.0127 + 0.005, y + 0.20, z + 0.004)), pad((x + 0.0127 + 0.005, y - 0.20, z + 0.004))


def pan_grip_pads(pp):
    x, y, z = pp
    return pad((x, y + 0.1318 - 0.0085, z + 0.0625 + 0.0045)), pad((x, y - 0.1318 + 0.0085, z + 0.0625 + 0.0045))


def run(frames, tr=None):
    tr = tr or SC.BarPanTracker(wc_xy=WC, rim_z=RIM, stand_top=ST)
    rec = None
    for bar, pan, pads, q in frames:
        rec = tr.record(bar, pan, pads[0], pads[1], q78_mm=q, fixtures=FIX)
    return tr, rec


def rep(n, *f):
    return [tuple(f)] * n


def lerp_pose(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, b[1] if t > 0.5 else a[1])


def bar_phase():
    fr = rep(5, BAR_START, PAN_START, PARK, OPEN)
    for i in range(15):  # lift with both hands, 30 mm up
        bp = BAR_START[0] + np.array([0, 0, 0.002 * (i + 1)])
        fr.append(((bp, I3), PAN_START, bar_grip_pads(bp), LOADED))
    top = RIM + 0.060
    for i in range(10):  # raise above the rim (held)
        bp = np.array([0.2130, 0.0, ST + 0.030 + (top - ST - 0.030) * (i + 1) / 10.0])
        fr.append(((bp, I3), PAN_START, bar_grip_pads(bp), LOADED))
    for i in range(10):  # carry over the well (held)
        bp = np.array([0.2130 + (0.48051 - 0.2130) * (i + 1) / 10.0, -0.00066 * (i + 1) / 10.0, top])
        fr.append(((bp, I3), PAN_START, bar_grip_pads(bp), LOADED))
    for i in range(10):  # lower onto the rims (held)
        bp = np.array([0.48051, -0.00066, top + (RIM + 0.001 - top) * (i + 1) / 10.0])
        fr.append(((bp, I3), PAN_START, bar_grip_pads(bp), LOADED))
    fr.append((BAR_SEAT, PAN_START, bar_grip_pads(BAR_SEAT[0]), LOADED))
    fr += rep(30, BAR_SEAT, PAN_START, PARK, OPEN)
    return fr


def pan_carry(bar=BAR_SEAT, end=PAN_SEAT):
    fr = []
    start = PAN_START[0] + np.array([0, -0.4675, 0])
    for i in range(20):
        t = (i + 1) / 20.0
        pp = start + (end[0] + np.array([0, 0, 0.004]) - start) * t + np.array([0, 0, 0.30 * math.sin(math.pi * t)])
        fr.append((bar, (pp, end[1]), pan_grip_pads(pp), LOADED))
    fr.append((bar, end, pan_grip_pads(end[0]), LOADED))
    return fr


def goal_frames():
    return bar_phase() + pan_carry() + rep(30, BAR_SEAT, PAN_SEAT, PARK, OPEN)


TESTS = {}


def test(fn):
    TESTS[fn.__name__] = fn
    return fn


@test
def start_is_zero():
    _, r = run(rep(30, BAR_START, PAN_START, PARK, OPEN))
    assert r["score"] == 0.0, r


@test
def goal_is_one():
    _, r = run(goal_frames())
    assert r["score"] == 1.0 and r["latched"] is None, r


@test
def bar_only_is_half():
    _, r = run(bar_phase())
    assert r["score"] == 0.5, r


@test
def pan_dropped_in():
    fr = bar_phase() + pan_carry(end=(PAN_SEAT[0] + np.array([0, 0, 0.030]), PAN_SEAT[1]))[:-1]
    high = (PAN_SEAT[0] + np.array([0, 0, 0.030]), PAN_SEAT[1])
    fr += rep(2, BAR_SEAT, high, PARK, OPEN)  # released 30 mm above the seat
    fr += rep(30, BAR_SEAT, PAN_SEAT, PARK, OPEN)  # even if it then lands seated
    _, r = run(fr)
    assert r["score"] == 0.0 and r["latched"] == "pan_dropped_from_height", r


@test
def pan_fell_into_well():
    fr = bar_phase() + pan_carry() + rep(30, BAR_SEAT, PAN_FALLEN, PARK, OPEN)
    _, r = run(fr)
    assert r["score"] == 0.0 and r["latched"] == "pan_fell_in", r


@test
def missing_bar_control_pan_falls():
    fr = rep(60, None, PAN_START, PARK, OPEN) + pan_carry(bar=None)
    fr += rep(3, None, PAN_SEAT, PARK, OPEN)  # released at the seat pose: no bar under the inner flange
    fr += rep(30, None, PAN_FALLEN, PARK, OPEN)  # ...so it falls in
    _, r = run(fr)
    assert r["score"] == 0.0 and r["latched"] == "pan_fell_in", r


@test
def missing_bar_pan_at_seat_pose_scores_zero():
    _, r = run(rep(60, None, PAN_START, PARK, OPEN) + pan_carry(bar=None) + rep(30, None, PAN_SEAT, PARK, OPEN))
    assert r["score"] == 0.0 and not r["pan_seated"], r


@test
def bar_knocked_off_after_pan_placed():
    knocked = (np.array([0.47, -0.02, RIM - 0.06]), _Ry(25.0) @ np.eye(3))
    fr = goal_frames() + rep(5, knocked, PAN_FALLEN, PARK, OPEN) + rep(30, BAR_SEAT, PAN_SEAT, PARK, OPEN)
    tr, r = run(fr)
    assert r["score"] == 0.0 and r["latched"] in ("loaded_bar_unseated", "full_credit_lost", "bar_fell", "pan_fell_in"), r


@test
def bar_shifted_off_center_after_pan_placed():
    shifted = (BAR_SEAT[0] + np.array([0.030, 0.0, 0.0]), I3)
    fr = bar_phase() + pan_carry() + rep(3, BAR_SEAT, PAN_SEAT, PARK, OPEN) + rep(30, shifted, PAN_SEAT, PARK, OPEN)
    _, r = run(fr)
    assert r["score"] == 0.0 and r["latched"] == "loaded_bar_unseated", r


@test
def hands_still_near():
    near = (pad((0.60, 0.13, 0.90)), PARK[1])  # left pads 45-60 mm above the pan
    fr = bar_phase() + pan_carry() + rep(30, BAR_SEAT, PAN_SEAT, near, OPEN)
    _, r = run(fr)
    assert r["score"] == 0.0 and r["reason"] == "hands_within_80mm", r


@test
def full_credit_then_lost():
    near = (pad((0.60, 0.13, 0.90)), PARK[1])
    fr = goal_frames() + rep(3, BAR_SEAT, PAN_SEAT, near, OPEN) + rep(30, BAR_SEAT, PAN_SEAT, PARK, OPEN)
    _, r = run(fr)
    assert r["score"] == 0.0 and r["latched"] == "full_credit_lost", r


@test
def pan_still_moving():
    fr = bar_phase() + pan_carry()
    for i in range(25):
        fr.append((BAR_SEAT, (PAN_SEAT[0] + np.array([0.0, 0.0002 * i, 0.0]), PAN_SEAT[1]), PARK, OPEN))
    _, r = run(fr)
    assert r["score"] < 1.0, r


@test
def pan_in_near_channel_not_full():
    near_seat = (np.array([2 * 0.48 - PAN_SEAT[0][0], PAN_SEAT[0][1], PAN_SEAT[0][2]]), PAN_SEAT[1].T)
    _, r = run(bar_phase() + pan_carry(end=near_seat) + rep(30, BAR_SEAT, near_seat, PARK, OPEN))
    assert r["score"] < 1.0, r


@test
def bar_pushed_not_lifted():
    fr = rep(5, BAR_START, PAN_START, PARK, OPEN)
    for i in range(30):
        t = (i + 1) / 30.0
        bp = np.array([0.2130 + (0.48051 - 0.2130) * t, -0.00066 * t, ST + 0.0005 + (RIM - ST - 0.0005) * t])
        fr.append(((bp, I3), PAN_START, PARK, OPEN))
    fr += pan_carry() + rep(30, BAR_SEAT, PAN_SEAT, PARK, OPEN)
    _, r = run(fr)
    assert r["score"] == 0.0, r


@test
def stand_moved():
    tr = SC.BarPanTracker(wc_xy=WC, rim_z=RIM, stand_top=ST)
    fr = goal_frames()
    for bar, pan, pads, q in fr:
        tr.record(bar, pan, pads[0], pads[1], q78_mm=q, fixtures=FIX)
    moved = {"stand": (np.array([0.003, 0, 0]), I3), "warmer": FIX["warmer"]}
    r = tr.record(BAR_SEAT, PAN_SEAT, PARK[0], PARK[1], q78_mm=OPEN, fixtures=moved)
    assert r["score"] == 0.0 and "fixture" in str(r["latched"]), r


# centred seat (the submitted plan, SEAT_Y 0): native Brev readback pan (563.45, 0.02, 768.87) mm, tilt 2.967 deg
PAN_SEAT_C = (np.array([0.56345, 0.00002, 0.76887]), PAN_SEAT[1])


@test
def goal_centred_seat_is_one():
    fr = bar_phase() + pan_carry(end=PAN_SEAT_C) + rep(30, BAR_SEAT, PAN_SEAT_C, PARK, OPEN)
    _, r = run(fr)
    assert r["score"] == 1.0 and r["latched"] is None, r


@test
def missing_bar_centred_seat_scores_zero():
    _, r = run(rep(60, None, PAN_START, PARK, OPEN) + pan_carry(bar=None, end=PAN_SEAT_C) + rep(30, None, PAN_SEAT_C, PARK, OPEN))
    assert r["score"] == 0.0, r



@test
def spilled_pan_tilted_20_deg_in_carry():
    tilt_deg = 20.0
    fr = bar_phase() + pan_carry(end=PAN_SEAT_C)
    k = len(bar_phase()) + 8
    fr[k] = (fr[k][0], (fr[k][1][0], _Ry(tilt_deg)), fr[k][2], fr[k][3])
    fr += rep(30, BAR_SEAT, PAN_SEAT_C, PARK, OPEN)
    _, r = run(fr)
    assert r["score"] == 0.0 and r["latched"] == "exceeded_carrying_tilt", r


@test
def carry_tilt_12_deg_is_ok():
    tilt_deg = 12.0
    fr = bar_phase() + pan_carry(end=PAN_SEAT_C)
    k = len(bar_phase()) + 8
    fr[k] = (fr[k][0], (fr[k][1][0], _Ry(tilt_deg)), fr[k][2], fr[k][3])
    fr += rep(30, BAR_SEAT, PAN_SEAT_C, PARK, OPEN)
    _, r = run(fr)
    assert r["score"] == 1.0, r



# ---- review 1333 fixes ----
RZ180 = np.diag([-1.0, -1.0, 1.0])


@test
def seated_pan_turned_end_for_end_is_one():
    seat180 = (PAN_SEAT_C[0], PAN_SEAT_C[1] @ RZ180)
    fr = bar_phase() + pan_carry(end=seat180) + rep(30, BAR_SEAT, seat180, PARK, OPEN)
    _, r = run(fr)
    assert r["score"] == 1.0 and r["latched"] is None, r


@test
def pan_on_floor_is_zero():
    floor = (np.array([0.05, 0.40, -0.0125]), I3)
    fr = bar_phase() + rep(2, BAR_SEAT, (np.array([0.10, 0.45, 0.40]), I3), PARK, OPEN) + rep(30, BAR_SEAT, floor, PARK, OPEN)
    _, r = run(fr)
    assert r["score"] == 0.0 and r["latched"] == "pan_off_stand", r


@test
def pan_tipped_on_stand_is_zero():
    tipped = (np.array([0.2130, 0.4675, ST + 0.03]), _Ry(80.0))
    fr = bar_phase() + rep(30, BAR_SEAT, tipped, PARK, OPEN)
    _, r = run(fr)
    assert r["score"] == 0.0, r


@test
def pan_upright_on_stand_bar_seated_is_half():
    fr = bar_phase() + rep(30, BAR_SEAT, PAN_START, PARK, OPEN)
    _, r = run(fr)
    assert r["score"] == 0.5, r


@test
def pan_slid_off_stand_edge_is_zero():
    off = (np.array([0.2130, 0.70, ST + 0.0005]), I3)  # beyond the stand's +y edge (0.6095), not supported
    fr = bar_phase() + rep(30, BAR_SEAT, off, PARK, OPEN)
    _, r = run(fr)
    assert r["score"] == 0.0 and r["latched"] == "pan_off_stand", r


@test
def seated_pan_rotating_in_place_is_not_still():
    fr = bar_phase() + pan_carry(end=PAN_SEAT_C) + rep(20, BAR_SEAT, PAN_SEAT_C, PARK, OPEN)
    a = math.radians(2.0)
    Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    fr += rep(1, BAR_SEAT, (PAN_SEAT_C[0], Rz @ PAN_SEAT_C[1]), PARK, OPEN)  # 2 deg yaw = 4.6 mm at the pan ends, origin fixed
    tr, r = run(fr)
    assert r["score"] < 1.0, r


@test
def held_bar_edge_excursion_then_seated_is_one():
    fr = bar_phase()
    k = 12  # a held lift frame: carry the bar's origin past the stand's front edge (x 0.130) while both hands hold it
    bp = np.array([0.10, 0.0, ST + 0.10])
    fr[k] = ((bp, I3), PAN_START, bar_grip_pads(bp), LOADED)
    fr += pan_carry(end=PAN_SEAT_C) + rep(30, BAR_SEAT, PAN_SEAT_C, PARK, OPEN)
    _, r = run(fr)
    assert r["score"] == 1.0 and r["latched"] is None, r


@test
def bar_fallen_to_floor_is_zero():
    fr = bar_phase()[:20] + rep(30, (np.array([0.10, 0.0, -0.0125]), I3), PAN_START, PARK, OPEN)
    _, r = run(fr)
    assert r["score"] == 0.0 and r["latched"] == "bar_off_stand", r


if __name__ == "__main__":
    bad = 0
    for name, fn in TESTS.items():
        try:
            fn()
            print(f"PASS {name}")
        except AssertionError as e:
            bad += 1
            print(f"FAIL {name}: {e}")
    print(f"{len(TESTS) - bad}/{len(TESTS)} passed")
    sys.exit(1 if bad else 0)
