"""Offline checks of task.py (no GPU, no Isaac Sim): every scoring outcome from fabricated object states, and the
starting layout's geometry.

    python -m pytest test_scoring.py
"""

from __future__ import annotations

import importlib
import itertools
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from psukit import PartState  # noqa: E402

T = importlib.import_module("task")

SEAT = np.array([T.CAGE_XY[0], T.CAGE_XY[1], T.TOP])


def _state(pos, latch=0.0, tilt=0.0):
    quat = np.array([0.0, math.sin(tilt / 2), 0.0, math.cos(tilt / 2)])   # tilt about y
    p = np.asarray(pos, float)
    return PartState(pos=p, quat_xyzw=quat, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p, high=p,
                     sliders={"latch": latch, "plug": 0.0})


def _objects(failed=(0, 0, 0), spare=None, live=(0, 0, 0), spare_latch=0.0, spare_tilt=0.0):
    spare_pos = np.array(T.SPARE.pos) if spare is None else SEAT + spare
    return {
        "cage": None,
        "failed_psu": _state(SEAT + failed),
        "live_psu": _state(np.array(T.LIVE.pos) + live),
        "spare_psu": _state(spare_pos, spare_latch, tilt=spare_tilt),
    }


def _run(frames):
    T._reset_state()
    out = (0.0, False)
    for objs in frames:
        out = (T.score(objs), T.success(objs))
    return out


OUT = (-T.PULL, 0, 0)
AWAY = (-T.PULL, T.CLEAR_DY, 0)
HOME = (0.0, 0, 0)


def test_start_scores_zero():
    assert _run([_objects()]) == (0.0, False)


def test_pulled_two_cm():
    assert _run([_objects(failed=(-0.025, 0, 0))])[0] == 0.25


def test_pulled_out():
    assert _run([_objects(failed=OUT)])[0] == 0.5


def test_spare_ten_cm_in():
    in_x = T.IN_X + 0.001
    assert _run([_objects(failed=OUT), _objects(failed=AWAY, spare=(in_x, 0, 0))])[0] == 0.75
    assert _run([_objects(failed=OUT), _objects(failed=AWAY, spare=(in_x - 0.01, 0, 0))])[0] == 0.5


def test_spare_in_needs_failed_out_first():
    assert _run([_objects(spare=(-0.05, 0.13, 0))])[0] == 0.0


def test_seated_and_latched_succeeds_after_one_second():
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=HOME)] * 21
    assert _run(frames) == (1.0, True)
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=(0.0014, 0, 0))] * 21   # pushed to the stop
    assert _run(frames) == (1.0, True)


def test_seated_but_latch_not_caught():
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=HOME, spare_latch=-0.004)] * 21
    assert _run(frames)[0] == 0.75


def test_short_of_home():
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=(-0.003, 0, 0))] * 21
    assert _run(frames)[0] == 0.75


def test_seated_but_tilted():
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=HOME, spare_tilt=math.radians(5))] * 21
    assert _run(frames)[0] == 0.75


def test_failed_left_in_front_of_bay_blocks_full_score():
    # The failed supply pushed back into (or left in) the bay's line counts as not clear.
    frames = [_objects(failed=OUT)] + [_objects(failed=(-0.05, 0, 0), spare=HOME)] * 21
    assert _run(frames)[0] == 0.75


def test_live_supply_disturbed_is_zero_for_good():
    frames = [_objects(failed=OUT), _objects(failed=AWAY, live=(-0.006, 0, 0))]
    frames += [_objects(failed=AWAY, spare=HOME)] * 21
    assert _run(frames) == (0.0, False)


def test_live_supply_nudged_under_limit_is_fine():
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=HOME, live=(-0.004, 0, 0))] * 21
    assert _run(frames) == (1.0, True)


# -- geometry ---------------------------------------------------------------------------------------------------------

def _world_boxes(p):
    base = np.array(p.pos)
    out = [(p.name, base + np.array(b[0]), np.array(b[1])) for b in p.boxes if len(b) < 4 or not b[3]]
    for sl in p.sliders:
        out += [(f"{p.name}:{sl.name}", base + np.array(b[0]), np.array(b[1])) for b in sl.boxes]
    return out


def test_nothing_overlaps_at_the_start():
    boxes = [b for p in T.OBJECTS for b in _world_boxes(p)]
    hits = set()
    for a, b in itertools.combinations(boxes, 2):
        if a[0].split(":")[0] == b[0].split(":")[0]:
            continue
        depth = np.minimum(a[1] + a[2] / 2, b[1] + b[2] / 2) - np.maximum(a[1] - a[2] / 2, b[1] - b[2] / 2)
        if (depth > 1e-5).all():
            hits.add(tuple(sorted((a[0], b[0]))))
    assert hits == set()


def test_hooks_sit_in_their_notches_and_retract_clear():
    hook = T._latch().boxes[1]                        # the catch (the ramp behind it is checked below)
    y_out = hook[0][1] + hook[1][1] / 2                 # the hook's outer face, supply frame
    assert y_out - T.LATCH_TRAVEL <= T.PSU_W / 2 + 1e-9  # slid aside, it is flush with the supply's side ...
    assert y_out - T.LATCH_TRAVEL < T.BAY_W / 2 - 0.001  # ... and clear of the bay wall
    assert y_out > T.BAY_W / 2                          # at rest, it stands past the bay wall's face
    x0, x1 = hook[0][0] - hook[1][0] / 2, hook[0][0] + hook[1][0] / 2
    assert T.NOTCH_X[0] <= x0 and x1 + 0.0015 <= T.NOTCH_X[1]   # fits the notch, home or pushed to the stop
    (cx, cy, _), (length, thick, _), _, a = T._hook_ramp()
    ramp_back_x = cx + math.cos(a) * length / 2 + abs(math.sin(a)) * thick / 2
    assert ramp_back_x + 0.0015 <= T.NOTCH_X[1]        # the ramp fits the notch too
    assert math.degrees(abs(a)) < 30                    # shallow enough for the wall edge to cam the hook in


def test_finger_pads_clear_the_guide_arms():
    # The pads (27 mm wide along x, 6.4 mm thick, flat from their tips 14 mm above the table to 24 mm above that, plus
    # a 4 mm margin) close on the bar's faces;
    # before and after the 6 mm shift neither pad's volume may cross a guide arm.
    pad_x = (T.GRIP_X - 0.0137, T.GRIP_X + 0.0137)
    bar_lo, bar_hi = -0.004, 0.004
    pad_z = (0.014, 0.042)
    arms = [b for b in T._psu_boxes((0, 0, 0)) if abs(b[0][0] + 0.10825) < 1e-6]
    for dy in (0.0, -0.006):
        for y0, y1 in ((bar_lo - 0.0064 + dy, bar_lo + dy), (bar_hi + dy, bar_hi + 0.0064 + dy)):
            for c, s, _ in arms:
                ox = min(pad_x[1], c[0] + s[0] / 2) - max(pad_x[0], c[0] - s[0] / 2)
                oy = min(y1, c[1] + s[1] / 2) - max(y0, c[1] - s[1] / 2)
                oz = min(pad_z[1], c[2] + s[2] / 2) - max(pad_z[0], c[2] - s[2] / 2)
                assert not (ox > 0 and oy > 0 and oz > 0), (dy, y0, c)


def test_layout_has_room():
    # Pulled out, the failed supply's back face clears the live supply's handle; parked, it clears the bay's line.
    live_handle_front = T.CAGE_XY[0] - 0.132
    assert T.CAGE_XY[0] - T.PULL + T.PSU_L / 2 < live_handle_front - 0.005
    assert abs(T.CLEAR_DY) > T.PSU_W + 0.01
    assert T.SEAT_DX - T.PULL < T.OUT_X
    # The spare starts clear of the bay's line and of the failed supply's pulled-out spot.
    assert abs(T.SPARE_XY[1] - T.CAGE_XY[1]) > T.PSU_W + 0.05
