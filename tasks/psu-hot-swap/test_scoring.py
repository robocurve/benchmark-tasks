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
                     sliders={"latch": latch, "contact_l": 0.0, "contact_r": 0.0})


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
    for objs in frames:              # as the runtime does: success() then score() on each step's states
        ok = T.success(objs)
        out = (T.score(objs), ok)
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


HELD = 2 * T.HOLD_STEPS   # still over 1 s (21 samples), then held for 1 s


def test_seated_and_latched_succeeds_after_one_second():
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=HOME)] * HELD
    assert _run(frames) == (1.0, True)
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=(0.001, 0, 0))] * HELD   # against the back stop
    assert _run(frames) == (1.0, True)
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=HOME)] * (HELD - 1)
    assert _run(frames) == (0.75, False)   # one step short of the full hold: no success yet


def test_full_score_only_after_the_condition_has_held_one_second():
    # The spare comes to rest home: it must be still over 1 s (21 samples, 20 intervals), and that must then hold for
    # another 1 s (20 steps) before score() gives 1.0. success() turns true on that same step, never earlier.
    T._reset_state()
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=HOME)] * 60
    seen = []
    for objs in frames:
        ok = T.success(objs)
        seen.append((T.score(objs), ok))
    first_ok = next(i for i, (_, ok) in enumerate(seen) if ok)
    first_full = next(i for i, (sc, _) in enumerate(seen) if sc == 1.0)
    assert first_full == 1 + T.HOLD_STEPS + T.HOLD_STEPS - 1   # still over 1 s, then held for 20 steps (1 s)
    assert first_ok == first_full                              # success() only once the full hold is complete
    assert all(ok == (sc == 1.0) for sc, ok in seen)
    assert all(sc == 0.75 for sc, _ in seen[1:first_full])     # seated but not yet held: partial credit


def test_runtime_can_only_end_the_episode_at_full_score():
    # The runtime ends the episode once success() has been true for its own 1 s (round(1.0 * 20) steps in a row).
    # Whatever the spare does, the score on the step the episode ends is 1.0.
    hold = round(1.0 * 20)
    patterns = {
        "settles": [HOME] * 80,
        "knocked": [HOME] * 45 + [(-0.004, 0, 0)] + [HOME] * 60,
        "creeps then settles": [(-0.0012 + 0.0001 * i, 0, 0) for i in range(20)] + [HOME] * 70,
    }
    for name, spares in patterns.items():
        T._reset_state()
        held, ended = 0, None
        for i, sp in enumerate([None] + spares):
            objs = _objects(failed=OUT) if sp is None else _objects(failed=AWAY, spare=sp)
            held = held + 1 if T.success(objs) else 0
            sc = T.score(objs)
            if held >= hold:
                ended = (i, sc)
                break
        assert ended is not None and ended[1] == 1.0, name


def test_moving_spare_never_scores_full():
    # Home, but creeping 0.1 mm a step (2 mm over 1 s): never still, so never 1.0.
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=(-0.0012 + 0.0001 * (i % 25), 0, 0))
                                       for i in range(80)]
    T._reset_state()
    scores = []
    for objs in frames:
        T.success(objs)
        scores.append(T.score(objs))
    assert max(scores) == 0.75


def test_hold_resets_when_disturbed():
    # Held for 19 steps, then knocked out of place for one step: the 1 s count starts again.
    T._reset_state()
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=HOME)] * (2 * T.HOLD_STEPS - 1)
    frames += [_objects(failed=AWAY, spare=(-0.004, 0, 0))]
    frames += [_objects(failed=AWAY, spare=HOME)] * (2 * T.HOLD_STEPS - 1)
    out = []
    for objs in frames:
        T.success(objs)
        out.append(T.score(objs))
    assert max(out) == 0.75
    assert _run(frames + [_objects(failed=AWAY, spare=HOME)])[0] == 1.0


def test_score_without_success_call_is_the_same():
    # The runtime calls success() then score() on each step; called alone, score() still counts each step once.
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=HOME)] * (2 * T.HOLD_STEPS)
    T._reset_state()
    alone = [T.score(objs) for objs in frames]
    T._reset_state()
    paired = []
    for objs in frames:
        T.success(objs)
        paired.append(T.score(objs))
    assert alone == paired and alone[-1] == 1.0 and alone[-2] == 0.75


def test_seated_but_latch_not_caught():
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=HOME, spare_latch=-0.004)] * HELD
    assert _run(frames)[0] == 0.75


def test_short_of_home():
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=(-0.003, 0, 0))] * HELD
    assert _run(frames)[0] == 0.75


def test_seated_but_tilted():
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=HOME, spare_tilt=math.radians(5))] * HELD
    assert _run(frames)[0] == 0.75


def test_failed_left_in_front_of_bay_blocks_full_score():
    # The failed supply pushed back into (or left in) the bay's line counts as not clear.
    frames = [_objects(failed=OUT)] + [_objects(failed=(-0.05, 0, 0), spare=HOME)] * HELD
    assert _run(frames)[0] == 0.75


def test_live_supply_disturbed_is_zero_for_good():
    frames = [_objects(failed=OUT), _objects(failed=AWAY, live=(-0.006, 0, 0))]
    frames += [_objects(failed=AWAY, spare=HOME)] * HELD
    assert _run(frames) == (0.0, False)


def test_live_supply_nudged_under_limit_is_fine():
    frames = [_objects(failed=OUT)] + [_objects(failed=AWAY, spare=HOME, live=(-0.004, 0, 0))] * HELD
    assert _run(frames) == (1.0, True)


# -- geometry ---------------------------------------------------------------------------------------------------------

def _world_boxes(p):
    base = np.array(p.pos)
    out = [(p.name, base + np.array(b[0]), np.array(b[1])) for b in p.boxes if len(b) < 4 or not b[3]]
    for sl in p.sliders:
        out += [(f"{p.name}:{sl.name}", base + np.array(b[0]), np.array(b[1])) for b in sl.boxes]
    return out


def _footprint(b, base=(0.0, 0.0), dx=0.0, dy=0.0):
    """The box's corners in the table plane (x, y) and its z range."""
    (cx, cy, cz), (sx, sy, sz) = b[0], b[1]
    a = b[3] if len(b) > 3 else 0.0
    u, v = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
    c = np.array([cx + base[0] + dx, cy + base[1] + dy])
    return [c + i * sx / 2 * u + j * sy / 2 * v for i in (-1, 1) for j in (-1, 1)], (cz - sz / 2, cz + sz / 2)


def _overlap(a, b, tol=1e-5) -> bool:
    """Do two (possibly yawed) boxes overlap by more than ``tol``? Separating axes in the plane, plus z."""
    (pa, za), (pb, zb) = a, b
    if min(za[1], zb[1]) - max(za[0], zb[0]) <= tol:
        return False
    for poly in (pa, pb):
        for i, j in ((0, 1), (0, 2)):
            e = poly[j] - poly[i]
            n = np.array([-e[1], e[0]]) / np.linalg.norm(e)
            ra, rb = [p @ n for p in pa], [p @ n for p in pb]
            if min(max(ra), max(rb)) - max(min(ra), min(rb)) <= tol:
                return False
    return True


def _world_boxes(p):
    base = np.array(p.pos)[:2]
    out = [(p.name, _footprint(b, base)) for b in p.boxes]
    for sl in p.sliders:
        out += [(f"{p.name}:{sl.name}", _footprint(b, base)) for b in sl.boxes]
    return out


def test_nothing_overlaps_at_the_start():
    boxes = [b for p in T.OBJECTS for b in _world_boxes(p)]
    hits = set()
    for a, b in itertools.combinations(boxes, 2):
        if a[0].split(":")[0] == b[0].split(":")[0]:
            continue
        if _overlap(a[1], b[1]):
            hits.add(tuple(sorted((a[0], b[0]))))
    assert hits == set()


def test_hooks_sit_in_their_notches_and_retract_clear():
    hook = T._latch().boxes[2]                        # the catch (the ramp behind it is checked below)
    y_out = hook[0][1] + hook[1][1] / 2                 # the hook's outer face, supply frame
    assert y_out - T.LATCH_TRAVEL <= T.PSU_W / 2 + 1e-9  # tab pushed to its stop, the hook is flush with the side ...
    assert y_out - T.LATCH_TRAVEL < T.BAY_W / 2 - 0.001  # ... and clear of the bay wall
    assert y_out > T.BAY_W / 2                          # at rest, it stands past the bay wall's face
    x0, x1 = hook[0][0] - hook[1][0] / 2, hook[0][0] + hook[1][0] / 2
    back_stop_gap = T.BACK_X - T.PSU_L / 2              # how far past home a supply can be pushed
    assert T.NOTCH_X[0] <= x0 and x1 + back_stop_gap <= T.NOTCH_X[1]   # fits the notch, home or against the stop
    (cx, cy, _), (length, thick, _), _, a = T._hook_ramp()
    crest_x, crest_y = T.RAMP_TOP[0] / 1000, T.RAMP_TOP[1] / 1000
    wall_x = crest_x + (crest_y - T.BAY_W / 2) / math.tan(abs(a))   # where the ramp's face dips inside the wall's face
    assert T.NOTCH_X[0] <= crest_x and wall_x + back_stop_gap <= T.NOTCH_X[1]   # the ramp fits the notch too
    assert crest_y - T.LATCH_TRAVEL <= T.BAY_W / 2 - 0.001 + 1e-9                  # and retracts 1 mm clear of the wall
    assert crest_y - 0.001 >= y_out - 1e-9   # the crest stands 1 mm above the catch: the catch passes the mouth 1 mm in
    assert math.degrees(abs(a)) < 30         # shallower than the 30-degree lead-in, so the mouth's edge rides the ramp


def test_handle_is_fixed_and_tab_is_separate():
    # The handle bar is part of the supply's rigid body; the release tab is on the latch slider, beside it.
    handle = [b for b in T._psu_boxes((0, 0, 0)) if abs(b[0][0] - T.GRIP_X) < 1e-6]
    tab = [b for b in T._latch().boxes if abs(b[0][0] - T.GRIP_X) < 1e-6]
    assert len(handle) == 1 and len(tab) == 1
    h_hi = handle[0][0][1] + handle[0][1][1] / 2
    t_lo = tab[0][0][1] - tab[0][1][1] / 2
    assert abs(t_lo - T.TAB_Y[0]) < 1e-9 and abs(h_hi - T.HANDLE_Y[1]) < 1e-9
    assert t_lo - T.LATCH_TRAVEL - h_hi >= 0.001 - 1e-9     # pushed to its stop, the tab stays 1 mm off the handle
    assert T._latch().mass >= 0.03


PAD_T, PAD_W, PAD_TIP, PAD_FLAT = 0.0064, 0.027, 0.014, 0.024


def _pads(y_lo, y_hi):
    """The two pads' boxes (supply frame) with their inner faces at y_lo and y_hi, tips 14 mm above the table."""
    z = (PAD_TIP, PAD_TIP + PAD_FLAT)
    mk = lambda y0, y1: ((T.GRIP_X, (y0 + y1) / 2, sum(z) / 2), (PAD_W, y1 - y0, z[1] - z[0]))  # noqa: E731
    return [mk(y_lo - PAD_T, y_lo), mk(y_hi, y_hi + PAD_T)]


def test_pinch_fits_handle_and_tab():
    # Opened to 56 mm about GRIP_Y (the demo's PINCH_OPEN), the pads clear handle and tab by over 15 mm; closed, they
    # rest on the handle's outer face and on the tab pushed to its stop, and touch nothing else of the supply (handle
    # arms, tab link, face).
    opened = T.GRIP_Y - 0.028, T.GRIP_Y + 0.028
    assert opened[0] < T.HANDLE_Y[0] - 0.015 and opened[1] > T.TAB_Y[1] + 0.015
    closed = T.HANDLE_Y[0], T.TAB_Y[1] - T.LATCH_TRAVEL
    others = [b for b in T._psu_boxes((0, 0, 0)) if abs(b[0][0] - T.GRIP_X) > 1e-6]
    others += [b for b in T._latch().boxes if abs(b[0][0] - T.GRIP_X) > 1e-6]
    for y_lo, y_hi in (opened, closed):
        for pad in _pads(y_lo, y_hi):
            for b in others:
                assert not _overlap(_footprint(pad), _footprint(b), tol=-0.002), (pad, b)   # 2 mm margin


def _contact_spread_needed(s):
    """How far each connector contact must spread (m) with the supply ``s`` metres past home, from the geometry."""
    ridges = [b for b in T._blade(0.0) if len(b) > 3]
    need = []
    for sl in T._contacts():
        side = 1 if sl.sign > 0 else -1
        for d in np.arange(0.0, 0.006, 0.0001):
            box = _footprint(sl.boxes[0], dx=s, dy=side * d)
            if not any(_overlap(box, _footprint(r)) for r in ridges):
                need.append(d)
                break
        else:
            need.append(np.inf)
    return need


def test_connector_mates_over_the_last_eight_mm():
    profile = {round(s * 1000, 1): _contact_spread_needed(s) for s in np.arange(-0.012, 0.0016, 0.0005)}
    assert all(max(v) < 1e-6 for s, v in profile.items() if s <= -8.5)        # free until 8 mm short of home
    assert all(max(v) < 1e-6 for s, v in profile.items() if s >= -1.0)        # clear of the ridge from 1 mm short
    peak = max(max(v) for v in profile.values())
    assert 0.0019 < peak < T.CONTACT_TRAVEL - 0.002                           # spread 2 mm, well inside its travel
    # The mating force this needs (the contacts climbing the 50-degree ramp, friction 0.2): about 36 N.
    k, mu, a = T.CONTACT_RATE, 0.2, math.radians(T.RIDGE_DEG)
    force = 2 * k * 0.002 * (math.tan(a) + mu) / (1 - mu * math.tan(a))
    assert 30 < force < 42


def test_layout_has_room():
    # Pulled out, the failed supply's back face clears the live supply's handle; parked, it clears the bay's line.
    live_handle_front = T.CAGE_XY[0] - 0.132
    assert T.CAGE_XY[0] - T.PULL + T.PSU_L / 2 < live_handle_front - 0.005
    assert abs(T.CLEAR_DY) > T.PSU_W + 0.01
    assert T.SEAT_DX - T.PULL < T.OUT_X
    # The spare starts clear of the bay's line and of the failed supply's pulled-out spot.
    assert abs(T.SPARE_XY[1] - T.CAGE_XY[1]) > T.PSU_W + 0.05
