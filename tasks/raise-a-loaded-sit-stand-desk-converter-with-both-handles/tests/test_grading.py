"""Score tiers, latched failures and the two two-arm controls, on hand-made states."""

import math

import numpy as np

from rdb_desk_converter import layout as L
from rdb_desk_converter.grading import HOLD_S, CONTROL_HZ, make_grade

FAR = {"left": np.array((0., .3, .3)), "right": np.array((0., -.3, .3))}
SEATED = {"left": 0., "right": 0.}
OUT = {"left": .016, "right": .016}


def step(g, setting, latches, *, monitor_shift=(0., 0., 0.), monitor_tilt=0., tcps=None, touching=False, seat=True):
    """``setting`` is a notch number (or fraction); with all latches down and ``seat`` the top rests on the pins' low
    side, SEAT_PLAY below the notch's centre, as it does in the simulator."""
    a = L.theta(setting) - (L.SEAT_PLAY if seat and max(latches.values()) <= .002 else 0.)
    top = L.top_origin(a)
    c, s = math.cos(monitor_tilt), math.sin(monitor_tilt)
    rot = np.array(((c, 0., s), (0., 1., 0.), (-s, 0., c)))
    monitor = (top + (L.MONITOR_AT[0], L.MONITOR_AT[1], 0.) + monitor_shift, rot)
    keyboard = (top + (L.KEYBOARD_AT[0], L.KEYBOARD_AT[1], -L.TRAY_DROP), np.eye(3))
    if tcps is None:
        tcps = FAR
    return g.update(a, latches, top, np.eye(3), monitor, keyboard, tcps, touching)


def hold(g, setting, latches, n=None, **kw):
    out = None
    for _ in range(n or round(HOLD_S * CONTROL_HZ)):
        out = step(g, setting, latches, **kw)
    return out


def test_start_scores_zero():
    assert step(make_grade(), 4, SEATED)["score"] == 0.


def test_full_score_needs_lock_at_nine_loads_and_clear_hands():
    g = make_grade()
    step(g, 6, OUT)
    r = hold(g, 9, SEATED, n=2 * round(HOLD_S * CONTROL_HZ))      # 1 s clear for 1.0, then 1 s at 1.0 to end
    assert r["score"] == 1. and r["locked_at"] == 9 and g.finished


def test_hands_still_on_gives_point_eight():
    g = make_grade()
    hands = {s: L.grasp_point(s, L.theta(9)) for s in ("left", "right")}
    r = hold(g, 9, SEATED, tcps=hands, touching=True)
    assert r["score"] == .8


def test_held_at_nine_unlocked_is_point_five():
    g = make_grade()
    assert hold(g, 9, OUT)["score"] == .5


def test_one_lock_seated_is_not_locked():
    g = make_grade()
    r = hold(g, 9, {"left": 0., "right": .014})
    assert r["locked_at"] is None and r["score"] == .5


def test_release_mismatch_locks_short_at_eight():
    g = make_grade()
    step(g, 7.6, {"left": .014, "right": .016})
    assert hold(g, 8, SEATED)["score"] == .5


def test_raised_then_dropped_back_keeps_point_two():
    g = make_grade()
    step(g, 5.2, OUT)
    assert hold(g, 4, SEATED)["score"] == .2


def test_overshoot_locked_at_ten_is_point_five():
    g = make_grade()
    assert hold(g, 10, SEATED)["score"] == .5


def test_slid_monitor_costs_the_last_tier():
    g = make_grade()
    assert hold(g, 9, SEATED, monitor_shift=(.04, 0., 0.))["score"] == .8


def test_fallen_monitor_zeroes_the_trial_for_good():
    g = make_grade()
    step(g, 6, OUT, monitor_tilt=math.radians(60.))
    r = hold(g, 9, SEATED)
    assert r["score"] == 0. and r["fallen"] == "monitor"


def test_between_notches_is_not_locked():
    g = make_grade()
    r = hold(g, 8.5, SEATED)
    assert r["locked_at"] is None and r["score"] == .5


def test_seated_at_nine_reads_below_nine_and_is_locked_at_nine():
    g = make_grade()
    r = step(g, 9, SEATED)
    assert 8.9 < r["setting"] < 8.92 and r["locked_at"] == 9


def test_pins_on_the_high_side_of_the_play_still_name_the_notch():
    g = make_grade()
    r = step(g, 9 + L.SEAT_SETTINGS, SEATED, seat=False)
    assert r["locked_at"] == 9


def test_start_seated_scores_zero_and_slot_play_does_not_count_as_raised():
    g = make_grade()
    assert step(g, 4, SEATED)["score"] == 0.
    assert step(g, 4 + L.SEAT_SETTINGS, SEATED, seat=False)["score"] == 0.
