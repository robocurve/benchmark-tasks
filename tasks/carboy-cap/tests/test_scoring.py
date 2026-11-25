"""Unit tests of the carboy-cap scorer and thread geometry (pure Python; run with ``python -m pytest tests``)."""

import math
import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import layout as L  # noqa: E402
from scoring import Scorer  # noqa: E402

DT = 0.05
NONE = {"cap": False, "funnel": False, "bottle": False}


def run(scorer, n, **kw):
    level = 0.0
    args = dict(gap_mm=None, engaged=False, funnel_in_tray=False, funnel_on_bench=False, contact=NONE,
                bottle_tilt_deg=0.0)
    args.update(kw)
    if "phi_deg" not in kw and args["gap_mm"] is not None:  # consistent thread angle for the gap (helix)
        args["phi_deg"] = 360.0 * (L.TRAVEL * 1e3 - args["gap_mm"]) / (L.PITCH * 1e3)
    for _ in range(n):
        level = scorer.step(DT, **args)
    return level


def test_nothing_done_scores_zero():
    s = Scorer()
    assert run(s, 200) == 0.0


def test_safe_order_scores_one():
    s = Scorer()
    run(s, 20, funnel_on_bench=True, contact={"cap": True, "funnel": True, "bottle": False})  # funnel held, capping
    run(s, 20, gap_mm=0.05, engaged=True, funnel_on_bench=True, contact={"cap": False, "funnel": True, "bottle": False})
    assert run(s, 100, gap_mm=0.05, engaged=True, funnel_in_tray=True) == 1.0
    assert s.sealed_at_stow is True


def test_stow_then_close_earns_full_credit_after_release():
    s = Scorer()
    run(s, 20, funnel_in_tray=True)
    assert s.sealed_at_stow is False and s.veto is None
    assert run(s, 80, gap_mm=0.0, engaged=True, funnel_in_tray=True) == 1.0


def test_cap_on_but_not_down_is_open():
    s = Scorer()
    run(s, 20, gap_mm=4.0, engaged=True, funnel_on_bench=True)
    assert s.veto is None
    assert run(s, 30, gap_mm=4.0, engaged=True, funnel_in_tray=True) == 0.0
    assert s.last["level"] == 0 and not s.last["sealed"]


def test_sealed_but_funnel_not_stowed_is_half():
    s = Scorer()
    assert run(s, 100, gap_mm=0.1, engaged=True, funnel_on_bench=True) == 0.5


def test_sealed_needs_hands_off_for_full_score():
    s = Scorer()
    run(s, 20, gap_mm=0.1, engaged=True, funnel_on_bench=True)
    assert run(s, 40, gap_mm=0.1, engaged=True, funnel_in_tray=True,
               contact={"cap": True, "funnel": False, "bottle": False}) == 0.0


def test_tipped_bottle_vetoes():
    s = Scorer()
    run(s, 5, bottle_tilt_deg=40.0)
    assert s.veto == "bottle_tipped"
    assert run(s, 60, gap_mm=0.1, engaged=True, funnel_in_tray=True) == 0.0


def test_thread_geometry():
    assert abs(L.h_thread(0.0) - L.TRAVEL) < 1e-12
    assert abs(L.h_thread(2 * math.pi) - (L.TRAVEL - L.PITCH)) < 1e-12
    assert L.h_thread(L.PHI_SEAT) == 0.0
    assert L.h_thread(L.PHI_SEAT + 1.0) == 0.0
    # the seal threshold corresponds to about 21 deg short of the seat
    short = (L.SEALED_GAP / L.PITCH) * 360.0
    assert 15.0 < short < 30.0


def test_seeded_starts_are_varied_and_valid():
    xs = []
    for seed in range(300):
        sp = L.setup_spec(("seed", seed))
        d = math.hypot(sp["bx"] - L.NOMINAL[0], sp["by"] - L.NOMINAL[1])
        assert L.BOTTLE_D[0] - 1e-9 <= d <= L.BOTTLE_D[1] + 1e-9
        assert math.hypot(sp["cx"] - sp["bx"], sp["cy"] - sp["by"]) >= L.CAP_MIN_DIST - 1e-9
        xs.append(sp["bx"])
    assert max(xs) - min(xs) > 0.03


def test_pushed_down_short_of_seat_is_not_sealed():
    """A cap pressed within the gap line at 0.65 turns (hard push on the compliant thread) is not sealed."""
    s = Scorer()
    run(s, 20, gap_mm=0.05, engaged=True, funnel_on_bench=True, phi_deg=360.0 * 0.65,
        contact={"cap": True, "funnel": True, "bottle": False})
    run(s, 20, gap_mm=0.05, engaged=True, funnel_in_tray=True, phi_deg=360.0 * 0.65,
        contact={"cap": True, "funnel": False, "bottle": False})
    assert s.last["level"] == 0 and not s.last["sealed"]


def test_no_seeded_start_is_jointly_near_the_default():
    """Open-loop guard: no seeded start has bottle < 15 mm, tab < 25 deg and cap < 30 mm from the official start,
    and the documented evidence seeds keep their starts."""
    ref = L.setup_spec(("seed", L.OFFICIAL_SEED))
    for seed in range(20000):
        if seed == L.OFFICIAL_SEED:
            continue
        sp = L.setup_spec(("seed", seed))
        near = (math.hypot(sp["bx"] - ref["bx"], sp["by"] - ref["by"]) < L.GUARD_BOTTLE
                and abs(sp["tab"] - ref["tab"]) < L.GUARD_TAB_DEG)
        assert not (near and math.hypot(sp["cx"] - ref["cx"], sp["cy"] - ref["cy"]) < L.GUARD_CAP), seed
        assert math.hypot(sp["cx"] - sp["bx"], sp["cy"] - sp["by"]) >= L.CAP_MIN_DIST - 1e-9
    for seed in (L.OFFICIAL_SEED, 7, 3, 99, 11):
        assert L.setup_spec(("seed", seed)) == L._seeded(seed)


def test_release_dwell_starts_at_final_pose_and_restarts_after_interruption():
    final = dict(gap_mm=0.0, engaged=True, funnel_in_tray=True)
    for change in (dict(funnel_still=False), dict(funnel_in_tray=False),
                   dict(contact={"bottle": True}), dict(gap_mm=1.0)):
        s = Scorer()
        run(s, 100, gap_mm=0.0, engaged=True, funnel_on_bench=True)
        assert run(s, 79, **final) == 0.0
        run(s, 1, **(final | change))
        assert run(s, 79, **final) == 0.0
        assert run(s, 1, **final) == 1.0


def test_half_requires_four_seconds_with_seated_cap():
    s = Scorer()
    run(s, 100, funnel_on_bench=True)
    args = dict(gap_mm=0, engaged=True, funnel_on_bench=True)
    assert run(s, 79, **args) == 0
    assert run(s, 1, **args) == 0.5


def test_tray_requires_floor_support():
    fp = np.array([sum(L.TRAY_X) / 2, sum(L.TRAY_Y) / 2, L.TOP + L.TRAY_Z[0] + L.F_STEM_L])
    assert L.in_tray(fp, np.eye(3))
    assert not L.in_tray(fp + np.array([0, 0, 0.015]), np.eye(3))
    assert not L.in_tray(fp + np.array([0, 0.15, 0]), np.eye(3))


def test_stowed_funnel_with_open_bottle_or_reopened_cap_scores_zero():
    s = Scorer()
    assert run(s, 100, funnel_in_tray=True) == 0.0
    assert run(s, 40, gap_mm=0.0, engaged=True, funnel_in_tray=True) == 0.0
    assert run(s, 100, funnel_in_tray=True) == 0.0


def test_funnel_stem_must_drain_into_the_tray():
    rotation = np.array([[0., 0., 1.], [0., -1., 0.], [1., 0., 0.]])
    pos = np.array([L.TRAY_X[0] + .007, sum(L.TRAY_Y)/2, L.TOP + L.TRAY_Z[0] + L.F_R_TOP])
    assert not L.in_tray(pos, rotation)
    assert L.in_tray(pos + [.05, 0., 0.], rotation)
