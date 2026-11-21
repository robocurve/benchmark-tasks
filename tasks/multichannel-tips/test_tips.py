"""Offline checks of the tip-fit law, its calibration and the scoring, with no simulator: python -m pytest test_tips.py"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import tiplaw as L  # noqa: E402
from tip_rules import FAILS, Snapshot, Tracker, level, seated, tier  # noqa: E402


# -- the fit -------------------------------------------------------------------------------------------------------------

def test_seat_force_and_geometry():
    assert 3.5 <= L.F_SEAT <= 4.0
    assert L.fit_force(L.U_FULL, 0.0)[0] == pytest.approx(L.F_SEAT)
    assert L.S_FULL == pytest.approx(L.NOZZLE_LEN + L.TIP_LEN - L.FULL_SEAT_MM)
    assert L.S_SHORT == pytest.approx(L.S_FULL - 1.5)


def test_plastic_then_elastic():
    f, d = L.fit_force(1.0, 0.0)
    assert f == pytest.approx(L.K_CONE) and d == 1.0
    f, d = L.fit_force(0.9, 1.0)  # backing off: stiff elastic unloading, depth kept
    assert f == pytest.approx(L.K_CONE - L.K_ELASTIC * 0.1) and d == 1.0
    assert L.fit_force(0.5, 1.0)[0] == 0.0
    f, d = L.fit_force(L.U_FULL + 1.0, 1.0)  # past full seat: the rack plate gives
    assert f == pytest.approx(L.F_SEAT + L.K_HARD) and d == L.U_FULL


def test_straight_push_calibration():
    """A level straight push needs PEAK_STRAIGHT_N (within 10%) to bring all eight to full seat."""
    assert L.straight_push_total() == pytest.approx(L.PEAK_STRAIGHT_N, rel=0.10)


def test_straight_push_at_rocking_force_leaves_end_tips_short():
    """At the rocking peak's force, a level push brings none of the end tips to full seat."""
    for s in np.arange(L.S_CONTACT, L.S_FULL + 1.0, 0.01):
        f, d = L.fit_forces(L.row_depths(s, 0.0), np.zeros(8))
        if f.sum() >= L.PEAK_ROCK_N * 1.1:
            break
    assert d[0] < L.U_FULL and d[7] < L.U_FULL
    assert np.count_nonzero(d >= L.U_FULL) <= 2


def test_rocking_calibration():
    """The best quasi-static rocking seats all eight at about PEAK_ROCK_N, about 3 tips loaded at a time."""
    peak, log = L.best_rock_peak()
    assert peak == pytest.approx(L.PEAK_ROCK_N, rel=0.10)
    assert peak < 0.7 * L.ARM_PUSH_N


# -- the scoring ---------------------------------------------------------------------------------------------------------

def snap(n_full=8, n_short=0, grip=True, tilt=2.0, end_z=0.90, rim=0.80, fails=()):
    tips = [(True, L.FULL_SEAT_MM, end_z)] * n_full + [(True, L.FULL_SEAT_MM + 1.5, end_z - 0.0015)] * n_short
    tips += [(False, 80.0, 0.76)] * (8 - n_full - n_short)
    return Snapshot(tips=tips, in_gripper=grip, tilt_deg=tilt, rim_z=rim, fails=set(fails))


def run(snaps):
    tr = Tracker(dt=0.05)
    for s in snaps:
        tr.update(s)
    return tr


def test_seated_and_level():
    assert seated(True, 54.9) and not seated(True, 55.5) and not seated(False, 54.0)
    assert level([54.0, 54.9]) and not level([53.2, 54.9])


@pytest.mark.parametrize("n,expect", [(8, 1.0), (7, 0.5), (6, 0.25), (5, 0.0), (0, 0.0)])
def test_tiers(n, expect):
    assert run([snap(n_full=n)] * 41).score() == expect


def test_short_tip_does_not_count():
    assert run([snap(n_full=7, n_short=1)] * 41).score() == 0.5


def test_hold_two_seconds():
    assert run([snap()] * 39).score() == 0.0
    assert run([snap()] * 41).score() == 1.0
    assert run([snap()] * 30 + [snap(grip=False)] + [snap()] * 30).score() == 0.0


def test_held_conditions():
    assert run([snap(tilt=11.0)] * 50).score() == 0.0
    assert run([snap(end_z=0.825, rim=0.80)] * 50).score() == 0.0  # tips only 2.5 cm above the rim
    assert run([snap(grip=False)] * 50).score() == 0.0


def test_not_level_scores_zero():
    tips = [(True, 53.1, 0.9)] + [(True, 54.9, 0.9)] * 7
    assert run([Snapshot(tips=tips, in_gripper=True, tilt_deg=0.0, rim_z=0.8)] * 50).score() == 0.0


@pytest.mark.parametrize("fail", sorted(FAILS))
def test_fails_latch(fail):
    s = [snap()] * 10 + [snap(fails=[fail])] + [snap()] * 50
    tr = run(s)
    assert tr.score() == 0.0 and not tr.success_now()


def test_success_state():
    tr = run([snap()] * 5)
    assert tr.success_now()
    assert not run([snap(n_full=7)] * 5).success_now()


def test_tier_function():
    assert tier(8, True, 2.0, set()) == 1.0
    assert tier(8, False, 5.0, set()) == 0.0
    assert tier(8, True, 5.0, {"box_moved"}) == 0.0


# -- the tip-touch rule in the physics step (tipkit.Model, with the simulator's views stubbed) ---------------------------

def _model_with_stubs(contact):
    import types

    if "yam_arena.task_file" not in sys.modules:
        try:
            import yam_arena.task_file  # noqa: F401
        except ImportError:  # offline: tipkit's geometry needs only these two names to import
            pkg, tf = types.ModuleType("yam_arena"), types.ModuleType("yam_arena.task_file")
            tf.Box = tf.ObjectState = type("Stub", (), {})
            pkg.task_file = tf
            sys.modules.update({"yam_arena": pkg, "yam_arena.task_file": tf})
    import tipkit

    m = tipkit.Model.__new__(tipkit.Model)
    m.reset_state()
    m.tip_contacts = types.SimpleNamespace(get_contact_force_matrix=lambda dt: contact)
    high = np.array([0.31, -0.19, 0.95, 0.0, 0.0, 0.0, 1.0])  # the pipette far above the box: the fit model idles
    m._poses = lambda: [(high[:3], np.eye(3))] * 10
    return m


def test_gripper_touch_flagged_with_pipette_away_from_box():
    cm = np.zeros((8, 5, 3))
    m = _model_with_stubs(cm)
    m._fits(1.0 / 240.0)
    assert "tip_touched" not in m.flags
    cm[3, 2] = [0.0, 0.0, 0.2]  # a finger pad on tip D4
    m._fits(1.0 / 240.0)
    assert "tip_touched" in m.flags
    cm2 = np.zeros((8, 5, 3))
    cm2[5, 0] = [0.0, 0.0, 5.0]  # a tip on the box is not a touch
    m2 = _model_with_stubs(cm2)
    m2._fits(1.0 / 240.0)
    assert "tip_touched" not in m2.flags
