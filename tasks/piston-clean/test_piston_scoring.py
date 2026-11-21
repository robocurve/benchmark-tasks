"""Testy rdzenia punktacji: pełne czyszczenie (pociągnięcia + płukanie z ruchem) daje 1.0, exploity mniej.

Uruchomienie: python -m pytest -q tasks/piston-clean/test_piston_scoring.py
"""

import math

import numpy as np
import pytest

import piston_scoring as ps

TRAY = np.array([0.30, 0.20, 0.0])
TRAY_FLOOR = 0.0
STILL = np.zeros(3)
PAD_SIDE_Q = np.array([0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)])  # cienką osią wzdłuż x
FLAT_Q = np.array([0.0, 0.0, 0.0, 1.0])
PISTON_POS = np.array([0.0, 0.0, 0.10])
CORE = ps.PAD_SIZE[0] / 2 - ps.SOFT_LAYER


def yaw_q(a):
    return np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])


def bbox(pos):
    h = np.array([ps.PISTON_D / 2, ps.PISTON_D / 2, ps.PISTON_H / 2])
    return pos - h, pos + h


def step(ledger, pad_pos, pad_q, piston_q=FLAT_Q, piston_pos=PISTON_POS, vel=STILL, flat=None):
    lo, hi = bbox(piston_pos)
    fp, fq = (flat, FLAT_Q) if flat is not None else (None, None)
    ledger.step(piston_pos, piston_q, vel, lo, hi, pad_pos, pad_q, TRAY, TRAY_FLOOR, fp, fq)


def stroke(ledger, centre, q, direction, piston_q=FLAT_Q, length=0.02, passes=2, n=10):
    """Dociśnięta gąbka w tę i z powrotem wzdłuż `direction` (passes razy, amplituda length)."""
    direction = np.asarray(direction, dtype=float)
    for k in range(n * passes + 1):
        step(ledger, centre + direction * length * math.sin(2 * math.pi * k / n), q, piston_q)


def rinse(ledger, metres=0.10):
    base = TRAY + np.array([0.0, 0.0, ps.PAD_SIZE[2] / 2 + 0.002])
    for k in range(int(metres / 0.02) + 1):
        step(ledger, base + np.array([0.02 * (k % 2), 0.0, 0.0]), FLAT_Q)


def side_pad_pos(dz=0.0):
    return PISTON_POS + np.array([ps.PISTON_D / 2 + CORE - 0.001, 0.0, dz])


def wipe_everything(ledger, rinse_when_loaded=True):
    for k in range(ps.N_AROUND):
        if rinse_when_loaded and ledger.load > ps.CAPACITY - 25:
            rinse(ledger)
        stroke(ledger, side_pad_pos(-0.004), PAD_SIDE_Q, (0, 0, 1), yaw_q(k * 2 * math.pi / ps.N_AROUND), length=0.012)
    for dx, dy in [(a, b) for a in (-0.02, 0.0, 0.02) for b in (-0.02, 0.0, 0.02)]:
        if rinse_when_loaded and ledger.load > ps.CAPACITY - 16:
            rinse(ledger)
        top = PISTON_POS + np.array([dx, dy, ps.PISTON_H / 2 + CORE - 0.001])
        stroke(ledger, top, FLAT_Q, (1, 0, 0))


def wipe_rod(ledger, rod_pos=np.array([0.5, 0.0, 0.1])):
    lo, hi = bbox(PISTON_POS)
    hx, hy = ps.ROD_COLLIDER[0] / 2, ps.ROD_COLLIDER[1] / 2
    for off in ((hx + CORE - 0.001, 0, 0), (-(hx + CORE - 0.001), 0, 0), (0, hy + CORE - 0.001, 0), (0, -(hy + CORE - 0.001), 0)):
        for zc in (-0.03, 0.0, 0.025):
            if ledger.load > ps.CAPACITY - 12:
                rinse(ledger)
            for k in range(21):
                pad = rod_pos + np.array(off) + np.array([0, 0, zc + 0.012 * math.sin(2 * math.pi * k / 10)])
                ledger.step(PISTON_POS, FLAT_Q, STILL, lo, hi, pad, FLAT_Q, TRAY, TRAY_FLOOR, None, None, rod_pos, FLAT_Q)


def wipe_inside(ledger):
    for k in range(ps.N_AROUND):
        a = 2 * math.pi * k / ps.N_AROUND
        if ledger.load > ps.CAPACITY - 12:
            rinse(ledger)
        r = ps.INNER_R - CORE + 0.001
        c = PISTON_POS + np.array([r * math.cos(a), r * math.sin(a), -0.009])
        stroke(ledger, c, yaw_q(a), (0, 0, 1), length=0.008)
    for dx, dy in [(a, b) for a in (-0.012, 0.012) for b in (-0.012, 0.012)]:
        if ledger.load > ps.CAPACITY - 16:
            rinse(ledger)
        stroke(ledger, PISTON_POS + np.array([dx, dy, ps.UNDERCROWN_Z - CORE + 0.001]), FLAT_Q, (1, 0, 0), length=0.006)


def test_grid_size_matches_spec():
    assert ps.N_SKIRT == 100
    assert 38 <= ps.N_CROWN <= 50  # denko 75 mm: ok. 44 cm2
    assert 60 <= ps.N_INNER <= 90
    assert ps.CAPACITY < (ps.N_SKIRT + ps.N_CROWN) * 0.95 / 2  # pełny wynik wymaga co najmniej 2 płukań


def test_full_clean_with_rinses_scores_one():
    led = ps.Ledger()
    wipe_everything(led)
    wipe_rod(led)
    assert led.clean_fraction >= 0.95
    assert led.rinses >= 2
    assert led.score(on_mat=True, resting=True) == 1.0


def test_pressing_without_moving_cleans_nothing():
    led = ps.Ledger()
    for _ in range(40):
        step(led, side_pad_pos(), PAD_SIDE_Q)  # przyłożona i trzymana
    assert led.clean_fraction == 0.0


def test_short_dab_is_not_enough():
    led = ps.Ledger()
    stroke(led, side_pad_pos(), PAD_SIDE_Q, (0, 0, 1), length=0.003, passes=1)  # ok. 1,2 cm drogi
    assert led.clean_fraction == 0.0


def test_dip_without_agitation_does_not_rinse():
    led = ps.Ledger()
    stroke(led, side_pad_pos(), PAD_SIDE_Q, (0, 0, 1), length=0.012)
    loaded = led.load
    assert loaded > 0
    dip = TRAY + np.array([0.0, 0.0, ps.PAD_SIZE[2] / 2 + 0.002])
    for _ in range(30):
        step(led, dip, FLAT_Q)  # zanurzona, bez ruchu
    assert led.load == loaded and led.rinses == 0
    rinse(led)
    assert led.load == 0 and led.rinses == 1


def test_outside_wipe_does_not_clean_inside():
    led = ps.Ledger()
    wipe_everything(led)
    assert led.dirty[ps.N_SKIRT + ps.N_CROWN :].all()  # przez ściankę się nie czyści


def test_inside_not_scored_but_tracked():
    led = ps.Ledger()
    wipe_inside(led)
    assert led.clean_fraction == 0.0
    assert not led.dirty[ps.N_SKIRT + ps.N_CROWN :].all()


def test_rod_scoring_switch():
    led = ps.Ledger()
    wipe_everything(led)
    expected = 0.75 if ps.SCORE_ROD else 1.0  # 145/181 = 80% z korbowodem, 100% bez
    assert led.score(True, True) == expected


def test_still_in_gripper_caps_below_one():
    led = ps.Ledger()
    wipe_everything(led)
    wipe_rod(led)
    assert led.score(on_mat=True, resting=False) == 0.75


def test_never_rinsing_stalls_and_smears():
    led = ps.Ledger()
    wipe_everything(led, rinse_when_loaded=False)
    assert led.rinses == 0
    assert led.clean_fraction < 0.30  # nasycona gąbka rozmazuje z powrotem
    assert led.score(True, True) <= 0.25


def test_hovering_sponge_cleans_nothing():
    led = ps.Ledger()
    hover = PISTON_POS + np.array([ps.PISTON_D / 2 + ps.PAD_SIZE[2] / 2 + 0.002, 0.0, 0.0])
    stroke(led, hover, PAD_SIDE_Q, (0, 0, 1), length=0.012)
    assert led.clean_fraction == 0.0


def test_drop_zeroes_whole_episode():
    led = ps.Ledger()
    step(led, side_pad_pos(), PAD_SIDE_Q, vel=np.array([0, 0, -0.9]))
    wipe_everything(led)
    assert led.zeroed == "upuszczony tłok"
    assert led.score(True, True) == 0.0


def test_gentle_set_down_is_not_a_drop():
    led = ps.Ledger()
    step(led, side_pad_pos(), PAD_SIDE_Q, vel=np.array([0, 0, -0.3]))
    assert led.zeroed is None


def test_dunking_in_tray_zeroes():
    led = ps.Ledger()
    step(led, side_pad_pos(), PAD_SIDE_Q, piston_pos=TRAY + np.array([0, 0, 0.02]))
    assert led.zeroed == "tłok w rozpuszczalniku"


def test_off_mat_zeroes():
    led = ps.Ledger()
    wipe_everything(led)
    assert led.score(on_mat=False, resting=True) == 0.0


def test_crown_scrubbed_on_flat_pad_cleans_crown():
    led = ps.Ledger()
    core_half = ps.FLAT_PAD_SIZE[2] / 2 - ps.SOFT_LAYER
    far_pad = np.array([1.0, 1.0, 1.0])
    flat = PISTON_POS + np.array([0.0, 0.0, ps.PISTON_H / 2 + core_half - 0.001])
    for k in range(41):  # tłok przesuwany po nieruchomej gąbce
        piston = PISTON_POS + np.array([0.02 * math.sin(2 * math.pi * k / 10), 0.0, 0.0])
        step(led, far_pad, FLAT_Q, piston_pos=piston, flat=flat)
    crown = led.dirty[ps.N_SKIRT : ps.N_SKIRT + ps.N_CROWN]
    assert crown.sum() <= 3
    assert led.dirty[: ps.N_SKIRT].all()  # płaska gąbka nie sięga płaszcza


def test_rod_face_wipe():
    led = ps.Ledger()
    rod_pos = np.array([0.5, 0.0, 0.1])
    lo, hi = bbox(PISTON_POS)
    for k in range(21):
        pad = rod_pos + np.array([ps.ROD_COLLIDER[0] / 2 + CORE - 0.001, 0.0, 0.02 * math.sin(2 * math.pi * k / 10)])
        led.step(PISTON_POS, FLAT_Q, STILL, lo, hi, pad, FLAT_Q, TRAY, TRAY_FLOOR, None, None, rod_pos, FLAT_Q)
    cleaned = ~led.rod_dirty
    assert cleaned.sum() >= 4 and np.all(ps.ROD_NORMALS_LOCAL[cleaned][:, 0] > 0.5)  # tylko ściana +x


@pytest.mark.parametrize("frac,expected", [(0.24, 0.0), (0.25, 0.25), (0.5, 0.5), (0.8, 0.75), (0.95, 1.0)])
def test_ladder(frac, expected):
    assert ps.ladder(frac) == expected
