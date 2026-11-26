"""Offline checks of scoring.evaluate (no simulator): ``python test_scoring.py``."""

import numpy as np

import layout as L
import scoring

FILM_B, _, _, BINFO = L.film("bottom")
ZIP_B = np.array([vb for _, vb, _ in L.zip_pairs()])
T = [L.tortilla(k) for k in range(L.N_TORTILLAS)]
FACES = [t[1] for t in T]
PLATE = L.PLATE_AT


def _eval(points, open_u=(), speed=0.0, hands_open=True, hands_empty=True):
    return scoring.evaluate(points, FACES, [speed] * len(points), FILM_B, ZIP_B, list(open_u), PLATE, hands_open,
                            hands_empty)


def _start():
    return [t[0].copy() for t in T]


def _on_plate(p):
    q = p.copy()
    q[:, :2] += PLATE[:2] - q[:, :2].mean(axis=0)
    q[:, 2] = PLATE[2] + L.PLATE_WELL_Z + L.T_TH / 2
    return q


def test_start_scores_zero():
    assert _eval(_start())["score"] == 0.0


def test_zip_open_quarter():
    us = [u for _, _, u in L.zip_pairs()]
    assert _eval(_start(), open_u=us)["score"] == 0.25
    assert _eval(_start(), open_u=us[:10])["score"] == 0.0


def test_half_out():
    p = _start()
    p[-1] = p[-1] - [0.12, 0, 0]
    assert _eval(p)["score"] == 0.5


def test_full_on_plate():
    p = _start()
    p[-1] = _on_plate(p[-1])
    assert _eval(p)["score"] == 1.0
    assert _eval(p, hands_open=False)["score"] == 0.75
    assert _eval(p, speed=0.2)["score"] == 0.75


def test_folded_on_plate_is_not_flat():
    p = _start()
    q = _on_plate(p[-1])
    c = PLATE[0]
    far = q[:, 0] > c
    q[far, 0] = 2 * c - q[far, 0]  # fold the far half back over the near half
    q[far, 2] += 0.004
    p[-1] = q
    r = _eval(p)
    assert r["score"] == 0.75, r


def test_two_out_scores_zero():
    p = _start()
    p[-1] = _on_plate(p[-1])
    p[-2] = p[-2] - [0.15, 0, 0]
    assert _eval(p)["score"] == 0.0


def test_other_poking_out_a_little_is_still_in():
    p = _start()
    p[-1] = _on_plate(p[-1])
    p[-2] = p[-2] - [0.03, 0, 0]
    assert _eval(p)["score"] == 1.0
    p[-2] = p[-2] - [0.06, 0, 0]  # 9 cm out: more than a quarter of it
    assert _eval(p)["score"] == 0.0


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
