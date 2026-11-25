"""Unit tests for the geometry and the scorer (no simulator): python -m pytest test_sleeve.py"""

from __future__ import annotations

import math

import numpy as np

import sleevegeom as G
import sleevescore as S


def _frame(**kw):
    base = dict(t=0.0, squeeze={"left": 0.0, "right": 0.0}, cup_tilt_deg=0.5, cup_base_z=0.0, around=True,
                lows=[0.025] * 16, gaps=[0.0005] * 16, opened=0.07, robot_dist=0.10)
    base.update(kw)
    return S.Frame(**base)


def _ring_on_cup(seat=G.SEAT_H, dx=0.0, tilt_lows=0.0):
    """Strip corners of the sleeve opened round on a cup standing at the origin, its lower edge `seat` above the
    base: the inverse of the flat pack, from the frustum's corner radii."""
    pts = []
    for k in range(G.N_FACETS):
        a0, a1 = 2 * math.pi * k / G.N_FACETS, 2 * math.pi * (k + 1) / G.N_FACETS
        zb, zt = seat, seat + G.SLEEVE_H
        inner = [(G.RB * math.cos(a0) + dx, G.RB * math.sin(a0), zb), (G.RB * math.cos(a1) + dx, G.RB * math.sin(a1), zb),
                 (G.RT * math.cos(a1) + dx, G.RT * math.sin(a1), zt), (G.RT * math.cos(a0) + dx, G.RT * math.sin(a0), zt)]
        inner = np.array(inner)
        pts.append(np.vstack([inner, inner * (1.0 + G.BOARD_T / G.RB) * (1, 1, 0) + inner * (0, 0, 1)]))
    return pts


def test_sleeve_fits_this_cup_where_stated():
    assert abs(G.R_IN_BOTTOM - G.cup_radius(G.SEAT_H)) < 1e-9
    assert 0.0315 < G.R_IN_BOTTOM < 0.0325 and 0.0388 < G.R_IN_TOP < 0.0398  # 64.1 and 78.6 mm across
    assert G.CREASE_SPAN_B > 0.094 and G.CREASE_SPAN_T > 0.094  # wider than one gripper opens, flat


def test_flat_pack_layers_lie_on_each_other():
    f = G.flat_facets((0.25, 0.0, 0.75))
    for k in range(8):
        a, b = f[k].mean(axis=0), f[15 - k].mean(axis=0)
        assert abs(a[1] - b[1]) < 1e-6 and abs(a[2] - b[2]) < 1e-6
        assert 0.0015 < abs(a[0] - b[0]) < 0.0025  # board thickness plus a hair
    assert G.opened_size(f) < 0.005


def test_on_cup_measures_a_seated_sleeve():
    pts = _ring_on_cup()
    oc = G.on_cup(pts, np.zeros(3), np.array((0, 0, 0, 1.0)))
    assert oc["around"]
    assert all(abs(h - G.SEAT_H) < 1e-6 for h in oc["low"])
    assert max(abs(g) for g in oc["gap"]) < 0.0005
    assert G.opened_size(pts) > 0.06


def test_on_cup_off_axis_is_not_snug():
    oc = G.on_cup(_ring_on_cup(dx=0.006), np.zeros(3), np.array((0, 0, 0, 1.0)))
    assert max(oc["gap"]) > S.SNUG_M


def test_snug_allows_a_little_stand_off_near_a_crease():
    sc = S.Scorer()
    sc.update(_frame(gaps=[0.0005] * 13 + [0.003, 0.0035, 0.002]))
    assert sc.score == 1.0


def test_full_score_needs_clear_hands():
    sc = S.Scorer()
    sc.update(_frame(robot_dist=0.01))
    assert sc.score == 0.5 and not sc.success
    sc.update(_frame())
    assert sc.score == 1.0 and sc.success


def test_squeeze_latches_zero():
    sc = S.Scorer()
    sc.update(_frame(squeeze={"left": 0.0, "right": 6.5}))
    sc.update(_frame())
    assert sc.score == 0.0 and not sc.success


def test_tip_latches_zero():
    sc = S.Scorer()
    sc.update(_frame(cup_tilt_deg=35.0))
    sc.update(_frame())
    assert sc.score == 0.0


def test_partial_credit():
    sc = S.Scorer()
    sc.update(_frame(lows=[0.050] * 16))  # round the cup, but too high
    assert sc.score == 0.5
    sc = S.Scorer()
    sc.update(_frame(gaps=[0.003] * 16))  # loose all round
    assert sc.score == 0.5
    sc = S.Scorer()
    sc.update(_frame(gaps=[0.0005] * 15 + [0.005]))  # one strip standing off
    assert sc.score == 0.5
    sc = S.Scorer()
    sc.update(_frame(lows=[0.020] * 8 + [0.035] * 8))  # crooked: 15 mm edge to edge
    assert sc.score == 0.5
    sc = S.Scorer()
    sc.update(_frame(around=False, opened=0.07))
    sc.update(_frame(around=False, opened=0.01))  # opened once, then flattened
    assert sc.score == 0.25
    sc = S.Scorer()
    sc.update(_frame(around=False, opened=0.003))  # never opened
    assert sc.score == 0.0
    sc = S.Scorer()
    sc.update(_frame(cup_base_z=0.03))  # cup still held up
    assert sc.score == 0.25
