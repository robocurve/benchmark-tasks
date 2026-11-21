"""Offline checks of the scoring rule and the tote's geometry (no simulator; needs yam_arena importable):
python -m pytest test_scoring.py"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task as T  # noqa: E402
import totekit as K  # noqa: E402

D = math.radians


@pytest.fixture(autouse=True)
def fresh():
    T._clear()
    yield
    T._clear()


def g(a, b, **kw):
    kw = {"moved": False, "tipped": False, "held": False, **kw}
    return T.grade(D(a), D(b), **kw)


# -- grading: every outcome ------------------------------------------------------------------------------------------
def test_open_lids_score_zero():
    assert g(180, 180) == 0.0


def test_one_lid_seated_other_open_scores_a_quarter():
    assert g(0, 180) == 0.25 and g(180, 0) == 0.25
    assert g(0, 10.1) == 0.25                       # the other lid open 10 degrees or more


def test_one_lid_down_but_not_seated_other_open_scores_zero():
    assert g(5, 180) == 0.0 and g(180, 9.9) == 0.0


def test_one_seated_one_propped_scores_half():
    assert g(0, 1.4) == 0.5 and g(1.4, 0) == 0.5 and g(0, 9.9) == 0.5 and g(0.49, 0.56) == 0.5


def test_both_down_not_seated_scores_a_quarter():
    assert g(5, 5) == 0.25 and g(9.9, 9.9) == 0.25


def test_both_seated_without_the_hold_scores_three_quarters():
    assert g(0, 0) == 0.75 and g(0.49, 0.49) == 0.75 and g(0.5, 0) == 0.75


def test_tiers_are_monotonic_in_the_lids_state():
    """More done never scores less: both seated + hold 1.0 > both seated 0.75 > one seated one propped 0.5 > the quarters > 0."""
    assert 1.0 > g(0, 0) > g(0, 5) > g(0, 180) == g(5, 5) > g(5, 180) == 0.0


def test_full_score_needs_the_hold():
    assert g(0, 0, held=True) == 1.0 and g(0, 0) == 0.75


def test_grippers_not_clear_at_the_end_scores_zero():
    for a, b in ((0, 0), (0, 1.4), (0, 180), (5, 5)):
        assert g(a, b, clear=False) == 0.0


def test_holding_the_lids_down_with_a_gripper_at_timeout_scores_zero():
    """Both lids pressed flat, a gripper still on the tote when the run ends: the hold never completes, grippers not clear."""
    T._STATE.update(clear=False, held_steps=0)
    objs = {T.NAME: type("S", (), {"joints": {"lid_pos": 0.0, "lid_neg": 0.0}})()}
    assert T.score(objs) == 0.0


def test_propped_case_through_score():
    T._STATE.update(clear=True, held_steps=0)
    objs = {T.NAME: type("S", (), {"joints": {"lid_pos": 0.0, "lid_neg": D(1.4)}})()}
    assert T.score(objs) == 0.5


def test_full_score_is_rechecked_at_the_end_not_remembered():
    """The 1.0 needs the last 2 s to be clean: a hold that was reached and then broken (held_steps back to 0) does not keep it."""
    T._STATE.update(clear=True, held_steps=0)
    objs = {T.NAME: type("S", (), {"joints": {"lid_pos": 0.0, "lid_neg": 0.0}})()}
    assert T.score(objs) == 0.75 and not T.success(objs)
    T._STATE.update(clear=False, held_steps=0)
    assert T.score(objs) == 0.0


def test_both_seated_grippers_clear_held_scores_one_through_score():
    T._STATE.update(clear=True, held_steps=T.HOLD_STEPS)
    objs = {T.NAME: type("S", (), {"joints": {"lid_pos": 0.0, "lid_neg": 0.0}})()}
    assert T.score(objs) == 1.0 and T.success(objs)


def test_moved_or_tipped_scores_zero_whatever_the_lids_do():
    assert g(0, 0, held=True, moved=True) == 0.0
    assert g(0, 0, held=True, tipped=True) == 0.0
    assert g(5, 5, moved=True) == 0.0


def test_every_outcome_is_a_listed_score():
    for a in (0, 0.5, 2, 5, 9.9, 10, 40, 180):
        for b in (0, 0.5, 2, 5, 9.9, 10, 40, 180):
            for clear in (True, False):
                assert g(a, b, clear=clear) in (0.0, 0.25, 0.5, 0.75)


def test_seated_means_seam_edge_within_two_mm_of_the_rim_plane_at_the_gauge():
    assert T.SEATED_MM == 2.0
    for side in (+1, -1):
        assert T.seated(side, 0.0) and T.seated(side, D(0.49)) and not T.seated(side, D(0.53)) and not T.seated(side, math.pi)
        assert not T.seated(side, D(1.4))                                      # a lid propped at 1.4 degrees is down but not seated
        assert K.edge_height(side, 0.0) == 0.0
        assert K.edge_height(side, D(1.4)) == pytest.approx(0.0055, abs=0.0002)  # 5.5 mm at the measuring point: the propped state in the model
        assert K.edge_height(side, D(0.5)) == pytest.approx(0.00196, abs=0.00005)
    assert T.SEATED_DEG == pytest.approx(0.51, abs=0.01)                        # 2 mm at 22.45 cm
    assert K.gauge_u(+1) == pytest.approx(K.HINGE_Y + K.SEAM_OFFSET) == pytest.approx(K.gauge_u(-1))
    # each gauge is the middle of a long straight run where that lid is the wider one: R1 (x -9.4 cm) for the +y lid, R2 (x +9.4 cm) for the -y lid
    assert K.GAUGE_X[+1] == pytest.approx((K.SEAM_X[1] + K.SEAM_X[2]) / 2) == pytest.approx(-0.094)
    assert K.GAUGE_X[-1] == pytest.approx((K.SEAM_X[3] + K.SEAM_X[4]) / 2) == pytest.approx(+0.094)
    assert [r[3] for r in K.RUNS] == [-1, +1, -1, +1]
    for side in (+1, -1):                                  # the gauge and a 7.5 cm stretch centred on it (the inclinometer's footprint) lie wholly inside the straight run
        run = next(r for r in K.RUNS if r[0] <= K.GAUGE_X[side] <= r[1])
        assert run[3] == side and run[0] <= K.GAUGE_X[side] - 0.0375 and K.GAUGE_X[side] + 0.0375 <= run[1]


# -- clearance of the grippers ------------------------------------------------------------------------------------------
def test_box_distance():
    o = T.start()
    top = o[2] + K.HEIGHT + K.LID_T + K.BLOCK[2]
    assert T.box_distance(np.array([o[0], o[1], top + 0.05]), o) == pytest.approx(0.05)
    assert T.box_distance(np.array([o[0], o[1] + K.WIDTH / 2 + 0.04, o[2] + 0.1]), o) == pytest.approx(0.04)
    assert T.box_distance(np.array([o[0], o[1], o[2] + 0.1]), o) == 0.0


def test_tilt():
    assert T.tilt((0, 0, 0, 1)) == pytest.approx(0.0)
    s = math.sin(D(10) / 2)
    assert T.tilt((s, 0, 0, math.cos(D(10) / 2))) == pytest.approx(D(10))


# -- the tote ------------------------------------------------------------------------------------------------------------
def test_outside_dimensions_are_the_catalogue_ones():
    assert (K.LENGTH, K.WIDTH, K.HEIGHT) == pytest.approx((21.5 * 0.0254, 15 * 0.0254, 9 * 0.0254), abs=1e-3)


def test_the_zones_tile_each_lid_and_the_seam_has_four_runs_and_three_diagonals():
    for side in (+1, -1):
        zones = sorted(K.lid_zones(side), key=lambda z: z["x0"])
        assert zones[0]["x0"] == pytest.approx(-K.LID_LEN / 2) and zones[-1]["x1"] == pytest.approx(K.LID_LEN / 2)
        for z0, z1 in zip(zones, zones[1:]):
            assert z0["x1"] == pytest.approx(z1["x0"])
        over = [z["run"] for z in zones if z["role"] == "over"]
        assert over == ([1, 3] if side == +1 else [0, 2])           # the +y lid is wider in the low runs R1 and R3, the -y lid in R0 and R2
        assert sum(z["run"] is None for z in zones) == 3 and all(z.get("diag") for z in zones if z["run"] is None)     # three diagonals, one straight piece each
    assert list(K.SEAM_X) == sorted(K.SEAM_X)
    assert len(K.RUNS) == 4 and len(K.DIAGONALS) == 3
    assert K.GAUGE_X[+1] == pytest.approx(-K.GAUGE_X[-1])


def test_each_lid_has_one_full_tooth_and_a_half_tooth_and_the_seam_is_point_symmetric():
    """A lid's teeth are its runs where it is the wider one; a half tooth runs to the lid's end, a full tooth is between diagonals."""
    for side in (+1, -1):
        wide = [r for r in K.RUNS if r[3] == side]
        end = [r for r in wide if r[0] <= -K.LID_LEN / 2 + 1e-9 or r[1] >= K.LID_LEN / 2 - 1e-9]
        full = [r for r in wide if r not in end]
        assert len(wide) == 2 and len(end) == 1 and len(full) == 1
        assert (end[0][1] - end[0][0]) < 0.5 * (full[0][1] - full[0][0])      # the half tooth is the shorter one
    for r, q in zip(K.RUNS, reversed(K.RUNS)):                                # a half turn about the centre maps the seam onto itself
        assert (r[0], r[1]) == pytest.approx((-q[1], -q[0])) and r[2] == pytest.approx(-q[2]) and r[3] == -q[3]
    assert K.SEAM_X == pytest.approx(tuple(-x for x in reversed(K.SEAM_X)))
    assert 0.0 < K.SEAM_X[3] - K.SEAM_X[2] < 0.08                            # the short middle diagonal (about a tenth of the lid)
    for x0, x1, i0, i1 in K.DIAGONALS:
        assert 0.04 < x1 - x0 < 0.08 and abs(K.RUNS[i1][2] - K.RUNS[i0][2]) == pytest.approx(2 * K.SEAM_OFFSET)


def test_the_hang_limit_bounds_the_seam_offset():
    assert K.HINGE_Y + K.SEAM_OFFSET < K.HEIGHT - 0.003
    assert max(r[2] for r in K.RUNS) == pytest.approx(K.SEAM_OFFSET) == pytest.approx(-min(r[2] for r in K.RUNS))
    assert 0.020 <= K.SEAM_OFFSET <= 0.035                                   # the photos' reading, +-1.5 cm (these bounds are tighter)


def test_the_demos_press_points_are_on_the_full_thickness_plate_of_both_lids():
    import demo as D_

    for side, x in D_.X_PRESS.items():
        z = next(z for z in K.lid_zones(side) if z["x0"] <= x <= z["x1"])
        assert D_.U_PRESS + 0.015 < z["u_end"] - (K.LAP_OV + K.LAP_CLEAR if z["role"] == "over" else 0.0)   # the pad (about 1.5 cm) stays on the full plate


def test_the_diagonals_keep_a_constant_2mm_gap_between_the_lids_plain_edges():
    zp, zn = (sorted(K.lid_zones(s), key=lambda z: z["x0"]) for s in (+1, -1))
    for a, b in zip(zp, zn):
        assert a["x0"] == pytest.approx(b["x0"]) and a["x1"] == pytest.approx(b["x1"])
        ends = [(a["u_a"], b["u_a"]), (a["u_b"], b["u_b"])] if a.get("diag") else [(a["u_end"], b["u_end"])]
        for ua, ub in ends:
            assert (2 * K.HINGE_Y - ua - ub) == pytest.approx(K.SEAM_GAP, abs=1e-9)     # runs and diagonals alike: flush tops, 2 mm seam


# the traced seam polyline (README table, whose 26.9 cm end points are the lid's half length 26.905 cm; x, y in m from the tote's centre, y positive to the left)
POLYLINE = [(-K.LID_LEN / 2, 0.03), (-0.215, 0.03), (-0.158, -0.03), (-0.03, -0.03), (0.03, 0.03), (0.158, 0.03), (0.215, -0.03), (K.LID_LEN / 2, -0.03)]


def _poly_y(x):
    return float(np.interp(x, [p[0] for p in POLYLINE], [p[1] for p in POLYLINE]))


def test_every_lid_outline_vertex_lies_on_the_traced_polyline_and_the_diagonals_are_straight():
    gap = K.SEAM_GAP
    for side in (+1, -1):
        out = K.lid_outline(side)
        assert len(out) == 14                                    # 4 runs + 3 diagonals, two vertices each
        for x, y in out:
            assert any(abs(x - px) < 1e-9 for px, _ in POLYLINE)             # every vertex is at a polyline breakpoint in x
            # in y it is on the polyline (the wider lid's edge in a run, either end of a diagonal) or one 2 mm gap beside it
            assert abs(y - _poly_y(x)) < 1e-9 or abs(abs(y - _poly_y(x)) - gap) < 1e-9
        for (x0, y0), (x1, y1) in zip(out[0::2], out[1::2]):                # each piece's edge, from its first vertex to its second
            if abs(y1 - y0) < 1e-9:
                continue                                                    # a straight run
            k = (_poly_y(x1) - _poly_y(x0)) / (x1 - x0)                      # the polyline's slope on this diagonal
            assert (y1 - y0) / (x1 - x0) == pytest.approx(k, rel=0.04)       # the edge is straight and parallel to the polyline
            assert abs(math.degrees(math.atan(k))) == pytest.approx(45, abs=2.5)
            for t in np.linspace(0, 1, 21):                                  # every point of the edge is within one gap (vertically) of the line
                x = x0 + t * (x1 - x0)
                assert abs((y0 + t * (y1 - y0)) - _poly_y(x)) <= gap + 1e-9
                assert abs((y0 + t * (y1 - y0)) - _poly_y(x)) * math.cos(math.atan(abs(k))) <= 1.5e-3      # at most 1.5 mm perpendicular
    assert K.diag_prisms(+1).keys() == K.diag_prisms(-1).keys() and len(K.diag_prisms(+1)) == 3


def _plan_pieces(side):
    """Plan-view convex polygons (x, y) of the lid's top-layer plate pieces: plate and overhang boxes, and the diagonal trapezoids."""
    prisms, out = K.diag_prisms(side), []
    for tag, u0, u1, w0, w1, x0, x1 in K.lid_parts(side):
        if tag in prisms:
            v = prisms[tag][:4]
            out.append(np.array([(x, side * (K.HINGE_Y - u)) for x, u, _ in v]))
        elif tag.split("_")[0] in ("edge", "over"):
            out.append(np.array([(x, side * (K.HINGE_Y - u)) for x, u in ((x0, u0), (x1, u0), (x1, u1), (x0, u1))]))
    return out


def _plan_gap(p, q):
    """Separation of two convex polygons along the best edge normal (negative if they overlap)."""
    best = -1e9
    for poly_ in (p, q):
        for i in range(len(poly_)):
            e = poly_[(i + 1) % len(poly_)] - poly_[i]
            n = np.array([-e[1], e[0]])
            if np.linalg.norm(n) < 1e-12:
                continue
            n /= np.linalg.norm(n)
            a, b = p @ n, q @ n
            best = max(best, max(b.min() - a.max(), a.min() - b.max()))
    return best


def test_the_lids_do_not_overlap_and_keep_the_2mm_gap_along_the_diagonals():
    prisms = K.diag_prisms(+1)
    pa, pb = _plan_pieces(+1), _plan_pieces(-1)
    worst = 1e9
    for a in pa:
        for b in pb:
            if max(a[:, 0].min(), b[:, 0].min()) < min(a[:, 0].max(), b[:, 0].max()) - 1e-9:      # pieces that share some x range
                worst = min(worst, _plan_gap(a, b))
    assert worst >= K.SEAM_GAP * math.cos(math.radians(48)) - 1e-9     # never overlapping; across a diagonal's edge the 2 mm (y) gap is at least 1.3 mm (1.35 mm at the steepest, 47.4 degrees)
    # the diagonals: the vertical (y) gap is 2 mm at every x, so the gap measured across the edge is 2 mm x cos(slope) (about 1.4 mm at 45 degrees)
    for side_pos, side_neg in zip(sorted(K.lid_zones(+1), key=lambda z: z["x0"]), sorted(K.lid_zones(-1), key=lambda z: z["x0"])):
        if not side_pos.get("diag"):
            continue
        for x in np.linspace(side_pos["x0"], side_pos["x1"], 11):
            yp = K.HINGE_Y - K.diag_edge_u(side_pos, x)
            yn = -(K.HINGE_Y - K.diag_edge_u(side_neg, x))
            assert yp - yn == pytest.approx(K.SEAM_GAP, abs=1e-9)
    assert len(prisms) == 3
    # the prisms are convex, 4 mm thick, flush with the plate top
    for side in (+1, -1):
        for v in K.diag_prisms(side).values():
            assert len(v) == 8 and {w for _, _, w in v} == {0.0, K.LID_T}


def test_the_tops_are_flush_and_the_overhang_covers_the_other_lids_tongue_in_each_of_the_four_straight_runs():
    def pick(side, x):
        return {p[0].split("_")[0]: p for p in K.lid_parts(side) if p[0].split("_")[0] in ("edge", "over", "tongue") and p[5] <= x <= p[6]}

    for zone_x, wide in ((-0.24, -1), (-0.094, +1), (0.094, -1), (0.24, +1)):
        yc = K.centre_y(zone_x)
        pw, pn = pick(wide, zone_x), pick(-wide, zone_x)
        assert set(pw) == {"edge", "over"} and set(pn) == {"edge", "tongue"}
        top_w = wide * (K.HINGE_Y - pw["over"][2])                  # the wider lid's top edge (world y): exactly the seam line
        top_n = -wide * (K.HINGE_Y - pn["edge"][2])                 # the other lid's top edge
        tip_n = -wide * (K.HINGE_Y - pn["tongue"][2])               # the tongue's tip
        assert top_w == pytest.approx(yc)
        assert abs(top_w - top_n) == pytest.approx(K.SEAM_GAP)       # the seam seen from above
        assert pw["over"][4] == pn["edge"][4] == K.LID_T             # both top faces at the same height: flush
        assert (tip_n - top_w) * wide == pytest.approx(K.LAP_OV)     # the tongue's tip is LAP_OV past the seam line, under the overhang
        assert pn["tongue"][4] < pw["over"][3]                       # tongue top below the overhang's underside: no contact when shut
        stop_w = wide * (K.HINGE_Y - pw["edge"][2])                  # the wider lid's full plate stops short of the tongue's tip
        assert (stop_w - tip_n) * wide == pytest.approx(K.LAP_CLEAR)


def test_the_overlap_is_under_the_flush_top():
    tops = {p[4] for side in (+1, -1) for p in K.lid_parts(side) if p[0].split("_")[0] in ("edge", "over")}
    assert tops == {K.LID_T}                                      # nothing stands proud of the plate near the seam: tops are flush
    assert K.OVER_W0 - K.TONGUE_T >= 0.0005                       # a clear gap between tongue top and overhang underside


def test_flush_lap_stops_a_second_lid_just_above_flat_and_lets_two_lids_down_together():
    """2D cross-section of the lids in both kinds of straight run of the S-shaped seam (lip2d.py; see README)."""
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    import lip2d

    lip2d.D, lip2d.G, lip2d.OV, lip2d.CL, lip2d.W0, lip2d.TT = K.SEAM_OFFSET, K.SEAM_GAP, K.LAP_OV, K.LAP_CLEAR, K.OVER_W0, K.TONGUE_T
    assert not lip2d.collide(0, 0)
    jam = lip2d.jam_angle()
    assert 1.0 < jam < 2.0                                  # the second lid rests propped open by this much: the tongue lands on the overhang's top
    assert jam == pytest.approx(math.degrees(math.asin(K.LID_T / (K.HINGE_Y + K.SEAM_OFFSET - K.SEAM_GAP))), abs=0.3)   # about plate thickness / lever arm
    assert all(not lip2d.collide(ta, ta) for ta in np.arange(40, -0.01, -0.2))        # the same angle: always free


def test_closed_lid_edge_sits_at_the_rim():
    for side in (+1, -1):
        z = max(K.lid_zones(side), key=lambda z: z["u_end"])
        p = K.lid_point(side, 0.0, z["u_end"])
        assert p[1] == pytest.approx(side * (K.HINGE_Y - z["u_end"])) and p[2] == pytest.approx(K.HEIGHT)


def test_open_flat_lid_points_outward_and_the_face_normal_points_down():
    p = K.lid_point(+1, math.pi, 0.2, offset=0.01)
    assert p[1] == pytest.approx(K.HINGE_Y + 0.2) and p[2] == pytest.approx(K.HEIGHT - 0.01)


def test_start_layout_is_fixed_and_reachable():
    assert T.OBJECTS[0].pos == (T.TOTE_XY[0], 0.0, T.TOP)
    # nearest lid end is clear of the arms' wrists; far end inside the table
    assert T.TOTE_XY[0] - K.LENGTH / 2 > 0.24 and T.TOTE_XY[0] + K.LENGTH / 2 < 0.9


# -- product-photo details (read from product photos, see README) -------------------------------------------------------------------
def _boxes(parts, collide=True):
    return [p for p in parts if p[4] == collide]


def test_tub_tapers_in_and_the_rim_band_is_the_catalogue_outline():
    assert K.A_BASE > K.A_TOP > 0
    assert K.inset(0.0) == pytest.approx(K.A_BASE) and K.inset(K.ZT) == pytest.approx(K.A_TOP)
    assert 2 * K.A_BASE < 0.06                                       # base footprint 2.5 cm in on each side
    assert 3.0 < math.degrees(K.PHI) < 8.0
    parts = {p[0]: p for p in K.tub_parts()}
    rim = parts["rim_pos"]
    assert rim[1][2] + rim[2][2] / 2 == pytest.approx(K.HEIGHT)      # the rim top is the lids' rest plane, as before
    assert rim[1][1] + rim[2][1] / 2 == pytest.approx(K.WIDTH / 2)   # band outline = the catalogue outside width
    assert 0.03 <= K.RIM_H <= 0.04


def test_tub_collision_parts_stay_inside_the_outline_and_start_at_the_table():
    lo = hi = 0.0
    for name, c, size, q, collide in K.tub_parts():
        if not collide:
            continue
        assert abs(c[0]) + size[0] / 2 <= K.LENGTH / 2 + 1e-6, name
        assert abs(c[1]) + size[1] / 2 <= K.WIDTH / 2 + 1e-6 or name.startswith("wall_end"), name
        lo, hi = min(lo, c[2] - size[2] / 2 * math.cos(K.PHI)), max(hi, c[2] + size[2] / 2)
    assert lo > -1e-3 and hi <= K.HEIGHT + 1e-6


def test_slots_are_real_gaps_and_ribs_are_visual_only():
    parts = K.tub_parts()
    assert all(not p[4] for p in parts if p[0].startswith("rib_"))
    assert sum(p[0].startswith("rib_long") for p in parts) == 2 * 6          # 5 panels per long wall -> 6 ribs
    assert sum(p[0].startswith("rib_end") for p in parts) == 2 * 1           # 2 panels per end wall -> 1 inner rib (corners shared)
    for tag in ("wall_long_p", "wall_end_p"):                                # the slot band is split into segments around the slots
        assert sum(p[0].startswith(tag + "_mid") for p in parts) >= 3 or tag == "wall_end_p"
    assert K.ZT - K.SLOT_GAP - K.SLOT_H > 0.1


def test_lid_additions_do_not_change_the_mechanism_numbers():
    assert K.HINGE_Y == pytest.approx(K.WIDTH / 2 + K.HINGE_OUT)
    assert K.LUG_U0 + K.LUG[1] < 0.09 and K.LUG_X0 + K.LUG[0] <= K.LID_LEN / 2          # lugs end 2 cm short of the pinch line (u 0.10, x -0.259); they start past the rim band (hanging clearance)
    assert K.LUG[2] <= K.BLOCK[2]                                                        # no taller than the seal blocks
    assert K.HOLE < min(K.BLOCK[0], K.BLOCK[1]) / 2


# -- hinge range, supports (the lids start flat on fixed supports; real lids hang about 270 degrees) -------------------------
def _rot(q):
    return K._rotation((q[1], q[2], q[3], q[0]))


def _sep(c1, R1, h1, c2, R2, h2):
    """Separating-axis gap between two oriented boxes (m): positive when apart, negative when they overlap."""
    axes = [R1[:, i] for i in range(3)] + [R2[:, i] for i in range(3)]
    axes += [np.cross(R1[:, i], R2[:, j]) for i in range(3) for j in range(3)]
    best = -1e9
    for a in axes:
        n = np.linalg.norm(a)
        if n < 1e-9:
            continue
        a = a / n
        r1 = sum(h1[i] * abs(a @ R1[:, i]) for i in range(3))
        r2 = sum(h2[i] * abs(a @ R2[:, i]) for i in range(3))
        best = max(best, abs((c2 - c1) @ a) - r1 - r2)
    return best


def _tub_boxes():
    return [(np.array(c), np.eye(3) if q is None else _rot(q), np.array(size) / 2) for _, c, size, q, collide in K.tub_parts() if collide]


def test_hinge_range_is_about_270_degrees_and_the_trial_starts_flat_at_180():
    assert 255 <= math.degrees(K.THETA_STOP) <= 275 and K.THETA_START == pytest.approx(math.pi)
    assert K.THETA_START < K.THETA_STOP                      # the start is inside the hinge range, so the supports (not a stop) hold the lids


def test_a_lid_hanging_at_the_stop_clears_the_tub_and_the_table():
    tub = _tub_boxes()
    for side in (+1, -1):
        lowest = 1e9
        for part in K.lid_parts(side):
            c, R, h = K.lid_box_world(side, K.THETA_STOP, part)
            lowest = min(lowest, c[2] - sum(h[i] * abs(R[2, i]) for i in range(3)))
            for tc, tR, th in tub:
                assert _sep(c, R, h, tc, tR, th) > -0.0005, (side, part[0])      # touches at most (the plate's hinge corner on the rim band)
        assert lowest > 0.002                                   # the hanging lid stays above the table
    assert K.max_hang_reach() < K.HEIGHT - 0.003 and K.max_hang_reach() == pytest.approx(K.HINGE_Y + K.SEAM_OFFSET)


def test_the_hanging_lid_hangs_almost_straight_down():
    c, R, h = K.lid_box_world(+1, K.THETA_STOP, ("plate", 0.0, 0.2, 0.0, K.LID_T, -0.2, 0.2))
    assert c[2] < K.HEIGHT - 0.09 and 0.0 < c[1] - K.HINGE_Y < 0.02 + 0.2 * math.sin(math.radians(10))     # below the hinge, leaning out slightly
    assert abs(math.degrees(K.THETA_STOP) - 270) <= 10


def test_supports_are_fixed_flat_topped_blocks_at_the_plate_underside_height():
    sup = [o for o in T.OBJECTS if o.name.startswith("support_")]
    assert len(sup) == 2 and all(o.mass is None for o in sup)                  # no mass: fixed to the table, never part of the tote
    for o in sup:
        assert o.pos[2] + o.size[2] / 2 == pytest.approx(T.TOP + K.HEIGHT - K.LID_T)       # top = the plate's underside lying flat
        assert o.pos[2] - o.size[2] / 2 == pytest.approx(T.TOP)                            # standing on the table
    assert K.SUPPORT_TOP == pytest.approx(K.HEIGHT - K.LID_T)


def test_the_lids_near_end_overhangs_each_support_by_10_cm():
    for side, x0, x1, y0, y1, top in K.support_specs():
        assert x0 - (-K.LID_LEN / 2) == pytest.approx(0.10) == pytest.approx(K.SUPPORT_OVERHANG)
        # the pinch (x -0.259, 1 cm in from the end, u 0.10 plus the pad's width) is on the free overhang and not over the support
        assert x0 > -0.259 + 0.06
        assert min(abs(y0), abs(y1)) > K.HINGE_Y and max(abs(y0), abs(y1)) < K.HINGE_Y + K.SUPPORT_U1 + 1e-9


def test_a_lid_lies_flat_on_its_support_without_its_raised_features_touching_it():
    for side, x0, x1, y0, y1, top in K.support_specs():
        sc, sh = np.array([(x0 + x1) / 2, (y0 + y1) / 2, top / 2]), np.array([x1 - x0, y1 - y0, top]) / 2
        for part in K.lid_parts(side):
            c, R, h = K.lid_box_world(side, K.THETA_START, part)
            s = _sep(c, R, h, sc, np.eye(3), sh)
            if part[0].split("_")[0] in ("edge", "over"):
                assert s > -1e-9                                 # the flat top surface lies on the support top, no overlap
            elif part[0].startswith("tongue"):
                assert s > -1e-9                                 # the thin tongue is on the lid's other face, up in the air
            else:
                assert s > 0.005, part[0]                        # lugs, seal blocks and bosses hang below the plate: 5 mm clear
        lowest = min(K.lid_box_world(side, K.THETA_START, p)[0][2] - K.lid_box_world(side, K.THETA_START, p)[2][2] for p in K.lid_parts(side)
                     if p[0].split("_")[0] in ("edge", "over"))
        assert lowest == pytest.approx(top)                      # the flat top surface level with the support top: the lid lies at 180 degrees


def test_supports_avoid_the_bosses_and_the_logo_plates_and_the_lid_lies_flat():
    """The bosses are collision boxes, 3 mm proud of the top face: the supports start beyond them and beyond the embossed logo plates."""
    assert K.BOSS_H == pytest.approx(0.003)
    for side, x0, x1, y0, y1, top in K.support_specs():
        bosses = [p for p in K.lid_parts(side) if p[0].startswith("boss")]
        assert len(bosses) == 2
        u_support = min(abs(y0), abs(y1)) - K.HINGE_Y
        for b in bosses:
            assert b[4] - b[3] == pytest.approx(0.003)
            assert u_support - b[2] >= 0.020                        # 2 cm or more from the bosses
        assert u_support >= 0.115                                   # the logo plates (u up to about 11.5 cm, photo 5) are not carried
        assert K.SUPPORT_X1 < K.LID_LEN / 2 - 0.040 and K.SUPPORT_X1 < 0.239 - 0.020       # 2 cm or more short of the far seal block
        for p in K.lid_parts(side):                                 # every raised feature stands clear of the support footprint
            if p[0].split("_")[0] in ("lug", "block", "boss"):
                ux = (x0 <= p[6] and p[5] <= x1) and (u_support <= p[2] and p[1] <= u_support + K.SUPPORT_U1 - K.SUPPORT_U0)
                assert not ux, p[0]
        # the lid's weight acts at u of about 10 cm: the support (u from 12 cm) is beyond it, so the hinge and the support hold the lid level
        assert u_support > 0.10


def test_supports_clear_the_tote_the_arms_and_the_lids_swing():
    tub = _tub_boxes()
    for side, x0, x1, y0, y1, top in K.support_specs():
        sc, sh = np.array([(x0 + x1) / 2, (y0 + y1) / 2, top / 2]), np.array([x1 - x0, y1 - y0, top]) / 2
        assert all(_sep(sc, np.eye(3), sh, tc, tR, th) > 0.04 for tc, tR, th in tub)             # 4 cm or more from the tote
        # in world coordinates the arms stand at x 0 and the grippers work at x <= tote x - 0.159 (pinch plus the pad): supports start beyond
        assert T.TOTE_XY[0] + x0 > 0.30
        # a lid lifted from its support rotates about an axis at the rim plane height, so its plate never goes below the support top
        for th in np.radians(np.arange(90, 180, 5)):
            for part in K.lid_parts(side):
                if part[0].split("_")[0] in ("edge", "over"):
                    c, R, h = K.lid_box_world(side, th, part)
                    assert c[2] - sum(h[i] * abs(R[2, i]) for i in range(3)) >= top - 1e-9


def test_supports_do_not_enter_the_tote_scoring():
    assert T.NAME == "tote" and all(o.name != T.NAME for o in T.OBJECTS[1:])
    import inspect

    assert "support" not in inspect.getsource(T._watch)        # the moved/tilted rule reads only the tote


def test_lip2d_constants_match_the_kit():
    import lip2d

    assert (lip2d.H, lip2d.WIDTH, lip2d.HY, lip2d.T) == pytest.approx((K.HEIGHT, K.WIDTH, K.HINGE_Y, K.LID_T))
    assert (lip2d.G, lip2d.OV, lip2d.CL, lip2d.W0, lip2d.TT, lip2d.D) == pytest.approx(
        (K.SEAM_GAP, K.LAP_OV, K.LAP_CLEAR, K.OVER_W0, K.TONGUE_T, K.SEAM_OFFSET))


def test_demo_starts_from_the_flat_pose():
    import demo as D_

    assert D_.THETA0 == pytest.approx(K.THETA_START)
