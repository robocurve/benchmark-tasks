"""Consistency tests for geometry.py (stdlib unittest, numpy only): does the box geometry do what its comments say?

All boxes are axis-aligned and the plug and the block are both at yaw 0, so everything is checked with
axis-aligned box overlap maths in the block frame. The plunger is a cylinder; it is approximated by its bounding
box and checked separately from the rigid parts.

Run:  python -m unittest discover -s <this folder> -p 'test_geometry.py' -v
Derived clearances are printed to stderr by ``TestClearances.test_clearance_report``.
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import geometry as G  # noqa: E402
import scoring as S  # noqa: E402

VOL_TOL = 1e-9  # m^3 (1 mm^3): overlaps up to this are float noise or flush contact, not interpenetration
EPS = 1e-12

BLOCK_NAMES = ["block", "flange", "left wall", "right wall", "bottom wall", "top wall (left of slot)",
               "top wall (right of slot)", "key-slot cap"]
PLUG_NAMES = ["housing", "nose root", "nose groove piece", "nose tip", "key rib"]
I_BLOCK, I_FLANGE, I_LEFT, I_RIGHT, I_BOTTOM, I_TOP_L, I_TOP_R, I_CAP = range(8)
I_HOUSING, I_ROOT, I_GROOVE, I_TIP, I_KEY = range(5)

SEATED = np.array(G.seated_plug_origin_in_block_frame(), dtype=float)

# exact (integer) transforms about the plug origin, applied to the plug's box corners
MIRROR_Z = np.diag([1.0, 1.0, -1.0])  # "key down" as the task brief describes it
ROLL_180 = np.diag([-1.0, 1.0, -1.0])  # a true half turn about the nose axis (y)
ROLL_90 = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
YAW_90 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
YAW_180 = np.diag([-1.0, -1.0, 1.0])


def report(text):
    print(text, file=sys.stderr)


def aabb(box):
    """``(lo, hi)`` corners of a ``(centre, size, colour)`` box."""
    centre, size, _ = box
    c, h = np.asarray(centre, dtype=float), np.asarray(size, dtype=float) / 2.0
    return c - h, c + h


def block_aabbs():
    return [aabb(b) for b in G.block_boxes()]


def plug_aabbs(origin=SEATED, matrix=None):
    """Axis-aligned hulls of the plug's boxes with the plug origin at ``origin`` (block frame); ``matrix``
    (3x3, applied about the plug origin) transforms the box corners first."""
    out = []
    for box in G.plug_boxes():
        lo, hi = aabb(box)
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        if matrix is not None:
            corners = corners @ np.asarray(matrix, dtype=float).T
        corners = corners + np.asarray(origin, dtype=float)
        out.append((corners.min(axis=0), corners.max(axis=0)))
    return out


def overlap_volume(a, b):
    d = np.minimum(a[1], b[1]) - np.maximum(a[0], b[0])
    return float(np.prod(np.maximum(d, 0.0)))


def worst_overlap(plug, block):
    """``(volume, plug index, block index)`` of the most interpenetrating pair."""
    best = (0.0, -1, -1)
    for i, p in enumerate(plug):
        for j, b in enumerate(block):
            v = overlap_volume(p, b)
            if v > best[0]:
                best = (v, i, j)
    return best


def describe(worst):
    v, i, j = worst
    if i < 0:
        return "no overlap"
    return f"{PLUG_NAMES[i]} / {BLOCK_NAMES[j]}: {v * 1e9:.3f} mm^3"


class _OverlapAssertions:
    def assertClear(self, plug, msg=""):
        worst = worst_overlap(plug, block_aabbs())
        self.assertLessEqual(worst[0], VOL_TOL, f"{msg} plug interpenetrates the block ({describe(worst)})")

    def assertRejected(self, plug, msg=""):
        worst = worst_overlap(plug, block_aabbs())
        self.assertGreater(worst[0], VOL_TOL, f"{msg} geometry should reject this plug pose ({describe(worst)})")
        return worst


# -- the seated plug fits ------------------------------------------------------------------------------------------
class TestSeatedFit(_OverlapAssertions, unittest.TestCase):
    def test_seated_origin_is_the_documented_one(self):
        self.assertEqual(tuple(SEATED), (0.0, G.MOUTH_Y, 0.0))

    def test_seated_plug_overlaps_no_block_box(self):
        self.assertClear(plug_aabbs(), "seated:")
        for i, p in enumerate(plug_aabbs()):
            for j, b in enumerate(block_aabbs()):
                with self.subTest(plug=PLUG_NAMES[i], block=BLOCK_NAMES[j]):
                    self.assertLessEqual(overlap_volume(p, b), VOL_TOL)

    def test_housing_sits_flush_on_the_socket_mouth(self):
        housing = plug_aabbs()[I_HOUSING]
        shell = block_aabbs()[I_LEFT]
        self.assertAlmostEqual(shell[0][1], G.MOUTH_Y, delta=EPS)  # shell front face
        self.assertAlmostEqual(housing[1][1], shell[0][1], delta=EPS)  # housing back face touches it, no gap
        # and it really bears on the shell: the housing is wider than the shell, and its faces meet it in x and z
        self.assertGreater(housing[1][0] - housing[0][0], 2 * G.SHELL_HX)
        self.assertGreaterEqual(housing[0][2], -G.SHELL_HZ)
        self.assertLessEqual(housing[1][2], G.SHELL_HZ)

    def test_nose_tip_stops_short_of_the_cavity_floor(self):
        tip_y = plug_aabbs()[I_TIP][1][1]
        gap = G.FLOOR_Y - tip_y
        self.assertAlmostEqual(gap, G.SHELL_L - G.NOSE_L, delta=EPS)
        self.assertGreater(gap, 0.0)
        report(f"[geometry] seated nose tip is {gap * 1000:.2f} mm short of the cavity floor")

    def test_seated_depth_is_the_nose_length_and_scoring_depth_is_reachable(self):
        self.assertAlmostEqual(S.insertion_depth(SEATED), G.NOSE_L, delta=EPS)
        self.assertLess(S.SEATED_DEPTH, G.NOSE_L)

    def test_block_parts_are_stacked_without_overlap(self):
        boxes = block_aabbs()
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                with self.subTest(a=BLOCK_NAMES[i], b=BLOCK_NAMES[j]):
                    self.assertLessEqual(overlap_volume(boxes[i], boxes[j]), VOL_TOL)
        # block -> flange -> shell are face to face along y
        self.assertAlmostEqual(boxes[I_BLOCK][0][1], -G.BLOCK_SIZE[1] / 2, delta=EPS)
        self.assertAlmostEqual(boxes[I_FLANGE][1][1], G.FACE_Y, delta=EPS)
        self.assertAlmostEqual(boxes[I_FLANGE][0][1], G.FLOOR_Y, delta=EPS)
        self.assertAlmostEqual(boxes[I_LEFT][1][1], G.FLOOR_Y, delta=EPS)

    def test_shell_boxes_tile_the_wall_exactly(self):
        # shell outer box minus cavity minus key slot should be exactly what the five wall boxes add up to
        boxes = block_aabbs()
        walls = sum(float(np.prod(boxes[i][1] - boxes[i][0])) for i in range(I_LEFT, I_CAP + 1))
        outer = 2 * G.SHELL_HX * 2 * G.SHELL_HZ * G.SHELL_L
        cavity = 2 * G.CAV_HX * 2 * G.CAV_HZ * G.SHELL_L
        slot = (G.SLOT_X1 - G.SLOT_X0) * (G.SLOT_Z1 - G.CAV_HZ) * G.SHELL_L
        self.assertAlmostEqual(walls, outer - cavity - slot, delta=1e-15)

    def test_socket_axis_height_and_start_pose(self):
        self.assertAlmostEqual(G.BLOCK_POS[2], G.SEAT_Z, delta=EPS)  # socket axis is the block centre line
        self.assertAlmostEqual(G.PLUG_Z0 - G.HOUSING[2] / 2, G.TOP, delta=EPS)  # lies on its housing
        self.assertGreater(G.PLUG_Z0 - G.NOSE_H / 2, G.TOP)  # the nose does not touch the table
        self.assertLess(G.SEAT_Z - G.PLUG_Z0, S.LIFT_HEIGHT)


# -- clearances ----------------------------------------------------------------------------------------------------
class TestClearances(unittest.TestCase):
    def gaps(self):
        blk, plg = block_aabbs(), plug_aabbs()
        nose, key = plg[I_ROOT], plg[I_KEY]
        return {
            "nose -x side": nose[0][0] - blk[I_LEFT][1][0],
            "nose +x side": blk[I_RIGHT][0][0] - nose[1][0],
            "nose -z side (bottom wall)": nose[0][2] - blk[I_BOTTOM][1][2],
            "nose +z side (top wall, left of slot)": blk[I_TOP_L][0][2] - nose[1][2],
            "nose +z side (top wall, right of slot)": blk[I_TOP_R][0][2] - nose[1][2],
            "key -x side (slot edge)": key[0][0] - blk[I_TOP_L][1][0],
            "key +x side (slot edge)": blk[I_TOP_R][0][0] - key[1][0],
            "key +z (slot cap)": blk[I_CAP][0][2] - key[1][2],
        }

    def test_nose_clears_the_cavity_by_CLEAR_on_each_side(self):
        for name, gap in self.gaps().items():
            if name.startswith("nose"):
                with self.subTest(name):
                    self.assertAlmostEqual(gap, G.CLEAR, delta=EPS)

    def test_key_clears_the_slot_by_CLEAR_on_each_side(self):
        for name, gap in self.gaps().items():
            if name.startswith("key"):
                with self.subTest(name):
                    self.assertAlmostEqual(gap, G.CLEAR, delta=EPS)

    def test_all_three_nose_pieces_are_the_same_height_and_the_tip_is_full_width(self):
        plg = plug_aabbs()
        for i in (I_ROOT, I_GROOVE, I_TIP):
            self.assertAlmostEqual(plg[i][0][2], -G.NOSE_H / 2, delta=EPS)
            self.assertAlmostEqual(plg[i][1][2], G.NOSE_H / 2, delta=EPS)
        self.assertAlmostEqual(plg[I_ROOT][1][1], plg[I_GROOVE][0][1], delta=EPS)  # pieces abut along the nose
        self.assertAlmostEqual(plg[I_GROOVE][1][1], plg[I_TIP][0][1], delta=EPS)
        self.assertAlmostEqual(plg[I_TIP][1][1], SEATED[1] + G.NOSE_L, delta=EPS)
        self.assertAlmostEqual(plg[I_TIP][1][0] - plg[I_TIP][0][0], G.NOSE_W, delta=EPS)

    def test_key_sits_on_the_top_of_the_nose(self):
        plg = plug_aabbs()
        self.assertAlmostEqual(plg[I_KEY][0][2], plg[I_ROOT][1][2], delta=EPS)  # rib base on the nose top face
        self.assertAlmostEqual(plg[I_KEY][1][2] - plg[I_KEY][0][2], G.KEY_H, delta=EPS)
        self.assertAlmostEqual(plg[I_KEY][1][0] - plg[I_KEY][0][0], G.KEY_W, delta=EPS)
        self.assertGreater(plg[I_KEY][0][0], plg[I_ROOT][0][0])  # rib lies within the nose width
        self.assertLess(plg[I_KEY][1][0], plg[I_ROOT][1][0])

    def play_in_each_direction(self):
        """How far the plug can be shifted from the seated pose before any block box is touched, per direction."""
        out = {}
        for name, d in [("+x", (1, 0, 0)), ("-x", (-1, 0, 0)), ("+z", (0, 0, 1)), ("-z", (0, 0, -1))]:
            lo, hi = 0.0, 0.002
            for _ in range(60):
                mid = (lo + hi) / 2
                if worst_overlap(plug_aabbs(SEATED + mid * np.array(d, dtype=float)), block_aabbs())[0] > 0.0:
                    hi = mid
                else:
                    lo = mid
            out[name] = lo
        return out

    def max_mechanical_tilt_deg(self, axis):
        """Largest rotation about the plug-frame ``axis`` (0 = x pitch, 1 = y roll, 2 = z yaw), about the plug
        origin, at which the sampled nose volume still fits inside the free cavity (key rib ignored)."""
        hx, hz = G.NOSE_W / 2, G.NOSE_H / 2
        xs = np.linspace(-hx, hx, 29)
        ys = np.linspace(0.0, G.NOSE_L, 19)
        zs = np.linspace(-hz, hz, 17)
        pts = np.array([[x, y, z] for x in xs for y in ys for z in zs])

        def fits(theta):
            c, s = math.cos(theta), math.sin(theta)
            R = {0: np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
                 1: np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
                 2: np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]
            q = pts @ R.T
            inside = q[:, 1] >= 0.0  # the part still outside the mouth is unconstrained
            q = q[inside]
            return bool(np.all(np.abs(q[:, 0]) <= G.CAV_HX + 1e-12) and np.all(np.abs(q[:, 2]) <= G.CAV_HZ + 1e-12))

        lo, hi = 0.0, math.radians(30.0)
        for _ in range(50):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if fits(mid) else (lo, mid)
        return math.degrees(lo)

    def test_clearance_report(self):
        lines = ["[geometry] derived clearances (mm):"]
        for name, gap in self.gaps().items():
            lines.append(f"    {name:42s} {gap * 1000:6.3f}  (CLEAR = {G.CLEAR * 1000:.3f})")
        play = self.play_in_each_direction()
        lines.append("    free shift of the seated plug before touching: " +
                     ", ".join(f"{k} {v * 1000:.3f}" for k, v in play.items()))
        tilts = {name: self.max_mechanical_tilt_deg(a) for name, a in (("pitch (x)", 0), ("roll (y)", 1), ("yaw (z)", 2))}
        lines.append("    max rotation about the plug origin that still fits (deg): " +
                     ", ".join(f"{k} {v:.2f}" for k, v in tilts.items()) +
                     f"   [scorer: seated needs <= {S.SEATED_ANG_DEG} deg, lateral <= {S.SEATED_LATERAL * 1000:.1f} mm]")
        report("\n".join(lines))
        for direction, shift in play.items():
            with self.subTest(direction=direction):
                self.assertAlmostEqual(shift, G.CLEAR, delta=1e-9)
        for name, deg in tilts.items():
            with self.subTest(tilt=name):
                self.assertGreater(deg, 0.0)

    def test_scorer_lateral_tolerance_is_looser_than_the_mechanical_play(self):
        self.assertGreater(S.SEATED_LATERAL, G.CLEAR)  # a plug the geometry lets in always counts as aligned


# -- the spring plunger --------------------------------------------------------------------------------------------
def plunger_aabb():
    """Bounding box of the plunger tip (block frame) at rest: x-y footprint is its circle, z is +-PLUNGER_H/2."""
    centre, cylinders = G.plunger_slider_spec()
    (_, radius, height, _, axis), = cylinders
    assert axis == "Z"
    c = np.asarray(centre, dtype=float)
    half = np.array([radius, radius, height / 2.0])
    return c - half, c + half


class TestPlunger(unittest.TestCase):
    def test_spec_matches_the_constants(self):
        centre, ((cyl_centre, radius, height, _, axis),) = G.plunger_slider_spec()
        self.assertEqual(tuple(cyl_centre), tuple(centre))
        self.assertEqual((radius, height, axis), (G.PLUNGER_R, G.PLUNGER_H, "Z"))
        inner_edge = centre[0] + radius
        self.assertAlmostEqual(inner_edge, -G.CAV_HX + G.PLUNGER_PROUD, delta=EPS)
        self.assertAlmostEqual(centre[1], G.MOUTH_Y + G.PLUNGER_S, delta=EPS)

    def test_inner_edge_lies_inside_the_groove(self):
        groove_floor = -G.NOSE_W / 2 + G.GROOVE_DEPTH  # x of the groove's bottom surface (nose surface is -NOSE_W/2)
        inner_edge = -G.CAV_HX + G.PLUNGER_PROUD
        self.assertGreater(inner_edge, -G.NOSE_W / 2)  # past the un-grooved nose surface ...
        self.assertLess(inner_edge, groove_floor)  # ... but short of the groove's floor
        report(f"[geometry] plunger inner edge x = {inner_edge * 1000:.2f} mm; groove spans "
               f"{-G.NOSE_W / 2 * 1000:.2f} .. {groove_floor * 1000:.2f} mm: "
               f"{(groove_floor - inner_edge) * 1000:.2f} mm free to the groove floor")
        self.assertGreaterEqual(groove_floor - inner_edge, 1e-4)  # clearly free in the groove (>= 0.1 mm), not touching
        lo, hi = plunger_aabb()
        self.assertAlmostEqual(hi[0], inner_edge, delta=EPS)

    def test_plunger_tip_is_proud_of_the_wall_by_PLUNGER_PROUD(self):
        lo, hi = plunger_aabb()
        wall_face = block_aabbs()[I_LEFT][1][0]
        self.assertAlmostEqual(hi[0] - wall_face, G.PLUNGER_PROUD, delta=EPS)
        self.assertGreaterEqual(lo[0], -G.SHELL_HX)  # the body is buried in the left wall, not poking out of the shell
        self.assertGreaterEqual(lo[1], G.MOUTH_Y)
        self.assertLessEqual(hi[1], G.FLOOR_Y)

    def test_plunger_height_fits_the_nose(self):
        lo, hi = plunger_aabb()
        self.assertLess(hi[2], G.NOSE_H / 2)
        self.assertLess(G.PLUNGER_H, G.NOSE_H / 2)  # also true if PLUNGER_H were a half-length

    def penetration_when_nose_origin_is(self, s):
        """Plunger bounding box vs the nose pieces when the plug origin is ``s`` metres further in than seated
        (so the plunger is at distance PLUNGER_S - s along the nose). Returns {piece: (x, y, z overlaps)}."""
        p = plunger_aabb()
        out = {}
        for i, piece in enumerate(plug_aabbs(SEATED + [0.0, s, 0.0])):
            if i in (I_ROOT, I_GROOVE, I_TIP):
                out[PLUG_NAMES[i]] = tuple(np.minimum(p[1], piece[1]) - np.maximum(p[0], piece[0]))
        return out

    def test_seated_plunger_rests_in_the_groove_touching_nothing(self):
        for name, (dx, dy, dz) in self.penetration_when_nose_origin_is(0.0).items():
            with self.subTest(piece=name):
                self.assertLessEqual(min(dx, dy, dz), 0.0)  # no volume overlap
        pen = self.penetration_when_nose_origin_is(0.0)["nose groove piece"]
        self.assertLessEqual(pen[0], -1e-4)  # at least 0.1 mm clear of the groove floor in x (1 mm with this geometry)
        self.assertGreater(pen[1], 0.0)  # it is level with the groove along the nose (y), z too
        self.assertGreater(pen[2], 0.0)

    def test_plunger_sits_in_the_middle_of_the_groove_along_the_nose(self):
        g0, g1 = G.GROOVE
        lo, hi = plunger_aabb()
        margin_root = (lo[1] - G.MOUTH_Y) - g0
        margin_tip = (G.MOUTH_Y + g1) - hi[1]
        self.assertGreater(margin_root, 0.0)
        self.assertGreater(margin_tip, 0.0)
        self.assertTrue(0.0 < g0 < g1 < G.NOSE_L)
        report(f"[geometry] plunger has {margin_root * 1000:.2f} mm (root side) and {margin_tip * 1000:.2f} mm "
               f"(tip side) of margin inside the {(g1 - g0) * 1000:.1f} mm wide groove")

    def test_plunger_presses_into_the_ungrooved_nose_by_half_a_millimetre(self):
        centre_s = G.PLUNGER_S
        for name, s_on_nose in [("nose root", 0.004), ("nose tip", 0.015)]:
            with self.subTest(piece=name):
                pen = self.penetration_when_nose_origin_is(centre_s - s_on_nose)[name]
                self.assertAlmostEqual(pen[0], G.PLUNGER_PROUD - G.CLEAR, delta=EPS)  # 0.5 mm of x overlap
                self.assertGreater(pen[1], 0.0)
                self.assertGreater(pen[2], 0.0)
        self.assertAlmostEqual(G.PLUNGER_PROUD - G.CLEAR, 0.0005, delta=EPS)

    def test_deflection_stays_within_travel_and_the_detent_threshold_separates_the_states(self):
        pressed = G.PLUNGER_PROUD - G.CLEAR  # how far the un-grooved nose pushes the plunger back
        self.assertLess(pressed, G.PLUNGER_TRAVEL)  # the plunger never bottoms out
        self.assertLess(-pressed, S.DETENT_MIN)  # riding on the nose: detent reads below the seated threshold
        self.assertGreater(0.0, S.DETENT_MIN)  # in the groove (free): detent reads 0, above it
        # the spring force when riding on the nose is what resists insertion
        report(f"[geometry] plunger deflection on the nose {pressed * 1000:.2f} mm, spring force "
               f"{G.PLUNGER_RATE * pressed:.1f} N; scorer's detent threshold {S.DETENT_MIN * 1000:.2f} mm")

    def test_plunger_overlaps_the_left_wall_box_which_is_why_it_is_excluded(self):
        # the plunger body is embedded in the (solid) left wall box: documented here so nobody counts it as a fault
        v = overlap_volume(plunger_aabb(), block_aabbs()[I_LEFT])
        self.assertGreater(v, 0.0)


# -- poses the geometry must reject, and the insertion path ---------------------------------------------------------
class TestRejectedPoses(_OverlapAssertions, unittest.TestCase):
    def test_shifted_sideways_by_play_plus_half_a_mm(self):
        s = G.CLEAR + 0.0005  # the play is CLEAR per side, so anything past it must touch
        for name, d in [("+x", (s, 0, 0)), ("-x", (-s, 0, 0)), ("+z", (0, 0, s)), ("-z", (0, 0, -s))]:
            with self.subTest(shift=name):
                worst = self.assertRejected(plug_aabbs(SEATED + np.array(d)), f"{s * 1000:.1f} mm {name}:")
                report(f"[geometry] seated plug shifted {s * 1000:.1f} mm {name}: worst overlap {describe(worst)}")

    def test_play_is_exactly_CLEAR(self):
        for name, d in [("+x", (1, 0, 0)), ("-x", (-1, 0, 0)), ("+z", (0, 0, 1)), ("-z", (0, 0, -1)),
                        ("+x+z", (1, 0, 1)), ("-x-z", (-1, 0, -1)), ("+x-z", (1, 0, -1))]:
            d = np.array(d, dtype=float)
            with self.subTest(shift=name):
                self.assertClear(plug_aabbs(SEATED + 0.9 * G.CLEAR * d), f"0.9 CLEAR {name}:")
                self.assertRejected(plug_aabbs(SEATED + 1.2 * G.CLEAR * d), f"1.2 CLEAR {name}:")

    def test_rolled_180_about_the_nose_axis_key_down(self):
        for name, m in [("z mirrored (as in the brief)", MIRROR_Z), ("true half turn about y", ROLL_180)]:
            with self.subTest(transform=name):
                worst = self.assertRejected(plug_aabbs(SEATED, m), f"{name}:")
                report(f"[geometry] plug key-down, {name}: worst overlap {describe(worst)}")

    def test_rolled_90_about_the_nose_axis(self):
        for sign, m in [(+1, ROLL_90), (-1, ROLL_90.T)]:
            with self.subTest(sign=sign):
                self.assertRejected(plug_aabbs(SEATED, m), "roll 90:")

    def test_yawed_90_degrees(self):
        for sign, m in [(+1, YAW_90), (-1, YAW_90.T)]:
            with self.subTest(sign=sign):
                worst = self.assertRejected(plug_aabbs(SEATED, m), f"yaw {sign * 90}:")
                report(f"[geometry] plug yawed {sign * 90} deg: worst overlap {describe(worst)}")

    def test_yawed_180_degrees_nose_pointing_away(self):
        self.assertRejected(plug_aabbs(SEATED, YAW_180), "yaw 180:")

    def test_pushed_5mm_further_in_housing_inside_the_shell(self):
        worst = self.assertRejected(plug_aabbs(SEATED + [0.0, 0.005, 0.0]), "5 mm too far in:")
        report(f"[geometry] plug 5 mm further in: worst overlap {describe(worst)}")
        # it is the housing that hits the shell walls
        v, i, j = worst
        self.assertEqual(PLUG_NAMES[i], "housing")

    def test_housing_hits_the_shell_as_soon_as_it_goes_any_further_in(self):
        self.assertClear(plug_aabbs(SEATED + [0.0, -1e-6, 0.0]), "1 um short:")
        self.assertRejected(plug_aabbs(SEATED + [0.0, 0.0005, 0.0]), "0.5 mm too far:")


class TestInsertionPath(_OverlapAssertions, unittest.TestCase):
    STEPS = 41  # 0.04 m in front of the mouth to seated, 1 mm apart

    def origins(self, dx=0.0, dz=0.0):
        return [np.array([dx, G.MOUTH_Y - 0.04 + 0.001 * k, dz]) for k in range(self.STEPS)]

    def test_sweep_from_4cm_out_to_seated_never_touches_the_block(self):
        origins = self.origins()
        self.assertAlmostEqual(origins[-1][1], G.MOUTH_Y, delta=1e-12)
        for k, origin in enumerate(origins):
            with self.subTest(mm_before_seated=40 - k):
                self.assertClear(plug_aabbs(origin), f"{40 - k} mm before seated:")

    def test_sweep_is_clear_for_every_offset_within_the_play(self):
        off = 0.9 * G.CLEAR
        for dx in (-off, 0.0, off):
            for dz in (-off, 0.0, off):
                for k, origin in enumerate(self.origins(dx, dz)):
                    with self.subTest(dx=dx, dz=dz, mm_before_seated=40 - k):
                        self.assertClear(plug_aabbs(origin), "offset sweep:")

    def test_the_final_millimetres_are_blocked_by_a_1_2_CLEAR_offset(self):
        off = 1.2 * G.CLEAR
        for dx, dz in [(off, 0.0), (-off, 0.0), (0.0, off), (0.0, -off)]:
            with self.subTest(dx=dx, dz=dz):
                self.assertClear(plug_aabbs(self.origins(dx, dz)[0]), "start, 4 cm out:")  # not yet in the socket
                self.assertRejected(plug_aabbs(self.origins(dx, dz)[-1]), "seated:")

    def test_key_down_plug_cannot_enter(self):
        first_blocked = None
        for k, origin in enumerate(self.origins()):
            if worst_overlap(plug_aabbs(origin, MIRROR_Z), block_aabbs())[0] > VOL_TOL:
                first_blocked = k
                break
        self.assertIsNotNone(first_blocked)
        # blocked as soon as the nose tip gets into the mouth: origin 1 mm closer than the tip-at-mouth position
        tip_at_mouth = G.MOUTH_Y - G.NOSE_L
        self.assertLessEqual(abs(self.origins()[first_blocked][1] - tip_at_mouth), 0.001 + 1e-9)
        report(f"[geometry] key-down plug is stopped {(G.MOUTH_Y - self.origins()[first_blocked][1]) * 1000:.0f} mm "
               f"short of seated (nose tip at the mouth is {G.NOSE_L * 1000:.0f} mm short)")
        # the true half turn also fails to enter
        self.assertTrue(any(worst_overlap(plug_aabbs(o, ROLL_180), block_aabbs())[0] > VOL_TOL
                            for o in self.origins()))

    def test_yawed_plug_cannot_enter(self):
        for name, m in [("+90", YAW_90), ("-90", YAW_90.T)]:
            with self.subTest(yaw=name):
                self.assertTrue(any(worst_overlap(plug_aabbs(o, m), block_aabbs())[0] > VOL_TOL
                                    for o in self.origins()))


if __name__ == "__main__":
    unittest.main()
