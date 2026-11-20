"""Offline checks of the scorer, the stripper's geometry and the wire kit's geometry. No simulator needed:

    python -m unittest discover -s tasks/strip-wire-push-in-receptacle/tests -v      (with the yam_arena environment's Python)
"""

import pathlib
import sys
import unittest
from types import SimpleNamespace

import numpy as np

HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

import stripper as S  # noqa: E402
import task as T  # noqa: E402
import wirekit as wk  # noqa: E402


def wire(bare=0.016, latched="brass_b", depth=0.0155, nicked=False, crushed=False, slug_clear=True):
    return SimpleNamespace(bare=bare, latched=latched, depth=depth, nicked=nicked, crushed=crushed, slug_clear=slug_clear)


class Scoring(unittest.TestCase):
    def setUp(self):
        T.TOUCH.now = {"black": False}

    def score(self, **kw):
        return T.score({"black": wire(**kw)})

    def test_full_credit(self):
        self.assertAlmostEqual(self.score(), 1.0)
        self.assertTrue(T.success({"black": wire()}))

    def test_reversed_polarity_scores_zero(self):
        self.assertEqual(self.score(latched="silver_a"), 0.0)
        self.assertEqual(self.score(latched="silver_b", bare=0.0, depth=0.0), 0.0)

    def test_nicked_or_damaged_wire_scores_zero(self):
        self.assertEqual(self.score(nicked=True), 0.0)
        self.assertEqual(self.score(crushed=True), 0.0)

    def test_strip_length_windows(self):
        self.assertAlmostEqual(self.score(bare=0.014, depth=0.014), 1.0)
        self.assertAlmostEqual(self.score(bare=0.018, depth=0.0172), 1.0)
        self.assertAlmostEqual(self.score(bare=0.020, depth=0.0175, latched=None), 0.15)  # partial strip, too long to seat
        self.assertAlmostEqual(self.score(bare=0.012, depth=0.012), 0.15 + 0.7)  # partial strip credit, seated
        self.assertEqual(self.score(bare=0.010, depth=0.0, latched=None), 0.0)  # too short: no credit, cannot reach the clamp

    def test_exposed_copper_loses_termination(self):
        self.assertAlmostEqual(self.score(bare=0.018, depth=0.016), 0.30)  # 2 mm showing
        self.assertAlmostEqual(self.score(bare=0.018, depth=0.0166), 1.0)  # 1.4 mm showing

    def test_jacket_inside_the_hole_loses_termination(self):
        self.assertAlmostEqual(self.score(bare=0.016, depth=0.0172), 1.0)  # 1.2 mm of jacket nested in the lead-in
        self.assertAlmostEqual(self.score(bare=0.012, depth=0.0137), 0.15)  # 1.7 mm of jacket forced in: the clamp is on insulation

    def test_slug_left_on_or_unstripped(self):
        self.assertEqual(self.score(slug_clear=False), 0.0)
        self.assertEqual(self.score(bare=0.0, latched=None, depth=0.0), 0.0)

    def test_still_held_loses_termination(self):
        T.TOUCH.now = {"black": True}
        self.assertAlmostEqual(self.score(), 0.30)

    def test_not_pushed_home(self):
        self.assertAlmostEqual(self.score(latched=None, depth=0.0), 0.30)


class StripperGeometry(unittest.TestCase):
    def test_dimensions_follow_the_klein_11055(self):
        self.assertAlmostEqual(S.JAW_TIP - S.HANDLE_END, 0.181, places=3)
        self.assertAlmostEqual(S.MASS, 0.154)

    def test_holes_larger_nearer_the_pivot(self):
        xs = [S.HOLES[g][0] for g in (10, 12, 14, 16, 18)]
        ds = [S.HOLES[g][1] for g in (10, 12, 14, 16, 18)]
        self.assertEqual(xs, sorted(xs))
        self.assertEqual(ds, sorted(ds, reverse=True))

    def test_blade_holds_jacket_and_passes_copper(self):
        rb = S.blade_r(S.HOLES[14][1])
        self.assertGreater(rb, 0.000815)  # 1.63 mm copper passes
        self.assertLess(rb, 0.001346)  # 2.69 mm jacket bears on it
        self.assertGreater(S.HOLE / 2, 0.001346 + 0.0005)  # the notch does not pinch the jacket

    def test_handle_spans_fit_the_gripper(self):
        open_ = S.handle_point(-0.115, "lower", S.OPEN_J)[1] - S.handle_point(-0.115, "upper", S.OPEN_J)[1]
        shut = S.handle_point(-0.115, "lower", S.SHUT_J)[1] - S.handle_point(-0.115, "upper", S.SHUT_J)[1]
        self.assertLess(open_ + S.HANDLE_W, 0.090)  # the YAM's fingers open about 94 mm
        self.assertAlmostEqual(shut, 2 * S.HANDLE_Y, places=6)

    def test_mechanism_builds(self):
        text = S.make("s", (0, 0, 0)).build()
        self.assertIn("upper_joint", text)
        self.assertEqual(text.count("torsionalPatchRadius"), 2)


class WireGeometry(unittest.TestCase):
    def test_ring_boundaries_fall_on_the_gauge(self):
        p = wk.Pigtail("w", pos=(0, 0, 0), size=(0.003, 0.003, 0.15))
        from_end = [round((p.length / 2 - (p.zone_start + k * p.pitch)) * 1000, 3) for k in range(p.n_rings + 1)]
        self.assertIn(16.0, from_end)
        self.assertIn(14.0, from_end)
        self.assertIn(18.0, from_end)

    def test_latch_ratchet(self):
        p = wk.Pigtail("w", pos=(0, 0, 0), size=(0.003, 0.003, 0.15))
        d = p.latch_depths
        self.assertAlmostEqual(d[0], 0.012)
        self.assertAlmostEqual(d[-1], 0.0175)
        self.assertTrue(all(abs(b - a - 0.0005) < 1e-9 for a, b in zip(d, d[1:])))

    def test_plane_hit_and_hole_lookup(self):
        hit = wk.plane_hit(np.array([0.026, 0.0003, -0.05]), np.array([0.0, 0.0, 1.0]), np.zeros(3), np.eye(3))
        self.assertAlmostEqual(hit[1], 0.05)
        self.assertEqual(wk.hole_at(hit[0][:2], S.HOLES, S.HOLE / 2), 14)
        self.assertIsNone(wk.hole_at((0.029, 0.0), S.HOLES, S.HOLE / 2))

    def test_jacket_stops_at_the_receptacle(self):
        self.assertGreater(2 * 0.001346, T.CHANNEL)  # the jacket cannot enter a push-in channel
        self.assertLess(2 * 0.000815, T.CHANNEL)  # the copper can
        self.assertGreaterEqual(T.CH_DEPTH, T.BARE_FULL[1] - T.EXPOSED_MAX)


class Layout(unittest.TestCase):
    def lead_lengths(self):
        pts = np.array(T.LEAD)
        return np.linalg.norm(np.diff(pts, axis=0), axis=1)

    def test_box_is_a_b118a(self):
        inch = 0.0254
        self.assertAlmostEqual(T.BOX_HT, 3.75 * inch, places=3)
        self.assertAlmostEqual(T.BOX_W, 2.25 * inch, places=3)
        self.assertAlmostEqual(T.BOX_D, 2.875 * inch, places=3)

    def test_conductor_meets_the_box_fill_rules(self):
        d = self.lead_lengths()
        outside = d[1:].sum() + T.CORE_L  # from the box's front to the end
        self.assertGreater(d.sum() + T.CORE_L, 0.150)  # NEC 300.14: at least 6 in of free conductor
        self.assertGreater(outside, 0.075)  # and at least 3 in outside the box's opening
        self.assertGreater(T.LEAD[0][0], T.WALL_X)  # anchored inside the box
        self.assertLess(T.LEAD[1][0], T.WALL_X)  # and out through its front

    def test_lead_joins_the_end_and_stays_above_the_table(self):
        self.assertTrue(np.allclose(T.LEAD[-1], T.CORE_C - [0, 0, T.CORE_L / 2]))
        self.assertTrue(all(p[2] > T.TOP + T.WIRE_R for p in T.LEAD))

    def test_receptacle_on_its_tray_brass_on_minus_y(self):
        self.assertAlmostEqual(T.REC_START[2], T.TRAY_TOP + 0.0005)
        self.assertTrue(all(dy < 0 for n, (_, dy) in T.HOLES.items() if n.startswith("brass")))
        self.assertTrue(all(dy > 0 for n, (_, dy) in T.HOLES.items() if n.startswith("silver")))
        self.assertTrue(all(h[1] == "receptacle" for h in [(None, T.OBJECTS[3].hole_body)]))

    def test_copper_yield_moment(self):
        d = 0.00163  # 14 AWG conductor
        self.assertAlmostEqual(70e6 * d ** 3 / 6, wk.LEAD_YIELD, delta=0.01)  # annealed copper, plastic section modulus


if __name__ == "__main__":
    unittest.main()
