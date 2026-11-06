"""Offline predicted scores of every run.sh mode through the REAL Scorer (idealised physics, see predict_scores.py)."""

from __future__ import annotations

import itertools
import os
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
sys.path.insert(0, str(HERE))

import geometry as g
import logic
import predict_scores as ps

EPS = g.PUSH_OVERSHOOT * 1000


class TestPredictedScores(unittest.TestCase):
    def tearDown(self):
        logic.set_variant(0)

    def test_demo_scores_one_on_every_variant_even_with_placement_offsets(self):
        for v in range(3):
            for off in itertools.product((-EPS, 0.0, EPS), repeat=2):
                self.assertEqual(ps.replay("demo", v, offset=off)["final"], 1.0, f"variant {v} offset {off}")

    def test_recover_scores_one(self):
        self.assertEqual(ps.replay("recover", 0)["final"], 1.0)

    def test_no_read_is_capped_at_half(self):
        for off in itertools.product((-EPS, EPS), repeat=2):
            self.assertEqual(ps.replay("no_read", 0, offset=off)["final"], 0.5)

    def test_axis_search_stays_below_the_top_rung(self):
        for v in range(3):
            for off in itertools.product((-EPS, EPS), repeat=2):
                r = ps.replay("axis_search", v, offset=off)
                self.assertLess(r["final"], 1.0, f"variant {v}")
                self.assertLess(r["final_r"], 0.95)
        self.assertEqual(ps.replay("axis_search", 0)["final"], 0.75)

    def test_memorised_peak_works_only_on_variant_zero(self):
        """N3: a policy that has memorised the variant-0 peak and takes 3 separated readings scores 1.0 on variant 0 only."""
        self.assertEqual(ps.replay("memorize", 0)["final"], 1.0)
        for v in (1, 2):
            self.assertLess(ps.replay("memorize", v)["final"], 0.75)

    def test_settle_scores_zero(self):
        self.assertEqual(ps.replay("settle", 0)["final"], 0.0)

    def test_demo_uses_at_most_eight_presses_and_enough_separated_ones(self):
        for v in range(3):
            r = ps.replay("demo", v)
            self.assertLessEqual(r["presses"], 8)
            self.assertGreaterEqual(r["separated"], logic.MIN_SEPARATED_FOR_1_00)

    def test_a_press_that_never_registers_scores_below_a_quarter_pass(self):
        """If READ never fires (the N1 failure) the demo cannot pass 0.5: documents what the old design would have scored."""
        r = ps.replay("demo", 0, press_ok=False)
        self.assertLessEqual(r["final"], 0.5)
        self.assertEqual(r["presses"], 0)


if __name__ == "__main__":
    unittest.main()
