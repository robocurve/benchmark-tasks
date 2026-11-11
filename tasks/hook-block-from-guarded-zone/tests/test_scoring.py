"""Tests for scoring ladder and zero-gates in hook-block-from-guarded-zone."""

import unittest
import numpy as np

import geometry as g
from scoring import Scorer


class TestScoring(unittest.TestCase):
    def setUp(self):
        self.scorer = Scorer(hold_s=2.0, rest_s=0.3, hz=20.0)

    def test_ladder_progression(self):
        safe_tcps = {
            "left": (0.200, 0.280, 0.850),
            "right": (0.200, -0.280, 0.850),
        }

        # Step 0: start
        res = self.scorer.update(
            block_pos=g.BLOCK_INIT_POS,
            rod_pos=g.ROD_INIT_POS,
            arm_tcps=safe_tcps,
        )
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)

        # Step 1: Rod lifted by arm -> 0.25
        rod_lifted = (0.210, 0.250, g.TOP + 0.050)
        res = self.scorer.update(
            block_pos=g.BLOCK_INIT_POS,
            rod_pos=rod_lifted,
            arm_tcps=safe_tcps,
        )
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

        # Step 2: Block moved >= 60 mm (x <= 0.520) -> 0.50
        block_moved = (0.500, 0.000, g.TOP + g.BLOCK_HEIGHT / 2)
        res = self.scorer.update(
            block_pos=block_moved,
            rod_pos=rod_lifted,
            arm_tcps=safe_tcps,
        )
        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.success)

        # Step 3: Block outside zone (x < 0.380 resting) -> 0.75
        block_outside = (0.350, 0.000, g.TOP + g.BLOCK_HEIGHT / 2)
        for _ in range(7):
            res = self.scorer.update(
                block_pos=block_outside,
                rod_pos=rod_lifted,
                block_vel=(0.0, 0.0, 0.0),
                arm_tcps=safe_tcps,
            )
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

        # Step 4: Block in tray, rod parked outside zone, arms clear -> 1.00 after 2.0 s (40 steps)
        block_in_tray = (g.TRAY_POS[0], g.TRAY_POS[1], g.TRAY_SEAT_Z)
        rod_parked = g.ROD_PARK_POS

        for _ in range(40):
            res = self.scorer.update(
                block_pos=block_in_tray,
                rod_pos=rod_parked,
                block_vel=(0.0, 0.0, 0.0),
                rod_vel=(0.0, 0.0, 0.0),
                arm_tcps=safe_tcps,
            )
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)

    def test_zero_gate_boundary_violation(self):
        safe_tcps = {"left": (0.200, 0.280, 0.850), "right": (0.200, -0.280, 0.850)}
        # Advance to 0.75
        rod_lifted = (0.210, 0.250, g.TOP + 0.050)
        block_outside = (0.350, 0.000, g.TOP + g.BLOCK_HEIGHT / 2)
        for _ in range(7):
            res = self.scorer.update(
                block_pos=block_outside,
                rod_pos=rod_lifted,
                block_vel=(0.0, 0.0, 0.0),
                arm_tcps=safe_tcps,
            )
        self.assertEqual(res.score, 0.75)

        # Arm breaches yellow line (x > 0.410) -> score collapses to 0.00
        fault_tcps = {"left": (0.425, 0.100, 0.800), "right": (0.200, -0.280, 0.850)}
        res = self.scorer.update(
            block_pos=block_outside,
            rod_pos=rod_lifted,
            block_vel=(0.0, 0.0, 0.0),
            arm_tcps=fault_tcps,
        )
        self.assertEqual(res.score, 0.00)
        self.assertFalse(res.success)

    def test_scorer_reset(self):
        self.scorer.update(
            block_pos=g.BLOCK_INIT_POS,
            rod_pos=(0.210, 0.250, g.TOP + 0.050),
            arm_tcps={"left": (0.200, 0.280, 0.850), "right": (0.200, -0.280, 0.850)},
        )
        self.assertEqual(self.scorer.best_score, 0.25)
        self.scorer.reset()
        self.assertEqual(self.scorer.best_score, 0.0)


if __name__ == "__main__":
    unittest.main()
