"""Unit tests for scoring ladder, partial credit tiers, and zero-gates."""

import unittest
import numpy as np

import geometry as g
from scoring import Scorer


class TestScoring(unittest.TestCase):
    def setUp(self):
        self.scorer = Scorer(hold_s=2.0, hz=20.0)

    def test_ladder_progression(self):
        tray_block_pos = (g.TRAY_POS[0], g.TRAY_POS[1], g.TOP + 0.020)

        # 0.00 at start
        res = self.scorer.update(0.0, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)

        # 0.25: E-STOP pressed and pusher stopped (6 steps)
        self.scorer.update(-0.007, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        for _ in range(6):
            res = self.scorer.update(0.0, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

        # 0.50: Block extracted into green tray
        res = self.scorer.update(0.0, 0.0, 0.0, tray_block_pos, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.success)

        # 0.75: RESET then START pressed
        self.scorer.update(0.0, -0.007, 0.0, tray_block_pos, {"left": g.HOME_L, "right": g.HOME_R})
        res = self.scorer.update(0.0, 0.0, -0.007, tray_block_pos, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

        # Hold for 40 steps (2.0 s)
        for _ in range(40):
            res = self.scorer.update(0.0, 0.0, 0.0, tray_block_pos, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)

    def test_no_estop_fault_latches_zero(self):
        # Reach in at step 0 while running
        tcp_in_danger = (0.400, 0.100, g.TOP + 0.050)
        res = self.scorer.update(0.0, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": tcp_in_danger, "right": g.HOME_R})
        self.assertEqual(res.score, 0.0)
        self.assertTrue(self.scorer.latched_fault)

        # Even if E-STOP pressed later, score remains 0.0
        for _ in range(10):
            res = self.scorer.update(-0.007, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)

    def test_start_early_fault_caps_at_half(self):
        # E-STOP stopped -> 0.25
        self.scorer.update(-0.007, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        for _ in range(6):
            res = self.scorer.update(0.0, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertEqual(res.score, 0.25)

        # RESET pressed -> 0.50
        res = self.scorer.update(0.0, -0.007, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertEqual(res.score, 0.50)

        # START pressed while block still in lane -> FAULT
        res = self.scorer.update(0.0, 0.0, -0.007, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertEqual(res.score, 0.50)
        self.assertTrue(self.scorer.latched_fault)

        # Even if block is removed later, score cannot advance past 0.50
        tray_pos = (g.TRAY_POS[0], g.TRAY_POS[1], g.TOP + 0.020)
        for _ in range(40):
            res = self.scorer.update(0.0, 0.0, 0.0, tray_pos, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.success)

    def test_skip_reset_stays_half(self):
        # E-STOP stopped -> 0.25
        self.scorer.update(-0.007, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})
        for _ in range(6):
            self.scorer.update(0.0, 0.0, 0.0, g.BLOCK_INIT_POS, {"left": g.HOME_L, "right": g.HOME_R})

        # Block cleared to tray -> 0.50
        tray_pos = (g.TRAY_POS[0], g.TRAY_POS[1], g.TOP + 0.020)
        res = self.scorer.update(0.0, 0.0, 0.0, tray_pos, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertEqual(res.score, 0.50)

        # START pressed without RESET -> START ignored
        res = self.scorer.update(0.0, 0.0, -0.007, tray_pos, {"left": g.HOME_L, "right": g.HOME_R})
        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.success)


if __name__ == "__main__":
    unittest.main()
