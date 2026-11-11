"""Tests for scoring ladder and zero-gates in bridge-plate-across-trench."""

import unittest
import numpy as np

import geometry as g
from scoring import Scorer


class TestScoring(unittest.TestCase):
    def setUp(self):
        self.scorer = Scorer(hold_s=2.0, hz=20.0)

    def test_ladder_progression(self):
        arm_tcps = {
            "left": (0.200, 0.280, 0.880),
            "right": (0.200, -0.280, 0.850),
        }

        # Step 0: start configuration -> 0.00
        res = self.scorer.update(
            bridge_pos=g.BRIDGE_START_POS,
            block_pos=g.BLOCK_START_POS,
            arm_tcps=arm_tcps,
        )
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)
        self.assertIn("dbg_fell", res.details)
        self.assertIn("dbg_bridge_seated", res.details)

        # Step 1: Bridge seated across trench -> 0.25 (7 steps to settle resting)
        for _ in range(8):
            res = self.scorer.update(
                bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
                block_pos=g.BLOCK_START_POS,
                arm_tcps=arm_tcps,
            )
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

        # Step 2: Block on bridge in crossing zone -> 0.50
        block_on_bridge = (0.280, 0.000, g.PLATFORM_TOP_Z + g.BRIDGE_SIZE[2] + g.BLOCK_SIZE[2] / 2.0)
        res = self.scorer.update(
            bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
            block_pos=block_on_bridge,
            arm_tcps=arm_tcps,
        )
        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.success)

        # Step 3: Block on far platform -> 0.75
        block_far = (0.300, -0.080, g.PLATFORM_TOP_Z + g.BLOCK_SIZE[2] / 2.0)
        res = self.scorer.update(
            bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
            block_pos=block_far,
            arm_tcps=arm_tcps,
        )
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

        # Step 4: Block on target mark held 2.0 s (40 steps at 20 Hz) -> 1.00
        block_target = (g.TARGET_MARK_POS[0], g.TARGET_MARK_POS[1], g.PLATFORM_TOP_Z + g.BLOCK_SIZE[2] / 2.0)
        for _ in range(40):
            res = self.scorer.update(
                bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
                block_pos=block_target,
                arm_tcps=arm_tcps,
            )
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)

    def test_zero_gate_fell_in_trench(self):
        arm_tcps = {"left": (0.200, 0.280, 0.880), "right": (0.200, -0.280, 0.850)}

        # Seat bridge -> 0.25
        for _ in range(8):
            res = self.scorer.update(
                bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
                block_pos=g.BLOCK_START_POS,
                arm_tcps=arm_tcps,
            )
        self.assertEqual(res.score, 0.25)

        # Block falls into trench (z < FELL_Z_THRESHOLD)
        res = self.scorer.update(
            bridge_pos=g.BRIDGE_SEATED_TARGET_POS,
            block_pos=(0.280, 0.000, g.FELL_Z_THRESHOLD - 0.010),
            arm_tcps=arm_tcps,
        )
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)
        self.assertTrue(res.details["dbg_fell"])


if __name__ == "__main__":
    unittest.main()
