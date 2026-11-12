"""Unit tests for Scorer class, dwell accumulator, and ladder scoring in pack-cold-chain-shipper-with-thermal-buffer."""

from __future__ import annotations

import unittest
import numpy as np

import geometry as g
import scoring


class TestScoring(unittest.TestCase):
    def setUp(self):
        self.scorer = scoring.Scorer(hold_s=3.0, hz=20.0)
        self.brick_start = (g.PAD_RIGHT_X, g.PAD_RIGHT_Y, g.BRICK_STAGING_Z)
        self.buffer_start = (g.PAD_FRONT_X, g.PAD_FRONT_Y, g.BUFFER_STAGING_Z)
        self.carton_start = (g.PAD_LEFT_X, g.PAD_LEFT_Y, g.CARTON_STAGING_Z)

        self.brick_seated = (g.BRICK_TARGET_X, g.BRICK_TARGET_Y, g.BRICK_TARGET_Z)
        self.buffer_seated = (g.BUFFER_TARGET_X, g.BUFFER_TARGET_Y, g.BUFFER_TARGET_Z)
        self.carton_seated = (g.CARTON_TARGET_X, g.CARTON_TARGET_Y, g.CARTON_TARGET_Z)
        self.arms_home = {"left": list(g.HOME_L), "right": list(g.HOME_R)}

    def test_initial_zero(self):
        res = self.scorer.update(
            brick_pos=self.brick_start,
            buffer_pos=self.buffer_start,
            carton_pos=self.carton_start,
        )
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)
        self.assertEqual(res.details["dwell_s"], 0.0)

    def test_ladder_progression(self):
        # 1. Gel brick placed -> 0.25
        r1 = self.scorer.update(
            brick_pos=self.brick_seated,
            buffer_pos=self.buffer_start,
            carton_pos=self.carton_start,
        )
        self.assertEqual(r1.score, 0.25)
        self.assertFalse(r1.success)

        # 2. Buffer pad placed over brick -> 0.50
        r2 = self.scorer.update(
            brick_pos=self.brick_seated,
            buffer_pos=self.buffer_seated,
            carton_pos=self.carton_start,
        )
        self.assertEqual(r2.score, 0.50)
        self.assertFalse(r2.success)

        # 3. Carton placed over buffer -> 0.75
        r3 = self.scorer.update(
            brick_pos=self.brick_seated,
            buffer_pos=self.buffer_seated,
            carton_pos=self.carton_seated,
        )
        self.assertEqual(r3.score, 0.75)
        self.assertFalse(r3.success)

    def test_100_requires_full_3s_hold(self):
        # 60 steps at 20 Hz = 3.0 s
        # Steps 1 to 59: score must be 0.75
        for i in range(59):
            res = self.scorer.update(
                brick_pos=self.brick_seated,
                buffer_pos=self.buffer_seated,
                carton_pos=self.carton_seated,
                arm_tcps=self.arms_home,
            )
            self.assertEqual(res.score, 0.75, f"Premature 1.00 at step {i+1}")
            self.assertFalse(res.success)

        # Step 60: exactly 3.0 s reached -> 1.00 and success=True
        res60 = self.scorer.update(
            brick_pos=self.brick_seated,
            buffer_pos=self.buffer_seated,
            carton_pos=self.carton_seated,
            arm_tcps=self.arms_home,
        )
        self.assertEqual(res60.score, 1.00)
        self.assertTrue(res60.success)

    def test_undoing_lowers_score(self):
        # Place all 3
        self.scorer.update(
            brick_pos=self.brick_seated,
            buffer_pos=self.buffer_seated,
            carton_pos=self.carton_seated,
        )
        # Pull carton back to staging
        res = self.scorer.update(
            brick_pos=self.brick_seated,
            buffer_pos=self.buffer_seated,
            carton_pos=self.carton_start,
        )
        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.success)

    def test_contact_freezing_hazard_caps_at_025(self):
        # Carton placed directly on brick
        carton_direct = (g.SHIPPER_X, g.SHIPPER_Y, g.BRICK_TOP_SEATED_Z + g.CARTON_HEIGHT_Z / 2.0)
        res = self.scorer.update(
            brick_pos=self.brick_seated,
            buffer_pos=self.buffer_start,
            carton_pos=carton_direct,
        )
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)
        self.assertTrue(res.details["latched_violation"])

    def test_reset_clears_state(self):
        carton_direct = (g.SHIPPER_X, g.SHIPPER_Y, g.BRICK_TOP_SEATED_Z + g.CARTON_HEIGHT_Z / 2.0)
        self.scorer.update(
            brick_pos=self.brick_seated,
            buffer_pos=self.buffer_start,
            carton_pos=carton_direct,
        )
        self.assertTrue(self.scorer.latched_violation)
        self.scorer.reset()
        self.assertFalse(self.scorer.latched_violation)
        self.assertEqual(self.scorer.dwell_s, 0.0)


if __name__ == "__main__":
    unittest.main()
