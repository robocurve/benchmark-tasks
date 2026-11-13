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
        self.lid_seated = (g.SHIPPER_X, g.SHIPPER_Y, g.LID_SEATED_Z)
        self.latch_rest = {"latch_pos_y": 0.0, "latch_neg_y": 0.0}

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

    def _closed(self, **over):
        kw = dict(
            brick_pos=self.brick_seated,
            buffer_pos=self.buffer_seated,
            carton_pos=self.carton_seated,
            arm_tcps=self.arms_home,
            lid_pos=self.lid_seated,
            latch_press=self.latch_rest,
        )
        kw.update(over)
        return self.scorer.update(**kw)

    def test_100_requires_full_3s_hold(self):
        # 60 steps at 20 Hz = 3.0 s
        # Steps 1 to 59: score must be 0.75
        for i in range(59):
            res = self._closed()
            self.assertEqual(res.score, 0.75, f"Premature 1.00 at step {i+1}")
            self.assertFalse(res.success)

        # Step 60: exactly 3.0 s reached -> 1.00 and success=True
        res60 = self._closed()
        self.assertEqual(res60.score, 1.00)
        self.assertTrue(res60.success)

    def test_hold_restarts_when_the_lid_is_lifted(self):
        for _ in range(59):
            self._closed()
        # the lid pops up 6 mm for one step: the hold restarts and the score is the state now
        res = self._closed(lid_pos=(g.SHIPPER_X, g.SHIPPER_Y, g.LID_SEATED_Z + 0.006))
        self.assertEqual(res.score, 0.75)
        self.assertEqual(res.details["dwell_s"], 0.0)
        for i in range(59):
            self.assertEqual(self._closed().score, 0.75, f"hold did not restart (step {i + 1})")
        self.assertEqual(self._closed().score, 1.0)

    def test_no_lid_never_reaches_full_score(self):
        for _ in range(200):
            res = self.scorer.update(
                brick_pos=self.brick_seated,
                buffer_pos=self.buffer_seated,
                carton_pos=self.carton_seated,
                arm_tcps=self.arms_home,
                lid_pos=(g.LID_PAD_X, g.LID_PAD_Y_A, g.LID_STAGING_Z),
                latch_press=self.latch_rest,
            )
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

    def test_lid_laid_on_the_latch_heads_is_not_closed(self):
        on_heads = (g.SHIPPER_X, g.SHIPPER_Y, g.LID_ON_LATCH_BOTTOM_Z + g.LID_THICK_Z / 2.0)
        for _ in range(200):
            res = self._closed(lid_pos=on_heads)
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

    def test_lid_needs_both_latches(self):
        for pushed in ("latch_pos_y", "latch_neg_y"):
            self.scorer.reset()
            press = dict(self.latch_rest)
            press[pushed] = 0.006  # this head is pushed out past the lid edge: it does not hang over the lid
            for _ in range(200):
                res = self._closed(latch_press=press)
            self.assertEqual(res.score, 0.75, pushed)
            self.assertFalse(res.success, pushed)

    def test_undoing_the_lid_lowers_the_score(self):
        for _ in range(60):
            res = self._closed()
        self.assertEqual(res.score, 1.0)
        res = self._closed(lid_pos=(g.SHIPPER_X, g.SHIPPER_Y, g.LID_ON_LATCH_BOTTOM_Z + g.LID_THICK_Z / 2.0))
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

    def test_lid_on_a_carton_resting_on_the_brick_is_the_latched_violation(self):
        carton_direct = (g.SHIPPER_X, g.SHIPPER_Y, g.BRICK_TOP_SEATED_Z + g.CARTON_HEIGHT_Z / 2.0)
        for _ in range(200):
            res = self._closed(buffer_pos=self.buffer_start, carton_pos=carton_direct)
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)
        self.assertTrue(res.details["latched_violation"])
        # the violation stays latched even if the stack is later put right
        res = self._closed()
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

    def test_closed_lid_with_a_missing_layer_scores_only_the_layers(self):
        cases = (
            # brick + buffer, no carton: 0.50
            dict(carton_pos=self.carton_start, expect=0.50),
            # brick only: 0.25
            dict(buffer_pos=self.buffer_start, carton_pos=self.carton_start, expect=0.25),
            # no brick on its seat: 0.0
            dict(brick_pos=self.brick_start, buffer_pos=self.buffer_start, carton_pos=self.carton_start, expect=0.0),
        )
        for case in cases:
            self.scorer.reset()
            expect = case.pop("expect")
            for _ in range(200):
                res = self._closed(**case)
            self.assertEqual(res.score, expect, case)
            self.assertFalse(res.success)

    def test_arms_in_the_way_hold_the_score_at_075(self):
        busy = {"left": [g.SHIPPER_X, g.SHIPPER_Y, g.CAVITY_RIM_Z + 0.010], "right": list(g.HOME_R)}
        for _ in range(200):
            res = self._closed(arm_tcps=busy)
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

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
