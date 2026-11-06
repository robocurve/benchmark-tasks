"""Tests for geometry constants and layout in lockout-tagout-before-service."""

import math
import unittest
import numpy as np

import geometry as g


class TestGeometry(unittest.TestCase):
    def test_fixture_dimensions(self):
        self.assertEqual(len(g.BASE_SIZE), 3)
        self.assertGreater(g.BASE_SIZE[0], 0.15)
        self.assertGreater(g.BASE_SIZE[1], 0.40)
        self.assertEqual(g.TOP, 0.7475)
        self.assertGreater(g.BASE_TOP_Z, g.TOP)

    def test_relay_reach_separation(self):
        """Right arm cannot reach pin stand (> 0.50 m), enforcing bimanual operation."""
        d_right_to_pin = math.hypot(g.PIN_STAND_POS[0] - g.RIGHT_BASE[0], g.PIN_STAND_POS[1] - g.RIGHT_BASE[1])
        self.assertGreater(d_right_to_pin, g.MAX_ARM_REACH)
        self.assertGreater(d_right_to_pin, 0.55)

    def test_left_arm_reaches_pin_and_breaker(self):
        d_left_to_pin = math.hypot(g.PIN_STAND_POS[0] - g.LEFT_BASE[0], g.PIN_STAND_POS[1] - g.LEFT_BASE[1])
        self.assertLess(d_left_to_pin, g.MAX_ARM_REACH)

        d_left_to_breaker = math.hypot(g.LEVER_START_X - g.LEFT_BASE[0], g.LEVER_Y - g.LEFT_BASE[1])
        self.assertLess(d_left_to_breaker, g.MAX_ARM_REACH)

    def test_slider_travels_and_thresholds(self):
        self.assertEqual(g.LEVER_THROW, 0.040)
        self.assertLessEqual(g.BREAKER_OFF_THRESHOLD, g.LEVER_THROW)
        self.assertEqual(g.COVER_TRAVEL, 0.090)
        self.assertLessEqual(g.COVER_OPEN_MIN, g.COVER_TRAVEL)
        self.assertLess(g.COVER_FAULT_OPEN, g.COVER_OPEN_MIN)

    def test_pin_and_hole_alignment(self):
        self.assertEqual(g.HOLE_POS[0], 0.280)
        self.assertEqual(g.HOLE_POS[1], g.LEVER_Y)
        self.assertLess(g.PIN_RADIUS * 2, 0.018)  # fits in 18 mm hole


if __name__ == "__main__":
    unittest.main()

