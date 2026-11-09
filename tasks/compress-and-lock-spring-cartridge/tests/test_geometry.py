"""Tests for geometry and layout constants in compress-and-lock-spring-cartridge."""

import math
import unittest
import numpy as np

import geometry as g


class TestGeometry(unittest.TestCase):
    def test_dimensions(self):
        self.assertEqual(len(g.BASE_SIZE), 3)
        self.assertEqual(len(g.PLUNGER_KNOB_SIZE), 3)
        self.assertEqual(len(g.LOCK_BAR_HANDLE_SIZE), 3)
        self.assertEqual(len(g.CARTRIDGE_HOUSING_SIZE), 3)
        self.assertEqual(len(g.LAMP_SIZE), 3)

        # Plunger knob finger width requirement (>= 40 mm)
        self.assertGreaterEqual(g.PLUNGER_KNOB_SIZE[1], 0.040)
        # Lock bar handle width
        self.assertGreaterEqual(g.LOCK_BAR_HANDLE_SIZE[0], 0.040)

    def test_spring_rate_and_force(self):
        # Return spring rate <= 50 N/m, force at full stroke <= 2.5 N per lesson 10
        self.assertLessEqual(g.PLUNGER_SPRING_K, 50.0)
        max_force = g.PLUNGER_SPRING_K * g.PLUNGER_TRAVEL
        self.assertLessEqual(max_force, 2.50)
        self.assertAlmostEqual(max_force, 2.25, delta=0.01)

    def test_bimanual_reach_separation(self):
        # Left arm cannot reach lock bar handle start (> 0.50 m)
        d_left_to_bar = math.hypot(g.LOCK_BAR_REST_X - g.LEFT_BASE[0], g.LOCK_BAR_REST_Y - g.LEFT_BASE[1])
        self.assertGreater(d_left_to_bar, g.MAX_ARM_REACH)

        # Right arm cannot reach plunger knob start (> 0.50 m)
        d_right_to_plunger = math.hypot(g.PLUNGER_REST_X - g.RIGHT_BASE[0], g.PLUNGER_REST_Y - g.RIGHT_BASE[1])
        self.assertGreater(d_right_to_plunger, g.MAX_ARM_REACH)

        # Left arm can reach plunger knob
        d_left_to_plunger = math.hypot(g.PLUNGER_REST_X - g.LEFT_BASE[0], g.PLUNGER_REST_Y - g.LEFT_BASE[1])
        self.assertLessEqual(d_left_to_plunger, g.MAX_ARM_REACH)
        self.assertGreaterEqual(d_left_to_plunger, g.MIN_ARM_REACH)

        # Right arm can reach lock bar handle
        d_right_to_bar = math.hypot(g.LOCK_BAR_REST_X - g.RIGHT_BASE[0], g.LOCK_BAR_REST_Y - g.RIGHT_BASE[1])
        self.assertLessEqual(d_right_to_bar, g.MAX_ARM_REACH)
        self.assertGreaterEqual(d_right_to_bar, g.MIN_ARM_REACH)

    def test_arm_arm_clearance(self):
        # Lesson 5: work of the two arms separated in y, arm-arm clearance >= 30 mm
        y_separation_start = abs(g.PLUNGER_REST_Y - g.LOCK_BAR_REST_Y)
        self.assertGreaterEqual(y_separation_start, 0.250)

        # At full lock bar stroke (y = -0.100 + 0.070 = -0.030)
        y_separation_locked = abs(g.PLUNGER_REST_Y - (g.LOCK_BAR_REST_Y + g.LOCK_BAR_TRAVEL))
        self.assertGreaterEqual(y_separation_locked, 0.200)
        self.assertGreaterEqual(y_separation_locked, g.MIN_ARM_ARM_CLEARANCE)

    def test_stroke_thresholds(self):
        # Stop box blocks early push at 15 mm
        self.assertEqual(g.BAR_STOP_EARLY_LIMIT, 0.015)
        # Plunger compressed threshold is 38 mm <= 45 mm travel
        self.assertEqual(g.PLUNGER_COMPRESS_THRESHOLD, 0.038)
        self.assertLess(g.PLUNGER_COMPRESS_THRESHOLD, g.PLUNGER_TRAVEL)
        # Lock bar locked threshold is 55 mm <= 70 mm travel
        self.assertEqual(g.BAR_LOCKED_THRESHOLD, 0.055)
        self.assertLess(g.BAR_LOCKED_THRESHOLD, g.LOCK_BAR_TRAVEL)

    def test_local_slider_boxes(self):
        # Lesson 1: verify boxes for the part and sliders
        import task
        fixture = task.CARTRIDGE_FIXTURE
        for (bx, by, bz), sz, col in fixture.boxes:
            self.assertLessEqual(abs(bx), fixture.size[0] / 2 + 1e-3)
            self.assertLessEqual(abs(by), fixture.size[1] / 2 + 1e-3)
            self.assertLessEqual(abs(bz), fixture.size[2] / 2 + 1e-3)


if __name__ == "__main__":
    unittest.main()
