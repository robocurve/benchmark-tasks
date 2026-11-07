"""Unit tests for geometry constants, layouts, and danger zone boundaries."""

import math
import unittest
import numpy as np

import geometry as g


class TestGeometry(unittest.TestCase):
    def test_table_and_reach_constants(self):
        self.assertEqual(g.TOP, 0.7475)
        self.assertEqual(g.MIN_ARM_REACH, 0.10)
        self.assertEqual(g.MAX_ARM_REACH, 0.50)

    def test_danger_zone_definition_and_checks(self):
        self.assertEqual(g.LANE_X_MIN, 0.190)
        self.assertEqual(g.LANE_X_MAX, 0.490)
        self.assertEqual(g.LANE_Y_MIN, 0.025)
        self.assertEqual(g.LANE_Y_MAX, 0.175)
        self.assertAlmostEqual(g.DANGER_Z_MAX, g.TOP + 0.120)

        # Stuck block is inside danger zone
        self.assertTrue(g.in_danger_zone(g.BLOCK_INIT_POS))
        # Center of lane is inside danger zone
        self.assertTrue(g.in_danger_zone((0.340, 0.100, g.TOP + 0.050)))

        # Tray, buttons, and high z are NOT inside danger zone
        self.assertFalse(g.in_danger_zone(g.TRAY_POS))
        self.assertFalse(g.in_danger_zone(g.ESTOP_REST_POS))
        self.assertFalse(g.in_danger_zone((0.340, 0.100, g.TOP + 0.150)))  # above danger zone height

    def test_pusher_geometry_and_motion(self):
        self.assertEqual(g.PUSHER_SIZE, (0.030, 0.120, 0.025))
        self.assertEqual(g.PUSHER_STROKE, 0.090)
        self.assertEqual(g.PUSHER_PERIOD, 4.0)
        self.assertAlmostEqual(g.pusher_x_at_time(0.0), 0.240)
        self.assertAlmostEqual(g.pusher_x_at_time(2.0), 0.330)
        self.assertAlmostEqual(g.pusher_x_at_time(4.0), 0.240)

    def test_stuck_block_properties(self):
        self.assertEqual(g.BLOCK_SIZE, (0.040, 0.040, 0.040))
        self.assertLessEqual(g.BLOCK_MASS, 0.050)  # Lesson 3: mass <= 0.05 kg
        self.assertGreaterEqual(g.BLOCK_FRICTION, 1.0)
        self.assertEqual(g.BLOCK_INIT_POS[:2], (0.400, 0.100))

    def test_pushbuttons_spacing_and_travel(self):
        self.assertEqual(g.BUTTON_TRAVEL, 0.008)
        self.assertEqual(g.BUTTON_PRESS_THRESHOLD, -0.0064)
        self.assertAlmostEqual(g.RESET_BTN_POS_XY[0] - g.ESTOP_BTN_POS_XY[0], 0.060)
        self.assertAlmostEqual(g.START_BTN_POS_XY[0] - g.RESET_BTN_POS_XY[0], 0.060)
        self.assertEqual(g.ESTOP_BTN_POS_XY[1], -0.200)
        self.assertEqual(g.RESET_BTN_POS_XY[1], -0.200)
        self.assertEqual(g.START_BTN_POS_XY[1], -0.200)

    def test_tray_and_parked_pad(self):
        self.assertTrue(g.in_tray((g.TRAY_POS[0], g.TRAY_POS[1], g.TOP + 0.020)))
        self.assertFalse(g.in_tray((g.TRAY_POS[0] + 0.100, g.TRAY_POS[1], g.TOP + 0.020)))
        self.assertTrue(g.in_parked_pad((g.PARK_PAD_POS[0], g.PARK_PAD_POS[1], g.TOP + 0.010)))

    def test_bimanual_separation_and_relay_reach(self):
        # Left arm to E-STOP button > 0.50 m
        d_left_to_estop = math.hypot(g.ESTOP_BTN_POS_XY[0] - g.LEFT_BASE[0], g.ESTOP_BTN_POS_XY[1] - g.LEFT_BASE[1])
        self.assertGreater(d_left_to_estop, g.MAX_ARM_REACH)

        # Right arm to stuck block > 0.50 m
        d_right_to_block = math.hypot(g.BLOCK_INIT_POS[0] - g.RIGHT_BASE[0], g.BLOCK_INIT_POS[1] - g.RIGHT_BASE[1])
        self.assertGreater(d_right_to_block, g.MAX_ARM_REACH)

        # Right arm reaches buttons
        for btn_xy in (g.ESTOP_BTN_POS_XY, g.RESET_BTN_POS_XY, g.START_BTN_POS_XY):
            d = math.hypot(btn_xy[0] - g.RIGHT_BASE[0], btn_xy[1] - g.RIGHT_BASE[1])
            self.assertLess(d, g.MAX_ARM_REACH)
            self.assertGreater(d, g.MIN_ARM_REACH)

        # Left arm reaches block and tray
        d_block = math.hypot(g.BLOCK_INIT_POS[0] - g.LEFT_BASE[0], g.BLOCK_INIT_POS[1] - g.LEFT_BASE[1])
        self.assertLess(d_block, g.MAX_ARM_REACH)
        d_tray = math.hypot(g.TRAY_POS[0] - g.LEFT_BASE[0], g.TRAY_POS[1] - g.LEFT_BASE[1])
        self.assertLess(d_tray, g.MAX_ARM_REACH)


    def test_free_body_box_centres(self):
        # Lesson 1: box centres are local to the body origin
        import task
        for (bx, by, bz), sz, col in task.ORANGE_BLOCK.boxes:
            self.assertLessEqual(abs(bx), sz[0] / 2 + 1e-4)
            self.assertLessEqual(abs(by), sz[1] / 2 + 1e-4)
            self.assertLessEqual(abs(bz), sz[2] / 2 + 1e-4)

    def test_local_slider_boxes(self):
        # Lesson 1: verify boxes for fixture sliders are local to body
        import task
        console = task.CONSOLE_FIXTURE
        for slider in console.sliders:
            for (bx, by, bz), sz, col in slider.boxes:
                # Local slider box must be within console footprint
                self.assertLessEqual(abs(bx), g.CONSOLE_BASE_SIZE[0] / 2 + 1e-3)
                self.assertLessEqual(abs(by), g.CONSOLE_BASE_SIZE[1] / 2 + 1e-3)


if __name__ == "__main__":
    unittest.main()
