"""Unit tests for Task 1 geometry: clearances, dimensions, layout, reach constraints, and camera visibility."""

from __future__ import annotations

import math
import unittest
import numpy as np

import geometry as g


class TestTask1Geometry(unittest.TestCase):
    def test_fixture_base_and_cold_plate_dimensions(self) -> None:
        self.assertEqual(g.BASE_SIZE, (0.200, 0.260, 0.015))
        self.assertEqual(g.COLD_PLATE_SIZE, (0.140, 0.200, 0.010))
        self.assertGreaterEqual(g.FIXTURE_MASS, 5.0)
        self.assertEqual(g.FIXTURE_MASS, 6.0)
        self.assertEqual(g.FIXTURE_FRICTION, 1.0)
        self.assertAlmostEqual(g.BASE_POS[0], 0.280)
        self.assertAlmostEqual(g.BASE_POS[1], 0.0)

    def test_cold_plate_elevation(self) -> None:
        expected_top = g.TOP + g.BASE_SIZE[2] + g.COLD_PLATE_SIZE[2]
        self.assertAlmostEqual(g.COLD_PLATE_TOP_Z, expected_top)
        self.assertAlmostEqual(g.COLD_PLATE_TOP_Z, g.TOP + 0.025)

    def test_bolt_corner_spacing_and_positions(self) -> None:
        self.assertEqual(len(g.CORNERS), 4)
        dx = g.BOLT_X_FAR - g.BOLT_X_NEAR
        self.assertAlmostEqual(dx, 0.110)
        self.assertGreaterEqual(dx, 0.100)
        self.assertLessEqual(dx, 0.120)

        dy = g.BOLT_Y_LEFT - g.BOLT_Y_RIGHT
        self.assertAlmostEqual(dy, 0.210)
        self.assertGreaterEqual(dy, 0.200)
        self.assertLessEqual(dy, 0.220)

        center_x = (g.BOLT_X_NEAR + g.BOLT_X_FAR) / 2
        self.assertAlmostEqual(center_x, 0.280)
        center_y = (g.BOLT_Y_LEFT + g.BOLT_Y_RIGHT) / 2
        self.assertAlmostEqual(center_y, 0.0)

    def test_diagonal_partner_pairs(self) -> None:
        self.assertEqual(g.DIAGONAL_PARTNER["L_near"], "R_far")
        self.assertEqual(g.DIAGONAL_PARTNER["R_far"], "L_near")
        self.assertEqual(g.DIAGONAL_PARTNER["L_far"], "R_near")
        self.assertEqual(g.DIAGONAL_PARTNER["R_near"], "L_far")
        for c in g.CORNERS:
            partner = g.DIAGONAL_PARTNER[c]
            self.assertNotEqual(c[0], partner[0], "Diagonal pair must be 1 left and 1 right")
            self.assertEqual(g.DIAGONAL_PARTNER[partner], c)

    def test_two_arm_reach_requirement(self) -> None:
        """Left arm cannot reach right-far bolt (> 0.50 m), right arm cannot reach left-far bolt (> 0.50 m)."""
        l_base = (0.0, 0.305)
        r_base = (0.0, -0.305)

        r_far_pos = g.BOLT_POS_XY["R_far"]
        dist_l_to_rfar = math.hypot(r_far_pos[0] - l_base[0], r_far_pos[1] - l_base[1])
        self.assertGreater(dist_l_to_rfar, 0.50, "Distance from left shoulder to right-far bolt must exceed 0.50 m")

        l_far_pos = g.BOLT_POS_XY["L_far"]
        dist_r_to_lfar = math.hypot(l_far_pos[0] - r_base[0], l_far_pos[1] - r_base[1])
        self.assertGreater(dist_r_to_lfar, 0.50, "Distance from right shoulder to left-far bolt must exceed 0.50 m")

        # Each arm reaches its own bolts within 0.50 m
        for c in ("L_near", "L_far"):
            pos = g.BOLT_POS_XY[c]
            dist = math.hypot(pos[0] - l_base[0], pos[1] - l_base[1])
            self.assertLess(dist, 0.50)

        for c in ("R_near", "R_far"):
            pos = g.BOLT_POS_XY[c]
            dist = math.hypot(pos[0] - r_base[0], pos[1] - r_base[1])
            self.assertLess(dist, 0.50)

    def test_button_spring_and_travel(self) -> None:
        self.assertEqual(g.BUTTON_TRAVEL, 0.008)
        self.assertEqual(g.BUTTON_RATE, 100.0)
        self.assertEqual(g.BUTTON_MASS, 0.004)
        # Cap sag on 100 N/m spring: m*g/k = 0.004 * 9.81 / 100 = 0.39 mm < 0.5 mm
        sag_mm = (g.BUTTON_MASS * 9.81 / g.BUTTON_RATE) * 1000
        self.assertLess(sag_mm, 0.5)

        # Force at hard stop
        force_at_stop = g.BUTTON_RATE * g.BUTTON_TRAVEL
        self.assertAlmostEqual(force_at_stop, 0.8)

        # Commanded press depth exceeds stop travel
        self.assertGreater(g.BUTTON_PRESS_DEPTH_CMD, g.BUTTON_TRAVEL)
        self.assertAlmostEqual(g.BUTTON_PRESS_DEPTH_CMD - g.BUTTON_TRAVEL, 0.006)

    def test_cyan_stage_lamps(self) -> None:
        self.assertGreaterEqual(g.STAGE_LAMP_R * 2, 0.028)
        self.assertEqual(len(g.STAGE_LAMP_POS), 12)
        for (corner, stage), pos in g.STAGE_LAMP_POS.items():
            self.assertIn(corner, g.CORNERS)
            self.assertIn(stage, (1, 2, 3))
            # Clearance to bolt center
            bolt_pos = g.BOLT_POS_XY[corner]
            dist_to_bolt = math.hypot(pos[0] - bolt_pos[0], pos[1] - bolt_pos[1])
            # Cap half-width 18 mm + lamp radius 14 mm = 32 mm
            self.assertGreaterEqual(dist_to_bolt, 0.032 + 0.010, "Lamp disc must clear button cap by >= 10 mm")

    def test_status_lamp(self) -> None:
        self.assertGreaterEqual(g.STATUS_LAMP_R * 2, 0.050)
        self.assertAlmostEqual(g.STATUS_LAMP_POS[0], 0.280)
        self.assertAlmostEqual(g.STATUS_LAMP_POS[1], 0.0)
        for corner, (bx, by) in g.BOLT_POS_XY.items():
            dist = math.hypot(g.STATUS_LAMP_POS[0] - bx, g.STATUS_LAMP_POS[1] - by)
            self.assertGreater(dist, 0.100, "Status lamp must be >= 100 mm from bolt centers")

    def test_arm_park_separation(self) -> None:
        dist = math.hypot(g.HOME_L[0] - g.HOME_R[0], g.HOME_L[1] - g.HOME_R[1])
        self.assertGreaterEqual(dist, 0.100)
        self.assertAlmostEqual(dist, 0.440)


if __name__ == "__main__":
    unittest.main()
