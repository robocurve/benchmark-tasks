"""Unit tests for the thermal soak task geometry: dimensions, clearances, travel limits, spring forces, and thermal constants."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g


class TestGeometry(unittest.TestCase):
    def test_table_height(self):
        self.assertAlmostEqual(g.TOP, 0.7475, places=4)

    def test_fixture_dimensions(self):
        self.assertEqual(g.FIXTURE_SIZE, (0.160, 0.160, 0.030))
        self.assertGreaterEqual(g.FIXTURE_MASS, 5.0)
        self.assertEqual(g.FIXTURE_FRICTION, 1.0)
        self.assertAlmostEqual(g.FIXTURE_TOP_Z, g.TOP + 0.030, places=4)

    def test_sample_pad_dimensions(self):
        self.assertEqual(g.PAD_SIZE[:2], (0.100, 0.100))
        self.assertAlmostEqual(g.PAD_TOP_Z, g.FIXTURE_TOP_Z + g.PAD_SIZE[2], places=4)

    def test_button_specifications(self):
        # 70 mm pitch along y
        pitch = g.HEAT_BTN_POS[1] - g.OFF_BTN_POS[1]
        self.assertAlmostEqual(pitch, 0.070, places=4)

        # Cap dimensions and mass
        self.assertEqual(g.BUTTON_CAP_SIZE, (0.036, 0.036, 0.012))
        self.assertAlmostEqual(g.BUTTON_MASS, 0.004, places=4)  # 4-5 g mass
        self.assertEqual(g.BUTTON_TRAVEL, 0.008)  # 8 mm travel
        self.assertEqual(g.BUTTON_RATE, 100.0)    # 100 N/m spring
        self.assertEqual(g.BUTTON_DAMPING, 4.0)

        # Force at stop: 0.8 N
        self.assertAlmostEqual(g.BUTTON_RATE * g.BUTTON_TRAVEL, 0.8, places=4)

        # Thresholds
        self.assertAlmostEqual(g.BUTTON_TRIGGER_Z, -0.0064, places=4)  # <= -6.4 mm
        self.assertAlmostEqual(g.BUTTON_REARM_Z, -0.0035, places=4)    # >= -3.5 mm

        # Sag on spring: m * g / k
        sag = g.BUTTON_MASS * 9.81 / g.BUTTON_RATE  # ~0.39 mm
        self.assertLess(sag, abs(g.BUTTON_REARM_Z))  # re-arm level is well above resting sag

    def test_sample_blocks_and_clearances(self):
        self.assertEqual(g.SAMPLE_SIZE, (0.060, 0.060, 0.040))
        self.assertEqual(g.SAMPLE_MASS, 0.300)
        self.assertEqual(g.SAMPLE_FRICTION, 0.8)

        # Clearance in 94 mm open jaw: 17 mm per side
        clearance = (g.FINGER_OPEN_M - g.SAMPLE_SIZE[0]) / 2.0
        self.assertAlmostEqual(clearance, 0.017, places=4)

        # Staging pick targets spacing >= 120 mm
        staging_dist = math.dist(g.START_POS_1, g.START_POS_2)
        self.assertGreaterEqual(staging_dist, 0.120)

        # Cool pad placement spots spacing >= 120 mm
        cool_dist = math.dist(g.COOL_SPOT_1, g.COOL_SPOT_2)
        self.assertGreaterEqual(cool_dist, 0.120)

        # Fixture to staging clearance >= 120 mm between centres
        fix_to_stage = math.dist(g.FIXTURE_POS[:2], g.START_POS_1[:2])
        self.assertGreaterEqual(fix_to_stage, 0.120)

    def test_indicator_specifications(self):
        # Status lamp diameter >= 50 mm
        self.assertGreaterEqual(g.STATUS_LAMP_R * 2.0, 0.050)

        # 9 temperature bar discs >= 28 mm diameter
        self.assertEqual(len(g.BAR_DISC_X), 9)
        self.assertGreaterEqual(g.BAR_LAMP_R * 2.0, 0.028)
        self.assertEqual(len(g.BAR_DISC_TEMP), 9)
        self.assertEqual(g.BAR_DISC_TEMP, (30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0))

    def test_thermal_constants_recorded(self):
        self.assertEqual(g.PLATE_TEMP_MIN, 20.0)
        self.assertEqual(g.PLATE_TEMP_MAX, 110.0)
        self.assertEqual(g.PLATE_TEMP_SETPOINT, 110.0)
        self.assertEqual(g.PLATE_HEAT_RATE, 10.0)
        self.assertEqual(g.PLATE_COOL_RATE, 1.5)
        self.assertEqual(g.TAU_SURF_HEAT, 10.0)
        self.assertEqual(g.TAU_CORE, 18.0)
        self.assertEqual(g.TAU_COOL, 50.0)
        self.assertEqual(g.CORE_FIRE_LIMIT, 90.0)
        self.assertEqual(g.CORE_BAND_MIN, 60.0)
        self.assertEqual(g.CORE_BAND_MAX, 70.0)
        self.assertEqual(g.DEMO_NOMINAL_DWELL, 14.0)

    def test_fixture_boxes_and_cylinders(self):
        fb = g.fixture_boxes()
        self.assertEqual(len(fb), 5)
        for c, s, col in fb:
            self.assertEqual(len(c), 3)
            self.assertEqual(len(s), 3)
            self.assertEqual(len(col), 3)

        fc = g.fixture_cylinders()
        self.assertEqual(len(fc), 10)  # 1 status lamp + 9 bar discs
        for c, r, h, col, ax in fc:
            self.assertEqual(len(c), 3)
            self.assertGreater(r, 0)
            self.assertGreater(h, 0)
            self.assertEqual(len(col), 3)
            self.assertEqual(ax, "Z")


if __name__ == "__main__":
    unittest.main()
