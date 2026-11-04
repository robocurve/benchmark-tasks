"""Unit tests for Task G geometry: dimensions, clearances, travel limits, and spring forces."""

from __future__ import annotations

import unittest
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g


class TestTaskGGeometry(unittest.TestCase):
    def test_base_plate_dimensions(self):
        self.assertEqual(g.BASE_SIZE, (0.300, 0.200, 0.010))
        self.assertEqual(g.BASE_MASS, 5.0)
        self.assertEqual(g.BASE_FRICTION, 1.0)

    def test_track_dimensions_and_spacing(self):
        self.assertEqual(g.TRACK_L, 0.140)
        self.assertEqual(g.TRACK_CHANNEL_W, 0.036)
        self.assertEqual(g.TRACK_WALL_T, 0.006)
        self.assertEqual(g.TRACK_WALL_H, 0.020)
        self.assertEqual(g.TRACK_OUTER_W, 0.048)

        # 120 mm apart between track centerlines
        spacing = g.TRACK_Y["left"] - g.TRACK_Y["right"]
        self.assertAlmostEqual(spacing, 0.120, places=5)

    def test_knob_dimensions_and_clearances(self):
        self.assertEqual(g.KNOB_SIZE, (0.030, 0.030, 0.014))
        self.assertEqual(g.KNOB_MASS, 0.080)
        self.assertEqual(g.KNOB_FRICTION, 0.5)

        # Clearance inside 36 mm channel: 3.0 mm per side (>= 1.0 mm rule)
        self.assertGreaterEqual(g.KNOB_LATERAL_CLEARANCE, 0.001)
        self.assertAlmostEqual(g.KNOB_LATERAL_CLEARANCE, 0.003, places=5)

        # Travel along track: 100 mm
        self.assertEqual(g.KNOB_TRAVEL, 0.100)
        # Knob travel fits inside inner channel length (130 mm - 30 mm = 100 mm)
        self.assertEqual(g.TRACK_INNER_L - g.KNOB_L, g.KNOB_TRAVEL)

        # Start positions are at setting 30 mm (>= 25 mm gap for 15.4 mm closed fingers)
        self.assertAlmostEqual(g.knob_setting_mm_from_cx(g.START_POS["left"][0]), 30.0, places=4)
        self.assertAlmostEqual(g.knob_setting_mm_from_cx(g.START_POS["right"][0]), 30.0, places=4)

        # Eliminates 2 mm floor penetration at spawn: knob bottom rests exactly on floor top
        knob_bottom = g.KNOB_CZ - g.KNOB_H / 2
        floor_top = g.BASE_TOP_Z + g.TRACK_FLOOR_T
        self.assertAlmostEqual(knob_bottom, floor_top, places=5)

    def test_meter_and_scale_dimensions(self):
        self.assertEqual(g.METER_SIZE, (0.040, 0.200, 0.030))
        self.assertEqual(g.SCALE_W, 0.160)
        self.assertEqual(g.SCALE_L, 0.024)
        self.assertEqual(g.SCALE_H, 0.004)

        # Green band is over last 8 mm (152 to 160 mm)
        self.assertAlmostEqual(g.SCALE_WHITE_LEN, 0.152, places=5)
        self.assertAlmostEqual(g.SCALE_GREEN_LEN, 0.008, places=5)
        self.assertAlmostEqual(g.SCALE_WHITE_LEN + g.SCALE_GREEN_LEN, g.SCALE_W, places=5)

    def test_needle_dimensions_and_travel(self):
        self.assertEqual(g.NEEDLE_SIZE, (0.034, 0.012, 0.008))
        self.assertEqual(g.NEEDLE_MASS, 0.010)
        self.assertEqual(g.NEEDLE_MAX_TRAVEL, 0.160)

        # At r = 0, needle is at zero mark
        p0 = g.needle_pos_from_r(0.0)
        self.assertAlmostEqual(p0[1], g.SCALE_ORIGIN_Y, places=5)

        # At r = 0.95 (threshold), needle center reaches green band left edge exactly
        p095 = g.needle_pos_from_r(0.95)
        green_start = g.SCALE_ORIGIN_Y + g.SCALE_WHITE_LEN
        self.assertAlmostEqual(p095[1], green_start, places=5)

        # At r = 1.0, needle center reaches scale bar right edge
        p1 = g.needle_pos_from_r(1.0)
        scale_end = g.SCALE_ORIGIN_Y + g.SCALE_W
        self.assertAlmostEqual(p1[1], scale_end, places=5)

    def test_read_button_dynamics(self):
        self.assertEqual(g.BUTTON_CAP_SIZE, (0.040, 0.040, 0.012))
        self.assertEqual(g.BUTTON_MASS, 0.020)
        self.assertEqual(g.BUTTON_TRAVEL, 0.008)
        self.assertEqual(g.BUTTON_RATE, 100.0)
        self.assertEqual(g.BUTTON_DAMPING, 4.0)
        self.assertAlmostEqual(g.BUTTON_RATE * g.BUTTON_TRAVEL, 0.8, places=6)  # 0.8 N at full travel
        self.assertLess((g.BUTTON_RATE / g.BUTTON_MASS) ** 0.5, 400.0)  # no ringing above the physics rate

    def test_fixture_boxes_validity(self):
        boxes = g.fixture_boxes()
        self.assertGreater(len(boxes), 5)
        for centre, size, color in boxes:
            self.assertEqual(len(centre), 3)
            self.assertEqual(len(size), 3)
            self.assertEqual(len(color), 3)
            self.assertTrue(all(s > 0 for s in size), f"box {centre} has non-positive size {size}")
            self.assertTrue(all(0.0 <= c <= 1.0 for c in color), f"box {centre} has invalid color {color}")


if __name__ == "__main__":
    unittest.main()
