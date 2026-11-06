"""Unit tests for geometry.py: dimensions, fits, tolerances and compliance models."""

import math
import os
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

import geometry as g


class TestGeometry(unittest.TestCase):
    def test_table_height(self):
        self.assertAlmostEqual(g.TOP, 0.7475)

    def test_base_plate_dimensions(self):
        self.assertGreaterEqual(g.BASE_L, 0.300)
        self.assertGreaterEqual(g.BASE_W, 0.400)
        self.assertEqual(g.BASE_H, 0.010)
        self.assertEqual(g.BASE_MASS, 5.0)
        self.assertEqual(g.BASE_FRICTION, 1.0)

    def test_track_dimensions_and_travel(self):
        self.assertEqual(g.KNOB_TRAVEL, 0.100)  # 100 mm travel
        self.assertEqual(g.TRACK_CHANNEL_W, 0.036)  # 36 mm inner channel
        self.assertEqual(g.KNOB_W, 0.030)  # 30 mm knob width
        self.assertAlmostEqual(g.KNOB_LATERAL_CLEARANCE, 0.003)  # 3 mm per side >= 1 mm
        self.assertGreaterEqual(g.KNOB_LATERAL_CLEARANCE, g.CLEAR_MIN)

    def test_rail_limits_along_x(self):
        # Knob center travels from 0.220 to 0.320 m
        cx_0 = g.knob_cx_from_setting_mm(0.0)
        cx_100 = g.knob_cx_from_setting_mm(100.0)
        self.assertAlmostEqual(cx_0, 0.220)
        self.assertAlmostEqual(cx_100, 0.320)
        self.assertAlmostEqual(cx_100 - cx_0, 0.100)

        # Inverse mapping
        self.assertAlmostEqual(g.knob_setting_mm_from_cx(0.220), 0.0)
        self.assertAlmostEqual(g.knob_setting_mm_from_cx(0.320), 100.0)

    def test_start_positions_and_setpoints(self):
        self.assertEqual(g.KNOB_START_SETTING_MM["left"], 30.0)
        self.assertEqual(g.KNOB_START_SETTING_MM["right"], 70.0)
        self.assertEqual(g.SETPOINT_MM["left"], 65.0)
        self.assertEqual(g.SETPOINT_MM["right"], 40.0)
        self.assertEqual(g.TARGET_BAND_TOL_MM, 2.5)

    def test_lock_button_spec(self):
        self.assertEqual(g.BUTTON_CAP_L, 0.036)  # 36 mm
        self.assertEqual(g.BUTTON_CAP_W, 0.036)  # 36 mm
        self.assertEqual(g.BUTTON_CAP_H, 0.012)  # 12 mm
        self.assertEqual(g.BUTTON_TRAVEL, 0.008)  # 8 mm travel
        self.assertEqual(g.BUTTON_RATE, 100.0)  # 100 N/m
        # Pocket ring clearance (inner width 38 mm vs cap 36 mm -> 1.0 mm per side)
        pocket_inner_w = g.BUTTON_BASE_W - 2 * g.POCKET_WALL_T
        cap_clearance = (pocket_inner_w - g.BUTTON_CAP_W) / 2
        self.assertGreaterEqual(cap_clearance, g.CLEAR_MIN)

    def test_button_compliance_model(self):
        travel = g.button_equilibrium_travel()
        self.assertAlmostEqual(travel, g.BUTTON_TRAVEL)  # reaches end stop

    def test_push_force_model(self):
        m = g.push_force_model()
        self.assertGreater(m["normal"], 0.0)
        self.assertGreater(m["horizontal"], 0.0)
        self.assertLess(m["stall_lag_mm"], 5.0)

    def test_fixture_boxes_consistency(self):
        world_boxes = g.fixture_boxes_world()
        local_boxes = g.fixture_boxes()
        self.assertEqual(len(world_boxes), len(local_boxes))
        for (w_c, w_sz, w_col), (l_c, l_sz, l_col) in zip(world_boxes, local_boxes):
            self.assertEqual(w_sz, l_sz)
            self.assertEqual(w_col, l_col)
            # Local pos + base pos should equal world pos
            for i in range(3):
                self.assertAlmostEqual(l_c[i] + g.BASE_POS[i], w_c[i])


if __name__ == "__main__":
    unittest.main()
