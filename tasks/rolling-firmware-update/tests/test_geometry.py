"""Tests for rolling-firmware-update geometry and clearances."""

import unittest
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g


class TestGeometry(unittest.TestCase):
    def test_fixture_dimensions_and_mass(self):
        self.assertGreaterEqual(g.BASE_MASS, 5.0, "Base fixture must be heavy to prevent movement")
        self.assertEqual(g.BASE_MASS, 8.0)
        self.assertGreater(g.BASE_LX, 0.200)
        self.assertGreater(g.BASE_LY, 0.250)
        self.assertGreater(g.BASE_LZ, 0.010)

    def test_button_pocket_clearances(self):
        """Pocket inner width must have >= 1.0 mm clearance per side around the 38 mm cap."""
        clearance_x = (g.POCKET_INNER_LX - g.BUTTON_CAP_LX) / 2
        clearance_y = (g.POCKET_INNER_LY - g.BUTTON_CAP_LY) / 2
        self.assertGreaterEqual(clearance_x, g.CLEAR_MIN, f"Clearance in x {clearance_x*1000:.2f} mm < 1 mm")
        self.assertGreaterEqual(clearance_y, g.CLEAR_MIN, f"Clearance in y {clearance_y*1000:.2f} mm < 1 mm")
        self.assertAlmostEqual(clearance_x, 0.002, delta=0.0005)
        self.assertAlmostEqual(clearance_y, 0.005, delta=0.0005)

    def test_button_travel_and_springs(self):
        self.assertAlmostEqual(g.BUTTON_TRAVEL, 0.008, delta=1e-5)
        self.assertAlmostEqual(g.BUTTON_SPRING_RATE, 100.0, delta=1e-5)
        self.assertAlmostEqual(g.BUTTON_CAP_MASS, 0.004, delta=1e-5)
        self.assertLess(g.BUTTON_PRESS_THRESHOLD, -0.006)
        self.assertGreater(g.BUTTON_REARM_THRESHOLD, -0.004)

    def test_service_door_specs(self):
        self.assertAlmostEqual(g.DOOR_TRAVEL, 0.025, delta=1e-5)
        self.assertAlmostEqual(g.DOOR_SPRING_RATE, 80.0, delta=1e-5)
        # 80 N/m * 0.025 m = 2.0 N
        self.assertAlmostEqual(g.DOOR_SPRING_RATE * g.DOOR_TRAVEL, 2.0, delta=0.1)
        self.assertAlmostEqual(g.DOOR_DEFLECTION_THRESHOLD, 0.018, delta=1e-5)
        self.assertAlmostEqual(g.DOOR_RELEASED_THRESHOLD, 0.002, delta=1e-5)

    def test_lamp_sizes(self):
        # ACTIVE lamp >= 40 mm diameter
        self.assertGreaterEqual(g.ACTIVE_LAMP_R * 2, 0.040)
        # VERSION lamp >= 40 mm diameter
        self.assertGreaterEqual(g.VERSION_LAMP_R * 2, 0.040)
        # 4-lamp progress bar >= 28 mm diameter
        self.assertGreaterEqual(g.PROGRESS_LAMP_R * 2, 0.028)
        # LOAD lamp >= 60 mm diameter
        self.assertGreaterEqual(g.LOAD_LAMP_R * 2, 0.060)

    def test_bimanual_reach_and_interlock_distance(self):
        """Distance between door handle and button on each unit must be >= 100 mm."""
        for u in ("A", "B"):
            hx = g.DOOR_HANDLE_REST_CX
            hy = g.DOOR_HANDLE_REST_CY[u]
            bx = g.UPDATE_BTN_CX
            by = g.UPDATE_BTN_CY[u]
            dist_rest = np.hypot(bx - hx, by - hy)
            self.assertGreaterEqual(dist_rest, 0.100, f"Unit {u} rest distance {dist_rest*1000:.1f} mm < 100 mm")

            # During hold: handle is at -25 mm along x (pushed to 0.200)
            hx_held = hx - g.DOOR_TRAVEL
            dist_held = np.hypot(bx - hx_held, by - hy)
            self.assertGreaterEqual(dist_held, 0.100, f"Unit {u} held distance {dist_held*1000:.1f} mm < 100 mm")

    def test_stowed_z_under_table(self):
        self.assertLess(g.STOWED_Z, g.TOP - 0.05)


if __name__ == "__main__":
    unittest.main()
