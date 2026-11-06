"""Unit tests for geometry.py in buffer-socket-module-permutation."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g


class TestGeometry(unittest.TestCase):
    def test_station_and_material_constants(self):
        self.assertEqual(g.TOP, 0.7475)
        self.assertEqual(g.CLEAR, 0.001)
        self.assertEqual(g.MODULE_MASS, 0.080)
        self.assertEqual(g.FIXTURE_MASS, 8.0)
        self.assertEqual(g.MODULE_FRICTION, 0.8)
        self.assertEqual(g.FIXTURE_FRICTION, 1.0)
        self.assertAlmostEqual(g.PLUG_YAW_SEATED, -math.pi / 2)

    def test_sockets_layout_and_pitch(self):
        self.assertEqual(len(g.SOCKET_Y), 4)
        self.assertEqual(g.SOCKET_PITCH, 0.070)
        self.assertEqual(g.MOUTH_X, 0.240)

        # Check exact y coordinates
        expected_y = (-0.260, -0.190, -0.120, -0.050)
        self.assertEqual(g.SOCKET_Y, expected_y)

        # Check pitch spacing
        for i in range(len(g.SOCKET_Y) - 1):
            dy = g.SOCKET_Y[i + 1] - g.SOCKET_Y[i]
            self.assertAlmostEqual(dy, g.SOCKET_PITCH, places=5)

    def test_socket_lamp_discs(self):
        # Sockets 1-3 have lamp discs; socket 4 has no lamp disc
        self.assertEqual(set(g.SOCKET_LAMP_POS.keys()), {0, 1, 2})
        self.assertEqual(set(g.SOCKET_TARGET_COLOURS.keys()), {0, 1, 2})
        self.assertEqual(g.SOCKET_TARGET_COLOURS[0], "red")
        self.assertEqual(g.SOCKET_TARGET_COLOURS[1], "green")
        self.assertEqual(g.SOCKET_TARGET_COLOURS[2], "blue")

        # Check lamp disc colours
        self.assertEqual(g.SOCKET_TARGET_RGB[0], g.BAND_RED)
        self.assertEqual(g.SOCKET_TARGET_RGB[1], g.BAND_GREEN)
        self.assertEqual(g.SOCKET_TARGET_RGB[2], g.BAND_BLUE)

        # Lamp diameter >= 20 mm
        self.assertGreaterEqual(g.SOCKET_LAMP_R * 2, 0.020)

    def test_module_and_cavity_fit(self):
        # 1 mm clearance per side
        self.assertAlmostEqual(g.CAV_HX - g.NOSE_W / 2, g.CLEAR)
        self.assertAlmostEqual(g.CAV_HZ - g.NOSE_H / 2, g.CLEAR)
        self.assertEqual(g.HOUSING, (0.045, 0.018, 0.028))

    def test_controller_box_and_button(self):
        # Controller box coordinates on left side (y > 0)
        self.assertEqual(g.CTRL_POS[0], 0.280)
        self.assertEqual(g.CTRL_POS[1], 0.150)

        # COMMIT button properties
        self.assertEqual(g.BUTTON_TRAVEL, 0.008)
        self.assertEqual(g.BUTTON_RATE, 100.0)
        self.assertEqual(g.BUTTON_MASS, 0.004)
        self.assertEqual(g.BUTTON_CAP_SIZE, (0.036, 0.036, 0.012))

        # Status lamp >= 50 mm diameter
        self.assertGreaterEqual(g.STATUS_LAMP_R * 2, 0.050)

    def test_initial_module_positions(self):
        # Socket 1 (index 0) holds green module
        self.assertEqual(g.START_POS["module_green"], (0.240, -0.260, g.SOCKET_Z))
        # Socket 2 (index 1) holds red module
        self.assertEqual(g.START_POS["module_red"], (0.240, -0.190, g.SOCKET_Z))
        # Socket 3 (index 2) holds blue module
        self.assertEqual(g.START_POS["module_blue"], (0.240, -0.120, g.SOCKET_Z))

    def test_fixture_shapes_generation(self):
        boxes = g.fixture_boxes()
        self.assertGreater(len(boxes), 10)
        for centre, size, col in boxes:
            self.assertEqual(len(centre), 3)
            self.assertEqual(len(size), 3)
            self.assertEqual(len(col), 3)
            self.assertTrue(all(s > 0 for s in size))

        cyls = g.fixture_cylinders()
        self.assertGreaterEqual(len(cyls), 4)  # status lamp dome + 3 socket lamp discs
        for centre, rad, h, col, axis in cyls:
            self.assertEqual(len(centre), 3)
            self.assertGreater(rad, 0)
            self.assertGreater(h, 0)
            self.assertEqual(axis, "Z")


if __name__ == "__main__":
    unittest.main()
