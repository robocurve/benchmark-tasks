"""Unit tests for Task 4 geometry: clearances, dimensions, layout, detents, and camera visibility."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent.parent
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
sys.path.insert(0, str(HERE))

import geometry as g


def ray_intersects_box(origin, target, box_min, box_max) -> bool:
    """True if the ray segment from origin to target intersects the axis-aligned box."""
    d = target - origin
    tmin, tmax = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < 1e-9:
            if origin[i] < box_min[i] or origin[i] > box_max[i]:
                return False
        else:
            t1 = (box_min[i] - origin[i]) / d[i]
            t2 = (box_max[i] - origin[i]) / d[i]
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return False
    return True


class TestTask4Geometry(unittest.TestCase):
    def test_table_height_and_masses(self) -> None:
        self.assertAlmostEqual(g.TOP, 0.7475)
        self.assertEqual(g.FIXTURE_MASS, 8.0)
        self.assertEqual(g.FIXTURE_FRICTION, 1.0)
        self.assertEqual(g.SERVO_MASS, 0.080)
        self.assertEqual(g.SERVO_FRICTION, 0.8)

    def test_four_sockets_layout(self) -> None:
        self.assertEqual(len(g.SOCKET_Y), 4)
        self.assertEqual(g.SOCKET_PITCH, 0.070)
        for i in range(len(g.SOCKET_Y) - 1):
            pitch = abs(g.SOCKET_Y[i] - g.SOCKET_Y[i + 1])
            self.assertAlmostEqual(pitch, 0.070, places=4)
        self.assertEqual(g.SOCKET_Y, (-0.050, -0.120, -0.190, -0.260))
        self.assertEqual(g.MOUTH_X, 0.240)
        self.assertEqual(g.SOCKET_Z, g.TOP + 0.025)

    def test_servo_nose_and_cavity_clearances(self) -> None:
        self.assertEqual(g.CLEAR, 0.001)
        # Lateral half-width clearance
        self.assertAlmostEqual(g.CAV_HX - g.NOSE_W / 2, g.CLEAR, places=4)
        # Vertical half-height clearance
        self.assertAlmostEqual(g.CAV_HZ - g.NOSE_H / 2, g.CLEAR, places=4)
        # Key slot clearance
        self.assertAlmostEqual((g.SLOT_X1 - g.SLOT_X0 - g.KEY_W) / 2, g.CLEAR, places=4)

    def test_plungers_specifications(self) -> None:
        self.assertEqual(g.PLUNGER_TRAVEL, 0.0015)
        self.assertEqual(g.PLUNGER_RATE, 12000.0)
        self.assertEqual(g.PLUNGER_DAMPING, 40.0)
        self.assertEqual(g.PLUNGER_MASS, 0.020)
        self.assertAlmostEqual(g.PLUNGER_PROUD, g.CLEAR + 0.0005, places=4)

    def test_diag_controller_box_and_button(self) -> None:
        self.assertEqual(g.BUTTON_TRAVEL, 0.008)
        self.assertEqual(g.BUTTON_RATE, 100.0)
        self.assertEqual(g.BUTTON_MASS, 0.004)
        self.assertEqual(g.BUTTON_CAP_SIZE, (0.036, 0.036, 0.012))
        self.assertAlmostEqual(g.BUTTON_REST_TOP_Z, g.TOP + 0.060, places=4)
        self.assertAlmostEqual(g.BUTTON_PRESS_THRESHOLD, 0.0064, places=4)
        self.assertAlmostEqual(g.BUTTON_RELEASE_THRESHOLD, 0.0035, places=4)

    def test_status_lamp_dimensions_and_position(self) -> None:
        # Lamp must have diameter >= 60 mm (radius >= 30 mm)
        self.assertGreaterEqual(g.STATUS_LAMP_R * 2, 0.060)
        self.assertEqual(g.STATUS_LAMP_R, 0.030)
        self.assertEqual(g.STATUS_LAMP_H, 0.015)
        # Sits on controller box top
        ctrl_top = g.CTRL_POS[2] + g.CTRL_SIZE[2] / 2
        lamp_bottom = g.STATUS_LAMP_POS[2] - g.STATUS_LAMP_H / 2
        self.assertAlmostEqual(lamp_bottom, ctrl_top, places=4)

    def test_camera_visibility_ray_test(self) -> None:
        """Top camera (-0.19, 0, 1.18) must have unoccluded sightline to status lamp."""
        cam = np.array([-0.19, 0.0, 1.18])
        lamp = np.array(g.STATUS_LAMP_POS)

        # Button cap bounding box
        btn_c = np.array(g.DIAG_BTN_POS)
        btn_s = np.array(g.BUTTON_CAP_SIZE)
        btn_min = btn_c - btn_s / 2
        btn_max = btn_c + btn_s / 2
        btn_max[2] += 0.006

        hit = ray_intersects_box(cam, lamp, btn_min, btn_max)
        self.assertFalse(hit, "Status lamp occluded by DIAG button cap")

    def test_spare_module_staging_spot(self) -> None:
        self.assertEqual(g.SPARE_STAGE_X, 0.380)
        self.assertEqual(g.SPARE_STAGE_Y, -0.150)
        self.assertAlmostEqual(g.SPARE_STAGE_Z, g.TOP + g.HOUSING[2] / 2, places=4)

    def test_socket_dot_markers_count(self) -> None:
        """Check that fixture boxes contain exactly 1, 2, 3, 4 white dots for sockets 1..4."""
        fb = g.fixture_boxes()
        white_dots = [b for b in fb if b[2] == g.WHITE and b[1] == (0.004, 0.004, 0.001)]
        # Total dots: 1 + 2 + 3 + 4 = 10 dots
        self.assertEqual(len(white_dots), 10)


if __name__ == "__main__":
    unittest.main()
