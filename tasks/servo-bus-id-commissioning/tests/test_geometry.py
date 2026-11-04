"""Unit tests for Task F geometry: clearances, dimensions, layout, detents, and camera visibility."""

from __future__ import annotations

import unittest
import numpy as np

import geometry as g


def ray_intersects_box(p0: np.ndarray, p1: np.ndarray, bmin: np.ndarray, bmax: np.ndarray) -> bool:
    """True if segment p0->p1 intersects axis-aligned box [bmin, bmax]."""
    d = p1 - p0
    tmin, tmax = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < 1e-12:
            if p0[i] < bmin[i] or p0[i] > bmax[i]:
                return False
        else:
            t1 = (bmin[i] - p0[i]) / d[i]
            t2 = (bmax[i] - p0[i]) / d[i]
            tmin = max(tmin, min(t1, t2))
            tmax = min(tmax, max(t1, t2))
    return tmin <= tmax


class TestTaskFGeometry(unittest.TestCase):
    def test_servo_module_dimensions(self) -> None:
        self.assertEqual(g.NOSE_W, 0.028)
        self.assertEqual(g.NOSE_H, 0.016)
        self.assertEqual(g.NOSE_L, 0.018)
        self.assertEqual(g.HOUSING, (0.045, 0.018, 0.028))
        self.assertEqual(g.SERVO_MASS, 0.080)
        self.assertEqual(g.CLEAR, 0.001)

    def test_socket_cavity_clearances(self) -> None:
        cavity_width = 2 * g.CAV_HX
        self.assertAlmostEqual(cavity_width - g.NOSE_W, 2 * g.CLEAR)
        self.assertAlmostEqual(cavity_width, 0.030)  # 30 mm cavity for 28 mm nose

        cavity_height = 2 * g.CAV_HZ
        self.assertAlmostEqual(cavity_height - g.NOSE_H, 2 * g.CLEAR)
        self.assertAlmostEqual(cavity_height, 0.018)  # 18 mm cavity for 16 mm nose

    def test_key_slot_clearance(self) -> None:
        slot_w = g.KEY_W + 2 * g.CLEAR
        self.assertAlmostEqual(slot_w - g.KEY_W, 2 * g.CLEAR)
        self.assertAlmostEqual(slot_w, 0.008)  # 8 mm slot for 6 mm key rib

    def test_socket_pitch(self) -> None:
        self.assertEqual(len(g.SOCKET_Y), 3)
        self.assertAlmostEqual(abs(g.SOCKET_Y[0] - g.SOCKET_Y[1]), 0.070)
        self.assertAlmostEqual(abs(g.SOCKET_Y[1] - g.SOCKET_Y[2]), 0.070)

    def test_button_travel_and_force(self) -> None:
        self.assertEqual(g.BUTTON_TRAVEL, 0.008)
        # soft cap spring: F = k * travel = 100 * 0.008 = 0.8 N at the end stop (see test_button_press.py)
        force = g.BUTTON_RATE * g.BUTTON_TRAVEL
        self.assertAlmostEqual(force, 0.8)

    def test_button_spacing(self) -> None:
        dist = np.linalg.norm(np.array(g.ASSIGN_BTN_POS) - np.array(g.RESET_BTN_POS))
        self.assertGreaterEqual(dist, 0.050)  # >= 50 mm apart

    def test_fixture_boxes_validity(self) -> None:
        boxes = g.fixture_boxes()
        self.assertGreater(len(boxes), 10)
        for centre, size, colour in boxes:
            self.assertEqual(len(centre), 3)
            self.assertEqual(len(size), 3)
            self.assertEqual(len(colour), 3)
            self.assertTrue(all(s > 0 for s in size))
            self.assertTrue(all(0.0 <= c <= 1.0 for c in colour))

    def test_servo_boxes_validity(self) -> None:
        for col in (g.RED, g.GREEN, g.BLUE):
            boxes = g.servo_boxes(col)
            self.assertGreater(len(boxes), 5)
            for centre, size, colour in boxes:
                self.assertEqual(len(centre), 3)
                self.assertEqual(len(size), 3)
                self.assertEqual(len(colour), 3)
                self.assertTrue(all(s > 0 for s in size))
                self.assertTrue(all(0.0 <= c <= 1.0 for c in colour))

    def test_plunger_in_groove_alignment(self) -> None:
        """Verify plunger sits on +y wall and tip overlaps nose detent groove in x and y (F2)."""
        for sy in g.SOCKET_Y:
            # Plunger on +y wall
            plunger_x = g.MOUTH_X + g.PLUNGER_S
            plunger_y = sy + g.CAV_HX + g.PLUNGER_R - g.PLUNGER_PROUD
            plunger_tip_inner_y = plunger_y - g.PLUNGER_R  # edge extending into cavity

            # Seated servo at this socket (yaw -pi/2: plug x -> world -y, plug y -> world x)
            # Groove in plug frame: x in [-0.014, -0.0125], y in [0.0075, 0.0125]
            # In world frame:
            # groove_x = MOUTH_X + [0.0075, 0.0125] = [0.2475, 0.2525]
            # groove_y = sy - [-0.014, -0.0125] = [sy + 0.0125, sy + 0.0140]
            groove_x_min = g.MOUTH_X + g.GROOVE[0]
            groove_x_max = g.MOUTH_X + g.GROOVE[1]
            groove_y_min = sy + (g.NOSE_W / 2 - g.GROOVE_DEPTH)
            groove_y_max = sy + g.NOSE_W / 2

            # Plunger cylinder spans x:
            p_x_min = plunger_x - g.PLUNGER_R
            p_x_max = plunger_x + g.PLUNGER_R

            # Assert x overlap
            self.assertGreaterEqual(p_x_min, groove_x_min - 1e-4)
            self.assertLessEqual(p_x_max, groove_x_max + 1e-4)

            # Assert tip extends into groove depth
            # Plunger tip inner y must be inside groove depth: [groove_y_min, groove_y_max]
            self.assertGreater(plunger_tip_inner_y, groove_y_min)
            self.assertLess(plunger_tip_inner_y, groove_y_max)

    def test_world_key_slot_clearance(self) -> None:
        """Verify key slot in fixture_boxes matches key rib with >= 1.0 mm clearance per side (F3)."""
        boxes = g.fixture_boxes()
        for sy in g.SOCKET_Y:
            # Key rib in world coordinates when seated (yaw = -pi/2):
            # Key rib has width KEY_W (0.006) and centre at plug x = KEY_X (+0.001)
            # World y of key rib centre = sy - KEY_X
            key_cy = sy - g.KEY_X
            key_y0 = key_cy - g.KEY_W / 2
            key_y1 = key_cy + g.KEY_W / 2

            # Find the top wall left, right, and cap boxes for this socket
            # Socket mouth x = MOUTH_X, shell length = SHELL_L
            shell_cx = g.MOUTH_X + g.SHELL_L / 2
            top_wall_boxes = [
                b for b in boxes
                if abs(b[0][0] - shell_cx) < 1e-4 and abs(b[0][2] - (g.SOCKET_Z + g.CAV_HZ + (g.SHELL_HZ - g.CAV_HZ) / 2)) < 1e-4
            ]
            self.assertGreaterEqual(len(top_wall_boxes), 2)

            # Slot boundaries formed by top wall boxes around this socket
            # Left wall box is at lower y, right wall box at higher y
            y_left_wall_edge = max(b[0][1] + b[1][1] / 2 for b in top_wall_boxes if b[0][1] < sy)
            y_right_wall_edge = min(b[0][1] - b[1][1] / 2 for b in top_wall_boxes if b[0][1] > sy)

            clearance_left = key_y0 - y_left_wall_edge
            clearance_right = y_right_wall_edge - key_y1

            self.assertAlmostEqual(clearance_left, g.CLEAR, places=4, msg="Left clearance must be >= 1.0 mm")
            self.assertAlmostEqual(clearance_right, g.CLEAR, places=4, msg="Right clearance must be >= 1.0 mm")

    def test_readout_camera_visibility_ray_test(self) -> None:
        """Ray test verifying top camera (-0.19, 0, 1.18) has unoccluded sightline to all readout lamps (F11)."""
        cam = np.array([-0.19, 0.0, 1.18])

        # Controller box bounding box
        ctrl_min = np.array(g.CTRL_POS) - np.array(g.CTRL_SIZE) / 2
        ctrl_max = np.array(g.CTRL_POS) + np.array(g.CTRL_SIZE) / 2
        # Include raised button caps
        ctrl_max[2] = g.BUTTON_REST_Z + g.BUTTON_CAP_SIZE[2] / 2

        # Check every readout lamp (centre + 8 sample points around dome radius)
        for row_name, rx in g.READOUT_ROWS.items():
            for col_idx, cy in g.READOUT_COLS.items():
                lamp_z = g.READOUT_LAMP_Z
                # Sample points: centre and perimeter
                angles = np.linspace(0, 2 * np.pi, 8, endpoint=False)
                sample_pts = [np.array([rx, cy, lamp_z])]
                for th in angles:
                    sample_pts.append(np.array([
                        rx + g.READOUT_LAMP_R * np.cos(th),
                        cy + g.READOUT_LAMP_R * np.sin(th),
                        lamp_z,
                    ]))

                for pt in sample_pts:
                    hit = ray_intersects_box(cam, pt, ctrl_min, ctrl_max)
                    self.assertFalse(hit, f"Lamp at row {row_name} col {col_idx} occluded by controller box")

    def test_status_lamp_on_controller_box(self) -> None:
        """Verify status lamp sits on controller box top and not floating (F12)."""
        box_min = np.array(g.CTRL_POS) - np.array(g.CTRL_SIZE) / 2
        box_max = np.array(g.CTRL_POS) + np.array(g.CTRL_SIZE) / 2
        lamp_pos = np.array(g.STATUS_LAMP_POS)

        # Lamp centre must be within box xy footprint
        self.assertGreaterEqual(lamp_pos[0] - g.STATUS_LAMP_R, box_min[0])
        self.assertLessEqual(lamp_pos[0] + g.STATUS_LAMP_R, box_max[0])
        self.assertGreaterEqual(lamp_pos[1] - g.STATUS_LAMP_R, box_min[1])
        self.assertLessEqual(lamp_pos[1] + g.STATUS_LAMP_R, box_max[1])
        # Lamp bottom must rest directly on box top
        lamp_bottom = lamp_pos[2] - g.STATUS_LAMP_H / 2
        self.assertAlmostEqual(lamp_bottom, box_max[2], places=4)


if __name__ == "__main__":
    unittest.main()
