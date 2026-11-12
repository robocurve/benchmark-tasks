"""Unit tests for state evaluation and freezing hazard logic in pack-cold-chain-shipper-with-thermal-buffer."""

from __future__ import annotations

import unittest
import numpy as np

import geometry as g
import logic


class TestLogic(unittest.TestCase):
    def setUp(self):
        self.brick_start = (g.PAD_RIGHT_X, g.PAD_RIGHT_Y, g.BRICK_STAGING_Z)
        self.buffer_start = (g.PAD_FRONT_X, g.PAD_FRONT_Y, g.BUFFER_STAGING_Z)
        self.carton_start = (g.PAD_LEFT_X, g.PAD_LEFT_Y, g.CARTON_STAGING_Z)

    def test_initial_state(self):
        st, viol = logic.evaluate_step(
            brick_pos=self.brick_start,
            buffer_pos=self.buffer_start,
            carton_pos=self.carton_start,
            latched_violation=False,
        )
        self.assertEqual(st.score, 0.0)
        self.assertFalse(st.brick_placed)
        self.assertFalse(st.buffer_placed)
        self.assertFalse(st.carton_placed)
        self.assertFalse(st.safe_order)
        self.assertFalse(st.direct_contact_violation)
        self.assertFalse(viol)
        self.assertTrue(st.arms_clear)

    def test_brick_placed_025(self):
        brick_seated = (g.BRICK_TARGET_X, g.BRICK_TARGET_Y, g.BRICK_TARGET_Z)
        st, viol = logic.evaluate_step(
            brick_pos=brick_seated,
            buffer_pos=self.buffer_start,
            carton_pos=self.carton_start,
            latched_violation=False,
        )
        self.assertTrue(st.brick_placed)
        self.assertFalse(st.buffer_placed)
        self.assertFalse(st.carton_placed)
        self.assertEqual(st.score, 0.25)
        self.assertFalse(viol)

    def test_buffer_placed_050(self):
        brick_seated = (g.BRICK_TARGET_X, g.BRICK_TARGET_Y, g.BRICK_TARGET_Z)
        buffer_seated = (g.BUFFER_TARGET_X, g.BUFFER_TARGET_Y, g.BUFFER_TARGET_Z)
        st, viol = logic.evaluate_step(
            brick_pos=brick_seated,
            buffer_pos=buffer_seated,
            carton_pos=self.carton_start,
            latched_violation=False,
        )
        self.assertTrue(st.brick_placed)
        self.assertTrue(st.buffer_placed)
        self.assertFalse(st.carton_placed)
        self.assertEqual(st.score, 0.50)
        self.assertFalse(viol)

    def test_carton_placed_075(self):
        brick_seated = (g.BRICK_TARGET_X, g.BRICK_TARGET_Y, g.BRICK_TARGET_Z)
        buffer_seated = (g.BUFFER_TARGET_X, g.BUFFER_TARGET_Y, g.BUFFER_TARGET_Z)
        carton_seated = (g.CARTON_TARGET_X, g.CARTON_TARGET_Y, g.CARTON_TARGET_Z)
        st, viol = logic.evaluate_step(
            brick_pos=brick_seated,
            buffer_pos=buffer_seated,
            carton_pos=carton_seated,
            latched_violation=False,
        )
        self.assertTrue(st.brick_placed)
        self.assertTrue(st.buffer_placed)
        self.assertTrue(st.carton_placed)
        self.assertTrue(st.safe_order)
        self.assertEqual(st.score, 0.75)
        self.assertFalse(viol)

    def test_direct_contact_violation_caps_at_025(self):
        # Carton placed directly onto gel brick omitting buffer pad
        brick_seated = (g.BRICK_TARGET_X, g.BRICK_TARGET_Y, g.BRICK_TARGET_Z)
        # Sitting directly on top of 25 mm brick -> Z = 0.7825 + 0.0125 = 0.7950 m
        carton_direct = (g.SHIPPER_X, g.SHIPPER_Y, g.BRICK_TOP_SEATED_Z + g.CARTON_HEIGHT_Z / 2.0)

        # Buffer pad left at staging
        st, viol = logic.evaluate_step(
            brick_pos=brick_seated,
            buffer_pos=self.buffer_start,
            carton_pos=carton_direct,
            latched_violation=False,
        )
        self.assertTrue(viol)
        self.assertTrue(st.direct_contact_violation)
        self.assertEqual(st.score, 0.25)  # capped at 0.25

    def test_latched_violation_persists(self):
        # If violation was previously latched, score cannot exceed 0.25 even if buffer is inserted later
        brick_seated = (g.BRICK_TARGET_X, g.BRICK_TARGET_Y, g.BRICK_TARGET_Z)
        buffer_seated = (g.BUFFER_TARGET_X, g.BUFFER_TARGET_Y, g.BUFFER_TARGET_Z)
        carton_seated = (g.CARTON_TARGET_X, g.CARTON_TARGET_Y, g.CARTON_TARGET_Z)

        st, viol = logic.evaluate_step(
            brick_pos=brick_seated,
            buffer_pos=buffer_seated,
            carton_pos=carton_seated,
            latched_violation=True,
        )
        self.assertTrue(viol)
        self.assertTrue(st.direct_contact_violation)
        self.assertEqual(st.score, 0.25)  # Capped at 0.25!

    def test_excessive_tilt_fails_placement(self):
        brick_seated = (g.BRICK_TARGET_X, g.BRICK_TARGET_Y, g.BRICK_TARGET_Z)
        # 10 degree tilt (exceeds 4 deg limit)
        angle_rad = np.radians(10.0)
        qx = np.sin(angle_rad / 2.0)
        qw = np.cos(angle_rad / 2.0)
        quat = (qx, 0.0, 0.0, qw)

        st, _ = logic.evaluate_step(
            brick_pos=brick_seated,
            brick_quat=quat,
            buffer_pos=self.buffer_start,
            carton_pos=self.carton_start,
        )
        self.assertFalse(st.brick_placed)
        self.assertEqual(st.score, 0.0)

    def test_arms_clear_detection(self):
        # Arm hovering right inside shipper work zone
        arm_tcps_in_shipper = {
            "left": [g.SHIPPER_X, g.SHIPPER_Y, g.CAVITY_RIM_Z + 0.010],
            "right": list(g.HOME_R),
        }
        self.assertFalse(logic.check_arms_clear(arm_tcps_in_shipper))

        # Both arms at home standby
        arm_tcps_home = {
            "left": list(g.HOME_L),
            "right": list(g.HOME_R),
        }
        self.assertTrue(logic.check_arms_clear(arm_tcps_home))


if __name__ == "__main__":
    unittest.main()
