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


class TestLidLogic(unittest.TestCase):
    """The closing step: lid seated flat on the rim with both latch heads over its end edges."""

    def setUp(self):
        self.brick = (g.BRICK_TARGET_X, g.BRICK_TARGET_Y, g.BRICK_TARGET_Z)
        self.buffer = (g.BUFFER_TARGET_X, g.BUFFER_TARGET_Y, g.BUFFER_TARGET_Z)
        self.carton = (g.CARTON_TARGET_X, g.CARTON_TARGET_Y, g.CARTON_TARGET_Z)
        self.lid = (g.SHIPPER_X, g.SHIPPER_Y, g.LID_SEATED_Z)
        self.rest = {"latch_pos_y": 0.0, "latch_neg_y": 0.0}

    def _eval(self, **over):
        kw = dict(brick_pos=self.brick, buffer_pos=self.buffer, carton_pos=self.carton, lid_pos=self.lid, latch_press=self.rest)
        kw.update(over)
        return logic.evaluate_step(**kw)

    def test_lid_closed_on_a_correct_stack(self):
        st, viol = self._eval()
        self.assertTrue(st.lid_seated)
        self.assertTrue(st.latch_pos_engaged and st.latch_neg_engaged)
        self.assertTrue(st.lid_closed)
        self.assertFalse(viol)
        self.assertEqual(st.score, 0.75)  # 1.00 comes only after the 3.0 s hold (scoring.py)

    def test_head_overlap_numbers(self):
        # seated and centred, each head hangs LATCH_OVERLAP (4 mm) over its end edge
        over_pos, over_neg = logic.latch_overlaps(0.0, self.rest)
        self.assertAlmostEqual(over_pos, g.LATCH_OVERLAP, places=6)
        self.assertAlmostEqual(over_neg, g.LATCH_OVERLAP, places=6)
        # a slider pushed out by the full overlap leaves the head level with the edge (0 mm over)
        over_pos, _ = logic.latch_overlaps(0.0, {"latch_pos_y": g.LATCH_OVERLAP, "latch_neg_y": 0.0})
        self.assertAlmostEqual(over_pos, 0.0, places=6)

    def test_lid_needs_both_latches(self):
        for name in logic.LATCH_NAMES:
            press = dict(self.rest)
            press[name] = g.LATCH_TRAVEL
            st, _ = self._eval(latch_press=press)
            self.assertTrue(st.lid_seated)
            self.assertFalse(st.lid_closed, name)
            self.assertEqual(st.latch_pos_engaged, name != "latch_pos_y")
            self.assertEqual(st.latch_neg_engaged, name != "latch_neg_y")

    def test_unknown_latch_state_is_not_engaged(self):
        st, _ = self._eval(latch_press=None)
        self.assertTrue(st.lid_seated)
        self.assertFalse(st.lid_closed)

    def test_lid_on_the_latch_heads_is_not_seated(self):
        z = g.LID_ON_LATCH_BOTTOM_Z + g.LID_THICK_Z / 2.0
        self.assertGreater(z - g.LID_SEATED_Z, 0.015)
        st, _ = self._eval(lid_pos=(g.SHIPPER_X, g.SHIPPER_Y, z))
        self.assertFalse(st.lid_seated)
        self.assertFalse(st.lid_closed)

    def test_lid_must_be_flat_still_and_centred(self):
        tilt = np.radians(6.0)
        quat = (np.sin(tilt / 2.0), 0.0, 0.0, np.cos(tilt / 2.0))
        self.assertFalse(self._eval(lid_quat=quat)[0].lid_seated)
        self.assertFalse(self._eval(lid_vel=(0.2, 0.0, 0.0))[0].lid_seated)
        self.assertFalse(self._eval(lid_pos=(g.SHIPPER_X + 0.02, g.SHIPPER_Y, g.LID_SEATED_Z))[0].lid_seated)
        # a lid shifted 5 mm along Y still sits on the rim but a head no longer hangs over that edge
        st, _ = self._eval(lid_pos=(g.SHIPPER_X, g.SHIPPER_Y + 0.005, g.LID_SEATED_Z))
        self.assertTrue(st.lid_seated)
        self.assertFalse(st.lid_closed)
        # shifted 3 mm: both heads still hang over by 1 mm or more
        st, _ = self._eval(lid_pos=(g.SHIPPER_X, g.SHIPPER_Y + 0.003, g.LID_SEATED_Z))
        self.assertTrue(st.lid_closed)

    def test_lid_on_a_carton_resting_on_the_brick_is_the_violation(self):
        carton_direct = (g.SHIPPER_X, g.SHIPPER_Y, g.BRICK_TOP_SEATED_Z + g.CARTON_HEIGHT_Z / 2.0)
        st, viol = self._eval(buffer_pos=(g.PAD_FRONT_X, g.PAD_FRONT_Y, g.BUFFER_STAGING_Z), carton_pos=carton_direct)
        self.assertTrue(viol)
        self.assertFalse(st.lid_closed)
        self.assertEqual(st.score, 0.25)

    def test_lid_adds_nothing_without_the_layers_under_it(self):
        start_buffer = (g.PAD_FRONT_X, g.PAD_FRONT_Y, g.BUFFER_STAGING_Z)
        start_carton = (g.PAD_LEFT_X, g.PAD_LEFT_Y, g.CARTON_STAGING_Z)
        st, _ = self._eval(buffer_pos=start_buffer, carton_pos=start_carton)
        self.assertFalse(st.lid_closed)
        self.assertEqual(st.score, 0.25)
        st, _ = self._eval(carton_pos=start_carton)
        self.assertFalse(st.lid_closed)
        self.assertEqual(st.score, 0.50)

    def test_violation_latched_earlier_blocks_the_lid(self):
        st, viol = self._eval(latched_violation=True)
        self.assertTrue(viol)
        self.assertFalse(st.lid_closed)
        self.assertEqual(st.score, 0.25)

    def test_shipper_nudged_moves_the_latch_frame_with_it(self):
        # the whole shipper and lid shifted 20 mm along Y: the lid is still latched in the shipper's frame
        # (it fails the layer check in world coordinates, so lid_closed is False, but the latch maths follows the shipper)
        shift = 0.02
        seated, (eng_pos, eng_neg) = logic.evaluate_lid(
            (g.SHIPPER_X, g.SHIPPER_Y + shift, g.LID_SEATED_Z), latch_press=self.rest, shipper_pos=(g.SHIPPER_X, g.SHIPPER_Y + shift, g.TABLE_Z)
        )
        self.assertTrue(seated and eng_pos and eng_neg)


if __name__ == "__main__":
    unittest.main()
