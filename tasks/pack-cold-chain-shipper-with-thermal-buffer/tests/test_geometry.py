"""Unit tests for geometry constants, clearances, and coordinates in pack-cold-chain-shipper-with-thermal-buffer."""

from __future__ import annotations

import unittest
import geometry as g


class TestGeometry(unittest.TestCase):
    def test_table_elevation(self):
        self.assertAlmostEqual(g.TABLE_Z, 0.7475, places=4)
        self.assertAlmostEqual(g.TOP, 0.7475, places=4)

    def test_clearance_meets_four_mm_rule(self):
        # Long axis (Y): every part keeps >= 4 mm per side (the design gives 10 mm or more)
        self.assertGreaterEqual(g.BRICK_CLEARANCE_Y, 0.004)
        self.assertGreaterEqual(g.BUFFER_CLEARANCE_Y, 0.004)
        self.assertGreaterEqual(g.CARTON_CLEARANCE_Y, 0.004)
        self.assertAlmostEqual(g.BRICK_CLEARANCE_Y, 0.010, places=4)
        self.assertAlmostEqual(g.BUFFER_CLEARANCE_Y, 0.011, places=4)
        self.assertAlmostEqual(g.CARTON_CLEARANCE_Y, 0.021, places=4)

    def test_fingers_fit_beside_pinched_part(self):
        # Pinched axis (X): at least 15 mm per side so the 12 mm fingers go down beside the part
        self.assertAlmostEqual(g.BRICK_CLEARANCE_X, 0.037, places=4)
        self.assertAlmostEqual(g.BUFFER_CLEARANCE_X, 0.038, places=4)
        self.assertAlmostEqual(g.CARTON_CLEARANCE_X, 0.040, places=4)
        for free in (g.BRICK_CLEARANCE_X, g.BUFFER_CLEARANCE_X, g.CARTON_CLEARANCE_X):
            self.assertGreaterEqual(free, g.FINGER_FREE_PER_SIDE)
            # the finger body, with the pad gap opened for release, still leaves >= 4 mm to the wall
        for width, free in (
            (g.BRICK_WIDTH_X, g.BRICK_CLEARANCE_X),
            (g.BUFFER_WIDTH_X, g.BUFFER_CLEARANCE_X),
            (g.CARTON_WIDTH_X, g.CARTON_CLEARANCE_X),
        ):
            release_gap = -2.0 * g.FINGER_RELEASE
            finger_outer_from_part_side = (release_gap - width) / 2.0 + g.FINGER_THICK
            self.assertGreaterEqual(free - finger_outer_from_part_side, 0.004)

    def test_shipper_cavity_dimensions(self):
        self.assertAlmostEqual(g.SHIPPER_X, 0.280, places=3)
        self.assertAlmostEqual(g.SHIPPER_Y, 0.000, places=3)
        self.assertAlmostEqual(g.CAVITY_LENGTH_Y, 0.172, places=3)
        self.assertAlmostEqual(g.CAVITY_WIDTH_X, 0.150, places=3)
        self.assertAlmostEqual(g.CAVITY_DEPTH_Z, 0.070, places=3)
        self.assertAlmostEqual(g.CAVITY_FLOOR_Z, g.TABLE_Z + 0.010, places=4)
        self.assertAlmostEqual(g.CAVITY_RIM_Z, g.CAVITY_FLOOR_Z + 0.070, places=4)

    def test_component_dimensions_and_masses(self):
        # Gel brick, about 6 x 3 x 1 in
        self.assertAlmostEqual(g.BRICK_LENGTH_Y, 0.152, places=3)
        self.assertAlmostEqual(g.BRICK_WIDTH_X, 0.076, places=3)
        self.assertAlmostEqual(g.BRICK_HEIGHT_Z, 0.025, places=3)
        self.assertAlmostEqual(g.BRICK_MASS, 0.33, places=2)

        # Corrugated thermal buffer barrier pad
        self.assertAlmostEqual(g.BUFFER_LENGTH_Y, 0.150, places=3)
        self.assertAlmostEqual(g.BUFFER_WIDTH_X, 0.074, places=3)
        self.assertAlmostEqual(g.BUFFER_HEIGHT_Z, 0.015, places=3)
        self.assertAlmostEqual(g.BUFFER_MASS, 0.04, places=2)

        # Medication carton
        self.assertAlmostEqual(g.CARTON_LENGTH_Y, 0.130, places=3)
        self.assertAlmostEqual(g.CARTON_WIDTH_X, 0.070, places=3)
        self.assertAlmostEqual(g.CARTON_HEIGHT_Z, 0.025, places=3)
        self.assertAlmostEqual(g.CARTON_MASS, 0.18, places=2)

    def test_pinched_widths_fit_the_gripper(self):
        for width in (g.BRICK_WIDTH_X, g.BUFFER_WIDTH_X, g.CARTON_WIDTH_X):
            self.assertLessEqual(width, g.MAX_PINCH_WIDTH + 1e-9)
            self.assertLessEqual(width, g.GRIPPER_MAX_OPEN - 0.014 + 1e-9)  # >= 7 mm per side
        # the pinched side is the short side of every part
        self.assertLess(g.BRICK_WIDTH_X, g.BRICK_LENGTH_Y)
        self.assertLess(g.BUFFER_WIDTH_X, g.BUFFER_LENGTH_Y)
        self.assertLess(g.CARTON_WIDTH_X, g.CARTON_LENGTH_Y)

    def test_no_static_overlaps_at_spawn(self):
        # staging pads clear the shipper walls and each other; parts sit on their pads
        shipper_x_lo = g.SHIPPER_X - g.SHIPPER_EXT_X / 2.0
        self.assertLess(g.PAD_FRONT_X + g.PAD_SIZE_X / 2.0, shipper_x_lo - 0.004)
        for px, py in ((g.PAD_LEFT_X, g.PAD_LEFT_Y), (g.PAD_RIGHT_X, g.PAD_RIGHT_Y)):
            self.assertGreater(abs(py) - g.PAD_SIZE_Y / 2.0, g.SHIPPER_Y + g.SHIPPER_EXT_Y / 2.0)
            self.assertGreater(abs(py) - g.PAD_SIZE_Y / 2.0, g.PAD_FRONT_Y + g.PAD_SIZE_Y / 2.0)
        for length, width in (
            (g.BRICK_LENGTH_Y, g.BRICK_WIDTH_X),
            (g.BUFFER_LENGTH_Y, g.BUFFER_WIDTH_X),
            (g.CARTON_LENGTH_Y, g.CARTON_WIDTH_X),
        ):
            self.assertLessEqual(length, g.PAD_SIZE_Y)
            self.assertLessEqual(width, g.PAD_SIZE_X)
        for z, h in (
            (g.BRICK_STAGING_Z, g.BRICK_HEIGHT_Z),
            (g.BUFFER_STAGING_Z, g.BUFFER_HEIGHT_Z),
            (g.CARTON_STAGING_Z, g.CARTON_HEIGHT_Z),
        ):
            self.assertGreaterEqual(z - h / 2.0, g.PAD_SURFACE_Z)

    def test_staging_layout(self):
        self.assertAlmostEqual(g.PAD_LEFT_X, 0.220, places=3)
        self.assertAlmostEqual(g.PAD_LEFT_Y, 0.220, places=3)
        self.assertAlmostEqual(g.PAD_RIGHT_X, 0.220, places=3)
        self.assertAlmostEqual(g.PAD_RIGHT_Y, -0.220, places=3)
        self.assertAlmostEqual(g.PAD_FRONT_X, 0.115, places=3)
        self.assertAlmostEqual(g.PAD_FRONT_Y, 0.000, places=3)

    def test_vertical_stacking_sequence(self):
        # Layer 1: Gel brick on cavity floor
        self.assertAlmostEqual(g.BRICK_TARGET_Z, 0.7700, places=4)
        self.assertAlmostEqual(g.BRICK_TOP_SEATED_Z, 0.7825, places=4)

        # Layer 2: Buffer pad over gel brick
        self.assertAlmostEqual(g.BUFFER_TARGET_Z, 0.7900, places=4)
        self.assertAlmostEqual(g.BUFFER_TOP_SEATED_Z, 0.7975, places=4)

        # Layer 3: Medication carton over buffer pad
        self.assertAlmostEqual(g.CARTON_TARGET_Z, 0.8100, places=4)
        self.assertAlmostEqual(g.CARTON_TOP_SEATED_Z, 0.8225, places=4)

        # Carton top must be below the shipper rim
        self.assertLess(g.CARTON_TOP_SEATED_Z, g.CAVITY_RIM_Z)

    def test_contact_freezing_hazard_threshold(self):
        # Vertical distance between carton and brick with buffer present
        safe_sep = g.CARTON_TARGET_Z - g.BRICK_TARGET_Z
        self.assertAlmostEqual(safe_sep, 0.0400, places=4)
        self.assertGreater(safe_sep, g.CONTACT_FREEZE_DIST_Z)

        # Vertical distance without buffer pad (carton sitting directly on brick)
        unsafe_carton_z = g.BRICK_TOP_SEATED_Z + g.CARTON_HEIGHT_Z / 2.0
        unsafe_sep = unsafe_carton_z - g.BRICK_TARGET_Z
        self.assertAlmostEqual(unsafe_sep, 0.0250, places=4)
        self.assertLess(unsafe_sep, g.CONTACT_FREEZE_DIST_Z)

    def test_lid_and_latch_dimensions(self):
        # the lid is the shipper's outer footprint, 8 mm thick, with a 40 x 30 x 22 mm grip tab
        self.assertAlmostEqual(g.LID_WIDTH_X, 0.170, places=4)
        self.assertAlmostEqual(g.LID_LENGTH_Y, 0.192, places=4)
        self.assertAlmostEqual(g.LID_THICK_Z, 0.008, places=4)
        self.assertAlmostEqual(g.LID_MASS, 0.12, places=3)
        self.assertAlmostEqual(g.LID_TAB_X, 0.040, places=4)
        self.assertGreaterEqual(g.LID_TAB_Z, 0.020)
        self.assertLessEqual(g.LID_TAB_Z, 0.025)
        # it rests on the rim, 5 mm above the carton (no contact with the layers)
        self.assertAlmostEqual(g.LID_SEAT_BOTTOM_Z, g.CAVITY_RIM_Z, places=6)
        self.assertAlmostEqual(g.LID_SEAT_BOTTOM_Z - g.CARTON_TOP_SEATED_Z, 0.005, places=4)
        # latches: 0.5 N/mm, 8 mm travel, head 4 mm over the lid edge, 1.5 mm above a seated lid
        self.assertAlmostEqual(g.LATCH_RATE, 500.0)
        self.assertAlmostEqual(g.LATCH_TRAVEL, 0.008)
        self.assertAlmostEqual(g.LATCH_OVERLAP, 0.004)
        head_bottom = g.TABLE_Z + g.LATCH_CENTRE_Z - g.LATCH_RADIUS
        self.assertAlmostEqual(head_bottom - g.LID_TOP_SEATED_Z, g.LATCH_GAP_ABOVE_LID, places=6)
        self.assertGreater(g.LATCH_TRAVEL, g.LATCH_OVERLAP)
        # the head's rod must let the lid's lower end edge slide past it without wedging: the edge meets the circle at
        # an angle whose tangent exceeds the combined friction (shipper and lid friction average)
        d = g.LATCH_RADIUS - g.LATCH_OVERLAP
        tan_contact = d / (g.LATCH_RADIUS**2 - d**2) ** 0.5
        self.assertGreater(tan_contact, 0.5 * (g.SHIPPER_FRICTION + g.LID_FRICTION))
        # the lid on the heads rests well above the rim (that is the ablation pose)
        self.assertGreater(g.LID_ON_LATCH_BOTTOM_Z - g.LID_SEAT_BOTTOM_Z, 0.015)
        # the post stands outside the end wall (no overlap with the shipper), the head over the lid
        self.assertGreater(g.LATCH_POST_Y0, g.SHIPPER_EXT_Y / 2.0)
        self.assertLess(g.LATCH_TIP_Y, g.LID_LENGTH_Y / 2.0)

    def test_lid_staging_clears_everything(self):
        for y in (g.LID_PAD_Y_A, g.LID_PAD_Y_B):
            gap = abs(y) - g.LID_PAD_SIZE_Y / 2.0 - (abs(g.PAD_LEFT_Y) + g.PAD_SIZE_Y / 2.0)
            self.assertGreaterEqual(gap, 0.004)
            self.assertGreater(g.LID_PAD_SIZE_X, g.LID_WIDTH_X)
            self.assertGreater(g.LID_PAD_SIZE_Y, g.LID_LENGTH_Y)
        self.assertAlmostEqual(g.LID_STAGING_Z - g.LID_THICK_Z / 2.0, g.PAD_SURFACE_Z + g.SPAWN_GAP, places=6)

    def test_tolerances(self):
        self.assertAlmostEqual(g.HOLD_DWELL_S, 3.0, places=1)
        self.assertEqual(g.HOLD_STEPS, 60)
        self.assertAlmostEqual(g.TILT_TOL_DEG, 4.0, places=1)
        self.assertAlmostEqual(g.POS_TOL_MM, 3.0, places=1)
        self.assertAlmostEqual(g.ROT_TOL_DEG, 3.0, places=1)
        self.assertAlmostEqual(g.LIMIT_MARGIN_RAD, 0.05, places=2)


if __name__ == "__main__":
    unittest.main()
