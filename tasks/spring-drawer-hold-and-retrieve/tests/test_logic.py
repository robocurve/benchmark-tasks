"""Unit tests for logic in spring-drawer-hold-and-retrieve."""
import sys
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g
import logic


class TestLogic(unittest.TestCase):
    def setUp(self):
        self.dl = logic.DrawerLogic()

    def test_initial_state(self):
        st = self.dl.state
        self.assertFalse(st.drawer_opened_1s)
        self.assertFalse(st.block_lifted_30mm)
        self.assertFalse(st.block_on_shelf)
        self.assertFalse(st.cyan_lamp)
        self.assertFalse(st.pinch)

    def test_open_detection(self):
        # Step 22 steps (1.1 s) at 95 mm open
        for _ in range(22):
            st = self.dl.step(0.05, drawer_disp=0.095, block_pos=g.BLOCK_START_POS)
        self.assertTrue(st.cyan_lamp)
        self.assertTrue(st.drawer_opened_1s)

    def test_pinch_condition(self):
        # Drawer closing (disp = 0.010 m < 30 mm) while the block is in the cabinet mouth zone
        drawer_lip_x = g.CABINET_FRONT_X - 0.010
        block_at_lip = (drawer_lip_x, 0.0, g.BLOCK_REST_Z)
        st = self.dl.step(0.05, drawer_disp=0.010, block_pos=block_at_lip)
        self.assertTrue(st.pinch)

    def test_no_pinch_with_block_deep_in_drawer(self):
        # Closed drawer with the block at its start position, far behind the mouth zone: no pinch.
        st = self.dl.step(0.05, drawer_disp=0.0, block_pos=g.BLOCK_START_POS)
        self.assertFalse(st.pinch)

    def test_dropped_block(self):
        block_dropped = (0.3, 0.0, g.TOP - 0.10)
        st = self.dl.step(0.05, drawer_disp=0.0, block_pos=block_dropped)
        self.assertTrue(st.block_dropped)


if __name__ == "__main__":
    unittest.main()

