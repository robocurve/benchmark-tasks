import sys, unittest
sys.path.insert(0, '/home/freax/Documents/GitHub/bt-clear-loose-router-collet-wrench-from-benchtop-router-table-before-powering-on/tasks/clear-loose-router-collet-wrench-from-benchtop-router-table-before-powering-on')
import clear_loose_router_collet_wrench_from_benchtop_router_table_before_powering_on as m
import mechkit as mk

class TestRouterTableScoring(unittest.TestCase):
    def setUp(self):
        mk.MEMORY.clear()
        m._TEST_TCPS = [(0.160, 0.250, 0.940), (0.160, -0.250, 0.940)]
        m._TEST_ROBOT_BOXES = []

    def test_initial_state(self):
        objs = {o.name: o for o in m.OBJECTS}
        self.assertEqual(m.score(objs), 0.0)

    def test_wrench_cleared(self):
        objs = {o.name: o for o in m.OBJECTS}
        mk.MEMORY["wrench_cleared"] = True
        self.assertEqual(m.score(objs), 0.25)

    def test_wrench_in_tray(self):
        objs = {o.name: o for o in m.OBJECTS}
        mk.MEMORY["wrench_cleared"] = True
        mk.MEMORY["wrench_in_tray"] = True
        self.assertEqual(m.score(objs), 0.50)

    def test_switch_on_without_hold(self):
        objs = {o.name: o for o in m.OBJECTS}
        mk.MEMORY["wrench_cleared"] = True
        mk.MEMORY["wrench_in_tray"] = True
        mk.MEMORY["switch_on"] = True
        self.assertEqual(m.score(objs), 0.90)

    def test_full_success(self):
        objs = {o.name: o for o in m.OBJECTS}
        mk.MEMORY["wrench_cleared"] = True
        mk.MEMORY["wrench_in_tray"] = True
        mk.MEMORY["switch_on"] = True
        mk.MEMORY["hold_steps"] = 20
        self.assertEqual(m.score(objs), 1.0)
        self.assertTrue(m.success(objs))

    def test_defect_blocks_score(self):
        objs = {o.name: o for o in m.OBJECTS}
        mk.MEMORY["defect"] = True
        self.assertEqual(m.score(objs), 0.0)

if __name__ == '__main__':
    unittest.main()
