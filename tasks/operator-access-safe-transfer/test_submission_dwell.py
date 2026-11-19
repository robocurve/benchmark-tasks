"""Portable, independent native-free regression of scorer duration and release."""
import sys,unittest
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import transfer_scoring as t
import personnel_guard as p
from test_transfer_scoring import cube,step

class DwellContract(unittest.TestCase):
    def setUp(self):t.reset_for_new_trial()
    def test_ten_stage_samples(self):
        t.STATE['source_lift_seen']=True
        self.assertEqual(t.STAGE_DWELL,10)
        for _ in range(9):self.assertEqual(step(cube(t.STAGE,t.STAGE_CUBE_Z),gap=.04),0.0)
        self.assertEqual(step(cube(t.STAGE,t.STAGE_CUBE_Z),gap=.04),0.5)
    def test_twenty_final_samples(self):
        t.STATE.update(stage_qualified=True,stage_relift_seen=True,source_lift_seen=True)
        self.assertEqual(t.GOAL_DWELL,20)
        for _ in range(19):self.assertEqual(step(cube(t.GOAL,t.GOAL_CUBE_Z),gap=.04),0.5)
        self.assertEqual(step(cube(t.GOAL,t.GOAL_CUBE_Z),gap=.04),1.0)
    def test_no_fake_release(self):
        t.STATE.update(stage_qualified=True,stage_relift_seen=True,source_lift_seen=True)
        for _ in range(100):self.assertEqual(step(cube(t.GOAL,t.GOAL_CUBE_Z),gap=.002),0.5)
    def test_personnel_clearance_distinct_from_fixture(self):
        self.assertEqual(p.MIN_PERSONNEL_GAP_M,.030)
        self.assertEqual(p.GRIPPER_CLEARANCE_AT_RELEASE_M,.025)

if __name__=='__main__':unittest.main()
