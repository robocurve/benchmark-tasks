import json
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scoring import IceScorer


class ScoreTests(unittest.TestCase):
    def setUp(self):
        self.layout=json.loads((Path(__file__).resolve().parents[1]/'layout.json').read_text())
        self.scorer=IceScorer(self.layout)
        self.poses={n:np.r_[r['position'],r.get('rotation_xyzw',[0,0,0,1])] for n,r in self.layout['objects'].items() if r['dynamic']}
        self.vel={n:np.zeros(6) for n in self.poses}
        self.tcps={'left':np.array([0,.5,1.2]),'right':np.array([0,-.5,1.2])}

    def steps(self,n=30,contacts=(),healthy=True):
        for _ in range(n):r=self.scorer.update(self.poses,self.vel,contacts,self.tcps,healthy)
        return r

    def open_lid(self):
        self.poses['CleanLid'][2]+=.08
        return self.steps()

    def lift(self,name='CleanIce_001'):
        self.poses['Scoop']=np.array([.385,-.18,.94,0,0,0,1.])
        self.poses[name]=np.array([.415,-.18,.96,0,0,0,1.])
        return self.steps(30,[('Scoop',name)])

    def deliver(self,name='CleanIce_001'):
        self.poses[name]=np.r_[np.array(self.layout['objects']['Bowl']['position'])+[0,0,.016],0,0,0,1]
        return self.steps()

    def test_initial_zero(self):self.assertEqual(self.steps()['score'],0)
    def test_lid_progress_quarter(self):self.assertEqual(self.open_lid()['score'],.25)
    def test_supported_scoop_progress_quarter(self):self.assertEqual(self.lift()['score'],.25)

    def test_one_clean_cube_is_full_credit(self):
        self.open_lid();self.lift();self.assertEqual(self.deliver()['score'],1)

    def test_fish_cube_delivery_is_half_credit(self):
        self.lift('FishIce_001');self.assertEqual(self.deliver('FishIce_001')['score'],.5)

    def test_clean_cube_without_lid_history_is_half_credit(self):
        self.lift();self.assertEqual(self.deliver()['score'],.5)

    def test_contaminated_scoop_delivery_is_half_credit(self):
        self.open_lid();self.steps(1,[('Scoop','Fish')]);self.lift()
        self.assertEqual(self.deliver()['score'],.5)

    def test_direct_gripper_transfer_gets_no_delivery_credit(self):
        self.steps(1,[('CleanIce_001','right_robot')]);self.lift()
        self.assertEqual(self.deliver()['score'],0)

    def test_placement_without_scoop_history_is_zero(self):self.assertEqual(self.deliver()['score'],0)

    def test_lid_can_close_after_valid_opening(self):
        self.open_lid();self.lift();self.poses['CleanLid'][2]-=.08
        self.assertEqual(self.deliver()['score'],1)

    def test_more_than_one_clean_cube_is_allowed(self):
        self.open_lid();self.lift();self.deliver();self.lift('CleanIce_002')
        self.assertEqual(self.deliver('CleanIce_002')['score'],1)

    def test_mixed_delivery_cannot_get_full_credit(self):
        self.open_lid();self.lift('FishIce_001');self.deliver('FishIce_001');self.lift()
        self.assertEqual(self.deliver()['score'],.5)

    def test_still_supported_ice_is_not_delivered(self):
        self.open_lid();self.lift();self.deliver()
        self.assertEqual(self.steps(30,[('Scoop','CleanIce_001')])['score'],.25)

    def test_unhealthy_contact_reporting_is_zero(self):
        self.open_lid();self.lift();self.deliver();self.assertEqual(self.steps(healthy=False)['score'],0)

    def test_reset_clears_history(self):
        self.open_lid();self.lift();self.deliver();self.scorer.reset();self.assertEqual(self.steps()['score'],.25)
        self.assertEqual(self.scorer.scooped,set())

    def test_nonfinite_state_is_zero(self):
        self.poses['Scoop'][0]=np.nan;self.assertEqual(self.steps()['score'],0)


if __name__=='__main__':unittest.main()
