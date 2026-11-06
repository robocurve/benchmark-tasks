import copy
import json
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scoring import BodyState,TaskScorer


class ScoringTests(unittest.TestCase):
    def setUp(self):
        self.layout=json.loads((Path(__file__).resolve().parents[1]/'layout.json').read_text())
        self.scorer=TaskScorer(self.layout)
        self.bodies={}
        for name,row in self.layout['objects'].items():
            if name in ('CleanSpoon','UsedSpoon') or name.startswith(('Meatball_','RawChicken_')):
                self.bodies[name]=self.body(row['position'],ball=name.startswith('Meatball_'))
        self.tcps={'left':np.array([0,.3,1.]),'right':np.array([0,-.3,1.])}
        self.ball='Meatball_1'

    def body(self,p,*,ball=False):
        p=np.array(p,float)
        lo=np.array([-.018,-.018,0]) if ball else np.array([-.12,-.03,0])
        hi=np.array([.018,.018,.036]) if ball else np.array([.10,.03,.04])
        return BodyState(np.r_[p,0,0,0,1],np.zeros(3),np.zeros(3),p+lo,p+hi)

    def steps(self,n=25,contacts=()):
        for _ in range(n):r=self.scorer.update(self.bodies,self.tcps,contacts)
        return r

    def lift(self,tool='CleanSpoon'):
        origin=np.array([.32,.0,.85])
        self.bodies[tool]=self.body(origin)
        self.bodies[self.ball]=self.body(origin+[.054,0,.004],ball=True)
        return self.steps(30,[(tool,self.ball)])

    def deliver(self,name=None):
        name=name or self.ball
        self.bodies[name]=self.body(self.layout['objects']['Plate']['position']+np.array([0,0,.004]),ball=True)
        return self.steps(30,[(name,'Plate')])

    def test_initial_scene_is_zero(self):
        self.assertEqual(self.steps()['score'],0)

    def test_clean_supported_lift_without_delivery_is_zero(self):
        self.assertEqual(self.lift()['score'],0)

    def test_clean_scoop_then_released_delivery_is_one(self):
        self.lift();r=self.deliver()
        self.assertEqual(r['score'],1);self.assertTrue(r['ball_released'])

    def test_placement_without_scoop_history_is_zero(self):
        self.assertEqual(self.deliver()['score'],0)

    def test_used_spoon_completed_delivery_is_half(self):
        self.assertEqual(self.lift('UsedSpoon')['score'],0)
        self.assertEqual(self.deliver()['score'],.5)

    def test_contamination_persists_after_contact(self):
        self.steps(1,[('CleanSpoon','RawChicken_1')])
        self.assertEqual(self.lift()['score'],0)
        self.assertEqual(self.deliver()['score'],.5)

    def test_contamination_propagates_through_contacts(self):
        self.lift()
        r=self.steps(1,[('UsedSpoon','Plate'),('Plate',self.ball)])
        self.assertIn(self.ball,r['contaminated'])
        self.assertEqual(self.deliver()['score'],.5)

    def test_robot_pickup_is_not_a_spoon_scoop(self):
        self.steps(1,[(self.ball,'left_robot')]);self.lift()
        self.assertEqual(self.deliver()['score'],0)

    def test_two_balls_on_plate_is_zero(self):
        # Exercise the reusable scorer with an additional food instance.
        self.layout['objects']['Meatball_2']=copy.deepcopy(self.layout['objects']['Meatball_1'])
        self.bodies['Meatball_2']=self.body(self.layout['objects']['Meatball_2']['position'],ball=True)
        self.scorer=TaskScorer(self.layout)
        self.lift();self.deliver()
        self.assertEqual(self.deliver('Meatball_2')['score'],0)

    def test_ball_must_leave_the_spoon(self):
        self.lift();self.deliver()
        self.assertEqual(self.steps(30,[(self.ball,'CleanSpoon')])['score'],0)

    def test_fallen_food_is_zero(self):
        self.lift();self.deliver()
        self.bodies['Meatball_1']=self.body([.4,0,.1],ball=True)
        self.assertEqual(self.steps()['score'],0)

    def test_contact_reporting_failure_is_zero(self):
        self.lift();self.deliver()
        self.assertEqual(self.scorer.update(self.bodies,self.tcps,[],contact_healthy=False)['score'],0)

    def test_reset_clears_trial_history(self):
        self.lift();self.deliver();self.scorer.reset()
        self.assertEqual(self.steps()['score'],0)
        self.assertEqual(self.scorer.transported,set())

    def test_nonfinite_body_is_zero(self):
        self.bodies[self.ball].pose[0]=np.nan
        self.assertEqual(self.steps()['score'],0)

    def test_nonfinite_robot_state_is_zero(self):
        self.lift();self.tcps['left'][0]=np.nan
        self.assertEqual(self.steps()['score'],0)

    def test_supported_meatball_near_plate_rim_is_served(self):
        self.lift()
        self.bodies[self.ball]=self.body([.2046,.0542,.7515],ball=True)
        self.assertEqual(self.steps(30,[(self.ball,'Plate')])['score'],1)

    def test_near_plate_without_physical_support_is_zero(self):
        self.lift()
        self.bodies[self.ball]=self.body([.13,0,.7515],ball=True)
        self.assertEqual(self.steps(30)['score'],0)


if __name__=='__main__':unittest.main()
