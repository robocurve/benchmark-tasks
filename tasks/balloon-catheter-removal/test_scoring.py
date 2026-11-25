"""Numeric protocol and shortcut checks; synthetic fixtures are not robot demos."""
import copy
import unittest
import numpy as np
from scoring import TrialScorer,INITIAL
from layout import VALVE_QUATERNION
from mechanics import DrainState
from test_mechanics import aligned

def initial():return {k:np.array([*v,*(VALVE_QUATERNION if k=='valve' else (0,0,0,1.))]) for k,v in INITIAL.items()}
ROBOT={s:{'tcp':[0.,(-.6 if s=='left' else .6),1.3],'fingers':[.04,.04]} for s in ('left','right')}

class ScoringTests(unittest.TestCase):
    def setUp(self):self.s=TrialScorer();self.n=0;self.state=DrainState();self.p=initial()
    def update(self,p=None,robot=None,mechanics=None):
        self.n+=1;p=self.p if p is None else p
        m=self.state.update(p) if mechanics is None else mechanics
        m.setdefault('tip_device_forces_N',{'left':[[0.,1.],[0.,1.]],'right':[[1.,0.],[1.,0.]]})
        return self.s.update(step=self.n,objects={k:{'lin_vel':[0,0,0],'ang_vel':[0,0,0]} for k in p},
            root_poses=p,robot=robot or ROBOT,mechanics=m)
    def connect(self):
        self.update();self.p=aligned();self.assertTrue(self.state.connect(self.p));return self.update()
    def deflate(self):
        self.connect();self.p['plunger'][0]-=.05;return self.update()
    def withdraw(self):
        self.deflate()
        self.p['barrel'][0]-=.013;self.p['plunger'][0]-=.013;self.update()
        # Move catheter straight through the neck before carrying it to storage.
        for y in np.linspace(-.045,-.22,25):
            self.p['catheter'][1]=y;self.p['valve'][1]=y-.094;self.update()
        return self.update()
    def test_connected_withdrawal_latches_failure_even_if_disconnected_later(self):
        self.deflate()
        self.p['catheter'][1] -= .03
        result=self.update()
        self.assertEqual(result['failure'],'withdrawal_before_disconnection')
        self.p['barrel'][0] -= .013
        self.p['plunger'][0] -= .013
        self.assertEqual(self.update()['score'],0.)
    def test_rest_cannot_substitute_for_grasping_the_valve(self):
        self.update();self.p=aligned();self.assertTrue(self.state.connect(self.p))
        m=self.state.update(self.p)
        m['tip_device_forces_N']={'left':[[0.,1.],[0.,1.]],'right':[[0.,0.],[0.,0.]]}
        result=self.update(mechanics=m)
        self.assertEqual(result['score'],0.)
        self.assertEqual(result['failure'],'connection_without_opposing_device_grasps')
    def test_initial_and_waiting_score_zero(self):
        for _ in range(30):self.assertEqual(self.update()['score'],0.)
    def test_connection_and_deflation_partial_credit(self):
        self.assertEqual(self.connect()['score'],.1)
        self.p['plunger'][0]-=.05;self.assertEqual(self.update()['score'],.4)
    def test_pushing_receiver_back_revokes_deflation_credit(self):
        self.deflate();self.p['plunger'][0]+=.025
        self.assertEqual(self.update()['score'],.1)
    def test_inflated_extraction_latches_zero(self):
        self.connect();self.p['catheter'][1]=-.2
        self.assertEqual(self.update()['score'],0.)
        self.p['plunger'][0]-=.05;self.assertEqual(self.update()['score'],0.)
    def test_premature_receiver_release_latches_zero(self):
        self.connect();self.p['barrel'][0]-=.013;self.p['plunger'][0]-=.013
        self.assertEqual(self.update()['score'],0.)
    def test_withdrawal_and_final_supported_release(self):
        self.assertEqual(self.withdraw()['score'],.7)
        self.p['catheter'][:3]=[.240,-.356,.878]
        self.p['valve'][:3]=[.310,-.400,.793]
        self.p['barrel'][:3]=[.300,.300,.820];self.p['plunger'][:3]=[.250,.300,.820]
        for _ in range(19):self.assertFalse(self.update()['success'])
        self.assertTrue(self.update()['success'])
    def test_old_low_and_outside_height_placements_never_finish(self):
        for z in (.843,.867,.889):
            with self.subTest(z=z):
                self.setUp();self.withdraw()
                self.p['catheter'][:3]=[.240,-.356,z]
                self.p['valve'][:3]=[.240,-.420,.8475]
                self.p['barrel'][:3]=[.300,.300,.820];self.p['plunger'][:3]=[.250,.300,.820]
                for _ in range(25):
                    result=self.update()
                    self.assertFalse(result['catheter_parked']);self.assertEqual(result['score'],.7)
    def test_hands_near_prevent_full_score(self):
        self.withdraw();self.p['catheter'][:3]=[.240,-.356,.878]
        self.p['valve'][:3]=[.310,-.400,.793];self.p['barrel'][:3]=[.300,.300,.820]
        self.p['plunger'][:3]=[.250,.300,.820]
        r=copy.deepcopy(ROBOT);r['left']['tcp']=self.p['catheter'][:3].tolist()
        for _ in range(30):self.assertEqual(self.update(robot=r)['score'],.7)
    def test_holding_extended_thumb_pad_is_not_hands_clear(self):
        self.withdraw();self.p['catheter'][:3]=[.240,-.356,.878]
        self.p['valve'][:3]=[.310,-.400,.793];self.p['barrel'][:3]=[.300,.300,.820]
        self.p['plunger'][:3]=[.250,.300,.820]
        r=copy.deepcopy(ROBOT);r['right']['tcp']=[.178,.300,.820]
        for _ in range(25):self.assertFalse(self.update(robot=r)['hands_clear'])
    def test_missing_nan_or_duplicated_measurements_rejected(self):
        self.update()
        with self.assertRaises(ValueError):self.s.update(step=1,objects={},root_poses={},robot={},mechanics={})
        self.p['valve'][0]=np.nan
        with self.assertRaises(ValueError):self.update()
    def test_deflation_without_connection_cannot_score(self):
        self.update();m=self.state.update(self.p);m.update(collapsed=True,drained_fraction=1.,balloon_radius_m=.004)
        self.assertEqual(self.update(mechanics=m)['score'],0.)
    def test_non_neck_exit_is_invalid(self):
        self.deflate();self.p['catheter'][0]+=.02;self.p['catheter'][1]-=.15
        self.assertEqual(self.update()['score'],0.)
    def test_drop_revokes_all_partial_credit(self):
        self.deflate();self.p['barrel'][2]=.6
        self.assertEqual(self.update()['score'],0.)

if __name__=='__main__':unittest.main()
