"""Hypothetical90deg geometry with explicit synthetic forces; not native proof."""
import unittest
from types import SimpleNamespace
import numpy as np
from yam_arena import ik
from yam_arena.constants import READY,FINGER_OPEN
from turnover import CartonTurnoverProbe,push_target,solve_target
from turnover_safety import BASES
from check_refined_turnover import Screen
from config import CUTTER_START
from test_turnover_transactional import Tensor
from test_turnover_force_acquisition import ControlledReader


def make_probe(right_sign=1.):
 screen=Screen(10);origin,R,obs=screen.carton(90.)
 front=max(float(t[:,:,2].max()) for _,n,t in screen.geometry.local_hand((.012,.012)) if n.startswith('tip_'))
 p,Q=push_target(origin,R,'bottom',front,.0002,roll_deg=90.)
 q,pe,re=solve_target(p,Q,np.array(READY),BASES['right'],starts=10)
 if pe>.0003 or re>.003:raise AssertionError('hypothetical bottom contact IK failed')
 joints=[np.array(READY),q];ready=np.r_[READY,1.,READY,1.]
 def pose(name):
  if name=='cutter':return Tensor([[*CUTTER_START,0.,0.,0.,1.]])
  p,Q,_=obs[name];return Tensor([[*p,*ik.matrix_to_quat_xyzw(Q)]])
 scene={s+'_robot':SimpleNamespace(data=SimpleNamespace(joint_pos=Tensor([[*joints[i],*( [FINGER_OPEN]*2 if i==0 else [-.012]*2)]]))) for i,s in enumerate(('left','right'))}
 env=SimpleNamespace(_ready=Tensor([ready]),_low=np.r_[ik.LOWER,0.,ik.LOWER,0.],_high=np.r_[ik.UPPER,1.,ik.UPPER,1.],_ids={s+'_robot':(list(range(6)),[6,7]) for s in ('left','right')},rig=SimpleNamespace(bases=BASES),_base=SimpleNamespace(arena_world=SimpleNamespace(get_pose_w=pose),scene=scene))
 env._turnover_contact_reader=ControlledReader((0.,0.,0.),-right_sign*R[:,2]*.5)
 probe=CartonTurnoverProbe(env);m=probe.measure();probe.change('bottom_contact',0.,m);probe.angle=probe.last_raw=probe.best_angle=90.;probe.previous=np.r_[READY,1.,q,.012/abs(FINGER_OPEN)]
 return probe,R

class Tests(unittest.TestCase):
 def test_right_only_five_frame_gate_and_accepted_preload_handoff(self):
  p,R=make_probe();left=p.previous[:7].copy()
  for i in range(5):
   _,reports=p.command(3.+i*.05);self.assertIsNone(reports[0]['failure']);self.assertEqual(p.stage,'bottom_contact')
  _,reports=p.command(3.25);self.assertIsNone(reports[0]['failure']);self.assertEqual(p.stage,'second_roll')
  self.assertEqual(reports[0]['geometric_opposed_settle']['required_sides'],['right'])
  self.assertTrue(reports[0]['geometric_opposed_settle']['contact_force_verified']);np.testing.assert_array_equal(p.previous[:7],left)
  np.testing.assert_array_equal(p.bias[1],np.zeros(3));offset=p.bottom_preload_offset.copy()
  self.assertGreater(offset[2],0.)
  _,reports=p.command(3.3);self.assertIsNone(reports[0]['failure']);np.testing.assert_array_equal(p.bottom_preload_offset,offset)
  self.assertEqual(reports[0]['contact_force_evidence']['required_sides'],['right'])
 def test_zero_or_wrong_signed_right_force_cannot_start_second_roll(self):
  for sign in (0.,-1.):
   p,_=make_probe(sign)
   for i in range(6):
    _,reports=p.command(3.+i*.05);self.assertIsNone(reports[0]['failure']);self.assertEqual(p.stage,'bottom_contact');self.assertEqual(p.contact_stable,0)

if __name__=='__main__':unittest.main()
