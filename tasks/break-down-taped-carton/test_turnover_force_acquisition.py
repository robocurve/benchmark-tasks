"""Force acquisition contracts; synthetic clocks/forces are NOT native proof."""
import copy,unittest
import numpy as np
from turnover_contacts import TARGETS

class ControlledReader:
 def __init__(self,left=(0.,.5,0.),right=(0.,-.5,0.),change=None):self.force=[left,right];self.change=change;self.calls=[]
 def read(self,time,phase='pre_action'):
  self.calls.append((time,phase));clock=100.+time
  s={'requested_control_time_s':time,'sample_id':int(round(time*20))+1,'clock_verified':True,'sensor_clock_s':{'left':clock,'right':clock},'sensor_update_time_s':{'left':clock,'right':clock},'sides':{side:{'body_names':['tip_left','tip_right'],'configured_target_family':list(TARGETS),'family_normal_force_on_fingertips_w_N':list(self.force[i])} for i,side in enumerate(('left','right'))}}
  return s if self.change is None else self.change(s)

class Tests(unittest.TestCase):
 def probe(self):
  from test_turnover_transactional import make_probe
  p=make_probe();p.env._turnover_contact_reader=ControlledReader();return p
 def test_signed_normal_forces_not_force_magnitude_gate(self):
  for left,right in [((0,0,0),(0,-1,0)),((0,-1,0),(0,-1,0)),((1,0,0),(0,-1,0)),((0,1,0),(0,1,0)),((0,.1,0),(0,-1,0))]:
   p=self.probe();p.env._turnover_contact_reader=ControlledReader(left,right)
   self.assertFalse(p.contact_forces(0.,np.eye(3))['both_opposed_normal_forces'])
  p=self.probe();r=p.contact_forces(0.,np.eye(3));self.assertTrue(r['both_opposed_normal_forces']);self.assertEqual(r['signed_normal_force_N'],[.5,.5])
 def test_zero_or_wrong_sign_forces_cannot_enter_first_roll_after_settling(self):
  for left,right in [((0,0,0),(0,-.5,0)),((0,-.5,0),(0,-.5,0)),((0,.5,0),(0,.5,0))]:
   p=self.probe();m=p.measure();p.change('contact',0.,m);p.phase_goal_q=[v[0].copy() for v in m]
   from yam_arena.constants import FINGER_OPEN
   p.previous=np.r_[m[0][0],.012/abs(FINGER_OPEN),m[1][0],.012/abs(FINGER_OPEN)]
   p.env._turnover_contact_reader=ControlledReader(left,right)
   for i in range(6):
    _,reports=p.command(3.+i*.05);self.assertIsNone(reports[0]['failure']);self.assertEqual(p.stage,'contact');self.assertEqual(p.contact_stable,0)
    self.assertFalse(reports[0]['geometric_opposed_settle']['contact_force_verified'])
 def test_contact_acquisition_does_not_extend_original18s_deadline(self):
  p=self.probe();m=p.measure();p.change('contact',0.,m);p.phase_goal_q=[v[0].copy() for v in m];p.contact_acquire_started=3.
  p.env._turnover_contact_reader=ControlledReader((0,0,0),(0,-.5,0))
  _,reports=p.command(18.01);self.assertIn('bounded endpoint time',reports[0]['failure']);self.assertEqual(p.started,0.);self.assertFalse(p.done)
 def test_force_signs_follow_measured_body_frame(self):
  from turnover import rx
  R=rx(np.pi/2);p=self.probe();p.env._turnover_contact_reader=ControlledReader(R[:,1]*.5,-R[:,1]*.5)
  self.assertTrue(p.contact_forces(0.,R)['both_opposed_normal_forces'])
 def acquire_wall_probe(self):
  p=self.probe();m=p.measure();p.change('contact',0.,m);p.phase_goal_q=[v[0].copy() for v in m]
  from yam_arena.constants import FINGER_OPEN
  p.previous=np.r_[m[0][0],.012/abs(FINGER_OPEN),m[1][0],.012/abs(FINGER_OPEN)]
  for i in range(6):
   _,reports=p.command(3.+i*.05);self.assertIsNone(reports[0]['failure'])
  self.assertEqual(p.stage,'first_roll');return p
 def test_accepted_wall_preload_survives_first_roll_handoff_without_double_pi(self):
  from turnover import body_frame
  from turnover_targets import wall_target
  from model import HEIGHT,WIDTH
  p=self.acquire_wall_probe();old=p.previous.copy();o,R=body_frame(p.poses());offset=p.wall_preload_offsets[0].copy()
  from yam_arena import ik
  expected=ik.tcp_pose(old[:6],p.env.rig.bases['left'])[0]
  base,_=wall_target(o,R,'left',HEIGHT,WIDTH,p.tip_vertices,0.)
  np.testing.assert_allclose(base+R@offset,expected,atol=1e-10)
  for bias in p.bias:np.testing.assert_array_equal(bias,np.zeros(3))
  _,reports=p.command(3.3);self.assertIsNone(reports[0]['failure'])
  np.testing.assert_allclose(reports[0]['targets_tcp_world_m'][0],expected,atol=1e-9)
  np.testing.assert_array_equal(p.wall_preload_offsets[0],offset)
 def test_fresh_force_loss_is_diagnosed_without_restarting_progress_clock(self):
  p=self.acquire_wall_probe();deadline=p.last_progress;p.env._turnover_contact_reader.force[0]=(0.,0.,0.)
  for i in range(8):
   _,reports=p.command(3.3+i*.05)
   if reports[0]['failure']:break
  self.assertIn('acquired contact force lost',reports[0]['failure']);self.assertEqual(p.last_progress,deadline)
 def test_bottom_gate_requires_only_correctly_signed_right_force(self):
  from turnover import rx
  R=rx(-np.pi/2)
  p=self.probe();p.env._turnover_contact_reader=ControlledReader((0.,0.,0.),-R[:,2]*.5)
  evidence=p.contact_forces(0.,R,'bottom');self.assertTrue(evidence['required_force_contacts']);self.assertFalse(evidence['both_opposed_normal_forces']);self.assertEqual(evidence['required_sides'],['right'])
  for vector in (R[:,2]*.5,np.zeros(3)):
   p=self.probe();p.env._turnover_contact_reader=ControlledReader((0.,0.,0.),vector)
   self.assertFalse(p.contact_forces(0.,R,'bottom')['required_force_contacts'])
 def test_missing_stale_wrong_clock_and_duplicate_sample_fail_closed(self):
  p=self.probe();del p.env._turnover_contact_reader
  with self.assertRaisesRegex(ValueError,'missing'):p.contact_forces(0.,np.eye(3))
  p=self.probe();p.contact_forces(0.,np.eye(3));p.contact_forces(.05,np.eye(3))
  with self.assertRaisesRegex(ValueError,'stale'):p.contact_forces(.05,np.eye(3))
  changes=[lambda s:dict(s,clock_verified=False),lambda s:dict(s,sensor_clock_s={'left':np.nan,'right':100.}),lambda s:dict(s,sensor_update_time_s={'left':99.,'right':100.}),lambda s:dict(s,sensor_clock_s={'left':100.1,'right':100.})]
  for change in changes:
   p=self.probe();p.env._turnover_contact_reader=ControlledReader(change=change)
   with self.assertRaises(ValueError):p.contact_forces(0.,np.eye(3))
  p=self.probe();p.contact_forces(.1,np.eye(3))
  with self.assertRaises(ValueError):p.contact_forces(.05,np.eye(3))

if __name__=='__main__':unittest.main()
