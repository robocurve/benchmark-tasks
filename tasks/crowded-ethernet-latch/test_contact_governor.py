"""Contact uses the same bounded reference governor, with no physical rebase."""
import unittest
from unittest.mock import patch
import numpy as np
from yam_arena import ik
from robot import PressTrackingProbe,ReleaseProbe,PREPRESS_DISTANCE,PRESS_GRIP,GRIP
from test_robot import environment
from config import SERVICE,RELEASE_TRAVEL,port

SAFE={'rigid_gap_m':.01,'other_hand_gap_m':.01,'rotation_chord_bound_m':0.}


class ContactGovernorTests(unittest.TestCase):
 def fixture(self):
  probe=PressTrackingProbe(environment(),np.array(port(SERVICE)))
  plug=np.r_[port(SERVICE),0,0,0,1.]
  position=plug[:3]+probe.local_contact+[0,0,.0041-PREPRESS_DISTANCE]-probe.pressR@probe.point
  reference,pe,re=ik.solve(position,ik.matrix_to_quat_xyzw(probe.pressR),probe.q[1],base_pos=(0,-.305,.75),iters=350,tol=1e-6)
  self.assertLess(pe,1e-6)
  probe.start_contact(reference,reference+np.array((.001,-.001,.002,-.002,.003,-.003)))
  p,r=ik.tcp_pose(reference,(0,-.305,.75))
  measured=[probe.arm('left'),(reference,np.array((-.028,-.028)),p,r)]
  probe.right_native_joint_velocity=np.zeros(6);probe.command_dt=.05
  endpoint=plug[:3]+probe.local_contact-[0,0,.0023]-probe.pressR@probe.point
  return probe,plug,measured,endpoint

 def test_warm_seed_keeps_unbiased_reference_and_applied_command_once(self):
  probe,plug,measured,endpoint=self.fixture()
  np.testing.assert_array_equal(probe.press_joint_servo.command(),probe.previous[7:13])
  np.testing.assert_array_equal(probe.press_joint_servo.reference,measured[1][0])
  np.testing.assert_array_equal(probe.press_joint_servo.damping,np.zeros(6))
  self.assertIsNone(probe.entry_joint_servo.reference);self.assertIsNone(probe.contact_path)
  self.assertFalse(hasattr(ReleaseProbe,'start_contact'))
  for reference,applied in (([0]*5,[0]*6),([0]*6,[np.nan]*6),([0]*6,[.020001]*6)):
   with self.assertRaises(ValueError):probe.start_contact(reference,applied)

 def test_contact_uses_live_world_goal_and_fixed_reference_start_without_rebase(self):
  probe,plug,measured,endpoint=self.fixture();before=probe.press_joint_servo.bias.copy()
  plug[0]+=.0001
  targets=(*measured[0][2:],GRIP,*measured[1][2:],PRESS_GRIP)
  with patch.object(probe,'rigid_motion_certificate',return_value=SAFE) as certificate:
   probe.release_targets(.05,.05,'depress_clip',plug,np.eye(3),0.,measured,targets)
  self.assertIsNone(probe.failure);self.assertTrue(certificate.call_count>=2)
  path=probe.contact_path
  np.testing.assert_array_equal(path['start'][0],ik.tcp_pose(measured[1][0],(0,-.305,.75))[0])
  expected=plug[:3]+probe.local_contact-[0,0,.0023]-probe.pressR@probe.point
  np.testing.assert_allclose(path['target'],expected,atol=1e-15)
  self.assertLessEqual(np.max(np.abs(probe.press_joint_servo.bias-before)),.00025+1e-15)
  self.assertGreater(path['clock'],0);self.assertLessEqual(path['clock'],.05)
  distance=np.linalg.norm(path['end'][0]-path['start'][0])
  self.assertGreaterEqual(path['duration'],1.5*distance/.0007)
  witness=path['end'][0]+path['end'][1]@probe.point
  self.assertAlmostEqual(probe.local_contact[2]-(witness-plug[:3])[2],.0023,delta=1e-6)

 def test_zero_slack_holds_contact_clock_and_path_is_never_rebased(self):
  probe,plug,measured,endpoint=self.fixture()
  with patch.object(probe,'rigid_motion_certificate',return_value=SAFE):probe.governed_prepress(measured,endpoint,probe.pressR,contact=True)
  path=probe.contact_path;start=path['start'][0].copy();clock=path['clock']
  measured[1]=(measured[1][0],measured[1][1],measured[1][2]+[0,.001,0],measured[1][3])
  tight=dict(SAFE,rigid_gap_m=.0005)
  with patch.object(probe,'rigid_motion_certificate',side_effect=[tight,SAFE]):probe.governed_prepress(measured,endpoint,probe.pressR,contact=True)
  self.assertIs(probe.contact_path,path);np.testing.assert_array_equal(path['start'][0],start)
  self.assertEqual(path['clock'],clock);self.assertGreater(probe.trajectory_governor['mesh_error_m'],.0005)
  with patch.object(probe,'rigid_motion_certificate',return_value=SAFE):probe.governed_prepress(measured,endpoint+[.0003,0,0],probe.pressR,contact=True)
  self.assertIn('moved more than 0.2 mm',probe.failure)

 def test_contact_certificate_excludes_only_controlled_tab_and_hook(self):
  probe,plug,measured,endpoint=self.fixture()
  arrays=[np.zeros((1,3,3))+i for i in range(4)]
  probe.current_press_obstacles=[(f'clip_{SERVICE}','tab',arrays[0]),(f'clip_{SERVICE}','hook',arrays[1]),('other_clip','tab',arrays[2]),('switch','housing',arrays[3])]
  probe.current_press_left=[arrays[3]]
  exclude=((f'clip_{SERVICE}','tab'),(f'clip_{SERVICE}','hook'))
  with patch('robot.minimum_separation',return_value=.01) as gap:
   probe.rigid_motion_certificate(measured,measured[1][2:],measured[1][2:],[(None,arrays[0])],exclude=exclude)
  retained=gap.call_args_list[0].args[1]
  self.assertEqual(len(retained),2);self.assertIs(retained[0],arrays[2]);self.assertIs(retained[1],arrays[3])

 def test_actual_release_requires_five_frames_and_does_not_pull_or_need_left_reference(self):
  probe,plug,measured,endpoint=self.fixture()
  targets=(*measured[0][2:],GRIP,*measured[1][2:],PRESS_GRIP)
  with patch.object(probe,'rigid_motion_certificate',return_value=SAFE):
   for i in range(4):
    probe.release_targets(.05*i,.05*i,'depress_clip',plug,np.eye(3),RELEASE_TRAVEL,measured,targets)
    self.assertEqual(probe.phase,'depress_clip')
   probe.release_targets(.2,.2,'depress_clip',plug,np.eye(3),RELEASE_TRAVEL,measured,targets)
  self.assertEqual(probe.phase,'withdraw_with_clip_pressed');self.assertEqual(probe.pull_distance,0.)
  self.assertIsNone(probe.entry_joint_servo.reference);self.assertIsNone(probe.failure)

 def test_contact_missing_reference_fails_cleanly(self):
  probe,plug,measured,endpoint=self.fixture();probe.press_joint_servo.reset()
  p,r,end=probe.governed_prepress(measured,endpoint,probe.pressR,contact=True)
  self.assertFalse(end);self.assertIsNone(probe.contact_path);self.assertIn('unbiased reference',probe.failure)


if __name__=='__main__':unittest.main()
