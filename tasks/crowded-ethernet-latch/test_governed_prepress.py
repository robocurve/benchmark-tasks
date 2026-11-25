"""FK-based reference governor and trajectory PI integration regressions."""
import json
import unittest
from unittest.mock import patch
import numpy as np
from yam_arena import ik
from robot import PressTrackingProbe,PREPRESS_DURATION
from feedback import JointEndpointCorrection,mesh_pose_distance
from handoff_fixtures import CASES
from test_robot import environment
from config import SERVICE,port


class GovernedPrepressTests(unittest.TestCase):
 def fixture(self):
  case=CASES['safe020'];probe=PressTrackingProbe(environment(),np.array(port(SERVICE)))
  probe.phase='prepress_endpoint';probe.press_joint_servo.latch(case['reference'])
  probe.press_joint_servo.bias[:]=[.001,-.002,.003,.004,-.005,.006];probe.press_joint_servo.freeze()
  probe.press_servo_stage='HOVER';probe.command_dt=.05
  probe.right_native_joint_velocity=np.zeros(6)
  measured=[tuple(np.array(v) for v in arm) for arm in case['measured']]
  position,rotation=ik.tcp_pose(np.array(case['reference']),(0,-.305,.75))
  endpoint=position+np.array((0,0,-.002))
  probe.previous[7:13]=probe.press_joint_servo.command()
  return probe,measured,endpoint,rotation

 def test_zero_slack_holds_reference_but_updates_all_joint_pi_without_reset(self):
  probe,measured,endpoint,rotation=self.fixture();reference=probe.press_joint_servo.reference.copy()
  actual=reference.copy();actual[2]+=.003;p,r=ik.tcp_pose(actual,(0,-.305,.75));measured[1]=(actual,measured[1][1],p,r)
  certificates=[{'rigid_gap_m':.0004,'other_hand_gap_m':.01,'rotation_chord_bound_m':0.},
                {'rigid_gap_m':.01,'other_hand_gap_m':.01,'rotation_chord_bound_m':0.}]
  before=probe.press_joint_servo.bias.copy()
  with patch.object(probe,'rigid_motion_certificate',side_effect=certificates) as check:
   _,_,at_end=probe.governed_prepress(measured,endpoint,rotation)
  self.assertFalse(at_end);self.assertEqual(probe.prepress_path['clock'],0.)
  np.testing.assert_array_equal(probe.press_joint_servo.reference,reference)
  self.assertAlmostEqual(probe.press_joint_servo.bias[2],before[2]-.00025)
  np.testing.assert_array_equal(probe.press_joint_servo.bias[[0,1,3,4,5]],before[[0,1,3,4,5]])
  self.assertFalse(probe.press_joint_servo.frozen);self.assertEqual(check.call_count,2)

 def test_reference_advance_uses_slack_not_half_absolute_error_budget(self):
  probe,measured,endpoint,rotation=self.fixture()
  safe={'rigid_gap_m':.0005,'other_hand_gap_m':.01,'rotation_chord_bound_m':0.}
  with patch.object(probe,'rigid_motion_certificate',return_value=safe):probe.governed_prepress(measured,endpoint,rotation)
  path=probe.prepress_path;s=.5
  q,_,_=ik.solve(path['start'][0]+s*(path['end'][0]-path['start'][0]),ik.matrix_to_quat_xyzw(rotation),path['reference'],base_pos=(0,-.305,.75),iters=220,tol=1e-6)
  current=ik.tcp_pose(q,(0,-.305,.75));path['clock']=PREPRESS_DURATION/2.;path['reference']=q.copy()
  measured[1]=(q,measured[1][1],current[0]+[0,.00024,0],current[1]);old_clock=path['clock']
  with patch.object(probe,'rigid_motion_certificate',return_value=safe):probe.governed_prepress(measured,endpoint,rotation)
  report=probe.trajectory_governor
  self.assertGreater(report['mesh_error_m'],report['tracking_budget_m']/2.)
  self.assertGreater(path['clock'],old_clock);self.assertLess(path['clock'],old_clock+.05)
  self.assertLessEqual(report['accepted_mesh_step_m'],report['allowed_mesh_step_m']+1e-12)

 def test_unsafe_correction_never_advances_clock_or_integrates(self):
  probe,measured,endpoint,rotation=self.fixture();before=probe.press_joint_servo.bias.copy()
  with patch.object(probe,'rigid_motion_certificate',side_effect=[{'rigid_gap_m':.01,'other_hand_gap_m':.01,'rotation_chord_bound_m':0.},
                                                               {'rigid_gap_m':.0001,'other_hand_gap_m':.01,'rotation_chord_bound_m':0.}]):
   _,_,at_end=probe.governed_prepress(measured,endpoint,rotation)
  self.assertFalse(at_end);self.assertEqual(probe.prepress_path['clock'],0.)
  np.testing.assert_array_equal(probe.press_joint_servo.bias,before);self.assertIn('clearance',probe.failure)

 def test_cursor_end_keeps_pi_active_without_relatched_or_reset_bias(self):
  probe,measured,endpoint,rotation=self.fixture();safe={'rigid_gap_m':.01,'other_hand_gap_m':.01,'rotation_chord_bound_m':0.}
  with patch.object(probe,'rigid_motion_certificate',return_value=safe):probe.governed_prepress(measured,endpoint,rotation)
  path=probe.prepress_path;path['clock']=PREPRESS_DURATION
  q,_,_=ik.solve(endpoint,ik.matrix_to_quat_xyzw(rotation),path['reference'],base_pos=(0,-.305,.75),iters=220,tol=1e-6)
  path['reference']=q.copy();actual=q.copy();actual[2]+=.003;p,r=ik.tcp_pose(actual,(0,-.305,.75))
  measured[1]=(actual,measured[1][1],p,r);before=probe.press_joint_servo.bias.copy()
  with patch.object(probe,'rigid_motion_certificate',return_value=safe):_,_,at_end=probe.governed_prepress(measured,endpoint,rotation)
  self.assertTrue(at_end);self.assertEqual(probe.press_servo_stage,'PREPRESS')
  self.assertAlmostEqual(probe.press_joint_servo.bias[2],before[2]-.00025)
  np.testing.assert_array_equal(probe.press_joint_servo.reference,q)

 def test_conditional_antiwindup_respects_native_and_action_limits(self):
  servo=JointEndpointCorrection();servo.latch(np.zeros(6));servo.bias[:]=.01
  before=servo.bias.copy();servo.track(np.zeros(6),-np.ones(6),.05,np.full(6,-.04),np.full(6,.01),velocity=np.zeros(6))
  np.testing.assert_array_equal(servo.bias,before)
  servo.track(np.zeros(6),np.ones(6),.05,np.full(6,-.04),np.full(6,.01),velocity=np.zeros(6))
  np.testing.assert_allclose(servo.bias,before-.00025)

 def test_native_numpy_certificate_telemetry_is_json_serializable(self):
  probe,measured,endpoint,rotation=self.fixture()
  safe={'rigid_gap_m':np.float64(.01),'other_hand_gap_m':np.float64(.01),'rotation_chord_bound_m':np.float64(0.)}
  with patch.object(probe,'rigid_motion_certificate',return_value=safe):
   probe.governed_prepress(measured,endpoint,rotation)
  restored=json.loads(json.dumps(probe.trajectory_governor))
  self.assertIs(restored['correction_safe'],True)


if __name__=='__main__':unittest.main()
