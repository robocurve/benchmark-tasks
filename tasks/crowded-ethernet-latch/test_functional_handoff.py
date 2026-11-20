"""Recorded plateau and measured-progress functional handoff regressions."""
from copy import deepcopy
from unittest.mock import patch
import unittest
import numpy as np
from yam_arena import ik
from yam_arena.urdf import quat_xyzw_to_matrix
from robot import ReleaseProbe,PREPRESS_DISTANCE,PREPRESS_DURATION
from test_robot import environment
from config import SERVICE,port
from handoff_fixtures import CASES


class FunctionalHandoffTests(unittest.TestCase):
 def test_mesh_error_uses_latched_pose_when_live_goal_drifts(self):
  for phase,age in (('approach_above_clip',4.),('prepress_endpoint',PREPRESS_DURATION)):
   probe=ReleaseProbe(environment(),np.array(port(SERVICE)));probe.phase=phase
   measured=[probe.arm(s) for s in ('left','right')];plug=np.r_[port(SERVICE),0,0,0,1.]
   latched=np.array((.418,-.025,.980));rotation=probe.pressR
   if phase=='prepress_endpoint':
    live=plug[:3]+probe.local_contact+[0,0,.0041-PREPRESS_DISTANCE]-rotation@probe.point
    reference,pe,re=ik.solve(live+[.0001,0,0],ik.matrix_to_quat_xyzw(rotation),measured[1][0],base_pos=(0,-.305,.75),iters=350,tol=1e-6)
    self.assertLess(pe,1e-6)
    latched,rotation=ik.tcp_pose(reference,(0,-.305,.75))
    probe.press_joint_servo.latch(reference);probe.press_servo_stage='PREPRESS'
    probe.right_native_joint_velocity=np.zeros(6)
    measured[1]=(reference,np.array((-.028,-.028)),latched.copy(),rotation.copy())
    probe.prepress_path={'clock':PREPRESS_DURATION,'reference':reference.copy(),
                        'start':(latched+[0,0,PREPRESS_DISTANCE],rotation),'end':(latched,rotation),'target':latched.copy()}
    self.assertGreater(np.linalg.norm(latched-live),0.)
    self.assertLess(np.linalg.norm(latched-live),.0002)
   targets=(measured[0][2],measured[0][3],.1,measured[1][2],measured[1][3],.6)
   safe={'rigid_gap_m':.01,'other_hand_gap_m':.01,'rotation_chord_bound_m':0.}
   with patch.object(probe,'stationary_press_correction',return_value=(latched,rotation)),patch.object(probe,'rigid_motion_certificate',return_value=safe),patch.object(probe,'press_endpoint_ready',return_value=False) as ready:
    probe.release_targets(age,age,phase,plug,np.eye(3),0.,measured,targets)
   self.assertIsNone(probe.failure)
   np.testing.assert_array_equal(ready.call_args.args[1],latched)

 def fixture(self,name='safe020'):
  case=deepcopy(CASES[name]);probe=ReleaseProbe(environment(),np.array(port(SERVICE)))
  measured=[tuple(np.array(v) for v in arm) for arm in case['measured']]
  probe.press_joint_servo.latch(case['reference']);probe.press_servo_stage='HOVER';probe.right_joint_velocity=np.zeros(6)
  position,rotation,plug,point,target=[np.array(case[k]) for k in ('position','rotation','plug','point','next_target')]
  for _ in range(5):self.assertTrue(probe.press_endpoint_ready(measured,position,rotation,'HOVER'))
  def certificate(arms,*args):
   kind='actual_certificate' if arms is measured else 'nominal_certificate'
   probe.press_approach_certificate=deepcopy(case[kind])
   c=probe.press_approach_certificate
   return min(c['rigid_gap_m'],c['other_hand_gap_m'])-c['rotation_chord_bound_m']>=.0002
  probe.rigid_motion_certificate=lambda *a,**kw:{k:case['nominal_certificate'][k] for k in ('rigid_gap_m','other_hand_gap_m','rotation_chord_bound_m')}
  args=(measured,target,rotation,'prepress_endpoint','HOVER',position,rotation,plug,quat_xyzw_to_matrix(plug[3:]),point,())
  return probe,case,args,certificate

 def test_accepts_recorded_020_safe_plateau_with_measured_mesh_reserve(self):
  probe,case,args,certificate=self.fixture()
  with patch.object(probe,'certify_press_path',side_effect=certificate):self.assertTrue(probe.functional_press_certificate(*args))
  report=probe.functional_handoff_report
  self.assertAlmostEqual(report['max_mesh_error_5_frames_m'],.0002408,delta=.000002)
  self.assertGreater(report['non_target_rigid_and_hand_reserve_m'],.0006)
  self.assertGreater(probe.hover_departure['planned_down_m'],.0005)

 def test_rejects_recorded_019_partial_compensation(self):
  probe,case,args,certificate=self.fixture('partial019')
  with patch.object(probe,'certify_press_path',side_effect=certificate):self.assertFalse(probe.functional_press_certificate(*args))
  self.assertLess(min(case['nominal_certificate']['rigid_gap_m'],case['nominal_certificate']['other_hand_gap_m'])-max(probe.functional_errors),.0002)

 def test_rejects_unsafe_actual_path_or_insufficient_nominal_reserve(self):
  for actual,nominal in ((.0001,.001),(.001,.00035)):
   with self.subTest(actual=actual,nominal=nominal):
    probe,case,args,certificate=self.fixture()
    case['actual_certificate']['rigid_gap_m']=actual;case['nominal_certificate']['rigid_gap_m']=nominal
    with patch.object(probe,'certify_press_path',side_effect=certificate):self.assertFalse(probe.functional_press_certificate(*args))

 def test_maximum_over_five_frames_and_velocity_guard_are_required(self):
  probe,case,args,certificate=self.fixture();probe.functional_errors[0]=.001
  with patch.object(probe,'certify_press_path',side_effect=certificate):self.assertFalse(probe.functional_press_certificate(*args))
  probe.right_joint_velocity[2]=.003
  self.assertFalse(probe.press_endpoint_ready(args[0],args[5],args[6],'HOVER'))
  self.assertEqual(probe.functional_errors,[])

 def test_wrong_next_direction_and_wire_reserve_claim_are_rejected(self):
  probe,case,args,certificate=self.fixture();up=list(args);up[1]=args[5]+args[8][:,2]*.0005
  with patch.object(probe,'certify_press_path',side_effect=certificate):self.assertFalse(probe.functional_press_certificate(*up))
  probe,case,args,certificate=self.fixture();case['actual_certificate']['wire_gap_m']=0.;case['nominal_certificate']['wire_gap_m']=0.
  with patch.object(probe,'certify_press_path',side_effect=certificate):self.assertTrue(probe.functional_press_certificate(*args))
  self.assertIn('wire_scope',probe.functional_handoff_report)

 def test_prepress_requires_actual_witness_and_q3_progress_with_last_motion_negative(self):
  probe,case,args,certificate=self.fixture()
  with patch.object(probe,'certify_press_path',side_effect=certificate):self.assertTrue(probe.functional_press_certificate(*args))
  pre=list(args);pre[4]='PREPRESS';pre[5]=args[1];pre[1]=args[1]-args[8][:,2]*.0059
  probe.press_servo_stage='PREPRESS'
  with patch.object(probe,'certify_press_path',side_effect=certificate) as check:
   self.assertFalse(probe.functional_press_certificate(*pre));check.assert_not_called()
  q,_,_=ik.solve(pre[5],ik.matrix_to_quat_xyzw(pre[6]),args[0][1][0],base_pos=(0,-.305,.75),iters=220,tol=1e-6)
  p,r=ik.tcp_pose(q,(0,-.305,.75));pre[0]=[args[0][0],(q,args[0][1][1],p,r)]
  probe.functional_errors=[0.]*5;probe.prepress_last_q3_movement=.00002
  with patch.object(probe,'certify_press_path',side_effect=certificate):self.assertFalse(probe.functional_press_certificate(*pre))
  probe.prepress_last_q3_movement=-.00002
  with patch.object(probe,'certify_press_path',side_effect=certificate):self.assertTrue(probe.functional_press_certificate(*pre))


if __name__=='__main__':unittest.main()
