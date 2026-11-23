"""Bounded breakaway damping and native-velocity controller regressions."""
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from trajectory_control import bounded_joint_pid_step
from feedback import JointEndpointCorrection
from robot import PressTrackingProbe
from test_robot import Array,environment
import test_governed_prepress as governed_tests
from config import SERVICE,port


class JointPidTests(unittest.TestCase):
 def step(self,bias,error,velocity):
  return bounded_joint_pid_step(np.full(6,bias),np.full(6,error),np.zeros(6),np.full(6,velocity),.05)

 def test_toward_breakaway_freezes_integral_and_d_opposes_velocity(self):
  bias,d,total,mask=self.step(.002,.00056,.016)
  np.testing.assert_array_equal(bias,np.full(6,.002));self.assertFalse(mask.any())
  np.testing.assert_allclose(d,-.0008);np.testing.assert_allclose(total,.0012)

 def test_away_motion_keeps_restorative_error_directed_integral(self):
  bias,_,_,mask=self.step(.002,.001,-.01)
  np.testing.assert_allclose(bias,.0021);self.assertTrue(mask.all())
  bias,_,_,mask=self.step(.00001,-.001,.01)
  np.testing.assert_allclose(bias,-.00009);self.assertTrue(mask.all())

 def test_rest_normal_integral_and_total_twenty_milliradian_cap(self):
  bias,d,total,mask=self.step(0,.001,0)
  np.testing.assert_allclose(bias,.0001);np.testing.assert_allclose(d,0);self.assertTrue(mask.all())
  bias,d,total,_=self.step(.02,-.001,-1)
  np.testing.assert_allclose(total,.02);np.testing.assert_allclose(d,.002)
  self.assertTrue(np.all(np.abs(total)<=.02))

 def test_d_is_transient_not_integrated_and_limits_include_it(self):
  servo=JointEndpointCorrection();servo.latch(np.zeros(6));servo.bias[:]=.001
  # Toward motion too slow to freeze I: damping makes final command cross the
  # lower action limit even though the integral-only command stays inside.
  servo.track(np.zeros(6),np.full(6,.02),.05,np.full(6,.00074),np.ones(6),velocity=np.full(6,-.001))
  np.testing.assert_allclose(servo.bias,.00075)
  np.testing.assert_allclose(servo.damping,.00005)
  servo.bias[:]=.001
  servo.track(np.zeros(6),np.full(6,.02),.05,np.full(6,.00074),np.ones(6),velocity=np.full(6,.001))
  np.testing.assert_allclose(servo.bias,.001) # away motion unwinds, but D makes .00070 unsafe
  self.assertFalse(servo.integral_mask.any())
  np.testing.assert_allclose(servo.command(),.00095)
  servo.track(np.zeros(6),np.zeros(6),.05,-np.ones(6),np.ones(6),velocity=np.zeros(6))
  np.testing.assert_allclose(servo.bias,.001);np.testing.assert_allclose(servo.damping,0)

 def test_requires_finite_native_velocity(self):
  for velocity in (None,np.full(6,np.nan),np.zeros(5)):
   with self.assertRaises(ValueError):bounded_joint_pid_step(np.zeros(6),np.zeros(6),np.zeros(6),velocity,.05)
  probe=PressTrackingProbe(environment(),np.array(port(SERVICE)))
  del probe.base.scene['right_robot'].data.joint_vel
  _,report=probe.command(.5)
  self.assertIn('native right joint velocity',report['failure'])

 def test_total_correction_saturation_also_blocks_outward_integral(self):
  servo=JointEndpointCorrection();servo.latch(np.zeros(6));servo.bias[:]=.019
  # Positive away velocity generates negative D; proposed negative I would
  # wind farther into total-correction saturation and must be rejected.
  servo.bias[:]=-.019
  servo.track(np.zeros(6),np.full(6,.1),.05,-np.ones(6),np.ones(6),velocity=np.full(6,.04))
  np.testing.assert_allclose(servo.bias,-.019)
  np.testing.assert_allclose(servo.command(),-.02)
  self.assertFalse(servo.integral_mask.any())

 @unittest.skipUnless(Path('/private/tmp/ethernet-tracking027.jsonl').exists(),'local native027 artifact')
 def test_027_q4_away_motion_does_not_veto_corrective_integral(self):
  rows=[json.loads(line) for line in Path('/private/tmp/ethernet-tracking027.jsonl').read_text().splitlines()]
  witnessed=0
  for row in rows:
   if row['phase']!='prepress_endpoint':continue
   control=row['robot_control'];reference=np.asarray(control['press_joint_reference'])
   actual=np.asarray(control['arms'][1]['actual_joints']);old=np.asarray(control['press_joint_bias_rad'])
   velocity=np.asarray(control['right_native_joint_velocity_rad_s'])
   if reference[3]-actual[3]<-.002 and velocity[3]>0 and old[3]<=0:
    bias,d,total,mask=bounded_joint_pid_step(old,reference,actual,velocity,.05)
    self.assertTrue(mask[3]);self.assertLess(bias[3],old[3]);self.assertLess(d[3],0)
    self.assertLessEqual(abs(total[3]),.02);witnessed+=1
  self.assertGreater(witnessed,0)

 def test_proxy_native_velocity_selected_instead_of_finite_difference(self):
  env=environment();probe=PressTrackingProbe(env,np.array(port(SERVICE)))
  env._base.scene['right_robot'].data.joint_vel=SimpleNamespace(torch=Array([[.01,.02,.03,.04,.05,.06,9,9]]))
  probe.last_right_joints=probe.arm('right')[0].copy()
  _,report=probe.command(.5)
  np.testing.assert_allclose(report['right_joint_velocity_rad_s'],0)
  np.testing.assert_allclose(report['right_native_joint_velocity_rad_s'],[.01,.02,.03,.04,.05,.06])
  probe,measured,endpoint,rotation=governed_tests.GovernedPrepressTests().fixture()
  probe.right_native_joint_velocity=np.full(6,.01);probe.right_joint_velocity=np.full(6,-99.)
  safe={'rigid_gap_m':.01,'other_hand_gap_m':.01,'rotation_chord_bound_m':0.}
  with patch.object(probe,'rigid_motion_certificate',return_value=safe):probe.governed_prepress(measured,endpoint,rotation)
  np.testing.assert_allclose(probe.press_joint_servo.damping,-.0005)
  json.dumps(report)

 @unittest.skipUnless(Path('/private/tmp/ethernet-tracking026.jsonl').exists(),'local native026 artifact')
 def test_026_breakaway_direction_replay(self):
  rows=[json.loads(line) for line in Path('/private/tmp/ethernet-tracking026.jsonl').read_text().splitlines()]
  row=min((r for r in rows if r['phase']=='prepress_endpoint'),key=lambda r:abs(r['robot_control']['phase_age_s']-5.15))
  control=row['robot_control'];reference=np.asarray(control['press_joint_reference'])
  actual=np.asarray(control['arms'][1]['actual_joints']);old=np.asarray(control['press_joint_bias_rad'])
  # 026 predates native velocity telemetry. Its recorded finite difference is
  # used solely for this offline directional replay, never as controller data.
  velocity=np.asarray(control['right_joint_velocity_rad_s'])
  bias,d,total,mask=bounded_joint_pid_step(old,reference,actual,velocity,.05)
  self.assertGreater(velocity[1]*(reference[1]-actual[1]),0)
  self.assertFalse(mask[1]);self.assertEqual(bias[1],old[1]);self.assertLess(d[1]*velocity[1],0)


if __name__=='__main__':unittest.main()
