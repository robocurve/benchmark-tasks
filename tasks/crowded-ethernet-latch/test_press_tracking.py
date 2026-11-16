"""The tracking prefix holds the left robot ready and retains real guards."""
from unittest.mock import patch
import unittest
import numpy as np
from robot import PressTrackingProbe,PREPRESS_DISTANCE,PREPRESS_DURATION
from config import SERVICE,port
from test_robot import environment


class PressTrackingTests(unittest.TestCase):
 def test_right_only_probe_starts_without_grasp_and_keeps_left_ready(self):
  env=environment();probe=PressTrackingProbe(env,np.array(port(SERVICE)))
  self.assertEqual(probe.phase,'prepare_press');self.assertIsNone(probe.entry_joint_servo.reference)
  action,report=probe.command(.5)
  np.testing.assert_array_equal(action[:7],env._ready.numpy()[:7])
  self.assertIsNone(probe.hold_offset);self.assertIn('functional_handoff',report)
  self.assertAlmostEqual(PREPRESS_DISTANCE,.002);self.assertAlmostEqual(PREPRESS_DURATION,3.)

 def test_safety_is_fresh_without_left_reference_and_probe_does_not_press_early(self):
  probe=PressTrackingProbe(environment(),np.array(port(SERVICE)));probe.phase='prepress_endpoint'
  safe={'rigid_gap_m':.01,'other_hand_gap_m':.01,'right_non_target_rigid_gap_m':.01}
  with patch.object(probe,'stationary_hold_safety',return_value=safe) as safety:
   action,report=probe.command(1.)
  safety.assert_called_once();self.assertIsNone(probe.entry_joint_servo.reference)
  self.assertEqual(probe.phase,'prepress_endpoint');self.assertLess(report['press_depth_m'],0.)
  np.testing.assert_array_equal(action[:7],probe.env._ready.numpy()[:7])


if __name__=='__main__':unittest.main()
