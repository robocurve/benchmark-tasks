import unittest
import numpy as np
from joint_tracking import JointTracking


class JointTrackingTests(unittest.TestCase):
    def test_only_right_arm_is_corrected_and_bias_is_bounded(self):
        t=JointTracking();ref=np.zeros(14);observed=np.ones(14)
        previous=np.zeros(6)
        for _ in range(100):
            action=t.action(ref,observed,enabled=True)
            self.assertTrue(np.all(abs(t.bias-previous)<=.00600001))
            self.assertTrue(np.all(abs(t.bias)<=.12000001))
            np.testing.assert_array_equal(action[:7],ref[:7])
            self.assertEqual(action[13],ref[13]);previous=t.bias.copy()

    def test_disabled_feedback_clears_memory_and_preserves_reference(self):
        t=JointTracking();t.action(np.ones(14),np.zeros(14),enabled=True)
        ref=np.linspace(0,1,14);np.testing.assert_array_equal(t.action(ref,np.zeros(14),enabled=False),ref)
        np.testing.assert_array_equal(t.bias,np.zeros(6));np.testing.assert_array_equal(t.integral,np.zeros(6))

    def test_rejects_bad_ordinary_observation(self):
        with self.assertRaises(ValueError):JointTracking().action(np.zeros(14),np.full(14,np.nan),enabled=True)
        with self.assertRaises(ValueError):JointTracking().action(np.zeros(14),np.zeros(12),enabled=True)


if __name__=='__main__':unittest.main()
