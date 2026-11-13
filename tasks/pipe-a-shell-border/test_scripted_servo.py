"""Check disclosed demo compensation without importing the GPU simulator."""
import unittest
import numpy as np
from scripted_servo import ScriptedServo


class ScriptedServoTests(unittest.TestCase):
    def test_unassisted_phases_preserve_actions(self):
        policy=ScriptedServo();reference=np.arange(14)/100
        # No tool state is read for pickup/release phases.
        result=policy.command({},reference,reference,None,None)
        np.testing.assert_array_equal(result,reference)
        self.assertFalse(policy.last['active'])
        result[0]+=1
        self.assertEqual(reference[0],0)

    def test_invalid_tool_measurement_fails_before_ik(self):
        policy=ScriptedServo();ref=np.zeros(14)
        with self.assertRaises(ValueError):
            policy.command({'scripted_tool_target':{'barrel_position_m':[0,0,0]}},ref,ref,np.zeros(6),np.zeros(7))

if __name__=='__main__':unittest.main()
