"""Offline guard checks for recorded-trial feedforward; no simulator or API."""
import copy
import unittest
import numpy as np
from learn_tracking import compile_feedforward


class LearningChecks(unittest.TestCase):
    def fixture(self):
        frames=[{'action14':[0.]*14,'scripted_tool_target':{'piston_m':.001}} for _ in range(12)]
        trace=[{'step':0}]+[{'step':i+1,'reference_action14':[0.]*14,
            'applied_action14':[.8]*14,'observed_action14':[0.]*14} for i in range(12)]
        return {'frames':frames},trace

    def test_only_bounded_barrel_joint_offsets_and_recipe_unchanged(self):
        plan,trace=self.fixture();before=copy.deepcopy(plan)
        result=compile_feedforward(plan,trace)
        offsets=np.asarray([f['joint_feedforward14'] for f in result['frames']])
        self.assertEqual(plan,before)
        self.assertTrue(np.all(offsets[:,:7]==0) and np.all(offsets[:,13]==0))
        self.assertLessEqual(float(np.max(np.abs(offsets))),.16)
        for old,new in zip(plan['frames'],result['frames']):
            self.assertEqual(old['scripted_tool_target'],new['scripted_tool_target'])
            self.assertEqual(old['action14'],new['action14'])

    def test_incomplete_or_mismatched_reference_fails(self):
        plan,trace=self.fixture()
        with self.assertRaises(ValueError):compile_feedforward(plan,trace[:-1])
        trace[5]['reference_action14'][8]=.01
        with self.assertRaises(ValueError):compile_feedforward(plan,trace)

    def test_no_offsets_outside_calibrated_tool_phases(self):
        plan,trace=self.fixture();del plan['frames'][6]['scripted_tool_target']
        result=compile_feedforward(plan,trace)
        self.assertEqual(result['frames'][6]['joint_feedforward14'],[0.]*14)

if __name__=='__main__':unittest.main()
