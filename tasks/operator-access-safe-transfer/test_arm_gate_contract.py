"""Primary official score gate: release, vacancy, one snapshot, and latches."""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
HERE=Path(__file__).resolve().parent
T=HERE if (HERE/'arm_safety.py').exists() else HERE.parent/'benchmark_source/tasks/operator-access-safe-transfer'
sys.path.insert(0,str(T))
import arm_safety as a

class ScoreGate(unittest.TestCase):
    def setUp(self):a.reset_for_new_trial()
    def test_supported_stage_before_vacancy_has_no_credit(self):
        with patch.object(a,'assess_poses',return_value=True):
            self.assertEqual(a.enforce(.5,{'stage_qualified':True},{}),0.)

    def test_staging_after_vacancy_has_partial_credit(self):
        a._STATE['vacancy_qualified']=True
        with patch.object(a,'assess_poses',return_value=True):
            self.assertEqual(a.enforce(.5,{'stage_qualified':True},{}),.5)

    def test_full_credit_requires_vacancy(self):
        with patch.object(a,'assess_poses',return_value=True):
            self.assertEqual(a.enforce(1.,{'stage_qualified':True},{}),0.)
        a._STATE['vacancy_qualified']=True
        with patch.object(a,'assess_poses',return_value=True):
            self.assertEqual(a.enforce(1.,{'stage_qualified':True},{}),1.)

    def test_passed_tensor_snapshot_is_not_reread(self):
        a._STATE['vacancy_qualified']=True
        with patch.object(a,'_physics_poses',side_effect=RuntimeError('MUST_NOT_CALL')), \
             patch.object(a,'assess_poses',return_value=True):
            self.assertEqual(a.enforce(1.,{'stage_qualified':True},{'snapshot':'provided'}),1.)

    def test_violation_always_latches_even_if_later_healthy(self):
        a.latch_external('PERSONNEL_GAP_VIOLATION')
        a._STATE['vacancy_qualified']=True
        with patch.object(a,'assess_poses',return_value=True):
            self.assertEqual(a.enforce(1.,{'stage_qualified':True},{}),0.)
            self.assertEqual(a.enforce(1.,{'stage_qualified':True},{}),0.)
        self.assertEqual(a.state()['reason'],'PERSONNEL_GAP_VIOLATION')

if __name__=='__main__':unittest.main()
