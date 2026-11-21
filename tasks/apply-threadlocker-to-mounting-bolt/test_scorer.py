import unittest
from dataclasses import replace
from scorer import Scorer,Evidence

def state(**kw):return replace(Evidence(0.,0.,True,False,True,True,True,True,0.),**kw)
class ScoringTests(unittest.TestCase):
 def test_partial_and_root_requirement(self):
  s=Scorer();self.assertEqual(s.update(state()),0.)
  self.assertEqual(s.update(state(coverage=1,root_coverage=0)),0.)
  self.assertEqual(s.update(state(coverage=.34,root_coverage=.34)),.25)
  self.assertEqual(s.update(state(coverage=.7,root_coverage=.7)),.5)
 def test_full_requires_clear_application_and_dwell(self):
  e=state(coverage=1,root_coverage=1,applicator_clear=True);s=Scorer()
  for i in range(9):self.assertEqual(s.update(e),.5)
  self.assertEqual(s.update(e),1.)
  self.assertEqual(s.update(replace(e,band_unobstructed=False)),.5)
  self.assertEqual(s.dwell,0)
 def test_partial_cannot_substitute_for_pose(self):
  for field in ['applicator_clear','band_unobstructed','stable']:
   s=Scorer();e=state(coverage=1,root_coverage=1,applicator_clear=True);e=replace(e,**{field:False})
   for _ in range(15):s.update(e)
   self.assertEqual(s.score,.5)
 def test_permanent_invalid_and_fresh_reset(self):
  for kw in [{'head_clean':False},{'inside_workspace':False},{'intact':False},{'chassis_tilt_deg':15.1},{'coverage':float('nan')},{'coverage':1.01}]:
   s=Scorer();s.update(state(**kw));self.assertEqual(s.update(state(coverage=1,root_coverage=1,applicator_clear=True)),0.);self.assertTrue(s.invalid)
  self.assertFalse(Scorer().invalid)
if __name__=='__main__':unittest.main()
