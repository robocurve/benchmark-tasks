import unittest
from scoring import EthernetScorer
class ScorerTests(unittest.TestCase):
 def sample(self,t,**kw):
  s=dict(time=t,clip_travel=0.,source_insertion=.015,target_insertion=-.02,target_lateral_error=.03,target_angle=0.,speed=0.,height=.95,max_neighbour_withdrawal=0.,hands_clear=True,hook_retained=False);s.update(kw);return s
 def test_pulling_without_release_invalid(self):
  m=EthernetScorer();m.update(self.sample(0));m.update(self.sample(.05,source_insertion=-.003));self.assertEqual(m.value,0.);self.assertIsNotNone(m.invalid)
 def test_latch_must_reextend_and_be_retained(self):
  m=EthernetScorer();m.update(self.sample(0,clip_travel=.002));m.update(self.sample(.05,source_insertion=-.003));
  for i in range(1,70):m.update(self.sample(.05+i*.05,source_insertion=-.04,target_insertion=.012,target_lateral_error=0.,clip_travel=.002,hook_retained=True))
  self.assertEqual(m.value,.25)
 def test_released_complete_sequence(self):
  m=EthernetScorer();m.update(self.sample(0,clip_travel=.002));m.update(self.sample(.05,source_insertion=-.003))
  for i in range(1,45):m.update(self.sample(.05+i*.05,source_insertion=-.04,target_insertion=.012,target_lateral_error=0.,hook_retained=True))
  self.assertEqual(m.value,1.)
 def test_neighbour_disturbance_invalidates(self):
  m=EthernetScorer();m.update(self.sample(0,max_neighbour_withdrawal=.02));self.assertIsNotNone(m.invalid)
