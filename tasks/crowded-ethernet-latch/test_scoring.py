import unittest
from scoring import EthernetScorer,HOLD_S,TUG_S,LINGER_S

HOME=.0175


class ScoringTest(unittest.TestCase):
 def sample(self,t,**kw):
  s=dict(time=t,clip_travel=0.,source_insertion=HOME,target_insertion=-.02,target_lateral_error=.014,target_angle=0.,
         speed=0.,height=.95,max_neighbour_withdrawal=0.,neighbour_median_depth=HOME,hands_clear=False,hook_retained=False)
  s.update(kw);return s

 def run_seq(self,m,t0,n,**kw):
  t=t0
  for _ in range(n):m.update(self.sample(t,**kw));t=round(t+.05,10)
  return t

 def home(self,**kw):
  return dict(source_insertion=-.02,target_insertion=HOME-.0005,target_lateral_error=.0003,**kw)

 def test_untouched_scores_zero(self):
  m=EthernetScorer();self.run_seq(m,0.,10);self.assertEqual(m.value,0.)

 def test_out_scores_quarter(self):
  m=EthernetScorer();self.run_seq(m,0.,3,source_insertion=-.003,target_insertion=-.02);self.assertEqual(m.value,.25)

 def test_home_scores_half_but_shallow_does_not(self):
  m=EthernetScorer();t=self.run_seq(m,0.,2,source_insertion=-.003)
  self.run_seq(m,t,2,source_insertion=-.02,target_insertion=HOME-.003,target_lateral_error=.0003);self.assertEqual(m.value,.25)
  m=EthernetScorer();t=self.run_seq(m,0.,2,source_insertion=-.003);self.run_seq(m,t,2,**self.home());self.assertEqual(m.value,.5)

 def test_hold_then_tug_then_linger(self):
  m=EthernetScorer();t=self.run_seq(m,0.,2,source_insertion=-.003)
  t=self.run_seq(m,t,int(HOLD_S/.05)+1,**self.home(hands_clear=True))
  self.assertEqual(m.value,.75);self.assertTrue(m.tug_active)
  t=self.run_seq(m,t,int(TUG_S/.05)+1,**self.home(hands_clear=True))
  self.assertEqual(m.value,1.);self.assertFalse(m.finished(t-.05))
  t=self.run_seq(m,t,int(LINGER_S/.05)+1,**self.home(hands_clear=True));self.assertTrue(m.finished(t-.05))

 def test_unlatched_plug_fails_tug_and_must_rehold(self):
  m=EthernetScorer();t=self.run_seq(m,0.,2,source_insertion=-.003)
  t=self.run_seq(m,t,int(HOLD_S/.05)+1,**self.home(hands_clear=True))
  m.update(self.sample(t,source_insertion=-.02,target_insertion=HOME-.0035,target_lateral_error=.0003,hands_clear=True))
  self.assertFalse(m.tug_active);self.assertEqual(m.hold,0.);self.assertEqual(m.value,.25)

 def test_hands_on_plug_never_reaches_three_quarters(self):
  m=EthernetScorer();t=self.run_seq(m,0.,2,source_insertion=-.003)
  self.run_seq(m,t,200,**self.home(hands_clear=False));self.assertEqual(m.value,.5)

 def test_neighbour_pulled_invalidates(self):
  m=EthernetScorer();m.update(self.sample(0,max_neighbour_withdrawal=.0025));self.assertEqual(m.value,0.);self.assertIsNotNone(m.invalid)

 def test_broken_tab_invalidates(self):
  m=EthernetScorer();m.update(self.sample(0,clip_travel=.0027));self.assertIsNotNone(m.invalid)

 def test_drop_invalidates(self):
  m=EthernetScorer();m.update(self.sample(0,height=.70));self.assertIsNotNone(m.invalid)

 def test_missing_samples_invalidate(self):
  m=EthernetScorer();m.update(self.sample(0));m.update(self.sample(.2));self.assertIsNotNone(m.invalid)


if __name__=='__main__':unittest.main()
