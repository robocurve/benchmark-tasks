import unittest
from scoring import BreakdownScorer
from test_scoring import state,update
from crease import BOTTOM_LOW,BOTTOM_HIGH
class Tests(unittest.TestCase):
 def test_physical_limits_exclude_inward_opening(self):self.assertEqual(BOTTOM_LOW,0.);self.assertGreater(BOTTOM_HIGH,270.)
 def test_inward_motion_is_invalid_even_after_cut(self):
  t=BreakdownScorer(require_turnover=False);s=state(bottom_angles=[-10,0,0,0]);update(t,s);self.assertIn('outward-only',t.invalid)
 def test_missing_native_orientation_fails_closed(self):
  t=BreakdownScorer();update(t,state());self.assertIn('orientation',t.invalid)
 def test_bottom_up_initialization_is_not_turnover(self):
  t=BreakdownScorer();update(t,state(body_up_dot=-1.));self.assertIn('start upright',t.invalid)
 def test_requires_upright_then_measured_inverted_hold(self):
  t=BreakdownScorer();s=state(body_up_dot=1.);update(t,s);self.assertFalse(t.turnover_complete)
  for _ in range(12):s.update(time=s['time']+.05,body_up_dot=-1.);update(t,s)
  self.assertTrue(t.turnover_complete);self.assertEqual(t.value,0.)
if __name__=='__main__':unittest.main()
