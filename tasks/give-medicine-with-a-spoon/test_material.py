"""Causal fluid/contact checks; CPU checks are not a native demo."""
import unittest
import numpy as np
from model import Model,MOUTH,REST
from liquid import Liquid

def ry(a):
 a=np.radians(a);return np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])

class Checks(unittest.TestCase):
 def test_stationary_spoon_preserves_its_liquid(self):
  m=Liquid()
  for _ in range(100):m.advance(REST,np.eye(3),.01,lambda a,b:None)
  self.assertAlmostEqual(m.volume*1e6,5.);self.assertEqual(m.delivered,0.)
 def test_tilting_away_spills_instead_of_delivering(self):
  m=Model()
  for _ in range(300):m.liquid.advance(REST,ry(40),.01,m.receiver)
  self.assertGreater(m.liquid.spilled*1e6,4.5);self.assertEqual(m.liquid.delivered,0.)
  self.assertLess(m.liquid.max_balance_error,1e-12)
 def test_pouring_at_cheek_does_not_count(self):
  m=Model()
  self.assertEqual(m.receiver(MOUTH+[-.01,.05,0],MOUTH+[.01,.05,0]),'spill')
 def test_lip_crossing_alone_is_not_collector_delivery(self):
  m=Model();self.assertIsNone(m.receiver(MOUTH+[-.001,0,0],MOUTH+[.001,0,0]))
  self.assertEqual(m.receiver(MOUTH+[.005,0,-.009],MOUTH+[.005,0,-.011]),'delivered')
 def test_pouring_through_top_of_head_cannot_deliver(self):
  m=Model();self.assertEqual(m.receiver(MOUTH+[.02,0,.03],MOUTH+[.02,0,.015]),'spill')
 def test_every_drop_is_accounted_for(self):
  m=Model()
  for _ in range(300):m.liquid.advance(MOUTH+[-.014,0,.01],ry(35),.01,m.receiver)
  q=m.liquid.measure();self.assertLess(q['balance_error_ml'],1e-9)
  self.assertAlmostEqual(sum(q[k] for k in ['in_spoon_ml','delivered_ml','spilled_ml','airborne_ml']),5.)
 def test_dose_does_not_override_contact_failure(self):
  m=Model();m.liquid.delivered=5e-6;m.release=2;m.fail('face contact');self.assertEqual(m.score(),0.)
 def test_empty_spoon_gets_no_progress(self):
  m=Model();m.liquid.volume=0;self.assertEqual(m.score(),0.)
 def test_spill_above_limit_zeroes_all_credit(self):
  m=Model();m.liquid.delivered=4.6e-6;m.liquid.spilled=.3e-6;m.release=2;self.assertEqual(m.score(),0.)

if __name__=='__main__':unittest.main()
