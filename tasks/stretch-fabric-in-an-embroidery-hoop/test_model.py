"""Material counterfactuals: screw tightening is not automatic flattening."""
import unittest
import numpy as np
from common import down
from model import Model,CENTER
HANDS={s:dict(tcp=np.array([.15,y,1.1]),rotation=down(),opening=1.) for s,y in [('left',.3),('right',-.3)]}
class Checks(unittest.TestCase):
 def test_idle_cloth_stays_slack(self):
  m=Model()
  for _ in range(80):m.update(HANDS)
  self.assertGreater(m.measure()['max_central_deflection_mm'],5.);self.assertEqual(m.score(),0.)
 def test_tightening_alone_does_not_flatten(self):
  m=Model();m.tightness=1.;m.turns=.25
  for _ in range(80):m.update(HANDS)
  self.assertGreater(m.measure()['max_central_deflection_mm'],5.);self.assertEqual(m.score(),.5)
 def test_flattening_without_clamp_is_not_full_credit(self):
  m=Model();m.soft.p[:,2]=CENTER[2];m.release=4
  self.assertEqual(m.score(),.25)
  m.tightness=1;self.assertEqual(m.score(),1.)
  m.release=0;self.assertEqual(m.score(),.75)
  m.failed=True;self.assertEqual(m.score(),0.)
if __name__=='__main__':unittest.main()
