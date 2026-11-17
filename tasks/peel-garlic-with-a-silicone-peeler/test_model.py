"""Counterfactual checks for the material work and observable endpoints."""
import unittest
import numpy as np
from common import TABLE,down
from model import Model,BOARD

def hand(x=.34,z=1.0):return dict(tcp=np.array([x,0,z]),rotation=down(),opening=.05)
class Checks(unittest.TestCase):
 def test_stationary_compression_does_not_peel(self):
  m=Model();h={'right':hand(z=BOARD+.035-.012+.01284)}
  for _ in range(120):m.update(h)
  self.assertEqual(m.measure()['peeled_fraction'],0.);self.assertEqual(m.score(),0.)
 def test_unloaded_motion_does_not_peel(self):
  m=Model()
  for i in range(120):m.update({'right':hand(.34+.04*np.sin(i/8))})
  self.assertEqual(m.measure()['peeled_fraction'],0.)
 def test_crushing_cannot_earn_credit(self):
  m=Model();m.skin[:]=0;m.inside=False;m.clove=np.array([.2,-.1,BOARD+.0105]);m.release=4;m.crushed=True
  self.assertEqual(m.score(),0.)
 def test_recovery_and_separation_required(self):
  m=Model();m.skin[:]=0;m.release=4
  self.assertEqual(m.score(),.25)
  m.inside=False;self.assertEqual(m.score(),.25)
  m.clove=np.array([.2,-.1,BOARD+.0105]);self.assertEqual(m.score(),1.)
  m.release=0;self.assertEqual(m.score(),.75)
 def test_final_partial_skin_thresholds(self):
  m=Model();m.inside=False;m.clove=np.array([.2,-.1,BOARD+.0105]);m.release=4
  m.skin.flat[:64]=0;self.assertEqual(m.score(),.25)
  m.skin.flat[:96]=0;self.assertEqual(m.score(),.5)
  m.skin.flat[:122]=0;self.assertEqual(m.score(),1.)
if __name__=='__main__':unittest.main()
