"""A puncture history alone must never count as a secured hem."""
import unittest
import numpy as np
from model import Model

class OutcomeTests(unittest.TestCase):
 def threaded(self):
  m=Model();i=m.index['pin_arm'];m.poses[i,:3]=[.3,.1,.9];m.R[i]=np.eye(3)
  for n,x in [('front_7_0',.035),('hem_7_1',.036)]:
   m.holes[n]=np.zeros(3);m.poses[m.index[n],:3]=[.3+x,.1,.9];m.R[m.index[n]]=np.eye(3)
  return m
 def test_open_or_held_pin_only_earns_partial_credit(self):
  m=self.threaded();self.assertEqual(m.score(),.5)
  m.latched=True;self.assertEqual(m.score(),.5)
  m.release=3.;self.assertEqual(m.score(),1.)
 def test_withdrawn_layers_do_not_count_as_a_repair(self):
  m=self.threaded();m.latched=True;m.release=3.
  m.disabled.add('hole_hem_7_1');self.assertEqual(m.score(),.25)
  m.disabled.add('hole_front_7_0');self.assertEqual(m.score(),0.)
 def test_puncture_count_does_not_replace_retention(self):
  m=self.threaded();m.latched=True;m.release=3.
  m.poses[m.index['front_7_0'],0]+=.1;m.poses[m.index['hem_7_1'],0]+=.1
  self.assertEqual(m.score(),0.)
 def test_back_layer_or_broken_fabric_invalidates_result(self):
  m=self.threaded();m.latched=True;m.release=3.;m.holes['back_7_0']=np.zeros(3)
  self.assertEqual(m.score(),0.)
  m=self.threaded();m.latched=True;m.release=3.;m.broken.add('hole_hem_7_1')
  self.assertEqual(m.score(),0.)
 def test_puncture_outside_loose_section_invalidates_result(self):
  m=self.threaded();m.latched=True;m.release=3.;m.holes['front_2_0']=np.zeros(3)
  self.assertEqual(m.score(),0.)

if __name__=='__main__':unittest.main()
