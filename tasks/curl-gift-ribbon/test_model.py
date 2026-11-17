"""Counterfactuals: work at the edge and retained geometry are both necessary."""
import unittest
import numpy as np
from model import Model,EDGE,DS
from common import TABLE,down
class Checks(unittest.TestCase):
 def test_no_contact_no_plasticity(self):
  m=Model();m.load=2.;m.plastic_contact(.05)
  self.assertEqual(float(m.memory.sum()),0.)
 def test_static_loaded_edge_does_not_accumulate_work(self):
  m=Model();lo,hi=6,74;m.grips.held={'left':{'index':hi},'right':{'index':lo}}
  m.p[:,0]=EDGE[0];m.p[:,2]=EDGE[2]-.08*abs(m.p[:,1])/.24
  for _ in range(100):m.plastic_contact(.05)
  self.assertEqual(m.contact_distance,0.);self.assertEqual(float(m.memory.sum()),0.)
 def test_memory_is_not_success(self):
  m=Model();m.memory[:]=.65;m.release=4
  self.assertEqual(m.score(),0.)
 def test_actual_turns_release_and_damage(self):
  m=Model();angle=np.arange(len(m.p))*.2;r=.015
  m.p[:,0]=.22+angle*.002;m.p[:,1]=-.15+r*np.cos(angle);m.p[:,2]=TABLE+.0155+r*np.sin(angle)
  m.release=0;self.assertEqual(m.score(),.75)
  m.release=4;self.assertEqual(m.score(),1.)
  m.p[:,1]=-.15-r*np.cos(angle);self.assertEqual(m.score(),1.)
  before=m.measure()['retained_turns'];p=m.p.copy()
  m.p[:,0]=.22+p[:,1]+.15;m.p[:,1]=-.15-(p[:,0]-.22)
  self.assertAlmostEqual(m.measure()['retained_turns'],before)
  m.p[:]=p
  m.p[:,2]+=.05;self.assertEqual(m.score(),0.)
  m.p[:,2]-=.05
  m.failed=True;self.assertEqual(m.score(),0.)
 def test_overlapping_loops_cannot_pass(self):
  m=Model();t=np.arange(len(m.p))*.2
  m.p[:,0]=.22;m.p[:,1]=-.15+.015*np.cos(t);m.p[:,2]=TABLE+.0155+.015*np.sin(t);m.release=4
  self.assertGreater(m.overlap_mm(),9.);self.assertEqual(m.score(),0.)
if __name__=='__main__':unittest.main()
