"""Analytic scorer fixtures, never used as a simulation policy."""
import copy,unittest
import numpy as np
from chain_task.scoring import *

def fixture(count=3):
 b={'stand':Body(STAND_POS.copy(),np.eye(3),np.zeros(3),np.zeros(3))}
 for i in range(3):
  if i<count:r=rz(np.deg2rad(-10-90*(i%2)))@rx(np.pi/2);p=np.array([.42,0,TABLE+.2225+.00475-.030-i*.0555])
  else:r=np.eye(3);p=np.array([.25,.2-i*.2,TABLE+RADIUS])
  b['link'+str(i)]=Body(p,r,np.zeros(3),np.zeros(3));b['gate'+str(i)]=Body(p+r@GATE_OFFSET,r,np.zeros(3),np.zeros(3))
 return b
BOUNDS={str(i):(np.array([-2,-2,2.]),np.array([-1.9,-1.9,2.1])) for i in range(6)}
def settled(b,h=None,bounds=BOUNDS):
 h=History() if h is None else h
 for _ in range(41):h.update(b,bounds,.05)
 return h
class ScorerTests(unittest.TestCase):
 def test_serial_prefixes_and_empty(self):
  for n in range(4):self.assertAlmostEqual(settled(fixture(n)).score,n/3.)
 def test_continuous_two_seconds_and_motion(self):
  b=fixture();h=History()
  for _ in range(39):h.update(b,BOUNDS,.05)
  self.assertEqual(h.score,0.);b['gate2'].v=np.array([.02,0,0]);self.assertEqual(h.update(b,BOUNDS,.05),0.)
 def test_held_result_and_table_support(self):
  b=fixture();h=settled(b);bounds=copy.deepcopy(BOUNDS);p=b['link2'].p;bounds['0']=(p-.02,p+.02);self.assertEqual(h.update(b,bounds,.05),0.)
  b=fixture();dz=TABLE+.019-(b['link2'].p[2]-.030-RADIUS)
  for n in b:
   if n!='stand':b[n].p=b[n].p+[0,0,dz]
  self.assertLess(settled(b).score,1.)
 def test_reopening_loses_credit(self):
  b=fixture();h=settled(b);f=b['link1'];r=f.r@rz(np.deg2rad(20));b['gate1'].r=r;b['gate1'].p=f.p+f.r@HINGE-r@GATE_ANCHOR
  self.assertLess(h.update(b,BOUNDS,.05),1.)
 def test_missing_nan_broken_and_stand_displaced_latch(self):
  for kind in ['missing','nan','hardware','tunnel','stand']:
   b=fixture();h=settled(b);args={}
   if kind=='missing':del b['gate0']
   elif kind=='nan':b['gate0'].p[0]=np.nan
   elif kind=='hardware':args['hardware_valid']=False
   elif kind=='tunnel':args['tunnel_invalid']=True
   else:b['stand'].p[0]+=.002
   self.assertEqual(h.update(b,BOUNDS,.05,**args),0.);self.assertEqual(settled(fixture(),h).score,0.)
  self.assertEqual(settled(fixture()).score,1.)
 def test_branches_independent_hooks_cycles(self):
  self.assertEqual(max(map(len,serial_paths({'eye':{'a','b'},'a':{'eye','c'},'b':{'eye'},'c':{'a'}}))),1)
  self.assertEqual(max(map(len,serial_paths({'eye':{'a'},'a':{'eye','b','c'},'b':{'a'},'c':{'a'}}))),1)
  self.assertLessEqual(max(map(len,serial_paths({'eye':{'a'},'a':{'eye','b'},'b':{'a','eye'}}))),2)
if __name__=='__main__':unittest.main()
