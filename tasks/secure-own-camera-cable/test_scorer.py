import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from scorer import Scorer,MOUNT,B,STRAP_ROOT
class ScoringTests(unittest.TestCase):
 def scene(self):
  c=MOUNT.copy();wire=[]
  for y in np.arange(40)*(.7217926530318888/40)-.35:wire.append(np.r_[c+B@[.015,y,.013],Rotation.from_matrix(np.column_stack([B[:,1],B[:,2],B[:,0]])).as_quat()])
  strap=[]
  start=STRAP_ROOT.copy()
  for angle in [90,0,0,-45,-45,-45,-40,-90,-90,-90]:
   angle=np.deg2rad(angle)
   axis=np.array([np.cos(angle),0,np.sin(angle)])
   across=np.array([0,-1,0])
   R=Rotation.from_matrix(B@np.column_stack([axis,across,np.cross(axis,across)]))
   strap.append(np.r_[c+B@(start+.007*axis),R.as_quat()])
   start+=.014*axis
  return np.array(wire),np.array(strap),np.eye(4),c+np.array([.3,0,0])
 def test_partial_and_support_free(self):
  w,f,T,tcp=self.scene();s=Scorer()
  for _ in range(8):r=s.update(w,f,T,tcp,.09,.25)
  self.assertEqual(r['score'],1.)
  r=s.update(w,f,T,MOUNT,.09,.25);self.assertEqual(r['score'],.75)
 def test_unsecured_and_disconnected(self):
  w,f,T,tcp=self.scene();s=Scorer();f[:,:3]+=.2
  for _ in range(8):r=s.update(w,f,T,tcp,.09,.25)
  self.assertEqual(r['score'],.5)
  w[1,:3]+=.1;r=s.update(w,f,T,tcp,.09,.25);self.assertEqual(r['score'],0.)
 def test_release_dwell_restarts(self):
  w,f,T,tcp=self.scene();s=Scorer()
  for _ in range(7):r=s.update(w,f,T,tcp,.09,.25)
  self.assertLess(r['score'],1.)
  s.update(w,f,T,tcp,.01,.25)
  r=s.update(w,f,T,tcp,.09,.25);self.assertEqual(r['release_hold_s'],.25)
 def test_air_gap_does_not_fastening_score(self):
  w,f,T,tcp=self.scene();f[:,:3]+=B[:,0]*.009;s=Scorer()
  for _ in range(10):r=s.update(w,f,T,tcp,.09,.25)
  self.assertEqual(r['score'],.5)
 def test_hovering_cable_has_no_route_credit(self):
  w,f,T,tcp=self.scene();w[:,:3]+=B[:,2]*.018;s=Scorer()
  for _ in range(10):r=s.update(w,f,T,tcp,.09,.25)
  self.assertEqual(r['score'],0.)
 def test_long_strap_can_close_before_its_tip(self):
  w,f,T,tcp=self.scene()
  axis=Rotation.from_quat(f[-1,3:]).as_matrix()[:,0]
  extension=np.repeat(f[-1:],2,axis=0)
  extension[:,:3]+=np.arange(1,3)[:,None]*.014*axis
  long_strap=np.vstack([f,extension])
  s=Scorer()
  for _ in range(10):r=s.update(w,long_strap,T,tcp,.09,.25)
  self.assertEqual(r['score'],1.)
 def test_pad_contact_with_open_gap_only_gets_routing_credit(self):
  w,f,T,tcp=self.scene()
  f[6,:3]+=B[:,2]*.016
  s=Scorer()
  for _ in range(10):r=s.update(w,f,T,tcp,.09,.25)
  self.assertTrue(r['strap_overlap'])
  self.assertFalse(r['cable_enclosed'])
  self.assertEqual(r['score'],.5)
if __name__=='__main__':unittest.main()
