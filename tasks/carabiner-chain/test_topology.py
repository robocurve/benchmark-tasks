import unittest
import numpy as np
from chain_task.topology import linked,CrossingAudit,segment_distance,segment_matrix_distance,RodAudit
from chain_task.geometry import layout,loop,eye_loop,STAND_POS,EYE_OFFSET,rx,rz,RADIUS,GATE_OFFSET

def circles(center=1.):
 a=np.linspace(0,2*np.pi,65);return np.column_stack([np.cos(a),np.sin(a),a*0]),np.column_stack([center+np.cos(a),a*0,np.sin(a)])
class TopologyTests(unittest.TestCase):
 def test_linked_and_unlinked(self):
  a,b=circles();self.assertTrue(linked(a,b));_,c=circles(3.);self.assertFalse(linked(a,c));self.assertFalse(linked(a,a+[0,0,.1]));self.assertFalse(linked(a,a+[2.1,0,0]))
 def test_tangent_and_coplanar_conservative(self):
  a,b=circles(2.);self.assertFalse(linked(a,b));self.assertFalse(linked(a,a));self.assertFalse(linked(a,a+[.5,0,0]))
 def test_vertex_crossing_and_rigid_transform(self):
  a,b=circles();r=rz(.7)@rx(.4);p=np.array([.3,-.1,.8]);self.assertTrue(linked(a@r.T+p,b@r.T+p));self.assertTrue(linked(a[::-1],b[::-1]))
 def test_closed_loop_crossing_invalidates(self):
  a,b=circles();h=CrossingAudit();h.update({'eye':a,'link':b+[4,0,0]},{'eye':True,'link':True});h.update({'eye':a,'link':b},{'eye':True,'link':True});self.assertTrue(h.invalid)
  h=CrossingAudit();h.update({'eye':a,'link':b+[4,0,0]},{'eye':True,'link':True});h.update({'eye':a,'link':b},{'eye':True,'link':False});h.update({'eye':a,'link':b},{'eye':True,'link':True});self.assertFalse(h.invalid)
 def test_remote_gate_open_does_not_authorize_later_tunneling(self):
  a,b=circles();h=CrossingAudit();h.update({'eye':a,'link':b+[5,0,0]},{'eye':True,'link':False});h.update({'eye':a,'link':b+[5,0,0]},{'eye':True,'link':True});h.update({'eye':a,'link':b},{'eye':True,'link':True});self.assertTrue(h.invalid)
 def test_original_carabiner_hole_and_suspended_link(self):
  eye=eye_loop();r=rz(np.deg2rad(-10))@rx(np.pi/2);p=STAND_POS+EYE_OFFSET+[0,0,-.025+.00475-.030];g=p+r@GATE_OFFSET;self.assertTrue(linked(eye,loop(p,r,g,r)))
 def test_layout_reproducibility_and_two_extra_seeds(self):
  for seed in [17,23,41]:
   a,b=layout(seed),layout(seed)
   for k in a:np.testing.assert_array_equal(a[k]['position'],b[k]['position']);np.testing.assert_array_equal(a[k]['rotation'],b[k]['rotation'])
 def test_segment_distances(self):
  self.assertAlmostEqual(segment_distance([0,0,0],[1,0,0],[.5,-1,0],[.5,1,0]),0.);self.assertAlmostEqual(segment_distance([0,0,0],[1,0,0],[0,1,0],[1,1,0]),1.)
 def test_vectorized_distances_against_scalar(self):
  rng=np.random.default_rng(3);a=rng.normal(size=(20,2,3));b=rng.normal(size=(15,2,3));exact=np.array([[segment_distance(*x,*y) for y in b] for x in a]);np.testing.assert_allclose(segment_matrix_distance(a,b),exact,atol=1e-12)
 def test_substep_sweep_catches_between_endpoint_crossing(self):
  fixed=np.array([[[-.02,0,0],[.02,0,0]]]);moving=np.array([[[0,-.02,.01],[0,.02,.01]]]);r=np.array([.00225]);a=RodAudit();self.assertTrue(a.update({'a':(fixed,r),'b':(moving,r)}));self.assertFalse(a.update({'a':(fixed,r),'b':(moving-[0,0,.02],r)}));self.assertIn('penetration',a.reason['reason'])
 def test_substep_clearance_and_fresh_reset(self):
  r=np.array([.00225]);a=np.array([[[-.02,0,0],[.02,0,0]]]);b=a+[0,.01,0];audit=RodAudit();self.assertTrue(audit.update({'a':(a,r),'b':(b,r)}));self.assertTrue(audit.update({'a':(a,r),'b':(b+[.001,0,0],r)}));self.assertTrue(RodAudit().update({'a':(a,r),'b':(b,r)}))
 def test_flat_gate_end_does_not_acquire_spherical_cap(self):
  cylinder=np.array([[[0,0,-.0195],[0,0,.0195]]]);bar=np.array([[[0,-.015,.022],[0,.015,.022]]]);a=RodAudit();self.assertTrue(a.update({'gate':(cylinder,np.array([.00225]),np.array([True])),'bar':(bar,np.array([.0025]))}));self.assertGreater(a.narrow_checks,0)
  a=RodAudit();self.assertFalse(a.update({'gate':(cylinder,np.array([.00225]),np.array([True])),'bar':(bar-[0,0,.0015],np.array([.0025]))}))
 def test_flat_cylinder_pair_end_gap_and_real_overlap(self):
  a=np.array([[[0,0,-.0195],[0,0,.0195]]]);b=a+[0,0,.0391];r=np.array([.00225]);kind=np.array([True]);self.assertTrue(RodAudit().update({'a':(a,r,kind),'b':(b,r,kind)}));self.assertFalse(RodAudit().update({'a':(a,r,kind),'b':(b-[0,0,.002],r,kind)}))
if __name__=='__main__':unittest.main()
