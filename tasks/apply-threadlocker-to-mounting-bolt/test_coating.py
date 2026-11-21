import unittest,math
import numpy as np
from coating import Coating
import spec,thread_geometry
class CoatingTests(unittest.TestCase):
 def test_contact_and_sliding_are_required(self):
  c=Coating();p=[.008,0,.007]
  for force,slide,contact in [(1,.1,False),(0,.1,True),(1,0,True)]:self.assertEqual(c.deposit(p,force,slide,native_contact=contact),0.)
  self.assertEqual(c.measurements()['coverage'],0.)
 def test_contact_stays_local_and_conserves_volume(self):
  c=Coating();v=c.deposit([.008,0,.007],1,.010,native_contact=True)
  self.assertGreater(v,0.);self.assertAlmostEqual(v,(c.film*c.area).sum(),places=15)
  self.assertAlmostEqual(c.remaining_volume+v,c.initial_volume,places=15)
  self.assertTrue(np.all(c.film[abs(c.theta-np.pi)<.2]==0))
  self.assertTrue(np.all(c.film[c.height>.02]==0))
 def test_source_and_cell_capacity_are_finite(self):
  c=Coating();c.remaining_volume=1e-12
  c.deposit([.008,0,.007],100,10,native_contact=True)
  self.assertAlmostEqual(c.remaining_volume,0.,places=15);self.assertLessEqual(c.film.max(),spec.FILM_CAP_M+1e-12)
 def test_reset_and_head_contamination(self):
  c=Coating();c.deposit([.012,0,.085],1,.01,native_contact=True);self.assertFalse(c.measurements()['head_clean'])
  fresh=Coating();self.assertTrue(fresh.measurements()['head_clean']);self.assertEqual(fresh.film.sum(),0.)
 def test_full_circumference_can_be_represented_without_oracle_score(self):
  c=Coating()
  for a in np.linspace(0,2*np.pi,72,endpoint=False):
   for z in np.linspace(.004,.010,13):c.deposit([.008*np.cos(a),.008*np.sin(a),z],1,.001,native_contact=True)
  m=c.measurements();self.assertGreater(m['coverage'],.95);self.assertGreater(m['root_coverage'],.95)
  self.assertAlmostEqual(m['used_volume_m3']+m['remaining_volume_m3'],c.initial_volume,places=12)
 def test_real_thread_dimensions_and_pitch(self):
  for a in np.linspace(0,2*np.pi,13):
   for z in np.linspace(.002,.014,37):
    r=spec.thread_radius(a,z);self.assertGreaterEqual(r,spec.MINOR_RADIUS-1e-9);self.assertLessEqual(r,spec.MAJOR_RADIUS+1e-9)
    self.assertAlmostEqual(r,spec.thread_radius(a,z+spec.PITCH),places=12)
  self.assertGreater(len(list(thread_geometry.ridge_segments())),100)
class WetContactTests(unittest.TestCase):
 def test_wet_contact_requires_reset(self):
  c=Coating();point=[spec.MAJOR_RADIUS,0,.006]
  c.check_nonapplicator_contact(point);self.assertFalse(c.film_disturbed)
  c.film[:]=spec.FILM_VISIBLE_M
  c.check_nonapplicator_contact([spec.MAJOR_RADIUS,0,.030]);self.assertFalse(c.film_disturbed)
  c.check_nonapplicator_contact(point);self.assertTrue(c.film_disturbed)
  self.assertFalse(Coating().film_disturbed)
if __name__=='__main__':unittest.main()
