"""Refinement conservation and exact batched geometric-query negatives."""
import math,unittest
import numpy as np
from config import *
from airflow import HairThermal,segment_distance,ray_segment_distances

class RefinementTests(unittest.TestCase):
 def test_fourfold_real_body_sampling_preserves_mass_length_surface_and_heat(self):
  self.assertEqual((LOCKS,SEGMENTS),(24,10))
  self.assertAlmostEqual(SEGMENTS*LENGTH,.25)
  self.assertAlmostEqual(LOCKS*SEGMENTS*2*math.pi*RADIUS*LENGTH,REFERENCE_LOCKS*REFERENCE_SEGMENTS*2*math.pi*REFERENCE_RADIUS*REFERENCE_LENGTH)
  m=HairThermal()
  self.assertEqual(len(m.patches),960)
  self.assertAlmostEqual(sum(p.water for p in m.patches),.0064)
  self.assertAlmostEqual(sum(p.dry_heat_capacity for p in m.patches),.032*1400)
 def test_old_centrelines_are_subdivided_not_shortened_or_restretched(self):
  for k in range(LOCKS):
   coarse=coarse_points_local(k);fine=points_local(k)
   for i,p in enumerate(coarse):self.assertEqual(fine[i*AXIAL_REFINEMENT],p)
   for a,b in zip(fine,fine[1:]):self.assertAlmostEqual(math.dist(a,b),LENGTH)
 def test_distributed_bend_energy_for_uniform_curvature_and_lock_split(self):
  theta=.2
  old=REFERENCE_LOCKS*REFERENCE_SEGMENTS*.5*.00006*theta**2
  new=LOCKS*SEGMENTS*.5*BEND_STIFFNESS*(theta/AXIAL_REFINEMENT)**2
  self.assertAlmostEqual(new,old)
  self.assertEqual(BEND_LIMIT_DEG*AXIAL_REFINEMENT,70.)
  for k in range(LOCKS):
   for n in range(SEGMENTS):self.assertEqual(bend_axis(k,n),'X' if (k+n//AXIAL_REFINEMENT)%2 else 'Y')
 def test_batched_distance_matches_reference_at_parallel_and_degenerate_cases(self):
  rng=np.random.default_rng(719)
  starts=rng.normal(size=(257,3));ends=rng.normal(size=(257,3))
  starts[:4]=[[0,0,0],[1,0,0],[0,1,0],[2,0,0]];ends[:4]=[[0,0,0],[2,0,0],[1,1,0],[3,0,0]]
  for a,b in [([0,0,0],[1,0,0]),([0,0,0],[0,0,0]),([1,2,3],[1,2,3+1e-9]),(rng.normal(size=3),rng.normal(size=3))]:
   got=ray_segment_distances(a,b,starts,ends)
   want=[segment_distance(a,b,c,d)for c,d in zip(starts,ends)]
   np.testing.assert_allclose(got,want,atol=3e-14,rtol=3e-14)

if __name__=='__main__':unittest.main()
