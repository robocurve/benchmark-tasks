"""Check product mass anchor, shell inertia and passive impact planning."""
import json,unittest
from pathlib import Path
import numpy as np
from pxr import Usd,UsdPhysics
from rally_task.dynamics import impact,K,R
ROOT=Path(__file__).resolve().parent
D=json.loads((ROOT/'physical_design.json').read_text())
class PhysicalDesign(unittest.TestCase):
 def test_paddle_mass_and_com(self):
  parts=D['paddle']['parts'];mass=sum(p['mass_kg'] for p in parts)
  self.assertAlmostEqual(mass,.170)
  com=sum(p['mass_kg']*np.array(p['centre_m']) for p in parts)/mass
  np.testing.assert_allclose(com,D['paddle']['centre_of_mass_m'])
 def test_ball_shell_and_native_assets(self):
  np.testing.assert_allclose(D['ball']['inertia_kg_m2'],[2/3*.0027*.02**2]*3)
  for name,mass in [('ball',.0027),('paddle',.170)]:
   stage=Usd.Stage.Open(str(ROOT/'assets'/f'{name}.usda'));p=stage.GetDefaultPrim();self.assertAlmostEqual(UsdPhysics.MassAPI(p).GetMassAttr().Get(),mass,places=7)
   inertia=UsdPhysics.MassAPI(p).GetDiagonalInertiaAttr().Get();self.assertTrue(all(x>0 for x in inertia))
 def test_board_mass_uses_declared_material_volumes(self):
  b=D['board'];self.assertAlmostEqual(b['mdf_mass_kg'],.015*.6*.3*700);self.assertAlmostEqual(b['rubber_mass_kg'],.002*.6*.3*1000);self.assertAlmostEqual(b['panel_total_mass_kg'],2.25)
 def test_fixed_surface_impact_cannot_create_energy(self):
  rng=np.random.default_rng(123)
  for _ in range(300):
   v=rng.normal(size=2)*3;v[0]=-abs(v[0]);w=float(rng.normal()*200)
   out,wo=impact(v,w,[1.,0.]);energy=lambda q,o:float(q@q+K*R*R*o*o)
   self.assertLessEqual(energy(out,wo),energy(v,w)+1e-10)
 def test_separating_ball_is_not_hit_again(self):
  v,w=impact([2.,1.],50.,[1.,0.]);np.testing.assert_array_equal(v,[2.,1.]);self.assertEqual(w,50.)
if __name__=='__main__':unittest.main()
