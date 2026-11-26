import sys,unittest,math
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scoring import Scorer
from mechanism import E,U
class ScoreTests(unittest.TestCase):
 def frame(self,theta=math.radians(11),load=8):
  m={'finite':True,'cam_angle_rad':theta,'cam_speed_rad_s':0.,'rear_q_m':.00003+load/200000,'rear_dq_m_s':0.,'axial_elastic_load_estimate_N':load,'drag_torque_estimate_Nm':.35*load*.009525/2,'gap_fraction':.35+theta/(2*math.pi/3),'robot_contact_impulse_Ns':{'cam':0.,'rear':0.}}
  r={s:{'tcp':[0,1,1.2],'fingers':[-.04695,-.04695]}for s in ['left','right']}
  return dict(objects={},root_poses={'cam':[.3,0,.8375,0,0,0,1]},robot=r,mechanics=m)
 def test_initial_excessive_native_model_state_is_zero(self):
  a=self.frame(theta=0.,load=24.);self.assertEqual(Scorer().update(1,**a)['score'],0)
 def test_unloaded_partial_excludes_direct_rear_contact(self):
  a=self.frame(theta=math.radians(22),load=0);a['mechanics']['rear_q_m']=0.
  a['mechanics']['robot_contact_impulse_Ns']['rear']=.001
  self.assertEqual(Scorer().update(1,**a)['score'],0)
  a['mechanics']['robot_contact_impulse_Ns']['rear']=0
  self.assertEqual(Scorer().update(1,**a)['score'],.25)
 def test_proper_release_requires_consecutive_hold(self):
  s=Scorer();a=self.frame()
  for i in range(1,40):self.assertEqual(s.update(i,**a)['score'],.5)
  self.assertEqual(s.update(40,**a)['score'],1.)
  self.assertEqual(s.update(40,**a)['score'],0.)
 def test_robot_missing_or_touching_cannot_certify_endpoint(self):
  a=self.frame();a['robot'].pop('right');self.assertEqual(Scorer().update(1,**a)['score'],0)
  a=self.frame();a['mechanics']['robot_contact_impulse_Ns']['cam']=.001;self.assertEqual(Scorer().update(1,**a)['score'],0)
 def test_overload_and_ramp_index_are_latched(self):
  for a in [self.frame(load=112),self.frame(theta=math.radians(80))]:
   s=Scorer();self.assertEqual(s.update(1,**a)['score'],0);self.assertTrue(s.failure);self.assertEqual(s.update(2,**self.frame())['score'],0)
 def test_unknown_contact_or_nonfinite_is_zero(self):
  a=self.frame();del a['mechanics']['robot_contact_impulse_Ns']['rear'];self.assertEqual(Scorer().update(1,**a)['score'],0)
  a=self.frame();a['mechanics']['cam_angle_rad']=float('nan');self.assertEqual(Scorer().update(1,**a)['score'],0)
class UnitsTests(unittest.TestCase):
 def test_per_degree_spring_has_correct_si_energy_gradient(self):
  # Native angular drive is authored per degree. One radian of deflection
  # must give k Nm, rather than57.3k.
  kdeg=E['spring_Nm_rad']*math.pi/180
  self.assertAlmostEqual(kdeg*180/math.pi,E['spring_Nm_rad'])
 def test_inferred_drag_does_not_relabel_repeatability_as_backlash(self):
  self.assertLess(U['drag_torque_Nm'][0],.35*8*.009525/2)
  self.assertGreater(U['drag_torque_Nm'][1],.35*8*.009525/2)
if __name__=='__main__':unittest.main()
