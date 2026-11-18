"""Adversarial released-state and material-consistency checks."""
import json,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from surface_material_models import WetFilm,SpreadFilm
from surface_scoring import ScoreState
ROOT=Path(__file__).resolve().parents[1]

class OutcomeTests(unittest.TestCase):
 def wet(self):
  m=WetFilm();pose=np.array([[.33,.025,.777575,0,0,0,1.]])
  for _ in range(100):m.step(pose[0],.05)
  paths=['/World/envs/env_0/wet_coverslip'];filters=paths+['/World/envs/env_0/wet_slide','/World/envs/env_0/LeftArm/grip','/World/envs/env_0/RightArm/grip'];s=ScoreState('wet_mount',paths,filters,filters[-2:],m)
  vel=np.zeros((1,6));rc=np.zeros((2,4));oc=np.zeros((1,4));oc[0,1]=1
  return s,pose,vel,rc,oc
 def hold(self,data,n=20):
  s,*a=data
  for _ in range(n):s.step(*a,.05);s.update()
  return s.value
 def test_good_release_requires_whole_second(self):
  d=self.wet();self.assertEqual(self.hold(d,19),0);self.assertEqual(self.hold(d,1),1)
 def test_pose_without_support_gets_zero(self):
  d=self.wet();d[-1][:]=0;self.assertEqual(self.hold(d),0)
 def test_held_cover_gets_zero(self):
  d=self.wet();d[-2][0,0]=1;self.assertEqual(self.hold(d),0)
 def test_moving_cover_gets_zero(self):
  d=self.wet();d[2][0,0]=.003;self.assertEqual(self.hold(d),0)
 def test_overhang_gets_zero(self):
  d=self.wet();d[1][0,1]+=.01;self.assertEqual(self.hold(d),0)
 def test_interarm_contact_persists(self):
  d=self.wet();d[-2][0,3]=1;self.hold(d,1);d[-2][:]=0;self.assertEqual(self.hold(d,30),0)
 def test_missing_and_invalid_native_values_fail_closed(self):
  d=self.wet();d[1][0,0]=np.nan;self.assertEqual(self.hold(d),0);self.assertTrue(d[0].invalid)
 def test_ending_state_is_not_a_peak_score(self):
  d=self.wet();self.assertEqual(self.hold(d),1);d[-2][0,0]=1;self.assertEqual(self.hold(d,1),0)
 def test_plain_cover_face_flip_is_equivalent(self):
  d=self.wet();d[1][0,3:]=R.from_euler('x',180,degrees=True).as_quat()
  m=WetFilm()
  for _ in range(100):m.step(d[1][0],.05)
  np.testing.assert_allclose(m.water,d[0].model.water,atol=1e-9)
  d[0].model=m;self.assertEqual(self.hold(d),1)

class PetriSymmetryTests(unittest.TestCase):
 def test_quadrants_partition_the_scored_circle_without_double_counting(self):
  m=SpreadFilm();m.water[:]=0;m.water[(m.xx==0)|(m.yy==0)]=.006
  stats=m.metrics();self.assertAlmostEqual(sum(stats['quadrant_wet_fraction'])/4,stats['wet_fraction'])
 def test_round_lid_yaw_does_not_reset_release_hold(self):
  paths=['/World/envs/env_0/'+x for x in ['petri_base','petri_lid','cell_spreader']]
  filters=paths+['/World/envs/env_0/'+x for x in ['tool_rest0','tool_rest1','tool_head_rest','table/geometry/mesh']]
  q=R.from_euler('z',123,degrees=True).as_quat()
  pose=np.array([[.33,.04,.7475,0,0,0,1],[.33,.04,.763,*q],[.43,-.12,.8035,0,0,0,1]])
  oc=np.zeros((3,len(filters)));oc[0,6]=1;oc[1,0]=1;oc[2,3]=1
  s=ScoreState('spread_plate',paths,filters,[],SpreadFilm())
  for _ in range(20):s.step(pose,np.zeros((3,6)),np.zeros((0,len(filters))),oc,.05)
  self.assertAlmostEqual(s.hold,1.)

class MaterialBoundaryTests(unittest.TestCase):
 def test_deep_tool_penetration_is_damage_not_no_contact(self):
  m=SpreadFilm();m.step([.33,.04,.7475,0,0,0,1],[.33,-.028,.752,0,0,0,1],1.,.05);self.assertGreater(m.metrics()['damage_area_mm2'],1)
 def test_upturned_dish_loses_finite_liquid(self):
  m=SpreadFilm();b=np.r_[[.33,.04,.7475],R.from_euler('x',180,degrees=True).as_quat()];m.step(b,[.33,-.028,.80,0,0,0,1],0,.05);self.assertEqual(m.metrics()['water_on_agar_ul'],0);self.assertEqual(m.metrics()['lost_ul'],100)
 def test_tilted_dish_drains_conservatively(self):
  m=SpreadFilm();b=np.r_[[.33,.04,.7475],R.from_euler('x',45,degrees=True).as_quat()]
  for _ in range(300):m.step(b,[.33,-.028,.9,0,0,0,1],0,.05)
  self.assertGreater(m.lost,0);self.assertLess(m.metrics()['conservation_error_ul'],1e-7)

if __name__=='__main__':unittest.main()
