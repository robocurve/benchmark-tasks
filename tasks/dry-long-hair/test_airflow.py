import unittest
from airflow import segment_distance,HairThermal
from config import *
I=[[1,0,0],[0,1,0],[0,0,1]]
class ExposureTests(unittest.TestCase):
 def test_segment_crossing_parallel_endpoints(self):
  self.assertAlmostEqual(segment_distance((0,0,0),(1,0,0),(.5,-1,0),(.5,1,0)),0.)
  self.assertAlmostEqual(segment_distance((0,0,0),(1,0,0),(0,1,0),(1,1,0)),1.)
  self.assertAlmostEqual(segment_distance((0,0,0),(0,0,0),(1,0,0),(2,0,0)),1.)
 def test_power_off_cannot_dry(self):
  m=HairThermal();ss=[{'pos':segment(k,j),'rotation':I} for k in range(LOCKS) for j in range(SEGMENTS)]
  result=m.step(.05,ss,(.2,0,1.),(1,0,0),0.)
  self.assertEqual(result['water_removed_step_kg'],0.)
  self.assertEqual(result['delivered_energy_J'],0.)
  self.assertTrue(all(f==[0.,0.,0.] for f in result['wind_forces_N']))
 def test_front_exposure_leaves_hidden_patches_wet(self):
  m=HairThermal();ss=[{'pos':segment(k,j),'rotation':I} for k in range(LOCKS) for j in range(SEGMENTS)]
  r=m.step(.05,ss,(.20,0,1.02),(1,0,0),1.)
  self.assertLess(r['exposed_patches'],len(m.patches)/2)
  self.assertEqual(r['worst_remaining_water_fraction'],1.)
  self.assertLessEqual(r['convective_power_W'],HEATER_LIMIT)
  self.assertLessEqual(sum(sum(v*v for v in f)**.5 for f in r['wind_forces_N']),.2500001)

class PatchTelemetryTests(unittest.TestCase):
 def test_patch_ids_and_measured_occlusion_preserve_water(self):
  m=HairThermal();ss=[{'pos':segment(k,j),'rotation':I} for k in range(LOCKS) for j in range(SEGMENTS)]
  result=m.step(.05,ss,(.20,0,1.02),(1,0,0),1.,patch_samples=True)
  patches=result['patch_samples']
  self.assertEqual(len(patches),LOCKS*SEGMENTS*PATCHES_PER_SEGMENT)
  self.assertEqual([p['patch_index'] for p in patches],list(range(len(patches))))
  for p in patches:
   self.assertEqual(p['patch_index'],(p['lock']*SEGMENTS+p['segment'])*PATCHES_PER_SEGMENT+p['quadrant'])
   if p['occluder'] is not None:
    self.assertEqual(p['exposed_fraction'],0.)
    self.assertEqual(p['remaining_water_fraction'],1.)
  self.assertAlmostEqual(sum(p['water_kg'] for p in patches),result['water_kg'])
  self.assertLessEqual(max(p['temperature_C'] for p in patches),AIR_TEMPERATURE)
  self.assertTrue(any(p['occluder']=='scalp' for p in patches))
 def test_live_scalp_capsule_penetration_is_reported(self):
  m=HairThermal();ss=[{'pos':(2+i,0,2),'rotation':I} for i in range(LOCKS*SEGMENTS)]
  ss[0]={'pos':HEAD,'rotation':I}
  result=m.step(.05,ss,(.2,0,1.),(1,0,0),0.)
  self.assertAlmostEqual(result['min_scalp_capsule_clearance_m'],-HEAD_RADIUS-RADIUS)
