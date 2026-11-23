"""Geometry feedback regressions; no simulated completion claims."""
import unittest
import numpy as np
from feedback import capsule_gap,retaining_face,plug_fit,BootJawPredictor
from geometry import box_triangles
from collision_geometry import FingerGeometry
from robot import BOOT_LOCAL,R_GRIP
from config import *

class FeedbackGeometryTests(unittest.TestCase):
 def test_exact_capsule_support_does_not_use_square_radial_corners(self):
  # Radius1: a small cube around (.9,.9,0) is outside the round cylinder,
  # despite overlapping its enclosing square [-1,1]^2.
  triangles=box_triangles((.9,.9,0),(.02,.02,.02))
  self.assertGreater(capsule_gap(triangles,np.zeros(3),np.array((0,0,1)),2.,1.,.01),.2)
  touching=box_triangles((.8,0,0),(.1,.1,.1))
  self.assertEqual(capsule_gap(touching,np.zeros(3),np.array((0,0,1)),2.,1.),0.)
 def test_retention_uses_actual_clip_face_not_plug_reconstruction(self):
  clip=np.r_[np.array(port(TARGET))+[0,0,CLIP_Z],0,0,0,1.]
  self.assertTrue(retaining_face(clip,TARGET)['retained'])
  clip[0]-=.002
  self.assertFalse(retaining_face(clip,TARGET)['retained'])
  clip[0]+=.002;clip[2]-=.002
  self.assertFalse(retaining_face(clip,TARGET)['retained'])
 def test_actual_guide_footprint_requires_submillimetre_alignment(self):
  plug=np.r_[port(TARGET),0,0,0,1.]
  self.assertAlmostEqual(plug_fit(plug,TARGET)['minimum_clearance_m'],.0003)
  plug[2]+=.00025
  self.assertLess(plug_fit(plug,TARGET)['minimum_clearance_m'],.0001)
  plug[2]-=.00025;plug[1]+=.0006
  self.assertLess(plug_fit(plug,TARGET)['minimum_clearance_m'],0.)
 def test_expected_level_jaw_stops_are_independent_of_force_target(self):
  plug=np.r_[port(SERVICE),0,0,0,1.]
  predictor=BootJawPredictor(FingerGeometry())
  result=predictor.predict(plug,plug[:3]+BOOT_LOCAL,R_GRIP,np.array((-.0035,-.0035)),command_opening=.0015)
  full=predictor.predict(plug,plug[:3]+BOOT_LOCAL,R_GRIP,np.array((-.0035,-.0035)),command_opening=0.)
  np.testing.assert_allclose(result['opening_m'],full['opening_m'],atol=1e-12)
  self.assertTrue(all(result['contact_possible']))

 def test_nonfinite_geometry_fails_closed(self):
  from feedback import clearance
  triangle=box_triangles((0,0,0),(1,1,1));triangle[0,0,0]=np.nan
  result=clearance([triangle],({},{},{}))
  self.assertEqual(result['minimum_gap_m'],0.)
  self.assertIn('nonfinite',result['invalid'])
  clip=np.array([np.nan,0,0,0,0,0,1.])
  self.assertFalse(retaining_face(clip,TARGET)['retained'])

class BootCacheTests(unittest.TestCase):
 class Predictor:
  def __init__(self,possible=True):self.calls=0;self.surface_calls=0;self.possible=possible
  def predict(self,plug,tcp,rotation,fingers,command_opening):
   self.calls+=1
   return {'opening_m':[.00033,.00138],'actual_surface_gap_m':[0.,0.],
           'contact_possible':[self.possible,self.possible]}
  def surface_gaps(self,plug,tcp,rotation,fingers):
   self.surface_calls+=1;return [abs(float(fingers[0]))/100,abs(float(fingers[1]))/100]
 def test_cache_invalidates_relative_pose_and_rechecks_current_surfaces(self):
  from feedback import BootJawContactCache
  from yam_arena.urdf import axis_angle_matrix
  predictor=self.Predictor();cache=BootJawContactCache(predictor)
  plug=np.array([0,0,0,0,0,0,1.]);tcp=np.array([-.0395,-.0035,-.0005]);rotation=np.eye(3);fingers=np.array([-.00033,-.00138])
  result=cache.evaluate(plug,tcp,rotation,fingers,1.,0.)
  self.assertFalse(result['cache_hit']);self.assertEqual(predictor.calls,1)
  moved=tcp+[.00001,0,0]
  result=cache.evaluate(plug,moved,rotation,fingers*.9,1.05,0.)
  self.assertTrue(result['cache_hit']);self.assertEqual(predictor.calls,1);self.assertEqual(predictor.surface_calls,1)
  self.assertEqual(result['cache_timestamp_s'],1.)
  cache.evaluate(plug,tcp+[.00004,0,0],rotation,fingers,1.1,0.)
  self.assertEqual(predictor.calls,2)
  turned=axis_angle_matrix(np.array((0,0,1.)),np.radians(.04))
  cache.evaluate(plug,tcp+[.00004,0,0],turned,fingers,1.15,0.)
  self.assertEqual(predictor.calls,3)
  cache.evaluate(plug,tcp+[.00004,0,0],turned,fingers,1.2,0.,fresh=True)
  self.assertEqual(predictor.calls,4)
 def test_empty_cache_and_no_contact_never_become_contact(self):
  from feedback import BootJawContactCache
  predictor=self.Predictor(False);cache=BootJawContactCache(predictor)
  plug=np.array([0,0,0,0,0,0,1.]);tcp=np.zeros(3);rotation=np.eye(3);fingers=np.zeros(2)
  for time in (1.,1.05):
   result=cache.evaluate(plug,tcp,rotation,fingers,time,0.)
   self.assertFalse(any(result['contact_possible']))
  self.assertEqual(predictor.calls,1)

class FullClosureContactTests(unittest.TestCase):
 def test_full_closure_can_reach_shallow_rear_boot_contact(self):
  case={'plug': [0.4043436646461487, -0.024108493700623512, 0.9497970938682556, -3.773825665120967e-05, -0.02525138296186924, 0.009082457982003689, 0.9996398091316223], 'tcp': [0.36416357608065275, -0.029300051936031968, 0.9483286460421125], 'rotation': [[-0.0028258684263408346, 0.17301760603399177, 0.9849146777614309], [0.012578395392743105, -0.9848345417171862, 0.17303961803599727], [0.9999168957653337, 0.01287763343811342, 0.000606728978007011]], 'fingers': [-0.0016366339987143874, -0.0013667733874171972]}
  predictor=BootJawPredictor(FingerGeometry())
  args=[np.array(case[name]) for name in ('plug','tcp','rotation','fingers')]
  partial=predictor.predict(*args,command_opening=.0015)
  full=predictor.predict(*args,command_opening=0.)
  self.assertFalse(all(partial['contact_possible']))
  self.assertTrue(all(full['contact_possible']))
  self.assertLess(full['opening_m'][0],.0005)
  self.assertGreater(full['opening_m'][0],.00015)
  self.assertLess(full['opening_m'][1],.0015)


class EndpointCorrectionTests(unittest.TestCase):
 def test_moving_stages_do_not_learn_and_endpoint_step_cap_and_freeze(self):
  from feedback import EndpointCorrection
  correction=EndpointCorrection();error=np.array((.001,-.0005,.0003))
  for _ in range(60):np.testing.assert_array_equal(correction.update(error,.05,False),np.zeros(3))
  first=correction.update(error,.05,True)
  self.assertGreater(np.linalg.norm(first),0.)
  self.assertLessEqual(np.linalg.norm(first),.00005+1e-12)
  frozen=correction.update(-error,.05,False)
  np.testing.assert_array_equal(first,frozen)
  for _ in range(100):correction.update(error,.05,True)
  self.assertLessEqual(np.linalg.norm(correction.value),.0025+1e-12)
  correction.reset();np.testing.assert_array_equal(correction.value,np.zeros(3))


class JointEndpointCorrectionTests(unittest.TestCase):
 def test_no_moving_reference_and_fixed_unbiased_target_under_updates(self):
  from feedback import JointEndpointCorrection
  servo=JointEndpointCorrection()
  servo.update(np.ones(6),.05)
  self.assertIsNone(servo.command());np.testing.assert_array_equal(servo.bias,np.zeros(6))
  reference=np.arange(6)*.1;servo.latch(reference)
  for _ in range(20):servo.update(reference-.011,.05)
  np.testing.assert_array_equal(servo.reference,reference)
  self.assertFalse(servo.reference.flags.writeable)
  self.assertTrue(np.all(servo.command()>reference))
 def test_observed_11_mrad_load_is_compensated_without_10_mrad_saturation(self):
  from feedback import JointEndpointCorrection
  servo=JointEndpointCorrection();reference=np.zeros(6);servo.latch(reference)
  load=np.array((.003,-.002,.002,-.004,.006,.0112))
  for _ in range(200):
   before=servo.bias.copy();actual=servo.command()-load;servo.update(actual,.05)
   self.assertLessEqual(float(np.max(np.abs(servo.bias-before))),.00025+1e-12)
  self.assertGreater(servo.bias[5],.011)
  self.assertLess(np.linalg.norm(servo.command()-load-reference),.0001)
  self.assertLessEqual(float(np.max(np.abs(servo.bias))),.02)
 def test_cap_freeze_and_unsafe_or_nonfinite_state_do_not_integrate(self):
  from feedback import JointEndpointCorrection
  servo=JointEndpointCorrection();servo.latch(np.zeros(6))
  for _ in range(200):servo.update(np.full(6,-1.),.05)
  np.testing.assert_allclose(servo.bias,.02)
  servo.freeze();before=servo.bias.copy()
  for _ in range(10):servo.update(np.ones(6),.05)
  np.testing.assert_array_equal(servo.bias,before)
  servo.reset();servo.latch(np.zeros(6));servo.update(np.ones(6),.05,allowed=False)
  np.testing.assert_array_equal(servo.bias,np.zeros(6))
  servo.update(np.full(6,np.nan),.05)
  np.testing.assert_array_equal(servo.bias,np.zeros(6));self.assertIsNotNone(servo.invalid)


class ForceBalancedClosureTests(unittest.TestCase):
 def test_candidate_preload_includes_finger_gravity(self):
  from feedback import predicted_jaw_normal_forces
  from robot import GRIP_OPENING
  forces=predicted_jaw_normal_forces([.00367787,.00340215],GRIP_OPENING)
  np.testing.assert_allclose(forces,[2.5,2.5],atol=.02)
  self.assertLess(abs(forces[0]-forces[1]),.02)
 def test_final_millimetre_closes_slowly_and_hold_command_is_constant(self):
  from feedback import closing_opening
  from robot import GRIP_OPENING
  times=np.arange(0,5.01,.05)
  openings=np.array([closing_opening(t,.0065,GRIP_OPENING) for t in times])
  self.assertTrue(np.all(np.diff(openings)<=1e-12))
  last=[closing_opening(t,.0065,GRIP_OPENING) for t in np.arange(2.,4.001,.05)]
  self.assertLessEqual(float(np.abs(np.diff(last)).max()),.00004)
  for time in (4.,5.,20.):self.assertAlmostEqual(closing_opening(time,.0065,GRIP_OPENING),GRIP_OPENING)


class GravityAwareContactGateTests(unittest.TestCase):
 def test_empty_jaws_at_gravity_equilibrium_are_not_contact(self):
  from feedback import boot_contact_force_gate
  from robot import GRIP_OPENING
  gravity_offset=.069657*9.81/5000.
  result=boot_contact_force_gate([GRIP_OPENING+gravity_offset,GRIP_OPENING-gravity_offset],GRIP_OPENING)
  self.assertFalse(result['blocked'])
  np.testing.assert_allclose(result['normal_estimate_N'],[0.,0.],atol=1e-12)
 def test_one_sided_contact_is_not_two_jaw_contact(self):
  from feedback import boot_contact_force_gate
  from robot import GRIP_OPENING
  gravity_offset=.069657*9.81/5000.
  for openings in ([GRIP_OPENING+gravity_offset+.0003,GRIP_OPENING-gravity_offset],
                   [GRIP_OPENING+gravity_offset,GRIP_OPENING-gravity_offset+.0003]):
   self.assertFalse(boot_contact_force_gate(openings,GRIP_OPENING)['blocked'])
 def test_actual_011_upper_gravity_support_meets_same_normal_force_margin(self):
  from feedback import boot_contact_force_gate
  from robot import GRIP_OPENING
  openings=np.array([.003389360150322318,.0031129373237490654])
  self.assertFalse(np.all(openings>GRIP_OPENING+.00015))
  result=boot_contact_force_gate(openings,GRIP_OPENING)
  self.assertTrue(result['blocked'])
  self.assertGreater(min(result['normal_estimate_N']),1.)
