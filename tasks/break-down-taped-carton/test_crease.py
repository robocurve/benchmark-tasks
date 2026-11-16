import unittest,math
import numpy as np
from crease import physical_bottom_angle,joint_parameters,BOTTOM_OFFSET
from scene import layout
from collapse_geometry import rx,audit

class CreaseRepairTests(unittest.TestCase):
 def test_angle_branch_preserves_inward_motion_and_decodes_outer_fold(self):
  for raw,wanted in ((0,0),(-10,-10),(-90,270),(-179,181),(180,180),(95,95)):
   self.assertAlmostEqual(physical_bottom_angle(raw),wanted)
 def test_centered_limits_and_sealed_zero_pose(self):
  for h in layout()[1]:
   if not h['b'].endswith('_bottom'):continue
   p=joint_parameters(h);self.assertEqual(p['center'],95.);self.assertAlmostEqual(p['low'],-179.9);self.assertAlmostEqual(p['high'],179.9)
   if h['sealed']:self.assertEqual(joint_parameters(h,True)['low'],-95.);self.assertEqual(joint_parameters(h,True)['high'],-95.)
 def test_explicit_material_units_and_backing_collisions(self):
  for h in layout()[1]:
   p=joint_parameters(h)
   if h['b'].endswith('_bottom'):self.assertEqual(p['stiffness'],0.);self.assertAlmostEqual(p['damping'],.001*math.pi/180);self.assertFalse(p['collision_enabled'])
   elif h['b'].endswith('_top'):self.assertEqual(p['stiffness'],.0002);self.assertTrue(p['collision_enabled'])
   else:self.assertAlmostEqual(p['stiffness'],.002*math.pi/180);self.assertAlmostEqual(p['damping'],.001*math.pi/180)
 def test_closed_bottom_body_centres_are_preserved(self):
  parts,_=layout();p={p['name']:p for p in parts}
  self.assertAlmostEqual(p['north_bottom']['pos'][1],.09-.087/2);self.assertAlmostEqual(p['north_bottom']['pos'][2],0.)
  self.assertAlmostEqual(p['east_bottom']['pos'][0],.11-.087/2);self.assertAlmostEqual(p['east_bottom']['pos'][2],.003)
 def test_repaired_anchor_equality_at_every_nonzero_angle(self):
  from collapse_geometry import configuration
  for angle in (-84.,0.,90.,180.,270.,274.):
   defs,hs,positions,rotations=configuration(90.,{n:angle for n in ('north','south','east','west')})
   for h in hs:
    if not h['b'].endswith('_bottom'):continue
    a=positions[h['a']]+rotations[h['a']]@h['anchor_a'];b=positions[h['b']]+rotations[h['b']]@h['anchor_b'];np.testing.assert_allclose(a,b,atol=1e-12)
    self.assertEqual(h['anchor_b'][2],-BOTTOM_OFFSET)
 def test_backed_layers_and_final_slab_geometry(self):
  for theta in (90.,45.,10.,2.,1.1):self.assertFalse(audit(theta)['penetrations'])
  final=audit(1.1,{n:268.5 for n in ('north','south','east','west')});self.assertFalse(final['penetrations']);self.assertLess(final['height_span_m'],.024)
