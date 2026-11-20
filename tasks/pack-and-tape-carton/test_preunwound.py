"""Prepared ordinary ribbon is real finite material, never a grasp fixture."""
import math,unittest
import numpy as np
from config import *
class Tests(unittest.TestCase):
 def test_free_tail_is_continuous_tangent_and_unmodified_film(self):
  self.assertEqual(PREUNWOUND_CELLS,12);self.assertAlmostEqual(PREUNWOUND_LENGTH,.060)
  p,y=tape_cell(WOUND_CELLS-1);direction=np.array((math.cos(y),math.sin(y),0.))
  for i in range(WOUND_CELLS,CELLS):
   a,ay=tape_cell(i-1);b,by=tape_cell(i)
   np.testing.assert_allclose(np.array(b)-a,CELL_LENGTH*direction,atol=1e-12);self.assertAlmostEqual(ay,by)
  self.assertEqual(CELL_LENGTH*CELLS,ACTIVE_LENGTH)
 def test_only_wound_cells_have_tack_and_free_tail_is_connected(self):
  from pxr import Usd,UsdPhysics
  from tape_native import install_joints
  s=Usd.Stage.CreateInMemory();parent='/World/envs/env_0';install_joints(s,parent)
  for i in range(CELLS):
   self.assertEqual(bool(s.GetPrimAtPath(parent+f'/tape_{i}/wound_tack')),i<WOUND_CELLS)
   if i:
    joint=UsdPhysics.RevoluteJoint(s.GetPrimAtPath(parent+f'/tape_{i}/film_bend'))
    self.assertTrue(joint);self.assertEqual(str(joint.GetBody0Rel().GetTargets()[0]),parent+f'/tape_{i-1}')
    self.assertEqual(str(joint.GetBody1Rel().GetTargets()[0]),parent+f'/tape_{i}')
    # Authored hinge frames coincide at the same physical world point.
    a,ay=tape_cell(i-1);b,by=tape_cell(i)
    def rotation(y):return np.array(((math.cos(y),-math.sin(y),0.),(math.sin(y),math.cos(y),0.),(0.,0.,1.)))
    x=np.array(a)+rotation(ay)@np.array(joint.GetLocalPos0Attr().Get())
    z=np.array(b)+rotation(by)@np.array(joint.GetLocalPos1Attr().Get())
    np.testing.assert_allclose(x,z,atol=2e-8)
 def test_free_end_has_no_added_handle_or_fixture(self):
  from pxr import Usd
  from tape_native import install_joints
  s=Usd.Stage.CreateInMemory();parent='/World/envs/env_0';install_joints(s,parent)
  for prim in s.Traverse():
   self.assertNotIn('fixture',str(prim.GetPath()));self.assertNotIn('tab',prim.GetName())
if __name__=='__main__':unittest.main()
