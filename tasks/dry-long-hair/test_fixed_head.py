"""Fixed-world head topology/scoring; USD checks are not native dynamics proof."""
import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch
from config import HEAD,HEAD_QUAT,TOP,LOCKS,SEGMENTS,multiply,rotate,root_local,segment_quat,segment
from scoring import HairScorer,measured_head_motion


def sample(time=0.,**changes):
 result=dict(time=time,head_pose=[*HEAD,*HEAD_QUAT],root_separation=0.,min_height=.1,
             dry_fraction=0.,worst_remaining_water_fraction=1.,max_temperature_C=25.,dryer_speed=0.,
             power_on=True,water_removed_step_kg=1e-7,dryer_parked=False,hands_clear=False)
 result.update(changes)
 return result


class FixedHeadScoringTests(unittest.TestCase):
 def test_nominal_and_subthreshold_head_keep_partial_and_full_credit(self):
  for dry,expected in [(0.,.25),(.5,.5),(.9,.75)]:
   scorer=HairScorer();s=sample(dry_fraction=dry);s['head_pose'][1]+=.0019999
   self.assertEqual(scorer.update(s)['score'],expected)
  scorer=HairScorer()
  for i in range(30):m=scorer.update(sample(i*.05,dry_fraction=1.,worst_remaining_water_fraction=.05,power_on=False,water_removed_step_kg=0.,dryer_parked=True,hands_clear=True))
  self.assertEqual(m['score'],1.);self.assertIsNone(m['invalid'])

 def test_translation_over2mm_invalidates_every_partial_and_latches(self):
  for dry in (0.,.5,.9,1.):
   for axis in range(3):
    scorer=HairScorer();s=sample(dry_fraction=dry);s['head_pose'][axis]+=.0020001
    m=scorer.update(s);self.assertEqual(m['score'],0.);self.assertIn('translated',m['invalid'])
    self.assertEqual(scorer.update(sample(.05,dry_fraction=1.))['score'],0.)

 def test_rotation_over1deg_all_axes_invalidates_despite_zero_translation(self):
  for axis in range(3):
   for angle,invalid in ((.999999,False),(1.000001,True),(180.,True)):
    s=sample(dry_fraction=1.);a=math.radians(angle)/2;delta=[0.,0.,0.,math.cos(a)];delta[axis]=math.sin(a);s['head_pose'][3:]=multiply(HEAD_QUAT,delta)
    m=HairScorer().update(s);self.assertEqual(m['invalid'] is not None,invalid)
    if invalid:self.assertEqual(m['score'],0.)

 def test_missing_nonfinite_and_zero_quaternion_fail_closed(self):
  for pose in (None,[],[*HEAD,0,0,1],[*HEAD,0,0,0,0],[*HEAD,float('nan'),0,0,1],[float('inf'),*HEAD[1:],0,0,0,1]):
   s=sample(dry_fraction=1.,head_pose=pose,head_translation_m=0.,head_rotation_deg=0.)
   m=HairScorer().update(s);self.assertEqual(m['score'],0.);self.assertFalse(m['head_pose_valid']);self.assertIsNone(m['head_translation_m']);self.assertIsNone(m['head_rotation_deg'])
  s=sample();del s['head_pose'];self.assertEqual(HairScorer().update(s)['score'],0.)

 def test_reference_is_declared_world_pose_and_quaternion_sign_is_equivalent(self):
  scorer=HairScorer();s=sample();s['head_pose'][0]+=.02
  self.assertIn('translated',scorer.update(s)['invalid']) # first sample cannot rebase
  for q in (HEAD_QUAT,tuple(-v for v in HEAD_QUAT),tuple(v*.99999999 for v in HEAD_QUAT)):
   m=measured_head_motion([*HEAD,*q]);self.assertTrue(m['head_pose_valid']);self.assertEqual(m['head_translation_m'],0.);self.assertEqual(m['head_rotation_deg'],0.)


try:
 from pxr import Usd,UsdGeom,UsdPhysics,Gf
 HAVE_USD=True
except ImportError:HAVE_USD=False


@unittest.skipUnless(HAVE_USD,'OpenUSD is unavailable in this Python; run with the USD test environment')
class FixedHeadUsdTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  # The local OpenUSD wheel omits the NVIDIA PhysxSchema plugin. Only supply
  # an unused import namespace: these exercised native functions call REAL
  # UsdPhysics APIs and do not touch PhysxSchema. No PhysX behavior is mocked
  # or claimed. Full export/runtime still needs the native installed plugin.
  try:
   from pxr import PhysxSchema
   modules={}
  except ImportError:modules={'pxr.PhysxSchema':types.ModuleType('pxr.PhysxSchema')}
  spec=importlib.util.spec_from_file_location('_hair_native_topology',Path(__file__).with_name('native.py'))
  cls.native=importlib.util.module_from_spec(spec)
  with patch.dict(sys.modules,modules):spec.loader.exec_module(cls.native)

 def stage(self):
  stage=Usd.Stage.CreateInMemory();head=UsdGeom.Xform.Define(stage,'/Env/mannequin/head');self.native.pose_hair_body(head,'head');UsdPhysics.RigidBodyAPI.Apply(head.GetPrim())
  self.native.install_joints(stage,'/Env/mannequin',include_dryer=False)
  return stage

 def test_world_fixed_mount_has_declared_rest_rotation_and_no_yaw_dof(self):
  stage=self.stage();p=stage.GetPrimAtPath('/Env/mannequin/head/neck_bearing');self.assertTrue(p.IsA(UsdPhysics.FixedJoint));self.assertFalse(p.IsA(UsdPhysics.RevoluteJoint));j=UsdPhysics.FixedJoint(p)
  self.assertEqual(j.GetBody0Rel().GetTargets(),[]);self.assertEqual([str(x)for x in j.GetBody1Rel().GetTargets()],['/Env/mannequin/head']);self.assertTrue(j.GetExcludeFromArticulationAttr().Get())
  a0=j.GetLocalPos0Attr().Get();a1=j.GetLocalPos1Attr().Get()
  for i in range(3):self.assertAlmostEqual(a0[i],HEAD[i]+a1[i],places=6)
  self.assertAlmostEqual(a0[2],TOP+.12,places=6)
  self.assertEqual(j.GetLocalRot0Attr().Get(),Gf.Quatf(HEAD_QUAT[3],Gf.Vec3f(*HEAD_QUAT[:3])));self.assertEqual(j.GetLocalRot1Attr().Get(),Gf.Quatf(1.))
  attrs={a.GetName()for a in p.GetAuthoredAttributes()};self.assertNotIn('physics:axis',attrs);self.assertNotIn('physics:lowerLimit',attrs);self.assertNotIn('physics:upperLimit',attrs)


 def test_opposite_nose_root_and_fixed_mount_share_one_world_frame(self):
  stage=self.stage();cache=UsdGeom.XformCache()
  head=stage.GetPrimAtPath('/Env/mannequin/head');world=cache.GetLocalToWorldTransform(head)
  nose=world.Transform(Gf.Vec3d(-.065,0,0))
  self.assertAlmostEqual(nose[0],HEAD[0]+.065,places=7)
  self.assertAlmostEqual(nose[1],HEAD[1],places=7)
  for k in range(LOCKS):
   j=UsdPhysics.RevoluteJoint.Get(stage,f'/Env/mannequin/hair_{k}_0/bend_{k}_0')
   # USD local root offset must remain head-local: applying the new head
   # orientation once gives the rotated segment endpoint, never twice.
   root=world.Transform(Gf.Vec3d(*j.GetLocalPos0Attr().Get()))
   expected=tuple(c+d for c,d in zip(HEAD,rotate(HEAD_QUAT,root_local(k))))
   for a,b in zip(root,expected):self.assertAlmostEqual(a,b,places=7)
   hair=UsdGeom.Xform.Define(stage,f'/Env/mannequin/hair_{k}_0')
   self.native.pose_hair_body(hair,f'hair_{k}_0')
   childworld=UsdGeom.XformCache().GetLocalToWorldTransform(hair.GetPrim())
   childroot=childworld.Transform(Gf.Vec3d(*j.GetLocalPos1Attr().Get()))
   for a,b in zip(root,childroot):self.assertAlmostEqual(a,b,places=7)

 def test_all60_passive_bends_and_separate_skin_attachment_remain(self):
  stage=self.stage();bends=[p for p in stage.Traverse()if p.IsA(UsdPhysics.RevoluteJoint)]
  self.assertEqual(len(bends),LOCKS*SEGMENTS);self.assertEqual(len(bends),60)
  for p in bends:
   j=UsdPhysics.RevoluteJoint(p);self.assertEqual(j.GetLowerLimitAttr().Get(),-70.);self.assertEqual(j.GetUpperLimitAttr().Get(),70.);d=UsdPhysics.DriveAPI(p,'angular');self.assertAlmostEqual(d.GetStiffnessAttr().Get(),.00006,places=10);self.assertAlmostEqual(d.GetDampingAttr().Get(),.000015,places=10)
  self.native.install_skin_attachment(stage,'/Env');j=UsdPhysics.FixedJoint.Get(stage,'/Env/scalp_collision/skin_attachment');self.assertTrue(j);self.assertTrue(j.GetExcludeFromArticulationAttr().Get());self.assertFalse(j.GetCollisionEnabledAttr().Get());self.assertEqual([str(x)for x in j.GetBody0Rel().GetTargets()],['/Env/mannequin/head'])

if __name__=='__main__':unittest.main()
