"""Exercise native replay against real USD transforms, without a GPU."""
import hashlib,json,tempfile,unittest
from pathlib import Path
from pxr import Gf,Usd,UsdGeom
from replay_native import build,sha
class ReplayTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name);self.capture=self.root/'capture';self.capture.mkdir();(self.capture/'source').mkdir();(self.capture/'source-sha256.json').write_text('{}')
  stage=Usd.Stage.CreateNew(str(self.capture/'scene.usdc'));owners={}
  for index in range(116):
   path='/World/body'+str(index);UsdGeom.Xform.Define(stage,path);owners[path]={}
  stage.GetRootLayer().Save();self.owners=owners
  manifest=dict(owners=owners,scene_file='scene.usdc',scene_sha256=sha(self.capture/'scene.usdc'),capture_hz=20,pose_space='world',quaternion_order='xyzw',interpolated=False,virtual_review_views=[dict(name='review_top',eye=[0,0,2],target=[0,0,0],focal_length_mm=18,width=480,height=270)])
  # Choose a nonparallel up vector for the real view constructor.
  manifest['virtual_review_views'][0]['eye']=[1,1,2]
  (self.capture/'capture-manifest.json').write_text(json.dumps(manifest))
  self.row=dict(time=.05,native_owner_poses={p:[.1,.2,.3,0,0,0,1] for p in owners},metrics={'score':1.})
  initial=dict(time=0.,native_owner_poses={p:[0,0,0,0,0,0,1] for p in owners})
  (self.capture/'initial-state.json').write_text(json.dumps(initial));self.write_trace();(self.capture/'result.json').write_text('{"task_score":1.0}')
 def write_trace(self):
  (self.capture/'trajectory.jsonl').write_text(json.dumps(self.row)+'\n');self.digest=sha(self.capture/'trajectory.jsonl');self.audit=self.root/'audit.json';self.audit.write_text(json.dumps(dict(passed=True,trace_sha256=self.digest,score=1.,control_rows=1)))
 def test_world_pose_and_disclosed_views(self):
  result=build(self.capture,self.root/'replay',self.digest,self.audit);stage=Usd.Stage.Open(str(self.root/'replay/replay.usdc'));x=UsdGeom.Xformable(stage.GetPrimAtPath('/World/body0'))
  self.assertTrue(x.GetResetXformStack());self.assertEqual(tuple(x.ComputeLocalToWorldTransform(1).ExtractTranslation()),(.1,.2,.3));self.assertEqual(result['frames'],2);self.assertFalse(result['native_rgb'])
 def test_incomplete_cannot_claim_complete(self):
  (self.capture/'result.json').write_text('{"task_score":0.25}')
  with self.assertRaisesRegex(ValueError,'incomplete'):build(self.capture,self.root/'bad',self.digest,self.audit)
 def test_missing_pose_rejected(self):
  self.row['native_owner_poses'].pop('/World/body3');self.write_trace()
  with self.assertRaisesRegex(ValueError,'missing native body'):build(self.capture,self.root/'bad',self.digest,self.audit)
 def test_missing_sample_rejected(self):
  self.row['time']=.10;self.write_trace()
  with self.assertRaisesRegex(ValueError,'missing control'):build(self.capture,self.root/'bad',self.digest,self.audit)
if __name__=='__main__':unittest.main()
