"""Offline replay of captured actual native poses, with no simulation or FK."""
import argparse,hashlib,json,math
from pathlib import Path
from pxr import Gf,Sdf,Usd,UsdGeom

LABEL='OFFLINE PHYSICS REPLAY - NOT NATIVE RGB'
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def require(value,message):
 if not value:raise ValueError(message)
def valid_pose(value):
 require(len(value)==7 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in value),'invalid actual pose')
 require(abs(sum(v*v for v in value[3:])-1)<.002,'invalid quaternion')
def quaternion(v):return Gf.Quatd(v[3],Gf.Vec3d(*v[:3]))
def ops(prim):
 require(bool(prim) and not prim.IsInstanceProxy(),'missing editable native actor')
 x=UsdGeom.Xformable(prim);x.ClearXformOpOrder();x.SetResetXformStack(True)
 return x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble,'recorded'),x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble,'recorded')
def build(capture,out,trace_hash,audit,diagnostic=False):
 capture=Path(capture);out=Path(out);trace=capture/'trajectory.jsonl'
 require(sha(trace)==trace_hash,'trace hash mismatch')
 manifest=json.loads((capture/'capture-manifest.json').read_text());source=json.loads((capture/'source-sha256.json').read_text())
 for name,digest in source.items():require(Path(name).name==name and sha(capture/'source'/name)==digest,'frozen source changed')
 require(Path(manifest['scene_file']).name==manifest['scene_file'],'unsafe scene filename')
 scene=capture/manifest['scene_file'];require(sha(scene)==manifest['scene_sha256'],'scene hash mismatch')
 require(manifest['capture_hz']==20 and manifest['pose_space']=='world' and manifest['quaternion_order']=='xyzw' and manifest['interpolated'] is False,'unsupported capture')
 require(len(manifest['owners'])==116,'missing native owners')
 rows=[json.loads((capture/'initial-state.json').read_text()),*[json.loads(line) for line in trace.read_text().splitlines()]]
 for index,row in enumerate(rows):
  require(abs(row['time']-index/20)<1e-6,'missing control sample')
  require(set(row['native_owner_poses'])==set(manifest['owners']),'missing native body pose')
  for pose in row['native_owner_poses'].values():valid_pose(pose)
 if not diagnostic:
  proof=json.loads(Path(audit).read_text());result=json.loads((capture/'result.json').read_text())
  require(result['task_score']==1 and rows[-1]['metrics']['score']==1,'incomplete native task')
  require(proof.get('passed') is True and proof.get('trace_sha256')==trace_hash and proof.get('score')==1 and proof.get('control_rows')==len(rows)-1,'full score audit missing/mismatched')
 out.mkdir(parents=True,exist_ok=False);layer=Sdf.Layer.CreateNew(str((out/'replay.usdc').resolve()));layer.subLayerPaths=[str(scene.resolve())];layer.customLayerData={'label':LABEL,'traceSha256':trace_hash,'diagnostic':diagnostic}
 stage=Usd.Stage.Open(layer);stage.SetInterpolationType(Usd.InterpolationTypeHeld);stage.SetTimeCodesPerSecond(20);stage.SetFramesPerSecond(20);stage.SetStartTimeCode(0);stage.SetEndTimeCode(len(rows)-1)
 operations={path:ops(stage.GetPrimAtPath(path)) for path in manifest['owners']}
 for index,row in enumerate(rows):
  for path,pose in row['native_owner_poses'].items():
   t,r=operations[path];t.Set(Gf.Vec3d(*pose[:3]),index);r.Set(quaternion(pose[3:]),index)
 cameras={}
 for spec in manifest['virtual_review_views']:
  path='/OfflineReview/'+spec['name'];cam=UsdGeom.Camera.Define(stage,path);t,r=ops(cam.GetPrim());eye=Gf.Vec3d(*spec['eye']);target=Gf.Vec3d(*spec['target']);rotation=Gf.Matrix4d().SetLookAt(eye,target,Gf.Vec3d(0,0,1)).GetInverse().ExtractRotationQuat();t.Set(eye);r.Set(rotation)
  cam.CreateFocalLengthAttr(spec['focal_length_mm']);cam.CreateHorizontalApertureAttr(20.955);cam.CreateVerticalApertureAttr(20.955*spec['height']/spec['width']);cam.CreateClippingRangeAttr(Gf.Vec2f(.03,20));cameras[spec['name']]=path
 layer.Save();result=dict(label=LABEL,diagnostic=diagnostic,native_rgb=False,trace_sha256=trace_hash,scene_sha256=manifest['scene_sha256'],usd_file='replay.usdc',usd_sha256=sha(out/'replay.usdc'),frames=len(rows),first_frame=0,last_frame=len(rows)-1,fps=20,cameras=cameras,appearance='Captured geometry/materials and actual native actor poses; virtual fixed review views; no FK or physics; integer samples only.')
 (out/'replay-output-manifest.json').write_text(json.dumps(result,indent=2)+'\n');return result
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('capture',type=Path);p.add_argument('--out',type=Path,required=True);p.add_argument('--trace-sha256',required=True);p.add_argument('--audit',type=Path);p.add_argument('--diagnostic',action='store_true');a=p.parse_args();print(json.dumps(build(a.capture,a.out,a.trace_sha256,a.audit,a.diagnostic),indent=2))
