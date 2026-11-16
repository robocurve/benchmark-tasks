"""Read-only native episode capture/scoring shared by model and clock policies."""
import hashlib,json
from pathlib import Path
import numpy as np
from config import USED,LINKS,SERVICE,TARGET,CLIP_Z,PLUG_X,port


def array(value):
 value=value.torch if hasattr(value,'torch') else value
 return value.detach().cpu().numpy() if hasattr(value,'detach') else np.asarray(value)


def joint_state(env):
 from yam_arena.constants import FINGER_OPEN
 result=[]
 for side in ('left','right'):
  ids,fids=env._ids[side+'_robot'];q=array(env._base.scene[side+'_robot'].data.joint_pos)[0]
  result.extend([*q[ids],float(np.clip(q[fids].mean()/FINGER_OPEN,0.,1.))])
 return np.array(result,dtype=float)


class NativeEpisode:
 def __init__(self,env,out):
  from collision_geometry import FingerGeometry
  from scoring import EthernetScorer
  from yam_arena.robot import asset_link_paths
  self.env=env;self.base=env._base;self.out=Path(out);self.geometry=FingerGeometry()
  self.scorer=EthernetScorer();self.previous_plug=None;self.rows=0
  self.names=[n for i in USED for n in (f'plug_{i}',f'clip_{i}')]+[f'wire_{i}_{k}' for i in USED for k in range(LINKS)]
  self.owners={};links=asset_link_paths()
  for side,prim in (('left','LeftArm'),('right','RightArm')):
   entity=side+'_robot';names=list(self.base.scene[entity].body_names)
   if len(names)!=9:raise ValueError('expected nine actual native robot links')
   for i,n in enumerate(names):self.owners['/World/envs/env_0/'+prim+links[n]]=dict(kind='robot',entity=entity,index=i,name=n)
  for n in self.names:self.owners['/World/envs/env_0/'+n]=dict(kind='object',entity=n)
  for path in self.owners:
   if not self.base.scene.stage.GetPrimAtPath(path):raise ValueError('missing native actor '+path)
   parents=[p for p in self.owners if path.startswith(p+'/')]
   self.owners[path]['parent_body_owner']=max(parents,key=len) if parents else None
  layer=self.base.scene.stage.Flatten();scene=self.out/'capture-scene.usdc'
  if not layer.Export(str(scene)):raise RuntimeError('native scene export failed')
  x,y,z=.41,0.,.96
  cameras=[dict(name=n,eye=list(e),target=[x,y,z],focal_length_mm=18.,width=480,height=270) for n,e in zip(('review_top','review_left','review_right'),((x-.40,y-.32,z+.43),(x-.30,y+.32,z+.25),(x-.30,y-.30,z+.23)))]
  manifest=dict(scene_file=scene.name,scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),owners=self.owners,capture_hz=20,quaternion_order='xyzw',pose_space='world',interpolated=False,virtual_review_views=cameras,scope='actual native poses; offline rendering is not native RGB')
  (self.out/'capture-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
  self.stream=(self.out/'trajectory.jsonl').open('w')
  (self.out/'initial-state.json').write_text(json.dumps(self.read(),indent=2)+'\n')

 def read(self):
  import torch
  from diagnostics import robot_snapshot,connector_fit
  chunks=[]
  for n in self.names:
   v=self.base.arena_world.get_pose_w(n);chunks.append(v.torch if hasattr(v,'torch') else v)
  raw=array(torch.cat(chunks,dim=0));bodies={n:raw[i].copy() for i,n in enumerate(self.names)}
  robot={side:array(self.base.scene[side+'_robot'].data.body_link_pose_w)[0].copy() for side in ('left','right')}
  owners={}
  for path,spec in self.owners.items():
   p=bodies[spec['entity']] if spec['kind']=='object' else robot[spec['entity'].split('_')[0]][spec['index']]
   if p.shape!=(7,) or not np.isfinite(p).all() or abs(np.linalg.norm(p[3:])-1)>1e-3:raise ValueError('invalid native actor pose')
   owners[path]=p.tolist()
  return dict(time=self.env._t,poses={n:p.tolist() for n,p in bodies.items()},native_owner_poses=owners,
              native_hands=robot_snapshot(self.env,array),connector_fit={str(i):connector_fit(bodies[f'plug_{i}'],bodies[f'clip_{i}'],i) for i in USED})

 def step(self,action):
  from feedback import retaining_face,clearance
  from yam_arena.urdf import quat_xyzw_to_matrix
  row=self.read();bodies={n:np.array(p) for n,p in row['poses'].items()};p=bodies[f'plug_{SERVICE}'];c=bodies[f'clip_{SERVICE}'];R=quat_xyzw_to_matrix(p[3:]);travel=float(CLIP_Z-(R.T@(c[:3]-p[:3]))[2]);tip=p[:3]+R@np.array([.012,0.,0.])
  def insertion(i):
   center=np.array(port(i));return float(tip[0]-(.412-.027/2)) if np.linalg.norm(p[1:3]-center[1:3])<.007 and R[0,0]>.8 else -.020
  target=np.array(port(TARGET));face=retaining_face(c,TARGET);tcp_clear=all(np.linalg.norm(np.array(h['tcp'])-p[:3])>.070 for h in row['native_hands']);mesh=None
  if tcp_clear:
   parts=[]
   for h in row['native_hands']:parts.extend(a@np.array(h['tcp_rotation']).T+np.array(h['tcp']) for _,a in self.geometry.pieces(h['finger_joints']))
   state=({i:bodies[f'plug_{i}'] for i in USED},{i:bodies[f'clip_{i}'] for i in USED},{(i,k):bodies[f'wire_{i}_{k}'] for i in USED for k in range(LINKS)})
   mesh=clearance(parts,state)
  sample=dict(time=self.env._t,clip_travel=travel,source_insertion=insertion(SERVICE),target_insertion=insertion(TARGET),target_lateral_error=float(np.linalg.norm(p[1:3]-target[1:3])),target_angle=float(np.arccos(np.clip(R[0,0],-1.,1.))),speed=0. if self.previous_plug is None else float(np.linalg.norm(p[:3]-self.previous_plug)/.05),height=float(p[2]),max_neighbour_withdrawal=max(max(0.,PLUG_X-bodies[f'plug_{i}'][0]) for i in USED if i!=SERVICE),hands_clear=bool(tcp_clear and mesh is not None and mesh['minimum_gap_m']>=.005),hook_retained=face['retained'])
  self.previous_plug=p[:3].copy();self.rows+=1
  row.update(action_14d=np.asarray(action,dtype=np.float32).astype(float).tolist(),score_sample=sample,metrics=self.scorer.update(sample),actual_retaining_face=face,hands_mesh_clearance=mesh,neighbour_retaining_faces={str(i):retaining_face(bodies[f'clip_{i}'],i) for i in USED if i!=SERVICE})
  self.stream.write(json.dumps(row,allow_nan=False)+'\n');self.stream.flush()
  (self.out/'result.json').write_text(json.dumps(dict(task_score=self.scorer.value,metrics=row['metrics'],steps=self.rows,source='actual native state; no external bench forces'),indent=2)+'\n')
  return row

 def close(self):self.stream.close()
