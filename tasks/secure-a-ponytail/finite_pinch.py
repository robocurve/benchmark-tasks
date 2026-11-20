"""Geometry-gated native vertex pinch, with an approximate elastic-tension slip guard.

The central 8 mm of the closing finger faces must enclose the 3 mm cord. A native attachment then
binds the measured contact patch to the real gripper link. Opening, estimated load
above 18 N, or 8 mm slip releases it. No task-body or material pose is assigned.
The tension estimate is uncalibrated; it is not a measured native force limit.
"""
import numpy as np


def candidate(point,radius,hand,thickness,half_width,z_range):
 local=(np.asarray(point)-hand['tcp'])@hand['rotation']
 q=np.asarray(hand.get('finger_joint_positions',[-hand['opening']*.04695]*2))
 left=q[0]+thickness;right=-q[1]-thickness
 if abs((local[0]-radius)-left)>.001 or abs((local[0]+radius)-right)>.001:return None
 if abs(local[1])>half_width or not z_range[0]<=local[2]<=z_range[1]:return None
 return float(np.linalg.norm(local))


class FinitePinch:
 def __init__(self,model,names,radius,thickness,half_width,z_range):
  from pxr import UsdPhysics
  self.m=model;self.names=list(names);self.radius=radius;self.thickness=thickness
  self.half_width=half_width;self.z_range=z_range;self.held={};self.history={s:[] for s in ['left','right']};self.blocked=set();self.serial=0
  self.grippers={};self.hand_bodies={s:[] for s in ["left","right"]};root=None
  for p in model.stage.Traverse():
   path=str(p.GetPath())
   if p.HasAPI(UsdPhysics.RigidBodyAPI) and p.GetName()=='gripper':
    for side,arm in [('left','LeftArm'),('right','RightArm')]:
     if arm in path:self.grippers[side]=path
   if p.HasAPI(UsdPhysics.RigidBodyAPI) and p.GetName() in ('gripper','tip_left','tip_right'):
    for side,arm in [('left','LeftArm'),('right','RightArm')]:
     if arm in path:self.hand_bodies[side].append(path)
   if '/materials/' in path and p.GetName()=='elastic':root=path.rsplit('/',1)[0]
  assert set(self.grippers)=={'left','right'} and root is not None
  self.root=root
  # Fixed from reset: the soft surface supplies finger contact. These small
  # rigid patches provide hair/head contact without duplicate finger contact.
  from pxr import Sdf
  for name in self.names:
   prim=model.stage.GetPrimAtPath(root+'/'+name)
   pairs=UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel()
   pairs.SetTargets(pairs.GetTargets()+[Sdf.Path(path) for paths in self.hand_bodies.values() for path in paths])
 def release(self,side,reason):
  g=self.held.pop(side);g['attachment'].GetAttribute('omniphysics:attachmentEnabled').Set(False)
  for f in g['filters']:f.GetAttribute('omniphysics:filterEnabled').Set(False)
  self.m.events.append(dict(event='pinch_released' if reason=='opened' else 'pinch_slipped',side=side,body=g['body'],time=self.m.time,reason=reason))
  if reason!='opened':self.blocked.add(side)
 def tension(self,indices):
  from geometry import M,POINTS
  # Load enters the held patch at its exposed cord segments. Checking only
  # the central vertex would miss the load once its neighbours are held too.
  held=set(indices);loads=[]
  for index in held:
   p=self.m.nodes[M+index]
   for j in [(index-1)%M,(index+1)%M]:
    if j in held:continue
    length=np.linalg.norm(self.m.nodes[M+j]-p)
    rest=np.linalg.norm(POINTS[M+j]-POINTS[M+index])
    from geometry import ELASTIC_YOUNGS_MODULUS
    loads.append(ELASTIC_YOUNGS_MODULUS*np.pi*.0015**2*max(0.,length/rest-1.))
  return max(loads,default=0.)
 def update(self,hands):
  from pxr import Sdf,Vt,UsdPhysics
  from geometry import M,TRIS
  from yam_arena.constants import TCP_OFFSET
  for side,h in hands.items():
   history=self.history[side];history.append(float(h['opening']));del history[:-8]
   q=np.asarray(h.get('finger_joint_positions',[-h['opening']*.04695]*2));aperture=-q.sum()-2*self.thickness
   if aperture>2*self.radius+.002:
    if side in self.held:self.release(side,'opened')
    if h['opening']>.30:self.blocked.discard(side)
    continue
   if side in self.held:
    g=self.held[side];point=self.m.nodes[M+g['index']];local=(point-h['tcp'])@h['rotation'];g['estimated_load_n']=self.tension(g['indices'])
    if g['estimated_load_n']>18.:self.release(side,'estimated local cord tension exceeded 18 N')
    elif np.linalg.norm(local-g['local_point'])>.008:self.release(side,'material moved 8 mm through the grip')
   # Wait for close opposing-face contact before fixing the initial friction patch.
   # Capturing the first edge while the jaws are still wider locks an asymmetric arc.
   if side in self.held or side in self.blocked or max(history)-h['opening']<.008 or aperture>2*self.radius+.00075:continue
   used={g['body'] for g in self.held.values()};choices=[]
   for name in self.names:
    if name in used:continue
    index=int(name.rsplit('_',1)[1]);point=self.m.nodes[M+index]
    distance=candidate(point,self.radius,h,self.thickness,self.half_width,self.z_range)
    if distance is not None:choices.append((distance,name,index,point.copy()))
   if not choices:continue
   _,name,index,point=min(choices,key=lambda x:x[0]);self.serial+=1
   # The primary pinch must be centred, but a real flat face also holds
   # adjacent cord already enclosed by that same face. Preserve that finite
   # contact patch rather than treating a broad finger as a frictionless pin.
   ids=[]
   for row in range(3):
    for offset in range(-8,9):
     j=row*M+(index+offset)%M;v=self.m.nodes[j]
     if candidate(v,self.radius,h,self.thickness,.012,self.z_range) is not None:ids.append(j)
   if M+index not in ids:ids.append(M+index)
   ids=sorted(set(ids));locals=(self.m.nodes[ids]-h['tcp'])@h['rotation']
   a=self.m.stage.DefinePrim(self.root+f'/finger_pinch_{side}_{self.serial}','OmniPhysicsVtxXformAttachment')
   a.GetAttribute('omniphysics:attachmentEnabled').Set(False)
   local=(point-h['tcp'])@h['rotation']
   with Sdf.ChangeBlock():
    a.GetRelationship('omniphysics:src0').SetTargets([Sdf.Path(self.root+'/elastic')])
    a.GetRelationship('omniphysics:src1').SetTargets([Sdf.Path(self.grippers[side])])
    a.GetAttribute('omniphysics:vtxIndicesSrc0').Set(ids)
    a.GetAttribute('omniphysics:localPositionsSrc1').Set(Vt.Vec3fArray((locals+np.asarray(TCP_OFFSET)).astype(np.float32).tolist()))
    a.GetAttribute('omniphysics:attachmentEnabled').Set(True)
   # The pinch attachment represents local friction. Avoid contradictory
   # collision constraints only at this held mesh patch, as PhysX recommends.
   triangles=np.flatnonzero(np.any(np.isin(TRIS,ids),axis=1)).tolist();filters=[]
   for n,hand_path in enumerate(self.hand_bodies[side]):
    f=self.m.stage.DefinePrim(self.root+f'/pinch_filter_{side}_{self.serial}_{n}','OmniPhysicsElementCollisionFilter')
    f.GetAttribute('omniphysics:filterEnabled').Set(False)
    with Sdf.ChangeBlock():
     f.GetRelationship('omniphysics:src0').SetTargets([Sdf.Path(self.root+'/elastic')])
     f.GetRelationship('omniphysics:src1').SetTargets([Sdf.Path(hand_path)])
     f.GetAttribute('omniphysics:groupElemCounts0').Set([len(triangles)])
     f.GetAttribute('omniphysics:groupElemIndices0').Set(triangles)
     f.GetAttribute('omniphysics:groupElemCounts1').Set([0])
     f.GetAttribute('omniphysics:groupElemIndices1').Set([])
     f.GetAttribute('omniphysics:filterEnabled').Set(True)
    filters.append(f)
   self.held[side]=dict(body=name,index=index,indices=[j-M for j in ids if M<=j<2*M],attachment=a,local_point=local.copy(),estimated_load_n=0.,filters=filters)
   self.m.events.append(dict(event='pinch_captured',side=side,body=name,time=self.m.time,slip_estimate_limit_n=18.,source='native contact patch attachment to actual gripper link',contact_vertices=ids))
  self.m.grasp_state={side:dict(body=g['body'],estimated_load_n=g['estimated_load_n'],slip_estimate_limit_n=18.) for side,g in self.held.items()}
