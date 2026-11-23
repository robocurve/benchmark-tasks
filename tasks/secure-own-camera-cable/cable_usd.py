"""Pre-reset construction of a connected capsule cable. No runtime pose writes.
Connected capsule segments with spherical joints; elastic drives are configured by hardware.py.
Robot endpoint prim paths are explicit inputs, never guessed from stage names.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def build(stage, root_path, endpoint_prim_paths, geometry=None):
 from pxr import Gf,Sdf,UsdGeom,UsdPhysics,PhysxSchema
 if geometry is None:raise ValueError('Cable geometry is required')
 g=geometry
 if len(endpoint_prim_paths)!=2:raise ValueError('Exactly two physical attachments required')
 for path in endpoint_prim_paths:
  prim=stage.GetPrimAtPath(path)
  if not prim.IsValid() or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
   raise ValueError('Endpoint must be an existing rigid robot link: '+path)
 if stage.GetPrimAtPath(root_path).IsValid():raise ValueError('Cable already exists')
 UsdGeom.Xform.Define(stage,root_path)
 centers=np.array(g['centres_world']);lengths=np.array(g['lengths_m']);quats=np.array(g['rotations_xyzw']);vertices=np.array(g['vertices_world'])
 def vec(x):return Gf.Vec3f(*map(float,x))
 paths=[]
 for i,(c,L,q) in enumerate(zip(centers,lengths,quats)):
  path=root_path+f'/segment_{i:03d}';paths.append(path)
  xf=UsdGeom.Xform.Define(stage,path);xf.AddTranslateOp().Set(Gf.Vec3d(*map(float,c)));xf.AddOrientOp().Set(Gf.Quatf(float(q[3]),vec(q[:3])))
  prim=xf.GetPrim();UsdPhysics.RigidBodyAPI.Apply(prim);UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(g['mass_kg']/len(centers)))
  ph=PhysxSchema.PhysxRigidBodyAPI.Apply(prim);ph.CreateSolverPositionIterationCountAttr(32);ph.CreateSolverVelocityIterationCountAttr(8)
  ph.CreateLinearDampingAttr(.05);ph.CreateAngularDampingAttr(.05)
  cap=UsdGeom.Capsule.Define(stage,path+'/collision');cap.CreateAxisAttr('X');cap.CreateHeightAttr(float(L-2*g['radius_m']));cap.CreateRadiusAttr(g['radius_m']);cap.CreateDisplayColorAttr([Gf.Vec3f(.12,.18,.22)])
  UsdPhysics.CollisionAPI.Apply(cap.GetPrim());cp=PhysxSchema.PhysxCollisionAPI.Apply(cap.GetPrim());cp.CreateContactOffsetAttr(.0005);cp.CreateRestOffsetAttr(0.)
 def joint(name,a,b,pa,pb):
  j=UsdPhysics.SphericalJoint.Define(stage,root_path+'/'+name)
  j.CreateBody0Rel().SetTargets([Sdf.Path(a)]);j.CreateBody1Rel().SetTargets([Sdf.Path(b)])
  j.CreateLocalPos0Attr(vec(pa));j.CreateLocalPos1Attr(vec(pb))
  # Connected bodies must not collide at their shared endpoint.
  j.CreateCollisionEnabledAttr(False)
  j.CreateExcludeFromArticulationAttr(True)
  return j
 for i in range(len(paths)-1):joint(f'bend_{i:03d}',paths[i],paths[i+1],[lengths[i]/2,0,0],[-lengths[i+1]/2,0,0])
 joint('anchor_proximal',endpoint_prim_paths[0],paths[0],g['attachments'][0]['local_xyz'],[-lengths[0]/2,0,0])
 joint('anchor_distal',paths[-1],endpoint_prim_paths[1],[lengths[-1]/2,0,0],g['attachments'][1]['local_xyz'])
 return paths
