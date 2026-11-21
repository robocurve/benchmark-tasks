"""Exact local vertices of every original device collision primitive.

Native cooking is independently checked to retain every convex-mesh vertex.
The minimum height of a convex shape is attained at a vertex, including the
clevis, pin, eye, grip and strain relief rather than just the visible barrel.
"""
from functools import lru_cache
import numpy as np
from . import fixed_scene as F

@lru_cache(maxsize=1)
def collision_vertices():
 from pxr import Gf,Usd,UsdGeom,UsdPhysics
 stage=Usd.Stage.Open(str(F.USD.resolve()));cache=UsdGeom.XformCache();out=[]
 if stage is None:raise RuntimeError('Cannot read device collision geometry')
 for name in ('head','handle'):
  actor=stage.GetPrimAtPath('/Fixture/Actors/'+name);inverse=cache.GetLocalToWorldTransform(actor).GetInverse();vertices=[];labels=[]
  for prim in actor.GetChildren():
   if not prim.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():continue
   if prim.IsA(UsdGeom.Mesh):points=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(),float)
   elif prim.IsA(UsdGeom.Cube):
    half=float(UsdGeom.Cube(prim).GetSizeAttr().Get())/2;points=np.array([(x,y,z) for x in (-half,half) for y in (-half,half) for z in (-half,half)])
   else:raise RuntimeError('Unsupported device collision primitive '+str(prim.GetPath()))
   T=cache.GetLocalToWorldTransform(prim);v=np.array([inverse.Transform(T.Transform(Gf.Vec3d(*p))) for p in points]);vertices.extend(v);labels.extend([name+'/'+prim.GetName()]*len(v))
  v=np.asarray(vertices,float)
  if v.ndim!=2 or v.shape[1]!=3 or not len(v) or not np.isfinite(v).all():raise RuntimeError('Invalid source collision vertices')
  v.setflags(write=False);out.append((v,tuple(labels)))
 return tuple(out)

def world_device_vertices(poses,rotations):
 rows=[];labels=[]
 for p,R,(v,names) in zip(poses,rotations,collision_vertices(),strict=True):rows.append(np.asarray(p[:3])+v@R.T);labels.extend(names)
 return np.concatenate(rows),labels
