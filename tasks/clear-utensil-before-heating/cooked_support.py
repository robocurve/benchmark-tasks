"""Read actual immutable PhysX-cooked local hull vertices once; no writes or fallback."""
import numpy as np
_vertices=None

def vertices():
 if _vertices is None:raise RuntimeError('Actual cooked support geometry unavailable')
 return _vertices

def prepare(environment):
 global _vertices
 if _vertices is not None:return
 from pxr import UsdUtils,UsdGeom,Usd,PhysicsSchemaTools,Gf
 from omni.physx import get_physx_cooking_interface
 from omni.physx.bindings._physx import PhysxCollisionRepresentationResult
 stage=environment.unwrapped.sim.stage;root=stage.GetPrimAtPath('/World/envs/env_0/wood')
 stage_id=UsdUtils.StageCache.Get().GetId(stage).ToLongInt();cache=UsdGeom.XformCache(Usd.TimeCode.Default())
 parts=[];results=[];count=0
 for mesh in Usd.PrimRange(root):
  if not mesh.GetName().startswith('hull_'):continue
  count+=1;transform=cache.ComputeRelativeTransform(mesh,root)[0]
  def receive(result,convexes,transform=transform):
   results.append(result)
   if result!=PhysxCollisionRepresentationResult.RESULT_VALID:return
   for convex in convexes:
    v=np.array([list(transform.Transform(Gf.Vec3d(*map(float,p)))) for p in convex.vertices])
    if len(v):parts.append(v)
  get_physx_cooking_interface().request_convex_collision_representation(stage_id=stage_id,collision_prim_id=PhysicsSchemaTools.sdfPathToInt(str(mesh.GetPath())),run_asynchronously=False,on_result=receive)
 if not count or len(results)!=count or any(r!=PhysxCollisionRepresentationResult.RESULT_VALID for r in results) or not parts:
  raise RuntimeError('Actual cooked support query failed; refusing to score')
 value=np.concatenate(parts)
 if value.ndim!=2 or value.shape[1]!=3 or len(value)<4 or not np.isfinite(value).all():raise RuntimeError('Invalid actual cooked support geometry')
 _vertices=value
 print('ACTUAL_COOKED_SUPPORT hulls',count,'vertices',len(value),'local_bounds',value.min(0).tolist(),value.max(0).tolist(),flush=True)
