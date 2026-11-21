"""Conservative lower bound on separation between the two sets of collision meshes."""
import numpy as np

STOP_MM = 15.0

def local_bounds(stage,links):
    from pxr import Usd,UsdGeom,UsdPhysics
    cache=UsdGeom.XformCache()
    bounds={}
    for prim in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        owner=prim
        while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
            owner=owner.GetParent()
        if not owner or str(owner.GetPath()) not in links:
            continue
        if not prim.IsA(UsdGeom.Mesh):
            raise ValueError(f"Unsupported arm collider {prim.GetPath()}")
        local=cache.GetLocalToWorldTransform(prim)*cache.GetLocalToWorldTransform(owner).GetInverse()
        v=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())@np.asarray(local)[:3,:3]+np.asarray(local)[3,:3]
        bounds.setdefault(str(owner.GetPath()),[]).append(v)
    if set(bounds)!=set(links):
        raise ValueError(f"Missing link geometry for {sorted(set(links)-set(bounds))}")
    result=[]
    for link in links:
        v=np.concatenate(bounds[link])
        result.append([v.min(axis=0),v.max(axis=0)])
    return np.asarray(result)

class Separation:
    def __init__(self,bounds,names):
        self.center=bounds.mean(axis=2)
        self.half=(bounds[:,:,1]-bounds[:,:,0])/2
        self.names=names

    def measure(self,pos,rot):
        center=pos+(rot@self.center[...,None])[...,0]
        radius=(abs(rot)@self.half[...,None])[...,0]
        if not np.isfinite(center).all() or not np.isfinite(radius).all():
            raise ValueError("Nonfinite arm transform")
        gap=np.maximum(0,abs(center[0,:,None]-center[1,None,:])-radius[0,:,None]-radius[1,None,:])
        squared=np.sum(gap*gap,axis=-1)
        i,j=np.unravel_index(squared.argmin(),squared.shape)
        return float(np.sqrt(squared[i,j])*1000),[self.names[0][i],self.names[1][j]]

def rotation_xyzw(q):
    if not np.allclose(np.linalg.norm(q,axis=-1),1,atol=1e-4):
        raise ValueError("Invalid link quaternion")
    x,y,z,w=np.moveaxis(q,-1,0)
    return np.stack([1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)],axis=-1).reshape((*q.shape[:-1],3,3))
