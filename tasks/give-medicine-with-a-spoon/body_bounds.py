"""Measured bounds of every native dynamic collision shape; no physics edits."""
from dataclasses import dataclass
import itertools
import numpy as np

@dataclass
class Collider:
    path:str
    local_corners:np.ndarray


class NativeBodyBounds:
    def __init__(self,stage,sim,*,excluded_body_paths=(),allowed_shape_paths=()):
        """Discover every other dynamic rigid collider, including robot links.

        We fail on an unknown collider type instead of silently dropping it.
        Shape-local AABBs are transformed into rigid-body space once, and then
        updated with tensor poses during physics. No collider/contact is changed.
        """
        from pxr import Gf,UsdGeom,UsdPhysics
        self.bodies=[];self.events=[];self.time_s=0.;self.body_bounds={}
        self.world_bounds={}
        excluded=set(excluded_body_paths);allowed=set(allowed_shape_paths)
        cache=UsdGeom.XformCache()
        grouped={}
        for prim in stage.Traverse():
            if not prim.HasAPI(UsdPhysics.CollisionAPI) or str(prim.GetPath()) in allowed:continue
            body=prim
            while body and not body.IsPseudoRoot() and not body.HasAPI(UsdPhysics.RigidBodyAPI):body=body.GetParent()
            if not body or body.IsPseudoRoot() or str(body.GetPath()) in excluded:continue
            if not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():continue
            if prim.IsA(UsdGeom.Mesh):
                pts=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(),float)
                if pts.ndim!=2 or pts.shape[1]!=3 or not len(pts):raise RuntimeError('Collider mesh lacks points')
                low,high=pts.min(0),pts.max(0)
            elif prim.IsA(UsdGeom.Cube):
                half=float(UsdGeom.Cube(prim).GetSizeAttr().Get())/2
                low=np.full(3,-half);high=-low
            elif prim.IsA(UsdGeom.Sphere):
                radius=float(UsdGeom.Sphere(prim).GetRadiusAttr().Get());low=np.full(3,-radius);high=-low
            elif prim.IsA(UsdGeom.Capsule) or prim.IsA(UsdGeom.Cylinder):
                capsule=prim.IsA(UsdGeom.Capsule)
                shape=UsdGeom.Capsule(prim) if capsule else UsdGeom.Cylinder(prim)
                radius=float(shape.GetRadiusAttr().Get());half=float(shape.GetHeightAttr().Get())/2
                axis='XYZ'.index(str(shape.GetAxisAttr().Get()))
                high=np.full(3,radius);high[axis]=half+(radius if capsule else 0);low=-high
            else:raise RuntimeError(f'Unsupported material-contact collider type: {prim.GetTypeName()}')
            relative=cache.GetLocalToWorldTransform(prim)*cache.GetLocalToWorldTransform(body).GetInverse()
            corners=np.array([relative.Transform(Gf.Vec3d(*p)) for p in itertools.product(*zip(low,high))])
            path=str(body.GetPath())
            grouped.setdefault(path,[]).append(Collider(str(prim.GetPath()),corners))
        self.view=sim.create_rigid_body_view(list(grouped)) if grouped else None
        paths=list(self.view.prim_paths) if self.view is not None else []
        if len(paths)!=len(grouped) or set(paths)!=set(grouped):raise RuntimeError('Missing measured material-contact body')
        self.bodies=[(path,row,grouped[path]) for row,path in enumerate(paths)]
        for path,_,shapes in self.bodies:
            corners=np.concatenate([shape.local_corners for shape in shapes])
            self.body_bounds[path]=np.array(list(itertools.product(*zip(corners.min(0),corners.max(0)))))

    def read(self):
        from common import rotation
        value=self.view.get_transforms();value=getattr(value,'torch',value)
        poses=value.detach().cpu().numpy()
        bounds={}
        for path,row,shapes in self.bodies:
            pose=poses[row];R=rotation(pose[3:])
            for shape in shapes:
                p=shape.local_corners@R.T+pose[:3]
                bounds[shape.path]=[p.min(0).tolist(),p.max(0).tolist()]
        return bounds
