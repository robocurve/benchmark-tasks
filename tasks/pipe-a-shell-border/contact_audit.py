"""Conservative validity boundary for contacts the icing solver cannot resolve.

Only the metal nozzle lip has an explicit force coupling. If any other
moving rigid collider enters icing or cake, the trial becomes invalid rather
than allowing an invisible hand to pass through material and retain a score.
Collision-shape bounding boxes overestimate contact, so this gate can reject a
near miss; it cannot certify detailed finger/cream interaction. The normal
station/table support and the cake itself are explicitly excluded by path.
"""
from __future__ import annotations

from dataclasses import dataclass
import itertools
import numpy as np


def intersects_columns(corners, x, y, base, height, *, minimum_icing_height=.00005,
                       cake_mask=None, cake_depth_tolerance=.001, margin=.00025):
    """Conservative world-AABB overlap with the finite material column field.

    Corners come from the *measured* rigid-body transform, not a commanded
    end-effector pose. Half-cell margins cover area between grid centres.
    """
    c=np.asarray(corners,float);x,y,base,height=(np.asarray(v,float) for v in (x,y,base,height))
    if c.shape!=(8,3) or not np.isfinite(c).all():raise ValueError('Eight finite collider corners required')
    if base.shape!=(len(x),len(y)) or height.shape!=base.shape:raise ValueError('Invalid material grid')
    lo,hi=c.min(0),c.max(0)
    dx=max(float(np.diff(x).max()),float(np.diff(y).max()))/2+margin
    ix=np.flatnonzero((x>=lo[0]-dx)&(x<=hi[0]+dx))
    iy=np.flatnonzero((y>=lo[1]-dx)&(y<=hi[1]+dx))
    if not len(ix) or not len(iy):return False
    z=base[np.ix_(ix,iy)];h=height[np.ix_(ix,iy)]
    icing=(h>=minimum_icing_height)&(z+h+margin>=lo[2])&(z-margin<=hi[2])
    cake=False
    if cake_mask is not None:
        mask=np.asarray(cake_mask,bool)[np.ix_(ix,iy)]
        # The material model represents a 50 mm cake, not the whole column down
        # to the floor. Ignore bodies strictly below its board and above its top.
        cake=bool(np.any(mask&(lo[2]<z-cake_depth_tolerance)&(hi[2]>z-.05)))
    return bool(np.any(icing) or cake)


@dataclass
class Collider:
    path:str
    local_corners:np.ndarray


class NativeContactAudit:
    def __init__(self,stage,sim,material,*,excluded_body_paths,allowed_shape_paths=()):
        """Discover every other dynamic rigid collider, including robot links.

        We fail on an unknown collider type instead of silently dropping it.
        Shape-local AABBs are transformed into rigid-body space once, and then
        updated with tensor poses during physics. No collider/contact is changed.
        """
        from pxr import Gf,UsdGeom,UsdPhysics
        self.material=material;self.bodies=[];self.events=[];self.time_s=0.;self.body_bounds={}
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

    def update(self,time_s):
        from native import rotation
        m=self.material;height=m.height
        if self.view is None:return False
        value=self.view.get_transforms();value=getattr(value,'torch',value)
        poses=value.detach().cpu().numpy() if hasattr(value,'detach') else np.asarray(value)
        low=np.array((m.x[0]-m.dx/2,m.y[0]-m.dx/2,float(m.base.min())-.05))
        high=np.array((m.x[-1]+m.dx/2,m.y[-1]+m.dx/2,float((m.base+height).max())))
        padding=m.dx/2+.00025
        for path,row,shapes in self.bodies:
            pose=poses[row]
            r=rotation(pose[3:])
            bounds=self.body_bounds[path]@r.T+pose[:3]
            self.world_bounds[path]=[bounds.min(0).tolist(),bounds.max(0).tolist()]
            if np.any(bounds.max(0)<low-padding) or np.any(bounds.min(0)>high+padding):continue
            for collider in shapes:
                corners=collider.local_corners@r.T+pose[:3]
                if np.any(corners.max(0)<low-padding) or np.any(corners.min(0)>high+padding):continue
                if intersects_columns(corners,m.x,m.y,m.base,height,cake_mask=m.on_cake):
                    if not any(row['collider']==collider.path for row in self.events):
                        self.events.append({'time_s':float(time_s),'collider':collider.path,
                                            'reason':'outside modeled nozzle-only material contact'})
        self.time_s=float(time_s)
        return bool(self.events)

    def summary(self):
        return {'invalid':bool(self.events),'events':self.events,
                'rigid_bodies_checked':len(self.bodies),
                'colliders_checked':sum(len(shapes) for _,_,shapes in self.bodies),
                'method':'conservative measured collision-shape world AABBs'}
