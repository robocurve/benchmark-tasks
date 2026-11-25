"""Video-equivalent tire clearance from the fixed floor/sill surfaces.

The floor and sill span the scored workspace in Y, so the distance is the
exact XZ separation of the projected convex tire and support profiles. This
uses visible geometry only, without hidden contact-force telemetry.
"""
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import spec


def tread_points(pose):
    angle=np.arange(64)*2*np.pi/64
    ring=np.c_[spec.WHEEL_RADIUS*np.cos(angle),spec.WHEEL_RADIUS*np.sin(angle)]
    local=np.concatenate([np.c_[ring,np.full(len(ring),z)] for z in [-spec.WHEEL_WIDTH/2,spec.WHEEL_WIDTH/2]])
    return local@Rotation.from_quat(pose[3:]).as_matrix().T+np.asarray(pose[:3])


def polygon_box_gap(polygon,low,high):
    """Distance between a CCW convex polygon and a closed axis-aligned box."""
    low,high=np.asarray(low),np.asarray(high)
    start=np.asarray(polygon);end=np.roll(start,-1,axis=0);direction=end-start
    # Segment/box slab test also detects crossings with no contained vertex.
    t0=np.zeros(len(start));t1=np.ones(len(start));possible=np.ones(len(start),dtype=bool)
    for axis in range(2):
        parallel=np.abs(direction[:,axis])<1e-14
        possible &= ~parallel | ((start[:,axis]>=low[axis])&(start[:,axis]<=high[axis]))
        inverse=np.divide(1.,direction[:,axis],out=np.zeros(len(start)),where=~parallel)
        a=(low[axis]-start[:,axis])*inverse;b=(high[axis]-start[:,axis])*inverse
        t0=np.maximum(t0,np.where(parallel,-np.inf,np.minimum(a,b)))
        t1=np.minimum(t1,np.where(parallel,np.inf,np.maximum(a,b)))
    if np.any(possible&(t0<=t1+1e-12)):return 0.
    corners=np.array([[low[0],low[1]],[high[0],low[1]],[high[0],high[1]],[low[0],high[1]]])
    offsets=corners[:,None,:]-start[None,:,:]
    cross=direction[None,:,0]*offsets[:,:,1]-direction[None,:,1]*offsets[:,:,0]
    if np.any(np.all(cross>=-1e-12,axis=1)):return 0.
    delta=np.maximum(np.maximum(low-start,start-high),0.)
    vertex_gap=float(np.linalg.norm(delta,axis=1).min())
    fraction=np.clip(np.sum(offsets*direction[None,:,:],axis=2)/np.sum(direction**2,axis=1),0.,1.)
    edge_gap=float(np.linalg.norm(offsets-fraction[:,:,None]*direction[None,:,:],axis=2).min())
    return min(vertex_gap,edge_gap)


def clearance(pose):
    points=tread_points(pose)[:,[0,2]]
    floor=max(0.,float(points[:,1].min()))
    if floor==0.:return 0.
    outline=points[ConvexHull(points).vertices]
    sill=polygon_box_gap(outline,[spec.SILL_X[0],0.],[spec.SILL_X[1],spec.SILL_HEIGHT])
    return min(floor,sill)


def support_gap(poses):
    """At least one tire must remain within the visible support tolerance."""
    return min(clearance(poses['wheel_'+side]) for side in ['left','right'])
