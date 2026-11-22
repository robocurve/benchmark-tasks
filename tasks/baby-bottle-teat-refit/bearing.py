"""Estimate overlap beneath both physical thread flanks using vertical lines through the authored hulls."""
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from threads import slices,STARTS,LUG_Z,coords


def equations(vertices):
    # coordinates and tolerances in millimetres
    return [ConvexHull(np.asarray(v)*1000).equations for v in vertices]

MALE=equations(slices(False))
LUGS=list(slices(True))


def intervals(planes,xy):
    """Exact vertical line intersection with each convex component."""
    normal=planes[:,:3];signed=xy@normal[:,:2].T + planes[:,3]
    up=normal[:,2]>1.e-8;down=normal[:,2]<-1.e-8
    hi=(-signed[:,up]/normal[up,2]).min(axis=1)
    lo=(-signed[:,down]/normal[down,2]).max(axis=1)
    side=~(up|down)
    inside=(lo<=hi+1e-7)&(signed[:,side]<=1e-7).all(1)
    return lo,hi,inside


def inspect_pair(collar,bottle):
    """Fraction of 15 samples on each lug with a male flank directly above, gaps in mm."""
    b=Rotation.from_quat(bottle[3:])
    rel=b.inv()*Rotation.from_quat(collar[3:])
    xyz=b.inv().apply(collar[:3]-bottle[:3])*1000
    points=[]
    for k,start in enumerate(STARTS):
        samples=np.array([coords(r,start+d,LUG_Z) for d in (-3,-1.5,0,1.5,3) for r in (27.35,27.75,28.1)])
        xy=(rel.apply(samples)*1000+xyz)[:,:2]
        top=np.full(len(samples),-np.inf)
        for vertices in LUGS[2*k:2*k+2]:
            planes=ConvexHull(rel.apply(vertices)*1000+xyz).equations
            _,hi,ok=intervals(planes,xy)
            top=np.maximum(top,np.where(ok,hi,-np.inf))
        gaps=np.full(len(samples),np.inf)
        # Select the nearest flank above this lug. Overlap exceeding 0.4 mm is invalid.
        for planes in MALE:
            lo,_,ok=intervals(planes,xy)
            d=lo-top
            gaps=np.minimum(gaps,np.where(ok&(d>=-.4),d,np.inf))
        points.append(dict(under_fraction=float((gaps<=2.).mean()),
                           min_gap_mm=float(gaps.min()) if np.isfinite(gaps).any() else None))
    return dict(lugs=points,height_mm=float(xyz[2]),ecc_mm=float(np.linalg.norm(xyz[:2])),
                tilt_deg=float(np.rad2deg(np.arccos(np.clip(rel.as_matrix()[2,2],-1,1)))),
                yaw_deg=float(rel.as_euler('xyz',degrees=True)[2]))
