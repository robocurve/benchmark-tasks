"""Conservative separation of authored collision envelopes, including runtime poses."""
from pathlib import Path
import json
import numpy as np

ROOT=Path(__file__).resolve().parent


def export_envelopes():
    from pxr import Usd, UsdGeom, UsdPhysics
    result={}
    for file in ('spoon_clean.usda','spoon_used.usda','raw_chicken.usda'):
        stage=Usd.Stage.Open(str(ROOT/'assets'/file))
        cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','guide'],False,True)
        bounds=[]
        for p in stage.Traverse():
            if p.HasAPI(UsdPhysics.CollisionAPI):
                b=cache.ComputeWorldBound(p).ComputeAlignedRange()
                lo,hi=np.array(b.GetMin()),np.array(b.GetMax())
                assert np.isfinite(lo).all() and np.isfinite(hi).all()
                bounds.append([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
        result[file]=bounds
    (ROOT/'validation/collision_envelopes.json').write_text(json.dumps(result)+'\n')
    return result


def rotation(pose):
    x,y,z,w=np.asarray(pose[3:])
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def distance(corners_a,pose_a,corners_b,pose_b):
    ra,rb=rotation(pose_a),rotation(pose_b)
    points_a=np.asarray(corners_a)@ra.T+pose_a[:3]
    points_b=np.asarray(corners_b)@rb.T+pose_b[:3]
    lo_a,hi_a=points_a.min(axis=1),points_a.max(axis=1)
    lo_b,hi_b=points_b.min(axis=1),points_b.max(axis=1)
    gap=np.maximum(np.maximum(lo_a[:,None,:]-hi_b[None,:,:],lo_b[None,:,:]-hi_a[:,None,:]),0)
    lower=np.linalg.norm(gap,axis=-1)
    # Oriented-box separating axes avoid false overlap from long diagonal handles.
    axes=[*np.eye(3),*ra.T,*rb.T]
    axes += [np.cross(a,b) for a in ra.T for b in rb.T]
    axes=np.array([a/np.linalg.norm(a) for a in axes if np.linalg.norm(a)>1e-7])
    pa,pb=points_a@axes.T,points_b@axes.T
    la,ha=pa.min(axis=1),pa.max(axis=1)
    lb,hb=pb.min(axis=1),pb.max(axis=1)
    gap=np.maximum(np.maximum(la[:,None,:]-hb[None,:,:],lb[None,:,:]-ha[:,None,:]),0)
    return float(np.maximum(lower,gap.max(axis=-1)).min())


def measure(envelopes,poses):
    clean=envelopes['spoon_clean.usda']; used=envelopes['spoon_used.usda']; raw=envelopes['raw_chicken.usda']
    return {'clean_to_raw_meat_m':min(distance(clean,poses['CleanSpoon'],raw,p) for name,p in poses.items() if name.startswith('RawChicken_')),
            'spoon_to_spoon_m':distance(clean,poses['CleanSpoon'],used,poses['UsedSpoon'])}
