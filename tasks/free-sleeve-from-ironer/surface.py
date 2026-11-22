"""Lower bounds on right moving-link separation from the station, using USD collider boxes."""
import json
from pathlib import Path
import numpy as np
from fixture import WORLD
from yam_arena import station

def boxes(entries):
    values = np.asarray(entries,dtype=float).reshape(-1,6)
    lo,hi = values[:,:3],values[:,3:]
    return (lo+hi)/2,(hi-lo)/2

_DATA = json.loads((Path(__file__).parent/"assets/yam-collider-bounds.json").read_bytes())
BOUNDS = {name:boxes(np.array(entries)*1e-6) for name,entries in _DATA["links"].items()}
MOVING = tuple(name for name in BOUNDS if name!="base")
MOUNT = {k:np.array(v)*1e-6 for k,v in _DATA["mount_vertices"].items()}
BASE_C, BASE_R = BOUNDS["base"]
BASE_ORIGINS = {s:np.array(station.BASES[s])-WORLD for s in ("left","right")}
FEED_C = np.array([-.219,-.043,.054])
FEED_R = np.array([.198,.211,.016])
BENCH_C = np.array(station.TABLE_CENTER)-WORLD
BENCH_R = np.array(station.TABLE_SIZE)/2

def gap(c1,r1,c2,r2):
    # One world-axis separation is enough to certify disjoint boxes.
    return np.maximum(0.,np.max(np.abs(c1-c2)-r1-r2,axis=-1))

def clearance(poses):
    centres=[];radii=[];own=[]
    for name in MOVING:
        position,rotation=poses[name]
        c,r=BOUNDS[name]
        centres.append(c@rotation.T+position)
        radii.append(r@np.abs(rotation.T))
        own.extend([name not in ("link1","link2")]*len(c))  # base and link1 share the mounting joint
    c=np.concatenate(centres);r=np.concatenate(radii)
    def base(side,mask):
        b=BASE_C+BASE_ORIGINS[side]
        return float(gap(c[mask,None,:],r[mask,None,:],b[None,:,:],BASE_R[None,:,:]).min())
    # Project actual shoulder collider vertices on the vertical axis.
    p,rot = poses["link2"]
    z_min = float((MOUNT["link2"]@rot[2]).min()+p[2])
    shoulder_gap = max(0.,z_min-(MOUNT["base"][:,2].max()+BASE_ORIGINS["right"][2])-2e-6)
    both_base=min(shoulder_gap,base("right",np.array(own)),base("left",slice(None)))
    return dict(base_clear_m=both_base,feed_clear_m=float(gap(c,r,FEED_C,FEED_R).min()),
                bench_clear_m=float(gap(c,r,BENCH_C,BENCH_R).min()))
