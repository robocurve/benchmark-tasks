"""Fit the last component against measured, rigid collider footprints.

This returns robot placement targets. It never modifies objects or scoring.
Arena's geometry centroid is the transformed LOCAL bounds centre, not a new
world-axis-aligned bounds centre; root poses are reconstructed accordingly.
"""
import math
from functools import lru_cache
import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

@lru_cache(maxsize=32)
def _shape(geometry,index):
    cells=geometry.cells_for(index)
    c=np.array([(min(x for x,_ in cells)+max(x for x,_ in cells)+1)*geometry.STEP/2,
                (min(y for _,y in cells)+max(y for _,y in cells)+1)*geometry.STEP/2,0.])
    blocks=[]
    for x0,x1,y0,y1 in geometry.rectangles(cells):
        blocks.append((np.array([(x0+x1)*geometry.STEP/2,(y0+y1)*geometry.STEP/2,0.]),
                       np.array([(x1-x0)*geometry.STEP/2,(y1-y0)*geometry.STEP/2,geometry.HEIGHT/2])))
    return c,blocks

def fit_corner(objects,geometry,goals,initial_xy,initial_yaw):
    static=[]
    for i in range(3):
        obj=objects[f'square_api_piece_{i+1}']
        R=quat_xyzw_to_matrix(obj.quat_xyzw)
        centroid,blocks=_shape(geometry,i);root=obj.pos-R@centroid
        axes=np.array([[-R[1,j],R[0,j]] for j in range(3)])
        norms=np.linalg.norm(axes,axis=1);axes=axes[norms>1e-10]/norms[norms>1e-10,None]
        for center,half in blocks:static.append(((root+R@center)[:2],R,half,axes))
    center4,blocks4=_shape(geometry,3)
    initial_xy=np.asarray(initial_xy);goals=np.asarray(goals);best=None
    for stage in (0,1):
        center=initial_xy if best is None else np.array(best['xy'])
        yawcenter=initial_yaw if best is None else best['yaw']
        span,step,angspan,angstep=(.0025,.000125,1.5,.1) if stage==0 else (.0002,.000025,.2,.025)
        offsets=np.array([(x,y) for x in np.arange(-span,span+step/2,step) for y in np.arange(-span,span+step/2,step)])
        candidates=center+offsets
        valid=np.linalg.norm(candidates-goals[3],axis=1)<.0039
        for i in (1,2):
            actual=objects[f'square_api_piece_{i+1}'].pos[:2]
            valid &= np.max(np.abs(candidates-actual-(goals[3]-goals[i])),axis=1)<.0019
        candidates=candidates[valid]
        if not len(candidates):raise ValueError('No last-piece target lies within the original placement and seam tolerances')
        for yaw in yawcenter+np.deg2rad(np.arange(-angspan,angspan+angstep/2,angstep)):
            if abs(yaw)>math.radians(7.5):continue
            c,s=math.cos(yaw),math.sin(yaw);R=np.array([[c,-s],[s,c]])
            gaps=np.full(len(candidates),np.inf)
            for center,half in blocks4:
                ac=R@(center[:2]-center4[:2])
                for bc,br,bh,bax in static:
                    axes=np.r_[R.T,bax]
                    radius=np.abs(axes@R)@half[:2]+np.abs(axes@br[:2,:])@bh
                    separation=(np.abs((candidates+ac-bc)@axes.T)-radius).max(axis=1)
                    gaps=np.minimum(gaps,separation)
            objective=gaps-.005*np.linalg.norm(candidates-initial_xy,axis=1)
            k=int(objective.argmax())
            if best is None or objective[k]>best['objective']:
                best={'objective':float(objective[k]),'clearance':float(gaps[k]),'xy':candidates[k].tolist(),'yaw':float(yaw)}
    return best
