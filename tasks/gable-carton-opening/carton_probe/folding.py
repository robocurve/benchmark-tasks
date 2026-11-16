"""Exact rigid-fold geometry for initialization and mechanism checks only.

These transforms are not an automatic task animation or a runtime controller.
Native contacts/constraints still determine every post-reset object motion.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from . import geometry as G


def landmarks(outer_deg=0., spout_deg=0.):
    if not 0<=outer_deg<=120 or not 0<=spout_deg<=90:raise ValueError('Fold out of range')
    if spout_deg and outer_deg!=120:raise ValueError('Open side folds before the inner rigid-fold branch')
    axis=G.INNER-G.B_MINUS;axis=axis/np.linalg.norm(axis)
    tip=G.B_MINUS+Rotation.from_rotvec(axis*np.deg2rad(outer_deg)).apply(G.TIP-G.B_MINUS)
    apex=G.INNER.copy()
    if spout_deg:
        t=np.deg2rad(spout_deg)
        apex=np.array([-G.A+G.A*np.cos(t)-G.A*np.sin(t),0.,G.A*np.sin(t)+G.A*np.cos(t)])
        s=G.INNER;b=G.B_MINUS;d=np.linalg.norm(apex-s);ex=(apex-s)/d
        i=(b-s)@ex;j=np.linalg.norm(b-s-i*ex);ey=(b-s-i*ex)/j;ez=np.cross(ex,ey)
        x=d/2;y=(G.A**2-2*G.A**2+i*i+j*j-2*i*x)/(2*j)
        z2=G.A**2-x*x-y*y
        if z2 < -1e-10:raise ValueError('Inconsistent crease lengths')
        candidates=[s+x*ex+y*ey+sign*np.sqrt(max(0,z2))*ez for sign in (-1,1)]
        tip=min(candidates,key=lambda v:v[1])
    return apex,tip,tip*np.array([1.,-1.,1.])


def fit(original,desired):
    c0=original.mean(axis=0);c1=desired.mean(axis=0)
    u,_,vh=np.linalg.svd((original-c0).T@(desired-c1))
    r=vh.T@np.diag([1,1,np.linalg.det(vh.T@u.T)])@u.T
    if np.max(np.linalg.norm((original-c0)@r.T+c1-desired,axis=1))>1e-8:raise ValueError('Facet stretches')
    return c1,r


def poses(outer_deg=0.,spout_deg=0.):
    apex,minus,plus=landmarks(outer_deg,spout_deg)
    verts={
        'carton_spout':np.array([G.B_MINUS,G.B_PLUS,apex]),
        'carton_inner_minus':np.array([G.B_MINUS,minus,apex]),
        'carton_inner_plus':np.array([G.B_PLUS,apex,plus]),
        'carton_roof_minus':np.array([G.B_MINUS,G.INNER,minus]),
        'carton_roof_plus':np.array([G.B_PLUS,plus,G.INNER]),
    }
    result={n:fit(G.FACETS[n],v) for n,v in verts.items()}
    for side in ('minus','plus'):
        roof='carton_roof_'+side;fin='carton_fin_'+side;c,r=result[roof]
        result[fin]=(r@(G.CENTRES[fin]-G.CENTRES[roof])+c,r)
    # The two inner sealing-panel halves share their central crease. Merely
    # attaching rigid fins to the inner triangles would split that crease as
    # the mouth opens. Their common upward edge is perpendicular to BOTH lower
    # pouring edges; at the flat toggle its continuous limit is 45 degrees.
    edge = minus-apex
    up = np.array([-edge[2],0.,edge[0]])
    if np.linalg.norm(up)<1e-8:
        up=np.array([1.,0.,1.])/np.sqrt(2) if spout_deg<45 else np.array([0.,0.,1.])
    else:
        up/=np.linalg.norm(up)
        if up[2]<0:up=-up
    d=up*G.INNER_FIN
    result['carton_inner_fin_minus']=fit(G.FACETS['carton_inner_fin_minus'],np.array([minus,apex,apex+d,minus+d]))
    result['carton_inner_fin_plus']=fit(G.FACETS['carton_inner_fin_plus'],np.array([apex,plus,plus+d,apex+d]))
    result['carton_body']=(G.CENTRES['carton_body'].copy(),np.eye(3))
    return {n:dict(center=c.tolist(),quat_xyzw=Rotation.from_matrix(r).as_quat().tolist()) for n,(c,r) in result.items()}


def check_path():
    # Sweep both branches, testing every real hinge edge, not only one marker.
    worst=0.
    for a,b in [(v,0.) for v in np.linspace(0,120,121)]+[(120.,v) for v in np.linspace(.1,90,181)]:
        states=poses(a,b)
        for _,n0,n1,p0,p1 in G.HINGES:
            ends=[]
            for n in (n0,n1):
                r=Rotation.from_quat(states[n]['quat_xyzw'])
                ends.append(r.apply(np.array([p0,p1])-G.CENTRES[n])+states[n]['center'])
            worst=max(worst,float(np.max(np.linalg.norm(ends[0]-ends[1],axis=1))))
    return dict(configurations=302,max_crease_gap_m=worst,no_runtime_pose_control=True)
