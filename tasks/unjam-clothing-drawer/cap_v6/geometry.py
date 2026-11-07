"""Same conservative hull-box test and margins; grouped/vectorized broad phase.
All 315 collision-hull boxes remain. No collision checks or segment samples are removed.
"""
from functools import lru_cache
import numpy as np
from yam_arena.analysis import link_hulls, fk_link_poses_world
from yam_arena.constants import ARM_JOINTS,FINGER_JOINTS,FINGER_OPEN
from yam_arena.station import BASES

class Boxes:
    def __init__(self,names,c,r,h,group_ids):
        self.names=names;self.c=np.asarray(c);self.r=np.asarray(r);self.h=np.asarray(h)
        self.e=np.einsum('nij,nj->ni',abs(self.r),self.h)
        self.groups=[np.flatnonzero(np.asarray(group_ids)==i) for i in np.unique(group_ids)]
        self.lo=np.array([(self.c[g]-self.e[g]).min(0) for g in self.groups])
        self.hi=np.array([(self.c[g]+self.e[g]).max(0) for g in self.groups])
    def __len__(self):return len(self.names)
    def __getitem__(self,i):return self.names[i],self.c[i],self.r[i],self.h[i]

@lru_cache(None)
def local_arrays():
    names=[];centers=[];half=[];links=[];indices=[]
    for index,(link,hulls) in enumerate(link_hulls().items()):
        links.append(link)
        for n,hull in enumerate(hulls):
            lo=hull.vertices.min(0);hi=hull.vertices.max(0)
            names.append(f'{link}/{n}');centers.append((lo+hi)/2);half.append((hi-lo)/2);indices.append(index)
    return names,np.array(centers),np.array(half),links,np.array(indices)

@lru_cache(maxsize=256)
def cached_arm(q,arm):
    # Eight physical coordinates: six revolute joints and two independent
    # prismatic finger positions in metres. Never average measured fingers.
    joints=dict(zip(ARM_JOINTS,q[:6]));joints.update(dict(zip(FINGER_JOINTS,q[6:8])))
    poses=fk_link_poses_world(joints,BASES[arm])
    names,c,h,links,indices=local_arrays()
    t=np.array([poses[link] for link in links])[indices]
    return Boxes([arm+'/'+n for n in names],np.einsum('nij,nj->ni',t[:,:3,:3],c)+t[:,:3,3],t[:,:3,:3],h,indices)

def arm_boxes(q,arm,fingers=None):
    q=np.asarray(q,float)
    if q.shape!=(7,) or not np.isfinite(q).all():raise ValueError('Invalid robot observation')
    fingers=np.full(2,np.clip(q[6],0,1)*FINGER_OPEN) if fingers is None else np.asarray(fingers,float)
    if fingers.shape!=(2,) or not np.isfinite(fingers).all():raise ValueError('Invalid measured finger positions')
    return cached_arm(tuple(np.r_[q[:6],fingers]),arm)

def arm_envelope(q,arm,ranges):
    """Enclose every independent finger position in its given interval.

    The URDF fingers are prismatic: at fixed arm angles their boxes translate
    without rotating. The union's local-axis box therefore encloses the full
    stroke, including one finger stopping before the other.
    """
    lo=arm_boxes(q,arm,ranges[:,0]);hi=arm_boxes(q,arm,ranges[:,1])
    if not np.allclose(lo.r,hi.r,atol=1e-12):raise ValueError('Finger envelope requires prismatic joints')
    delta=np.einsum('nji,nj->ni',lo.r,hi.c-lo.c)
    out=Boxes(lo.names,(lo.c+hi.c)/2,lo.r,lo.h+abs(delta)/2,local_arrays()[4])
    return out

def axis_gap(a,b):
    _,ca,ra,ha=a;_,cb,rb,hb=b
    axes=np.vstack((ra.T,rb.T,np.cross(ra.T[:,None,:],rb.T[None,:,:]).reshape(-1,3)))
    lens=np.linalg.norm(axes,axis=1);axes=axes[lens>1e-9]/lens[lens>1e-9,None]
    return float((np.abs(axes@(cb-ca))-np.abs(axes@ra)@ha-np.abs(axes@rb)@hb).max())

def pack(items):
    if isinstance(items,Boxes):return items
    return Boxes([i[0] for i in items],[i[1] for i in items],[i[2] for i in items],[i[3] for i in items],np.arange(len(items)))

def nearest(a,b,margin):
    a=pack(a);b=pack(b)
    group_gap=np.maximum(b.lo[None,:]-a.hi[:,None],a.lo[:,None]-b.hi[None,:]).max(axis=2)
    for ga,gb in np.argwhere(group_gap<=margin):
        ai=a.groups[ga];bi=b.groups[gb]
        lower=(abs(a.c[ai,None]-b.c[None,bi])-a.e[ai,None]-b.e[None,bi]).max(axis=2)
        for ia,ib in np.argwhere(lower<=margin):
            i=ai[ia];j=bi[ib];gap=axis_gap(a[i],b[j])
            if gap<=margin:return gap,(a.names[i],b.names[j])
    return margin+1e-9,None

class Guard:
    def __init__(self,margin=.008,fixtures=(),cabinet_margin=.002):self.margin=margin;self.cabinet_margin=cabinet_margin;self.fixtures=list(fixtures);self._fixture_key=None;self._packed=None
    def check(self,q,fingers=None,finger_ranges=None):
        if fingers is not None:
            fingers=np.asarray(fingers,float)
            if fingers.shape!=(2,2) or not np.isfinite(fingers).all():raise ValueError('Invalid physical finger state')
        if finger_ranges is None:
            left=arm_boxes(q[:7],'left',None if fingers is None else fingers[0])
            right=arm_boxes(q[7:],'right',None if fingers is None else fingers[1])
        else:
            left=arm_envelope(q[:7],'left',finger_ranges[0]);right=arm_envelope(q[7:],'right',finger_ranges[1])
        gap,pair=nearest(left,right,self.margin)
        if pair:return dict(kind='interarm',pair=pair,gap_m=gap,margin_m=self.margin)
        if self.fixtures:
            key=tuple((f[0],*np.r_[f[1],f[2].ravel(),f[3]]) for f in self.fixtures)
            if key!=self._fixture_key:self._packed=pack(self.fixtures);self._fixture_key=key
            for arm in [left,right]:
                gap,pair=nearest(arm,self._packed,self.cabinet_margin)
                if pair:return dict(kind='cabinet',pair=pair,gap_m=gap,margin_m=self.cabinet_margin)
        return None
    def segment(self,a,b,fingers=None):
        delta=abs(np.asarray(b)-a)
        count=max(1,int(np.ceil(max(delta[np.r_[0:6,7:13]].max()/.005,delta[[6,13]].max()/.1))))
        start=np.repeat((np.asarray(a)[[6,13]]*FINGER_OPEN)[:,None],2,axis=1) if fingers is None else np.asarray(fingers,float)
        end=np.repeat((np.asarray(b)[[6,13]]*FINGER_OPEN)[:,None],2,axis=1)
        ranges=np.stack([np.minimum(start,end),np.maximum(start,end)],axis=-1)
        for alpha in np.linspace(0,1,count+1):
            hit=self.check(np.asarray(a)+alpha*(np.asarray(b)-a),finger_ranges=ranges)
            if hit:return dict(hit,alpha=float(alpha))
        return None
