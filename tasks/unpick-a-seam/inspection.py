"""Final geometric inspection and a conservative cloth-strain validity audit.

These routines inspect measured meshes and thread capsules. A disabled joint is
never evidence that a stitch was removed. The strain audit rejects excursions
outside the declared material envelope; it does not simulate or claim a tear.
"""
from __future__ import annotations

import numpy as np


def point_triangle_distance(points, vertices, faces, chunk=64):
    """Minimum Euclidean distance from each point to an entire triangle mesh."""
    p,v,f=(np.asarray(a) for a in (points,vertices,faces))
    if (p.ndim!=2 or p.shape[1]!=3 or v.ndim!=2 or v.shape[1]!=3 or
            f.ndim!=2 or f.shape[1]!=3 or not len(f) or
            not np.isfinite(p).all() or not np.isfinite(v).all()):
        raise ValueError('Finite points and a nonempty triangle mesh required')
    a,b,c=(v[f[:,i]] for i in range(3))
    e0,e1=b-a,c-a
    normal=np.cross(e0,e1);nn=np.einsum('ij,ij->i',normal,normal)
    if np.any(nn<1e-24):raise ValueError('Degenerate measured fabric triangle')
    d00=np.einsum('ij,ij->i',e0,e0)
    d01=np.einsum('ij,ij->i',e0,e1)
    d11=np.einsum('ij,ij->i',e1,e1)
    denom=d00*d11-d01*d01
    result=[]
    for start in range(0,len(p),chunk):
        q=p[start:start+chunk,None,:]
        relative=q-a
        d20=np.einsum('pij,ij->pi',relative,e0)
        d21=np.einsum('pij,ij->pi',relative,e1)
        u=(d11*d20-d01*d21)/denom
        w=(d00*d21-d01*d20)/denom
        inside=(u>=0)&(w>=0)&(u+w<=1)
        squared=np.einsum('pij,ij->pi',relative,normal)**2/nn
        squared=np.where(inside,squared,np.inf)
        for begin,end in ((a,b),(b,c),(c,a)):
            edge=end-begin
            t=np.einsum('pij,ij->pi',q-begin,edge)/np.einsum('ij,ij->i',edge,edge)
            separation=q-begin-np.clip(t,0,1)[...,None]*edge
            squared=np.minimum(squared,np.einsum('pij,pij->pi',separation,separation))
        result.extend(np.sqrt(np.maximum(0,squared.min(axis=1))))
    return np.asarray(result)


def inspect_removed_threads(thread_endpoints, cloth_vertices, cloth_faces,
                            *, radius=.0002, clearance=.001, sample_spacing=.0002):
    """Conservatively certify complete thread removal from both cloth patches.

    Sample every physical capsule axis, including its endpoints. Distance is
    1-Lipschitz, so subtract half the sample spacing as well as capsule radius.
    A passed result bounds *every* point, including between samples. This is a
    sufficient condition for removal, not a claim that all failed checks remain
    sewn. Real-world judging checks that no red thread remains in either patch.
    """
    if not 0<sample_spacing<=clearance or radius<=0:raise ValueError('Invalid inspection resolution')
    rows=[]
    for endpoints in thread_endpoints:
        e=np.asarray(endpoints,float)
        if e.ndim!=3 or e.shape[1:]!=(2,3) or not len(e) or not np.isfinite(e).all():
            raise ValueError('Each thread needs measured capsule endpoints')
        samples=[];max_interval=0.
        for begin,end in e:
            length=float(np.linalg.norm(end-begin))
            count=max(1,int(np.ceil(length/sample_spacing)))
            max_interval=max(max_interval,length/count)
            samples.extend(begin+(end-begin)*u for u in np.linspace(0,1,count+1))
        separation=min(float(point_triangle_distance(samples,v,f).min())
                       for v,f in zip(cloth_vertices,cloth_faces,strict=True))
        bound=separation-radius-max_interval/2
        rows.append({'removed':bool(bound>=clearance),
                     'minimum_surface_clearance_lower_bound_m':bound})
    return {'removed_count':sum(row['removed'] for row in rows),'threads':rows,
            'method':'capsule-axis samples with a Lipschitz clearance bound',
            'required_clearance_m':clearance}


class StrainAudit:
    """Track maximum principal stretch; latch an unvalidated-material excursion.

    The 2% default is a deliberately low engineering cutoff, not a measured
    cotton tearing strain. Published cotton tests show strong material and
    warp/weft variation, so this is a simulation-validity gate only and must not
    be reported as visible damage or as a calibrated cotton failure law.
    """
    def __init__(self, rest_vertices, faces, *, maximum_stretch=1.02):
        self.faces=np.asarray(faces,int)
        p=np.asarray(rest_vertices,float);a,b,c=(p[self.faces[:,i]] for i in range(3))
        first=b-a;length=np.linalg.norm(first,axis=1);axis=first/length[:,None]
        along=np.einsum('ij,ij->i',c-a,axis)
        across=np.linalg.norm((c-a)-along[:,None]*axis,axis=1)
        if np.any(length<1e-12) or np.any(across<1e-12):raise ValueError('Degenerate rest triangle')
        self.inverse=np.zeros((len(a),2,2))
        self.inverse[:,0,0]=1/length
        self.inverse[:,0,1]=-along/(length*across)
        self.inverse[:,1,1]=1/across
        self.limit=float(maximum_stretch)
        if not 1<self.limit<2:raise ValueError('Invalid cloth-stretch envelope')
        self.maximum_observed=1.;self.first_excursion=None;self.first_warning=None
        self.warning_limit=min(1.02,self.limit)

    def update(self,vertices,time_s):
        p=np.asarray(vertices,float)
        if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all():raise ValueError('Invalid measured cloth')
        a,b,c=(p[self.faces[:,i]] for i in range(3))
        # The rest inverse is upper triangular. Explicit deformation columns
        # avoid thousands of tiny general matrix products without changing the
        # metric or its principal-stretch threshold.
        first=b-a
        column0=first*self.inverse[:,0,0,None]
        column1=first*self.inverse[:,0,1,None]+(c-a)*self.inverse[:,1,1,None]
        g00=np.einsum('ij,ij->i',column0,column0)
        g01=np.einsum('ij,ij->i',column0,column1)
        g11=np.einsum('ij,ij->i',column1,column1)
        # Exact larger eigenvalue of the symmetric2x2 Gram matrix.
        largest=(g00+g11+np.hypot(g00-g11,2*g01))/2
        stretch=np.sqrt(np.maximum(0,largest))
        current=float(stretch.max())
        self.maximum_observed=max(self.maximum_observed,current)
        if current>self.warning_limit and self.first_warning is None:
            self.first_warning={'time_s':float(time_s),'triangle':int(stretch.argmax()),
                                'principal_stretch':current,'limit':self.warning_limit}
        if current>self.limit and self.first_excursion is None:
            self.first_excursion={'time_s':float(time_s),'triangle':int(stretch.argmax()),
                                  'principal_stretch':current,'limit':self.limit}
        return self.first_excursion is not None
