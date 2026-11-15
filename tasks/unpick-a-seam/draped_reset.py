"""Fixed initial cloth drape; intrinsic rest metrics remain flat.

This is a candidate reset, not a trajectory or equilibrium claim. The two cloth
surfaces gently rise over the lower thread runs rather than beginning suspended
and falling onto them. Native dynamics take over immediately after reset.
"""
import numpy as np
from geometry import TABLE_TOP, STITCH_Y


def initial_drape(mesh, upper):
    points=mesh.rest.copy()
    x,y=points[:,:2].T
    outside=np.maximum(np.maximum(.295-x,x-.305),0.)
    hill=np.exp(-(outside/.005)**2)*np.exp(-((y[:,None]-STITCH_Y[None])/.005)**2).sum(1)
    points[:,2]=TABLE_TOP+.0002+.00095*hill
    if upper:
        ramp=np.clip((x-.284)/.006,0.,1.)
        # Independently triangulated height fields need more than their nominal
        # thickness vertically: a full triangle-pair check gives 0.463 mm
        # minimum separation for this fixed 0.550 mm offset.
        points[:,2]+=.00055*ramp*ramp*(3-2*ramp)
    return points


def cylindrical_drape(mesh,upper):
    """Nearly isometric initial bend, without moving the needle openings.

    A 20 mm smooth ramp lies wholly outside the overlap. Its horizontal arc
    coordinate integrates sqrt(1-(dz/ds)^2), preserving the material length of
    the flat rest mesh. The finite triangle interpolation introduces only a
    small discretization strain; the ordinary audit measures it unchanged.
    """
    points=mesh.rest.copy()
    anchor,sign=(.290,-1.) if upper else (.310,1.)
    distance=np.maximum(sign*(points[:,0]-anchor),0.)
    width=.020;height=.00165 if upper else .00110;drop=height-.0002
    fraction=np.clip(distance/width,0.,1.)
    stop=np.minimum(distance,width)
    nodes,weights=np.polynomial.legendre.leggauss(32)
    sample=(nodes[None]+1.)*stop[:,None]/(2*width)
    slope=-drop/width*6*sample*(1-sample)
    if np.any(np.abs(slope)>=1.):raise ValueError('Initial bend has no real horizontal arc coordinate')
    horizontal=np.sum(weights[None]*np.sqrt(1-slope*slope),axis=1)*stop/2+np.maximum(distance-width,0.)
    outside=distance>0.
    points[outside,0]=anchor+sign*horizontal[outside]
    points[:,2]=TABLE_TOP+height-drop*fraction*fraction*(3-2*fraction)
    return points


def _point_triangle_squared(point,tri):
    a,b,c=tri[:,0],tri[:,1],tri[:,2]
    e0,e1=b-a,c-a;relative=point-a
    d00=np.einsum('ij,ij->i',e0,e0);d01=np.einsum('ij,ij->i',e0,e1)
    d11=np.einsum('ij,ij->i',e1,e1)
    d20=np.einsum('ij,ij->i',relative,e0);d21=np.einsum('ij,ij->i',relative,e1)
    determinant=d00*d11-d01*d01
    if np.any(determinant<=1e-24):raise ValueError('Degenerate triangle')
    u=(d11*d20-d01*d21)/determinant;v=(d00*d21-d01*d20)/determinant
    normal=np.cross(e0,e1)
    distance=np.einsum('ij,ij->i',relative,normal)**2/np.einsum('ij,ij->i',normal,normal)
    distance=np.where((u>=0)&(v>=0)&(u+v<=1),distance,np.inf)
    for begin,end in ((a,b),(b,c),(c,a)):
        direction=end-begin
        t=np.clip(np.einsum('ij,ij->i',point-begin,direction)/np.einsum('ij,ij->i',direction,direction),0,1)
        delta=point-begin-t[:,None]*direction
        distance=np.minimum(distance,np.einsum('ij,ij->i',delta,delta))
    return distance


def _segment_squared(a,b,c,d):
    u,v,w=b-a,d-c,a-c
    uu=np.einsum('ij,ij->i',u,u);uv=np.einsum('ij,ij->i',u,v)
    vv=np.einsum('ij,ij->i',v,v);uw=np.einsum('ij,ij->i',u,w)
    vw=np.einsum('ij,ij->i',v,w);det=uu*vv-uv*uv
    nonparallel=det>1e-14*uu*vv
    s=np.divide(uv*vw-vv*uw,det,out=np.zeros_like(det),where=nonparallel)
    t=np.divide(uu*vw-uv*uw,det,out=np.zeros_like(det),where=nonparallel)
    delta=w+s[:,None]*u-t[:,None]*v
    out=np.where(nonparallel&(s>=0)&(s<=1)&(t>=0)&(t<=1),np.einsum('ij,ij->i',delta,delta),np.inf)
    for p,q,r in ((a,c,d),(b,c,d),(c,a,b),(d,a,b)):
        edge=r-q
        z=np.clip(np.einsum('ij,ij->i',p-q,edge)/np.einsum('ij,ij->i',edge,edge),0,1)
        delta=p-q-z[:,None]*edge
        out=np.minimum(out,np.einsum('ij,ij->i',delta,delta))
    return out


def triangle_pair_distance(a,b):
    """Exact feature distance for matched nondegenerate triangle pairs."""
    from contact_audit import _hits
    a,b=np.asarray(a,float),np.asarray(b,float)
    if a.shape!=b.shape or a.ndim!=3 or a.shape[1:]!=(3,3):raise ValueError('Matched triangles required')
    out=np.full(len(a),np.inf)
    for i in range(3):
        out=np.minimum(out,_point_triangle_squared(a[:,i],b))
        out=np.minimum(out,_point_triangle_squared(b[:,i],a))
        hit=_hits(a[:,i],a[:,(i+1)%3],b)|_hits(b[:,i],b[:,(i+1)%3],a)
        out[hit]=0.
        for j in range(3):
            out=np.minimum(out,_segment_squared(a[:,i],a[:,(i+1)%3],b[:,j],b[:,(j+1)%3]))
    return np.sqrt(np.maximum(out,0.))


def surface_distance(first,first_faces,second,second_faces,search_m=.001):
    """Minimum triangle distance below a conservative AABB search radius.

    All excluded pairs are farther than search_m. If no candidate attains this
    bound, return search_m as a lower bound rather than an exact measurement.
    """
    a=np.asarray(first)[np.asarray(first_faces)];b=np.asarray(second)[np.asarray(second_faces)]
    blo,bhi=b.min(1),b.max(1);best=float(search_m);witness=None;tested=0
    for start in range(0,len(a),64):
        local=a[start:start+64];alo,ahi=local.min(1),local.max(1)
        gap=np.maximum(np.maximum(alo[:,None]-bhi[None],blo[None]-ahi[:,None]),0.)
        ai,bi=np.nonzero(np.einsum('ijk,ijk->ij',gap,gap)<=search_m**2)
        if not len(ai):continue
        distances=triangle_pair_distance(local[ai],b[bi]);tested+=len(ai)
        i=int(distances.argmin())
        if distances[i]<best:best=float(distances[i]);witness=[int(start+ai[i]),int(bi[i])]
    return {'minimum_surface_distance_m':best,'face_pair':witness,
            'exact_minimum':witness is not None,'tested_triangle_pairs':tested,
            'excluded_pairs_farther_than_m':search_m}
