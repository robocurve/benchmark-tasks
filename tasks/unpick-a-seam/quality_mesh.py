"""CPU-only candidate mesh with the same square holes and material surface.

This candidate is not selected by the native runner. It replaces recursive
rectilinear refinement with locally graded sampling and Delaunay triangles.
Explicit area/boundary checks fail closed if triangulation changes any opening.
No stiffness, colliders, attachment or physics parameters are changed here.
"""
from collections import Counter
import numpy as np
from geometry import Mesh, TABLE_TOP, STITCH_Y


def _clipped_area(triangle,low,high):
    polygon=[p for p in triangle]
    for axis,bound,greater in ((0,low[0],True),(0,high[0],False),
                               (1,low[1],True),(1,high[1],False)):
        if not polygon:return 0.
        output=[]
        for begin,end in zip(polygon,polygon[1:]+polygon[:1]):
            inside0=(begin[axis]>=bound) if greater else (begin[axis]<=bound)
            inside1=(end[axis]>=bound) if greater else (end[axis]<=bound)
            if inside0!=inside1:
                t=(bound-begin[axis])/(end[axis]-begin[axis])
                output.append(begin+t*(end-begin))
            if inside1:output.append(end)
        polygon=output
    if len(polygon)<3:return 0.
    p=np.asarray(polygon);q=np.roll(p,-1,axis=0)
    return float(abs(np.sum(p[:,0]*q[:,1]-p[:,1]*q[:,0]))/2)


def quality_patch(left=True):
    """Candidate for the tested 0.8 mm thread / 1.6 mm square-hole regime."""
    from scipy.spatial import Delaunay,cKDTree
    xmin,xmax=(.235,.310) if left else (.290,.365)
    ymin,ymax=-.040,.040
    hole_half=.0008
    centres=np.array([(x,y) for x in (.295,.305) for y in STITCH_Y])
    def square(centre,half,spacing):
        count=round(2*half/spacing)
        out=[]
        corners=np.array(((-half,-half),(half,-half),(half,half),(-half,half)))
        for a,b in zip(corners,np.roll(corners,-1,axis=0)):
            out.extend(centre+a+(b-a)*t/count for t in range(count))
        return out
    prescribed=[]
    for centre in centres:
        for half,spacing in ((.0008,.0004),(.0012,.0004),(.0020,.0008),(.0032,.0016)):
            prescribed.extend(square(centre,half,spacing))
    # Outer boundary is identical. Its spacing is unrelated to thin-thread
    # contact, so it retains roughly 3 mm elements away from needle openings.
    for start,end in (((xmin,ymin),(xmax,ymin)),((xmax,ymin),(xmax,ymax)),
                      ((xmax,ymax),(xmin,ymax)),((xmin,ymax),(xmin,ymin))):
        a,b=np.asarray(start),np.asarray(end)
        count=int(np.ceil(np.linalg.norm(b-a)/.003))
        prescribed.extend(a+(b-a)*t/count for t in range(count))
    prescribed=np.asarray(prescribed)
    coarse=np.array([(x,y) for x in np.linspace(xmin,xmax,26)[1:-1]
                           for y in np.linspace(ymin,ymax,28)[1:-1]])
    beyond_ring=(np.abs(coarse[:,None]-centres[None]).max(2)>.0032+1e-12).all(1)
    separated=cKDTree(prescribed).query(coarse)[0]>=.0008-1e-12
    xy=np.unique(np.round(np.vstack((prescribed,coarse[beyond_ring&separated])),12),axis=0)
    faces=Delaunay(xy).simplices
    keep=[]
    for face in faces:
        t=xy[face];lo=t.min(0);hi=t.max(0)
        candidates=np.flatnonzero(((hi>=centres-hole_half)&(lo<=centres+hole_half)).all(1))
        # Clip complete triangle against each potentially touched hole. Do not
        # approximate holes by deleting triangles solely on their centroid.
        if not any(_clipped_area(t,centres[i]-hole_half,centres[i]+hole_half)>1e-14 for i in candidates):
            keep.append(face)
    faces=np.asarray(keep,int)
    used,inverse=np.unique(faces,return_inverse=True);xy=xy[used];faces=inverse.reshape(-1,3)
    first=xy[faces[:,1]]-xy[faces[:,0]];second=xy[faces[:,2]]-xy[faces[:,0]]
    area2=first[:,0]*second[:,1]-first[:,1]*second[:,0]
    faces[area2<0]=faces[area2<0][:,[0,2,1]]
    points=np.column_stack((xy,np.full(len(xy),TABLE_TOP+(.004 if left else .0032))))
    mesh=Mesh(points,faces);audit=mesh.audit()
    expected=(xmax-xmin)*(ymax-ymin)-len(centres)*(2*hole_half)**2
    if audit['boundary_loops']!=13 or abs(audit['surface_area_m2']-expected)>1e-13:
        raise ValueError('Candidate mesh changed holes or physical area')
    edges=Counter(tuple(sorted((int(a),int(b)))) for f in faces for a,b in zip(f,np.roll(f,-1)))
    for edge,count in edges.items():
        if count!=1:continue
        p=xy[list(edge)]
        outer=any(np.all(np.abs(p[:,axis]-value)<1e-10) for axis,value in
                  ((0,xmin),(0,xmax),(1,ymin),(1,ymax)))
        if outer:continue
        on_hole=False
        for centre in centres:
            if np.all((p>=centre-hole_half-1e-10)&(p<=centre+hole_half+1e-10)):
                on_hole=any(np.all(np.abs(p[:,axis]-(centre[axis]+sign*hole_half))<1e-10)
                            for axis in (0,1) for sign in (-1,1))
                if on_hole:break
        if not on_hole or np.linalg.norm(p[1]-p[0])>.0004+1e-10:
            raise ValueError('Changed or underresolved needle-hole boundary')
    return mesh


def quality(mesh):
    t=mesh.rest[mesh.faces]
    edges=np.linalg.norm(np.roll(t,-1,axis=1)-t,axis=2)
    angles=[]
    for i in range(3):
        a=t[:,(i+1)%3]-t[:,i];b=t[:,(i+2)%3]-t[:,i]
        angles.append(np.arccos(np.clip(np.einsum('ij,ij->i',a,b)/(np.linalg.norm(a,axis=1)*np.linalg.norm(b,axis=1)),-1,1)))
    return {**mesh.audit(),'minimum_edge_m':float(edges.min()),'maximum_edge_m':float(edges.max()),
            'minimum_angle_degrees':float(np.rad2deg(np.min(angles))),
            'maximum_triangle_edge_ratio':float(np.max(edges.max(1)/edges.min(1)))}


if __name__=='__main__':
    import json
    from geometry import patch
    print(json.dumps({'candidate_native_run':False,
        'current':[quality(patch(left,hole_half=.0008,hole_mesh_spacing=.0004)) for left in (True,False)],
        'candidate':[quality(quality_patch(left)) for left in (True,False)]},indent=2))
