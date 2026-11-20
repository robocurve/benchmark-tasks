"""Uniform geodesic hemisphere, avoiding thin triangles at a latitude pole."""
import numpy as np
from scipy.spatial import ConvexHull


def hemisphere(subdivisions=3):
    phi=(1+np.sqrt(5.))/2
    vertices=np.array([(0,a,b*phi) for a in (-1,1) for b in (-1,1)]+
                      [(a,b*phi,0) for a in (-1,1) for b in (-1,1)]+
                      [(b*phi,0,a) for a in (-1,1) for b in (-1,1)],float)
    vertices/=np.linalg.norm(vertices,axis=1)[:,None]
    faces=ConvexHull(vertices).simplices.tolist()
    for _ in range(subdivisions):
        points=vertices.tolist();cache={};refined=[]
        def mid(a,b):
            key=tuple(sorted((a,b)))
            if key not in cache:
                p=vertices[a]+vertices[b];p/=np.linalg.norm(p)
                cache[key]=len(points);points.append(p.tolist())
            return cache[key]
        for a,b,c in faces:
            ab,bc,ca=mid(a,b),mid(b,c),mid(c,a)
            refined.extend([[a,ab,ca],[b,bc,ab],[c,ca,bc],[ab,bc,ca]])
        vertices=np.array(points);faces=refined
    points=vertices.tolist();cuts={};upper=[]
    def crossing(a,b):
        if abs(vertices[a,2])<1e-10:return a
        if abs(vertices[b,2])<1e-10:return b
        key=tuple(sorted((a,b)))
        if key not in cuts:
            t=vertices[a,2]/(vertices[a,2]-vertices[b,2]);p=(1-t)*vertices[a]+t*vertices[b]
            p[2]=0.;p/=np.linalg.norm(p);cuts[key]=len(points);points.append(p.tolist())
        return cuts[key]
    for face in faces:
        polygon=[]
        for a,b in zip(face,face[1:]+face[:1]):
            inside=vertices[a,2]>=-1e-10;next_inside=vertices[b,2]>=-1e-10
            if inside:polygon.append(a)
            if inside!=next_inside:polygon.append(crossing(a,b))
        polygon=list(dict.fromkeys(polygon))
        if len(polygon)>=3:
            upper.extend([[polygon[0],polygon[i],polygon[i+1]] for i in range(1,len(polygon)-1)])
    vertices=np.asarray(points);faces=np.asarray(upper)
    ids=np.unique(faces);mapping=np.full(len(vertices),-1);mapping[ids]=np.arange(len(ids))
    points=vertices[ids];faces=mapping[faces]
    tri=points[faces];wrong=np.einsum('ij,ij->i',np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),tri.mean(axis=1))<0
    faces[wrong]=faces[wrong][:,[0,2,1]]
    rim=np.flatnonzero(abs(points[:,2])<1e-10)
    rim=rim[np.argsort(np.mod(np.arctan2(points[rim,1],points[rim,0]),2*np.pi))]
    assert len(rim)%4==0
    edges=np.sort(np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]]),axis=1)
    unique,count=np.unique(edges,axis=0,return_counts=True)
    assert set(unique[count==1].ravel())==set(rim)
    assert np.allclose(points[rim[[0,len(rim)//4,len(rim)//2,3*len(rim)//4]]],[[1,0,0],[0,1,0],[-1,0,0],[0,-1,0]])
    return points,faces,rim
