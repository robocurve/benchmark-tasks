"""Conforming local refinement of the cloth band contacting the thin line.

Only rest-mesh resolution changes. Shape, surface area, material, mass, seams,
openings and every policy's physical reset stay the same. Edge midpoints are
shared globally, including transition triangles; there are no hanging nodes.
"""
import copy
import numpy as np
from cloth_mesh import Mesh,tshirt


def contact_band_mesh(max_edge=.0025):
    base=tshirt();points=base.rest.tolist();faces=base.faces.tolist()
    for _ in range(6):
        p=np.asarray(points);f=np.asarray(faces);tri=p[f]
        selected=(tri[:,:,0].max(1)>=-.14)&(tri[:,:,0].min(1)<=.14)&(tri[:,:,1].max(1)>=.03)&(tri[:,:,1].min(1)<=.17)
        for centre in (-.12,.12):
            selected|=(tri[:,:,0].max(1)>=centre-.015)&(tri[:,:,0].min(1)<=centre+.015)&(tri[:,:,1].min(1)<=.025)
        split=set()
        for row in f[selected]:
            for a,b in zip(row,np.roll(row,-1)):
                if np.linalg.norm(p[a]-p[b])>max_edge*1.000001:split.add(tuple(sorted((int(a),int(b)))))
        if not split:break
        mid={}
        for a,b in sorted(split):mid[a,b]=len(points);points.append(((p[a]+p[b])/2).tolist())
        updated=[]
        for row in faces:
            a,b,c=row;m=[mid.get(tuple(sorted(e))) for e in ((a,b),(b,c),(c,a))];n=sum(x is not None for x in m)
            if n==0:updated.append(row)
            elif n==3:
                ab,bc,ca=m;updated.extend(((a,ab,ca),(ab,b,bc),(ca,bc,c),(ab,bc,ca)))
            elif n==1:
                k=next(i for i,x in enumerate(m) if x is not None);a,b,c=[row[(k+i)%3] for i in range(3)];ab=m[k];updated.extend(((a,ab,c),(ab,b,c)))
            else:
                # Rotate so the two split edges are a-b and b-c.
                k=next(i for i in range(3) if m[i] is not None and m[(i+1)%3] is not None)
                a,b,c=[row[(k+i)%3] for i in range(3)];ab,bc=m[k],m[(k+1)%3];updated.extend(((b,bc,ab),(a,ab,c),(ab,bc,c)))
        faces=updated
    else:raise RuntimeError('Contact-band refinement did not converge')
    p=np.asarray(points);markers=copy.deepcopy(base.markers)
    markers.update(front=np.flatnonzero(p[:,2]>.0007999).tolist(),back=np.flatnonzero(p[:,2]<-.0007999).tolist(),contact_refinement=dict(region_xy_m=[[-.14,.14],[.03,.17]],hem_grasp_regions_xy_m=[[[-.135,-.105],[0.,.025]],[[.105,.135],[0.,.025]]],maximum_selected_edge_m=max_edge,source='Native064 line-axis crossings at material x−.09..+.04,y.06...12; wider fixed band covers both peg sites.'))
    mesh=Mesh(p,np.asarray(faces,int),markers);a=mesh.audit();b=base.audit()
    if len(a['boundary_loops'])!=4 or abs(a['surface_area_m2']-b['surface_area_m2'])>1e-12:raise ValueError('Refinement changed garment topology or area')
    return mesh


def pad_refined_mesh(max_edge=.00125):
    """Regular 2.5 mm cloth with conforming finer edges at both hem grips.

    Numerical contact convergence only. No change to area, layers, material,
    friction or mass. The broad regions include the measured corner contacts.
    """
    base=tshirt(spacing=.0025)
    points=base.rest.tolist();faces=base.faces.tolist()
    for _ in range(6):
        p=np.asarray(points);f=np.asarray(faces);tri=p[f]
        selected=(tri[:,:,1].min(1)<=.035)&(tri[:,:,0].max(1)>=-.15)&(tri[:,:,0].min(1)<=.15)
        selected&=(tri[:,:,0].min(1)<=-.095)|(tri[:,:,0].max(1)>=.095)
        split={tuple(sorted((int(a),int(b)))) for row in f[selected]
               for a,b in zip(row,np.roll(row,-1)) if np.linalg.norm(p[a]-p[b])>max_edge*1.000001}
        if not split:break
        mid={}
        for a,b in sorted(split):
            mid[a,b]=len(points);points.append(((p[a]+p[b])/2).tolist())
        updated=[]
        for row in faces:
            a,b,c=row;m=[mid.get(tuple(sorted(e))) for e in ((a,b),(b,c),(c,a))];n=sum(x is not None for x in m)
            if n==0:updated.append(row)
            elif n==3:
                ab,bc,ca=m;updated.extend(((a,ab,ca),(ab,b,bc),(ca,bc,c),(ab,bc,ca)))
            elif n==1:
                k=next(i for i,x in enumerate(m) if x is not None);a,b,c=[row[(k+i)%3] for i in range(3)];ab=m[k];updated.extend(((a,ab,c),(ab,b,c)))
            else:
                k=next(i for i in range(3) if m[i] is not None and m[(i+1)%3] is not None)
                a,b,c=[row[(k+i)%3] for i in range(3)];ab,bc=m[k],m[(k+1)%3];updated.extend(((b,bc,ab),(a,ab,c),(ab,bc,c)))
        faces=updated
    else:raise RuntimeError('Pad refinement did not converge')
    p=np.asarray(points);markers=copy.deepcopy(base.markers)
    markers.update(front=np.flatnonzero(p[:,2]>.0007999).tolist(),back=np.flatnonzero(p[:,2]<-.0007999).tolist(),
        pad_refinement=dict(maximum_selected_edge_m=max_edge,region_abs_x_m=[.095,.15],region_y_m=[0.,.035]))
    mesh=Mesh(p,np.asarray(faces,int),markers);a=mesh.audit();b=base.audit()
    if len(a['boundary_loops'])!=4 or abs(a['surface_area_m2']-b['surface_area_m2'])>1e-12:
        raise ValueError('Refinement changed garment topology or area')
    return mesh
