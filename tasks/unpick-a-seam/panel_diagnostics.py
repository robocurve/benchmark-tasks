"""Read-only diagnosis of native box-panel contact and stitch retention.

A zero-thickness crossing, finite capsule overlap and loss of a linked stitch
are different measurements. This module changes no state, guard or score. It
requires corrected raw thread poses; old panel arrays mislabeled as thread
poses are rejected. Linking numbers use the continuous reconstructed hole
boundary and must be interpreted alongside measured hinge/vertex gaps.
"""
from collections import Counter,defaultdict
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from contact_audit import capsule_endpoints
from facet_cloth import mechanics,measured_surface
from geometry import Mesh


def segment_box_distances(segments,half_extents):
    """Exact Euclidean distance from each segment to its axis-aligned box.

    Squared point-to-box distance is a convex piecewise quadratic along a
    segment. Test every face breakpoint and the stationary point in each
    interval, avoiding iterative optimization or sampling tolerances.
    """
    s=np.asarray(segments,float);h=np.asarray(half_extents,float)
    if s.ndim!=3 or s.shape[1:]!=(2,3) or h.shape!=(len(s),3):
        raise ValueError('Matched segments and positive box half extents required')
    if not np.isfinite(s).all() or not np.isfinite(h).all() or np.any(h<=0):
        raise ValueError('Finite segments and positive box half extents required')
    a=s[:,0];d=s[:,1]-a
    planes=np.stack((-h,h),axis=1)
    crossings=np.divide(planes-a[:,None],d[:,None],
                        out=np.zeros_like(planes),where=np.abs(d[:,None])>1e-30)
    breaks=np.sort(np.column_stack((np.zeros(len(s)),np.ones(len(s)),
                                    np.clip(crossings.reshape(len(s),6),0.,1.))),axis=1)
    middle=(breaks[:,:-1]+breaks[:,1:])/2
    points=a[:,None]+middle[:,:,None]*d[:,None]
    active=np.abs(points)>h[:,None]
    q=a[:,None]-np.sign(points)*h[:,None]
    numerator=np.sum(np.where(active,q*d[:,None],0.),axis=-1)
    denominator=np.sum(np.where(active,d[:,None]**2,0.),axis=-1)
    stationary=np.divide(-numerator,denominator,out=middle.copy(),where=denominator>0.)
    stationary=np.clip(stationary,breaks[:,:-1],breaks[:,1:])
    t=np.column_stack((breaks,stationary))
    candidates=a[:,None]+t[:,:,None]*d[:,None]
    distances=np.linalg.norm(np.maximum(np.abs(candidates)-h[:,None],0.),axis=-1)
    best=np.argmin(distances,axis=1);rows=np.arange(len(s))
    return distances[rows,best],t[rows,best],candidates[rows,best]


def boundary_loops(faces):
    counts=Counter(tuple(sorted((int(a),int(b)))) for f in faces for a,b in zip(f,np.roll(f,-1)))
    graph=defaultdict(list)
    for (a,b),count in counts.items():
        if count==1:graph[a].append(b);graph[b].append(a)
    if any(len(v)!=2 for v in graph.values()):raise ValueError('Closed manifold boundaries required')
    remaining=set(graph);loops=[]
    while remaining:
        start=min(remaining);row=[start];previous=None;current=start
        while True:
            nxt=next(n for n in graph[current] if n!=previous)
            if nxt==start:break
            if nxt in row:raise ValueError('Boundary is not a simple loop')
            row.append(nxt);previous,current=current,nxt
        remaining.difference_update(row);loops.append(np.asarray(row,int))
    return loops


def linking_number(first,second):
    """Signed polygon linking number from two independent generic projections.

    A disagreement or noninteger result is reported, never rounded into a pass.
    This is topology of the measured curves, not a finite contact-depth test.
    """
    values=[];clearances=[]
    cross=lambda x,y:x[...,0]*y[...,1]-x[...,1]*y[...,0]
    for angles in ((.37,.61,.23),(.91,-.43,.72)):
        rotation=Rotation.from_euler('xyz',angles).as_matrix()
        a=np.asarray(first)@rotation;b=np.asarray(second)@rotation
        u=np.roll(a,-1,axis=0)-a;v=np.roll(b,-1,axis=0)-b
        w=b[None,:,:2]-a[:,None,:2]
        determinant=cross(u[:,None,:2],v[None,:,:2]);good=np.abs(determinant)>1e-24
        t=np.divide(cross(w,v[None,:,:2]),determinant,out=np.zeros_like(determinant),where=good)
        q=np.divide(cross(w,u[:,None,:2]),determinant,out=np.zeros_like(determinant),where=good)
        hit=good&(t>0)&(t<1)&(q>0)&(q<1)
        dz=b[None,:,2]+q*v[None,:,2]-a[:,None,2]-t*u[:,None,2]
        values.append(float(.5*np.sum(np.sign(determinant[hit])*np.sign(dz[hit]))))
        clearances.extend(np.abs(dz[hit]).tolist())
    agreed=values[0]==values[1] and values[0].is_integer()
    return dict(value=int(values[0]) if agreed else None,projection_values=values,
                minimum_projected_crossing_separation_m=min(clearances,default=None))


def solid_contacts(model,poses,segments,radius=.0004,thickness=.0004,*,boundary_tolerance_m=0.):
    """Static native capsule/box measurements; no shared-vertex averaging."""
    if not 0.<=boundary_tolerance_m<=.0001:raise ValueError('Boundary tolerance outside0–0.1mm')
    points=model['points'];u=points[:,1]-points[:,0];v=points[:,3]-points[:,0]
    width=np.linalg.norm(u,axis=1);height=np.linalg.norm(v,axis=1)
    basis=np.stack((u/width[:,None],v/height[:,None],model['normals']),axis=2)
    rotation=Rotation.from_quat(np.asarray(poses)[:,3:]).as_matrix()@basis
    half=np.column_stack((width/2,height/2,np.full(len(points),thickness/2)))
    # Conservative world AABBs reject distant pairs before exact local work.
    # One-picometre padding only broadens the candidate list, not predicates.
    world_half=np.einsum('pij,pj->pi',np.abs(rotation),half)
    low=segments.min(axis=1)-radius-1e-12;high=segments.max(axis=1)+radius+1e-12
    possible=np.all((poses[:,None,:3]-world_half[:,None]<=high[None])&
                    (poses[:,None,:3]+world_half[:,None]>=low[None]),axis=-1)
    panel,segment=np.nonzero(possible)
    local=np.einsum('ned,ndk->nek',segments[segment]-poses[panel,None,:3],rotation[panel])
    keep=np.all((local.min(axis=1)<=half[panel]+radius)&
                (local.max(axis=1)>=-half[panel]-radius),axis=-1)
    panel,segment=panel[keep],segment[keep]
    selected=local[keep];h=half[panel]
    distance,t,closest=segment_box_distances(selected,h)
    overlapping=distance<radius
    rows=[]
    for k in np.flatnonzero(overlapping):
        rows.append(dict(panel=int(panel[k]),segment=int(segment[k]),
                         capsule_overlap_lower_bound_m=float(radius-distance[k]),
                         axis_to_solid_distance_m=float(distance[k]),
                         closest_axis_point_box_frame_m=closest[k].tolist()))
    direction=selected[:,1]-selected[:,0]
    plane_t=np.divide(-selected[:,0,2],direction[:,2],out=np.full(len(selected),np.nan),where=np.abs(direction[:,2])>1e-20)
    plane=selected[:,0]+plane_t[:,None]*direction
    inside=(plane_t>=0)&(plane_t<=1)&np.all(np.abs(plane[:,:2])<=h[:,:2],axis=1)
    intersections=[dict(panel=int(panel[k]),segment=int(segment[k]),
                        nearest_panel_edge_m=float(np.min(h[k,:2]-np.abs(plane[k,:2]))),
                        point_box_frame_m=plane[k].tolist()) for k in np.flatnonzero(inside)]
    tolerated=[]
    if boundary_tolerance_m and intersections:
        # Only actual outer/hole boundaries count. Interior triangulation or
        # panel edges cannot exempt a crossing through intact fabric.
        if '_boundary_panel_corners' not in model:
            owners=defaultdict(list)
            for i,ids in enumerate(model['vertex_ids']):
                for j in range(len(ids)):
                    owners[tuple(sorted((int(ids[j]),int(ids[(j+1)%len(ids)]))))].append((i,j,(j+1)%len(ids)))
            model['_boundary_panel_corners']=np.asarray([v[0] for v in owners.values() if len(v)==1],int)
        bp=model['_boundary_panel_corners']
        raw_rotation=Rotation.from_quat(np.asarray(poses)[:,3:]).as_matrix()
        local_edges=model['points'][bp[:,0,None],bp[:,1:]]-model['centres'][bp[:,0],None]
        ends=np.einsum('bij,bkj->bki',raw_rotation[bp[:,0]],local_edges)+poses[bp[:,0],None,:3]
        delta=ends[:,1]-ends[:,0];length2=np.sum(delta*delta,axis=1)
        retained=[]
        for row in intersections:
            point=rotation[row['panel']]@np.asarray(row['point_box_frame_m'])+poses[row['panel'],:3]
            along=np.clip(np.divide(np.sum((point-ends[:,0])*delta,axis=1),length2,
                out=np.zeros_like(length2),where=length2>0.),0.,1.)
            depth=float(np.linalg.norm(ends[:,0]+along[:,None]*delta-point,axis=1).min())
            row['distance_to_actual_boundary_m']=depth
            (tolerated if depth<=boundary_tolerance_m else retained).append(row)
        intersections=retained
    return dict(overlapping_pairs=len(rows),maximum_capsule_overlap_lower_bound_m=max(
                    (r['capsule_overlap_lower_bound_m'] for r in rows),default=0.),
                native_midplane_intersections=intersections,
                boundary_tolerance_m=boundary_tolerance_m,tolerated_boundary_intersections=tolerated,
                deepest_pairs=sorted(rows,key=lambda r:r['axis_to_solid_distance_m'])[:8])

def inspect(directory):
    directory=Path(directory)
    # NPZ fields are decompressed on each lookup. Read raw arrays once rather
    # than repeatedly inflating an entire multi-second trace for every frame.
    with np.load(directory/'state.npz') as archive:
        a={key:archive[key] for key in archive.files}
    report=json.loads((directory/'probe.json').read_text())
    if not report.get('box_panels'):raise ValueError('Actual box-panel evidence required')
    if 'thread_lengths' not in a:raise ValueError('Corrected native thread recording with lengths required')
    lengths=a['thread_lengths'];times=a['sample_times_s']
    if a['thread'].shape!=(len(times),len(lengths),7):raise ValueError('Invalid raw native thread shape')
    if len(lengths) not in (144,168):raise ValueError('Expected six24-link or28-link stitch loops')
    links_per_loop=len(lengths)//6
    shells=[];holes=[]
    for side in range(2):
        mesh=Mesh(a[f'rest_{side}'],a[f'faces_{side}']);model=mechanics(mesh,primitive_boxes=True)
        shells.append(dict(mesh=mesh,model=model))
        # Every needle hole has four edges. The outside boundary has more.
        holes.append([loop for loop in boundary_loops(mesh.faces) if len(loop)==4])
        if len(holes[-1])!=12:raise ValueError('Twelve physical needle-hole boundaries required')
    expected=[];series=[];changes=[];first_solid=None
    for frame,time in enumerate(times):
        segments=capsule_endpoints(a['thread'][frame],lengths)
        thread_loops=[segments[i:i+links_per_loop].reshape(-1,3) for i in range(0,len(lengths),links_per_loop)]
        row=dict(time_s=float(time),patches=[])
        for side,shell in enumerate(shells):
            poses=a[f'facet_poses_{side}'][frame]
            surface,gaps=measured_surface(shell,poses)
            contact=solid_contacts(shell['model'],poses,segments)
            if first_solid is None and contact['native_midplane_intersections']:
                first_solid=dict(time_s=float(time),side=side,**contact)
            if frame==0:
                for hole,ids in enumerate(holes[side]):
                    for thread,curve in enumerate(thread_loops):
                        link=linking_number(surface[ids],curve)
                        if link['value'] is None:raise ValueError('Ambiguous initial linking projection')
                        if link['value']:
                            expected.append(dict(side=side,hole=hole,thread=thread,value=link['value']))
            linked=0;ambiguous=0
            for target in (r for r in expected if r['side']==side):
                link=linking_number(surface[holes[side][target['hole']]],thread_loops[target['thread']])
                if link['value']==target['value']:linked+=1
                else:
                    ambiguous+=link['value'] is None
                    changes.append(dict(time_s=float(time),**target,measured=link))
            row['patches'].append(dict(maximum_hinge_endpoint_gap_m=gaps['maximum_hinge_endpoint_gap_m'],
                maximum_vertex_average_error_m=gaps['maximum_vertex_average_error_m'],
                maximum_capsule_overlap_lower_bound_m=contact['maximum_capsule_overlap_lower_bound_m'],
                overlapping_pairs=contact['overlapping_pairs'],
                native_midplane_intersections=len(contact['native_midplane_intersections']),
                retained_initial_links=linked,ambiguous_links=ambiguous))
        series.append(row)
    window=[row for row in series if row['time_s']>float(times[-1])-1.]
    last_second=[]
    for side in range(2):
        expected_count=sum(target['side']==side for target in expected)
        rows=[row['patches'][side] for row in window]
        last_second.append(dict(expected_links=expected_count,
            all_initial_links_retained=all(row['retained_initial_links']==expected_count for row in rows),
            minimum_retained_initial_links=min(row['retained_initial_links'] for row in rows),
            native_midplane_intersection_frames=sum(row['native_midplane_intersections']>0 for row in rows),
            maximum_capsule_overlap_lower_bound_m=max(row['maximum_capsule_overlap_lower_bound_m'] for row in rows),
            maximum_hinge_endpoint_gap_m=max(row['maximum_hinge_endpoint_gap_m'] for row in rows)))
    return dict(status='diagnostic_only',source=str(directory),frames=len(times),
        native_audit_status=report['status'],first_invalid_step=report.get('first_invalid_step'),
        expected_initial_links=expected,expected_initial_link_count=len(expected),
        first_actual_box_midplane_intersection=first_solid,link_changes=changes,
        last_one_second=dict(start_s=window[0]['time_s'],end_s=window[-1]['time_s'],
                             frames=len(window),patches=last_second),
        note='Linking uses reconstructed continuous hole boundaries; hinge gaps remain separately reported. Finite overlap is a contact error, not proof of persistent passage. No task score is assigned.',series=series)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--input',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();result=inspect(a.input);a.out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({key:result[key] for key in ('status','frames','expected_initial_link_count','first_invalid_step')},indent=2))
