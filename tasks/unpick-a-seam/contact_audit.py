"""Independent thread/fabric tunnelling guard; never a release controller.

A thread centreline may pass through an existing needle hole, but not an intact
cloth triangle. Check both current capsules and swept endpoints in each moving
triangle's frame. This detects clear native-contact failures; it does not claim
to resolve grazing fibres, finite-radius contact accuracy or fracture.
"""
from collections import Counter
import numpy as np


def match_capsule_order(initial_poses,threads,tolerance_m=1e-6):
    """Identify native tensor order once at reset, without steering the policy."""
    expected=np.concatenate([(t['nodes']+np.roll(t['nodes'],-1,axis=0))/2 for t in threads])
    native=np.asarray(initial_poses,float)[:,:3]
    if native.shape!=expected.shape:raise ValueError('Missing native thread bodies')
    distance=np.linalg.norm(expected[:,None,:]-native[None,:,:],axis=2)
    order=distance.argmin(axis=1)
    if len(np.unique(order))!=len(order) or np.max(distance[np.arange(len(order)),order])>tolerance_m:
        raise ValueError('Cannot identify initial native thread order unambiguously')
    return order


def capsule_endpoints(poses,lengths):
    p=np.asarray(poses,float);lengths=np.asarray(lengths,float)
    if p.shape!=(len(lengths),7) or not np.isfinite(p).all():
        raise ValueError('Invalid native thread capsule transforms')
    q=p[:,3:];norm=np.linalg.norm(q,axis=1)
    if np.any(norm<1e-9):raise ValueError('Invalid native thread rotation')
    x,y,z,w=(q/norm[:,None]).T
    axis=np.column_stack((1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y)))
    half=axis*lengths[:,None]/2
    return np.stack((p[:,:3]-half,p[:,:3]+half),axis=1)


def _intersection_coordinates(start,end,triangles):
    """Matched intersections, retaining the original audit tolerances."""
    e1=triangles[:,1]-triangles[:,0];e2=triangles[:,2]-triangles[:,0]
    d=end-start;s=start-triangles[:,0]
    h=np.cross(d,e2);det=np.einsum('ij,ij->i',e1,h)
    good=np.abs(det)>1e-20
    inv=np.divide(1.,det,out=np.zeros_like(det),where=good)
    u=np.einsum('ij,ij->i',s,h)*inv
    q=np.cross(s,e1);v=np.einsum('ij,ij->i',d,q)*inv
    t=np.einsum('ij,ij->i',e2,q)*inv
    eps=1e-7
    hits=good&(u>=-eps)&(v>=-eps)&(u+v<=1+eps)&(t>=-eps)&(t<=1+eps)
    return hits,np.column_stack((1-u-v,u,v)),t


def _hits(start,end,triangles):
    """Matched segment/triangle intersection, after an AABB broad phase."""
    return _intersection_coordinates(start,end,triangles)[0]


class ThreadClothAudit:
    def __init__(self,faces,*,boundary_tolerance_m=0.):
        if not 0.<=boundary_tolerance_m<=.0001:
            raise ValueError('Boundary contact tolerance must be between zero and0.1mm')
        self.boundary_tolerance_m=float(boundary_tolerance_m)
        self.tolerated_boundary_hits=0
        self.maximum_tolerated_depth_m=0.
        self.faces=np.asarray(faces,int);self.previous=None;self.previous_triangles=None
        self.previous_bounds=None
        self.previous_points=None
        self.first_failure=None;self.steps=0
        counts=Counter(tuple(sorted((int(a),int(b)))) for face in self.faces
                       for a,b in zip(face,np.roll(face,-1)))
        self.boundary_edges=np.asarray([edge for edge,count in counts.items() if count==1],int).reshape(-1,2)

    def _qualified_hits(self,start,end,triangles,points):
        hits,_,parameters=_intersection_coordinates(start,end,triangles)
        if not self.boundary_tolerance_m or not hits.any():return hits
        ends=points[self.boundary_edges];d=ends[:,1]-ends[:,0]
        dd=np.einsum('ij,ij->i',d,d)
        for k in np.flatnonzero(hits):
            point=start[k]+parameters[k]*(end[k]-start[k])
            fraction=np.clip(np.divide(np.einsum('ij,ij->i',point-ends[:,0],d),dd,
                out=np.zeros_like(dd),where=dd>0.),0.,1.)
            depth=float(np.linalg.norm(ends[:,0]+fraction[:,None]*d-point,axis=1).min())
            if depth<=self.boundary_tolerance_m:
                hits[k]=False;self.tolerated_boundary_hits+=1
                self.maximum_tolerated_depth_m=max(self.maximum_tolerated_depth_m,depth)
        return hits

    def _details(self,start,end,triangles,segment_ids,face_ids,points,space,limit=8):
        """Distances explain a failure; they never exempt or change one.

        Genuine mesh-boundary edges have one adjacent triangle. An internal
        triangulation edge is not a hole. Distance is Euclidean to the nearest
        deformed boundary segment, not a fracture or finite-radius penetration
        estimate. Barycentric coordinates expose tolerance-only grazing hits.
        """
        hits,barycentric,parameters=_intersection_coordinates(start,end,triangles)
        result=[]
        for k in np.flatnonzero(hits)[:limit]:
            hit=start[k]+parameters[k]*(end[k]-start[k])
            row={'segment_index':int(segment_ids[k]),'face_index':int(face_ids[k]),
                 'coordinate_frame':space,'intersection_m':hit.tolist(),
                 'barycentric':barycentric[k].tolist(),'segment_parameter':float(parameters[k]),
                 'strictly_inside_triangle':bool(np.all(barycentric[k]>=0.)),
                 'strictly_inside_segment':bool(0.<=parameters[k]<=1.)}
            if len(self.boundary_edges):
                ends=points[self.boundary_edges];d=ends[:,1]-ends[:,0]
                dd=np.einsum('ij,ij->i',d,d)
                fraction=np.clip(np.divide(np.einsum('ij,ij->i',hit-ends[:,0],d),dd,
                                           out=np.zeros_like(dd),where=dd>0.),0.,1.)
                nearest=ends[:,0]+fraction[:,None]*d
                distance=np.linalg.norm(nearest-hit,axis=1);edge=int(np.argmin(distance))
                row.update(distance_to_mesh_boundary_m=float(distance[edge]),
                           closest_boundary_edge=self.boundary_edges[edge].tolist(),
                           closest_boundary_point_m=nearest[edge].tolist(),
                           closest_boundary_edge_length_m=float(np.linalg.norm(d[edge])))
                direction=end[k]-start[k];squared=float(np.dot(direction,direction))
                along=np.clip((points-start[k])@direction/squared,0.,1.) if squared>0 else np.zeros(len(points))
                distances=np.linalg.norm(points-start[k]-along[:,None]*direction,axis=1)
                row.update(closest_boundary_vertices_distance_to_thread_axis_m=distances[self.boundary_edges[edge]].tolist(),
                           nearest_mesh_vertex_distance_to_thread_axis_m=float(distances.min()))
            result.append(row)
        return result

    def update(self,points,segments,time_s):
        self.steps+=1
        points=np.asarray(points,float)
        segments=np.asarray(segments,float);triangles=points[self.faces]
        if (segments.ndim!=3 or segments.shape[1:]!=(2,3) or
                not np.isfinite(segments).all() or not np.isfinite(triangles).all()):
            raise ValueError('Invalid measured thread/fabric geometry')
        # Explicit minima over two endpoints / three triangle corners avoid
        # the expensive tiny-axis reductions at every native substep. These
        # bounds are numerically identical to min/max(axis=1).
        low,high=np.minimum(segments[:,0],segments[:,1]),np.maximum(segments[:,0],segments[:,1])
        current_low=np.minimum(np.minimum(triangles[:,0],triangles[:,1]),triangles[:,2])
        current_high=np.maximum(np.maximum(triangles[:,0],triangles[:,1]),triangles[:,2])
        tl,th=current_low,current_high
        if self.previous is not None:
            low=np.minimum(low,np.minimum(self.previous[:,0],self.previous[:,1]))
            high=np.maximum(high,np.maximum(self.previous[:,0],self.previous[:,1]))
            tl=np.minimum(tl,self.previous_bounds[0]);th=np.maximum(th,self.previous_bounds[1])
        # Two-level exact AABB broad phase. Consecutive blocks cover one
        # authored 28-link tack; broad bounds never discard a possible hit.
        # This avoids allocating all 168 x 2492 x 3 candidate comparisons on
        # every physics step when each short thread touches a small mesh area.
        segment_ids=[];face_ids=[]
        for start in range(0,len(low),28):
            stop=min(start+28,len(low));lo=low[start:stop].min(0);hi=high[start:stop].max(0)
            candidates=np.flatnonzero((hi[0]>=tl[:,0]-1e-10)&(lo[0]<=th[:,0]+1e-10)&
                                     (hi[1]>=tl[:,1]-1e-10)&(lo[1]<=th[:,1]+1e-10)&
                                     (hi[2]>=tl[:,2]-1e-10)&(lo[2]<=th[:,2]+1e-10))
            if not len(candidates):continue
            hit=np.all((high[start:stop,None]>=tl[candidates][None]-1e-10)&
                       (low[start:stop,None]<=th[candidates][None]+1e-10),axis=2)
            si,fi=np.nonzero(hit);segment_ids.extend(si+start);face_ids.extend(candidates[fi])
        segment=np.asarray(segment_ids,int);face=np.asarray(face_ids,int)
        current=triangles[face]
        static=self._qualified_hits(segments[segment,0],segments[segment,1],current,points)
        swept=np.zeros(len(segment),dtype=bool)
        swept_details=[]
        if self.previous is not None:
            old=self.previous_triangles[face]
            displacement=current[:,0]-old[:,0]
            for endpoint in (0,1):
                # Check translation in both old and new triangle shape. The
                # elastic-domain audit separately bounds large deformation.
                a=self.previous[segment,endpoint];b=segments[segment,endpoint]
                old_hits=self._qualified_hits(a,b-displacement,old,self.previous_points)
                new_hits=self._qualified_hits(a+displacement,b,current,points)
                swept|=old_hits|new_hits
                if self.first_failure is None:
                    if old_hits.any():
                        swept_details.extend(self._details(a,b-displacement,old,segment,face,
                            self.previous_points,f'previous_triangle_endpoint_{endpoint}'))
                    if new_hits.any():
                        swept_details.extend(self._details(a+displacement,b,current,segment,face,
                            points,f'current_triangle_endpoint_{endpoint}'))
        if (static|swept).any() and self.first_failure is None:
            self.first_failure={'step':self.steps,'time_s':float(time_s),
                                'reason':'thread crosses intact fabric',
                                'static_intersections':int(static.sum()),
                                'swept_intersections':int(swept.sum()),
                                'static_crossing_details':self._details(segments[segment,0],segments[segment,1],
                                    current,segment,face,points,'current') if static.any() else [],
                                'swept_crossing_details':swept_details[:8]}
        self.previous=segments.copy();self.previous_triangles=triangles;self.previous_points=points.copy()
        self.previous_bounds=(current_low,current_high)
        return self.first_failure is None

    def summary(self):
        return {'valid':self.first_failure is None,'steps':self.steps,'first_failure':self.first_failure,
                'boundary_tolerance_m':self.boundary_tolerance_m,
                'tolerated_boundary_hits':self.tolerated_boundary_hits,
                'maximum_tolerated_depth_m':self.maximum_tolerated_depth_m}
