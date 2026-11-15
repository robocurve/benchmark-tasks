"""Optional post-failure contact measurements; never a validity decision.

The ordinary guard remains latched. A separate audit measures each native step
so one near-boundary contact can be distinguished from persistent intersection.
These measurements cannot certify extraction or identify microscopic fibres.
"""
import numpy as np
from contact_audit import ThreadClothAudit


class ContactHistory:
    def __init__(self,faces,radius_m,dt):
        if radius_m<=0 or dt<=0:raise ValueError('Positive radius and timestep required')
        self.audit=ThreadClothAudit(faces)
        self.radius=float(radius_m);self.dt=float(dt);self.rows=[]

    def update(self,points,segments,time_s):
        # This is a separate forensic object. Never clear the task's guard.
        self.audit.first_failure=None
        self.audit.update(points,segments,time_s)
        failure=self.audit.first_failure
        row={'step':self.audit.steps,'time_s':float(time_s),
             'static_intersections':0,'swept_intersections':0,'static_details':[]}
        if failure:
            row.update(static_intersections=failure['static_intersections'],
                       swept_intersections=failure['swept_intersections'])
            for detail in failure['static_crossing_details']:
                detail=dict(detail)
                tri=np.asarray(points)[self.audit.faces[detail['face_index']]]
                normal=np.cross(tri[1]-tri[0],tri[2]-tri[0])
                normal/=np.linalg.norm(normal)
                ends=np.asarray(segments)[detail['segment_index']]
                detail['axis_endpoints_signed_distance_to_triangle_plane_m']=((ends-tri[0])@normal).tolist()
                distances=detail.get('closest_boundary_vertices_distance_to_thread_axis_m',[])
                detail['boundary_vertex_capsule_overlap_m']=max([0.]+[self.radius-d for d in distances])
                row['static_details'].append(detail)
        self.rows.append(row)
        return row

    def summary(self):
        def intervals(key):
            runs=[];start=None
            for i,row in enumerate(self.rows):
                active=row[key]>0
                if active and start is None:start=i
                if start is not None and (not active or i==len(self.rows)-1):
                    end=i if active else i-1
                    runs.append({'first_step':self.rows[start]['step'],
                                 'last_step':self.rows[end]['step'],
                                 'sample_count':end-start+1,
                                 'sampled_duration_s':(end-start+1)*self.dt,
                                 'recovered_before_end':not active})
                    start=None
            return runs
        details=[d for row in self.rows for d in row['static_details']]
        return {'steps':len(self.rows),'dt_s':self.dt,
                'static_intersection_intervals':intervals('static_intersections'),
                'swept_intersection_intervals':intervals('swept_intersections'),
                'maximum_sampled_boundary_distance_m':max([0.]+[d['distance_to_mesh_boundary_m'] for d in details]),
                'maximum_sampled_boundary_vertex_capsule_overlap_m':max([0.]+[d['boundary_vertex_capsule_overlap_m'] for d in details]),
                'note':'At most eight static details per step; maxima are sampled lower bounds. Intervals describe sampled occupancy, not continuous-time residence or proof of permanent unthreading.'}
