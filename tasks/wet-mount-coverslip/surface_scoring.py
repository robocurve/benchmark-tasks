"""Outcome-only scoring for local surface-process implementations.

Numeric thresholds are defined in SCORING.md and PHYSICAL_GRADING.md. No arm role, trajectory, angle-of-approach or camera-use credit.
"""
import numpy as np
from scipy.spatial.transform import Rotation

CURRENT=None

class ScoreState:
    def __init__(self,kind,paths,filters,sensors,model):
        self.kind=kind;self.paths=paths;self.filters=filters;self.sensors=sensors;self.model=model
        self.seconds=0.;self.hold=0.;self.invalid=False;self.value=0.;self.metrics={}
    def step(self,poses,velocities,robot_counts,object_counts,dt):
        if not (np.isfinite(dt) and 0<dt<=.1 and all(np.isfinite(a).all() for a in (poses,velocities,robot_counts,object_counts)) and np.all(robot_counts>=0) and np.all(object_counts>=0)):
            self.invalid=True;self.hold=0.;self.value=0.;return
        self.seconds+=dt
        for i,s in enumerate(self.sensors):
            other='RightArm' if '/LeftArm/' in s else 'LeftArm'
            if any(robot_counts[i,j]>0 for j,f in enumerate(self.filters) if '/'+other+'/' in f):self.invalid=True
        def free(i):return not np.any(robot_counts[:,self.filters.index(self.paths[i])])
        def support(i,name):return object_counts[i,self.filters.index('/World/envs/env_0/'+name)]>0
        def still(i):return np.linalg.norm(velocities[i,:3])<=.002 and np.linalg.norm(velocities[i,3:])<=.1
        if self.kind=='wet_mount':
            r=Rotation.from_quat(poses[0,3:]);corners=r.apply([[x,y,0] for x in [-.011,.011] for y in [-.011,.011]])+poses[0,:3]
            inside=bool(np.all(np.abs(corners[:,:2]-[.33,.025])<=[.0375,.0125]))
            level=abs(r.as_matrix()[2,2])>=np.cos(np.deg2rad(3))
            ready=inside and level and free(0) and still(0) and support(0,'wet_slide')
        elif self.kind=='spread_plate':
            b,l,t=0,1,2
            closed=np.linalg.norm(poses[l,:2]-poses[b,:2])<=.002 and abs(poses[l,2]-poses[b,2]-.0155)<.001
            # A round lid has no keyed yaw; only its face-normal alignment matters.
            level=Rotation.from_quat(poses[l,3:]).as_matrix()[:,2]@Rotation.from_quat(poses[b,3:]).as_matrix()[:,2]>np.cos(np.deg2rad(5))
            returned=any(support(t,f) for f in ['tool_rest0','tool_rest1','tool_head_rest'])
            upright=Rotation.from_quat(poses[b,3:]).as_matrix()[2,2]>=np.cos(np.deg2rad(5))
            on_table=object_counts[b,self.filters.index('/World/envs/env_0/table/geometry/mesh')]>0
            ready=closed and level and upright and on_table and support(l,'petri_base') and returned and all(free(i) and still(i) for i in [b,l,t])
        elif self.kind=='stencil_paste':
            returned=any(support(0,f) for f in ['stencil_support0','stencil_support1'])
            clear=np.linalg.norm(poses[0,:2]-[.33,-.065])>.08 and poses[1,2]>.82
            ready=returned and clear and free(0) and still(0)
        else:ready=False
        self.hold=self.hold+dt if ready else 0.
    def update(self):
        m=self.model.metrics();self.metrics=dict(**m,released_supported_hold_s=self.hold,invalid=self.invalid)
        value=0.
        if self.hold>=1.-1e-9 and not self.invalid:
            if self.kind=='wet_mount':
                f=m['wet_specimen_fraction'];value=1. if f>=.98 and m['largest_trapped_air_mm2']<=.7854 else .5 if f>=.5 else 0.
            elif self.kind=='spread_plate' and m['damage_area_mm2']<1.:
                f=m['wet_fraction'];value=1. if f>=.85 and min(m['quadrant_wet_fraction'])>=.75 else .5 if f>=.5 else .25 if f>=.25 else 0.
            elif self.kind=='stencil_paste' and m['released'] and m['bridges']==0:
                f=m['qualified_pad_fraction'];mass=m['deposited_mass_mg'];value=1. if f>=.9 and m['all_pads_nonempty'] and m['on_pad_fraction']>=.8 and mass>=50. else .5 if f>=.5 and mass>=25. else .25 if f>=.25 and mass>=12.5 else 0.
        self.value=float(value);return self.value

def score():return float(CURRENT.value) if CURRENT is not None else 0.
