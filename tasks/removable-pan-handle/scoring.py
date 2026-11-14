"""Read-only scoring from complete native contacts and body poses.

Speeds use total observed pose motion in each control interval, including every
physics substep. Engine-reported velocities remain independent diagnostics.
"""
import math
import numpy as np
from .geometry import TABLE, TRIVET, CLOSED_ANGLE


def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(q@q-1) > .001:
        raise ValueError('Invalid native orientation')
    x,y,z,w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def angle(a,b):
    a=np.asarray(a,dtype=float); b=np.asarray(b,dtype=float)
    a=a/np.linalg.norm(a); b=b/np.linalg.norm(b)
    if a@b < 0: b=-b
    return float(4*math.atan2(np.linalg.norm(a-b),np.linalg.norm(a+b)))


def belongs(path, root):
    return path == root or path.startswith(root+'/')


def clamping_face(part, point):
    """Intended contact faces in geometry.py; 0.5 mm collision tolerance.

    Housing shelves and the handle grip body are deliberately ineligible.
    Update this geometric contract when refining the physical clamping faces.
    """
    x,y,z=point; tol=.0005
    if part=='jaw':
        return (-.017-tol <= x <= -.009+tol and .0065-tol <= abs(y) <= .0115+tol
                and abs(x+z+.0146) <= tol)
    if part!='housing': return False
    inner=(abs(x-.0122)<=tol and .012-tol<=abs(y)<=.020+tol and -.015-tol<=z<=.003+tol)
    a=-math.atan(.006/.080);c,s=math.cos(a),math.sin(a)
    v=np.array([[c,0,-s],[0,1,0],[s,0,c]])@(np.array(point)-[.006782,0,-.030])
    outer=(abs(v[0]-.0025)<=tol and abs(v[1])<=.005+tol and abs(v[2])<=.007+tol)
    a=math.pi/3;c,s=math.cos(a),math.sin(a)
    v=np.array([[c,0,-s],[0,1,0],[s,0,c]])@(np.array(point)-[-.001184,0,.00219])
    upper=(abs(v[0])<=.0045+tol and abs(v[1])<=.004+tol and abs(v[2]+.001)<=tol)
    return bool(inner or outer or upper)


class PanScore:
    def __init__(self, binding):
        self.paths=binding['bodies']; self.tips=binding['fingertips']
        self.supports=binding['supports']; self.dt=float(binding['physics_dt_s'])
        if not 0 < self.dt <= .005: raise ValueError('Insufficient native contact sampling')
        self.names={v:k for k,v in self.paths.items()}
        namespace=self.paths['pan'][:-len('/pan/pan')]
        self.robot_roots=(namespace+'/LeftArm',namespace+'/RightArm')
        self.previous=None; self.step=int(binding['first_physics_step'])-1
        self.source_ready=False; self.carry=False; self.arrived=False
        self.proven=False; self.reference=None; self.floor_loss=False; self.invalid_geometry=False
        self.lift_s=0.; self.rest_s=0.
        self.metrics=dict(success=False,score=0.,proof_hold_s=0.,released_hold_s=0.,
            observed_pan_speed_m_s=0.,observed_pan_angular_speed_rad_s=0.)

    def robot(self,path):
        return any(belongs(path,p) for p in self.robot_roots)

    def inspect(self, frame):
        poses={k:np.asarray(v,dtype=float) for k,v in frame['poses_xyzw'].items()}
        if any(v.shape!=(7,) or not np.isfinite(v).all() for v in poses.values()):
            raise ValueError('Missing or invalid native pose')
        rotations={k:rotation(v[3:]) for k,v in poses.items()}
        loaded=[r for r in frame['rows'] if abs(r['impulse_Ns'])>1e-9]
        pan_rows=[r for r in loaded if r['body']=='pan']
        partners={self.names.get(r['other'],r['other']) for r in pan_rows}
        proper=True
        for row in pan_rows:
            part=self.names.get(row['other'])
            if part in ('housing','jaw'):
                point=rotations[part].T@(np.asarray(row['point_m'])-poses[part][:3])
                proper &= clamping_face(part,point)
                if part=='jaw':
                    on_pan=rotations['pan'].T@(np.asarray(row['point_m'])-poses['pan'][:3])
                    proper &= .0835<=on_pan[2]<=.0955 and .088<=np.linalg.norm(on_pan[:2])<=.0985
        table=any(belongs(r['other'],self.supports['table']) for r in pan_rows)
        trivet=any(belongs(r['other'],self.supports['trivet']) for r in pan_rows)
        grasp=False
        for tips in self.tips.values():
            normals=[[np.asarray(r['normal']) for r in loaded
                if r['body']=='housing' and r['other']==tip] for tip in tips]
            grasp |= bool(all(normals) and any(a@b<-.5 for a in normals[0] for b in normals[1]))
        robot_contact=any(self.robot(r['other']) for r in loaded)
        forbidden=any(r['body']!='housing' and self.robot(r['other']) for r in loaded)
        H=rotations['housing']; P=rotations['pan']
        lever=H.T@rotations['lever']; lever_angle=math.atan2(lever[0,2],lever[0,0])
        closed=abs(lever_angle-CLOSED_ANGLE)<=math.radians(2)
        relative=(H.T@(poses['pan'][:3]-poses['housing'][:3]),H.T@P)
        connected=closed and proper and {'housing','jaw'} <= partners
        clean=connected and partners=={'housing','jaw'} and grasp and not forbidden
        source=table and np.linalg.norm(poses['pan'][:2]-[.30,.16])<=.030
        target=(trivet and np.linalg.norm(poses['pan'][:2]-np.asarray(TRIVET[:2]))<=.020
                and abs(poses['pan'][2]-(TABLE+.012))<=.002 and P[2,2]>=math.cos(math.radians(5)))
        target &= all(self.names.get(r['other']) in ('housing','jaw') or
                      belongs(r['other'],self.supports['trivet']) for r in pan_rows)
        landing=trivet and not forbidden and all(self.names.get(r['other']) in ('housing','jaw') or
            belongs(r['other'],self.supports['trivet']) for r in pan_rows)
        handle_table=any(r['body']!='pan' and belongs(r['other'],self.supports['table']) for r in loaded)
        return dict(poses=poses,relative=relative,connected=connected,clean=clean,
            source=source,table=table,trivet=trivet,target=target,landing=landing,grasp=grasp,
            invalid_geometry=any(r['separation_m']<-.002 for r in loaded),
            robot_contact=robot_contact,handle_table=handle_table,
            elevated=poses['pan'][2]>=TABLE+.020 and P[2,2]>=math.cos(math.radians(5)))

    def update(self,frames):
        if not frames:return self.metrics.copy()
        if len(frames)!=round(.05/self.dt): raise ValueError('Incomplete control interval')
        lift_ok=rest_ok=True; pan_distance=pan_angle=handle_distance=handle_angle=0.
        for frame in frames:
            if self.step is not None and frame['physics_step']!=self.step+1:
                raise ValueError('Missing or repeated native physics frame')
            self.step=frame['physics_step']; s=self.inspect(frame); poses=s['poses']
            if self.previous is not None:
                pan_distance+=np.linalg.norm(poses['pan'][:3]-self.previous['pan'][:3])
                pan_angle+=angle(poses['pan'][3:],self.previous['pan'][3:])
                handle_distance+=np.linalg.norm(poses['housing'][:3]-self.previous['housing'][:3])
                handle_angle+=angle(poses['housing'][3:],self.previous['housing'][3:])
            self.previous=poses
            self.invalid_geometry |= s['invalid_geometry']
            self.floor_loss |= any(p[2]<TABLE-.10 for p in poses.values())
            if s['table']:
                self.source_ready=s['source'] and s['connected']
                if not s['source']:self.carry=False
            elif not s['clean']:
                # Direct support or a free fall cannot preserve source eligibility.
                self.source_ready=False
            if s['clean'] and self.reference is None:self.reference=s['relative']
            intact=s['connected']
            if self.reference is not None:
                pos,R=s['relative'];a,b=self.reference
                turn=math.acos(np.clip((np.trace(R@b.T)-1)/2,-1,1))
                intact &= np.linalg.norm(pos-a)<=.002 and turn<=math.radians(2)
            if not intact:
                self.proven=False;self.lift_s=0.;self.carry=False;self.reference=None
                self.source_ready=False
            if self.source_ready and s['clean']:
                self.carry=True;self.source_ready=False
            if self.carry and not s['clean']:
                if s['landing'] and self.proven and intact:self.arrived=True
                self.carry=False
            if not s['table'] and not s['trivet'] and not s['clean']:
                self.proven=False;self.lift_s=0.
            if self.arrived and ((not s['trivet'] and not s['clean']) or s['table'] or
                    (not intact and not s['grasp'] and not s['handle_table'])):
                self.arrived=False
            lift_ok &= s['clean'] and s['elevated'] and intact
            rest_ok &= self.arrived and s['target'] and not s['robot_contact'] and (intact or s['handle_table'])
        duration=len(frames)*self.dt
        speed=pan_distance/duration;angular=pan_angle/duration
        stable=speed<=.020 and angular<=.1
        self.lift_s=self.lift_s+duration if lift_ok and stable else 0.
        if self.lift_s>=1.-1e-9:self.proven=True
        stable &= handle_distance/duration<=.020 and handle_angle/duration<=.1
        self.rest_s=self.rest_s+duration if rest_ok and stable else 0.
        full=self.rest_s>=2.-1e-9 and not (self.floor_loss or self.invalid_geometry)
        partial=self.proven and intact and (s['clean'] or s['table'] or s['trivet']) and not (self.floor_loss or self.invalid_geometry)
        self.metrics=dict(success=bool(full),score=1. if full else .5 if partial else 0.,
            proof_hold_s=self.lift_s,released_hold_s=self.rest_s,
            observed_pan_speed_m_s=speed,observed_pan_angular_speed_rad_s=angular,
            invalid_physics=bool(self.floor_loss or self.invalid_geometry))
        return self.metrics.copy()
