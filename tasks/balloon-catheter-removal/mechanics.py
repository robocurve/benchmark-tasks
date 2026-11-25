"""Measured connector/receiver model, independent of Isaac and XR.

Bench-scale pressure/volume surrogate, not a clinical fluid simulation.
"""
import math
import numpy as np

ROOT = '/World/MedicalStation'
BODY_PATHS = {'catheter': ROOT+'/Catheter', 'valve': ROOT+'/Valve',
              'barrel': ROOT+'/Syringe/Barrel', 'plunger': ROOT+'/Syringe/Plunger'}
INFLATED_RADIUS = .015
COLLAPSED_RADIUS = .004
RECEIVER_STROKE = .050
PULL_OFF_TRAVEL = .012
VALVE_MOUTH = np.array([-.014, 0., 0.])
SYRINGE_TIP = np.array([.062, 0., 0.])

def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < .5:
        raise ValueError('Finite XYZW quaternion required')
    x,y,z,w = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def point(pose, local):
    return np.asarray(pose[:3])+rotation(pose[3:7])@local

def receiver_stroke(poses):
    b,p = poses['barrel'],poses['plunger']
    return float(-(rotation(b[3:7]).T@(np.asarray(p[:3])-b[:3]))[0])

def connector_metrics(poses):
    v,b=poses['valve'],poses['barrel']
    mouth=point(v, VALVE_MOUTH)
    tip=point(b, SYRINGE_TIP)
    delta=rotation(v[3:7]).T@(tip-mouth)
    axis_dot=float(np.dot(rotation(v[3:7])[:,0],rotation(b[3:7])[:,0]))
    angle=math.acos(float(np.clip(axis_dot,-1,1)))
    return {'axial_insertion_m':float(delta[0]),'lateral_error_m':float(np.linalg.norm(delta[1:])),
            'angle_rad':angle,'stroke_m':receiver_stroke(poses),
            'aligned':bool(.001 <= delta[0] <= .007 and np.linalg.norm(delta[1:]) <= .002
                           and angle <= math.radians(10))}

class DrainState:
    def __init__(self):
        self.connected=False
        self.ever_connected=False
        self.start_stroke=0.
        self.fraction=0.
        self.radius=INFLATED_RADIUS
        self.failed=False
        self.reason=None
        self.connection_insertion=0.
        self.ever_disconnected=False

    def connect(self, poses):
        m=connector_metrics(poses)
        if not m['aligned'] or abs(m['stroke_m']) > .005 or self.connected:
            return False
        self.connected=True
        self.ever_connected=True
        self.start_stroke=m['stroke_m']
        self.connection_insertion=m['axial_insertion_m']
        return True

    def update(self, poses):
        if not all(np.asarray(p).shape==(7,) and np.isfinite(p).all() for p in poses.values()):
            raise ValueError('Finite measured rigid body poses required')
        m=connector_metrics(poses)
        # The self-closing balloon valve is held open by the inserted nozzle,
        # not by the outer mechanical detent. It closes as insertion drops below
        # 1 mm, before the detent is completely pulled off at 12 mm travel.
        fluid_open=bool(self.connected and m['axial_insertion_m']>=.001)
        if fluid_open:
            f=float(np.clip((receiver_stroke(poses)-self.start_stroke)/RECEIVER_STROKE,0,1))
            # A connected receiver can return its contents: do not latch the
            # maximum stroke and grant permanent clearance after reinflation.
            self.fraction=f
            self.radius=(INFLATED_RADIUS**3*(1-self.fraction)+COLLAPSED_RADIUS**3*self.fraction)**(1/3)
        if self.connected:
            if self.connection_insertion-m['axial_insertion_m'] >= PULL_OFF_TRAVEL:
                self.connected=False
                self.ever_disconnected=True
                if self.fraction<.995:
                    self.failed=True;self.reason='receiver_removed_before_deflation'
        balloon=point(poses['catheter'], np.array([0.,.132,0.]))
        # Neck centre y=.039, near mouth y=.016. A still-inflated retention
        # bulb passing the throat is an invalid extraction, including tunnelling.
        if balloon[1] < .035 and self.radius > .005:
            self.failed=True
            self.reason='inflated_balloon_passed_throat'
        return {'connected':self.connected,'ever_connected':self.ever_connected,
                'fluid_connected':fluid_open and self.connected,
                'ever_disconnected':self.ever_disconnected,
                'drained_fraction':self.fraction,'balloon_radius_m':self.radius,
                'collapsed':self.fraction>=.995,'failed':self.failed,'failure':self.reason,
                'balloon_center':balloon.tolist(),**m}
