"""Analytic accessory carabiners, an empty D eye, and reproducible loose layouts."""
import math
import numpy as np
TABLE=.7475
RADIUS=.00225
SOLVER_POSITION_ITERATIONS=64
SOLVER_VELOCITY_ITERATIONS=16
BODY_ANGULAR_DAMPING=5.
FRAME_MASS=.018
GATE_MASS=.002
GATE_LENGTH=.039
GATE_OFFSET=np.array([.016,0.,0.])
HINGE=np.array([.016,-GATE_LENGTH/2,0.])
GATE_ANCHOR=np.array([0.,-GATE_LENGTH/2,0.])
GATE_MAX_DEG=30.
GATE_REFERENCE_DEG=15.
SPRING_K=.0005
SPRING_D=.00001
SPRING_CAP=.020
FRAME_POINTS=np.array([[.016,.022,0],[.012,.028,0],[.004,.030,0],[-.008,.030,0],[-.014,.026,0],[-.016,.020,0],[-.016,-.020,0],[-.014,-.026,0],[-.008,-.030,0],[.004,-.030,0],[.012,-.028,0],[.016,-.022,0]])
STAND_POS=np.array([.50,0.,TABLE])
EYE_OFFSET=np.array([-.08,0.,.2475])
EYE_RADIUS=.0025
EYE_ROT=np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]])
EYE_POINTS=np.array([[-.025,.025,0],[.025,.025,0],[.025,0,0]]+[[.025*math.cos(a),.025*math.sin(a),0] for a in np.linspace(0,-math.pi,17)[1:]]+[[-.025,.025,0]])
NAMES=('link0','link1','link2')
COLORS=((.72,.12,.08),(.08,.28,.70),(.83,.57,.07))

def rx(a):
 c,s=math.cos(a),math.sin(a);return np.array([[1.,0,0],[0,c,-s],[0,s,c]])
def rz(a):
 c,s=math.cos(a),math.sin(a);return np.array([[c,-s,0],[s,c,0],[0,0,1.]])
def gate_gap(frame_p,frame_r,gate_p,gate_r):
 tip=np.asarray(gate_p)+np.asarray(gate_r)@np.array([0,GATE_LENGTH/2,0]);nose=np.asarray(frame_p)+np.asarray(frame_r)@FRAME_POINTS[0]
 return max(0.,float(np.linalg.norm(tip-nose))-RADIUS)
def gate_angle(frame_r,gate_r):
 r=np.asarray(frame_r).T@np.asarray(gate_r);return math.atan2(float(r[1,0]),float(r[0,0]))
def loop(frame_p,frame_r,gate_p,gate_r):
 frame=FRAME_POINTS@np.asarray(frame_r).T+np.asarray(frame_p)
 # Short centerline joins bridge only the physically measured closed-tip gap.
 lower=np.asarray(gate_p)+np.asarray(gate_r)@np.array([0,-GATE_LENGTH/2,0]);upper=np.asarray(gate_p)+np.asarray(gate_r)@np.array([0,GATE_LENGTH/2,0])
 return np.vstack([frame,lower,upper,frame[0]])
def eye_loop(stand_p=STAND_POS,stand_r=np.eye(3)):
 return (EYE_POINTS@EYE_ROT.T+EYE_OFFSET)@np.asarray(stand_r).T+np.asarray(stand_p)
def layout(seed=17):
 rng=np.random.default_rng(int(seed));zones=np.array([[.30,.22],[.28,0.],[.30,-.22]]);out={}
 for i,zone in enumerate(zones):
  p=np.r_[zone+rng.uniform(-.02,.02,2),TABLE+RADIUS];yaw=rng.uniform(-math.pi,math.pi);face=int(rng.integers(0,2));r=rz(yaw)@rx(math.pi*face)
  if any(np.linalg.norm(p[:2]-q['position'][:2])<.085 for q in out.values()):raise ValueError('Overlapping generated layout')
  out['link'+str(i)]=dict(position=p,rotation=r,yaw=yaw,face_down=face)
 return out
