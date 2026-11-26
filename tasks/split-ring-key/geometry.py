"""Declared, deterministic geometry; SI units. No simulator dependencies."""
import math
import numpy as np
CENTER=np.array([.40,0.,.9875])
W=np.array([[0.,0.,-1.],[1.,0.,0.],[0.,-1.,0.]])
CRANK_R=.085
CRANK_Z=-.100
TURN_DAMPING=.080
CONTACT_OFFSET=.000080
REST_OFFSET=0.
KEY_ANGLE=math.radians(-32)
TABLE_Z=.7475
OUTSIDE_DIAMETER=.038
WIRE_R=.00099
R=OUTSIDE_DIAMETER/2-WIRE_R
PITCH=2*WIRE_R
N=20
END_ANGLE=math.radians(700)
CLAMP_INDEX=5
SPRING_K=7.5
SPRING_DAMPING=.030
HOLE_R=.0035
BOW_R=.0055
KEY_MASS=.009
GRIP=np.array([.041,0.,0.])
START_ANGLE=math.radians(-5)
KEY_START=np.array([.375,-.14,.9675])
KEY_START_ANGLE=0.
# Leave the red free end clear for the bow by placing the catch behind it.
OPENER_ANGLE=END_ANGLE-4*math.pi-math.radians(25)
OPENER_STROKE=.008
OPENER_RETRACT=.140
OPENER_COMMAND_BIAS=.010
OPENER_HANDLE=.140
OPENER_GRIP_Z=.035
OPENER_LIFT=.000
OPENER_Z=0.
KEY_THICKNESS=.002
# Provisional, visible acceptance bands; replace after physical lot calibration.
MAX_RESIDUAL_TIP_GAP=.0005
MAX_DIAMETER_CHANGE=.001
MAX_ROUNDNESS_CHANGE=.001
MAX_VISIBLE_BREAK=.00025
MIN_HAND_CLEARANCE=.020

def ring_curve(a):
 """Nominal touching round-wire turns, with the crossover in the end sector.

 RS 866-579 supplies OD/wire dimensions, not this lot-specific 340 degree
 overlap or crossover. Those features and elastic stiffness remain provisional.
 The two broad planar turns touch at z=0; no artificial entry gap is authored.
 """
 a=np.asarray(a)
 start=END_ANGLE-2*math.pi
 u=np.clip((a-start)/(2*math.pi-start),0,1)
 z=-WIRE_R+PITCH*(u*u*(3-2*u))
 return np.stack([R*np.cos(a),R*np.sin(a),z],-1)

def rz(q):
 c,s=math.cos(q),math.sin(q)
 return np.array([[c,-s,0],[s,c,0],[0,0,1.]])

def rotation(quat):
 x,y,z,w=quat
 return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])

def ring_points():
 a=np.linspace(0,END_ANGLE,N+1)
 return ring_curve(a)

def key_rotation(a):
 radial=W@np.array([math.cos(a),math.sin(a),0.]);tangent=W@np.array([-math.sin(a),math.cos(a),0.]);normal=W[:,2]
 return np.stack([normal,tangent,-radial],1)
def grasp_rotation(a):
 kr=key_rotation(a);return np.stack([-kr[:,1],kr[:,2],-kr[:,0]],1)
def grasp_point(hole,a):return np.asarray(hole)+key_rotation(a)@GRIP

def hole_target(a,z):return CENTER+W@np.array([R*math.cos(a),R*math.sin(a),z])

def segment_curve(i):
 return ring_curve(segment_angles(i))
def segment_angles(i):
 a=np.linspace(END_ANGLE*i/N,END_ANGLE*(i+1)/N,7)
 # Explicit crossover corners keep the collision tubes on the authored curve.
 return np.unique(np.r_[a,[v for v in (END_ANGLE-2*math.pi,2*math.pi) if a[0]<v<a[-1]]])
def span_angles():
 return np.array([(a,b) for i in range(N) for a,b in zip(segment_angles(i)[:-1],segment_angles(i)[1:])])
def crank_point(q):return CENTER+W@np.array([CRANK_R*math.cos(q),CRANK_R*math.sin(q),CRANK_Z])
