"""Declared, deterministic geometry; SI units. No simulator dependencies."""
import math
import numpy as np
CENTER=np.array([.40,0.,.9875])
W=np.array([[0.,0.,-1.],[1.,0.,0.],[0.,-1.,0.]])
CRANK_R=.085
CRANK_Z=-.100
TURN_DAMPING=.080
CONTACT_OFFSET=.000080
REST_OFFSET=.000030
KEY_ANGLE=math.radians(-27)
TABLE_Z=.7475
R=.0185
WIRE_R=.00065
PITCH=.0016
N=20
END_ANGLE=math.radians(705)
CLAMP_INDEX=5
SPRING_K=1.4
SPRING_DAMPING=.010
HOLE_R=.0035
BOW_R=.0055
KEY_MASS=.009
GRIP=np.array([.041,0.,0.])
START_ANGLE=math.radians(-5)
KEY_START=np.array([.375,-.14,.9675])
KEY_START_ANGLE=0.
OPENER_ANGLE=math.radians(-35)
OPENER_STROKE=.008
OPENER_RETRACT=.140
OPENER_HANDLE=.140
OPENER_GRIP_Z=.035
OPENER_LIFT=.000

def rz(q):
 c,s=math.cos(q),math.sin(q)
 return np.array([[c,-s,0],[s,c,0],[0,0,1.]])

def rotation(quat):
 x,y,z,w=quat
 return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])

def ring_points():
 a=np.linspace(0,END_ANGLE,N+1)
 return np.stack([R*np.cos(a),R*np.sin(a),PITCH*(a/(2*math.pi)-.5)],1)

def key_rotation(a):
 radial=W@np.array([math.cos(a),math.sin(a),0.]);tangent=W@np.array([-math.sin(a),math.cos(a),0.]);normal=W[:,2]
 return np.stack([normal,tangent,-radial],1)
def grasp_rotation(a):
 kr=key_rotation(a);return np.stack([-kr[:,1],kr[:,2],-kr[:,0]],1)
def grasp_point(hole,a):return np.asarray(hole)+key_rotation(a)@GRIP

def hole_target(a,z):return CENTER+W@np.array([R*math.cos(a),R*math.sin(a),z])

def segment_curve(i):
 a=np.linspace(END_ANGLE*i/N,END_ANGLE*(i+1)/N,7)
 return np.stack([R*np.cos(a),R*np.sin(a),PITCH*(a/(2*math.pi)-.5)],1)
def crank_point(q):return CENTER+W@np.array([CRANK_R*math.cos(q),CRANK_R*math.sin(q),CRANK_Z])
