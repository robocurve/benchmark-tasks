"""Unfastened leather band; folds start outside the rear clasp faces."""
import numpy as np
from scipy.spatial.transform import Rotation
N=41
STEP=.005
RADIUS=.005
STEM=.005  # metal housing half-length before the leather can bend
LENGTH=(N-2)*STEP+2*STEM
STRAIGHT=LENGTH-2*STEM-2*np.pi*RADIUS
END_Y=STRAIGHT/2-STEM
Z=.761
POSITIONS=[];ROTATIONS=[]
clasp=Rotation.from_euler('z',-90,degrees=True).as_matrix()
samples=np.r_[0.,STEM+STEP/2+np.arange(N-2)*STEP,LENGTH]
for i,s in enumerate(samples):
 if s<=STEM:
  y=END_Y+s;z=Z;t=np.array([0.,1.,0.])
 elif s<=STEM+np.pi*RADIUS:
  a=(s-STEM)/RADIUS;y=END_Y+STEM+RADIUS*np.sin(a);z=Z+RADIUS*(1-np.cos(a));t=np.array([0.,np.cos(a),np.sin(a)])
 elif s<STEM+np.pi*RADIUS+STRAIGHT:
  y=END_Y+STEM-(s-STEM-np.pi*RADIUS);z=Z+2*RADIUS;t=np.array([0.,-1.,0.])
 elif s<LENGTH-STEM:
  a=(s-STEM-np.pi*RADIUS-STRAIGHT)/RADIUS;y=-END_Y-STEM-RADIUS*np.sin(a);z=Z+RADIUS*(1+np.cos(a));t=np.array([0.,-np.cos(a),-np.sin(a)])
 else:
  y=-END_Y-(LENGTH-s);z=Z;t=np.array([0.,1.,0.])
 POSITIONS.append([.34,y,z]);ROTATIONS.append(clasp if i in (0,N-1) else np.column_stack((t,[1.,0.,0.],np.cross(t,[1.,0.,0.]))))
POSITIONS=np.array(POSITIONS);ROTATIONS=np.array(ROTATIONS);QUATS=Rotation.from_matrix(ROTATIONS).as_quat()
ANCHOR0=[];ANCHOR1=[];JOINT_ROT0=[];JOINT_ROT1=[]
for i in range(N-1):
 mid=(POSITIONS[i]+POSITIONS[i+1])/2
 if i==0:mid=POSITIONS[0]+np.array([0.,STEM,0.])
 if i==N-2:mid=POSITIONS[-1]-np.array([0.,STEM,0.])
 ANCHOR0.append(ROTATIONS[i].T@(mid-POSITIONS[i]));ANCHOR1.append(ROTATIONS[i+1].T@(mid-POSITIONS[i+1]))
 frame=ROTATIONS[i+1] if i==0 else ROTATIONS[i]
 JOINT_ROT0.append(Rotation.from_matrix(ROTATIONS[i].T@frame).as_quat());JOINT_ROT1.append(Rotation.from_matrix(ROTATIONS[i+1].T@frame).as_quat())
ANCHOR0=np.array(ANCHOR0);ANCHOR1=np.array(ANCHOR1)
JOINT_ROT0=np.array(JOINT_ROT0);JOINT_ROT1=np.array(JOINT_ROT1)
