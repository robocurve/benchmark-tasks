"""Scoring boundary tests on synthetic observations, not demo evidence."""
import numpy as np
from scipy.spatial.transform import Rotation
from model import Model
from geometry import *

def wrapped(height=0.,clearance=0.):
 m=Model();t=np.linspace(0,3*np.pi,12001)
 xy=np.c_[(RX+clearance)*np.cos(t),(RY+clearance)*np.sin(t)]
 arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
 tt=np.interp(np.arange(N+1)*DX,arc,t)
 v=CENTER+np.c_[(RX+clearance)*np.cos(tt),(RY+clearance)*np.sin(tt),np.full(N+1,height)]
 for i,n in enumerate(CELLS):
  y=v[i+1]-v[i];y/=np.linalg.norm(y);x=np.array([0.,0.,1.]);z=np.cross(x,y)
  R=np.column_stack([x,y,z]);ix=m.index[n];m.poses[ix]=np.r_[(v[i]+v[i+1])/2,Rotation.from_matrix(R).as_quat()];m.R[ix]=R
 m.hold=3.1
 return m
m=Model();assert m.score()==0. and m.reset_valid()
m=wrapped();assert m.score()==1.,m.measure()
m.hold=2.9;assert m.score()==.5
m=wrapped(height=.016);assert m.score()==.5
m=wrapped(height=.031);assert m.score()==0.
m=wrapped(clearance=.011);assert m.score()<1.
m=wrapped(clearance=-.006);assert m.score()==0.
m=wrapped();m.failed=True;assert m.score()==0.
m=wrapped();i=m.index[CELLS[20]];m.R[i]=np.eye(3);assert m.score()==0.
m=wrapped();m.peak_gap=.004;assert m.integrity_failed()
print('Passed reset, full result, hold, level, slack, penetration, integrity and twist boundaries.')
