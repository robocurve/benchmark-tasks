"""YAM forward kinematics and segmented-curve geometry."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import YAM_URDF,station
from yam_arena.urdf import forward_kinematics
from yam_arena.constants import READY,ARM_JOINTS

def fk(q):
 frames=forward_kinematics(YAM_URDF,dict(zip(ARM_JOINTS,map(float,q))))
 for t in frames.values():t[:3,3]+=np.array(station.rig().bases['left'])
 return frames

def segments(vertices):
 d=np.diff(vertices,axis=0);lengths=np.linalg.norm(d,axis=1)
 if np.any(lengths<=.006):raise ValueError('Segments shorter than cable diameter')
 rotations=[]
 for u in d/lengths[:,None]:
  v=np.eye(3)[np.argmin(np.abs(u))];v-=u*np.dot(v,u);v/=np.linalg.norm(v)
  rotations.append(np.column_stack([u,v,np.cross(u,v)]))
 return (vertices[:-1]+vertices[1:])/2,lengths,np.array(rotations)
