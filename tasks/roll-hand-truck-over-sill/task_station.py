"""Unmodified YAM arms on 950 mm pedestals with a full-height floor view."""
import numpy as np
from yam_arena import station
from yam_arena.rig import Camera,Rig
from yam_arena.ik import matrix_to_quat_xyzw
BASES={'left':(0.,.305,.95),'right':(0.,-.305,.95)}

def rig():
    p=np.array([1.4,-1.,1.65]);target=np.array([.30,0.,.52])
    z=(p-target)/np.linalg.norm(p-target)
    x=np.cross([0.,0.,1.],z);x/=np.linalg.norm(x);y=np.cross(z,x)
    camera=Camera(tuple(p),tuple(matrix_to_quat_xyzw(np.column_stack([x,y,z]))),'opengl',station.lens(station.TOP_CAM_K))
    return Rig('floor-hand-truck',BASES,camera,{s:station.wrist_camera(s) for s in BASES},True)
