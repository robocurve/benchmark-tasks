"""Standard YAM arms/wrists with an oblique fixed view of the torque dial."""
import numpy as np
from yam_arena import station
from yam_arena.rig import Camera,Rig
from yam_arena.ik import matrix_to_quat_xyzw

def rig():
    p=np.array([-.12,0.,.87]);target=np.array([.45,0.,.83])
    z=(p-target)/np.linalg.norm(p-target)
    x=np.cross([0.,0.,1.],z);x/=np.linalg.norm(x);y=np.cross(z,x)
    camera=Camera(tuple(p),tuple(matrix_to_quat_xyzw(np.column_stack([x,y,z]))),'opengl',station.lens(station.TOP_CAM_K))
    return Rig('photoelectric-service',station.BASES,camera,
               {side:station.wrist_camera(side) for side in station.BASES},True)
