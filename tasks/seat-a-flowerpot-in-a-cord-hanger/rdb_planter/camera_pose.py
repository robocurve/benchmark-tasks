import numpy as np
from .fixed_scene import TOP

def side_camera():
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera,Lens
    eye=np.array([.99,-.10,1.21])
    back=eye-np.array([.37,.00,TOP+.19])
    back/=np.linalg.norm(back)
    right=np.cross([0,0,1.],back);right/=np.linalg.norm(right)
    return Camera(tuple(eye),tuple(matrix_to_quat_xyzw(np.column_stack([right,np.cross(back,right),back]))),'opengl',Lens.from_focal(19.))
