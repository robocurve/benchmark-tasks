"""Full-size keyboard on the original calibrated YAM table."""
from pathlib import Path
import numpy as np
from . import layout as L
try:
    from yam_arena import station as _station
    TOP=float(_station.TABLE_TOP)
    _BASES={a:np.asarray(_station.BASES[a]) for a in ('left','right')}
except ImportError:
    TOP=.7475
    _BASES={'left':np.array((0.,.305,.75)),'right':np.array((0.,-.305,.75))}
ARMS=('left','right')
BASES={a:p-(0.,0.,TOP) for a,p in _BASES.items()}
USD=Path(__file__).with_name('keyboard_feet.usda')
CONTROL_HZ,MAX_STEPS,INSTRUCTION=L.CONTROL_HZ,L.MAX_STEPS,L.INSTRUCTION
LINKS=L.LINKS
TABLE_BOUNDS=L.TABLE_BOUNDS
RESET_POSES=L.RESET_POSES


def side_camera():
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera,Lens
    eye=np.array((1.25,-.25,TOP+.65))
    back=eye-np.array((.43,0.,TOP+.16));back/=np.linalg.norm(back)
    right=np.cross((0.,0.,1.),back);right/=np.linalg.norm(right)
    rot=np.column_stack((right,np.cross(back,right),back))
    return Camera(tuple(eye),tuple(matrix_to_quat_xyzw(rot)),'opengl',Lens.from_focal(18.))
