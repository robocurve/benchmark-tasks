"""Stock YAM bench coordinates for the hair-dryer handle fold operation."""
from pathlib import Path
import numpy as np
from . import layout as L
try:
    from yam_arena import station
    TOP=float(station.TABLE_TOP)
    TABLE_SUPPORT_BOUNDS=(station.TABLE_CENTER[0]-station.TABLE_SIZE[0]/2,station.TABLE_CENTER[0]+station.TABLE_SIZE[0]/2,station.TABLE_CENTER[1]-station.TABLE_SIZE[1]/2,station.TABLE_CENTER[1]+station.TABLE_SIZE[1]/2)
    BASES={a:np.asarray(station.BASES[a])-(0.,0.,TOP) for a in ('left','right')}
except ImportError:
    TABLE_SUPPORT_BOUNDS=(-.19,.91,-.65,.65)
    TOP=.7475;BASES={'left':np.array((0.,.305,.0025)),'right':np.array((0.,-.305,.0025))}
ARMS=('left','right')
USD=Path(__file__).with_name('hairdryer.usda')
CONTROL_HZ,MAX_STEPS,INSTRUCTION=L.CONTROL_HZ,L.MAX_STEPS,L.INSTRUCTION
LINKS,RESET_POSES,TABLE_BOUNDS=L.LINKS,L.RESET_POSES,L.TABLE_BOUNDS

def side_camera():
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera,Lens
    eye=np.array((.62,-.40,TOP+.38));back=eye-np.array((.33,-.020,TOP+.060));back/=np.linalg.norm(back)
    right=np.cross((0.,0.,1.),back);right/=np.linalg.norm(right)
    return Camera(tuple(eye),tuple(matrix_to_quat_xyzw(np.column_stack((right,np.cross(back,right),back)))),'opengl',Lens.from_focal(20.))
