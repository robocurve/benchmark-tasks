"""Original stock YAM table, intact robot actions and policy observation spaces."""
from pathlib import Path
import numpy as np
from . import layout as L
try:
 from yam_arena import station as _station
 TOP=float(_station.TABLE_TOP);_BASES={a:np.asarray(_station.BASES[a]) for a in ('left','right')}
except ImportError:
 TOP=.7475;_BASES={'left':np.array((0.,.305,.750)),'right':np.array((0.,-.305,.750))}
ARMS=('left','right');BASES={a:p-(0.,0.,TOP) for a,p in _BASES.items()}
USD=Path(__file__).with_name('toaster_tray.usda')
CONTROL_HZ,MAX_STEPS,INSTRUCTION=L.CONTROL_HZ,L.MAX_STEPS,L.INSTRUCTION
LINKS=L.LINKS;TABLE_BOUNDS=L.TABLE_BOUNDS;RESET_POSES=L.RESET_POSES

def side_camera():
 from yam_arena.ik import matrix_to_quat_xyzw
 from yam_arena.rig import Camera,Lens
 eye=np.array((.20,-.85,TOP+.80));back=eye-np.array((.48,.20,TOP+.08));back/=np.linalg.norm(back)
 right=np.cross((0.,0.,1.),back);right/=np.linalg.norm(right);r=np.column_stack((right,np.cross(back,right),back))
 return Camera(tuple(eye),tuple(matrix_to_quat_xyzw(r)),'opengl',Lens.from_focal(16.))
