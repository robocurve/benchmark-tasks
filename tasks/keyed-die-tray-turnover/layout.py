import numpy as np
from scipy.spatial.transform import Rotation as R
PHYSICS_HZ = 480

PIVOT=np.array([.265,0,.788])
BASIS=R.from_euler('z',90,degrees=True).as_matrix()
RADIUS=.042
HANDLE_X=.105
HANDLE_Y=.043
RIM_Z=-.00046
BED_Z=-.00402
TRAY_YAW={'source':np.eye(3),'receiver':R.from_euler('z',-90,degrees=True).as_matrix()}
CELLS=[(-1,-1),(-1,0),(0,1),(1,1)]
def world(p):
    return PIVOT + BASIS@np.array(p)
def knob(sign,angle):
    rx=R.from_euler('x',angle,degrees=True)
    return world(rx.apply([sign*HANDLE_X,sign*HANDLE_Y,0]))

STAGE=np.array([.379,-.140,.776])
RECEIVING_CENTER=world([0,-RADIUS,-.00014])
KEY_CENTER=.0200
