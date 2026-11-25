"""Full-size wireless keyboard and two independently movable rear supports."""
import numpy as np
from scipy.spatial.transform import Rotation
CONTROL_HZ=20
PHYSICS_HZ=240
MAX_STEPS=1800
INSTRUCTION='With the keyboard already underside-up after cleaning, unfold both rear tilt feet, then turn it keys-up and leave it resting securely on both extended feet and its front pads. Leave both hands clear.'
TABLE_BOUNDS=(.045,.650,-.500,.500)
LINKS=('a_keyboard','b_left_foot','c_right_foot','d_notebook')
BODY,LEFT,RIGHT=0,1,2
WIDTH,DEPTH,HEIGHT=.450,.155,.0236
MASS=.550
LEG_LENGTH=.024
LEG_WIDTH=.015
LEG_THICKNESS=.0024
PIVOTS=np.array(((.056,.170,.0033),(.056,-.170,.0033)))
FOLD_ANGLE=0.
OPEN_ANGLE=np.deg2rad(-120.)
HINGE_PAD_STIFFNESS=10000.
HINGE_PAD_DAMPING=2.
HINGE_PRELOAD=.0001
START_CENTER=np.array((.3745,0.,.0496))
START_ROT=Rotation.from_euler('x',np.pi).as_matrix()
START_QUAT=Rotation.from_matrix(START_ROT).as_quat()
# The free protective book supports the rear key rows and mass centre while
# leaving the front edge accessible to an ordinary central pinch.
START_POSES=np.array([np.r_[START_CENTER,START_QUAT],*[np.r_[START_CENTER+START_ROT@p,START_QUAT] for p in PIVOTS],np.r_[[.4145,0.,.015],[0.,0.,0.,1.]]])
RESET_POSES=START_POSES
MASSES=np.array((MASS-.007,.0035,.0035,.30))
BOUNCE_THRESHOLD=.2
RUBBER_FRICTION=.75
LEG_FRICTION=.4

def quat_matrix(q):return Rotation.from_quat(q).as_matrix()
def rx(a):return Rotation.from_euler('x',a).as_matrix()
def ry(a):return Rotation.from_euler('y',a).as_matrix()
def rz(a):return Rotation.from_euler('z',a).as_matrix()
def angles(poses):
 R=quat_matrix(poses[0,3:]);out=[]
 for p in poses[1:3]:
  Q=R.T@quat_matrix(p[3:]);out.append(float(np.arctan2(Q[0,2],Q[0,0])))
 return np.array(out)
def corners(size,center=(0,0,0)):
 return np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])*np.asarray(size)+center
LEG_CORNERS=np.concatenate((corners((LEG_LENGTH,LEG_WIDTH,LEG_THICKNESS),(-LEG_LENGTH/2,0,0)),corners((.004,LEG_WIDTH,.003),(-LEG_LENGTH+.001,0,-.0003)),corners((.004,.015,.004)),corners((.004,.003,.004),(0,.008,0)),corners((.004,.003,.004),(0,-.008,0))))
FRONT_PADS=np.array(((-.067,.185,-.002),(-.067,-.185,-.002)))

FRONT_PAD_CORNERS=np.array([corners((.016,.025,.004),p) for p in FRONT_PADS])
