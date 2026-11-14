"""Vertical pipe-flange assembly with two horizontal locating pins."""
import numpy as np
from yam_arena.demo import Waypoint,pick_place_waypoints,grip_rotation
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix
from flange_assembly.policy import PoseWaypoint
from yam_arena.constants import FINGER_OPEN,FINGER_CLOSED,READY
from yam_arena import ik,station
from yam_arena.task_file import TABLE_TOP,Box,Cylinder

SCENE='station'
TOP=TABLE_TOP[SCENE]
HOLDER=(.3145,.06)
# Vertical fixture: CAD origin is at the mating face.
LOWER=Box('lower_flange',pos=(.3145,.06,TOP+.17),size=(.051,.15,.15))
UPPER=Box('upper_flange',pos=(.19,.20,TOP+.076),size=(.051,.15,.15),mass=.25)
PAD=Box('holder_pad',pos=(.40,.06,TOP+.065),size=(.05,.08,.13),color=(.23,.25,.27))
PIPE=Box('fixed_pipe',pos=(.4145,.06,TOP+.17),size=(.10,.054,.054),color=(.4,.46,.52))
PIN_START=[(.32,-.30),(.32,-.18)]
PINS=[Cylinder(f'pin_{i+1}',pos=(x,y,TOP+.195),radius=.007,height=.05,mass=.035,color=(.7,.5,.15)) for i,(x,y) in enumerate(PIN_START)]
LOWER_STAGE=Box('lower_stage',pos=(.10,.20,TOP+.028),size=(.06,.08,.056),color=(.34,.32,.28))
UPPER_STAGE=Box('upper_stage',pos=(.215,.20,TOP+.035),size=(.018,.08,.07),color=(.34,.32,.28))
LUG_STAGE=Box('lug_stage',pos=(.115,.295,TOP+.0275),size=(.025,.035,.055),color=(.34,.32,.28))
PIN_SUPPORTS=[Box(f'pin_support_{i+1}',pos=(x+.005,y,TOP+.0925),size=(.05,.028,.185),color=(.34,.32,.28)) for i,(x,y) in enumerate(PIN_START)]
OBJECTS=[LOWER,UPPER,PAD,PIPE,LOWER_STAGE,UPPER_STAGE,LUG_STAGE,*PINS,*PIN_SUPPORTS]
SCORE_OVER='end'


def score(o):
    from flange_assembly.geometry import score as horizontal_score
    return horizontal_score(o['lower_flange'], o['upper_flange'],
                            [o[f'pin_{i}'] for i in range(1, 3)])


def demo():
    pick=(.115,.295,TOP+.094)
    target=(.2195,.155,TOP+.188)
    correction=np.array([0.,-.001,0.])
    corrected=axis_angle_matrix(correction/np.linalg.norm(correction),np.linalg.norm(correction)) @ grip_rotation()
    rot=tuple(corrected.reshape(-1))
    left=[Waypoint(2.,(.115,.295,TOP+.16),FINGER_OPEN,'approach'),
          Waypoint(2.,pick,FINGER_OPEN,'descend'),
          Waypoint(1.5,pick,FINGER_CLOSED,'grasp'),
          Waypoint(3.,(.115,.295,TOP+.18),FINGER_CLOSED,'lift'),
          PoseWaypoint(5.,target,FINGER_CLOSED,'align',rotation=rot),
          PoseWaypoint(95.,target,FINGER_CLOSED,'hold flange',rotation=rot)]
    park_pos,park_rot=ik.tcp_pose(np.asarray(READY),station.BASES['right'])
    side_rot=axis_angle_matrix(np.array([0.,1.,0.]),-np.pi/2) @ grip_rotation()
    side=tuple(side_rot.reshape(-1))
    def P(t,p,f,label): return PoseWaypoint(t,p,f,label,rotation=side)
    right=[PoseWaypoint(40.,tuple(park_pos),FINGER_OPEN,'wait',rotation=tuple(park_rot.reshape(-1))),
           P(3.,(.265,-.18,TOP+.24),FINGER_OPEN,'approach above pin'),
           P(3.,(.265,-.18,TOP+.19),FINGER_OPEN,'lower beside pin'),
           P(2.,(.277,-.18,TOP+.19),FINGER_OPEN,'enter pin'),
           P(1.5,(.277,-.18,TOP+.19),FINGER_CLOSED,'grasp pin'),
           P(3.,(.277,-.18,TOP+.23),FINGER_CLOSED,'lift pin'),
           P(5.,(.20,.06,TOP+.1135),FINGER_CLOSED,'approach hole'),
           P(7.,(.292,.06,TOP+.1135),FINGER_CLOSED,'insert pin'),
           P(2.,(.292,.06,TOP+.1135),FINGER_CLOSED,'seat pin'),
           P(1.,(.292,.06,TOP+.1135),FINGER_OPEN,'release pin'),
           P(2.,(.245,.06,TOP+.1135),FINGER_OPEN,'back off pin'),
           P(1.,(.245,.06,TOP+.1135),FINGER_CLOSED,'close to push'),
           P(4.,(.285,.06,TOP+.1135),FINGER_CLOSED,'push pin head'),
           P(2.,(.285,.06,TOP+.0835),FINGER_CLOSED,'clear pin head'),
           P(3.,(.18,.06,TOP+.0835),FINGER_CLOSED,'withdraw'),
           P(3.,(.265,-.30,TOP+.24),FINGER_OPEN,'approach second pin'),
           P(3.,(.265,-.30,TOP+.19),FINGER_OPEN,'lower beside second pin'),
           P(2.,(.277,-.30,TOP+.19),FINGER_OPEN,'enter second pin'),
           P(1.5,(.277,-.30,TOP+.19),FINGER_CLOSED,'grasp second pin'),
           P(3.,(.277,-.30,TOP+.23),FINGER_CLOSED,'lift second pin'),
           P(5.,(.20,.06,TOP+.2235),FINGER_CLOSED,'approach second hole'),
           P(7.,(.292,.06,TOP+.2235),FINGER_CLOSED,'insert second pin'),
           P(2.,(.292,.06,TOP+.2235),FINGER_CLOSED,'seat second pin')]
    return {'left':left,'right':right}
