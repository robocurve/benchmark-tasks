"""Bimanual assembly of two 150 mm pipe flanges with four locating pins.

Place the inverted lower flange in the circular holder; align the upper flange
face-to-face, release it, then alternate arms to place the four pins.
"""
import numpy as np
from yam_arena.demo import Waypoint,pick_place_waypoints,grip_rotation
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix
from flange_assembly.policy import PoseWaypoint
from yam_arena.constants import FINGER_OPEN,FINGER_CLOSED,READY
from yam_arena import ik,station
from yam_arena.task_file import TABLE_TOP,Box,Cylinder

SCENE='station'
TOP=TABLE_TOP[SCENE]
HOLDER=(.28,-.08)
LOWER=Box('lower_flange',pos=(.20,.085,TOP+.106),size=(.15,.15,.051),mass=.15)
UPPER=Box('upper_flange',pos=(.20,-.26,TOP+.055),size=(.15,.15,.051),mass=.25)
PAD=Box('holder_pad',pos=(*HOLDER,TOP+.03),size=(.16,.16,.06),color=(.23,.25,.27))
PIN_START=[(.07,.28),(.07,.09),(.07,-.09),(.07,-.28)]
PINS=[Cylinder(f'pin_{i+1}',pos=(x,y,TOP+.025),radius=.005,height=.05,mass=.035,color=(.7,.5,.15)) for i,(x,y) in enumerate(PIN_START)]
LOWER_STAGE=Box('lower_stage',pos=(LOWER.pos[0],LOWER.pos[1],TOP+.0275),size=(.10,.10,.055),color=(.34,.32,.28))
UPPER_STAGE=Box('upper_stage',pos=(UPPER.pos[0],UPPER.pos[1],TOP+.0275),size=(.10,.10,.055),color=(.34,.32,.28))
OBJECTS=[LOWER,UPPER,PAD,LOWER_STAGE,UPPER_STAGE,*PINS]
SCORE_OVER='end'


def _holes(state):
    """World-space centers of the four bolt holes on this flange's mating plane."""
    rot=quat_xyzw_to_matrix(state.quat_xyzw)
    local=[(.05499,0,0),(-.05499,0,0),(0,.05499,0),(0,-.05499,0)]
    return [state.pos[:2]+(rot @ np.array(v))[:2] for v in local]


def _holder(o):
    a=o['lower_flange']
    return bool(np.linalg.norm(a.pos[:2]-HOLDER)<.012 and abs(a.low[2]-(TOP+.025))<.006 and
                (a.quat_xyzw[0]**2+a.quat_xyzw[1]**2)>.996 and a.resting())


def _aligned(o):
    a,b=o['lower_flange'],o['upper_flange']
    if not (_holder(o) and np.linalg.norm(a.pos[:2]-b.pos[:2])<.004 and
            abs(b.low[2]-a.high[2])<.003 and
            b.quat_xyzw[2]**2+b.quat_xyzw[3]**2>.996 and b.resting()): return False
    upper=_holes(b)
    return all(min(np.linalg.norm(x-y) for y in upper)<.003 for x in _holes(a))


def score(o):
    if not _holder(o): return 0.
    if not _aligned(o): return .2
    a=o['lower_flange']; out=.4; holes=_holes(a); claimed=set()
    for i in range(4):
        p=o[f'pin_{i+1}']; j=int(np.argmin([np.linalg.norm(p.pos[:2]-h) for h in holes]))
        if (j not in claimed and np.linalg.norm(p.pos[:2]-holes[j])<.003 and
            p.low[2] <= a.high[2]-.012 and p.high[2] >= a.high[2]+.016 and
            p.quat_xyzw[2]**2+p.quat_xyzw[3]**2>.996 and p.resting()):
            claimed.add(j); out+=.15
    return min(out,1.)


def demo():
    # The lower flange is transferred by its unmodified rim.
    def rotation(sign):
        return tuple((axis_angle_matrix(np.array([1.,0.,0.]),-sign*np.pi/2) @ grip_rotation()).reshape(-1))
    def W(t,p,f,label,sign): return PoseWaypoint(t,p,f,label,rotation=rotation(sign))
    sx,sy,sz=LOWER.pos[0]+.020,LOWER.pos[1]+.072,TOP+.098
    tx,ty,tz=HOLDER[0]+.053,HOLDER[1]+.042,TOP+.068
    left=[W(2.5,(sx,sy+.02,sz+.02),FINGER_OPEN,'approach lower rim',1),
          W(2.,(sx,sy,sz),FINGER_OPEN,'engage lower rim',1),
          W(1.,(sx,sy,sz),FINGER_CLOSED,'grasp lower rim',1),
          W(3.,(sx,sy,sz+.09),FINGER_CLOSED,'lift lower flange',1),
          W(6.,(tx,ty,tz+.09),FINGER_CLOSED,'carry to holder',1),
          W(2.5,(tx,ty,tz+.003),FINGER_CLOSED,'lower into holder',1),
          W(1.,(tx,ty,tz+.003),FINGER_OPEN,'release lower flange',1),
          W(1.5,(tx,ty+.10,tz+.003),FINGER_OPEN,'withdraw lower horizontally',1),
          W(1.5,(tx,ty+.10,tz+.08),FINGER_OPEN,'lift clear lower',1)]
    # The parked arm returns to its ready pose, outside the other arm's path.
    def parked(side,duration,label):
        pos,rot=ik.tcp_pose(np.asarray(READY),station.BASES[side])
        return PoseWaypoint(duration,tuple(pos),FINGER_OPEN,label,rotation=tuple(rot.reshape(-1)))
    # The right fingers close across x, leaving the north bolt hole open.
    def neck_rotation(yaw):
        return tuple(grip_rotation(yaw).reshape(-1))
    def R(t,p,f,label,yaw=np.pi/2): return PoseWaypoint(t,p,f,label,rotation=neck_rotation(yaw))
    upper_pick=(UPPER.pos[0],UPPER.pos[1],TOP+.103)
    # The neck shifts inside the gripper during the lift; compensate for the
    # measured offset before descending over the lower flange.
    upper_place=(HOLDER[0]+.0035,HOLDER[1]-.006,TOP+.124)
    hover=(upper_pick[0],upper_pick[1],TOP+.16)
    carry=(upper_place[0],upper_place[1],TOP+.16)
    right=[parked('right',23.,'park while lower flange is placed'),
           R(2.,hover,FINGER_OPEN,'approach upper flange'),
           R(2.2,upper_pick,FINGER_OPEN,'approach upper neck'),
           R(1.2,upper_pick,FINGER_CLOSED,'grasp upper neck'),
           R(1.8,hover,FINGER_CLOSED,'lift upper flange'),
           R(2.6,carry,FINGER_CLOSED,'carry upper flange',np.pi/2-.21),
           R(2.8,upper_place,FINGER_CLOSED,'align upper flange',np.pi/2-.21),
           R(1.5,upper_place,FINGER_OPEN,'release upper flange',np.pi/2-.21),
           R(1.,upper_place,FINGER_OPEN,'clear upper neck',np.pi/2-.21),
           R(2.,(upper_place[0],upper_place[1],TOP+.20),FINGER_OPEN,'withdraw above flange',np.pi/2-.21),
           parked('right',3.,'park clear of pin placement')]
    # Wait until the upper flange is released and the right arm has withdrawn.
    left += [parked('left',3.,'clear upper placement'),
             parked('left',20.,'park while upper flange is placed and released')]
    # Only one arm works at a time; after placing a pin it withdraws and
    # returns to its ready pose before the other arm leaves its ready pose.
    def add_pin(plan, side, start, pin, hole, place_yaw=0., travel_height=.22,
                place_height=.118, seat_hold=1.):
        delay=start-sum(w.duration for w in plan)
        if delay < -1e-6:
            raise ValueError(f'{side} arm has not parked before its next turn')
        if delay > 0:
            plan.append(parked(side,delay,'wait while other arm places pin'))
        pick=(pin.pos[0],pin.pos[1],TOP+.068)
        hover=(*pick[:2],TOP+travel_height)
        above=(HOLDER[0]+hole[0],HOLDER[1]+hole[1],TOP+travel_height)
        place=(*above[:2],TOP+place_height)
        moves=[Waypoint(t,p,f,pin.name+' '+label,place_yaw) for t,p,f,label in (
            (2.,hover,FINGER_OPEN,'to the object'),
            (2.,pick,FINGER_OPEN,'descend'),
            (1.,pick,FINGER_CLOSED,'close'),
            (2.,hover,FINGER_CLOSED,'lift'),
            (3.,above,FINGER_CLOSED,'carry'),
            (2.,above,FINGER_CLOSED,'settle above hole'),
            (3.,place,FINGER_CLOSED,'lower'),
            (seat_hold,place,FINGER_CLOSED,'settle pin in hole'),
            (1.,place,FINGER_OPEN,'release'),
            (2.,above,FINGER_OPEN,'retreat'))]
        plan.extend(moves)
        plan.append(parked(side,2.,'park after pin placement'))
    add_pin(left,'left',44.,PINS[0],(.0088,.0547),place_yaw=-np.pi/2,travel_height=.18)
    add_pin(right,'right',65.5,PINS[3],(-.0075,-.0543),place_yaw=np.pi/2,travel_height=.18)
    add_pin(left,'left',87.,PINS[1],(-.0537,.0083),travel_height=.18)
    # Keep every hole inside the upright wrist workspace. The tapered lead
    # engages before release, followed by a short settling pause.
    add_pin(right,'right',108.5,PINS[2],(.0551,-.008),travel_height=.22)
    return {'left':left,'right':right}
