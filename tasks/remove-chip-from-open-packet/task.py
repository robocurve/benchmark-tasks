"""Take one chip out of the open packet and put it on the plate. Leave the other two chips inside the packet and both hands clear."""
import math
import numpy as np
import design as d
from snack_runtime import Articulated
from snack_base import Compound
from scoring import Score,Sample
from yam_arena.task_file import TABLE_TOP
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D
from yam_arena.demo import Waypoint

SCENE='station'
TABLE=TABLE_TOP[SCENE]
SCORE_OVER='end'
SUCCESS_HOLD_S=1.
ORIGIN=(*d.PACKET_XY,TABLE+.002)
OBJECTS=[Articulated('packet',pos=ORIGIN,size=(.100,.080,.065),mass=.012,
    parts=tuple(d.packet_parts()),self_collisions=True,
    links=tuple(dict(name=name,pos=(0,y,.028),mass=.002,
                     parts=(d.box((0,0,.015),(.090,.0008,.030)),))
                for name,y in (('front',-.035),('back',.035))),
    joints=tuple(dict(name=name+'_flex',type='revolute',axis='X',body0='root',body1=name,
                     anchor0=(0,y,.028),anchor1=(0,0,0),lower=-35.,upper=35.,
                     stiffness=.001*math.pi/180,damping=.0001*math.pi/180,max_force=.01,
                     target=target)
                 for name,y,target in (('front',-.035,12.),('back',.035,-12.))),
    initial_joints=(('front_flex',math.radians(12)),('back_flex',math.radians(-12))),
    contact_pairs=(('root','front'),('root','back'),('front','back'))),
    Compound('plate',pos=(*d.PLATE_XY,TABLE),size=(.100,.100,.012),parts=tuple(d.plate_parts()))]
for i,p in enumerate(d.CHIP_ORIGINS):
    OBJECTS.append(Compound('chip'+str(i),pos=tuple(np.array(ORIGIN)+p),size=(.033,.040,.006),
                            mass=.0012,parts=tuple(d.chip_parts())))
_score=Score(TABLE)
_diag={}


def reset_history():
    global _score,_diag
    _score=Score(TABLE);_diag={}


def observe(states,t,base):
    global _diag
    origin=base.scene.env_origins[0].detach().cpu().numpy()
    art=base.scene['packet']
    pp=art.data.body_link_pose_w.torch[0,art.body_names.index('root')].detach().cpu().numpy()
    poses=[base.arena_world.get_pose_w('chip'+str(i))[0].detach().cpu().numpy() for i in range(3)]
    from yam_arena.runtime import pad_tips_world
    pads=np.array([pad_tips_world(base.scene[n])-origin for n in ('left_robot','right_robot')])
    targets=np.array([pp[:3]-origin]+[p[:3]-origin for p in poses])
    clear=float(np.linalg.norm(pads[:,None,:]-targets[None,:,:],axis=2).min())
    s=Sample(float(t),tuple(pp[:3]-origin),tuple(pp[3:]),tuple(tuple(p[:3]-origin) for p in poses),
             tuple(tuple(p[3:]) for p in poses),tuple(float(np.linalg.norm(states['chip'+str(i)].lin_vel)) for i in range(3)),
             tuple(float(np.linalg.norm(states['chip'+str(i)].ang_vel)) for i in range(3)),clear)
    _score.update(s)
    _diag=dict(t=float(t),score=_score.value,reason=_score.reason,invalid=_score.invalid,
               extracted=sorted(_score.extracted),chip_pos=s.chip_pos,hand_clearance=clear,packet_pose=pp.tolist(),pad_tips=pads.tolist())


def diagnostics():return _diag
def score(states):return _score.value
def success(states):return _score.value==1.
def distance(states):return 1.-_score.value


def demo():
    def wp(dt,p,f,label,yaw=0.):return Waypoint(dt,tuple(np.array(p)+[0,0,D]),f,label,yaw=yaw)
    fin=np.array(ORIGIN)+[-.069,.015,.026]
    chip=np.array([.2994,.0092,TABLE+.0061])
    place=np.array((*d.PLATE_XY,TABLE+.005))+[0,0,.002]
    # A partly closed gripper fits the already open mouth. Grip then extract
    # vertically above the entire mouth before carrying sideways.
    narrow=-.020
    left=[wp(5.,[.18,.20,TABLE+.15],O,'park clear'),
          wp(38.,[.18,.20,TABLE+.15],O,'remain clear')]
    right=[wp(4.,chip+[0,-.08,.12],-.024,'approach outside',math.pi/2),
           wp(4.,chip+[0,0,.10],-.024,'above mouth',math.pi/2),
           wp(3.,chip,-.024,'enter mouth',math.pi/2),wp(2.,chip,-.024,'settle approach',math.pi/2),
           wp(1.5,chip,C,'pinch top chip',math.pi/2),wp(1.,chip,C,'settle grasp',math.pi/2),
           wp(3.,chip+[0,0,.09],C,'extract vertically',math.pi/2),
           wp(2.,place+[0,0,.09],C,'carry to plate',math.pi/2),
           wp(2.,place,C,'lower',math.pi/2),wp(1.,place,-.024,'release',math.pi/2),
           wp(2.,place+[0,0,.11],-.024,'withdraw',math.pi/2),
           wp(10.,place+[0,0,.11],O,'both hands clear',math.pi/2)]
    return dict(left=left,right=right)
