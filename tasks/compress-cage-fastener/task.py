"""Fit the blue spring cage fastener into the square opening in the rack coupon.

Compress both spring legs before passing the hooks through the opening. Release the legs
behind the coupon so both hooks retain it. Keep the coupon within 5 mm and 5 degrees of its
starting pose. Release both grippers when the fastener is seated. All equipment is unpowered.
"""
import math
from collections import deque
import numpy as np
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint,pick_place_waypoints
from yam_arena.constants import FINGER_OPEN,FINGER_CLOSED,PAD_TIP_PAST_TCP
from yam_articulated import Articulated,Compound
from yam_arena.urdf import quat_xyzw_to_matrix

SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=1.;SCORE_OVER='end'
def p(pos,size): return dict(pos=pos,size=size)

panel=[p((0,.0285,.018),(.08,.023,.004)),p((0,-.0285,.018),(.08,.023,.004)),
       p((.0285,0,.018),(.023,.034,.004)),p((-.0285,0,.018),(.023,.034,.004)),
       p((0,.105,.022),(.025,.13,.008)),p((0,.17,.03),(.03,.02,.02)),
       p((0,.15,-.012),(.016,.016,.056))]
for x in [-.032,.032]:
    for y in [-.032,.032]: panel.append(p((x,y,-.012),(.008,.008,.056)))
PANEL=Compound('rack_coupon',pos=(.28,-.07,TOP+.04),size=(.08,.22,.08),mass=.24,
               color=(.65,.65,.65),parts=tuple(panel))
root_parts=(p((0,0,.036),(.024,.008,.012)),p((0,0,.021),(.012,.006,.030)),p((0,0,.010),(.052,.008,.008)))
links=[];joints=[]
for sign,name in [(1,'left_leg'),(-1,'right_leg')]:
    links.append(dict(name=name,mass=.006,pos=(0,sign*.014,0),parts=(
        p((0,0,.015),(.008,.004,.036)),p((0,sign*.002,-.003),(.008,.010,.006)),
        p((0,0,.036),(.016,.004,.012)))))
    joints.append(dict(name=name+'_spring',body0='root',body1=name,type='prismatic',axis='Y',
                       anchor0=(0,sign*.014,0),anchor1=(0,0,0),
                       lower=-.008 if sign>0 else 0.,upper=0. if sign>0 else .008,
                       stiffness=80.,damping=1.2,max_force=2.))
CAGE=Articulated('cage',pos=(.17,-.20,TOP+.006),size=(.052,.042,.048),mass=.04,
                 color=(.12,.35,.85),parts=root_parts,links=tuple(links),joints=tuple(joints))
OBJECTS=[CAGE,PANEL]
GOAL=np.array([.28,-.07,TOP+.074])
PANEL_START=np.array([.28,0,TOP+.04])
_violation=False;_open=False;_q=np.ones(2);_recent=deque(maxlen=20);_had_compression=False
_approached=False;_passage_good=True;_crossed_compressed=False

def reset_history():
    global _violation,_open,_q,_recent,_had_compression
    global _approached,_passage_good,_crossed_compressed
    _violation=False;_open=False;_q=np.ones(2);_recent=deque(maxlen=20);_had_compression=False
    _approached=False;_passage_good=True;_crossed_compressed=False

def observe(states,t,base):
    global _violation,_open,_q,_had_compression
    global _approached,_passage_good,_crossed_compressed
    s=states['rack_coupon']
    if np.linalg.norm(s.pos-PANEL_START)>.005 or 2*math.acos(min(1.,abs(float(s.quat_xyzw[3]))))>math.radians(5):
        _violation=True
    if states['cage'].low[2]<TOP-.008 or not np.all(np.isfinite(states['cage'].pos)): _violation=True
    art=base.scene['cage']
    _q=art.data.joint_pos.torch[0].detach().cpu().numpy()
    if np.all(np.abs(_q)>.005): _had_compression=True
    # Measure the hook tops in the actual coupon frame during passage.
    panel_rotation=quat_xyzw_to_matrix(s.quat_xyzw)
    panel_root=s.pos-panel_rotation@np.array([0.,.070,0.])
    origin=base.scene.env_origins[0].detach().cpu().numpy()
    poses=art.data.body_link_pose_w.torch[0].detach().cpu().numpy()
    hook_tops=[]
    for name,sign in [('left_leg',1),('right_leg',-1)]:
        pose=poses[art.body_names.index(name)]
        point=pose[:3]-origin+quat_xyzw_to_matrix(pose[3:])@np.array([0.,sign*.002,0.])
        hook_tops.append((point-panel_root)@panel_rotation)
    hook_tops=np.asarray(hook_tops)
    near_opening=bool(np.max(np.abs(hook_tops[:,:2]))<.025)
    if near_opening and np.min(hook_tops[:,2])>.022:
        _approached=True;_passage_good=True;_crossed_compressed=False
    if near_opening and _approached:
        for index,height in enumerate(hook_tops[:,2]):
            if .016<height<.022 and abs(_q[index])<.004:
                _passage_good=False;_crossed_compressed=False
        if np.max(hook_tops[:,2])<=.016 and _passage_good:_crossed_compressed=True
    _recent.append(states['cage'].pos.copy())
    from yam_arena.constants import FINGER_JOINTS
    opened=[]
    for name in ['left_robot','right_robot']:
        r=base.scene[name];ids=r.find_joints(FINGER_JOINTS,preserve_order=True)[0]
        opened.append(float(np.max(np.abs(r.data.joint_pos.torch[0,ids].detach().cpu().numpy())))>.035)
    _open=all(opened)

def _aligned(s,tolerance=.003):
    q=np.asarray(s.quat_xyzw,dtype=float)
    norm=np.linalg.norm(q)
    if not np.isfinite(norm) or norm<1e-8: return False
    x,y,z,w=q/norm
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    yaw=abs((yaw+math.pi/2)%math.pi-math.pi/2)
    return np.max(np.abs(s.pos-GOAL))<=tolerance and 1-2*(x*x+y*y)>math.cos(math.radians(5)) and yaw<math.radians(5)

def _still():
    return len(_recent)==20 and max(np.linalg.norm(v-_recent[0]) for v in _recent)<.001

def score(states):
    if _violation or 'cage' not in states:return 0.
    s=states['cage']
    if not all(np.all(np.isfinite(v)) for v in [s.pos,s.quat_xyzw,s.low,s.high]): return 0.
    if _aligned(s) and np.all(np.abs(_q)<.0015) and _had_compression and _crossed_compressed and _still():
        return 1. if _open else .75
    if _aligned(s,.006): return .5
    if np.linalg.norm(s.pos[:2]-GOAL[:2])<.03 and s.low[2]>TOP+.02:return .25
    return 0.

def success(states):return score(states)==1.
def distance(states):return float(np.linalg.norm(states['cage'].pos-GOAL)) if not _violation else 1.

def demo():
    left=[Waypoint(1.5,(.28,.10,TOP+.13),FINGER_OPEN,'above support tab'),
          Waypoint(1.5,(.28,.10,TOP+.083),FINGER_OPEN,'lower to tab'),
          Waypoint(.8,(.28,.10,TOP+.083),FINGER_CLOSED,'steady coupon'),
          Waypoint(18.,(.28,.10,TOP+.083),FINGER_CLOSED,'hold during installation'),
          Waypoint(.8,(.28,.10,TOP+.083),FINGER_OPEN,'release coupon'),
          Waypoint(1.2,(.28,.10,TOP+.13),FINGER_OPEN,'retreat')]
    safe=(.17,-.20,TOP+.14)
    right=[Waypoint(1.5,safe,FINGER_OPEN,'wait clear'),Waypoint(3.3,safe,FINGER_OPEN,'wait for support')]
    pick=(.17,-.20,TOP+.006+.030+PAD_TIP_PAST_TCP)
    place=(.28,-.07,TOP+.056+.030+PAD_TIP_PAST_TCP+.001)
    path=pick_place_waypoints(pick,place,travel_z=TOP+.14,speed=.8)
    right+=path[:5]
    right.append(Waypoint(.8,(place[0],place[1],TOP+.14),FINGER_CLOSED,'settle above opening'))
    right+=path[5:6]
    right.append(Waypoint(.5,place,FINGER_CLOSED,'hold seated before release'))
    right+=path[6:]
    return dict(left=left,right=right)
