"""Remove both transport keys to their return trays before extracting the motion carrier.

The two keys lock the carrier to its base. Overhanging rails prevent lifting the carrier out.
Pull the carrier out through the front opening after both keys are clear. Set it flat on the
white recovery pad and release all parts. Do not move the carrier more than 5 mm while a key
is still engaged. There is no powered motion stage or live electrical equipment.
"""
import math
import numpy as np
from yam_arena.task_file import Box,TABLE_TOP
from yam_arena.demo import Waypoint,pick_place_waypoints
from yam_arena.constants import FINGER_OPEN,FINGER_CLOSED,PAD_TIP_PAST_TCP
from yam_industrial import Compound

SCENE='station'
TOP=TABLE_TOP[SCENE]
SUCCESS_HOLD_S=1.
SCORE_OVER='end'

def p(pos,size): return dict(pos=pos,size=size)

def holed_plate(thickness):
    parts=[]
    # Two square bores at y plus or minus 40 mm. Every remaining solid is a cuboid.
    for y,width in [(-.056,.018),(0.,.066),(.056,.018)]:
        parts.append(p((0,y,0),(.08,width,thickness)))
    for y in [-.04,.04]:
        for x in [-.0235,.0235]:
            parts.append(p((x,y,0),(.033,.014,thickness)))
    return parts

carrier_parts=holed_plate(.02)
carrier_parts.append(p((-.02,0,.0275),(.025,.025,.035)))
for item in carrier_parts: item['pos']=(item['pos'][0],item['pos'][1],item['pos'][2]-.0175)
CARRIER=Compound('carrier',pos=(.30,0,TOP+.0475),size=(.08,.13,.055),mass=.18,
                 color=(.25,.65,.35),parts=tuple(carrier_parts),grasp_offset=(-.02,0,.01))
base_parts=holed_plate(.018)
for item in base_parts: item['pos']=(item['pos'][0],item['pos'][1],.001)
base_parts.append(p((0,0,-.009),(.08,.13,.002)))
BASE=Compound('stage_base',pos=(.30,0,TOP+.01),size=(.08,.13,.02),mass=None,
              color=(.55,.55,.55),parts=tuple(base_parts))

def key(name,y,color):
    return Compound(name,pos=(.30,y,TOP+.0375),size=(.026,.022,.071),mass=.025,color=color,
                    parts=(p((0,0,-.009),(.008,.008,.053)),p((0,0,.0265),(.026,.022,.018))),
                    grasp_offset=(0,0,.0265))
KL=key('blue_key',.04,(.1,.35,.85))
KR=key('orange_key',-.04,(.9,.4,.1))

def nest(name,xy):
    items=[p((0,0,-.009),(.16,.11,.004))]
    for sign in [-1,1]:
        items.append(p((sign*.078,0,.002),(.004,.11,.018)))
        items.append(p((0,sign*.053,.002),(.152,.004,.018)))
    return Compound(name,pos=(*xy,TOP+.011),size=(.16,.11,.022),mass=None,
                    color=(.85,.85,.85),parts=tuple(items))

NL=nest('left_return',(.19,.20))
NR=nest('right_return',(.19,-.20))
RAILS=[]
for sign in [-1,1]:
    RAILS.append(Box('channel_wall_'+('left' if sign>0 else 'right'),
                     pos=(.30,sign*.075,TOP+.022),size=(.13,.012,.044)))
    RAILS.append(Box('retaining_lip_'+('left' if sign>0 else 'right'),
                     pos=(.30,sign*.062,TOP+.045),size=(.13,.026,.006)))
PAD=Box('recovery_pad',pos=(.17,0,TOP+.0005),size=(.09,.15,.001),color=(.9,.9,.9))
OBJECTS=[CARRIER,BASE,KL,KR,NL,NR,PAD,*RAILS]
GOALS={KL.name:np.array([.17,.20,TOP+.0405]),KR.name:np.array([.17,-.20,TOP+.0405]),
       CARRIER.name:np.array([.17,0,TOP+.0285])}
_cleared=set()
_violation=False
_open=False
_stored_once=False

def reset_history():
    global _cleared,_violation,_open,_stored_once
    _cleared=set(); _violation=False; _open=False; _stored_once=False

def observe(states,t,base):
    global _violation,_open,_stored_once
    if any(o.name not in states for o in OBJECTS):
        _violation=True; return
    for key in [KL,KR]:
        s=states[key.name]
        if s.low[2]>TOP+.041: _cleared.add(key.name)
    if all(_settled(states,n) for n in ['blue_key','orange_key']):
        _stored_once=True
    moving=np.linalg.norm(states['carrier'].pos-np.array(CARRIER.pos))>.005
    engaged=any(states[k.name].low[2]<=TOP+.041 and np.linalg.norm(states[k.name].pos[:2]-np.array(k.pos[:2]))<.02 for k in [KL,KR])
    if moving and (not _stored_once or engaged):
        _violation=True
    for name in ['blue_key','orange_key','carrier']:
        if states[name].low[2]<TOP-.008 or not np.all(np.isfinite(states[name].pos)):
            _violation=True
    try:
        from yam_arena.constants import FINGER_JOINTS
        opened=[]
        for name in ['left_robot','right_robot']:
            r=base.scene[name]; ids=r.find_joints(FINGER_JOINTS,preserve_order=True)[0]
            opened.append(float(np.max(np.abs(r.data.joint_pos.torch[0,ids].detach().cpu().numpy())))>.035)
        _open=all(opened)
    except (AttributeError,KeyError,IndexError): _open=False

def _settled(states,name):
    if name not in states: return False
    s=states[name]
    if not all(np.all(np.isfinite(v)) for v in [s.pos,s.quat_xyzw,s.lin_vel,s.ang_vel]): return False
    if name in ['blue_key','orange_key']:
        y=.20 if name=='blue_key' else -.20
        inside=bool(s.low[0]>=.114 and s.high[0]<=.266 and s.low[1]>=y-.051 and s.high[1]<=y+.051)
        return inside and TOP+.003<=s.low[2]<=TOP+.007 and s.high[2]<=TOP+.10 and s.resting(.01,.1)
    norm=np.linalg.norm(s.quat_xyzw)
    if norm<1e-8: return False
    q=s.quat_xyzw/norm
    upright=1-2*(q[0]**2+q[1]**2)>math.cos(math.radians(8))
    yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
    aligned=abs((yaw+math.pi/2)%math.pi-math.pi/2)<math.radians(8)
    supported=abs(s.low[2]-(TOP+.001))<=.003
    return upright and aligned and supported and s.resting(.01,.1) and np.max(np.abs(s.pos-GOALS[name]))<=.006

def score(states):
    if _violation: return 0.
    value=.25*sum(_settled(states,n) for n in ['blue_key','orange_key'])
    value+=.5*(_settled(states,'carrier') and len(_cleared)==2 and _stored_once)
    return float(value if _open else min(value,.75))

def success(states): return score(states)==1.

def distance(states):
    if _violation or any(n not in states for n in GOALS): return 1.
    return float(np.mean([np.linalg.norm(states[n].pos-g) for n,g in GOALS.items()]))

def demo():
    left_pick=np.array(KL.pos)+[0,0,.0265+PAD_TIP_PAST_TCP]
    left_goal=GOALS[KL.name]+[0,0,.0265+PAD_TIP_PAST_TCP+.001]
    left=pick_place_waypoints(tuple(left_pick),tuple(left_goal),travel_z=TOP+.13)
    pause=sum(w.duration for w in left)+1.
    safe=(.17,-.20,TOP+.13)
    right=[Waypoint(1.5,safe,FINGER_OPEN,'park clear'),Waypoint(pause-1.5,safe,FINGER_OPEN,'wait for first key')]
    right_pick=np.array(KR.pos)+[0,0,.0265+PAD_TIP_PAST_TCP]
    right_goal=GOALS[KR.name]+[0,0,.0265+PAD_TIP_PAST_TCP+.001]
    right+=pick_place_waypoints(tuple(right_pick),tuple(right_goal),travel_z=TOP+.13)
    left.append(Waypoint(sum(w.duration for w in right)+1-sum(w.duration for w in left),left[-1].pos,FINGER_OPEN,'wait for second key'))
    z=TOP+.01+.0475+PAD_TIP_PAST_TCP
    left += [Waypoint(1.5,(.28,0,TOP+.13),FINGER_OPEN,'over carrier handle'),
             Waypoint(1.5,(.28,0,z),FINGER_OPEN,'grasp handle'),
             Waypoint(.8,(.28,0,z),FINGER_CLOSED,'close'),
             Waypoint(3.,(.15,0,z),FINGER_CLOSED,'slide through front opening'),
             Waypoint(1.2,(.15,0,TOP+.13),FINGER_CLOSED,'lift clear of channel'),
             Waypoint(1.5,(.15,0,TOP+.0285+.01+PAD_TIP_PAST_TCP+.001),FINGER_CLOSED,'lower onto recovery pad'),
             Waypoint(.8,(.15,0,TOP+.0285+.01+PAD_TIP_PAST_TCP+.001),FINGER_OPEN,'release'),
             Waypoint(1.5,(.15,0,TOP+.13),FINGER_OPEN,'retreat')]
    return dict(left=left,right=right)
