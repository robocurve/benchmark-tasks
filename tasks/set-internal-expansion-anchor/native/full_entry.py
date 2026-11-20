"""Scripted reference: set the anchor with the punch and hammer, then put both tools back.

It replays the nominal joint targets of geometry/full_plan.json (built by plan_reference.py) through the stock
14-D joint action and adds three kinds of feedback from actual poses and contacts:

- before each blow the left hand centres the punch on the anchor and the right hand levels the hammer face over
  the punch top (the arms sag a little under load);
- blows stop as soon as the punch shoulder is measured flush on the anchor, at most plan_reference.STRIKES;
- before the hammer is laid down it is brought back over its cradle.

Nothing here writes an object pose, applies a force, or changes the robot. Set ANCHOR_REFERENCE to play a
wrong-path variant: `no_strike` (every motion except the blows) or `leave_punch` (anchor set and hammer returned,
punch left standing in the anchor). Run this file directly for the offline self-test.
"""
from pathlib import Path
import os,sys,json,math
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
import task_runtime as rt
from task_runtime import OBJECTS,SCENE,SCORE_OVER,SUCCESS_HOLD_S,CONTROL_HZ,score,success
from yam_arena import ik
from yam_arena.demo import ArmPlan,Waypoint
from yam_arena.constants import READY,FINGER_OPEN
from material_grasp import MaterialGraspGuard,GraspFailure
from punch_alignment import PunchAlignment,collision_guard
from tool_servo import ToolServo
import plan_reference as ref

VARIANT=os.environ.get('ANCHOR_REFERENCE','full')
SETTLE=20            # controls held at the ready pose before the plan starts
EXTRA_MAX=40         # most extra controls an alignment phase may take
SEATED_GAP=.00008    # shoulder gap (m) at or below which the reference stops striking

def effective_plan(rows,variant):
 if variant=='full':return rows
 if variant=='no_strike':return[r for r in rows if not r['phase'].startswith(('align_','strike_','relift_'))]
 if variant=='leave_punch':
  cut=next(i for i,r in enumerate(rows)if r['phase']=='punch_withdraw');out=list(rows[:cut]);last=out[-1]
  back=np.array(next(r for r in reversed(rows[:cut])if r['phase']=='punch_hover')['left_q']);start=np.array(last['left_q'])
  def add(phase,q,g):out.append(dict(last,control=len(out),phase=phase,left_q=list(map(float,q)),left_grip=float(g)))
  for i in range(1,26):add('punch_let_go',start,i/25)
  for i in range(1,41):add('left_back_off',start+(back-start)*ik.smoothstep(i/40),1.)
  for _ in range(40):add('released_terminal_hold',back,1.)
  return out
 raise ValueError('Unknown ANCHOR_REFERENCE '+variant)

_PLAN=json.load(open(rt.GROOT/'full_plan.json'))
if _PLAN['source_geometry_sha256']!=rt.GSHA:raise RuntimeError('full_plan.json was built for another geometry.json; rerun plan_reference.py')
PLAN=effective_plan(_PLAN['rows'],VARIANT)
FIRST={};COUNT={}
for _i,_r in enumerate(PLAN):FIRST.setdefault(_r['phase'],_i);COUNT[_r['phase']]=COUNT.get(_r['phase'],0)+1
LEFT_FREE={'initial_settle','punch_hover','punch_approach','punch_close','punch_release','punch_retreat','punch_let_go','left_back_off','released_terminal_hold'}
RIGHT_HOLDS=('hammer_lift','hammer_transit','align_','strike_','relift_','seat_check','hammer_return_high','hammer_place_align','hammer_return')
HAMMER=next(p for p in rt.G['parts']if p['name']=='hammer')
PLACE_POINT=np.array(HAMMER['pos'])+[ref.GRASP_X,0,ref.LIFT];PLACE_R=Rotation.from_quat(HAMMER['quat_xyzw']).as_matrix()

GRASP_GUARD=MaterialGraspGuard();ALIGNER=PunchAlignment(rt.G)
STRIKE_SERVO=ToolServo(rt.G['robot_bases_m']['right'],'hammer',ref.FACE)
PLACE_SERVO=ToolServo(rt.G['robot_bases_m']['right'],'hammer',[ref.GRASP_X,0,0])
LEFT_DELTA=np.zeros(6);PTR=-1;LAST=-1;EXTRA=0;FAILED=None;HOLDCOUNT=0;STOP=False;PHASE='initial_settle';BLOWS=0
LASTCMD={s:np.r_[READY,1.]for s in ['left','right']}
rt.PHASE_PROVIDER=lambda:PHASE

def demo():return{s:[Waypoint(100.,(.3,0,.9),FINGER_OPEN,'anchorref_'+s)]for s in ['left','right']}
def pads_touch(side,row):
 keys=[side+'_tip_left',side+'_tip_right'];pairs=row['contacts']['pairs']
 return all(k in pairs and'counts'in pairs[k]and int(np.array(pairs[k]['counts']).sum())>0 and sum(abs(x['normal_force_N'])for x in pairs[k].get('entries',[]))>.05 for k in keys)
def body(row,name):
 a=np.array(row['rigid_bodies'][name]['pose_xyzw'],float);return a[:3],Rotation.from_quat(a[3:]).as_matrix()
def shoulder_gap(row):
 """Actual height of the punch shoulder above the anchor top, along the anchor axis (m)."""
 rp,rR=body(row,'anchor_ring');pp,pR=body(row,'punch')
 return float(np.dot(pp+pR[:,2]*.0414875-(rp+rR[:,2]*.0396875),rR[:,2]))
def punch_top(row):
 pp,pR=body(row,'punch');return pp+pR[:,2]*ref.PUNCH_TOP
def log(name,obj):
 rt.TRACE.mkdir(parents=True,exist_ok=True)
 with(rt.TRACE/name).open('a')as f:f.write(json.dumps(obj)+'\n')
def fade(phase,i):
 u=min(1.,(i-FIRST[phase]+1)/COUNT[phase]);return 1.-u*u*(3.-2.*u)

def settled(phase):
 """(arrived, fatal reason): whether an alignment phase's feedback has arrived; a coarse miss lets the plan go on."""
 if phase.startswith('align_'):
  if ALIGNER.streak>=5 and STRIKE_SERVO.streak>=3:return True,None
  coarse=(ALIGNER.latest.get('lateral_m',1.)<.001 and np.linalg.norm(STRIKE_SERVO.latest.get('point_error_m',[1.]))<.005
          and np.linalg.norm(STRIKE_SERVO.latest.get('rotation_error_rad',[1.]))<np.deg2rad(4))
  return False,None if coarse else'Alignment before the blow did not settle'
 if phase=='seat_check':return ALIGNER.streak>=5,None
 if phase=='hammer_place_align':return PLACE_SERVO.streak>=3,None
 return True,None

def decide(t):
 global PTR,LAST,EXTRA,FAILED,HOLDCOUNT,STOP,PHASE,LEFT_DELTA,BLOWS
 index=max(0,int(math.floor((t+1e-8)*20)))
 if index==LAST:return
 LAST=index;actual=rt.LAST_ACTUAL
 if index<SETTLE:
  log('controller.jsonl',dict(control=index,phase=PHASE,failed=FAILED,stop_requested=STOP,plan_index=PTR));return
 if not FAILED:
  nxt=PTR+1;cur=PLAN[PTR]['phase']if PTR>=0 else None
  if cur is not None and nxt<len(PLAN)and PLAN[nxt]['phase']!=cur:
   ok,fatal=settled(cur)
   if not ok and EXTRA<EXTRA_MAX:EXTRA+=1;nxt=PTR
   elif not ok and fatal:FAILED=fatal
   if nxt!=PTR and cur.startswith('relift_')and actual is not None and shoulder_gap(actual)<=SEATED_GAP:nxt=FIRST['seat_check']
  if nxt>=len(PLAN):STOP=True;nxt=len(PLAN)-1
  if not FAILED:
   if PLAN[nxt]['phase']!=cur:
    EXTRA=0
    if PLAN[nxt]['phase'].startswith('strike_')and not PLAN[nxt]['phase'].startswith('strike_settle_'):BLOWS+=1
   PTR=nxt
 row=PLAN[max(PTR,0)]
 if not FAILED:
  PHASE=row['phase']
  if actual is not None:
   if not all(np.isfinite(v['pose_xyzw']).all()for v in actual['rigid_bodies'].values()):FAILED='Nonfiniteactualbody'
   if any(not np.isfinite(v['joint_positions']).all()for v in actual['robots'].values()):FAILED='Nonfiniteactualrobot'
   if not FAILED:
    try:
     measurements=[]
     if PHASE not in LEFT_FREE:measurements.append(GRASP_GUARD.observe(actual,'left','punch',pads_touch('left',actual)))
     if PHASE.startswith(RIGHT_HOLDS):measurements.append(GRASP_GUARD.observe(actual,'right','hammer',pads_touch('right',actual)))
     if measurements:log('material-grasp.jsonl',dict(control=index,measurements=measurements))
    except (ValueError,GraspFailure)as e:
     FAILED=type(e).__name__+':'+str(e)
     if hasattr(e,'measurement'):rt.save('GRASP-GUARD-FAILURE.json',e.measurement)
 if FAILED:
  PHASE='guard_hold';HOLDCOUNT+=1;STOP=STOP or HOLDCOUNT>=20
 else:
  prior={s:v.copy()for s,v in LASTCMD.items()}
  try:
   left=np.array(row['left_q'],float);right=np.array(row['right_q'],float)
   if PHASE.startswith('align_')or PHASE=='seat_check':
    if actual is None:raise ValueError('Missingactualalignmentreadback')
    corrected,measurement=ALIGNER.update(actual,row['left_q'],PHASE);LEFT_DELTA=corrected-left
    log('punch-alignment.jsonl',measurement)
   if PHASE.startswith('align_'):
    _,measurement=STRIKE_SERVO.update(actual,right,punch_top(actual)+[0,0,ref.STROKE],ref.TOOL)
    log('hammer-alignment.jsonl',dict(measurement,phase=PHASE))
   if PHASE=='hammer_place_align':
    _,measurement=PLACE_SERVO.update(actual,right,PLACE_POINT,PLACE_R)
    log('hammer-alignment.jsonl',dict(measurement,phase=PHASE))
   wl=fade(PHASE,PTR)if PHASE in('punch_withdraw','left_back_off')else 0. if PHASE in('punch_stow_high','punch_lower_to_vrest','punch_release','punch_retreat','released_terminal_hold')else 1.
   past_strikes=PTR>=FIRST['hammer_return_high']
   wr=fade(PHASE,PTR)if PHASE=='hammer_return_high'else 0. if past_strikes else 1.
   left=left+wl*LEFT_DELTA;right=right+wr*STRIKE_SERVO.delta+(PLACE_SERVO.delta if past_strikes else 0.)
   if actual is not None:
    if np.linalg.norm(wl*LEFT_DELTA)>1e-10:collision_guard(actual,left,rt.G,'left','punch')
    if np.linalg.norm(right-np.array(row['right_q']))>1e-10:collision_guard(actual,right,rt.G,'right','hammer')
   LASTCMD['left']=np.r_[left,row['left_grip']];LASTCMD['right']=np.r_[right,row['right_grip']]
  except ValueError as e:
   for s in LASTCMD:LASTCMD[s]=prior[s]
   FAILED=type(e).__name__+':'+str(e);PHASE='guard_hold';HOLDCOUNT=1
 log('controller.jsonl',dict(control=index,phase=PHASE,failed=FAILED,stop_requested=STOP,plan_index=PTR,blows=BLOWS,
     shoulder_gap_m=None if actual is None else shoulder_gap(actual)))
old=ArmPlan.action7

def action(plan,t):
 label=plan.waypoints[0].label if plan.waypoints else''
 if label.startswith('anchorref_'):
  decide(t);side=label.rsplit('_',1)[1];return LASTCMD[side],PHASE
 return old(plan,t)
ArmPlan.action7=action

def policy_stop(mod):
 cls=mod.YamScriptedPolicy
 if getattr(cls,'_anchorref_stop',False):return
 old=cls.act
 def act(self,observation):
  out=old(self,observation)
  if STOP:
   for a in out.actions:a.meta.update(request_stop=True,stop_reason='reference_hold'if FAILED else'reference_plan_complete',stop_detail=FAILED or'The reference played its whole plan. The score comes from the outcome scorer only.')
  return out
 cls.act=act;cls._anchorref_stop=True
rt.ng.pk._Finder.targets['yam_arena.scripted']=policy_stop
if'yam_arena.scripted'in sys.modules:policy_stop(sys.modules['yam_arena.scripted'])

if __name__=='__main__':
 import selftest
 selftest.run()
