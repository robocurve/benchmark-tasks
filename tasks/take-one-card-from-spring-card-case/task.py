"""Take a single PVC loyalty card out of the spring-backed friction case."""
from pathlib import Path
import sys,json,os
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
import geometry as g
import cardkit as kit
from scoring import Monitor
from yam_arena.task_file import Box
SCENE='station';SCORE_OVER='best';SUCCESS_HOLD_S=1.0
INSTRUCTION='Take one membership card out of the blue card case and leave it on the cream mat. Leave all the other cards inside the case and let go.'
OBJECTS=[kit.Item('card_case',pos=tuple(map(float,g.ORIGIN)),size=(.100,.064,.016),mass=float(sum(s.mass for s in g.case_shapes())),kind='case')]
for name,p in g.initial_cards().items():OBJECTS.append(kit.Item(name,pos=tuple(map(float,p)),size=tuple(map(float,g.CARD_SIZE)),mass=float(g.CARD_MASS)))
# Ordinary desktop riser; no clamp, hidden anchor or fixed joint on the case.
OBJECTS.extend([Box('desk_riser',pos=(float(g.ORIGIN[0]),float(g.ORIGIN[1]),float(g.TABLE+.025)),size=(.050,.038,.050),mass=None,color=(.4,.32,.24)),
 Box('output_mat',pos=tuple(map(float,g.TRAY)),size=(.140,.100,.002),mass=None,color=(.9,.85,.7))])
MONITOR=Monitor();OBSERVATIONS=[]
def measured(s):return {k:np.asarray(getattr(s,k)).tolist()for k in ['pos','quat_xyzw','lin_vel','ang_vel']}
def clearance(env,states):
 # All robot collider vertices against oriented case bounding box and card OBBs.
 # Only a release observer: all actual native robot collision shapes remain active.
 from yam_arena import analysis
 from yam_arena.constants import ARM_JOINTS
 from yam_arena import station
 scene=env.unwrapped.scene;origin=kit.arr(scene.env_origins)[0]
 objects=[(states['card_case'],np.array([.100,.064,.032]),np.array([0,0,.008]))]
 objects.extend((states[n],g.CARD_SIZE,np.zeros(3))for n in g.initial_cards())
 minimum=1.
 for side in ['left','right']:
  a=scene[side+'_robot'];poses=kit.arr(a.data.body_link_pose_w)[0]
  byname=dict(zip(a.body_names,poses))
  for name,hs in analysis.link_hulls().items():
   if name not in byname:raise RuntimeError('unbound robot body '+name)
   p=byname[name];R=g.rotation(p[3:7])
   for h in hs:
    points=h.vertices@R.T+p[:3]-origin
    for s,size,offset in objects:
     O=g.rotation(s.quat_xyzw);center=np.array(s.pos)+O@offset
     local=(points-center)@O
     delta=np.maximum(np.maximum(local.min(0)-size/2,-size/2-local.max(0)),0.)
     minimum=min(minimum,float(np.linalg.norm(delta)))
 return minimum

def tick(env):
 from yam_arena.scene import object_states
 base=env.unwrapped;states=object_states(env,['card_case',*g.initial_cards()])
 counter=int(base._sim_step_counter);time=counter*float(base.physics_dt)
 row=dict(time_s=time,sample_id=counter,case=measured(states['card_case']),cards={n:measured(states[n])for n in g.initial_cards()},robot_clearance_m=clearance(env,states))
 res=MONITOR.update(row);OBSERVATIONS.append(row)
 if counter%50==0 or res.get('score',0)>0 or res.get('presented') or res.get('output'):
  c4=[round(float(x),4) for x in states['card_4'].pos]
  print('STEP',counter,'score=',res.get('score'),'c4=',c4,'in=',len(res.get('inside',[])),'out=',len(res.get('output',[])),'cand=',res.get('full_candidate'),'held=',round(float(res.get('held_s',0)),2),flush=True)
 trace=os.environ.get('ROBOCURVE_TRACE_DIR')
 if trace:
  p=Path(trace);p.mkdir(parents=True,exist_ok=True)
  with (p/'card-observations.jsonl').open('a')as f:f.write(json.dumps(row)+'\n')
def reset(env):
 global MONITOR,OBSERVATIONS
 MONITOR=Monitor();OBSERVATIONS=[]
 trace=os.environ.get('ROBOCURVE_TRACE_DIR')
 if trace:
  p=Path(trace);p.mkdir(parents=True,exist_ok=True);(p/'card-observations.jsonl').write_text('')
kit.STEP.append(tick);kit.RESET.append(reset)
def score(objects):return float(MONITOR.last['score'])
def success(objects):return bool(MONITOR.last['success'])
def demo():
 from reference import run
 yield from run()
