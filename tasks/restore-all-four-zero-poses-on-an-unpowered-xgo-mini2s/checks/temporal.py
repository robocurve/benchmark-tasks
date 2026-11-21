from pathlib import Path
import sys,copy,types
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import task
keys=('_known','_invalid','_clear','_held','_previous','_prev_eligible','_time','_certificate','_rest','_peak','_anchor','_aligned','_window','_supported')
old={k:copy.deepcopy(getattr(task,k)) for k in keys};hand=task.observation.hand_state;support=task.observation.arm_support_force;feet=task.observation.foot_supports
p=np.array(task.XGO.pos)-[0,0,.01292]
class Sim:
 t=0.;s=None
 def states(self):return {'xgo':self.s}
 def set_target(self,*args):pass
sim=Sim();result=[]
def init():
 for k,v in old.items():setattr(task,k,copy.deepcopy(v))
 sim.t=0.;task._time=0.;task._window=task.deque();task.observation.arm_support_force=lambda _:0.;task.observation.foot_supports=lambda _:[True]*4
def advance(s,clear=True):
 sim.t=round(sim.t+.05,8);sim.s=s
 task.observation.hand_state=lambda _:([np.array([[3.,3.,3.]])] if clear else [s.pos[None,:]],0.)
 task.step(sim);return task.score({'xgo':s})
try:
 init();ss=task.XGO.state(joints=task.GOAL,pos=(.3,0,task.TOP));p=np.array((.3,0,task.TOP-min(task.measure(ss)['feet'])));good=task.XGO.state(joints=task.GOAL,pos=p)
 values=[advance(good) for _ in range(21)];result.append(('full score after entire1s',values[-1]==1. and not any(values[:-1])))
 init();values=[advance(task.XGO.state(joints=task.GOAL,pos=p+[i*.0003,0,0])) for i in range(35)];result.append(('cumulative10mm drift cannot pass',not any(values)))
 # Still means no point of any link moves more than 2 mm within the scored second.
 init();values=[advance(task.XGO.state(joints=task.GOAL,pos=p+[i*.000125,0,0])) for i in range(60)];result.append(('2.5mm creep per second never counts as still',not any(values)))
 init();values=[advance(task.XGO.state(joints=task.GOAL,pos=p+[i*.00005,0,0])) for i in range(30)];result.append(('1mm creep per second still counts',values[-1]==1.))
 init();values=[advance(good,clear=i!=11) for i in range(30)];result.append(('interrupted release resets interval',not any(values)))
 init();advance(good);off=task.XGO.state(joints=task.GOAL,pos=p+[-.5,0,0]);advance(off)
 values=[advance(good) for _ in range(25)];result.append(('offtable and returned remainszero',not any(values)))
 init();values=[advance(good) for _ in range(15)];advance(task.XGO.state(joints=task.GOAL,pos=p+[0,0,.005]));values=[advance(good) for _ in range(15)];result.append(('footlift resets hold',not any(values)))
 # Reproduced independent-review counterexample: old failed angles must age out.
 def ground(angle):
  q={leg+'_calf_joint':np.deg2rad(angle) for leg in task.LEGS}
  ss=task.XGO.state(joints=q,pos=(.3,0,task.TOP));z=-min(task.measure(ss)['feet'])
  return task.XGO.state(joints=q,pos=(.3,0,task.TOP+z))
 init();bad=ground(25.2);recovered=ground(24.8)
 for _ in range(21):advance(bad)
 values=[advance(recovered) for _ in range(62)]
 result.append(('10.2to9.8 recovery earns full after fresh1s',values[-1]==1.))
 init();task.observation.arm_support_force=lambda _:9.
 values=[advance(good) for _ in range(25)]
 result.append(('forearm support earnszero',not any(values)))
finally:
 task.observation.hand_state=hand;task.observation.arm_support_force=support;task.observation.foot_supports=feet
 for k,v in old.items():setattr(task,k,v)
for label,passed in result:print(('ok' if passed else 'FAIL'),label)
raise SystemExit(sum(not passed for _,passed in result))
