"""Replays short state sequences through task.step() and task.score() (the trailing 1 s window, stillness,
release and the trial-long off-table zero). Run from the task folder with the yam_arena environment."""
from pathlib import Path
import sys,copy
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import task
keys=('_known','_invalid','_clear','_held','_previous','_time','_certificate','_rest','_peak','_aligned','_window')
old={k:copy.deepcopy(getattr(task,k)) for k in keys};hand=task.observation.hand_state;support=task.observation.arm_support_force
class Sim:
 t=0.;s=None
 def states(self):return {'xgo':self.s}
 def set_target(self,*args):pass
sim=Sim();result=[]
def init():
 for k,v in old.items():setattr(task,k,copy.deepcopy(v))
 sim.t=0.;task._time=0.;task._window=task.deque();task.observation.arm_support_force=lambda _:0.
def advance(s,clear=True):
 sim.t=round(sim.t+.05,8);sim.s=s
 task.observation.hand_state=lambda _:([np.array([[3.,3.,3.]])] if clear else [s.pos[None,:]],0.)
 task.step(sim);return task.score({'xgo':s})
def standing(q,dx=0.):
 values=dict(task.START);values.update(q)
 ss=task.XGO.state(joints=values,pos=(.3,0,task.TOP+.1));z=task.lowest(task.poses(ss))-task.TOP
 return task.XGO.state(joints=values,pos=(.3+dx,0,task.TOP+.1-z))
try:
 good=standing(task.GOAL);p=np.array(good.pos)
 init();values=[advance(good) for _ in range(21)];result.append(('full score only after an entire 1 s',values[-1]==1. and not any(values[:-1])))
 init();values=[advance(standing(task.GOAL,i*.0003)) for i in range(35)];result.append(('10 mm drift never counts as still',not any(values)))
 # Still means no point of any link moves more than 2 mm within the scored second.
 init();values=[advance(standing(task.GOAL,i*.000125)) for i in range(60)];result.append(('2.5 mm creep per second never counts as still',not any(values)))
 init();values=[advance(standing(task.GOAL,i*.00005)) for i in range(30)];result.append(('1 mm creep per second still counts',values[-1]==1.))
 init();values=[advance(good,clear=i!=11) for i in range(30)];result.append(('a touch during the second restarts it',not any(values)))
 init();advance(good);off=task.XGO.state(joints=dict(task.START)|task.GOAL,pos=p+[-.6,0,0]);advance(off)
 values=[advance(good) for _ in range(25)];result.append(('off the table and back stays zero',not any(values)))
 init();advance(good);sunk=task.XGO.state(joints=dict(task.START)|task.GOAL,pos=p+[0,0,-.012]);advance(sunk)
 values=[advance(good) for _ in range(25)];result.append(('12 mm below the tabletop once stays zero',not any(values)))
 # An earlier wrong angle ages out: a leg that is fixed and then held for a fresh second counts.
 init();bad=standing(task.GOAL|{'FL_shank_joint':task.QF+np.deg2rad(7.4)});fixed=standing(task.GOAL|{'FL_shank_joint':task.QF+np.deg2rad(6.6)})
 for _ in range(21):advance(bad)
 values=[advance(fixed) for _ in range(62)]
 result.append(('7.4 to 6.6 deg recovery earns full after a fresh 1 s',values[-1]==1.))
 init();task.observation.arm_support_force=lambda _:9.
 values=[advance(good) for _ in range(25)]
 result.append(('an arm link resting on the robot earns zero',not any(values)))
finally:
 task.observation.hand_state=hand;task.observation.arm_support_force=support
 for k,v in old.items():setattr(task,k,v)
for label,passed in result:print(('ok' if passed else 'FAIL'),label)
raise SystemExit(sum(not passed for _,passed in result))
