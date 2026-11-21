"""Adversarial twelve-axis, released hold and fail-closed scoring checks."""
import copy,math,sys
import numpy as np
import task
CASES=[('unobserved reset',{},0.,False)]
def run():
 names=('_known','_invalid','_clear','_held','_certificate','_aligned','_supported')
 old={n:copy.deepcopy(getattr(task,n)) for n in names};cases=[]
 p=np.array(task.XGO.pos)-[0,0,.01292]
 def state(q=None):
  values=dict(task.GOAL);values.update(q or {})
  ss=task.XGO.state(joints=values,pos=(.3,0,task.TOP));zz=-min(task.measure(ss)['feet'])
  return task.XGO.state(joints=values,pos=(.3,0,task.TOP+zz))
 def grade(s,known=True,clear=True,held=1.,invalid=False):
  task._known=known;task._supported=True;task._clear=clear;task._held=held;task._invalid=invalid;task._certificate=task.stamp(s);task._aligned=task.measure(s)['aligned']
  return task.score({'xgo':s})
 try:
  good=state();cases.append(('full twelve-axis endpoint',grade(good)==1.))
  for n in range(5):
   q=dict(task.GOAL)
   for leg in task.LEGS[n:]:
    q[leg+'_thigh_joint']=math.radians(20);lo,hi=-1.13,1.04
    for _ in range(40):
     mid=(lo+hi)/2;q[leg+'_calf_joint']=mid
     ss=task.XGO.state(joints=q,pos=good.pos);f=task.measure(ss)['feet'][task.LEGS.index(leg)]
     if f>0:lo=mid
     else:hi=mid
   ss=task.XGO.state(joints=q,pos=good.pos)
   cases.append((f'{n} complete chains, all feet down',abs(grade(ss)-n/4)<1e-9))
  for held in (0.,.05,.95,.999):cases.append((f'incomplete released hold {held}',grade(good,held=held)==0.))
  cases += [('unknown hands',grade(good,known=False)==0.),('held endpoint',grade(good,clear=False)==0.),('trial violation retained',grade(good,invalid=True)==0.)]
  cases.append(('feet hovering',grade(task.XGO.state(joints=task.GOAL,pos=np.array(good.pos)+[0,0,.004]))==0.))
  cases.append(('hidden one-axis error',grade(state({'fl_hip_joint':math.radians(11)}))==0.))
  # The XGO's own arm cannot reach the table from a scoring pose (lowest about 43 mm); the scorer still refuses a propped body.
  prop=dict(task.GOAL);prop.update(arm_thigh_joint=2.87,arm_calf_joint=-1.04)
  low=task.XGO.state(joints=prop,pos=np.array(good.pos)-[0,0,.050])
  cases.append(('own arm on the table',task.measure(low)['arm']<.010 and grade(low)==0.))
  grade(good);task._certificate=();cases.append(('stale certificate',task.score({'xgo':good})==0.))
  cases.append(('missing data',task.score({})==0.))
  cases.append(('empty hand cloud',not task.released([],good)))
  cases.append(('NaN hand cloud',not task.released([np.full((3,3),np.nan)],good)))
  # A hand face 0.5mm from a large torso face must fail without nearby vertices.
  _,c,r,size=task.XGO.world_boxes(task.poses(good))[0]
  face=c+r@np.array((0,size[1]/2+.0005,0));cases.append(('hand face0.5mm from torso',not task.released([face[None,:]],good)))
 finally:
  for n,v in old.items():setattr(task,n,v)
 for label,ok in cases:print(('ok' if ok else 'FAIL'),label)
 return sum(not ok for _,ok in cases)
if __name__=='__main__':sys.exit(run())
