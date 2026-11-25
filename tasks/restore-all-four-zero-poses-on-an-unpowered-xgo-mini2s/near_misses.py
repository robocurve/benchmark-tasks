"""Adversarial twelve-axis, released-hold and fail-closed scoring checks for the passive XGO-lite: both rear legs
must still be set; each front leg set earns half."""
import copy,math,sys
import numpy as np
import task
CASES=[('unobserved reset',{},0.,False)]
def run():
 names=('_known','_invalid','_clear','_held','_certificate','_aligned')
 old={n:copy.deepcopy(getattr(task,n)) for n in names};cases=[]
 def state(q=None,dz=0.,quat=None):
  values=dict(task.START);values.update(task.GOAL);values.update(q or {})
  ss=task.XGO.state(joints=values,pos=(.3,0,task.TOP+.1),quat_xyzw=quat);zz=task.lowest(task.poses(ss))-task.TOP
  return task.XGO.state(joints=values,pos=(.3,0,task.TOP+.1-zz+dz),quat_xyzw=quat)
 def grade(s,known=True,clear=True,held=1.,invalid=False):
  task._known=known;task._clear=clear;task._held=held;task._invalid=invalid;task._certificate=task.stamp(s);task._aligned=task.measure(s)['aligned']
  return task.score({'xgo':s})
 try:
  good=state();cases.append(('all four legs in the posture, standing, released and still',grade(good)==1.))
  cases.append(('front left lower leg still 20 deg toe-up: half',grade(state({'FL_shank_joint':task.QF+math.radians(20)}))==.5))
  cases.append(('front right lower leg still 20 deg toe-up: half',grade(state({'FR_shank_joint':task.QF+math.radians(20)}))==.5))
  cases.append(('both front lower legs still toe-up: zero',grade(state({'FL_shank_joint':task.QF+math.radians(20),'FR_shank_joint':task.QF+math.radians(20)}))==0.))
  cases.append(('both front legs set but a rear lower leg knocked 8 deg toe-up: zero',grade(state({'RR_shank_joint':task.QF+math.radians(8)}))==0.))
  cases.append(('one front leg set, a rear thigh knocked 8 deg: zero',grade(state({'FL_shank_joint':task.QF+math.radians(20),'RL_thigh_joint':math.radians(8)}))==0.))
  cases.append(('start pose',grade(state(dict(task.START)))==0.))
  for held in (0.,.05,.95,.999):cases.append((f'released for only {held} s',grade(good,held=held)==0.))
  cases += [('unknown hands',grade(good,known=False)==0.),('a hand still holding or touching',grade(good,clear=False)==0.),('off-table or below-table event earlier in the trial',grade(good,invalid=True)==0.)]
  cases.append(('one front shoulder 7.5 deg off: that leg not counted',grade(state({'FL_hip_joint':math.radians(7.5)}))==.5))
  cases.append(('one front thigh 7.5 deg off: that leg not counted',grade(state({'FR_thigh_joint':math.radians(7.5)}))==.5))
  cases.append(('one front lower leg 7.5 deg toe-down past the posture',grade(state({'FR_shank_joint':task.QF-math.radians(7.5)}))==.5))
  cases.append(('every thigh and lower leg 6.5 deg off still counts',grade(state({n:v+math.radians(6.5) for n,v in task.GOAL.items() if '_hip_' not in n}))==1.))
  cases.append(('front shoulder and thigh each 6.5 deg off: thigh 9.2 deg from vertical, not counted',grade(state({'FL_hip_joint':math.radians(6.5),'FL_thigh_joint':math.radians(6.5)}))==.5))
  # Pressing a toe straight down without holding the thigh turns the thigh too (passive_retention.py: 7.8 deg, lower
  # leg 9.4 deg off).
  cases.append(('front leg as left by pressing its toe down: thigh 7.8, lower leg 9.4 deg off',grade(state({'FL_thigh_joint':math.radians(7.8),'FL_shank_joint':task.QF+math.radians(9.4)}))==.5))
  # The robot's own arm cannot reach the table from a standing pose; the scorer still refuses a propped body.
  prop=dict(task.GOAL);prop.update(mainarm_joint=2.8,forearm_joint=-2.7)
  ss=state(prop);low=task.measure(ss)['arm']
  propped=state(prop,dz=-(low-.005)) if low>.005 else ss
  cases.append(('own arm on the table',task.measure(propped)['arm']<.010 and grade(propped)==0.))
  tilted=state(quat=(math.sin(math.radians(11)/2),0,0,math.cos(math.radians(11)/2)))
  cases.append(('body rolled 11 deg',grade(tilted)==0.))
  belly={**{l+'_thigh_joint':math.radians(80) for l in task.LEGS}}
  lying=state(belly);lying=state(belly,dz=-(task.measure(lying)['belly']-.005))
  cases.append(('lying on its belly with the legs folded',task.measure(lying)['belly']<.015 and grade(lying)==0.))
  grade(good);task._certificate=();cases.append(('stale certificate',task.score({'xgo':good})==0.))
  cases.append(('missing data',task.score({})==0.))
  cases.append(('empty hand cloud',not task.released([],good)))
  cases.append(('NaN hand cloud',not task.released([np.full((3,3),np.nan)],good)))
  # A hand face 0.5 mm from a large torso face must fail without nearby vertices.
  _,c,r,size=task.XGO.world_boxes(task.poses(good))[0]
  face=c+r@np.array((0,size[1]/2+.0005,0));cases.append(('hand face 0.5 mm from the chassis',not task.released([face[None,:]],good)))
  face=c+r@np.array((0,size[1]/2+.031,0));cases.append(('hand 31 mm from the chassis is clear',task.released([face[None,:]],good)))
 finally:
  for n,v in old.items():setattr(task,n,v)
 for label,ok in cases:print(('ok' if ok else 'FAIL'),label)
 return sum(not ok for _,ok in cases)
if __name__=='__main__':sys.exit(run())
