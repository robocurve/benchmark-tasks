"""Observable terminal-card acceptance; no controller phases or joint targets."""
import math
import numpy as np
import geometry as g

def finite_row(row):
 try:
  values=[row['time_s'],row['sample_id'],row['robot_clearance_m']]
  for v in [row['case'],*row['cards'].values()]:
   values.extend(v['pos']);values.extend(v['quat_xyzw']);values.extend(v['lin_vel']);values.extend(v['ang_vel'])
   if abs(np.linalg.norm(v['quat_xyzw'])-1)>.001:return False
  return bool(np.isfinite(values).all())and set(row['cards'])==set(g.initial_cards())
 except (KeyError,TypeError,ValueError):return False

def classify(row):
 if not finite_row(row):return dict(score=0.,success=False,valid=False,reason='missing/nonfinite observation')
 case=row['case'];R=g.rotation(case['quat_xyzw']);cp=np.asarray(case['pos'])
 inside=[];output=[];presented=[];other=[]
 stable=np.linalg.norm(case['lin_vel'])<.005 and np.linalg.norm(case['ang_vel'])<.12
 for name,v in row['cards'].items():
  verts=g.corners(np.array(v['pos']),v['quat_xyzw'],g.CARD_SIZE)
  local=(verts-cp)@R
  level=abs(g.rotation(v['quat_xyzw'])[2,2])>math.cos(math.radians(8))
  in_case=bool(np.max(abs(local[:,0]))<=.0580 and np.max(abs(local[:,1]))<=.0305 and local[:,2].min()>=.0010 and local[:,2].max()<=.0150)
  on_output=bool(abs(verts[:,0]-g.TRAY[0]).max()<.071 and abs(verts[:,1]-g.TRAY[1]).max()<.051 and abs(verts[:,2].min()-(g.TRAY[2]+.001))<.0020 and level)
  # A partial card must actually project far enough to pinch; command stroke is irrelevant.
  exposed=bool(local[:,0].max()>.070 and local[:,0].min()<.049 and abs(local[:,1]).max()<.0295 and local[:,2].min()>.011 and local[:,2].max()<.015)
  if in_case:
   inside.append(name)
   stable=stable and (np.linalg.norm(v['lin_vel'])<.005 and np.linalg.norm(v['ang_vel'])<.12)
  elif on_output:
   output.append(name)
   stable=stable and (np.linalg.norm(v['lin_vel'])<.005 and np.linalg.norm(v['ang_vel'])<.12)
  elif exposed:presented.append(name)
  else:other.append(name)
 integrity=not other and len(inside)+len(output)+len(presented)==g.N
 score=0.
 if integrity and len(inside)==g.N-1 and len(output)==1:score=.75
 elif integrity and len(inside)==g.N-1 and len(presented)==1:score=.25
 return dict(score=float(score),success=False,valid=True,inside=inside,output=output,presented=presented,other=other,
             full_candidate=bool(score==.75 and stable and row['robot_clearance_m']>=.015),stable=bool(stable))

class Monitor:
 def __init__(self):self.last=dict(score=0.,success=False,valid=False);self.time=None;self.sample=None;self.since=None
 def update(self,row):
  now=row.get('time_s');sample=row.get('sample_id')
  if not finite_row(row)or(self.time is not None and(now<=self.time or sample<=self.sample)):
   self.since=None;self.last=dict(score=0.,success=False,valid=False,reason='stale/nonfinite observation');return self.last
  if self.time is not None and now-self.time>.15:self.since=None
  self.time,self.sample=now,sample;result=classify(row)
  if result.get('full_candidate'):
   if self.since is None:self.since=now
   if now-self.since>=1.0-1e-8:result.update(score=1.,success=True)
  else:self.since=None
  result['held_s']=0. if self.since is None else now-self.since;self.last=result;return result
