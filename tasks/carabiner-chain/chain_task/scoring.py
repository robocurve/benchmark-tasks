"""Released serial-prefix scoring from measured closed-loop geometry."""
from dataclasses import dataclass
import math
import numpy as np
from chain_task.geometry import *
from chain_task.topology import linked

@dataclass
class Body:
 p:np.ndarray
 r:np.ndarray
 v:np.ndarray
 w:np.ndarray
 def valid(self):return all(np.isfinite(a).all() for a in [self.p,self.r,self.v,self.w]) and np.shape(self.p)==np.shape(self.v)==np.shape(self.w)==(3,) and np.shape(self.r)==(3,3) and np.linalg.norm(self.r.T@self.r-np.eye(3))<.001 and np.linalg.det(self.r)>.999
 def settled(self):return np.linalg.norm(self.v)<=.01 and np.linalg.norm(self.w)<=.2

def segments(bodies,index):
 f,g=bodies['link'+str(index)],bodies['gate'+str(index)];a=FRAME_POINTS@f.r.T+f.p;gate=np.array([[0,-GATE_LENGTH/2,0],[0,GATE_LENGTH/2,0]])@g.r.T+g.p
 return [(x,y,RADIUS) for x,y in zip(a[:-1],a[1:])]+[(gate[0],gate[1],RADIUS)]
def segment_box(a,b,low,high,margin):
 low,high=np.asarray(low)-margin,np.asarray(high)+margin;a,b=np.asarray(a),np.asarray(b);d=b-a;lo,hi=0.,1.
 for i in range(3):
  if abs(d[i])<1e-12:
   if a[i]<low[i] or a[i]>high[i]:return False
  else:
   x,y=(low[i]-a[i])/d[i],(high[i]-a[i])/d[i];lo=max(lo,min(x,y));hi=min(hi,max(x,y))
 return lo<=hi

def curves_and_closed(bodies):
 stand=bodies['stand'];curves={'eye':eye_loop(stand.p,stand.r)};closed={'eye':True}
 for i in range(3):
  f,g=bodies['link'+str(i)],bodies['gate'+str(i)];key='link'+str(i);curves[key]=loop(f.p,f.r,g.p,g.r);closed[key]=abs(gate_angle(f.r,g.r))<=math.radians(5) and gate_gap(f.p,f.r,g.p,g.r)<=.0015
 return curves,closed

def graph(curves,closed):
 names=list(curves);result={n:set() for n in names}
 for i,a in enumerate(names):
  for b in names[i+1:]:
   if closed[a] and closed[b] and linked(curves[a],curves[b]):result[a].add(b);result[b].add(a)
 return result

def serial_paths(edges):
 roots=edges['eye'];paths=[]
 for first in roots:
  path=[first];previous,current='eye',first
  while True:
   paths.append(tuple(path))
   if len(roots)!=1:break
   following=edges[current]-{previous}
   if len(following)!=1:break
   child=next(iter(following))
   if child=='eye' or child in path:break
   path.append(child);previous,current=current,child
 return paths

class History:
 def __init__(self):self.invalid=False;self.timers={};self.score=0.;self.reasons={}
 def update(self,bodies,robot_bounds,dt,hardware_valid=True,tunnel_invalid=False):
  required={'stand',*(f'link{i}' for i in range(3)),*(f'gate{i}' for i in range(3))}
  good=required<=set(bodies) and all(bodies[k].valid() for k in required) and hardware_valid and not tunnel_invalid and math.isfinite(dt) and 0<dt<=.1 and len(robot_bounds)>=6
  if good:
   good=np.linalg.norm(bodies['stand'].p-STAND_POS)<=.001 and np.linalg.norm(bodies['stand'].r-np.eye(3))<=.01
   for low,high in robot_bounds.values():
    if np.shape(low)!=(3,) or np.shape(high)!=(3,) or not np.isfinite([low,high]).all() or np.any(np.asarray(low)>high):good=False
  if not good:self.invalid=True
  if self.invalid:self.score=0.;self.timers={};self.reasons={'invalid':True};return 0.
  rods={i:segments(bodies,i) for i in range(3)}
  if min(min(a[2],b[2])-r for shapes in rods.values() for a,b,r in shapes)<TABLE-.003:self.invalid=True;return self.update(bodies,robot_bounds,dt)
  curves,closed=curves_and_closed(bodies);edges=graph(curves,closed);paths=serial_paths(edges)
  # Requiring both arms clear of every carabiner also rejects indirect robot
  # support transmitted through a lower connected link.
  released=not any(segment_box(a,b,low,high,r+.005) for shapes in rods.values() for a,b,r in shapes for low,high in robot_bounds.values())
  settled=all(bodies[k].settled() for k in required if k!='stand');timers={};best=0.
  for path in paths:
   suspended=all(min(min(a[2],b[2])-r for a,b,r in rods[int(k[-1])])>=TABLE+.020 for k in path)
   valid=released and settled and suspended
   timers[path]=self.timers.get(path,0.)+dt if valid else 0.
   if timers[path]>=2.-1e-9:best=max(best,len(path)/3.)
  self.timers=timers;self.score=best;self.reasons=dict(edges={k:sorted(v) for k,v in edges.items()},closed=closed,released=released,settled=settled,paths=[list(p) for p in paths],seconds={'/'.join(k):v for k,v in timers.items()},body_speeds={k:dict(linear_m_s=float(np.linalg.norm(bodies[k].v)),angular_rad_s=float(np.linalg.norm(bodies[k].w))) for k in required if k!='stand'})
  return self.score
