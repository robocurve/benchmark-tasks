"""Released four-point geometric endpoint, sampled at native 240 Hz."""
from collections import deque
from dataclasses import dataclass
import numpy as np
from . import layout as L

class KeyboardGrade:
 def __init__(self,sample_hz=240.):
  self.hz=float(sample_hz);self.sample=0;self.void=None;self.inverted=False
  self.opened=np.zeros(2,dtype=bool);self.window=deque(maxlen=round(self.hz)+1)
 def update(self,poses,velocities,*,robot_counts,support_loads_n,notebook_contacts):
  self.sample+=1;p=np.asarray(poses,float);v=np.asarray(velocities,float);load=np.asarray(support_loads_n,float)
  robot=np.asarray(robot_counts,float);book=np.asarray(notebook_contacts,float)
  if p.shape!=(len(L.LINKS),7) or v.shape!=(len(L.LINKS),6) or load.shape!=(4,) or robot.shape!=(18,4) or book.shape!=(3,1) or not all(np.isfinite(x).all() for x in (p,v,load,robot,book)) or np.any(robot<0) or np.any(book<0):
   self.void='invalid physical input';self.window.clear();return self.result(0.)
  if np.any(np.abs(np.linalg.norm(p[:,3:],axis=1)-1.)>.001):
   self.void='invalid physical orientation';self.window.clear();return self.result(0.)
  R=L.quat_matrix(p[0,3:]);q=L.angles(p);self.inverted|=bool(R[2,2]<-.96)
  self.opened|=(q<np.deg2rad(-110)) & bool(R[2,2]<-.96)
  if p[0,2]<-.04:self.void=self.void or 'keyboard fell off worktop'
  feet=np.array([(L.LEG_CORNERS@L.quat_matrix(x[3:]).T+x[:3])[:,2].min() for x in p[1:3]])
  front=(L.FRONT_PAD_CORNERS@R.T+p[0,:3])[:,:,2].min(axis=1)
  # Actual end points and the time between them, never one interval on sample1.
  flat=bool(R[2,2]>.96 and np.sin(np.deg2rad(4))<=R[2,0]<=np.sin(np.deg2rad(14)))
  expanded=bool(np.all(q<np.deg2rad(-115)))
  grounded=bool(np.all(np.abs(feet)<=.001) and np.all(np.abs(front)<=.001))
  free=not bool(np.any(robot_counts))
  outline=L.corners((L.DEPTH,L.WIDTH,L.HEIGHT),(0,0,.0078))@R.T+p[0,:3]
  contained=bool(np.all((outline[:,0]>L.TABLE_BOUNDS[0])&(outline[:,0]<L.TABLE_BOUNDS[1])&(outline[:,1]>L.TABLE_BOUNDS[2])&(outline[:,1]<L.TABLE_BOUNDS[3])))
  book_free=not bool(np.any(notebook_contacts))
  ready=bool(self.inverted and self.opened.all() and expanded and flat and grounded and free and contained and book_free)
  if ready:
   self.window.append((p[0,:3].copy(),p[0,3:].copy()))
   # Every accepted sample stays within the same marked resting pose.
   start_xyz,start_quat=self.window[0]
   turn=2*np.arccos(np.clip(abs(start_quat @ p[0,3:]),0.,1.))
   if np.linalg.norm(p[0,:3]-start_xyz)>.002 or turn>np.deg2rad(2):
    self.window.clear()
    self.window.append((p[0,:3].copy(),p[0,3:].copy()))
  else:self.window.clear()
  duration=max(0,len(self.window)-1)/self.hz
  supported=len(self.window)==self.window.maxlen
  success=bool(supported and not self.void)
  score=1. if success else .6 if self.opened.all() else .3 if self.opened.any() else 0.
  return self.result(0. if self.void else score,angles_deg=np.rad2deg(q).tolist(),inverted=self.inverted,
   feet_opened=self.opened.tolist(),flat=flat,grounded=grounded,feet_height_m=feet.tolist(),front_pad_height_m=front.tolist(),
   released=free,notebook_clear=book_free,contained=contained,stable_seconds=duration,support_loads_n=load.tolist(),actual_front_to_rear_tilt_deg=float(np.rad2deg(np.arcsin(R[2,0]))))
 def result(self,score,**kw):return dict(score=float(score),finished=bool(score==1. or self.void),void=self.void,sample=self.sample,sample_hz=self.hz,**kw)

@dataclass(frozen=True)
class KeyboardScorer:
 name:str='keyboard_feet_score'
 def __call__(self,record,target):
  from inspect_robots.scorer import Score
  if not record.steps:return Score(value=0.,explanation='No native physical steps')
  g=record.steps[-1].result.info['keyboard_grade'];return Score(value=g['score'],explanation=str(g))
def make_scorer():return KeyboardScorer()
