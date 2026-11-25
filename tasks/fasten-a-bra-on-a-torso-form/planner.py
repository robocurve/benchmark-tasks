"""Offline fixed joint trajectory authoring. No task state is used at run time."""
import numpy as np
from common import down

class Plan:
 def __init__(self):
  from pad_covers import configure_kinematics
  configure_kinematics()
  from yam_arena.constants import READY
  self.a=np.r_[READY,1.,READY,1.];self.frames=[];self.audit=[]
 def add(self,seconds,label,targets,mode='cartesian'):
  from yam_arena import ik,station
  from scipy.optimize import least_squares
  b=self.a.copy();poses={s:ik.tcp_pose(b[o:o+6],station.BASES[s]) for s,o in [('left',0),('right',7)]};goals={};errors=[0.,0.]
  if mode=='joint':
   for side,(p,R,g) in targets.items():
    o=0 if side=='left' else 7
    def f(q):
     pp,rr=ik.tcp_pose(q,station.BASES[side]);return np.r_[pp-p,.3*ik._rot_error(R,rr)]
    solutions=[least_squares(f,seed,bounds=(ik.LOWER,ik.UPPER),max_nfev=200) for seed in [b[o:o+6],(ik.LOWER+ik.UPPER)/2]+[np.random.default_rng(k).uniform(ik.LOWER,ik.UPPER) for k in [17,31,43]]]
    sol=min(solutions,key=lambda x:np.linalg.norm(x.fun))
    errors=[max(errors[0],float(np.linalg.norm(sol.fun[:3]))),max(errors[1],float(np.linalg.norm(sol.fun[3:])/.3))]
    if errors[0]>.015 or errors[1]>.10:raise ValueError((label,side,'unreachable',sol.fun))
    goals[side]=sol.x
  for k in range(1,round(seconds*20)+1):
   u=ik.smoothstep(k/(seconds*20))
   for side,(p,R,g) in targets.items():
    p=np.asarray(p);o=0 if side=='left' else 7;p0,R0=poses[side]
    if side in goals:self.a[o:o+6]=b[o:o+6]+u*(goals[side]-b[o:o+6])
    else:
     q,pe,re=ik.solve(p0+u*(p-p0),ik.matrix_to_quat_xyzw(ik.slerp_matrix(R0,R,u)),self.a[o:o+6],base_pos=station.BASES[side],iters=100,rot_weight=.08);self.a[o:o+6]=q;errors=[max(errors[0],float(pe)),max(errors[1],float(re))]
    self.a[o+6]=b[o+6]+u*(g-b[o+6])
   self.frames.append(dict(action14=self.a.tolist(),stage=label))
  self.audit.append(dict(stage=label,position_error_m=errors[0],rotation_error_rad=errors[1]))
  if errors[0]>.015:raise ValueError((label,'path unreachable',errors))
  return self
 def save(self,path):
  import json
  from yam_arena.constants import READY,SHOULDER,WRIST,GRIPPER,FINGER_OPEN
  limits=np.tile([SHOULDER['velocity']/20]*3+[WRIST['velocity']/20]*3+[GRIPPER['velocity']/abs(FINGER_OPEN)/20],2)
  a=np.vstack([np.r_[READY,1.,READY,1.]]+[f['action14'] for f in self.frames]);ratio=float(np.max(abs(np.diff(a,axis=0))/limits))
  if max(r['position_error_m'] for r in self.audit)>.015:raise ValueError(self.audit)
  if ratio>1.01:raise ValueError(('too fast',ratio))
  path.write_text(json.dumps(dict(control_hz=20,frames=self.frames,kinematic_audit=self.audit,maximum_velocity_ratio=ratio),separators=(',',':'))+'\n');return dict(steps=len(self.frames),max_velocity_ratio=ratio,audit=self.audit)
