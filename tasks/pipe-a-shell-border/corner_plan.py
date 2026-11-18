"""Extend the validated straight-border controller around a physical corner.

Only ordinary joint targets and the disclosed tool-pose servo are used. The
material model receives neither this path nor its intended shell positions.
"""
import copy,json
from pathlib import Path
import numpy as np
from plan import calibrated_hand_pose

def build():
 from yam_arena import ik,station
 from yam_arena.constants import READY,SHOULDER,WRIST,GRIPPER,FINGER_OPEN
 old=json.loads(Path(__file__).with_name('straight-reference-plan.json').read_text())
 frames=copy.deepcopy(old['frames'][:734]); current=np.array(frames[-1]['action14']); maximum=[0.,0.]
 R0=np.asarray(frames[-1]['scripted_tool_target']['barrel_rotation']); Q=np.array([[0,1,0],[-1,0,0],[0,0,1.]])
 # Original second shell starts at (235,-26) mm; new seventh at (239,45).
 origin=np.array([.235,-.026,.8035]); destination=np.array([.239,.045,.8035])
 def transform(p):
  # Keep the physical nozzle angle while changing travel direction at corner.
  tip=np.asarray(p)+R0[:,1]*.08
  return destination+Q@(tip-origin)-R0[:,1]*.08
 def move_hand(p,b):return np.asarray(p)+transform(b)-np.asarray(b)
 def solve(p,R,q,side):
  out,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),q,station.BASES[side],iters=200,tol=1e-6)
  maximum[0]=max(maximum[0],pe);maximum[1]=max(maximum[1],re)
  if pe>.001 or re>.01:raise RuntimeError((side,pe,re,p.tolist()))
  return out
 def transition(seconds,label,goals,tool):
  nonlocal current
  poses={s:ik.tcp_pose(current[o:o+6],station.BASES[s]) for s,o in [('left',0),('right',7)]}
  start=current.copy(); before=frames[-1].get('scripted_tool_target'); count=round(seconds*20)
  for k in range(1,count+1):
   w=ik.smoothstep(k/count)
   for side,(p,R,g) in goals.items():
    o=0 if side=='left' else 7;pp,rr=poses[side]
    current[o:o+6]=solve(pp+(p-pp)*w,ik.slerp_matrix(rr,R,w),current[o:o+6],side)
    current[o+6]=start[o+6]+w*(g-start[o+6])
   row=dict(phase=label,action14=current.tolist(),joint_feedforward14=[0.]*14)
   if tool:
    b0=np.array(before['barrel_position_m']);r0=np.array(before['barrel_rotation']);b1,r1=tool
    row['scripted_tool_target']=dict(barrel_position_m=(b0+w*(b1-b0)).tolist(),barrel_rotation=ik.slerp_matrix(r0,r1,w).tolist())
   frames.append(row)
 oldtip=np.array([.235,.044,.8035]); oldb=oldtip-R0[:,1]*.08
 lp,lR=ik.tcp_pose(current[:6],station.BASES['left'])
 transition(1.,'withdraw pusher before corner',{'left':(lp-R0[:,1]*.05,lR,0.)},(oldb,R0))
 raised=oldb+[0,0,.045];rp,rR=calibrated_hand_pose(raised,R0)
 transition(1.5,'lift nozzle before turning',{'right':(rp,rR,0.)},(raised,R0))
 # Move both free/held hands into the next side's orientation above the cake.
 template=old['frames'][474];a=np.array(template['action14']);nr=R0.copy()
 newb=transform(template['scripted_tool_target']['barrel_position_m'])+[0,0,.045]
 rp,rR=calibrated_hand_pose(newb,nr);p,r=ik.tcp_pose(a[:6],station.BASES['left']);lp=move_hand(p,template['scripted_tool_target']['barrel_position_m'])+[0,0,.045]-nr[:,1]*.05
 transition(4.,'turn dispenser around cake corner',{'right':(rp,rR,0.),'left':(lp,r,0.)},(newb,nr))
 newb-=np.array([0,0,.045]);rp,rR=calibrated_hand_pose(newb,nr);lp-=np.array([0,0,.045])
 transition(1.5,'lower nozzle onto second guide',{'right':(rp,rR,0.),'left':(lp,r,0.)},(newb,nr))
 # Five additional shells' worth of piston displacement relative to template.
 increment=old['frames'][492]['scripted_tool_target']['piston_m']-old['frames'][440]['scripted_tool_target']['piston_m'];shift=5*increment
 p,r=ik.tcp_pose(a[:6],station.BASES['left']);lp=move_hand(p,template['scripted_tool_target']['barrel_position_m'])+nr[:,1]*shift
 transition(1.,'approach plunger on second side',{'left':(lp,r,0.)},(newb,nr))
 side_start=len(frames)
 for template_index,original in enumerate(old['frames'][474:682]):
  row=copy.deepcopy(original);a=np.array(original['action14']);recipe=row['scripted_tool_target'];has_pressure='piston_m' in recipe
  j=template_index%52
  # Reallocate part of the pressure pulse from the head to the tail.
  # This changes only the scripted hand motion and piston target.
  correction=(-.0012*np.clip((j-18)/6,0,1) if 18<=j<31 else
              -.0012+.0016*np.clip((j-31)/15,0,1) if 31<=j<=46 else 0.)
  for side,o in [('left',0),('right',7)]:
   p,r=ik.tcp_pose(a[o:o+6],station.BASES[side]);p=move_hand(p,original['scripted_tool_target']['barrel_position_m'])
   if side=='left':p+=nr[:,1]*(shift+correction)
   current[o:o+6]=solve(p,r,current[o:o+6],side);current[o+6]=a[o+6]
  recipe['barrel_position_m']=transform(recipe['barrel_position_m']).tolist();recipe['barrel_rotation']=R0.tolist()
  if has_pressure:recipe['piston_m']+=shift+correction
  row['phase']='second side: '+row['phase'];row['action14']=current.tolist();row['joint_feedforward14']=[0.]*14;frames.append(row)
  # The orthogonal wrist motion lags the original travel direction. Give
  # each tail target twice as long, retaining ordinary bounded joint control.
  if 'draw tapered shell tail' in original['phase']:frames.append(copy.deepcopy(row))
 finalb=np.array(frames[-1]['scripted_tool_target']['barrel_position_m']);lp,lR=ik.tcp_pose(current[:6],station.BASES['left'])
 transition(1.,'withdraw pusher after second side',{'left':(lp-nr[:,1]*.05,lR,0.)},(finalb,nr))
 finalb+=np.array([0,0,.045]);rp,rR=calibrated_hand_pose(finalb,nr)
 transition(1.5,'lift nozzle from finished border',{'right':(rp,rR,0.)},(finalb,nr))
 target=old['frames'][773];goals={}
 for side,o in [('left',0),('right',7)]:
  aa=np.array(target['action14']);p,r=ik.tcp_pose(aa[o:o+6],station.BASES[side]);goals[side]=(p,r,aa[o+6])
 recipe=target['scripted_tool_target']
 transition(4.,'turn clear of decoration for return',goals,(np.array(recipe['barrel_position_m']),np.array(recipe['barrel_rotation'])))
 frames+=copy.deepcopy(old['frames'][774:])
 lim=np.tile([SHOULDER['velocity']/20]*3+[WRIST['velocity']/20]*3+[GRIPPER['velocity']/abs(FINGER_OPEN)/20],2)
 ratio=float(np.max(abs(np.diff(np.vstack([np.r_[READY,1.,READY,1.]]+[f['action14'] for f in frames]),axis=0))/lim))
 if ratio>1.001:raise RuntimeError(('velocity ratio',ratio))
 return dict(schema='state-assisted-recipe-v1',path_kind='right-angle-corner',frames=frames,control_hz=20,shell_count=10,physics_state_writes=False,assistance=old['assistance'],maximum_command_velocity_ratio=ratio,maximum_ik_position_error_m=maximum[0],maximum_ik_rotation_error_rad=maximum[1])
if __name__=='__main__':
 p=build();Path(__file__).with_name('demo-plan.json').write_text(json.dumps(p,indent=2)+'\n');print(json.dumps({k:v for k,v in p.items() if k!='frames'}))
