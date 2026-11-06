"""Reject unverified layout geometry before exposing a live reset to a policy.

Imports OpenUSD only when called after SimulationApp initialization.
"""
import numpy as np
from chain_task.geometry import *

def validate_layout(task_root,poses,rotations,generated):
 from chain_task.motion import Plan,pickup
 from chain_task.collision_checks import asset,static_scene,robot,against,cross,HULLS
 from yam_arena.constants import READY
 from chain_task.scoring import Body,curves_and_closed,graph
 bodies={n:Body(poses[n],rotations[n],np.zeros(3),np.zeros(3)) for n in poses};curves,closed=curves_and_closed(bodies)
 if any(graph(curves,closed).values()):raise ValueError('Reject initially interlinked layout')
 obstacles=static_scene(task_root)
 for i in range(3):
  name='link'+str(i)
  if np.linalg.norm(poses[name]-generated[name]['position'])>.0015 or np.linalg.norm(rotations[name]-generated[name]['rotation'])>.01:raise ValueError('Settled layout differs from its recorded generator: '+name)
  obstacles+=asset(name,task_root/'assets'/('frame'+str(i)+'.usda'),poses[name],rotations[name])+asset('gate'+str(i),task_root/'assets/gate.usda',poses['gate'+str(i)],rotations['gate'+str(i)])
 choices={};rejected=[]
 for i,side in [(0,'left'),(1,'right'),(2,'right')]:
  name='link'+str(i);other='right' if side=='left' else 'left';other_shapes=robot(np.array(READY),1.,other);found=None
  for flip in [0,1]:
   plan=Plan(pickup(poses[name],rotations[name],flip,'spine'),side);error=None;hits=[]
   for t in np.arange(.1,6.001,.1):
    try:action,label=plan.action7(float(t))
    except RuntimeError as exc:error=str(exc);break
    shapes=robot(action[:6],max(action[6],.0045/.0939),side);allowed={(tip,name) for tip in ['tip_left','tip_right']} if t>=4.5 else set();hits=against(shapes,obstacles,allowed)+[('other_arm',str(v)) for v in cross(shapes,other_shapes)]
    if hits:break
   if error is None and not hits:found=dict(side=side,jaw_flip=flip,max_position_error_m=plan.worst_position_error,max_rotation_error_rad=plan.worst_rotation_error,sampling_seconds=.1);break
   rejected.append(dict(link=name,side=side,jaw_flip=flip,error=error,hits=hits))
  if found is None:raise ValueError('Reject inaccessible carabiner layout: '+name+' '+str(rejected[-2:]))
  choices[name]=found
 return dict(initial_interlinks=False,generator_matches_settled_poses=True,robot_collision_hulls=len(HULLS),grasp_choices=choices,rejected_candidates=rejected,scope='Sampled normal flat-spine approaches from READY with the other arm READY; later clipping still requires separate path checks.')
