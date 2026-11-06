"""Actual 315-hull grasp screening; primitive holes remain open."""
from pathlib import Path
import json,numpy as np
from chain_task.geometry import *
from chain_task.motion import pickup,Plan
from collisions import *
ROOT=Path(__file__).resolve().parents[1];rows=[]
for seed in [17,23,41]:
 data=layout(seed);obs=static_scene(ROOT)
 for name,d in data.items():
  i=int(name[-1]);p,r=d['position'],d['rotation'];obs+=asset(name,ROOT/'assets'/('frame'+str(i)+'.usda'),p,r)+asset('gate'+str(i),ROOT/'assets/gate.usda',p+r@GATE_OFFSET,r)
 for i,side in [(0,'left'),(1,'right'),(2,'right')]:
  d=data['link'+str(i)];variants=[]
  for flip in [0,1]:
   plan=Plan(pickup(d['position'],d['rotation'],flip),side);hits=[];error=None
   for t in np.arange(.25,6.01,.25):
    try:a,label=plan.action7(t)
    except RuntimeError as exc:error=str(exc);break
    allow={(n,'link'+str(i)) for n in ['tip_left','tip_right']} if t>=4.5 else set()
    hits.extend((float(t),*v) for v in against(robot(a[:6],max(a[6],.0045/.0939),side),obs,allow))
   variants.append(dict(flip=flip,error=error,position_error=plan.worst_position_error,rotation_error=plan.worst_rotation_error,hits=hits))
  rows.append(dict(seed=seed,link=i,side=side,variants=variants))
print(json.dumps(dict(grasp=rows,robot_collision_hulls=len(HULLS)),indent=2))
