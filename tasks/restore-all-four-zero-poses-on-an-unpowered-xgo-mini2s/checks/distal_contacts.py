"""Reproduce the independent heel-only counterexample through the runtime contact reader.
No native simulator result is inferred from this tensor-view substitute.
"""
from pathlib import Path
import sys,types
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import task
from observation import distal_supports

def ground(q,foot):
 s=task.XGO.state(joints=q,pos=(.3,0,task.TOP));pp=task.poses(s);cloud=[]
 for leg in task.LEGS:
  n=leg+'_calf_joint';p,r=pp[n];v=task.VERTS[n]
  if foot:v=v[v[:,0]>v[:,0].max()-.020]
  cloud.extend(v@r.T+p)
 return task.XGO.state(joints=q,pos=(.3,0,task.TOP+task.TOP-np.min(np.array(cloud)[:,2])))
def data(s,foot):
 pp=task.poses(s);points=[]
 for leg in task.LEGS:
  n=leg+'_calf_joint';p,r=pp[n];v=task.VERTS[n]
  if foot:v=v[v[:,0]>v[:,0].max()-.020]
  world=v@r.T+p;points.append(world[np.argmin(world[:,2])])
 forces=np.zeros((64,1));forces[:4]=2.;pos=np.zeros((64,3));pos[:4]=points;norm=np.tile([0.,0.,1.],(64,1));sep=np.zeros((64,1));counts=np.ones((4,1),int);starts=np.arange(4)[:,None]
 return forces,pos,norm,sep,counts,starts
class View:
 def get_contact_data(self,dt):return self.data
view=View();task.observation._foot_view=view;task.observation._foot_shape=(4,1,3)
base=types.SimpleNamespace(scene=types.SimpleNamespace(env_origins=np.zeros((1,3))),sim=types.SimpleNamespace(get_physics_dt=lambda:1/240))
class Sim:
 env=types.SimpleNamespace(unwrapped=base);t=0.;s=None
 def states(self):return {'xgo':self.s}
 def set_target(self,*args):pass
sim=Sim();task.observation.hand_state=lambda _:([np.array([[3.,3.,3.]])],0.);task.observation.arm_support_force=lambda _:0.
def run(s,contact):
 sim.s=s;view.data=contact;task._window=task.deque();task._time=0.;task._invalid=False;task._rest={};sim.t=0.
 scores=[]
 for i in range(25):sim.t=(i+1)*.05;task.step(sim);scores.append(task.score({'xgo':s}))
 return scores
bad=ground({leg+'_calf_joint':np.deg2rad(10) for leg in task.LEGS},False)
print('heel-supported distal gaps mm',np.array(task.measure(bad)['feet'])*1000)
assert all(.001<g<.003 for g in task.measure(bad)['feet'])
assert not any(distal_supports(bad,np.zeros(3),data(bad,False)))
assert not any(run(bad,data(bad,False)))
print('ok exact reviewed hovering-foot endpoint earns zero')
good=ground(task.GOAL,True)
assert all(distal_supports(good,np.zeros(3),data(good,True)))
assert run(good,data(good,True))[-1]==1.
print('ok loaded distal-foot contacts qualify after released still interval')
touching=list(data(good,True));touching[0]=np.zeros((64,1))
assert all(distal_supports(good,np.zeros(3),tuple(touching)))
print('ok toes touching the tabletop (separation 0, no reported load) qualify')
for gap,want in ((.00015,False),(.0009,False),(.0001,True)):
 hover=list(data(good,True));hover[0]=np.zeros((64,1));hover[3]=np.full((64,1),gap)
 assert all(distal_supports(good,np.zeros(3),tuple(hover)))==want and (any(distal_supports(good,np.zeros(3),tuple(hover)))==want)
 print('ok toe-hull point',gap*1000,'mm from the tabletop with no load:','down' if want else 'not down')
broken=list(data(good,True));broken[1][0,0]=np.nan
assert not any(run(good,tuple(broken)))
print('ok unknown point data fail closed')
# Verify the coordinate transform also works in rotated/translated layout B.
s=task.XGO.state(joints=task.GOAL,pos=(.22,0,good.pos[2]),quat_xyzw=task.parts._matrix_to_quat(task.parts._rot((0,0,1),np.pi)))
assert all(distal_supports(s,np.zeros(3),data(s,True)))
print('ok distal-region transform in rotated layout B')
