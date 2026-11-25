"""Offline MuJoCo assays of the passive XGO-lite (maker model, 500 g, on a level table, friction 0.5). Model tests
only, not measurements of a real unit. Run from the task folder: python checks/passive_retention.py
(needs mujoco). Writes fixture/passive-model-assays.json.
 1. posture: all four legs in the illustrated posture, released for 10 s.
 2. start:   the reset pose (rear legs in the posture, front lower legs 22 and 26 deg toe-up), released for 10 s:
             the front legs must not fall into the posture.
 3. hold-and-turn: the chassis held and lifted 5 mm by one ideal grip, each front lower leg turned about its knee
    to the posture (3 deg past it) by an ideal pad grip, set down until 1.5 mm from touching, released; then 10 s
    hands-off.
 4. press only: each front toe pad pressed straight down onto the table by a 16 mm block, nothing else held.
A leg counts as complete with every angle within 7 degrees, as in task.py.
Modes for the gearbox hold: 'declared' (0.06 N m elasto-plastic, as task.py), 'native' (the maker export's own
joint damping and friction, motors off) and 'coulomb' (plain 0.06 N m joint friction)."""
from pathlib import Path
import json,math
import mujoco,numpy as np
P=Path(__file__).resolve().parents[1];A=P/'assets'
B,K,MU=.06,2.,.5;QF=-.2123;LEGS=('FL','FR','RR','RL');UP={'FL':22,'FR':26,'RR':0,'RL':0}
def model(mode):
 xml=(A/'offline.xml').read_text().replace('meshdir="."','meshdir="'+str(A)+'"')
 xml=xml.replace('<worldbody>',f'<worldbody><geom name="floor" type="plane" size="1 1 .01" friction="{MU} .005 .0001" contype="2" conaffinity="1"/>'
  '<body name="hand" mocap="true" pos="0 .3 .3"><geom name="hand" type="sphere" size=".002" contype="0" conaffinity="0"/></body>'
  '<body name="holder" mocap="true" pos="0 -.3 .3"><geom name="holder" type="sphere" size=".002" contype="0" conaffinity="0"/></body>'
  '<body name="block" mocap="true" pos="0 .5 .5"><geom name="block" type="box" size=".008 .008 .02" contype="2" conaffinity="1" friction="1 .005 .0001"/></body>',1)
 xml=xml.replace('</worldbody>','</worldbody><equality>'+''.join(f'<weld name="w{l}" body1="hand" body2="{l}_shank" active="false" solref=".004 1"/>' for l in LEGS)+'<weld name="hold" body1="holder" body2="body" active="false" solref=".004 1"/></equality>',1)
 xml=xml.replace('<option gravity="0 0 -9.81" integrator="RK4"/>','<option gravity="0 0 -9.81" integrator="implicitfast" timestep="0.001"/>')
 m=mujoco.MjModel.from_xml_string(xml);m.actuator_gainprm[:]=0;m.actuator_biasprm[:]=0
 m.body_mass[m.body('body').id]+=.500-sum(m.body_mass[b] for b in range(1,m.nbody) if mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,b) not in ('hand','holder','block'))
 for g in range(m.ngeom):
  n=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,g) or ''
  if n.endswith('_collision'):m.geom_contype[g]=1;m.geom_conaffinity[g]=2;m.geom_friction[g,0]=MU;m.geom_condim[g]=3
 hinges=[j for j in range(m.njnt) if m.jnt_type[j]==mujoco.mjtJoint.mjJNT_HINGE];va=m.jnt_dofadr[hinges]
 if mode=='declared':m.dof_damping[va]=.04;m.dof_frictionloss[va]=0;m.dof_armature[va]=.0005
 if mode=='coulomb':m.dof_frictionloss[va]=B
 return m,mujoco.MjData(m),np.asarray(m.jnt_qposadr[hinges]),np.asarray(va)
class Run:
 def __init__(s,mode):
  s.mode=mode;s.m,s.d,s.qa,s.va=model(mode);s.hand=None;s.holder=None;s.block=None
 def j(s,n):return s.m.jnt_qposadr[s.m.joint(n).id]
 def low(s,prefixes=('',)):
  m,d=s.m,s.d;out=[]
  for g in range(m.ngeom):
   n=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,g) or ''
   if n.endswith('_collision') and any(n.startswith(p) for p in prefixes):
    mid=m.geom_dataid[g];v=m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g];out.append(v[:,2].min())
  return min(out)
 def place(s,shank):
  m,d=s.m,s.d;mujoco.mj_resetData(m,d);d.qpos[3]=1
  d.qpos[s.j('mainarm_joint')]=-1.55;d.qpos[s.j('forearm_joint')]=1.3
  for l in LEGS:d.qpos[s.j(l+'_shank_joint')]=shank[l]
  mujoco.mj_forward(m,d);d.qpos[2]-=s.low()-.0005;mujoco.mj_forward(m,d);s.rest=d.qpos[s.qa].copy()
 def run(s,seconds):
  m,d=s.m,s.d
  for k in range(int(round(seconds/m.opt.timestep))):
   if s.mode=='declared':
    if round(d.time/m.opt.timestep)%50==0:s.rest=d.qpos[s.qa]-np.clip(d.qpos[s.qa]-s.rest,-B/K,B/K)
    d.qfrc_applied[s.va]=K*(s.rest-d.qpos[s.qa])
   if s.hand:d.mocap_pos[0],d.mocap_quat[0]=s.hand(d.time)
   if s.holder:d.mocap_pos[1]=s.holder(d.time)
   if s.block:d.mocap_pos[2]=s.block(d.time)
   mujoco.mj_step(m,d)
 def state(s):
  d=s.d;q=lambda n:math.degrees(d.qpos[s.j(n)])
  return dict(off_posture_deg={l:{'shoulder':round(q(l+'_hip_joint'),2),'thigh':round(q(l+'_thigh_joint'),2),'lower_leg':round(q(l+'_shank_joint')-math.degrees(QF),2)} for l in LEGS},
   body_mm=(1000*d.qpos[:3]).round(1).tolist(),belly_mm=round(1000*s.low(('body',)),1))
 def complete(s):
  st=s.state()['off_posture_deg'];return sum(all(abs(v)<=7 for v in st[l].values()) for l in LEGS)
 def complete_front(s):
  st=s.state()['off_posture_deg'];return sum(all(abs(v)<=7 for v in st[l].values()) for l in ('FL','FR'))
def weld(r,i,on):
 r.m.eq_data[i,:]=0;r.m.eq_data[i,6:10]=[1,0,0,0];r.m.eq_data[i,10]=1;r.d.eq_active[i]=on
def hold_and_turn(r):
 m,d=r.m,r.d
 for side in (('FR','FL'),):
  b=m.body('body').id;p0=d.xpos[b].copy();q=np.zeros(4);mujoco.mju_mat2Quat(q,d.xmat[b].copy());d.mocap_quat[1]=q;d.mocap_pos[1]=p0;weld(r,4,1)
  t0=d.time;r.holder=lambda t,p0=p0,t0=t0:p0+[0,0,.005*min(1,max(0,(t-t0)/1.))];r.run(1.5)
  for l in side:
   i=LEGS.index(l);kb=m.body(l+'_shank').id;knee=d.xpos[kb].copy();Rk=d.xmat[kb].reshape(3,3).copy();ax=Rk@np.array([0,-1.,0])
   total=(QF-math.radians(3))-d.qpos[r.j(l+'_shank_joint')];Kx=np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]])
   q=np.zeros(4);mujoco.mju_mat2Quat(q,Rk.reshape(-1));d.mocap_pos[0]=knee;d.mocap_quat[0]=q;weld(r,i,1);t1=d.time
   def hand(t,t1=t1,total=total,Kx=Kx,Rk=Rk,knee=knee):
    a=total*min(1,max(0,(t-t1-.5)/3.));R=(np.eye(3)+math.sin(a)*Kx+(1-math.cos(a))*Kx@Kx)@Rk;q=np.zeros(4);mujoco.mju_mat2Quat(q,R.reshape(-1));return knee,q
   r.hand=hand;r.run(4.);weld(r,i,0);r.hand=None;r.run(.5)
  t2=d.time;r.holder=lambda t,p0=p0,t2=t2:p0+[0,0,.005-.0015*min(1,max(0,(t-t2)/1.5))];r.run(2.);weld(r,4,0);r.holder=None;r.run(1.)
def press_only(r):
 m,d=r.m,r.d
 for l in ('FL','FR'):
  c=d.site_xpos[m.site(l).id].copy();top=c+[0,0,.029]
  path=[(0,top+[0,0,.04]),(1.5,top+[0,0,.005]),(4.,np.array([c[0],c[1],.025])),(5.,np.array([c[0],c[1],.025])),(6.5,top+[0,0,.05]),(7.,top+[0,0,.25]),(7.4,np.array([0,.5,.5]))]
  t0=d.time
  def block(t,path=path,t0=t0):
   t-=t0
   for (a,pa),(b,pb) in zip(path,path[1:]):
    if t<=b:return pa+(pb-pa)*min(1,max(0,(t-a)/(b-a)))
   return path[-1][1]
  r.block=block;r.run(7.5)
 r.block=None
out=dict(scope=__doc__.split('\n')[0],gearbox_hold=dict(breakaway_Nm=B,stiffness_Nm_rad=K,damping_Nms_rad=.04,elastic_limit_deg=round(math.degrees(B/K),2)),table_friction=MU,results=[])
for mode in ('declared','native','coulomb'):
 r=Run(mode);r.place({l:QF for l in LEGS});r.run(10.);out['results'].append(dict(mode=mode,assay='posture held 10 s',complete_legs=r.complete(),complete_front=r.complete_front(),**r.state()))
 r=Run(mode);r.place({l:QF+math.radians(UP[l]) for l in LEGS});r.run(10.);out['results'].append(dict(mode=mode,assay='start pose 10 s',complete_legs=r.complete(),complete_front=r.complete_front(),**r.state()))
r=Run('declared');r.place({l:QF+math.radians(UP[l]) for l in LEGS});r.run(2.);hold_and_turn(r);r.run(10.)
out['results'].append(dict(mode='declared',assay='hold the chassis, turn each front lower leg, then 10 s hands-off',complete_legs=r.complete(),complete_front=r.complete_front(),**r.state()))
r=Run('declared');r.place({l:QF+math.radians(UP[l]) for l in LEGS});r.run(2.);press_only(r);r.run(10.)
out['results'].append(dict(mode='declared',assay='press each front toe pad straight down, nothing held, then 10 s hands-off',complete_legs=r.complete(),complete_front=r.complete_front(),**r.state()))
for x in out['results']:print(x['mode'],'|',x['assay'],'| complete legs',x['complete_legs'],'| worst off-posture',max(abs(v) for l in x['off_posture_deg'].values() for v in l.values()),'deg')
(P/'fixture/passive-model-assays.json').write_text(json.dumps(out,indent=1))
