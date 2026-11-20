"""Offline self-test (no simulator): `python native/full_entry.py`.

Checks the plan against the arm's joint and speed limits and the run limit, the outcome scorer on synthetic
traces for every stage and for wrong paths, the sleeve's material law, and the reference's held-tool check.
"""
import json,copy
import numpy as np
from scipy.spatial.transform import Rotation

def run():
 import task_runtime as rt
 import full_entry as fe
 import plan_reference as ref
 from yam_arena import ik
 from yam_arena.analysis import link_hulls
 from yam_arena.constants import SHOULDER,WRIST
 from primary_score import PrimaryScore
 from engineering_parameters import PassivePlasticRoot,hinge_parameters
 from material_grasp import MaterialGraspGuard,GraspFailure
 G=rt.G;parts={p['name']:p for p in G['parts']};checks=0
 def ok(cond,what):
  nonlocal checks
  if not cond:raise AssertionError(what)
  checks+=1
 # -- plan ---------------------------------------------------------------------------------
 rows=json.load(open(rt.GROOT/'full_plan.json'));ok(rows['source_geometry_sha256']==rt.GSHA,'plan/geometry hash')
 rows=rows['rows'];ok(len(rows)+fe.SETTLE+(ref.STRIKES+2)*fe.EXTRA_MAX*0+60<=1440,'plan fits the 1440-control limit with 60 spare')
 speed=np.array([SHOULDER['velocity']]*3+[WRIST['velocity']]*3)/20
 for a,b in zip(rows,rows[1:]):
  for s in('left','right'):
   q=np.array(b[s+'_q']);ok(np.all(q>=ik.LOWER-1e-9)and np.all(q<=ik.UPPER+1e-9),'joint limit '+b['phase'])
   ok(np.all(np.abs(q-np.array(a[s+'_q']))<=speed+1e-9),'joint speed '+b['phase'])
 for v in('no_strike','leave_punch'):
  alt=fe.effective_plan(rows,v);ok(not any(r['phase'].startswith('strike_')for r in alt)if v=='no_strike'else not any(r['phase']=='punch_withdraw'for r in alt),'variant '+v)
  for a,b in zip(alt,alt[1:]):ok(np.all(np.abs(np.array(b['left_q'])-a['left_q'])<=speed)and np.all(np.abs(np.array(b['right_q'])-a['right_q'])<=speed),'variant continuity '+v)
 # -- scorer on synthetic traces --------------------------------------------------------------
 names=sorted(link_hulls());origin=np.array(G['frame_origin_m'])
 def pose(p,R=np.eye(3)):return list(map(float,p))+Rotation.from_matrix(R).as_quat().tolist()
 def frame(control,wedge=0.,punch=None,hammer=None,anchor=(0,0,0),arm_at=(0.,0.,2.),wall=(1,1,1,1),speed=0.,blow=0.):
  b={p['name']:dict(pose_xyzw=pose(p['pos'],Rotation.from_quat(p['quat_xyzw']).as_matrix()),native_velocity=[0.]*6)for p in G['parts']}
  b['anchor_ring']['pose_xyzw']=pose(origin+anchor)
  b['setting_wedge']['pose_xyzw']=pose(origin+[0,0,-wedge])
  b['punch']['pose_xyzw']=punch if punch is not None else pose(origin+[0,0,-wedge])
  if hammer is not None:b['hammer']['pose_xyzw']=hammer
  if speed:b['punch']['pose_xyzw'][0]+=speed*.05*(control%2)   # shuttles back and forth each control
  robots={s:dict(body_names=names,body_link_pose_w=[pose(np.array(arm_at)+[0,.5*(1 if s=='left'else-1),.1*i])for i in range(9)])for s in('left','right')}
  pairs={f'petal{k}_concrete':dict(counts=[[4 if wall[k]else 0]],entries=[])for k in range(4)}
  return dict(control=control,hammer_punch_peak_N=blow,rigid_bodies=b,robots=robots,material={f'petal_{k}':dict(plastic_angle=0.)for k in range(4)},contacts=dict(pairs=pairs,unavailable_views={}))
 stowed=pose(ref.STOW,np.column_stack([[0,0,-1.],[0,1.,0],[1.,0,0]]))   # punch axis along +x on the V blocks
 lifted=pose(origin+[0,0,.06])
 def play(frames):
  s=PrimaryScore();out=[]
  for i,kw in enumerate(frames,1):s.observe(frame(i,**kw),G);out.append(s.value)
  return out
 set_=dict(wedge=.0018);struck=[{},{},dict(set_,blow=60.)]   # untouched, untouched, then one blow drives the plug 1.8 mm
 ok(play([{}]*30)[-1]==0.,'untouched start scores 0')
 ok(play(struck+[set_]*5)[-1]==.5,'shoulder flush with the plug driven by a blow scores 0.5')
 ok(play([{},{}]+[set_]*8)[-1]==0.,'plug pushed down with no hammer on the punch scores 0')
 ok(play([{},{},dict(wedge=.0010,blow=60.)]+[dict(wedge=.0010)]*5)[-1]==0.,'plug driven only 1.0 mm scores 0')
 ok(play([{},{},dict(set_,blow=60.,wall=(1,1,0,1))]+[dict(set_,wall=(1,1,0,1))]*5)[-1]==0.,'a petal off the bore scores 0')
 v=play(struck+[set_]*3+[dict(set_,punch=lifted)]*12+[dict(set_,punch=stowed)]*25)
 ok(v[5]==.5 and v[17]==.75 and v[-1]==1.,'0.5 -> 0.75 -> 1.0 sequence %s'%[v[5],v[17],v[-1]])
 ok(play([{}]*3+[dict(punch=lifted)]*12+[dict(punch=stowed)]*25)[-1]==0.,'wrong path: tools put away, anchor never struck, scores 0')
 ok(play(struck+[set_]*40)[-1]==.5,'wrong path: anchor set, punch left in it, stays 0.5')
 ok(play(struck+[dict(set_,punch=lifted)]*40)[-1]==.75,'punch out but not on its rest stays 0.75')
 off=pose(np.array(parts['hammer']['pos'])+[.03,0,0])
 ok(play(struck+[dict(set_,punch=stowed,hammer=off)]*40)[-1]==.75,'hammer 30 mm off its cradle position stays 0.75')
 near=tuple(np.array(parts['hammer']['pos'])+[.1,0,.03])
 ok(play(struck+[dict(set_,punch=stowed,arm_at=near)]*40)[-1]==.75,'a robot link within 20 mm of the hammer stays 0.75')
 ok(play(struck+[dict(set_,punch=stowed,speed=.02)]*40)[-1]==.75,'a tool still moving at 20 mm/s stays 0.75')
 ok(play(struck+[dict(set_,punch=stowed,anchor=(0,0,.002))]*40)[-1]==0.,'anchor pulled 2 mm out of the hole scores 0')
 # -- sleeve material law ---------------------------------------------------------------------
 hp=hinge_parameters();m=PassivePlasticRoot(hp['stiffness_Nm_per_rad'],hp['yield_moment_Nm']);yield_angle=hp['yield_moment_Nm']/hp['stiffness_Nm_per_rad']
 ok(m.observe(.5*yield_angle,1.)['plastic_angle']==0.,'elastic below yield')
 r=m.observe(3*yield_angle,2.);ok(abs(r['plastic_angle']-2*yield_angle)<1e-12,'plastic set above yield')
 r=m.observe(2.5*yield_angle,3.);ok(abs(r['plastic_angle']-2*yield_angle)<1e-12 and r['plastic_dissipation_J']>0,'set is kept on unloading')
 # -- held-tool check -----------------------------------------------------------------------
 def held(c,turn_deg,touch=True):
  hand=pose([0,0,1.]);tool=pose([0,0,1.134],Rotation.from_euler('x',turn_deg,degrees=True).as_matrix())
  return dict(control=c,robots=dict(right=dict(body_names=names,body_link_pose_w=[hand if n=='gripper'else pose([0,0,3.])for n in names])),rigid_bodies=dict(hammer=dict(pose_xyzw=tool)))
 g=MaterialGraspGuard();g.observe(held(1,0.),'right','hammer',True)
 for c,turn in enumerate([12.,40.,40.,3.],2):g.observe(held(c,turn),'right','hammer',True)
 try:
  for c in range(6,10):g.observe(held(c,40.),'right','hammer',True)
  ok(False,'a tool turned 40 degrees for three controls must stop the reference')
 except GraspFailure:checks+=1
 print('self-test passed: %d checks'%checks)
