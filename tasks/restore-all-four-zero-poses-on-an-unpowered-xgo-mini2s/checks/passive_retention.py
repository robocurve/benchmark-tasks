from pathlib import Path
import mujoco,numpy as np,json,math
import sys
A=Path(__file__).resolve().parents[1]/'assets';P=Path(sys.argv[1]) if len(sys.argv)>1 else Path('passive-assay-output');P.mkdir(parents=True,exist_ok=True)
summaries=[]
for mode in ('native','coulomb','brake12','brake16','brake20'):
 m=mujoco.MjModel.from_xml_path(str(A/'offline.xml'));d=mujoco.MjData(m);m.actuator_gainprm[:]=0;m.actuator_biasprm[:]=0
 m.body_mass[m.body('base_model').id]+=.166186;mujoco.mj_setConst(m,d);d.qpos[:3]=0
 for n,q in [('arm_thigh_joint',-1.57),('arm_calf_joint',1.35),*((leg+'_calf_joint',math.radians(15)) for leg in ('fl','fr','br','bl'))]:d.qpos[m.jnt_qposadr[m.joint(n).id]]=q
 mujoco.mj_forward(m,d)
 def low(g):
  mid=m.geom_dataid[g];v=m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]]
  return (v@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g])[:,2].min()
 feet=[m.geom(leg+'_calf_collision').id for leg in ('fl','fr','br','bl')];d.qpos[2]-=min(map(low,feet));mujoco.mj_forward(m,d)
 hinges=np.where(m.jnt_type==mujoco.mjtJoint.mjJNT_HINGE)[0];qa=m.jnt_qposadr[hinges];va=m.jnt_dofadr[hinges];rest=d.qpos[qa].copy()
 legs=[m.joint(leg+'_'+j+'_joint').id for leg in ('fl','fr','br','bl') for j in ('hip','thigh','calf')];lqa=m.jnt_qposadr[legs];target=np.array([0,0,15]*4);trace=[]
 if mode=='coulomb':m.dof_frictionloss[va]=.12
 if mode.startswith('brake'):m.dof_damping[va]=.08
 for k in range(30001):
  if mode.startswith('brake'):
   cap=int(mode[-2:])/100
   if k%50==0:rest=d.qpos[qa]-np.clip(d.qpos[qa]-rest,-cap/4,cap/4)
   d.qfrc_applied[va]=np.clip(4*(rest-d.qpos[qa]),-cap,cap)
  if k%50==0:trace.append(dict(t=float(d.time),q=np.rad2deg(d.qpos[lqa]).tolist(),feet=[float(low(g)) for g in feet],root=d.qpos[:3].tolist()))
  mujoco.mj_step(m,d)
 tail=np.array([r['q'] for r in trace[-41:]]);s=dict(mode=mode,max_error_deg=float(abs(tail-target).max()),range2s_deg=float(np.ptp(tail,axis=0).max()),final=trace[-1]);summaries.append(s);print(s,flush=True)
 (P/('calibration-'+mode+'.json')).write_text(json.dumps(dict(summary=s,trace=trace),indent=2))
(P/'calibration-retention-summary.json').write_text(json.dumps(summaries,indent=2))
