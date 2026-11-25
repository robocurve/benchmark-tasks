from pathlib import Path
import argparse,json,time,traceback
p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--setup-only',action='store_true');a=p.parse_args();OUT=Path(a.out);OUT.mkdir(exist_ok=True,parents=True)
from isaaclab.app import AppLauncher
app=AppLauncher(headless=True,enable_cameras=True,device='cuda:0').app
try:
 import numpy as np,torch,carb
 carb.settings.get_settings().set('/rtx/post/histogram/enabled',False)
 from PIL import Image
 from scipy.spatial.transform import Rotation
 import window
 window.install()
 from yam_arena import station,ik,YAM_URDF
 from yam_arena.urdf import forward_kinematics
 from yam_arena.constants import READY,ARM_JOINTS
 import yam_arena.scene as sm
 from yam_arena.embodiment import BimanualYamEmbodiment
 station.ITEMS={};original=sm.station_assets
 def assets(cam):
  d=original(cam);d.pop('bottle',None);return d
 sm.station_assets=assets
 emb=BimanualYamEmbodiment(enable_cameras=True,overview_cam=True,contact_sensors=False,rig=station.rig(),cam_width=640,cam_height=352)
 emb.camera_config.left_cam.spawn.clipping_range=(.0002,20.)
 emb.camera_config.overview_cam.width=800;emb.camera_config.overview_cam.height=600;emb.camera_config.overview_cam.spawn.focal_length=26.
 env=sm.make_env(emb,scene='station',control_hz=20,device='cuda:0');b=env.unwrapped;env.reset()
 from isaaclab.sim.utils.stage import get_current_stage
 from pxr import UsdPhysics
 stage=get_current_stage();links={side:next(str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and arm in str(p.GetPath()) and str(p.GetPath()).endswith('/gripper')) for side,arm in [('left','LeftArm'),('right','RightArm')]}
 from brush_geometry import BRUSH
 from window import WINDOW_BODY
 from contact_model import brushing_patch
 physics_dt=float(b.sim.get_physics_dt())
 pv=b.sim.physics_manager.get_physics_sim_view();brushview=pv.create_rigid_body_view(BRUSH);paneview=pv.create_rigid_body_view(WINDOW_BODY);contacts=pv.create_rigid_contact_view(BRUSH,filter_patterns=[WINDOW_BODY]);mountcontacts=pv.create_rigid_contact_view(BRUSH,filter_patterns=[links['left']])
 P=Path(__file__).parent;plan=json.loads((P/'plan_pressure.json').read_text());ql=plan['left_q'];qr=np.array(READY,dtype=float);native=station.wrist_camera('left');Rc=Rotation.from_quat(native.rot_xyzw).as_matrix()
 rgba=np.array(Image.open(P/'dirty.png').convert('RGBA'));coverage=rgba[:,:,3].astype(float)/255;initial=coverage.copy();n=coverage.shape[0];x,y=np.meshgrid(np.linspace(-.007,.007,n),np.linspace(-.006,.006,n));previous=None;previous_rotation=None;rows=[];k=0;contact_steps=0;sliding_m=0.;opening=1.;pane_pose_error_max=0.;cleaned_gap_max=0.;grasp_achieved=False;first_grasp_step=None;grasp_window=[];pick=json.loads((P/'pickup_plan.json').read_text());grasp_reference=np.array(pick['brush_root_position']);max_lift=0.
 def joints(side):
  q=b.scene[side+'_robot'].data.joint_pos;q=q.torch if hasattr(q,'torch') else q;return q[0,:6].cpu().numpy()
 def optics():
  T=forward_kinematics(YAM_URDF,dict(zip(ARM_JOINTS,joints('left'))))['gripper'];T[:3,3]+=np.array(station.rig().bases['left']);return T[:3,3]+T[:3,:3]@native.pos,T[:3,:3]@Rc
 def move(target,steps,label,grip=None):
  global qr,k,previous,previous_rotation,coverage,contact_steps,sliding_m,opening,max_lift,grasp_achieved,first_grasp_step,grasp_window,pane_pose_error_max,cleaned_gap_max
  q0=qr.copy();g0=opening
  for j in range(1,steps+1):
   qr=q0+(target-q0)*ik.smoothstep(j/steps)
   opening=g0+(float(grip)-g0)*ik.smoothstep(j/steps) if grip is not None else opening
   env.step(torch.tensor([[*ql,1.,*qr,opening]],device=b.device,dtype=torch.float32));k+=1
   C,R=optics();tcp,Rr=ik.tcp_pose(joints('right'),station.rig().bases['right']);bt=brushview.get_transforms();bt=bt.numpy() if hasattr(bt,'numpy') else bt.cpu().numpy();bp=bt[0,:3].copy();Br=Rotation.from_quat(bt[0,3:7]).as_matrix();face=bp+Br[:,2]*.077;loc=(face-C)@R;sep=-loc[2]-.00315;max_lift=max(max_lift,float(bp[2]-grasp_reference[2]))
   # Grasp is latched only after 0.5 s of stable jaw-relative carriage >=4 cm above rest.
   grip_error=float(np.linalg.norm(bp-(tcp-Rr[:,2]*.004)));relative=Rr.T@(bp-(tcp-Rr[:,2]*.134))
   if opening<=.25 and float(bp[2]-grasp_reference[2])>=.04 and grip_error<=.015:
    grasp_window.append(relative.copy());grasp_window=grasp_window[-10:]
    if len(grasp_window)==10 and np.max(np.linalg.norm(np.array(grasp_window)-np.mean(grasp_window,axis=0),axis=1))<=.010:
     if not grasp_achieved:first_grasp_step=k
     grasp_achieved=True
   else:grasp_window=[]
   # Read the mounted glass body's actual frame and its own contact force.
   pt=paneview.get_transforms();pt=pt.numpy() if hasattr(pt,'numpy') else pt.cpu().numpy();pc=pt[0,:3].copy();Pr=Rotation.from_quat(pt[0,3:7]).as_matrix();pane_error=float(np.linalg.norm(pc-(C-R[:,2]*.003)));pane_pose_error_max=max(pane_pose_error_max,pane_error)
   local_face=(face-pc)@Pr;relative_rotation=Pr.T@Br;sep=-local_face[2]-.00015
   f=contacts.get_contact_force_matrix(physics_dt);f=f.numpy() if hasattr(f,'numpy') else f.cpu().numpy();fv=f.reshape(-1,3).sum(axis=0);force=float(np.linalg.norm(fv));normal_force=float(abs(fv@Pr[:,2]))
   mf=mountcontacts.get_contact_force_matrix(physics_dt);mf=mf.numpy() if hasattr(mf,'numpy') else mf.cpu().numpy();mount_force=float(np.linalg.norm(mf.reshape(-1,3).sum(axis=0)))
   active,slip,dose=brushing_patch(x,y,local_face,relative_rotation,previous,previous_rotation,normal_force)
   if pane_error>.0002:active[:]=False;dose[:]=0.
   touching=bool(np.any(active));travel=float(np.mean(slip[active])) if touching else 0.
   if touching:
    contact_steps+=1;coverage*=np.exp(-dose);sliding_m+=travel
    dx=x-local_face[0];dy=y-local_face[1];dz=-.00015-local_face[2];gap=dx*relative_rotation[0,2]+dy*relative_rotation[1,2]+dz*relative_rotation[2,2];cleaned_gap_max=max(cleaned_gap_max,float(np.max(abs(gap[active]))))
   previous=local_face.copy();previous_rotation=relative_rotation.copy()
   if k%5==0:
    if touching and travel>1e-5:
     texture=OUT/f'coverage_{k:04d}.png';out=rgba.copy();out[:,:,3]=np.clip(np.round(coverage*255),0,255).astype('uint8');Image.fromarray(out).save(texture)
     for mesh,shader,tex in window.DIRT:tex.GetInput('file').Set(str(texture.resolve()))
    eye=(np.array([.6,-.65,1.12]) if label in ['initial empty hand','approach brush','descend to brush','grasp brush','lift brush'] else C-R[:,2]*.035+R[:,0]*.12+R[:,1]*.085);b.scene['overview_cam'].set_world_poses_from_view(torch.tensor([eye],device=b.device,dtype=torch.float32),torch.tensor([bp if label in ['initial empty hand','approach brush','descend to brush','grasp brush','lift brush'] else C-R[:,2]*.003],device=b.device,dtype=torch.float32))
    row=dict(step=k,phase=label,face_xy_m=loc[:2].tolist(),gap_m=float(sep),force_N=force,pane_normal_force_N=normal_force,mount_force_N=mount_force,pane_pose_error_m=pane_error,active_glass_pixels=int(active.sum()),contact=touching,sliding_m=sliding_m,residual=float(coverage.sum()/initial.sum()),right_actual_q=joints('right').tolist(),left_actual_q=joints('left').tolist(),brush_position=bp.tolist(),brush_quat_xyzw=bt[0,3:7].tolist(),grip=opening,grasp_achieved=grasp_achieved,grasp_error_m=float(np.linalg.norm(bp-(tcp-Rr[:,2]*.004))));rows.append(row)
    for name in ['top_cam','left_cam','right_cam','overview_cam']:
     cam=b.scene[name];rgb=cam.data.output['rgb'];rgb=rgb.torch if hasattr(rgb,'torch') else rgb;Image.fromarray(rgb[0,:,:,:3].cpu().numpy().astype('uint8')).save(OUT/f'{name}_{k:04d}.png')
   if k%20==0:print(k,label,'gap mm',round(sep*1000,2),'force',round(force,3),'residual',round(float(coverage.sum()/initial.sum()),3),flush=True);(OUT/'trace.json').write_text(json.dumps(rows))
 move(qr,25,'initial empty hand',1.)
 if a.setup_only:
  import shutil
  shutil.copy2(OUT/'top_cam_0020.png',OUT/'setup.png');env.close();raise SystemExit(0)
 for q in pick['approach'][1:]:move(np.array(q),70,'approach brush')
 for q in pick['descent']:move(np.array(q),5,'descend to brush')
 move(qr,35,'grasp brush',0.)
 for q in list(reversed(pick['descent'][:-1]))+[pick['pregrasp_q']]:move(np.array(q),5,'lift brush')
 (OUT/'pickup_result.json').write_text(json.dumps({'max_lift_m':max_lift,'last_brush_position':rows[-1]['brush_position'],'grasp_error_m':rows[-1]['grasp_error_m'],'no_grasp_attachment':True},indent=2))
 if rows[-1]['grasp_error_m']>.015 or max_lift<.045 or not grasp_achieved:raise RuntimeError('physical pickup failed; do not continue wiping')
 # Retarget the saved brush-tip path using the actual physical grasp transform.
 from scipy.optimize import least_squares
 actual=rows[-1];tcp0,Rg0=ik.tcp_pose(np.array(actual['right_actual_q']),station.rig().bases['right']);gp0=tcp0-Rg0[:,2]*.134
 Rb0=Rotation.from_quat(actual['brush_quat_xyzw']).as_matrix();local_root=Rg0.T@(np.array(actual['brush_position'])-gp0);local_R=Rg0.T@Rb0;local_tip=local_root+local_R@np.array([0,0,.077])
 (OUT/'grasp_transform.json').write_text(json.dumps({'root_local':local_root.tolist(),'rotation_local':local_R.tolist(),'tip_local':local_tip.tolist()},indent=2))
 for key in ['approach','wipe']:
  transformed=[]
  for oldq in plan[key]:
   oldq=np.array(oldq);tp,Tr=ik.tcp_pose(oldq,station.rig().bases['right']);face_target=tp+Tr[:,2]*.073;Rtarget=Tr@local_R.T;tcp_target=face_target-Rtarget@(local_tip-np.array([0,0,.134]))
   def loss(q):
    pp,rr=ik.tcp_pose(q,station.rig().bases['right']);return np.r_[pp-tcp_target,.15*Rotation.from_matrix(Rtarget@rr.T).as_rotvec()]
   fit=least_squares(loss,oldq,bounds=(ik.LOWER+1e-7,ik.UPPER-1e-7),max_nfev=100)
   if np.linalg.norm(loss(fit.x)[:3])>.001:raise RuntimeError('grasp retarget IK failure')
   transformed.append(fit.x.tolist())
  plan[key]=transformed
 move(np.array(READY),60,'transport brush')
 for q in plan['approach'][1:]:move(np.array(q),55,'approach protective window')
 wipe=plan['wipe'][:60]+plan['wipe'][36:60]*2
 for q in wipe:move(np.array(q),4,'contact and wipe')
 # Fixed demonstration schedule: three additional round trips.
 extra_cycles=3
 for cycle in range(extra_cycles):
  for q in plan['wipe'][36:60]:move(np.array(q),4,'repeat existing wipe stroke')
 for q in plan['wipe'][60:]:move(np.array(q),4,'withdraw brush')
 # Persist and display the actual final surface state for post-trial photography.
 final_rgba=rgba.copy();final_rgba[:,:,3]=np.clip(np.round(coverage*255),0,255).astype('uint8');Image.fromarray(final_rgba).save(OUT/'final_contamination.png')
 for _,_,tx in window.DIRT:tx.GetInput('file').Set(str((OUT/'final_contamination.png').resolve()))
 move(qr,30,'observe after withdrawal')
 (OUT/'trace.json').write_text(json.dumps(rows));(OUT/'result.json').write_text(json.dumps({'scope':'empty-hand brush pickup then contact wiping','max_lift_m':max_lift,'no_grasp_attachment':True,'extra_wipe_cycles':extra_cycles,'contact_footprint_m':[.007,.025],'residual':float(coverage.sum()/initial.sum()),'contact_steps':contact_steps,'tangential_travel_m':sliding_m,'grasp_achieved':grasp_achieved,'first_grasp_step':first_grasp_step,'score_thresholds':{'grasp':.25,'recovery_25pct':.5,'recovery_50pct':.75,'recovery_90pct':1.},'test_card_in_scene':True,'glass_front_above_bezel_m':.0006,'physics_dt':physics_dt,'contact_force_target':'camera_window_body','pane_pose_error_max_m':pane_pose_error_max,'cleaned_contact_gap_max_m':cleaned_gap_max,'steps':k},indent=2));env.close()
except Exception:
 (OUT/'error.txt').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
finally:app.close()
