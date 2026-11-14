"""Record native YAM diagnostics, frozen-plan demos or direct Dream trials.

Requires the pinned team simulator. Dream receives standard observations.
An explicitly labelled scripted feasibility demo may use measured tool poses
to command ordinary robot motors. Saved execution is independently judged.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import traceback
import signal
import shutil
import numpy as np


def array(value):
    value=getattr(value,'torch',value)
    return value.detach().cpu().numpy() if hasattr(value,'detach') else np.asarray(value)


def main(*, default_policy='diagnostic', default_steps=60, default_review=False):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--steps',type=int,default=default_steps)
    p.add_argument('--physics-hz',type=int,choices=(240,480,960),default=240,
                   help='Native integration rate; control and cameras remain at20Hz')
    p.add_argument('--tool-substep-trace',action='store_true',
                   help='Record every native tool/material input for independent replay')
    p.add_argument('--policy',choices=('diagnostic','demo','dream'),default=default_policy)
    p.add_argument('--plan',type=Path,help='Optional precompiled fixed action plan; no object feedback')
    p.add_argument('--joint-feedback',action='store_true',help='Bounded demo PI tracking from standard joint observations only')
    p.add_argument('--state-assisted-demo',action='store_true',help='Disclosed native tool-pose feedback for the scripted demo only')
    p.add_argument('--tracking-proportional',type=float,default=1.5)
    p.add_argument('--tracking-integral',type=float,default=2.)
    p.add_argument('--tracking-slew',type=float,default=.006,help='Maximum change of demo joint-feedback bias per action, radians; ordinary action bounds still apply')
    p.add_argument('--external-forces-once-diagnostic',action='store_true',help='Matched PhysX force-integration control; material and actuator settings stay unchanged')
    p.add_argument('--judging-views',action='store_true',default=default_review,help='Record close-up review views; never policy inputs')
    a=p.parse_args()
    if not 1<=a.steps<=2400:p.error('Trial limited to 2400 control steps')
    if not 0<=a.tracking_proportional<=4 or not 0<=a.tracking_integral<=10 or not 0<a.tracking_slew<=.04:
        p.error('Joint tracking gains outside the bounded diagnostic range')
    if a.policy=='demo' and a.plan is None:p.error('Demo requires a frozen joint plan')
    if a.policy=='dream' and (a.plan or a.joint_feedback or a.state_assisted_demo):p.error('Dream uses its own ordinary actions')
    if a.state_assisted_demo and a.policy!='demo':p.error('State assistance is limited to the labelled scripted demo')
    plan=None
    if a.plan:
        plan=json.loads(a.plan.read_text())
        valid_kind=(plan.get('no_object_feedback') or
                    (a.state_assisted_demo and plan.get('schema')=='state-assisted-recipe-v1'))
        if plan.get('control_hz')!=20 or not valid_kind or not plan.get('frames'):
            p.error('Expected fixed 20 Hz plan, or an explicitly enabled state-assisted recipe')
        actions=np.asarray([f['action14'] for f in plan['frames']],float)
        if actions.shape!=(len(plan['frames']),14) or not np.isfinite(actions).all():p.error('Invalid plan actions')
        feedforward=np.asarray([f.get('joint_feedforward14',[0.]*14) for f in plan['frames']],float)
        if (feedforward.shape!=actions.shape or not np.isfinite(feedforward).all()
                or np.max(np.abs(feedforward))>.16 or np.any(feedforward[:,[6,13]]!=0)):
            p.error('Invalid bounded joint feedforward')
        if np.any(feedforward) and (a.policy!='demo' or not a.state_assisted_demo):
            p.error('Calibrated feedforward is limited to the disclosed scripted demo')
        a.steps=min(a.steps,len(actions))
    a.out.mkdir(parents=True,exist_ok=False);start=time.monotonic()
    frozen=a.out/'task-source';frozen.mkdir()
    for source in Path(__file__).parent.glob('*.py'):shutil.copyfile(source,frozen/source.name)
    metadata_path=Path(__file__).with_name('task.json')
    if not metadata_path.exists():metadata_path=Path(__file__).with_name('task.draft.json')
    metadata=json.loads(metadata_path.read_text())
    shutil.copyfile(metadata_path,frozen/'task-description.json')
    if a.plan:shutil.copyfile(a.plan,frozen/'fixed-plan.json')
    report={'status':'running','robot_demo':a.policy=='demo','policy':a.policy,
            'full_score_demonstrated':False,'preview_steps':a.steps,'control_hz':20,
            'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in frozen.glob('*') if p.is_file()},
            'plan_sha256':hashlib.sha256(a.plan.read_bytes()).hexdigest() if a.plan else None,
            'joint_feedback':a.joint_feedback,
            'state_assisted_demo':a.state_assisted_demo,
            'controller_assistance':plan.get('assistance') if plan and a.state_assisted_demo else None,
            'joint_feedforward':plan.get('joint_feedforward') if plan else None,
            'external_forces_once_diagnostic':a.external_forces_once_diagnostic}
    app=None;video=None;coupling=None;trace=[];substep_trace=[];dream_policy=None;request_audit=[];session_audit={'session_count':0}
    def progress(phase,step=0):
        report.update(phase=phase,completed_control_steps=step,wall_seconds=time.monotonic()-start)
        (a.out/'preview.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({'phase':phase,'control_steps':step,'wall_seconds':round(report['wall_seconds'],2)}),flush=True)
        if trace:(a.out/'trace.json').write_text(json.dumps(trace,indent=2)+'\n')
    def terminate(signum,frame):
        raise SystemExit(f'Bounded preview stopped by signal {signum}')
    signal.signal(signal.SIGTERM,terminate)
    try:
        progress('application launch')
        from isaaclab.app import AppLauncher
        app=AppLauncher(headless=True,enable_cameras=True).app
        import omni.physics.tensors as tensors
        import imageio.v2 as imageio
        from PIL import Image
        import torch
        from yam_arena import inspect_embodiment as wrapper
        wrapper._APP=app
        emb=wrapper.YamArenaEmbodiment(task=str(Path(__file__).with_name('scene.py')),
            object_jitter=0.,control_hz=20,cam_width=224,cam_height=224)
        progress('build YAM scene and initialize renderer')
        from yam_arena import scene as adapter
        original_make_env=adapter.make_env
        original_cfg_callback=adapter.env_cfg_callback
        original_dt=adapter.PHYSICS_DT
        adapter.PHYSICS_DT=1/a.physics_hz
        if a.external_forces_once_diagnostic:
            def configure_callback(*args,**kwargs):
                configure=original_cfg_callback(*args,**kwargs)
                def override(cfg):
                    cfg=configure(cfg)
                    if cfg.sim.physics is None:
                        from isaaclab_physx.physics import PhysxCfg
                        cfg.sim.physics=PhysxCfg()
                    cfg.sim.physics.enable_external_forces_every_iteration=False
                    return cfg
                return override
            adapter.env_cfg_callback=configure_callback
        if a.judging_views:
            from review_views import configure
            def make_env(embodiment,**kwargs):
                configure(embodiment)
                return original_make_env(embodiment,**kwargs)
            adapter.make_env=make_env
        try:
            emb._ensure_sim(None)
        finally:
            adapter.make_env=original_make_env
            adapter.env_cfg_callback=original_cfg_callback
            adapter.PHYSICS_DT=original_dt
        progress('native tensor and contact views')
        import sys
        # The task loader may give scene.py a generated module name. Its hook
        # closure owns the exact runtime, avoiding a second scene import.
        from yam_arena import scene as adapter
        namespace=adapter.task_object.__globals__
        roots,coupling=namespace['ROOTS'],namespace['COUPLING']
        report['tool_body_paths']={name:roots[name] for name in ('barrel','plunger')}
        sim=tensors.create_simulation_view('torch');sim.set_subspace_roots('/')
        barrel=sim.create_rigid_body_view(roots['barrel']);plunger=sim.create_rigid_body_view(roots['plunger'])
        import omni.usd
        from pxr import UsdPhysics,PhysxSchema
        stage=omni.usd.get_context().get_stage()
        scenes=[prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
        if len(scenes)!=1:raise RuntimeError('Expected one native physics scene')
        phys=PhysxSchema.PhysxSceneAPI(scenes[0])
        report['effective_external_forces_every_iteration']=bool(phys.GetEnableExternalForcesEveryIterationAttr().Get())
        if a.external_forces_once_diagnostic and report['effective_external_forces_every_iteration']:
            raise RuntimeError('Requested once-per-step integration did not reach native scene')
        from contact_audit import NativeContactAudit
        other_contact=NativeContactAudit(omni.usd.get_context().get_stage(),sim,coupling.material,
            excluded_body_paths=(),
            allowed_shape_paths=[roots['barrel']+f'/NozzleSector{i:02d}' for i in range(24)])
        base=emb._base;write=base.scene.write_data_to_sim
        report['physics_dt_s']=float(base.physics_dt)
        if not np.isclose(base.physics_dt,1/a.physics_hz,rtol=0,atol=1e-10):
            raise RuntimeError('Requested native integration rate did not reach the simulator')
        report['tool_substep_trace']=a.tool_substep_trace
        report['tool_substep_recording_precision']='Native float32 poses/velocities, exact float64 scalar inputs and material outputs'
        report['material_boundary_velocity']=coupling.boundary_velocity
        report['material_parameters']=vars(coupling.material.p).copy()
        report['dispenser_parameters']=vars(coupling.dispenser).copy()
        report['tool_local_coms']={name:array(view.get_coms())[0].tolist()
            for name,view in (('barrel',barrel),('plunger',plunger))}
        report['robot_body_names']={name:list(base.scene[name].body_names) for name in ('left_robot','right_robot')}
        def step_material(*args,**kwargs):
            out=write(*args,**kwargs)
            bp=array(barrel.get_transforms())[0].copy()
            bv=array(barrel.get_velocities())[0].copy()
            pp=array(plunger.get_transforms())[0].copy()
            fb,fp=coupling.advance(bp,bv,pp,float(base.physics_dt))
            if a.tool_substep_trace:
                substep_trace.append(np.r_[coupling.material.time_s,float(base.physics_dt),bp.astype(float),bv.astype(float),pp.astype(float),
                    coupling.last_piston_m,coupling.last_piston_velocity_m_s,coupling.last_reaction_n,
                    coupling.last_seal_force_n,coupling.material.emitted_m3])
            for view,force,torque in ((barrel,fb,coupling.last_contact_torque),(plunger,fp,np.zeros(3))):
                view.apply_forces_and_torques_at_position(force_data=torch.as_tensor(force,dtype=torch.float32,device=base.device).reshape(1,3),
                    torque_data=torch.as_tensor(torque,dtype=torch.float32,device=base.device).reshape(1,3),
                    position_data=torch.as_tensor(bp[:3] if view is barrel else pp[:3],dtype=torch.float32,device=base.device).reshape(1,3),
                    indices=torch.tensor([0],dtype=torch.int32,device=base.device),is_global=True)
            other_contact.update(coupling.material.time_s)
            return out
        base.scene.write_data_to_sim=step_material
        progress('episode reset')
        emb._env.reset()
        coupling.reset()
        coupling.prime_boundary(array(barrel.get_transforms())[0])
        substep_trace.clear()
        other_contact.events.clear()
        video=imageio.get_writer(a.out/'preview.mp4',fps=20,codec='libx264',pixelformat='yuv420p',
                                ffmpeg_params=['-movflags','+faststart'],macro_block_size=1)
        def snapshot(step):
            rgb=array(base.scene['top_cam'].data.output['rgb'])[0,:,:,:3].astype(np.uint8)
            if a.judging_views:
                from review_views import composite_frame
                rgb=composite_frame(base)
            video.append_data(rgb)
            if not step:Image.fromarray(rgb).save(a.out/'setup.png')
            trace.append({'step':step,
                          'time_s':step/20,
                          'native_physics_step':int(base.sim._physics_step_count),
                          'observed_action14':np.asarray(emb._observation().state['joint_pos'],float).tolist(),
                          'robot_joints':{name:array(base.scene[name].data.joint_pos)[0].tolist() for name in ('left_robot','right_robot')},
                          'robot_links_xyzw':{name:array(base.scene[name].data.body_link_pose_w)[0].tolist() for name in ('left_robot','right_robot')},
                          'rigid_bounds_m':dict(other_contact.world_bounds),
                          'barrel_pose_xyzw':array(barrel.get_transforms())[0].tolist(),
                          'plunger_pose_xyzw':array(plunger.get_transforms())[0].tolist(),
                          'piston_m':coupling.last_piston_m,'pressure_reaction_n':coupling.last_reaction_n,
                          'nozzle_contact_force_n':coupling.last_contact_force.tolist(),
                          'nozzle_contact_torque_nm':coupling.last_contact_torque.tolist(),
                          'emitted_ml':coupling.material.emitted_m3*1e6,
                          'material_contact_invalid':bool(other_contact.events)})
        snapshot(0)
        if a.policy=='dream':
            from inspect_robots import Scene
            from dream_policy import make_policy
            instruction=metadata['instruction']
            dream_policy=make_policy(request_audit,session_audit,rows_per_request=8)
            dream_policy.bind(emb.info)
            dream_policy.reset(Scene(id='pipe-shell-border',instruction=instruction))
            report['dream_model']=dream_policy.info.checkpoint
        progress('fixed action preview',0)
        from joint_tracking import JointTracking
        tracking=JointTracking(proportional=a.tracking_proportional,integral=a.tracking_integral,slew_limit=a.tracking_slew)
        from scripted_servo import ScriptedServo
        servo=ScriptedServo() if a.state_assisted_demo else None
        from yam_arena.constants import SHOULDER,WRIST,GRIPPER,FINGER_OPEN
        step_limit=np.tile(np.array([SHOULDER['velocity']/20]*3+[WRIST['velocity']/20]*3+
                                   [GRIPPER['velocity']/abs(FINGER_OPEN)/20]),2)
        previous_applied=np.asarray(emb._observation().state['joint_pos'],float)
        report['joint_tracking_parameters']={k:getattr(tracking,k) for k in ('kp','ki','limit','slew')}
        for i in range(a.steps):
            observation=emb._observation()
            if dream_policy is not None:
                chunk=dream_policy.act(observation)
                if len(chunk.actions)!=1:raise RuntimeError('Expected one ordinary Dream control action')
                reference=np.asarray(chunk.actions[0].data,float)
            else:
                reference=emb._ready[0].detach().cpu().numpy() if plan is None else actions[i]
            observed=np.asarray(observation.state['joint_pos'],float)
            assisted=reference if servo is None else servo.command(plan['frames'][i],reference,observed,
                array(barrel.get_transforms())[0],array(plunger.get_transforms())[0])
            requested=tracking.action(assisted,observed,enabled=a.joint_feedback and i>=340)
            joint_feedforward=np.zeros(14) if plan is None else feedforward[i]
            requested+=joint_feedforward
            applied=np.clip(np.clip(requested,previous_applied-step_limit,previous_applied+step_limit),emb._low,emb._high)
            previous_applied=applied.copy()
            action=torch.tensor([applied],device=base.device,dtype=torch.float32)
            emb._env.step(action);emb._t=(i+1)/20;coupling.update_surface();snapshot(i+1)
            trace[-1].update(reference_action14=reference.tolist(),requested_action14=requested.tolist(),
                             assisted_action14=assisted.tolist(),joint_feedforward14=joint_feedforward.tolist(),
                             applied_action14=applied.tolist(),joint_tracking_bias12=tracking.bias.tolist())
            if servo is not None:trace[-1]['scripted_servo']=servo.last
            if (i+1)%20==0 or i+1==a.steps:progress('fixed action preview',i+1)
        video.close();video=None
        for name in ('top_cam','left_cam','right_cam'):
            Image.fromarray(array(base.scene[name].data.output['rgb'])[0,:,:,:3].astype(np.uint8)).save(a.out/f'final-{name}.png')
        (a.out/'trace.json').write_text(json.dumps(trace,indent=2)+'\n')
        coupling.material.save(a.out/'material.npz')
        report.update(balance=coupling.material.balance(),other_contact=other_contact.summary())
        if a.judging_views:
            progress('final judging photographs',a.steps)
            from review_views import capture_final
            report['judging_cameras']=capture_final(base,a.out)
        report.update(status='complete',balance=coupling.material.balance(),score=None,
                      other_contact=other_contact.summary(),
                      note='Execution complete; independently rescore saved material and trajectory before any task-success claim.')
    except BaseException as exc:
        report.update(status='failed',error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc());raise
    finally:
        if video is not None:video.close()
        if coupling is not None and not (a.out/'material.npz').exists():
            coupling.material.save(a.out/'material.npz')
            report['partial_material_saved']=True
        if dream_policy is not None:
            from dream_policy import public_transcript
            (a.out/'dream-transcript.json').write_text(json.dumps(public_transcript(dream_policy),indent=2)+'\n')
        report.update(dream_requests=len(request_audit),dream_session_audit=session_audit)
        if request_audit:(a.out/'dream-requests.json').write_text(json.dumps(request_audit,indent=2)+'\n')
        report['wall_seconds']=time.monotonic()-start
        (a.out/'preview.json').write_text(json.dumps(report,indent=2)+'\n')
        if trace:(a.out/'trace.json').write_text(json.dumps(trace,indent=2)+'\n')
        if substep_trace:
            np.savez_compressed(a.out/'tool-substeps.npz',values=np.asarray(substep_trace,dtype=np.float64),
                columns=np.asarray(['time_after_s','dt_s']+
                    ['barrel_'+x for x in ('x','y','z','qx','qy','qz','qw')]+
                    ['barrel_'+x for x in ('vx','vy','vz','wx','wy','wz')]+
                    ['plunger_'+x for x in ('x','y','z','qx','qy','qz','qw')]+
                    ['piston_m','piston_velocity_m_s','pressure_reaction_n','seal_force_n','emitted_m3']))
        if app:app.close()


if __name__=='__main__':main()
