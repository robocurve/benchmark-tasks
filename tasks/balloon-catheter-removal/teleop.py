"""Native Isaac XR controller input -> bounded YAM joint targets + camera MP4.

No ROS/FFW controller, paid inference, scripted object motion or fake grasp.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import time

from isaaclab.app import AppLauncher

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--output-dir',type=Path,required=True)
p.add_argument('--smoke-seconds',type=float,default=0.)
p.add_argument('--no-xr',action='store_true',help='Recorder/controller plumbing smoke test only')
p.add_argument('--probe-controller',action='store_true',help='Synthetic XR-format input; smoke test only')
AppLauncher.add_app_launcher_args(p)
args=p.parse_args()
args.enable_cameras=True
app=AppLauncher(args).app

import numpy as np
import warp as wp
import carb
import omni.usd
import omni.replicator.core as rep
from pxr import UsdGeom, UsdPhysics, Gf
from isaaclab.sim import SimulationContext, SimulationCfg
import isaaclab.sim.utils.stage as stage_utils
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY, TCP_OFFSET, SHOULDER, WRIST, GRIPPER
from scene_runtime import SceneRuntime
from mechanics import rotation, point
from scoring import TrialScorer
import subprocess

out=args.output_dir
out.mkdir(parents=True,exist_ok=True)
running=True
def stop(*_):
    global running
    running=False
signal.signal(signal.SIGINT,stop)
signal.signal(signal.SIGTERM,stop)

def save_status(data):
    target=out/'teleop-status.json'
    tmp=target.with_suffix('.tmp')
    tmp.write_text(json.dumps(data,indent=2)+'\n')
    tmp.replace(target)

class Recorder:
    def __init__(self):
        self.process=None
        self.trace=None
        self.frames=0
        self.segments=[]
        self.index=0
        self.log=None
        self.started=None

    def start(self):
        if self.process:return
        self.index+=1
        stem=('smoke' if args.smoke_seconds else 'demo')+f'-{self.index:03d}'
        self.log=(out/(stem+'-ffmpeg.log')).open('w')
        self.process=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','warning','-y',
            '-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r','30','-i','pipe:0',
            '-an','-c:v','libx264','-threads','2','-preset','veryfast','-crf','21',
            '-pix_fmt','yuv420p','-movflags','+faststart',str(out/(stem+'-overview.mp4'))],
            stdin=subprocess.PIPE,stderr=self.log,start_new_session=True)
        self.trace=(out/(stem+'-trajectory.jsonl')).open('w')
        self.frames=0
        self.started=time.monotonic()
        self.segments.append({'video':stem+'-overview.mp4','trajectory':stem+'-trajectory.jsonl',
                              'kind':'plumbing-smoke' if args.smoke_seconds else 'manual-teleoperation',
                              'success':'not evaluated'})

    def frame(self,rgb,evidence):
        if not self.process:return
        data=np.asarray(rgb.get('data',[]) if isinstance(rgb,dict) else rgb)
        if data.shape[:2]!=(720,1280) or data.ndim!=3:return
        if not self.frames:
            import imageio.v3 as iio
            iio.imwrite(out/('setup-'+str(self.index)+'.png'),data[:,:,:3])
        payload=np.ascontiguousarray(data[:,:,:3],dtype=np.uint8).tobytes()
        # Preserve wall-clock timing for human demonstrations. Slow renderer
        # frames are held rather than making the person's motion play faster.
        count=min(120,max(1,round((time.monotonic()-self.started)*30)-self.frames))
        evidence['video_first_frame']=self.frames
        for _ in range(count):self.process.stdin.write(payload)
        self.frames+=count
        evidence['video_last_frame']=self.frames-1
        self.trace.write(json.dumps(evidence,allow_nan=False)+'\n')

    def finish(self):
        if not self.process:return
        self.process.stdin.close()
        rc=self.process.wait(timeout=20)
        self.trace.close();self.log.close()
        self.segments[-1].update(frames=self.frames,encoder_exit_code=rc)
        self.process=None;self.trace=None
        (out/'recordings.json').write_text(json.dumps(self.segments,indent=2)+'\n')
        if rc or self.frames==0:raise RuntimeError('Recording did not produce frames; inspect encoder log')

class Arm:
    def __init__(self,sim,side):
        self.side=side
        self.view=sim.physics_manager._view.create_articulation_view('/World/envs/env_0/'+('LeftArm' if side=='left' else 'RightArm'))
        if self.view.count!=1:raise RuntimeError('Expected exactly one '+side+' articulation')
        self.names=list(self.view.shared_metatype.dof_names)
        self.arm=[self.names.index('joint'+str(i)) for i in range(1,7)]
        self.fingers=[self.names.index('joint7'),self.names.index('joint8')]
        self.links=list(self.view.shared_metatype.link_names)
        self.base=self.view.get_root_transforms().numpy()[0]
        self.target=self.q().copy()
        self.reference=None
        self.last_pose=None
        self.last_squeeze=False
        self.active=False
        self.indices=wp.array([0],dtype=wp.int32,device=sim.device)
        self.device=sim.device
        gains=[SHOULDER if name in ('joint1','joint2','joint3') else
               WRIST if name in ('joint4','joint5','joint6') else GRIPPER for name in self.names]
        for method,key in [('set_dof_stiffnesses','stiffness'),('set_dof_dampings','damping'),
                           ('set_dof_max_forces','effort'),('set_dof_max_velocities','velocity')]:
            # PhysX property setters require CPU tensors, unlike state/target setters.
            values=wp.array(np.array([[g[key] for g in gains]],dtype=np.float32),device='cpu')
            getattr(self.view,method)(values,wp.array([0],dtype=wp.int32,device='cpu'))
        self.calibration_error=self.validate_fk()
        self.gripper_prim=next(p for p in omni.usd.get_context().get_stage().Traverse()
            if str(p.GetPath()).startswith('/World/envs/env_0/'+('LeftArm' if side=='left' else 'RightArm')+'/')
            and p.GetName()=='gripper' and p.HasAPI(UsdPhysics.RigidBodyAPI))

    def q(self):return self.view.get_dof_positions().numpy()[0].astype(float)

    def validate_fk(self):
        measured=self.view.get_link_transforms().numpy()[0,self.links.index('gripper')]
        tcp=point(measured,np.array(TCP_OFFSET))
        predicted,_=ik.tcp_pose(self.q()[self.arm],self.base[:3])
        err=float(np.linalg.norm(tcp-predicted))
        if err>.005:raise RuntimeError(f'{self.side} FK/calibration mismatch {err:.4f} m; control disabled')
        return err

    def hold(self):
        self.reference=None;self.last_pose=None;self.active=False
        self.target=self.q().copy()

    def apply(self,sample):
        q=self.q()
        if not sample or not sample['valid'] or sample['squeeze']<.6:
            if self.active:self.hold()
            return
        xyz=np.asarray(sample['position']);r=rotation(sample['orientation'])
        if self.last_pose is not None and np.linalg.norm(xyz-self.last_pose)>.15:
            self.hold();return
        self.last_pose=xyz
        if self.reference is None:
            tcp,rot=ik.tcp_pose(q[self.arm],self.base[:3])
            self.reference=(xyz.copy(),r.copy(),tcp,rot)
        anchor,anchor_rot,tcp,tcp_rot=self.reference
        delta=np.clip((xyz-anchor)*.7,-.25,.25)
        goal=np.clip(tcp+delta,np.minimum([.04,-.55,.780],tcp),np.maximum([.68,.55,1.45],tcp))
        goal_rot=r@anchor_rot.T@tcp_rot
        solved,pos_err,rot_err=ik.solve(goal,ik.matrix_to_quat_xyzw(goal_rot),q_init=q[self.arm],
                                       base_pos=self.base[:3],iters=10,damping=.05,rot_weight=.15)
        if pos_err<.015 and rot_err<.5:
            self.target[self.arm]+=np.clip(solved-self.target[self.arm],-.02,.02)
        finger=FINGER_OPEN*(1-float(np.clip(sample['trigger'],0,1)))
        self.target[self.fingers]=q[self.fingers]+np.clip(finger-q[self.fingers],-.001,.001)
        self.active=True

    def write(self):
        values=wp.array(np.asarray(self.target[None],dtype=np.float32),device=self.device)
        self.view.set_dof_position_targets(values,self.indices)
        # Match yam_arena.actions: hardware-style gravity compensation, while
        # keeping robot and task bodies under gravity and ordinary contacts.
        forces=self.view.get_gravity_compensation_forces().numpy()
        if forces.shape[1]==len(self.names)+6:forces=forces[:,6:]
        self.view.set_dof_actuation_forces(wp.array(np.asarray(forces,dtype=np.float32),device=self.device),self.indices)

    def render_error(self):
        import usdrt
        measured=self.view.get_link_transforms().numpy()[0,self.links.index('gripper'),:3]
        stage=usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
        prim=stage.GetPrimAtPath(str(self.gripper_prim.GetPath()))
        matrix=prim.GetAttribute('omni:fabric:worldMatrix')
        if matrix and matrix.Get() is not None:
            rendered=matrix.Get().ExtractTranslation()
        else:
            attr=usdrt.Rt.Xformable(prim).GetWorldPositionAttr()
            if not attr or attr.Get() is None:raise RuntimeError('No active Fabric render pose for '+self.side)
            rendered=attr.Get()
        return float(np.linalg.norm(np.asarray(rendered)-measured))

class XRInput:
    def __init__(self):
        import omni.kit.xr.core as xr
        self.xr=xr
        self.core=xr.XRCore.get_singleton()
        settings=xr.XRSettings.get_singleton()
        settings.set_setting('profile/persistent/adjustForUserHeight',False)
        settings.set_setting('profile/stage/customAnchor','/World/QuestEye')
        settings.set_setting('profile/persistent/anchorMode','custom anchor')
        xr.XRCore.request_enable_profile('vr')

    def read(self,side):
        if not self.core.has_input_device('/user/hand/'+side):return None
        d=self.core.get_input_device('/user/hand/'+side)
        if d is None:return None
        desc=d.get_virtual_world_pose_desc('aim' if d.has_pose('aim') else '')
        flags=self.xr.XRPoseValidityFlags
        needed=flags.POSITION_VALID|flags.ORIENTATION_VALID
        valid=(desc.validity_flags&needed)==needed
        m=desc.pose_matrix.RemoveScaleShear()
        q=m.ExtractRotationQuat();v=q.GetImaginary()
        def gesture(name,kind):
            return float(d.get_input_gesture_value(name,kind)) if d.has_input_gesture(name,kind) else 0.
        return {'valid':bool(valid),'position':list(m.ExtractTranslation()),
                'orientation':[float(v[0]),float(v[1]),float(v[2]),float(q.GetReal())],
                'squeeze':gesture('squeeze','value'),'trigger':gesture('trigger','value'),
                'click':gesture('thumbstick','click')>=.5}

recorder=Recorder()
status={'ready':False,'xr_enabled':not args.no_xr,'controllers_connected':False,
        'paid_inference_calls':0,'scope':'manual teleop; no completed-task success claim'}
save_status(status)
try:
    if not omni.usd.get_context().open_stage(str(Path(__file__).with_name('scene.usda'))):
        raise RuntimeError('Could not open task scene')
    for _ in range(20):app.update()
    stage=omni.usd.get_context().get_stage()
    eye=UsdGeom.Camera.Define(stage,'/World/QuestEye')
    eye.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(-.20,0,1.25),Gf.Vec3d(.37,.02,.82),Gf.Vec3d(0,0,1)).GetInverse())
    carb.settings.get_settings().set_bool('/rtx/raytracing/fractionalCutoutOpacity',True)
    stage.OverridePrim('/physicsScene').SetActive(False)
    stage_utils._context.stage=stage
    sim=SimulationContext(SimulationCfg(dt=1/120,device=args.device,use_fabric=True,
                                      physics_prim_path='/TeleopPhysicsScene'))
    sim.reset()
    runtime=SceneRuntime(stage,sim)
    arms={s:Arm(sim,s) for s in ('left','right')}
    # Standard initial robot reset, before any input/recording. Task objects
    # are never repositioned or attached to the fingers to fabricate a grasp.
    for arm in arms.values():
        arm.target[arm.arm]=READY
        arm.target[arm.fingers]=FINGER_OPEN
        arm.view.set_dof_positions(wp.array(np.asarray(arm.target[None],dtype=np.float32),device=arm.device),arm.indices)
        arm.view.set_dof_velocities(wp.zeros((1,len(arm.names)),device=arm.device),arm.indices)
    for _ in range(60):
        for arm in arms.values():
            arm.write()
        runtime.update(1/120)
        sim.step(render=False)
    for arm in arms.values():arm.hold()
    initial_q={s:a.q().copy() for s,a in arms.items()}
    initial_tcp={s:ik.tcp_pose(a.q()[a.arm],a.base[:3])[0] for s,a in arms.items()}
    product=rep.create.render_product('/World/envs/env_0/OverviewCam',(1280,720))
    rgb=rep.AnnotatorRegistry.get_annotator('rgb')
    rgb.attach([product])
    xr=None if args.no_xr else XRInput()
    for _ in range(8):sim.render()
    status.update(ready=True,calibration_error_m={s:a.calibration_error for s,a in arms.items()},
                  recording=False,output=str(out),controls={'squeeze':'hold to move each arm',
                  'trigger':'close gripper while squeezing','right_stick_click':'record toggle',
                  'left_stick_click':'unused; disconnect by physically pulling receiver 12 mm'})
    save_status(status)
    print('ROBOCURVE_QUEST_READY '+json.dumps(status),flush=True)
    old_click={'left':False,'right':False};auto_started=False;last_cmd=None
    start=time.monotonic();step=0;last_save=0.;score_step=0
    scorer=TrialScorer();primary={'score':0.,'success':False}
    while running and app.is_running():
        tick=time.monotonic()
        samples={s:xr.read(s) if xr else None for s in arms}
        real_connected=all(samples[s] and samples[s]['valid'] for s in arms)
        if args.probe_controller:
            samples={s:{'valid':True,'position':[min(step/100,.012),0,0],
                        'orientation':[0,0,0,1],'squeeze':1.,'trigger':.2,'click':False}
                     for s in arms}
        connected=all(samples[s] and samples[s]['valid'] for s in arms)
        status['controllers_connected']=bool(real_connected)
        status['synthetic_input_probe']=bool(args.probe_controller)
        for side in arms:
            sample=samples[side]
            click=bool(sample and sample['valid'] and sample['click'])
            if click and not old_click[side]:
                if side=='right':
                    auto_started=True
                    if recorder.process:recorder.finish()
                    else:recorder.start()
            old_click[side]=click
            arms[side].apply(sample)
            arms[side].write()
        command=out/'command.json'
        if command.exists():
            try:c=json.loads(command.read_text())
            except ValueError:c={}
            if c.get('id') and c['id']!=last_cmd:
                last_cmd=c['id']
                if c.get('action')=='record':
                    auto_started=True
                    if recorder.process:recorder.finish()
                    else:recorder.start()
                if c.get('action')=='detach':status['detach_command']='disabled; physically pull receiver 12 mm'
                status['last_command']=last_cmd
        if not auto_started and (connected or args.smoke_seconds):
            recorder.start();auto_started=True
        for _ in range(2):
            metrics=runtime.update(1/120)
            sim.step(render=False)
        sim.render()
        app.update()
        step+=1
        # 60 Hz teleop/render loop, exactly 20 Hz primary-score updates.
        if step%3==0:
            score_step+=1
            measured_robot={}
            for side,arm in arms.items():
                q=arm.q();gripper=arm.view.get_link_transforms().numpy()[0,arm.links.index('gripper')]
                measured_robot[side]={'joints':q[arm.arm].tolist(),'fingers':q[arm.fingers].tolist(),
                                      'tcp':point(gripper,np.asarray(TCP_OFFSET)).tolist()}
            primary_objects={k:{'lin_vel':v.get_velocities().numpy()[0,:3].tolist(),
                                'ang_vel':v.get_velocities().numpy()[0,3:].tolist()} for k,v in runtime.views.items()}
            primary_poses={k:v.tolist() for k,v in runtime.poses().items()}
            primary=scorer.update(step=score_step,objects=primary_objects,
                root_poses=primary_poses,robot=measured_robot,mechanics=metrics)
            with (out/'primary-trajectory.jsonl').open('a') as primary_log:
                primary_log.write(json.dumps({'step':score_step,'objects':primary_objects,'root_poses':primary_poses,
                    'robot':measured_robot,'mechanics':metrics,'score':primary},allow_nan=False)+'\n')
            with (out/'primary-score.jsonl').open('a') as score_log:
                score_log.write(json.dumps({'step':score_step,'sim_time_s':step/60,**primary})+'\n')
            (out/'primary-result.json').write_text(json.dumps({'kind':'plumbing-smoke' if args.smoke_seconds else 'manual-teleoperation',
                'steps':score_step,'max_steps':3600,'sim_time_s':step/60,**primary},indent=2)+'\n')
        if step%2==0 and recorder.process:
            pixels=rgb.get_data()
            status['rgb_shape']=list(np.asarray(pixels.get('data',[]) if isinstance(pixels,dict) else pixels).shape)
            recorder.frame(pixels,{'step':step,'sim_time_s':step/60,
                'wall_time_s':time.monotonic()-start,'controller':samples,
                'robot':{s:{'joints':a.q().tolist(),'clutch':a.active} for s,a in arms.items()},
                'objects':{k:v.tolist() for k,v in runtime.poses().items()},'mechanics':metrics,
                'primary_score_step':score_step,'primary_score':primary})
        if tick-last_save>1:
            errors={s:a.render_error() for s,a in arms.items()}
            if max(errors.values())>.002:
                for arm in arms.values():arm.hold()
                raise RuntimeError('PhysX/render gripper pose disagreement exceeds 2 mm: '+str(errors))
            status.update(recording=bool(recorder.process),frames=recorder.frames,mechanics=metrics,
                          primary_score=primary,primary_score_step=score_step,
                          elapsed_s=time.monotonic()-start,render_pose_error_m=errors)
            save_status(status);last_save=tick
        if args.smoke_seconds and step/60>=args.smoke_seconds:break
        if score_step>=3600:break
        remaining=1/60-(time.monotonic()-tick)
        if remaining>0:time.sleep(remaining)
    if args.probe_controller:
        moved={s:float(np.max(np.abs(a.q()[a.arm]-initial_q[s][a.arm]))) for s,a in arms.items()}
        error={s:float(np.linalg.norm(ik.tcp_pose(a.q()[a.arm],a.base[:3])[0]-initial_tcp[s]-[.0084,0,0])) for s,a in arms.items()}
        status['controller_probe']={'max_arm_joint_change_rad':moved,'tcp_goal_error_m':error,
                                    'passed':min(moved.values())>.0001 and max(error.values())<.020,
                                    'input':'synthetic XR-format samples, not a headset session'}
        if not status['controller_probe']['passed']:raise RuntimeError('Synthetic controller input did not move both arms')
except Exception as exc:
    status['error']=repr(exc)
    import traceback
    traceback.print_exc()
finally:
    try:recorder.finish()
    except Exception as exc:status['recording_error']=repr(exc)
    status.update(ready=False,stopped=True,recording=False,recordings=recorder.segments)
    save_status(status)
    app.close()
