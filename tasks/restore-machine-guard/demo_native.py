"""Closed-loop, physically executed YAM reference candidate.

Native Isaac/Inspect only. No direct writes to the guard articulation, no
teleporting joints, no scorer state injection. Commands only the 14 YAM joints
(6 arm joints + gripper per side); actual guard motion must arise from contact.

The reference is intentionally UNQUALIFIED until it earns a native score of 1.0.
"""
from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import geometry as g
from reference_policy import DoorClosurePlanner, BoltInsertionPlanner, ReferenceAbort, door_grip_command, ready_for_latch, keeper_clearance, can_insert, ALIGN_RAD

HZ=g.CONTROL_HZ
DEBUG=bool(os.getenv('GUARD_DEBUG'))


class Arm:
    def __init__(self,side):
        from yam_arena import ik,station
        from yam_arena.constants import READY
        self.side=side;self.base=np.asarray(station.BASES[side],float)
        self.q=np.asarray(READY,float)
        self.grip=1.0
        self.bias=np.zeros(6)
        self.measured=self.q.copy()
        self.releasing=False
        self.limits=np.column_stack((ik.LOWER,ik.UPPER))
        self.pos,self.rot=ik.tcp_pose(self.q,self.base)
        self.home=self.pos.copy(),self.rot.copy()
        self.max_ik_error_mm=0.0

    def aim(self,target,rot=None):
        from yam_arena import ik
        rot=self.rot if rot is None else rot
        q,err,_=ik.solve(target,ik.matrix_to_quat_xyzw(rot),
                         q_init=self.q,base_pos=self.base,iters=90)
        self.max_ik_error_mm=max(self.max_ik_error_mm,float(err)*1000)
        if not np.all(np.isfinite(q)) or not np.isfinite(err) or err>0.030:
            raise RuntimeError(f'{self.side}: IK invalid or >30mm error: {err}')
        self.q,self.pos,self.rot=np.asarray(q,float),np.asarray(target,float),np.asarray(rot,float)

    def move(self,target,seconds,grip=None,rot=None):
        from yam_arena import ik
        rot=self.rot if rot is None else rot
        p0=self.pos.copy();r0=self.rot.copy();g0=self.grip
        g1=g0 if grip is None else float(grip)
        n=max(1,round(float(seconds)*HZ))
        for i in range(1,n+1):
            u=ik.smoothstep(i/n)
            self.grip=g0+(g1-g0)*u
            self.aim(p0+(np.asarray(target,float)-p0)*u,ik.slerp_matrix(r0,rot,u))
            yield

    def feedback(self,measured):
        # Integral compensation for steady joint tracking error under handle load.
        # Only target robot joints; clip both rate and total trim.
        self.measured=np.asarray(measured[:6]).copy()
        if self.releasing:return
        error=self.q-self.measured
        self.bias=np.clip(self.bias+np.clip(0.05*error,-0.003,0.003),-0.28,0.28)

    def command(self):
        return np.r_[np.clip(self.q+self.bias,self.limits[:,0],self.limits[:,1]),self.grip]


def wait(seconds):
    for _ in range(int(round(seconds*HZ))):yield


def together(*gs):
    gs=[iter(g) for g in gs]
    while gs:
        for gen in list(gs):
            try:next(gen)
            except StopIteration:gs.remove(gen)
        yield


class Context:
    def __init__(self,emb,task):
        self.emb=emb;self.task=task
        self.left=Arm('left');self.right=Arm('right')
        self.phase='setup'
        self.commanded_hinge_rad=None
        self.commanded_bolt_m=None
        self.insertion_input=None

    def joint(self,name):
        from yam_arena.scene import object_states
        return float(object_states(self.emb._env,['machine_guard'])['machine_guard'].joints[name])

    def held(self,side):
        from grasp_evidence import verified
        sample=self.task._LAST_CONTACT
        return bool(sample) and verified(sample,side)

    def door_support_n(self):
        from grasp_evidence import supporting_force
        sample = self.task._LAST_CONTACT
        return supporting_force(sample['signatures']['left'], sample['physics_steps'])

    def measured_arms(self):
        from guardkit import arm_joints
        from yam_arena import ik
        from yam_arena.constants import ALL_JOINTS
        result = {}
        for arm in (self.left,self.right):
            q=arm_joints(self.emb._env,arm.side,ALL_JOINTS)
            arm.feedback(q)
            tcp,rot=ik.tcp_pose(q[:6],arm.base)
            center = (door_handle(self.joint('hinge'),self.task.ROOT_Z) if arm.side=='left'
                      else bolt_tab(self.joint('bolt_slide'),self.task.ROOT_Z))
            result[arm.side]=dict(joints=q.tolist(),tcp_from_measured_fk_m=tcp.tolist(),
                                  handle_in_jaws_mm=(1000*rot.T@(center-tcp)).tolist(),
                                  tracking_error_mm=float(np.linalg.norm(tcp-arm.pos)*1000))
        return result

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack
        return Action(data=pack(self.left.command(),self.right.command()))


def door_handle(theta,rootz):
    # Robot-side orange door handle (mid-point in native guard coordinates).
    dx,dy,dz=g.DOOR_HANDLE_LOCAL
    c,s=math.cos(theta),math.sin(theta)
    return np.array([g.HINGE_X+dx*c-dy*s,g.HINGE_Y+dx*s+dy*c,rootz+dz])


def bolt_tab(stroke,rootz):
    return np.array([g.HINGE_X-.014, g.HINGE_Y-.218+stroke, rootz+.068])+g.BOLT_PAD_LOCAL


def _require_closure(ctx):
    angle=ctx.joint('hinge')
    if abs(angle)>math.radians(2.5):
        raise ReferenceAbort(f'door_not_closed_before_latch: {math.degrees(angle):.2f} deg')
    if ctx.joint('switch_slide')<g.SWITCH_ACTIVE:
        raise ReferenceAbort('door_closed_but_switch_not_activated')


def script(ctx,mode):
    from yam_arena.demo import grip_rotation
    left,right=ctx.left,ctx.right
    z=ctx.task.ROOT_Z
    def left_rot(a):
        return grip_rotation(a-math.pi/2)
    right_rot=grip_rotation(0.)
    def leaf(a):
        return door_handle(a,z)+[0,0,.015]
    def bolt(b):
        return bolt_tab(b,z)+[0,0,.015]
    if mode=='idle':
        ctx.phase='idle'
        while True:
            yield  # stationary robot command throughout the caller's actual action budget
    def acquire(arm,point,rot,name):
        ctx.phase='approach_'+name
        yield from arm.move(point+[0,0,.065],2,grip=1.,rot=rot)
        yield from arm.move(point,1.6,grip=1.,rot=rot)
        ctx.phase='grasp_'+name
        yield from arm.move(point,.85,grip=(.32 if name=='door' else .35),rot=rot)
        consecutive=0
        for _ in range(round(.30*HZ)):
            yield
            consecutive=consecutive+1 if ctx.held(arm.side) else 0
        # Different native pad tolerances may require a smaller aperture.
        # Remain over the stationary handle. Never command a working stroke
        # to search for a missing second contact.
        while consecutive < 3 and arm.grip > .18:
            yield from arm.move(point,.15,grip=max(.18,arm.grip-.02),rot=rot)
            for _ in range(4):
                yield
                consecutive=consecutive+1 if ctx.held(arm.side) else 0
        if consecutive<3:
            last=getattr(ctx.task,'_LAST_CONTACT',{})
            reason=('no_physx_contact_reports' if last and not last.get('report_headers')
                    else 'bilateral_contact_missing')
            raise ReferenceAbort(name+'_'+reason)
    trim=0.
    def maintain():
        nonlocal trim
        # Reduce aperture gradually under load. Preserve a tighter native grasp.
        left.grip=door_grip_command(min(left.grip,max(.23,left.grip-.006)),ctx.door_support_n())
        a=ctx.joint('hinge')
        # Unload the closed stop if its pose masks a shaft/eyelet mismatch.
        # Move only the left robot target. No force command goes to the fixture.
        if keeper_clearance(a)<.00048:
            correction=float(np.clip(.10*(ALIGN_RAD-a),-.00055,.00055))
        else:
            correction=-.002*a
        trim=float(np.clip(trim+correction,-.028,.025))
        left.aim(leaf(0)+[trim+.006-.42*a,0.,0],left_rot(0))
    def supported(motion):
        for _ in motion:
            if mode!='skip_close':
                if not ctx.held('left'):
                    raise ReferenceAbort('lost_door_support_during_bolt_grasp')
                maintain()
            yield
    if mode!='skip_close':
        start=ctx.joint('hinge')
        yield from acquire(left,leaf(start),left_rot(start),'door')
        from yam_arena import ik
        def lateral_in_jaws():
            tcp,rot=ik.tcp_pose(left.measured,left.base)
            return float(rot[:,1] @ (door_handle(ctx.joint('hinge'),z)-tcp))
        grasp_y=lateral_in_jaws()
        travel_grip=max(.18,left.grip-.08)
        ctx.phase='close_door'
        planner=DoorClosurePlanner(start, step_rad=math.radians(0.22), timeout_ticks=430)
        while not planner.complete:
            if not ctx.held('left'):
                raise ReferenceAbort('lost_door_pinch_before_motion')
            # Take up compliance gradually as the opening spring loads the arm.
            # Native contact-force feedback can tighten past the feedforward target.
            preload=min(left.grip,max(travel_grip,left.grip-.002))
            left.grip=door_grip_command(preload,ctx.door_support_n())
            angle=planner.step(ctx.joint('hinge'),ctx.joint('bolt_slide'),ctx.joint('switch_slide'))
            ctx.commanded_hinge_rad=angle
            actual=ctx.joint('hinge')
            delta=1-actual/start
            tang=np.array([-math.sin(actual)*g.DOOR_HANDLE_LOCAL[0]-math.cos(actual)*g.DOOR_HANDLE_LOCAL[1],
                           math.cos(actual)*g.DOOR_HANDLE_LOCAL[0]-math.sin(actual)*g.DOOR_HANDLE_LOCAL[1],0.])
            tang=tang/np.linalg.norm(tang)
            rotation=left_rot(actual)
            recenter=rotation[:,1]*np.clip(lateral_in_jaws()-grasp_y,-.006,.006)
            left.aim(leaf(angle)+delta*tang*0.01+recenter,rotation)
            yield
        ctx.phase='seat_door'
        for _ in range(100):
            maintain()
            yield
            if ready_for_latch(ctx.joint('hinge'),ctx.joint('switch_slide')) and abs(ctx.joint('hinge'))<math.radians(.25):break
        aligned=0
        for _ in range(200):
            maintain()
            yield
            if ctx.held('left') and can_insert(ctx.joint('hinge'),ctx.joint('switch_slide')):
                aligned+=1
            else:
                aligned=0
            if aligned>=50:
                break
        if aligned<50:
            raise ReferenceAbort('keeper_alignment_not_acquired')
        _require_closure(ctx)
    if mode!='skip_bolt':
        yield from supported(acquire(right,bolt(ctx.joint('bolt_slide')),right_rot,'bolt'))
        ctx.phase='advance_bolt' if mode=='reference' else 'adversarial_advance_bolt'
        if mode=='reference':
            planner=BoltInsertionPlanner(ctx.joint('bolt_slide'))
            while not planner.complete:
                if not ctx.held('left'):
                    raise ReferenceAbort('lost_door_support_before_bolt_motion')
                if not ctx.held('right'):
                    raise ReferenceAbort('lost_bolt_pinch_before_motion')
                h=ctx.joint('hinge')
                sw=ctx.joint('switch_slide')
                if not can_insert(h,sw):
                    # No forward action against an obstructed slot. Resume only
                    # after a stationary right hand and three aligned samples.
                    right.aim(bolt(ctx.joint('bolt_slide')),right_rot)
                    aligned=0
                    for _ in range(100):
                        ctx.phase='hold_bolt_alignment'
                        ctx.commanded_bolt_m=None
                        maintain()
                        yield
                        if not (ctx.held('left') and ctx.held('right')):
                            raise ReferenceAbort('lost_pinch_during_alignment')
                        aligned=aligned+1 if can_insert(ctx.joint('hinge'),ctx.joint('switch_slide')) else 0
                        if aligned>=3:
                            break
                    if aligned<3:
                        raise ReferenceAbort('keeper_alignment_lost')
                    ctx.phase='advance_bolt'
                    planner.target_m=ctx.joint('bolt_slide')
                    continue
                bolt_now=ctx.joint('bolt_slide')
                ctx.insertion_input=dict(hinge=h,switch_slide=sw,bolt_slide=bolt_now)
                pos=planner.step(h,bolt_now,sw)
                ctx.commanded_bolt_m=pos
                right.aim(bolt(pos),right_rot)
                maintain()
                yield
        else:
            for pos in np.linspace(0,.048,110):
                ctx.commanded_bolt_m=float(pos)
                right.aim(bolt(pos),right_rot)
                yield
    if mode=='skip_bolt':
        ctx.phase='unlatched_return'
        for a in np.linspace(0,g.OPEN_RAD,195):
            left.aim(leaf(a),left_rot(a))
            yield
    from yam_arena import ik
    from yam_arena.constants import READY
    for arm in (left,right):
        if (arm is left and mode=='skip_close') or (arm is right and mode=='skip_bolt'):continue
        ctx.phase='release_'+arm.side
        # Rebase robot targets to the sensed joints before opening. Removing an
        # integral load with closed jaws can drag a spring-return bolt backwards.
        arm.releasing=True
        arm.q=arm.measured.copy()
        arm.bias[:]=0.
        arm.pos,arm.rot=ik.tcp_pose(arm.q,arm.base)
        yield from arm.move(arm.pos,.65,grip=.52)
        offset=[-.024,.029,.034] if arm is left else [-.012,-.006,.038]
        yield from arm.move(arm.pos+offset,1.4)
        ctx.phase='withdraw_'+arm.side
        initial=arm.q.copy()
        for u in np.linspace(0,1,68):
            arm.q=initial+(np.asarray(READY)-initial)*ik.smoothstep(u)
            arm.pos,arm.rot=ik.tcp_pose(arm.q,arm.base)
            arm.grip=.52+.48*ik.smoothstep(u)
            yield
        yield from wait(.5)
    ctx.phase='retention_check'
    yield from wait(3)


def run(outdir,mode='reference',max_steps=1200,video=True):
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from PIL import Image
    import imageio.v2 as imageio

    out=Path(outdir);out.mkdir(parents=True,exist_ok=True)
    video_dir=out/'videos'
    if video: video_dir.mkdir(parents=True,exist_ok=True)
    os.environ['GUARD_SETTLE_MEDIA_DIR']=str(out/'settle') if video else ''
    os.environ['GUARD_TRACE_JSONL']=str(out/'guard_trace.jsonl')
    os.environ['GUARD_RESET_PHASES_JSONL']=str(out/'guard_reset_phases.jsonl')
    emb=yam_arena_embodiment(task=str(HERE/'task.py'),object_jitter=0,
                             control_hz=HZ,cam_width=640,cam_height=352)
    task=emb.task.module
    obs=emb.reset(Scene(id='guard',instruction=task.INSTRUCTION),seed=0)
    # The native environment settles before reset returns. Assemble its
    # independently captured frames into a real setup/settling video even when
    # no-action smoke qualification later fails.
    settle_media=[]
    if video:
        for camera in ('top_cam','left_cam','right_cam'):
            frames=sorted((out/'settle').glob(f'{camera}_*.png'))
            if frames:
                with imageio.get_writer(str(video_dir/f'settle_{camera}.mp4'),
                     fps=HZ, codec='libx264', macro_block_size=1,
                     pixelformat='yuv420p', output_params=['-movflags','+faststart']) as writer:
                    for frame_path in frames:
                        writer.append_data(np.asarray(Image.open(frame_path).convert('RGB')))
                settle_media.append(f'videos/settle_{camera}.mp4')
    contacts=getattr(task,'_CONTACTS',None)
    if contacts is not None:
        binding=dict(robot_roots=contacts.roots,expected_fingertip_actors=contacts.fingers,
                     physics_device=contacts.physics_device, physics_dt_s=contacts.dt,physics_steps_per_action=contacts.steps)
        (out/'contact_bindings.json').write_text(json.dumps(binding,indent=2)+'\n')
    ctx=Context(emb,task)
    # Stream video during simulation: the original implementation retained
    # up to 1201 frames x 3 RGB cameras in RAM, >2 GB at 640x352.
    writers={}
    setup_written=False
    frame_count=0
    latest=None
    def grab(o):
        nonlocal setup_written,frame_count,latest
        if not video:
            return
        combined={}
        for k,v in o.images.items():
            frame=np.asarray(v)
            if frame.ndim != 3 or frame.shape[2] not in (3,4):
                raise RuntimeError(f'Unexpected camera frame shape {k}: {frame.shape}')
            frame=np.asarray(frame[...,:3],dtype=np.uint8)
            if not setup_written and 'top' in k.lower():
                Image.fromarray(frame).save(out/'setup.png')
                Image.fromarray(frame).save(video_dir/'setup.png')
                setup_written=True
            combined[k]=frame
            if k not in writers:
                writers[k]=imageio.get_writer(str(video_dir/f'{mode}_{k}.mp4'), fps=HZ,
                    codec='libx264',macro_block_size=1,
                    pixelformat='yuv420p', output_params=['-movflags','+faststart'])
            writers[k].append_data(frame)
        if all(k in combined for k in ('top_cam','left_cam','right_cam')):
            frame_count+=1
            latest=combined['top_cam'].copy()
            key='review_three_cameras'
            if key not in writers:
                writers[key]=imageio.get_writer(str(video_dir/'review_three_cameras.mp4'), fps=HZ,
                     codec='libx264',macro_block_size=1,
                     pixelformat='yuv420p', output_params=['-movflags','+faststart'])
            writers[key].append_data(np.concatenate([combined[k] for k in
                ('top_cam','left_cam','right_cam')],axis=1))
    grab(obs)
    gen=script(ctx,mode)
    controller_trace=out/'reference_controller.jsonl'
    controller_failure=None
    steps=0;ended=None;status='max_steps';info={}

    try:
        with controller_trace.open('w', encoding='utf8') as controller_log:
          while steps<max_steps:
            if ended is None:
                try:
                    next(gen)
                except StopIteration:
                    ended=steps
                except ReferenceAbort as exc:
                    controller_failure=str(exc)
                    status='controller_aborted'
                    break
            res=emb.step(ctx.action());steps+=1;info=res.info;grab(res.observation)
            measured=dict(getattr(task,'_LAST_JOINTS',{}))
            controller_log.write(json.dumps({
                'tick':steps,'time_s':steps/HZ,'phase':ctx.phase,
                'measured_joint_positions':measured,
                'measured_arms':ctx.measured_arms(),
                'contact':dict(task._LAST_CONTACT),
                'commanded_hinge_rad':ctx.commanded_hinge_rad,
                'commanded_bolt_m':ctx.commanded_bolt_m,
                'insertion_input':ctx.insertion_input if ctx.phase=='advance_bolt' else None,
                'keeper_clearance_mm':1000*keeper_clearance(measured['hinge']) if 'hinge' in measured else None,
                'left_tcp_requested_world_m':ctx.left.pos.tolist(),
                'right_tcp_requested_world_m':ctx.right.pos.tolist(),
                'left_gripper_command':ctx.left.grip,
                'right_gripper_command':ctx.right.grip,
                'scorer_failed':task.MONITOR.failed,
            },sort_keys=True)+'\n')
            if task.MONITOR.failed and mode=='reference':
                controller_failure='native_scorer_failure:'+task.MONITOR.failed
                status='native_safety_abort'
                break
            if steps%20==0:
                s=task.MONITOR.diagnostic()
                print(json.dumps({'step':steps,'phase':ctx.phase,**s,'joints':measured}),flush=True)
            if res.terminated:
                status=res.termination_reason or 'terminated';break
            if ended is not None and steps-ended>=50:
                status='script_ended';break
    finally:
        for writer in writers.values():
            writer.close()
    if latest is not None:
        Image.fromarray(latest).save(out/'terminal_top.png')
    s=task.MONITOR.diagnostic()
    result={'score':s['score'],'success':bool(s['complete'] and s['failed'] is None),
            'steps':steps,'termination':status,'monitor':s,'synchronized_frames':frame_count,
            'ik_error_mm':{'left':ctx.left.max_ik_error_mm,'right':ctx.right.max_ik_error_mm},
            'mode':mode,'native_simulator':'Isaac Lab-Arena/YAM',
            'controller':{'phase':ctx.phase,'failure':controller_failure,
                          'status':status,'trace':'reference_controller.jsonl'},
            'physics_profile':task.PHYSICS_PROFILE,
            'reset_joint_positions':dict(getattr(task,'_RESET_JOINTS',{})),
            'reset_phases':list(getattr(sys.modules.get('guardkit'), '_RESET_PHASES', [])),
            'final_joint_positions':dict(getattr(task,'_LAST_JOINTS',{})),
            'video_recorded':bool(writers) or bool(settle_media),
            'video_files':settle_media+[('videos/review_three_cameras.mp4' if k=='review_three_cameras' else f'videos/{mode}_{k}.mp4') for k in writers]}
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--out',required=True)
    p.add_argument('--mode',choices=['reference','idle','skip_close','skip_bolt'],default='reference')
    p.add_argument('--max-steps',type=int,default=1200)
    p.add_argument('--no-video',action='store_true')
    p.add_argument('--expect-idle-zero',action='store_true',
                   help='Fail if fixture auto-closes or actuates without YAM manipulation')
    a=p.parse_args()
    try:
        result=run(a.out,a.mode,a.max_steps,not a.no_video)
    except Exception as exc:
        import traceback
        failure={'mode':a.mode,'error_type':type(exc).__name__,
                 'error':str(exc),'traceback':traceback.format_exc()}
        Path(a.out).mkdir(parents=True,exist_ok=True)
        (Path(a.out)/'failure.json').write_text(json.dumps(failure,indent=2)+'\n')
        traceback.print_exc()
        sys.exit(5)
    if a.mode=='reference' and (not result['success'] or result.get('controller',{}).get('failure')):
        sys.exit(2) # fail-closed: never proceed to hosted Dream
    if a.expect_idle_zero:
        j=result.get('final_joint_positions',{})
        bad=(result['score']!=0.0 or result['monitor']['failed'] is not None
             or result['monitor']['closed'] or result['monitor']['switch']
             or result['monitor']['latched'] or not math.isfinite(j.get('hinge',float('nan')))
             or abs(j['hinge']-g.OPEN_RAD)>math.radians(8))
        if bad:
            print('FAIL: no-action baseline moved the guard or scoring progressed; '
                  'the native fixture is not qualified.',file=sys.stderr)
            sys.exit(6)
