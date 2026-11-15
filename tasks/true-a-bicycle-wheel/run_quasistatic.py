"""Record an ordinary YAM trial with the disclosed quasistatic wheel model."""
import argparse,gzip,hashlib,json,shutil,time,traceback
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--plan',type=Path)
    p.add_argument('--steps',type=int,default=700)
    p.add_argument('--dream',action='store_true')
    p.add_argument('--tool-servo',action='store_true')
    a=p.parse_args()
    if a.dream and (a.plan or a.tool_servo):p.error('Dream has no scripted feedback')
    if not a.dream and not a.plan:p.error('Demo requires a frozen ordinary-action plan')
    if not 1<=a.steps<=2400:p.error('Maximum2400 control steps')
    plan=json.loads(a.plan.read_text()) if a.plan else None
    if plan and a.steps>len(plan['frames']):p.error('Plan too short')
    a.out.mkdir(parents=True,exist_ok=False);frozen=a.out/'task-source';frozen.mkdir()
    for f in Path(__file__).parent.glob('*.py'):shutil.copyfile(f,frozen/f.name)
    if a.plan:shutil.copyfile(a.plan,frozen/'fixed-plan.json')
    started=time.monotonic();report=dict(status='initializing',model='quasistatic-elastic-wheel',policy='dream' if a.dream else 'scripted demo',
        control_hz=20,physics_hz=240,completed_steps=0,score=None,source_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in frozen.iterdir()})
    def save():
        report['wall_seconds']=time.monotonic()-started
        (a.out/'run.json').write_text(json.dumps(report,indent=2)+'\n')
    save();app=video=trace=policy=None;requests=[];sessions={'session_count':0}
    try:
        from isaaclab.app import AppLauncher
        app=AppLauncher(headless=True,enable_cameras=True,device='cpu').app
        import torch,imageio.v2 as imageio,omni.usd
        from PIL import Image
        import arena_wheel
        arena_wheel.FUNCTIONAL_MODEL=True
        arena_wheel.CLOCKING_RAD_OVERRIDE=np.deg2rad(55)
        arena_wheel.FLAT_PHASE_RAD_OVERRIDE=np.deg2rad(45)
        arena_wheel.FIXTURE_LAYOUT='forward10-right200-lower5'
        arena_wheel.SOLVER_ITERATIONS_OVERRIDE=32
        from yam_arena import inspect_embodiment as wrapper,scene as adapter
        from yam_arena.constants import SHOULDER,WRIST,GRIPPER,FINGER_OPEN
        wrapper._APP=app
        emb=wrapper.YamArenaEmbodiment(task=str(Path(__file__).with_name('task.py')),object_jitter=0.,control_hz=20,cam_width=224,cam_height=224,device='cpu')
        old_dt=adapter.PHYSICS_DT;adapter.PHYSICS_DT=1/240
        old_make=adapter.make_env
        from review_views import configure,composite_frame
        def make(embodiment,**kw):
            configure(embodiment,fixture_layout=arena_wheel.FIXTURE_LAYOUT)
            return old_make(embodiment,**kw)
        adapter.make_env=make
        try:emb._ensure_sim(None)
        finally:adapter.PHYSICS_DT=old_dt;adapter.make_env=old_make
        base=emb._base;emb._env.reset()
        from functional_wheel import install
        runtime=install(base,omni.usd.get_context().get_stage())
        ready=emb._ready[0].detach().cpu().numpy().astype(float)
        for _ in range(10):emb._env.step(torch.as_tensor(ready[None],device=base.device,dtype=torch.float32))
        runtime.reset()
        report['warmup']='0.5 s with ordinary ready joint targets; native free wrench settles on table'
        limits=np.tile([SHOULDER['velocity']/20]*3+[WRIST['velocity']/20]*3+[GRIPPER['velocity']/abs(FINGER_OPEN)/20],2)
        from joint_tracking import JointTracker
        tracker=JointTracker(ready,emb._low,emb._high,limits)
        from tool_pose_servo import ToolPoseServo
        servo=ToolPoseServo(plan,nipple_feedback=True) if a.tool_servo else None
        if a.dream:
            from dream_policy import make_policy
            from inspect_robots import Scene
            policy=make_policy(requests,sessions,rows_per_request=8);policy.bind(emb.info)
            policy.reset(Scene(id='true-a-bicycle-wheel',instruction='Use the spoke key to adjust the wheel until it runs straight, then spin it and leave it freely rotating.'))
        video=imageio.get_writer(a.out/'video.mp4',fps=20,codec='libx264',pixelformat='yuv420p',ffmpeg_params=['-movflags','+faststart'],macro_block_size=1)
        trace=gzip.open(a.out/'trace.jsonl.gz','wt',compresslevel=1)
        previous=ready.copy();report['status']='running'
        for step in range(a.steps+1):
            observation=emb._observation();state=arena_wheel.state(emb._env,'wheel_station')
            if step:
                nominal=(np.asarray(policy.act(observation).actions[0].data,float) if policy else np.array(plan['frames'][step-1]['action14']))
                target=nominal.copy()
                if servo:
                    spoke=servo.target_spoke(step)
                    pose=None if spoke is None else state.body_poses[list(state.body_names).index(f'wheel/nipple_{spoke:02d}')]
                    target=servo.command(step,target,np.asarray(observation.state['joint_pos']),state.body_poses[list(state.body_names).index('wrench')],pose)
                issued=tracker.command(target,np.asarray(observation.state['joint_pos'])) if not policy else np.clip(target,previous-limits,previous+limits)
                issued=np.clip(issued,emb._low,emb._high)
                emb._env.step(torch.as_tensor(issued[None],device=base.device,dtype=torch.float32));previous=issued.copy()
                emb._t=step/20;observation=emb._observation();state=arena_wheel.state(emb._env,'wheel_station')
            row=dict(step=step,time_s=step/20,action14=previous.tolist(),joint_pos=np.asarray(observation.state['joint_pos']).tolist(),
                body_names=list(state.body_names),body_poses=state.body_poses.tolist(),body_velocities=state.body_velocities.tolist(),
                material=runtime.model.record(),stage=plan['frames'][step-1]['stage'] if plan and step else 'reset',
                servo=servo.last if servo else None)
            trace.write(json.dumps(row)+'\n');trace.flush()
            frame=composite_frame(base);video.append_data(frame)
            if step in (0,a.steps):Image.fromarray(frame).save(a.out/('setup.png' if step==0 else 'final.png'))
            report['completed_steps']=step
            if step%20==0:report['material']=runtime.model.record();save()
        report.update(status='complete',material=runtime.model.record())
    except BaseException as exc:
        report.update(status='failed',error=str(exc),traceback=traceback.format_exc())
    finally:
        if trace:trace.close()
        if video:video.close()
        if policy:
            from dream_policy import public_transcript
            (a.out/'dream-transcript.json').write_text(json.dumps(public_transcript(policy),indent=2)+'\n')
        report.update(dream_requests=len(requests),dream_sessions=sessions)
        if requests:(a.out/'dream-requests.json').write_text(json.dumps(requests,indent=2)+'\n')
        save();print(json.dumps({k:report.get(k) for k in ('status','completed_steps','wall_seconds','error')}),flush=True)
        if app:app.close()


if __name__=='__main__':main()
