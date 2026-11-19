"""Record actual YAM robot execution with the disclosed reduced tie model."""
import argparse,hashlib,json,os,time
from pathlib import Path
import numpy as np
from material import Tie,score,rotation
HERE=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['demo','dream','hold'],default='demo');p.add_argument('--steps',type=int);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    plan=json.loads((HERE/'demo-plan.json').read_text());steps=a.steps or (len(plan['frames']) if a.mode=='demo' else 1800)
    if not 1<=steps<=1800 or (a.mode=='demo' and steps>len(plan['frames'])):p.error('Invalid action count')
    a.out.mkdir(parents=True,exist_ok=False)
    source=a.out/'source';source.mkdir()
    import shutil
    hashes={}
    for f in HERE.iterdir():
        if f.is_file():shutil.copyfile(f,source/f.name);hashes[f.name]=hashlib.sha256(f.read_bytes()).hexdigest()
    report=dict(status='initializing',mode=a.mode,steps=0,requested_steps=steps,source_sha256=hashes,control_hz=20,score=None)
    start=time.monotonic()
    def save():
        report['wall_seconds']=time.monotonic()-start;(a.out/'run.json').write_text(json.dumps(report,indent=2)+'\n')
    save();app=None;writers={};policy=None;requests=[];sessions={};trace=None
    try:
        from isaaclab.app import AppLauncher
        app=AppLauncher(headless=True,enable_cameras=True).app
        import torch,omni.usd,imageio.v2 as imageio
        from PIL import Image
        from yam_arena import inspect_embodiment as wrapper,scene
        from yam_arena.constants import TCP_OFFSET,FINGER_JOINTS,SHOULDER,WRIST,GRIPPER,FINGER_OPEN
        from review_views import configure,images
        original=scene.make_env
        def with_cameras(embodiment,**kwargs):configure(embodiment);return original(embodiment,**kwargs)
        scene.make_env=with_cameras;wrapper._APP=app
        emb=wrapper.YamArenaEmbodiment(task=str(HERE/'task.py'),object_jitter=0.,control_hz=20,cam_width=320,cam_height=176)
        emb._ensure_sim(None);base=emb._base;emb._env.reset();instruction=json.loads((HERE/'task.json').read_text())['instruction'];emb._instruction=instruction
        for _ in range(20):emb._env.step(emb._ready)
        tie=Tie()
        from render_tie import Drawing
        drawing=Drawing(omni.usd.get_context().get_stage());drawing.update(tie)
        def refresh():
            base.sim.render()
            for name in ['top_cam','left_cam','right_cam','judging_front','judging_rear','judging_side']:
                base.scene[name].update(0.,force_recompute=True)
        for _ in range(6):refresh()
        def array(value):return getattr(value,'torch',value).detach().cpu().numpy()
        def hands():
            out={}
            for side in ['left','right']:
                robot=base.scene[side+'_robot'];idx=list(robot.body_names).index('gripper');pose=array(robot.data.body_link_pose_w)[0,idx];R=rotation(pose[3:]);ids=robot.find_joints(list(FINGER_JOINTS),preserve_order=True)[0];opening=float(np.abs(array(robot.data.joint_pos)[0,ids]).sum()/.094)
                out[side]=dict(tcp=pose[:3]+R@np.asarray(TCP_OFFSET),rotation=R,opening=opening)
            return out
        for name in ['front','rear','side','policy']:
            writers[name]=imageio.get_writer(a.out/(name+'.mp4'),fps=20,codec='libx264',pixelformat='yuv420p',ffmpeg_params=['-movflags','+faststart'],macro_block_size=1)
        trace=(a.out/'trace.jsonl').open('w')
        def capture(step,action):
            obs=emb._observation();views=images(base)
            for label in ['front','rear','side']:
                rgb=views['judging_'+label];writers[label].append_data(rgb)
                if step in [0,steps]:Image.fromarray(rgb).save(a.out/(label+('-start.png' if step==0 else '-final.png')))
            writers['policy'].append_data(np.asarray(obs.images['top_cam']))
            row=dict(hands={s:{k:np.asarray(v).tolist() for k,v in h.items()} for s,h in hands().items()},reaction_forces_n={s:np.asarray(f).tolist() for s,f in tie.reaction_forces.items()},step=step,action14=list(map(float,action)),joints=np.asarray(obs.state['joint_pos']).tolist(),measurements=tie.measures(),grips={s:g['part'] for s,g in tie.grips.items()},events=tie.events)
            trace.write(json.dumps(row)+'\n');trace.flush()
        current=emb._ready.detach().cpu().numpy()[0].copy();capture(0,current)
        if a.mode=='dream':
            from inspect_robots import Scene
            from dream_policy import make_policy
            policy=make_policy(requests,sessions);policy.bind(emb.info);policy.reset(Scene(id='fit-a-loose-necktie',instruction=instruction))
        from joint_tracking import JointTracking
        trackers=[JointTracking(side=s) for s in ['left','right']]
        limit=np.tile([SHOULDER['velocity']/20]*3+[WRIST['velocity']/20]*3+[GRIPPER['velocity']/abs(FINGER_OPEN)/20],2)
        report.update(status='running',initial_measurements=tie.measures());save()
        for k in range(1,steps+1):
            if a.mode=='demo':
                wanted=np.asarray(plan['frames'][k-1]['action14']);obs=np.asarray(emb._observation().state['joint_pos'])
                for tracker in trackers:wanted=tracker.action(wanted,obs,enabled=True)
            elif policy is not None:
                chunk=policy.act(emb._observation());assert len(chunk.actions)==1;wanted=np.asarray(chunk.actions[0].data,float)
            else:wanted=current.copy()
            if wanted.shape!=(14,) or not np.isfinite(wanted).all():raise ValueError('Invalid ordinary robot action')
            wanted=np.clip(wanted,emb._low,emb._high);current=current+np.clip(wanted-current,-limit,limit)
            for side in ['left','right']:
                robot=base.scene[side+'_robot'];idx=list(robot.body_names).index('gripper')
                f=torch.as_tensor(tie.reaction_forces.get(side,np.zeros(3)),device=base.device,dtype=torch.float32).reshape(1,1,3)
                robot.set_external_force_and_torque(f,torch.zeros_like(f),body_ids=[idx],is_global=True)
            emb._env.step(torch.as_tensor(current[None,:],device=base.device,dtype=torch.float32));emb._t=k/20
            tie.update(hands());drawing.update(tie);refresh();capture(k,current)
            report.update(steps=k,measurements=tie.measures(),score=score(tie.measures()))
            if k%20==0:save()
        report.update(status='complete',events=tie.events,score=score(tie.measures()),requires_visual_review=True)
    except BaseException as error:
        report.update(status='failed',error_type=type(error).__name__,error_message=str(error));raise
    finally:
        if trace is not None:trace.close()
        for writer in writers.values():writer.close()
        if policy is not None:
            from dream_policy import public_transcript
            (a.out/'dream-transcript.json').write_text(json.dumps(public_transcript(policy),indent=2)+'\n')
        report.update(dream_requests=len(requests),dream_sessions=sessions);save()
        if app is not None:app.close()
if __name__=='__main__':main()
