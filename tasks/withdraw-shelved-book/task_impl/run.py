"""Run a fixed YAM task with Inspect Robots and record the actual world and grade."""
import argparse
import hashlib
from importlib.metadata import PackageNotFoundError, version
import json
import math
import os
from pathlib import Path
import re
import time


def run(task, mode, out, bridge=None, seed=0, task_id=None, stop_after=None):
    import imageio.v2 as imageio
    import numpy as np
    from PIL import Image
    from inspect_robots import Action, ActionChunk, Scene, eval as inspect_eval
    from inspect_robots.guard import ClampGuard, DeltaLimitGuard, GuardChain
    from inspect_robots.logging.sink import NullSink
    from inspect_robots.policy import PolicyBase, PolicyInfo
    from inspect_robots.task import Task
    from task_impl.adapter import make_embodiment, make_scripted
    from task_impl.grading import make_scorer
    from yam_arena.station import TABLE_TOP
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.urdf import quat_xyzw_to_matrix
    from scipy.spatial.transform import Rotation
    import itertools

    card = json.loads((task / 'task.json').read_text())
    limit = int(re.match(r'([0-9,]+)\s+steps?\b', card['run_limit'], re.I)[1].replace(',', ''))
    # Slow settling and multi-second hold rules can reward waiting late in a trial.
    # Give the stationary adversarial baseline the same horizon as both active policies.
    if stop_after is not None and (mode!='scripted' or not 1<=stop_after<=limit):
        raise ValueError('A diagnostic step cap applies only to a bounded scripted replay')
    horizon = limit if stop_after is None else stop_after
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    scorer = make_scorer()
    embodiment = make_embodiment()
    gauge_bodies={entry[key] for entry in embodiment.spec['metrics'] if entry['kind']=='hex_key_blocked'
                  for key in ('a','b')}
    def gauge_poses():
        return {body:{'positionM':embodiment.frames[body][0].tolist(),'rotation':embodiment.frames[body][1].tolist()}
                for body in sorted(gauge_bodies)}

    def fixture_projection(width,height):
        cam=embodiment.rig.top_cam
        if cam.convention!='opengl':
            raise RuntimeError('Unrecognized station camera convention')
        lens=cam.lens.resized(width,height)
        basis=quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw))
        bounds={}
        marks={}
        for body in embodiment.spec['actors']:
            center,rot,_=embodiment.frames[body['name']]
            projected=[]
            solids=body['parts']
            for index,part in enumerate(solids+body.get('visuals',[])):
                corners=np.array(list(itertools.product((-0.5,0.5),repeat=3)))*part['size']
                local=Rotation.from_euler('xyz',part['euler_deg'],degrees=True).apply(corners)+part['offset']
                world=local@rot.T+center+[0.,0.,float(TABLE_TOP)]
                optical=(world-cam.pos)@basis
                if np.any(optical[:,2]>=-0.01):
                    raise ValueError('Fixture geometry behind camera: '+body['name'])
                points=np.c_[lens.cx-lens.fx*optical[:,0]/optical[:,2],
                             lens.cy+lens.fy*optical[:,1]/optical[:,2]]
                in_frame=np.minimum(points.max(axis=0),[width,height])-np.maximum(points.min(axis=0),[0.,0.])
                if np.any(in_frame<=0):
                    raise ValueError(f"Fixture component outside setup camera: {body['name']} part {index}")
                if index<len(solids):
                    projected.extend(points)
                else:
                    mark=f"{body['name']}:{index-len(solids)}"
                    marks[mark]=[round(float(n),1) for n in (*points.min(axis=0),*points.max(axis=0))]
            lo=np.min(projected,axis=0);hi=np.max(projected,axis=0)
            visible=np.minimum(hi,[width,height])-np.maximum(lo,[0.,0.])
            if np.any(visible<1.0):
                raise ValueError('Fixture outside setup camera or below one pixel: '+body['name'])
            bounds[body['name']]=[round(float(n),1) for n in (*lo,*hi)]
        return bounds,marks

    class Idle(PolicyBase):
        info = PolicyInfo('hold_initial_joints', embodiment.info.action_space, control_hz=20.)
        held = None

        def act(self, observation):
            if self.held is None:
                self.held = np.array(observation.state['joint_pos'], copy=True)
            return ActionChunk([Action(self.held)], control_hz=20.)

    class Record(NullSink):
        critical = True
        steps = 0
        frames = 0

        def capture(self, obs, initial=False):
            cams = [np.asarray(obs.images[k]) for k in ('top_cam', 'left_cam', 'right_cam')]
            if any(c.dtype != np.uint8 or c.ndim != 3 or c.shape[-1] != 3 for c in cams):
                raise ValueError('Expected RGB uint8 arrays from all three YAM cameras')
            native=embodiment._base.scene['top_cam'].data.output['rgb']
            native=native.torch if hasattr(native,'torch') else native
            overhead=np.ascontiguousarray(native[0,...,:3].cpu().numpy(),dtype=np.uint8)
            height,width=overhead.shape[:2]
            if initial:
                if np.std(overhead) < 3:
                    raise ValueError('Overhead camera is empty')
                Image.fromarray(overhead).save(out / 'setup.png')
                self.projected,self.ink_bounds=fixture_projection(width,height)
                self.dimensions={'nativeOverview':[width,height],'policyRGB':[cams[0].shape[1],cams[0].shape[0]]}
            # Preserve original renderer pixels and the exact model inputs at their own resolution.
            frame = Image.new('RGB', (width+cams[1].shape[1],height))
            frame.paste(Image.fromarray(overhead))
            for i, image in enumerate(cams[1:]):
                frame.paste(Image.fromarray(image), (width,(height//2)*i))
            self.video.append_data(np.asarray(frame))
            self.frames+=1

        def log_step(self, t, observation, action, result):
            if t != self.steps:
                raise ValueError('Nonconsecutive Inspect Robots control steps')
            grade = dict(result.info['grade'])
            value = float(grade['score'])
            if not math.isfinite(value) or not 0 <= value <= 1:
                raise ValueError('Grader produced invalid score')
            self.steps += 1
            if self.steps % 2 == 0 or result.terminated or self.steps==horizon:
                self.capture(result.observation)
            measured=np.asarray(result.observation.state['joint_pos'])[[6,13]]
            jaws=(measured*(2*abs(float(FINGER_OPEN)))).round(6).tolist()
            row={'step':self.steps, 'grade':grade,'jawGapM':jaws}
            if gauge_bodies:
                row['gaugePoses']=gauge_poses()
            self.trace.write(json.dumps(row, allow_nan=False,
                                        default=lambda x:x.tolist())+'\n')
            self.final = value

    record = Record()
    unrecorded_reset=embodiment.reset
    def reset_and_record(scene,*,seed=None):
        obs=unrecorded_reset(scene,seed=seed)
        record.reset_measurements=embodiment._measure(obs)
        record.reset_gauge_poses=gauge_poses()
        record.capture(obs,initial=True)
        return obs
    embodiment.reset=reset_and_record
    if mode == 'dream':
        from inspect_robots_dream.policy import DreamPolicy
        policy = DreamPolicy(model='dreamlabs/dream-1-beta', server_url=bridge, record=False, timeout_s=65.)
    else:
        policy = make_scripted() if mode == 'scripted' else Idle()
    definition = Task(task_id or task.name, scenes=[Scene(id='fixed-start',instruction=card['instruction'])],
                      scorer=scorer, max_steps=horizon)
    space = embodiment.info.action_space
    guards = GuardChain(ClampGuard(space), DeltaLimitGuard(space,max_delta=1.),names=['clamp','delta-limit'])
    try:
        with (out/'trace.jsonl').open('x') as record.trace, imageio.get_writer(
                str(out/'trial.mp4'), fps=10, codec='libx264', pixelformat='yuv420p', macro_block_size=1,
                output_params=['-movflags','+faststart']) as record.video:
            log = inspect_eval(definition,policy,embodiment,log_dir=str(out/'inspect'),sinks=[record],
                               guard=guards,seed=seed,store_frames=False,store_records=True,fail_on_error=True)[0]
        if log.status != 'success' or len(log.samples) != 1:
            raise RuntimeError('Inspect Robots did not complete one scene')
        final = float(log.samples[0].reduced[scorer.name])
        if not record.steps or record.final != final or record.steps > horizon:
            raise ValueError('Inspect scorer and physical trace disagree')
        versions={}
        for package in ('isaacsim','isaaclab','isaaclab-arena','inspect-robots','yam-arena','inspect-robots-dream','dreamlabs-client'):
            try:versions[package]=version(package)
            except PackageNotFoundError:pass
        report = {'task':task_id or task.name,'policy':mode,'seed':seed,'model':'dreamlabs/dream-1-beta' if mode=='dream' else None,
                  'scorer':scorer.name,'score':final,'steps':record.steps,'step_limit':horizon,
                  'declaredStepLimit':limit,'diagnosticOnly':stop_after is not None,
                  'elapsed_s':round(time.monotonic()-started,2),'control_hz':20,
                  'initialBoundsPixels':record.projected,'initialMarkBoundsPixels':record.ink_bounds,
                  'initialMeasurements':record.reset_measurements,
                  'initialGaugePoses':record.reset_gauge_poses,
                  'video_frames':record.frames,'cameraPixels':record.dimensions,
                  'nativeVersions':versions,'gpu':embodiment._torch.cuda.get_device_name(0),
                  'observedSourceHashes':{name:hashlib.sha256((task/name).read_bytes()).hexdigest()
                       for name in ('task_impl/fixture.json','task_impl/grading.py','task_impl/adapter.py','task_impl/scripted.py')},
                  'inspect_robots':version('inspect-robots'),'yam_arena':version('yam-arena')}
        (out/'result.json').write_text(json.dumps(report, allow_nan=False, indent=2))
    finally:
        embodiment.close()
        if hasattr(policy,'close'):
            policy.close()


if __name__ == '__main__':
    import sys
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=('idle','scripted','dream'))
    source=Path(__file__).resolve().parent
    parser.add_argument('--task',type=Path,default=source.parent if source.name=='task_impl' else source)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--task-id',help='Stable task slug when the task is staged in a temporary directory')
    parser.add_argument('--bridge',help='Optional official Dream bridge URL')
    parser.add_argument('--stop-after',type=int,help='Optional shorter scripted diagnostic; excluded from qualification')
    args = parser.parse_args()
    sys.path.insert(0,str(args.task.resolve()))
    run(args.task.resolve(),args.mode,args.out.resolve(),args.bridge,task_id=args.task_id,stop_after=args.stop_after)
