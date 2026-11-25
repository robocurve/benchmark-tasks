"""Run one native scripted/idle/replay trial, with no model or external submission."""
import argparse
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import time
import traceback
import numpy as np


def source_hashes():
    package=Path(__file__).parent
    return {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(package.iterdir())
            if p.suffix in {'.py','.usda','.json'}}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('policy',choices=('scripted','idle','replay'))
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--steps',type=int,help='Shortened diagnostic; default is full declared budget')
    parser.add_argument('--actions',type=Path,help='Native actions.jsonl for replay')
    args=parser.parse_args()
    from inspect_robots import Action,ActionChunk,Scene,eval as inspect_eval
    from inspect_robots.guard import ClampGuard,DeltaLimitGuard,GuardChain
    from inspect_robots.policy import PolicyBase
    from inspect_robots.task import Task
    from . import fixed_scene as F
    from .adapter import make_embodiment,make_scripted
    from .grading import make_scorer
    limit=F.MAX_STEPS if args.steps is None else args.steps
    if not 1<=limit<=F.MAX_STEPS:parser.error(f'--steps must be in 1..{F.MAX_STEPS}')
    if args.policy=='replay' and args.actions is None:parser.error('replay requires --actions')
    args.out.mkdir(parents=True,exist_ok=False)
    initial_hashes=source_hashes();started=time.monotonic()
    rig=make_embodiment(media_dir=args.out/'media')
    original_step=rig.step
    action_log=(args.out/'actions.jsonl').open('x')
    def logged_step(action):
        result=original_step(action)
        action_log.write(json.dumps({'tick':rig.tick,'time_s':result.observation.state_time,
                                     'command':np.asarray(action.data,dtype=float).tolist()})+'\n')
        return result
    rig.step=logged_step
    script=make_scripted()
    class JointPolicy(PolicyBase):
        info=script.info
        def bind(self,embodiment_info):pass
        def reset(self,scene):
            self.index=0
            self.commands=([np.asarray(json.loads(line)['command'],dtype=float)
                            for line in args.actions.read_text().splitlines() if line.strip()]
                           if args.policy=='replay' else [])
        def act(self,observation):
            if args.policy=='idle':command=observation.state['joint_pos']
            else:
                if self.index>=len(self.commands):raise RuntimeError('Replay exhausted before task termination')
                command=self.commands[self.index];self.index+=1
            if np.asarray(command).shape!=(14,) or not np.isfinite(command).all():
                raise ValueError('Expected a finite 14-D actual robot action')
            return ActionChunk(actions=[Action(data=command,meta={})],control_hz=F.CONTROL_HZ)
    policy=script if args.policy=='scripted' else JointPolicy()
    try:
        guard=GuardChain(ClampGuard(rig.info.action_space),DeltaLimitGuard(rig.info.action_space,max_delta=1.),
                         names=['clamp','delta-limit'])
        task=Task('raise-a-keyboards-tilt-feet',scenes=[Scene(id='fixed-start',instruction=F.INSTRUCTION)],
                  scorer=make_scorer(),max_steps=limit)
        evaluation=inspect_eval(task,policy,rig,log_dir=args.out/'inspect',guard=guard,seed=0,
                                grader=None,store_frames=False,store_records=False,fail_on_error=True)[0]
        if not evaluation.samples or evaluation.status!='success':
            raise RuntimeError(str(evaluation.error or evaluation.status))
        score=evaluation.samples[0].reduced.get('keyboard_feet_score')
        if score is None:raise RuntimeError('Native task scorer produced no score')
        unchanged=initial_hashes==source_hashes()
        if not unchanged:raise RuntimeError('Task source changed during native trial')
        report={'policy':args.policy,'model':None,'score':score,'executed_steps':rig.tick,
            'step_limit':limit,'declared_step_limit':F.MAX_STEPS,'control_hz':F.CONTROL_HZ,
            'seed':0,'elapsed_s':round(time.monotonic()-started,2),
            'smoke_only':args.policy=='idle' or limit!=F.MAX_STEPS,
            'observations':{'cameras':['top_cam','left_cam','right_cam'],'state':['joint_pos'],'rgb_size':[320,176]},
            'scripted_reads_simulator_state':args.policy=='scripted','source_sha256':initial_hashes,
            'source_unchanged':unchanged,'runtime':{p:version(p) for p in
                ('isaacsim','inspect-robots','inspect-robots-yam','yam-arena')},
            'physics_hz':1/rig.physics_dt,'physics_samples':rig.recorder.physics_samples if rig.recorder else None,
            'dream_requests':0,'final_grade':rig.final_grade,'diagnostic_checkpoints':len(rig.diagnostics),
            'native_physics':rig.native_physics,
            'reference_phases':getattr(getattr(policy,'reference',None),'phases',None)}
        (args.out/'RESULT.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('diagnostics','source_sha256')}),flush=True)
        return int(args.policy!='idle' and score!=1.)
    except BaseException as exc:
        (args.out/'ERROR.json').write_text(json.dumps({'policy':args.policy,'error_type':type(exc).__name__,
            'message':str(exc),'traceback':traceback.format_exc(),'source_sha256':initial_hashes},indent=2)+'\n')
        raise
    finally:
        action_log.close();rig.close()


if __name__=='__main__':raise SystemExit(main())
