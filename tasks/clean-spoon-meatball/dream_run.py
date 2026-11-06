"""One recorded Dream-1 trial on the verified meatball environment."""
import argparse
import json
import os
from pathlib import Path
import signal
import time
import traceback

import task
from benchmark import CleanSpoonEmbodiment


class RecordingSpoon(CleanSpoonEmbodiment):
    def __init__(self,out):
        super().__init__();self.out=out;self.recorder=None;self.last_info={};self.started=time.monotonic()

    def progress(self,stage):
        data={'stage':stage,'control_steps':self.steps,'simulated_seconds':self.steps/20,
              'wall_seconds':time.monotonic()-self.started,**self.last_info}
        (self.out/'progress.json').write_text(json.dumps(data,indent=2)+'\n')

    def reset(self,scene,*,seed=None):
        self.progress('starting_simulator')
        observation=super().reset(scene,seed=seed)
        from PIL import Image
        from demo_runtime import VideoRecorder
        Image.fromarray(self.reader.rgb('top_cam')).save(self.out/'setup.png')
        self.recorder=VideoRecorder(self.reader,self.out);self.recorder.capture()
        self.progress('rollout')
        return observation

    def step(self,action):
        result=super().step(action);self.last_info=dict(result.info)
        self.recorder.capture()
        if self.steps%20==0 or result.terminated:self.progress('rollout')
        if self.steps%100==0:print('DREAM_STEP',self.steps,'score',self.last_info.get('score'),flush=True)
        return result

    def close_recording(self):
        if self.recorder is not None:
            self.recorder.close();self.recorder=None
        if self.reader is not None:
            from PIL import Image
            Image.fromarray(self.reader.rgb('top_cam')).save(self.out/'final.png')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True);parser.add_argument('--max-steps',type=int,default=2400)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    (args.out/'runner.pid').write_text(str(os.getpid()))
    def interrupted(*_):raise KeyboardInterrupt('Dream run interrupted')
    signal.signal(signal.SIGTERM,interrupted)
    # Resolve the supplied private file explicitly, rather than inheriting an image key.
    from dream import credentials
    loaded=credentials.load(env={'DREAM_CREDENTIALS':str(Path.home()/'.dreamlabs/credentials')})
    if not loaded.api_key:raise RuntimeError('Dream API key is absent from the private credentials file')
    os.environ['DREAM_API_KEY']=loaded.api_key
    os.environ['DREAM_GATEWAY_URL']=loaded.gateway_url
    os.environ['DREAM_API_URL']=loaded.platform_url
    os.environ['DREAM_DATA_URL']=loaded.data_url
    from inspect_robots import Task,Scene,eval
    from inspect_robots_dream.policy import DreamPolicy
    from yam_arena.scoring import task_score
    embodiment=RecordingSpoon(args.out);policy=None
    benchmark=Task(name='clean-spoon-meatball',scenes=[Scene(id='fixed-layout',instruction=task.INSTRUCTION)],
        scorer=task_score(),max_steps=args.max_steps,epochs=1,metadata={'fixed_layout':True,'object_jitter_m':0.0})
    result={'policy':'dream','max_steps':args.max_steps,'epochs':1,'seed':0,'instruction':task.INSTRUCTION,
            'environment_revision':embodiment.info.environment_revision,'evaluation_completed':False,'task_score':None}
    code=1;started=time.monotonic()
    try:
        policy=DreamPolicy(record=False)
        print('DREAM_RUN_START',json.dumps(result),flush=True)
        logs=eval(benchmark,policy,embodiment,log_dir=str(args.out/'inspect'),store_frames=True,seed=0,fail_on_error=True)
        result['statuses']=[log.status for log in logs]
        result['evaluation_completed']=bool(logs) and all(log.status=='success' for log in logs)
        result['results']=[{'total_trials':log.results.total_trials,'errored_trials':log.results.errored_trials,
                           'metrics':dict(log.results.metrics)} for log in logs]
        if result['evaluation_completed']:
            result['task_score']=float(logs[0].results.metrics['task_score']);code=0
        else:
            # Inspect records policy failures in the sample rather than raising.
            result['evaluation_errors']=[]
            for path in sorted((args.out/'inspect').glob('*.json')):
                saved=json.loads(path.read_text())
                result['evaluation_errors'].extend(sample['error'] for sample in saved.get('samples',[])
                                                   if sample.get('error'))
    except BaseException:
        result['error']=traceback.format_exc();traceback.print_exc()
    finally:
        result.update({'control_steps':embodiment.steps,'simulated_seconds':embodiment.steps/20,
                       'wall_seconds':time.monotonic()-started,'final_state':embodiment.last_info,
                       'goal_success':bool(embodiment.last_info.get('success',False)),
                       'runtime_audit':embodiment.audit,'dream':policy.transcript() if policy is not None else {}})
        try:embodiment.close_recording()
        except Exception:result['recording_error']=traceback.format_exc();code=1
        (args.out/'dream_result.json').write_text(json.dumps(result,indent=2).replace(loaded.api_key,'[REDACTED]')+'\n')
        embodiment.progress('completed' if result['evaluation_completed'] else 'error')
        print('DREAM_RUN_RESULT',json.dumps({k:result[k] for k in ['evaluation_completed','task_score','control_steps','goal_success']}),flush=True)
        if policy is not None:policy.close()
        embodiment.close();embodiment.shutdown()
    return code


if __name__=='__main__':raise SystemExit(main())
