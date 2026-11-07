"""Isolated single-GPU physical checks and scripted demonstrations."""
import argparse,fcntl,importlib,json,os,subprocess,threading,time,traceback
from pathlib import Path


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--mode',choices=['check','demo','knob'],default='check')
    parser.add_argument('--side',choices=['left','right'],default='right')
    parser.add_argument('--interactive',action='store_true')
    parser.add_argument('--physics-only',action='store_true')
    parser.add_argument('--both-knobs',action='store_true')
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    if args.both_knobs and args.mode!='knob':parser.error('--both-knobs requires --mode knob')
    visible=subprocess.check_output(['nvidia-smi','--query-gpu=uuid','--format=csv,noheader'],text=True).strip().splitlines()
    expected=os.environ.get('YAM_EXPECTED_GPU_UUID')
    if not expected:raise SystemExit('Set YAM_EXPECTED_GPU_UUID to the allocated GPU UUID')
    if visible!=[expected]:raise SystemExit('Expose exactly the explicitly allocated single GPU')
    lock=Path('/outputs/.scene-run.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    os.environ['OMNICLIENT_HUB_MODE']='disabled'
    from isaaclab.app import AppLauncher
    app=AppLauncher(headless=True,enable_cameras=not args.physics_only,device='cuda:0',multi_gpu=False,
        kit_args=f'--portable-root {Path.home()/"kit"} --/renderer/multiGpu/enabled=false --/renderer/multiGpu/autoEnable=false --/renderer/multiGpu/maxGpuCount=1 --/renderer/activeGpu=0 --/plugins/carb.tasking.plugin/threadCount=8').app
    import numpy as np
    import torch
    from PIL import Image
    import task,demo
    from demo_runtime import StateReader,VideoRecorder
    from yam_arena.constants import READY
    env=None;code=1
    try:
        env,specs=task.build_environment(enable_cameras=not args.physics_only,width=640,height=360)
        reader=StateReader(env,specs)
        command={'mode':args.mode,'side':args.side};trial=0
        outcomes=[]
        while True:
            folder=args.out/f'trial-{trial:02d}';folder.mkdir(exist_ok=True)
            env.reset();reader.reset()
            ready=torch.tensor([[*READY,1.,*READY,1.]],device='cuda:0')
            for _ in range(20):env.step(ready)
            reader.scorer.reset()
            if not args.physics_only:
                for _ in range(4):env.unwrapped.sim.render()
                Image.fromarray(reader.rgb('top_cam')).save(folder/'setup.png')
            report=dict(mode=command['mode'],gpu_count=1,scoring=reader.scorer.configuration())
            recorder=None;policy=None
            try:
                report['initial_controls']=reader.controls()
                if command['mode']=='check':
                    for _ in range(60):env.step(ready);result=reader.score()
                    report.update(result)
                    report['passed']=result['score']==0 and not result['invalid_state_ever'] and abs(result['button_travel_m'])<.001 and max(map(abs,result['knob_angles_deg']))<1
                else:
                    recorder=VideoRecorder(reader,folder) if not args.physics_only else None
                    policy=demo.Demo(env,reader,recorder=recorder,output=folder,parameters=command)
                    if recorder:recorder.capture()
                    report.update(policy.run())
                    report['passed']=report['score']==(.2 if command['mode']=='knob' else 1.)
            except Exception:
                report['error']=traceback.format_exc();report['passed']=False;traceback.print_exc()
                report.update(reader.score())
            finally:
                if recorder:recorder.close();report['video_frames']=recorder.frames
                if policy:report.update(policy.report())
                report['final_controls']=reader.controls()
                if not args.physics_only:Image.fromarray(reader.rgb('top_cam')).save(folder/'final.png')
                (folder/'result.json').write_text(json.dumps(report,indent=2)+'\n')
                print('TRIAL_RESULT',json.dumps({k:report.get(k) for k in ['mode','passed','score','button_travel_m','knob_angles_deg','error']}),flush=True)
            code=0 if report['passed'] else 2
            outcomes.append(bool(report['passed']))
            trial+=1
            if args.both_knobs and trial==1:
                command={'mode':'knob','side':'left' if args.side=='right' else 'right'}
                continue
            if args.both_knobs:
                code=0 if all(outcomes) else 2
                break
            if not args.interactive:break
            request=args.out/'command.json'
            print('WAITING_FOR_COMMAND',str(request),flush=True)
            while not request.exists():time.sleep(.5)
            command=json.loads(request.read_text());request.unlink()
            if command.get('mode')=='stop':break
            demo=importlib.reload(demo)
    finally:
        timer=threading.Timer(25,lambda:os._exit(code));timer.daemon=True;timer.start()
        try:
            if env is not None:env.close()
            app.close()
        finally:timer.cancel()
    return code


if __name__=='__main__':raise SystemExit(main())
