"""Single-GPU bottle task checks and contact-only demonstrations."""
import argparse,fcntl,importlib,json,os,subprocess,threading,time,traceback
from pathlib import Path


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--mode',choices=['check','demo','bath_held','unsafe_start'],default='check')
    parser.add_argument('--side',default='right',help=argparse.SUPPRESS)
    parser.add_argument('--interactive',action='store_true')
    parser.add_argument('--physics-only',action='store_true')
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    visible=subprocess.check_output(['nvidia-smi','--query-gpu=uuid','--format=csv,noheader'],text=True).strip().splitlines()
    expected=os.environ.get('YAM_EXPECTED_GPU_UUID')
    if visible!=[expected]:raise SystemExit('Expose exactly the explicitly allocated single GPU')
    lock=Path('/outputs/.scene-run.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    os.environ['OMNICLIENT_HUB_MODE']='disabled'
    print('BOTTLE_STAGE importing Isaac Lab',flush=True)
    from isaaclab.app import AppLauncher
    app=AppLauncher(headless=True,enable_cameras=not args.physics_only,device='cuda:0',multi_gpu=False,
        kit_args=f'--portable-root {Path.home()/"kit"} --/renderer/multiGpu/enabled=false --/renderer/multiGpu/autoEnable=false --/renderer/multiGpu/maxGpuCount=1 --/renderer/activeGpu=0 --/plugins/carb.tasking.plugin/threadCount=8').app
    import torch
    from PIL import Image
    import task,demo,demo_runtime,scoring
    from yam_arena.constants import READY
    env=None;code=1
    try:
        env,specs=task.build_environment(enable_cameras=not args.physics_only,width=640,height=360)
        command={'mode':args.mode};index=0
        while True:
            folder=args.out/f'trial-{index:02d}';folder.mkdir(exist_ok=True)
            reader=demo_runtime.StateReader(env,specs);env.reset();reader.reset()
            ready=torch.tensor([[*READY,1.,*READY,1.]],device='cuda:0')
            for _ in range(60):env.step(ready)
            reader.scorer.reset()
            if not args.physics_only:
                for _ in range(4):env.unwrapped.sim.render()
                Image.fromarray(reader.rgb('top_cam')).save(folder/'setup.png')
            report=dict(mode=command['mode'],scoring=reader.scorer.configuration(),initial=reader.controls())
            recorder=None;policy=None
            try:
                if command['mode']=='check':
                    for _ in range(30):env.step(ready);result=reader.score()
                    report.update(result);report['passed']=result['score']==0 and not result['invalid_state_ever'] and not result['bottle_in_bath'] and abs(result['door_angle_deg']-35)<2
                else:
                    recorder=demo_runtime.VideoRecorder(reader,folder) if not args.physics_only and command.get('record',True) else None
                    policy=demo.Demo(env,reader,recorder=recorder,output=folder,parameters=command)
                    if recorder:recorder.capture()
                    report.update(policy.run())
                    target={'demo':1.,'bath_held':.5,'unsafe_start':0.}[command['mode']]
                    report['passed']=report['score']==target and (command['mode']!='unsafe_start' or report['microwave_started'])
            except Exception:
                report['passed']=False;report['error']=traceback.format_exc();traceback.print_exc();report.update(reader.score())
            finally:
                if recorder:recorder.close();report['video_frames']=recorder.frames
                if policy:report.update(policy.report())
                report['final']=reader.controls()
                report['arms']={s:{'joints':reader.arm(s)[0].tolist(),'grip':reader.arm(s)[1],'tcp':reader.arm(s)[2].tolist(),'rotation':reader.arm(s)[3].tolist()} for s in ['left','right']}
                if not args.physics_only:Image.fromarray(reader.rgb('top_cam')).save(folder/'final.png')
                (folder/'result.json').write_text(json.dumps(report,indent=2)+'\n')
                print('TRIAL_RESULT',json.dumps({k:report.get(k) for k in ['mode','passed','score','bottle_in_bath','microwave_started','error']}),flush=True)
            code=0 if report['passed'] else 2;index+=1
            if not args.interactive:break
            request=args.out/'command.json';print('WAITING_FOR_COMMAND',str(request),flush=True)
            while not request.exists():time.sleep(.5)
            command=json.loads(request.read_text());request.unlink()
            if command.get('mode')=='stop':break
            importlib.invalidate_caches();scoring=importlib.reload(scoring);demo_runtime=importlib.reload(demo_runtime);demo=importlib.reload(demo)
    finally:
        timer=threading.Timer(25,lambda:os._exit(code));timer.daemon=True;timer.start()
        try:
            if env is not None:env.close()
            app.close()
        finally:timer.cancel()
    return code


if __name__=='__main__':raise SystemExit(main())
