"""Isolated GPU-2 scene check and fully recorded physical ice demo; no API calls."""
import argparse
import fcntl
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import traceback
import task


class RecordedEnv:
    def __init__(self,env,reader,out,recorder):
        self.env=env;self.reader=reader;self.out=out;self.recorder=recorder
        self.phase='start';self.steps=0;self.rows=[];self.actions=(out/'actions.jsonl').open('w');self.last_action=None

    def __getattr__(self,name):return getattr(self.env,name)

    def step(self,action):
        result=self.env.step(action);self.steps+=1;self.last_action=action.clone()
        self.actions.write(json.dumps(action[0].detach().cpu().tolist())+'\n')
        score=self.reader.score()
        if self.recorder:self.recorder.capture()
        if self.steps%10==0:
            poses,_=self.reader.snapshot()
            row={'step':self.steps,'phase':self.phase,'scoop_pose':poses['Scoop'].tolist(),'lid_pose':poses['CleanLid'].tolist(),**score}
            self.rows.append(row)
            if self.steps%20==0:(self.out/'progress.json').write_text(json.dumps(row,indent=2)+'\n')
        return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True);parser.add_argument('--mode',choices=['check','demo'],default='check')
    parser.add_argument('--steps',type=int,default=40);parser.add_argument('--episodes',type=int,default=1)
    parser.add_argument('--interactive',action='store_true');parser.add_argument('--physics-only',action='store_true')
    parser.add_argument('--verify-eval',action='store_true',help='Run two real Inspect hold-policy epochs after the recorded demo')
    parser.add_argument('--expected-gpu-uuid');args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    visible=subprocess.check_output(['nvidia-smi','--query-gpu=uuid','--format=csv,noheader'],text=True).strip().splitlines()
    if len(visible)!=1 or (args.expected_gpu_uuid and visible!=[args.expected_gpu_uuid]):raise SystemExit('Expose only the expected GPU 2')
    lock=Path('/outputs/.scene-run.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    (args.out/'runner.pid').write_text(str(os.getpid()))
    report={'passed':False,'checks':{},'mode':args.mode,'physical_gpu':2,'gpu_uuid':visible[0],
            'instruction':task.INSTRUCTION,'control_hz':20,'physics_hz':240,'dream_api_calls':0,'rendered':not args.physics_only}
    started=time.monotonic();app=env=reader=None;code=1
    try:
        import numpy as np
        import torch
        from PIL import Image
        from isaaclab.app import AppLauncher
        from yam_arena.constants import READY
        app=AppLauncher(headless=True,enable_cameras=not args.physics_only,device='cuda:0',multi_gpu=False,
            kit_args=f'--portable-root {Path.home()/"kit"} --/renderer/multiGpu/enabled=false --/renderer/multiGpu/autoEnable=false '
                     '--/renderer/multiGpu/maxGpuCount=1 --/renderer/activeGpu=0 --/plugins/carb.tasking.plugin/threadCount=8').app
        from runtime import Reader,VideoRecorder
        env=task.build_environment(enable_cameras=not args.physics_only)
        env.unwrapped.render_enabled=not args.physics_only
        reader=Reader(env)
        ready=torch.tensor([[*READY,1.,*READY,1.]],dtype=torch.float32,device='cuda:0')

        def reset():
            env.reset()
            authored,_=reader.snapshot()
            authored_error=max(float(np.linalg.norm(p[:3]-np.array(reader.layout['objects'][n]['position']))) for n,p in authored.items())
            for _ in range(60):env.step(ready)
            reader.reset_history()
            if not args.physics_only:
                for _ in range(4):env.unwrapped.sim.render()
            settled,velocity=reader.snapshot()
            counts={source:int(sum(abs(settled[n][0]-reader.layout['objects'][bin_name]['position'][0])<.120 and
                abs(settled[n][1]-reader.layout['objects'][bin_name]['position'][1])<.134 and
                reader.layout['objects'][bin_name]['position'][2]<settled[n][2]<reader.layout['objects'][bin_name]['position'][2]+.125
                for n,row in reader.layout['objects'].items() if row.get('ice_source')==source))
                for source,bin_name in [('clean','CleanBin'),('fish','FishBin')]}
            check={'authored_position_error_m':authored_error,'source_counts':counts,
                   'passed':authored_error<1e-5 and counts==reader.layout['ice_counts']}
            report.setdefault('reset_checks',[]).append(check)

        if args.mode=='check':
            reset();first,_=reader.snapshot();first={n:p.copy() for n,p in first.items()}
            for name in task.POLICY_CAMERAS:Image.fromarray(reader.rgb(name)).save(args.out/('initial_'+name+'.png'))
            for _ in range(args.steps):env.step(ready);reader.score()
            poses,vel=reader.snapshot();frames={n:reader.rgb(n) for n in task.POLICY_CAMERAS}
            report['cameras']={n:{'shape':list(f.shape),'std':float(f.std())} for n,f in frames.items()}
            for name,frame in frames.items():Image.fromarray(frame).save(args.out/('settled_'+name+'.png'))
            report['contacts']=reader.contacts.report()
            checks=report['checks'];checks['all_88_dynamic_objects']=len(poses)==88
            checks['valid_three_cameras']=all(f.shape==(360,640,3) and f.std()>5 for f in frames.values())
            checks['distinct_cameras']=all(np.abs(frames[a].astype(float)-frames[b].astype(float)).mean()>5 for a,b in [('top_cam','left_cam'),('top_cam','right_cam'),('left_cam','right_cam')])
            checks['finite_poses']=all(np.isfinite(p).all() for p in poses.values())
            checks['contact_reports_active']=reader.contacts.contact_count>0 and reader.contacts.error is None
            checks['ice_bin_contacts_reported']=any(any(n.startswith(('CleanIce_','FishIce_')) for n in p) for p in reader.contacts.seen)
            report['maximum_speed_m_s']=max(float(np.linalg.norm(v[:3])) for v in vel.values())
            reset();second,_=reader.snapshot()
            report['reset_position_error_m']={n:float(np.linalg.norm(first[n][:3]-p[:3])) for n,p in second.items()}
            checks['authored_resets_exact']=all(c['authored_position_error_m']<1e-5 for c in report['reset_checks'])
            checks['source_contents_restored']=all(c['source_counts']==reader.layout['ice_counts'] for c in report['reset_checks'])
            checks['tools_repeatable']=max(report['reset_position_error_m'][n] for n in ('Scoop','CleanLid'))<.005
            checks['clean_pile_repeatable']=max(v for n,v in report['reset_position_error_m'].items() if n.startswith('CleanIce_'))<.005
            report['passed']=all(checks.values())
        else:
            report['episodes']=[];index=0
            while True:
                folder=args.out/f'episode-{index:02d}';folder.mkdir(exist_ok=True);reset()
                if not args.physics_only:Image.fromarray(reader.rgb('top_cam')).save(folder/'setup.png')
                recorder=None if args.physics_only else VideoRecorder(reader,folder)
                proxy=RecordedEnv(env,reader,folder,recorder);episode={'episode':index,'passed':False};begin=time.monotonic()
                try:
                    if recorder:recorder.capture()
                    plans=json.loads((task.ROOT/'validation/ik_plan.json').read_text())
                    for name in ['CleanLid','Scoop']:
                        for row in plans[name]:
                            if row['phase']!=proxy.phase:print('DEMO_PHASE',row['phase'],flush=True)
                            proxy.phase=row['phase'];proxy.step(torch.tensor([row['action']],dtype=torch.float32,device='cuda:0'))
                    import scoop_trial
                    scoop_trial=importlib.reload(scoop_trial)
                    transfer=scoop_trial.run_scoop_trial(proxy,reader.layout,folder,initial_command=plans['Scoop'][-1]['action'][7:13],render_enabled=not args.physics_only)
                    proxy.phase='final_hold'
                    for _ in range(40):proxy.step(proxy.last_action)
                    episode.update(reader.last_score);episode['physical_transfer_passed']=transfer['passed']
                    episode['passed']=episode['score']==1. and transfer['passed']
                except Exception:
                    episode['error']=traceback.format_exc();traceback.print_exc();episode.update(reader.last_score)
                finally:
                    proxy.actions.close()
                    if recorder:recorder.close();episode['video_frames']=recorder.frames
                    if not args.physics_only:Image.fromarray(reader.rgb('top_cam')).save(folder/'final.png')
                    episode.update({'control_steps':proxy.steps,'simulated_seconds':proxy.steps/20,'wall_seconds':time.monotonic()-begin,
                                    'trajectory':proxy.rows,'scoring_events':reader.scorer.events,'contacts':reader.contacts.report()})
                    (folder/'episode.json').write_text(json.dumps(episode,indent=2)+'\n')
                report['episodes'].append({k:v for k,v in episode.items() if k not in ['trajectory','contacts']})
                report['passed']=any(e['passed'] for e in report['episodes'])
                report['checks']={'full_score_demonstrated':report['passed'],'reset_setup_valid':all(c['passed'] for c in report['reset_checks'])}
                (args.out/'demo_report.json').write_text(json.dumps(report,indent=2)+'\n')
                print('DEMO_RESULT',json.dumps({k:episode.get(k) for k in ['episode','passed','score','error']}),flush=True)
                index+=1
                if args.interactive:
                    request=args.out/'command.json';print('WAITING_FOR_COMMAND',request,flush=True)
                    while not request.exists():time.sleep(.5)
                    command=json.loads(request.read_text());request.unlink()
                    if command.get('command')=='stop':break
                elif index>=args.episodes:break
        if args.verify_eval:
            from inspect_robots import Scene,Task,eval
            from yam_arena.scoring import task_score
            from benchmark import CleanIceEmbodiment
            from evaluate import HoldReadyPolicy
            embodiment=CleanIceEmbodiment()
            embodiment.app,embodiment.env,embodiment.reader=app,env,reader
            benchmark=Task(name='clean-ice-fish-check',scenes=[Scene(id='fixed-layout',instruction=task.INSTRUCTION)],
                scorer=task_score(),max_steps=5,epochs=2,metadata={'fixed_layout':True,'object_jitter_m':0.0})
            logs=eval(benchmark,HoldReadyPolicy(embodiment.info),embodiment,log_dir=str(args.out/'eval'),store_frames=True,seed=0)
            resets=getattr(embodiment,'validation_resets',[])
            verification={'passed':bool(logs) and all(log.status=='success' for log in logs) and len(resets)==2 and all(c['passed'] for c in resets),
                'policy':'hold_ready','dream_api_calls':0,'statuses':[log.status for log in logs],
                'epochs':2,'max_steps':5,'resets':resets}
            (args.out/'eval/verification.json').write_text(json.dumps(verification,indent=2)+'\n')
            report['evaluation']=verification;report['checks']['no_api_evaluation']=verification['passed']
            report['passed']=report['passed'] and verification['passed']
            print('NO_API_EVALUATION',verification['passed'],flush=True)
        code=0 if report['passed'] else 2
    except BaseException:
        report['passed']=False
        report['error']=traceback.format_exc();traceback.print_exc()
    finally:
        report['elapsed_s']=round(time.monotonic()-started,2)
        (args.out/('scene_check.json' if args.mode=='check' else 'demo_report.json')).write_text(json.dumps(report,indent=2)+'\n')
        print('RESULT',json.dumps({'passed':report['passed'],'checks':report['checks'],'error':report.get('error')}),flush=True)
        timer=threading.Timer(25,lambda:os._exit(code));timer.daemon=True;timer.start()
        try:
            if reader:reader.contacts.close()
            if env:env.close()
            if app:app.close()
        finally:timer.cancel()
    return code


if __name__=='__main__':raise SystemExit(main())
