"""Run one isolated task with scripted or budget-guarded Dream control and MP4 recording."""
import argparse
import hashlib
import html
import importlib.util
import json
import shutil
from pathlib import Path
import subprocess
import time
import uuid

from isaaclab.app import AppLauncher
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--task',default=str(Path(__file__).with_name('task.py')))
p.add_argument('--output-dir',default='/outputs')
p.add_argument('--keep-open',action='store_true')
p.add_argument('--episodes',type=int,default=2)
p.add_argument('--policy',choices=('scripted','dream'),default='scripted')
p.add_argument('--budget-path',default='/dream-budget/ledger.sqlite3')
AppLauncher.add_app_launcher_args(p)
args=p.parse_args(); args.enable_cameras=True
if args.policy=='dream' and args.episodes!=1:p.error('Dream runs exactly one episode')
app=AppLauncher(args).app

import imageio.v3 as iio
import numpy as np
import torch
from yam_arena import station, ik
from yam_arena.constants import READY, FINGER_OPEN
from yam_arena.demo import ArmPlan
from yam_arena.embodiment import BimanualYamEmbodiment
from yam_arena.scene import object_states

path=Path(args.task).resolve()
source_hash=hashlib.sha256(path.read_bytes()).hexdigest()
spec=importlib.util.spec_from_file_location('selected_task',path)
task=importlib.util.module_from_spec(spec);spec.loader.exec_module(task)
output_dir=Path(args.output_dir).resolve()
root=output_dir/task.NAME/(time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])
root.mkdir(parents=True)
shutil.copytree(path.parent,root/'task-source',ignore=shutil.ignore_patterns('__pycache__','.runtime','.runs','.git','*.env','*.sqlite3*'))
shutil.copy2(__file__,root/'run.py')
if args.policy=='dream':
    for source in ('dream_runner.py','dream_budget.py'):
        shutil.copy2(Path(__file__).with_name(source),root/source)
source_snapshot=root/'task-source'
source_hashes={str(f.relative_to(source_snapshot)):hashlib.sha256(f.read_bytes()).hexdigest()
    for f in source_snapshot.rglob('*') if f.is_file() and f.suffix in ('.py','.json','.usda')}
cameras=('overview_cam','top_cam','left_cam','right_cam')
emb=BimanualYamEmbodiment(enable_cameras=True,contact_sensors=False,rig=station.rig(),overview_cam=True)
emb.camera_config.overview_cam.width=1280;emb.camera_config.overview_cam.height=720
eye=np.array([.95,-.80,1.45]); target=np.array([.22,0,.80])
forward=(target-eye)/np.linalg.norm(target-eye)
left=np.cross([0.,0.,1.],forward);left/=np.linalg.norm(left)
up=np.cross(forward,left)
emb.camera_config.overview_cam.offset.pos=tuple(eye)
emb.camera_config.overview_cam.offset.rot=tuple(ik.matrix_to_quat_xyzw(np.column_stack([forward,left,up])))
env=None
writers={}
dream_runner=None
def frame(name):
    x=env.unwrapped.scene[name].data.output['rgb']
    x=x.torch if hasattr(x,'torch') else x
    return np.ascontiguousarray(x[0,:,:,:3].cpu().numpy())

def heartbeat(step,summary=None):
    data={'ready':True,'scene':task.NAME,'control_steps':step,'cameras':list(cameras),
          'updated':time.time(),'run_dir':str(root),'result':summary,'policy':args.policy}
    tmp=output_dir/'status.tmp';tmp.write_text(json.dumps(data));tmp.replace(output_dir/'status.json')

def finish_writers():
    errors=[]
    for name,(proc,log) in writers.items():
        proc.stdin.close()
        if proc.wait(timeout=30): errors.append(name)
        log.close()
    writers.clear()
    if errors: raise RuntimeError(f'Video encoding failed: {errors}')

def write_report(results):
    sections=[]
    labels={'overview_cam':'전체 장면','top_cam':'상단','left_cam':'왼쪽 손목','right_cam':'오른쪽 손목'}
    for r in results:
        directory=f"episode-{r['episode']:02d}"
        videos=''.join(f'<figure><figcaption>{label}</figcaption><video controls preload="metadata" '
                       f'poster="{directory}/{cam}-setup.png" src="{directory}/{cam}.mp4"></video></figure>'
                       for cam,label in labels.items())
        sections.append(f'<section><h2>{html.escape(r["policy"])} {r["episode"]} · {r["score"]:.1f} / 1.0</h2>'
                        f'<p>{r["steps"]} steps · 시뮬레이션 {r["sim_seconds"]:.2f}초 · '
                        f'Dream API {r["dream_api_calls"]}회 · 종료: {html.escape(r["stop_reason"])} · '
                        f'<a href="{directory}/result.json">측정 결과</a></p>'
                        f'<div class="videos">{videos}</div></section>')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>트레이 분리 데모</title><style>body{margin:32px auto;padding:0 20px;max-width:1200px;background:#111820;color:#eaf0f5;font:16px system-ui}
h1{font-size:28px}p{color:#bac8d6}a{color:#8ccaff}.videos{display:grid;grid-template-columns:1fr 1fr;gap:20px}
figure{margin:0}video{width:100%;background:#000;border-radius:8px}figcaption{padding:8px 0}section{margin:36px 0}
@media(max-width:700px){.videos{grid-template-columns:1fr}}</style><h1>겹쳐진 트레이 분리</h1>
<p>안쪽 트레이를 들어 분리한 뒤, 두 트레이를 표시된 자리에 놓고 손을 뗍니다.</p>'''
    (root/'index.html').write_text(page+''.join(sections)+'</html>')

try:
    env=task.make_environment(emb,args.device)
    ready=torch.tensor([[*READY,1,*READY,1]],dtype=torch.float32,device=env.unwrapped.device)
    results=[]
    for episode in range(args.episodes):
        out=root/f'episode-{episode+1:02d}';out.mkdir()
        scorer=task.make_scorer()
        env.reset(seed=0)
        for _ in range(30):env.step(ready)
        plans={side:ArmPlan(points,station.BASES[side]) for side,points in task.demo().items()} if args.policy=='scripted' else {}
        duration=max((x.duration for x in plans.values()),default=float('inf'))
        for name in cameras:
            rgb=frame(name);iio.imwrite(out/f'{name}-setup.png',rgb)
            height,width=rgb.shape[:2]
            log=(out/f'{name}-ffmpeg.log').open('w')
            cmd=['ffmpeg','-hide_banner','-loglevel','warning','-y','-f','rawvideo','-pix_fmt','rgb24',
                 '-s',f'{width}x{height}','-r',str(task.HZ),'-i','pipe:0','-an','-c:v','libx264',
                 '-threads','2','-preset','veryfast','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(out/f'{name}.mp4')]
            writers[name]=(subprocess.Popen(cmd,stdin=subprocess.PIPE,stderr=log),log)
        held=0; dropped=False; started=time.monotonic(); peak=0.0
        stop_reason='step_limit'; steps=0; value=0.0; clear=True; m={}; clock=0.0
        action=np.array([*READY,1,*READY,1],dtype=np.float64)
        trace=(out/'trajectory.jsonl').open('w')
        initial={n:{'pos':s.pos.tolist(),'quat_xyzw':s.quat_xyzw.tolist()} for n,s in object_states(env,task.PARTS).items()}
        startup_error=None
        if args.policy=='dream':
            from dream_runner import Runner
            try:
                dream_runner=Runner(task.NAME,root.name+'-'+uuid.uuid4().hex[:12],task.INSTRUCTION,args.budget_path)
            except Exception as error:
                startup_error='dream_setup_error:'+type(error).__name__
                print('[dream] '+startup_error,flush=True)
        for step in range(task.MAX_STEPS):
            clock=(step+1)/task.HZ
            request_error=startup_error
            if args.policy=='scripted':
                action=np.concatenate([plans[side].action7(clock)[0] if side in plans else np.r_[READY,1] for side in ('left','right')])
            elif dream_runner is not None:
                state=[]
                for side in ('left','right'):
                    robot=env.unwrapped.scene[f'{side}_robot']
                    q=robot.data.joint_pos.torch[0].cpu().numpy(); names=robot.joint_names
                    state.extend(q[names.index(f'joint{i}')] for i in range(1,7))
                    state.append(float(np.clip(np.mean([q[names.index(f'joint{i}')]/FINGER_OPEN for i in (7,8)]),0,1)))
                try:
                    action=dream_runner.act({name:frame(name) for name in cameras if name!='overview_cam'},np.asarray(state))
                except Exception as error:
                    from dream_runner import RequestStopped
                    from dream_budget import BudgetStopped
                    request_error=str(error) if isinstance(error,(RequestStopped,BudgetStopped)) else 'dream_error:'+type(error).__name__
                    print('[dream] STOP '+request_error,flush=True)
            # Preserve a final recorded frame even if the first request/setup fails.
            env.step(torch.tensor(action,dtype=torch.float32,device=env.unwrapped.device)[None])
            steps=step+1
            objects=object_states(env,task.PARTS)
            poses={n:env.unwrapped.arena_world.get_pose_w(n)[0].cpu().numpy() for n in task.PARTS}
            measured={}
            for side in ('left','right'):
                robot=env.unwrapped.scene[f'{side}_robot']
                q=robot.data.joint_pos.torch[0].cpu().numpy()
                names=robot.joint_names
                joints=np.array([q[names.index(f'joint{i}')] for i in range(1,7)])
                fingers=np.array([q[names.index(f'joint{i}')] for i in (7,8)])
                tcp,_=ik.tcp_pose(joints,station.BASES[side])
                measured[side]={'joints':joints.tolist(),'fingers':fingers.tolist(),'tcp':tcp.tolist()}
            m=scorer.update(step=step+1,objects=objects,root_poses=poses,robot=measured)
            clear=m['clear']; dropped=m['dropped']; held=m['held_frames']
            value=m['score']
            peak=max(peak,value)
            trace.write(json.dumps({'step':step+1,'sim_time_s':clock,'action':action.tolist(),
                                   'robot':measured,'objects':{n:{'pos':s.pos.tolist(),'quat_xyzw':s.quat_xyzw.tolist(),
                                   'lin_vel':s.lin_vel.tolist(),'ang_vel':s.ang_vel.tolist(),
                                   'low':s.low.tolist(),'high':s.high.tolist()} for n,s in objects.items()},
                                   'root_poses':{n:x.tolist() for n,x in poses.items()},
                                   'metrics':m,'clear':clear,'dropped':dropped,'score':value})+'\n')
            for name,(proc,_) in writers.items():proc.stdin.write(frame(name).tobytes())
            if step%40==0:
                heartbeat(step+1)
                print(f'[task] episode={episode+1} step={step+1} score={value} clear={clear} errors='+str({n:round(d['bay_error_m'],3) for n,d in m['objects'].items()}),flush=True)
            if request_error:
                stop_reason=request_error;break
            if held>=round(task.SUCCESS_HOLD_S*task.HZ):
                stop_reason='success';break
            if clock>duration+5:
                stop_reason='script_finished';break
        trace.close();finish_writers()
        for name in cameras:iio.imwrite(out/f'{name}-final.png',frame(name))
        dream_report=None
        if dream_runner is not None:
            dream_runner.close()
            dream_report=dream_runner.report()
            (out/'dream.json').write_text(json.dumps(dream_report,indent=2))
            dream_runner=None
        api_calls=dream_report['budget']['reserved']['run_calls'] if dream_report else 0
        result={'task':task.NAME,'episode':episode+1,'success':held>=round(task.SUCCESS_HOLD_S*task.HZ),
                'score':value,
                'peak_score':peak,'steps':steps,'sim_seconds':clock,'wall_seconds':time.monotonic()-started,
                'clear':clear,'dropped':dropped,'final_metrics':m,'initial':initial,
                'max_ik_mm':{s:p.worst_ik_mm for s,p in plans.items()},'dream_api_calls':api_calls,
                'policy':args.policy,'stop_reason':stop_reason,'dream':dream_report,
                'evaluation_complete':stop_reason in ('success','step_limit'),
                'instruction':task.INSTRUCTION,'seed':0,
                'source_sha256':source_hashes,
                'task_sha256':source_hash,'isaac_sim':'6.1.0','control_hz':task.HZ}
        (out/'result.json').write_text(json.dumps(result,indent=2));results.append(result)
        write_report(results)
        print('[task] RESULT '+json.dumps(result),flush=True)
    (root/'summary.json').write_text(json.dumps(results,indent=2))
    # Keep the final scene available to the existing WebRTC browser.
    while args.keep_open and app.is_running():
        env.step(torch.tensor(action,dtype=torch.float32,device=env.unwrapped.device)[None])
        heartbeat(step+1,results)
        time.sleep(.05)
finally:
    if dream_runner is not None:dream_runner.close()
    if writers:finish_writers()
    if env is not None:env.close()
    app.close()
