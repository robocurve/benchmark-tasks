"""Record the complete rest/pickup/placement/ratcheting demonstration."""
import sys,json,traceback,time,subprocess
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import imageio.v3 as iio
from yam_arena.constants import READY
from ratchet_task.embodiment import make_embodiment,FEEDBACK,ROOT
from ratchet_task.policy import FastenPolicy
from ratchet_task.config import INSTRUCTION,MAX_STEPS,TOOL_PARK
from inspect_robots import Scene
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
def serial(v):
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 if isinstance(v,dict):return {k:serial(x) for k,x in v.items()}
 if isinstance(v,(list,tuple)):return [serial(x) for x in v]
 return v
emb=make_embodiment(task=str(ROOT/'task.py'),device='cpu',top_cam='demo',control_hz=20,cam_width=640,cam_height=360)
scene=Scene(id='confined-ratchet-rest-start-demo',instruction=INSTRUCTION)
rows=[];error=None;started=time.monotonic();last_phase=None;last_label=None;policy=FastenPolicy();initial=None;rest_verified=False;cradle_verified=False
writers={}
try:
 obs=emb.reset(scene,seed=17);policy.bind(emb.info);policy.reset(scene)
 initial=serial(FEEDBACK)
 rest_verified=all(np.max(np.abs(np.array(h['joints'])-READY))<.02 and -np.mean(h['fingers'])/.04695>.85 for h in FEEDBACK['hands'].values())
 cradle_verified=bool(np.linalg.norm(FEEDBACK['poses']['handle']-TOOL_PARK)<.005)
 if not rest_verified or not cradle_verified:raise RuntimeError('Initial rest/cradle configuration failed')
 print('REST_START_VERIFIED',rest_verified,cradle_verified,flush=True)
 for name,im in obs.images.items():
  height,width=im.shape[:2]
  writers[name]=subprocess.Popen(['ffmpeg','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{width}x{height}','-r','20','-i','-','-an','-c:v','libx264','-threads','1','-preset','veryfast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(out/(name+'.mp4'))],stdin=subprocess.PIPE)
  writers[name].stdin.write(im.tobytes());iio.imwrite(out/('initial-'+name+'.png'),im)
 for i in range(MAX_STEPS):
  action=policy.act(obs).actions[0];result=emb.step(action);obs=result.observation
  for name,writer in writers.items():writer.stdin.write(obs.images[name].tobytes())
  rows.append(serial(dict(step=i,phase=policy.phase,feedback=FEEDBACK,history=vars(emb.history))))
  label=action.meta.get('waypoint','unknown')
  if policy.phase!=last_phase or label!=last_label:
   for name,im in obs.images.items():iio.imwrite(out/(f'{i:04d}-{policy.phase}-{label.replace(" ","-")}-{name}.png'),im)
   print('ROBOT_PHASE',i,policy.phase,json.dumps(rows[-1]),flush=True);last_phase=policy.phase;last_label=label
  if i%10==0:iio.imwrite(out/(f'frame-{i:04d}.png'),obs.images['top_cam'])
  if emb.history.invalid:raise RuntimeError('Scorer invalid: '+emb.history.reason)
  if policy.stop or result.terminated:break
except Exception as e:
 error=type(e).__name__+': '+str(e);traceback.print_exc()
finally:
 for writer in writers.values():writer.stdin.close();writer.wait(timeout=30)
 result=dict(full_task_demo=True,walls=emb.walls,mechanisms=emb.mechanisms,start_from_rest=rest_verified,tool_starts_in_cradle=cradle_verified,pickup_verified=policy.pickup_verified,socket_fit_verified=policy.socket_fit_verified,max_steps=MAX_STEPS,seed=17,completed=bool(error is None and emb.history.score==1. and policy.pickup_verified and policy.socket_fit_verified),error=error,score=getattr(getattr(emb,'history',None),'score',0.),elapsed_seconds=time.monotonic()-started,steps=len(rows),revision=emb.info.environment_revision,initial=initial,last=rows[-1] if rows else None)
 result=serial(result)
 print('RATCHET_DEMO_COMPLETE',json.dumps(result),flush=True)
 (out/'demo-trace.json').write_text(json.dumps(rows))
 (out/'demo-result.json').write_text(json.dumps(result,indent=2)+'\n')
 emb.close()
