"""Record unaltered camera frames, joint commands, and native contact events."""
from pathlib import Path
import json,sys,time,traceback,subprocess,faulthandler
faulthandler.dump_traceback_later(90,repeat=True)
from inspect_robots import Scene
from rally_task.embodiment import make_embodiment,FEEDBACK
from rally_task.policy import RallyPolicy
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
emb=make_embodiment();pol=RallyPolicy()
from inspect_robots.approver import ChainApprover,ClampApprover,DeltaLimitApprover
guard=ChainApprover(ClampApprover(emb.info.action_space),DeltaLimitApprover(emb.info.action_space));guard_store={}
scene=Scene(id='rebound',instruction='Keep the practice ball returning from the tabletop rebound board for three complete rallies.')
import numpy as np
def serial(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)
writers={};started=time.monotonic();error=None;steps=0
try:
 obs=emb.reset(scene,seed=0);pol.reset(scene)
 from runtime_audit import capture
 (out/'runtime-audit.json').write_text(json.dumps(capture(emb,obs,['paddle', 'ball']),indent=2))
 import imageio.v3 as iio
 for name,im in obs.images.items():
  h,w=im.shape[:2];writers[name]=subprocess.Popen(['ffmpeg','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{w}x{h}','-r','20','-i','-','-an','-c:v','libx264','-threads','1','-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(out/(name+'.mp4'))],stdin=subprocess.PIPE)
  iio.imwrite(out/('initial-'+name+'.png'),im)
 for steps in range(480):
  a=guard.review(pol.act(obs).actions[0],guard_store);result=emb.step(a);obs=result.observation
  for n,w in writers.items():w.stdin.write(obs.images[n].tobytes())
  if steps%10==0:print('PROGRESS',steps,json.dumps(FEEDBACK,default=serial),flush=True)
  if result.terminated:break
except Exception as e:error=str(e);traceback.print_exc()
finally:
 for w in writers.values():w.stdin.close();w.wait(timeout=30)
 result=dict(score=emb.score,error=error,steps=steps+1,wall_seconds=time.monotonic()-started,events=getattr(emb,'events',[]),revision=emb.info.environment_revision)
 (out/'policy-debug.json').write_text(json.dumps(getattr(pol,'trace',[]),default=serial));(out/'result.json').write_text(json.dumps(result,indent=2,default=serial));(out/'physics.json').write_text(json.dumps(emb.trace,default=serial));(out/'substep-physics.json').write_text(json.dumps(getattr(emb,'physics_trace',[]),default=serial));print('RESULT',json.dumps(result,default=serial),flush=True);emb.close()
