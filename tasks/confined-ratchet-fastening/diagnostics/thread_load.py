"""Independent thread response with no socket, diagnostic fixtures only."""
import sys,json,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dataclasses import replace
from ratchet_task.embodiment import RatchetEmbodiment,ROOT
from ratchet_task.physics import Hardware
from ratchet_task.config import *
from inspect_robots import Scene
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
p=AXIS+np.array([0,0,.0145])
class Bench(RatchetEmbodiment):
 def _ensure_sim(self,goal):
  exists=self._env is not None;super()._ensure_sim(goal)
  if exists:return
  Hardware.entry_contact(type('StageOnly',(),{'stage':self._base.sim.stage})(),False)
  from pxr import UsdPhysics,Sdf,Gf
  j=UsdPhysics.FixedJoint.Define(self._base.sim.stage,'/World/StartSupport');j.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/bolt')]);j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,p)));j.CreateExcludeFromArticulationAttr(True)
emb=Bench(task=str(ROOT/'task.py'),device='cpu',top_cam='demo',control_hz=20,cam_width=640,cam_height=360)
emb.task=replace(emb.task,objects=tuple(replace(o,pos=tuple(map(float,p))) if o.name=='bolt' else o for o in emb.task.objects))
emb.reset(Scene(id='thread-load-bench',instruction='Diagnostic only'),seed=17)
from pxr import UsdPhysics
stage=emb._base.sim.stage;stage.RemovePrim('/World/StartSupport');emb._base.sim.step(render=False)
h=Hardware(stage,1/PHYSICS_HZ,pitch_sign=int(sys.argv[3]));b=h.pose('bolt');h.capture(b[0],b[2],b[3])
# The diagnostic motor replaces the passive angular bearing. A separate force
# brake applies only in drag mode, opposing rotation in physical SI units.
m=UsdPhysics.DriveAPI.Apply(h.thread_joint.GetPrim(),'angular');m.CreateTypeAttr('force');m.CreateStiffnessAttr(0.);m.CreateDampingAttr(.02);m.CreateMaxForceAttr(.06);m.CreateTargetVelocityAttr(-60.)
rows=[]
for i in range(1440):
 r=h.step();h.torque('bolt',float(sys.argv[2]));emb._base.sim.step(render=False);h.after_step();rows.append(r)
 if i%240==0:print('THREAD_LOAD_PROGRESS',i,json.dumps(r),flush=True)
f=rows[-1];result=dict(drag_nm=float(sys.argv[2]),pitch_sign=int(sys.argv[3]),turns=f['turns'],travel=f['positions']['bolt'][2]-rows[0]['positions']['bolt'][2],pitch_error=f['pitch_error_m'],max_pitch_error=max(abs(r['pitch_error_m']) for r in rows))
json.dump(result,open(out/'result.json','w'),indent=2);json.dump(rows,open(out/'trace.json','w'));print('THREAD_LOAD_COMPLETE',json.dumps(result),flush=True);emb.close()
