"""Known external spindle torque versus independent socket-contact meter.

Fixed bolt and handle pivot are diagnostic fixtures, not normal task/demo state.
"""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dataclasses import replace
from ratchet_task.embodiment import RatchetEmbodiment,ROOT
from ratchet_task.physics import Hardware
from ratchet_task.config import *
from inspect_robots import Scene
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
p=AXIS+np.array([0,0,.0007]);toolp=p+np.array([0,0,.0305])
class Bench(RatchetEmbodiment):
 def _ensure_sim(self,goal):
  exists=self._env is not None;super()._ensure_sim(goal)
  if exists:return
  from pxr import UsdPhysics,Sdf,Gf
  Hardware.entry_contact(type('StageOnly',(),{'stage':self._base.sim.stage})(),False)
  j=UsdPhysics.FixedJoint.Define(self._base.sim.stage,'/World/DiagnosticBoltSupport');j.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/bolt')]);j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,p)));j.CreateExcludeFromArticulationAttr(True)
  j=UsdPhysics.RevoluteJoint.Define(self._base.sim.stage,'/World/DiagnosticHandlePin');j.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/handle')]);j.CreateAxisAttr('Z');j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,toolp)));j.CreateExcludeFromArticulationAttr(True)
emb=Bench(task=str(ROOT/'task.py'),device='cpu',top_cam='demo',control_hz=20,cam_width=640,cam_height=360)
emb.task=replace(emb.task,objects=tuple(replace(o,pos=tuple(map(float,p))) if o.name=='bolt' else replace(o,pos=tuple(map(float,toolp))) if o.name in ['handle','drive','pawl'] else o for o in emb.task.objects))
emb.reset(Scene(id='known-torque-calibration',instruction='Bench calibration only'),seed=17)
h=Hardware(emb._base.sim.stage,1/PHYSICS_HZ);h.clutch.enabled=False;emb._base.sim.stage.RemovePrim('/World/envs/env_0/PawlGate')
rows=[]
for load in [.1,.3,.5,.7]:
 samples=[]
 for i in range(2*PHYSICS_HZ):
  h.step();h.torque('drive',-load);emb._base.sim.step(render=False);h.after_step()
  samples.append(h.telemetry.get('socket_torque_nm',0.))
 result=dict(input_torque_nm=load,mean_contact_torque_nm=float(np.mean(samples[PHYSICS_HZ:])),std_contact_torque_nm=float(np.std(samples[PHYSICS_HZ:])))
 rows.append(result);print('TORQUE_POINT',json.dumps(result),flush=True)
result=dict(bench_only=True,points=rows,passed=all(abs(r['mean_contact_torque_nm']-r['input_torque_nm'])<.02 for r in rows),revision=emb.info.environment_revision)
json.dump(result,open(out/'torque-meter-result.json','w'),indent=2);print('TORQUE_METER_COMPLETE',json.dumps(result),flush=True);emb.close()
