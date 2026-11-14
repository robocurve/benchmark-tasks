"""Known-load clutch stiffness and multi-turn pitch bench tests, not robot demos."""
import sys,json,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dataclasses import replace
from ratchet_task.embodiment import RatchetEmbodiment,ROOT
from ratchet_task.physics import Hardware
from ratchet_task.config import *
from inspect_robots import Scene
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
mode=sys.argv[2];hz=int(sys.argv[3]);solver_type=int(sys.argv[5]) if len(sys.argv)>5 else 1;solver_velocity=int(sys.argv[6]) if len(sys.argv)>6 else SOLVER_VELOCITY;p=AXIS+np.array([0,0,.020]);shaft=np.array([.55,0.,1.12])
class Bench(RatchetEmbodiment):
 def _ensure_sim(self,goal):
  exists=self._env is not None;super()._ensure_sim(goal)
  if exists:return
  from pxr import UsdPhysics,Sdf,Gf
  Hardware.entry_contact(type('StageOnly',(),{'stage':self._base.sim.stage})(),False)
  if mode=='thread':
   j=UsdPhysics.FixedJoint.Define(self._base.sim.stage,'/World/DiagnosticStart');j.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/bolt')]);j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,p)));j.CreateExcludeFromArticulationAttr(True)
  else:
   j=UsdPhysics.FixedJoint.Define(self._base.sim.stage,'/World/DiagnosticInput');j.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/handle')]);j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,shaft)));j.CreateExcludeFromArticulationAttr(True)
   j=UsdPhysics.RevoluteJoint.Define(self._base.sim.stage,'/World/DiagnosticOutput');j.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/drive')]);j.CreateAxisAttr('Z');j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,shaft)));j.CreateExcludeFromArticulationAttr(True)
emb=Bench(task=str(ROOT/'task.py'),device='cpu',top_cam='demo',control_hz=20,cam_width=640,cam_height=360,physics_hz=hz,solver_type=solver_type,solver_velocity=solver_velocity)
emb.task=replace(emb.task,objects=tuple(replace(o,pos=tuple(map(float,p))) if mode=='thread' and o.name=='bolt' else replace(o,pos=tuple(map(float,shaft))) if mode=='clutch' and o.name in ['handle','drive','pawl'] else o for o in emb.task.objects))
emb.reset(Scene(id='mechanism-qualification',instruction='Bench only'),seed=17)
from pxr import PhysxSchema
api=PhysxSchema.PhysxSceneAPI.Get(emb._base.sim.stage,'/physicsScene')
runtime=dict(requested_device=emb.device,base_device=str(emb._base.device),simulation_device=str(emb._base.sim.device),gpu_dynamics=api.GetEnableGPUDynamicsAttr().Get() if api else None,solver_type=solver_type,solver_velocity=solver_velocity)
print('PHYSICS_RUNTIME',json.dumps(runtime),flush=True)
h=Hardware(emb._base.sim.stage,1/hz);rows=[]
def step(load,body):
 h.step();h.torque(body,load);emb._base.sim.step(render=False);h.after_step();return dict(h.telemetry)
if mode=='clutch':
 points=[]
 for load in [.1,.3,.5,.7]:
  violations=[];last_angles=[]
  for i in range(2*hz):
   row=step(load,'drive');violations.append(h.clutch.gate_relative-h.clutch.gate_phase);last_angles.append(row['angles']['drive'])
   if i%24==0:rows.append(row)
  violation=float(np.max(np.abs(violations[-hz:])))
  speed=float((last_angles[-1]-last_angles[-hz])*hz/(hz-1))
  point=dict(external_torque_nm=load,gate_error_max_rad=violation,steady_output_speed_rad_s=speed,take_up_deg=math.degrees(h.clutch.gate_phase))
  points.append(point);print('CLUTCH_POINT',json.dumps(point),flush=True)
 start=h.pose('drive')[3]
 for i in range(4*hz):row=step(-.01,'drive')
 freewheel_angle=row['angles']['drive']-start
 result=dict(bench_only=True,mode=mode,physics_hz=hz,points=points,freewheel_clockwise_rad=freewheel_angle,passed=all(p['gate_error_max_rad']<.002 and abs(p['steady_output_speed_rad_s'])<.005 for p in points) and freewheel_angle<-2*math.pi)

else:
 from pxr import UsdPhysics
 emb._base.sim.stage.RemovePrim('/World/DiagnosticStart');emb._base.sim.step(render=False)
 b=h.pose('bolt');h.capture(b[0],b[2],b[3]);a0=b[3];z0=b[0][2]
 m=UsdPhysics.DriveAPI.Apply(h.thread_joint.GetPrim(),'angular');m.CreateTypeAttr('force');m.CreateStiffnessAttr(0.);m.CreateDampingAttr(.02);m.CreateMaxForceAttr(.12)
 maximum=0.;points=[]
 for sign,turns in [(-1,12),(1,10)]:
  m.CreateTargetVelocityAttr(float(sign*360.));start=h.pose('bolt')[3];reached=False
  for i in range(40*hz):
   row=step(-sign*.015,'bolt');maximum=max(maximum,abs(row['pitch_error_m']))
   if i%24==0:rows.append(row)
   if sign*(row['angles']['bolt']-start)>=turns*2*math.pi:reached=True;break
   if maximum>.0002:break
  angle=row['angles']['bolt']-a0;travel=row['positions']['bolt'][2]-z0
  point=dict(direction=sign,reached=reached,elapsed_s=(i+1)/hz,cumulative_turns=angle/(2*math.pi),travel_m=travel,expected_travel_m=PITCH*angle/(2*math.pi),pitch_error_m=row['pitch_error_m']);points.append(point);print('PITCH_POINT',json.dumps(point),flush=True)
 result=dict(bench_only=True,mode=mode,physics_hz=hz,points=points,maximum_pitch_error_m=maximum,passed=all(p['reached'] for p in points) and maximum<.00003)
result['revision']=emb.info.environment_revision;result['runtime']=runtime
json.dump(result,open(out/'qualification-result.json','w'),indent=2);json.dump(rows,open(out/'qualification-trace.json','w'));print('QUALIFICATION_COMPLETE',json.dumps(result),flush=True);emb.close()
