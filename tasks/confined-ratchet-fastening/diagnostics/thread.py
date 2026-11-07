"""Isolated thread travel test. Initial placement and motor are diagnostic fixtures."""
import sys,json,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ratchet_task.embodiment import RatchetEmbodiment,ROOT
from ratchet_task.physics import Hardware
from ratchet_task.config import *
from inspect_robots import Scene
from inspect_robots.types import Action
from yam_arena.constants import READY
import numpy as np
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
hz=int(sys.argv[2]) if len(sys.argv)>2 else PHYSICS_HZ
from dataclasses import replace
p=AXIS+np.array([0,0,.0145])
class ThreadBench(RatchetEmbodiment):
    def _ensure_sim(self,goal):
        existed=self._env is not None
        super()._ensure_sim(goal)
        if existed:return
        from ratchet_task.physics import Hardware
        Hardware.entry_contact(type('StageOnly',(),{'stage':self._base.sim.stage})(),False)
        from pxr import UsdPhysics,Sdf,Gf
        j=UsdPhysics.FixedJoint.Define(self._base.sim.stage,'/World/DiagnosticStartSupport')
        j.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/bolt')]);j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,p)));j.CreateLocalPos1Attr(Gf.Vec3f(0));j.CreateExcludeFromArticulationAttr(True)
        self.start_support=j
emb=ThreadBench(task=str(ROOT/'task.py'),device='cpu',top_cam='demo',control_hz=20,cam_width=640,cam_height=360,physics_hz=hz)
emb.task=replace(emb.task,objects=tuple(replace(o,pos=tuple(map(float,p))) if o.name=='bolt' else o for o in emb.task.objects))
emb.reset(Scene(id='thread-calibration',instruction='Diagnostic thread travel only'),seed=17)
from pxr import UsdPhysics
h=Hardware(emb._base.sim.stage,1/hz)
initial_pose=h.pose('bolt')
emb.start_support.CreateJointEnabledAttr(False)
h.capture(initial_pose[0],initial_pose[2],initial_pose[3])
initial=h.pose('bolt')[0]
json.dump(dict(expected=p.tolist(),actual=initial.tolist(),telemetry=h.telemetry),open(out/'thread-initial.json','w'),indent=2)
assert np.linalg.norm(initial-p)<.001,('Thread fixture not at declared starting position',initial,p,h.telemetry)

motor=UsdPhysics.DriveAPI.Apply(h.thread_joint.GetPrim(),'angular');motor.CreateTypeAttr('force');motor.CreateStiffnessAttr(0.);motor.CreateDampingAttr(.02);motor.CreateMaxForceAttr(.06)
original=emb._base.sim.step;rows=[]
def step(*args,**kwargs):
    h.step();result=original(*args,**kwargs);rows.append(dict(h.telemetry));return result
emb._base.sim.step=step
for i in range(320):
    motor.CreateTargetVelocityAttr(-60. if i<160 else 60.)
    emb.step(Action(data=np.r_[READY,1.,READY,1.]))
first=rows[hz//2];mid=rows[160*(hz//20)-1];last=rows[-1]
def interval(a,b):
    angle=b['angles']['bolt']-a['angles']['bolt'];travel=b['positions']['bolt'][2]-a['positions']['bolt'][2]
    expected=PITCH*angle/(2*math.pi)
    return dict(angle_rad=angle,travel_m=travel,expected_travel_m=expected,error_m=travel-expected)
forward=interval(first,mid);reverse=interval(mid,last)
result=dict(physics_hz=hz,forward=forward,reverse=reverse,thread_pass=forward['angle_rad']<-2*math.pi and reverse['angle_rad']>2*math.pi and abs(forward['error_m'])<.0001 and abs(reverse['error_m'])<.0001,revision=emb.info.environment_revision)
json.dump(result,open(out/'thread-result.json','w'),indent=2);json.dump(rows,open(out/'thread-trace.json','w'))
print('THREAD_COMPLETE',json.dumps(result),flush=True);emb.close()
