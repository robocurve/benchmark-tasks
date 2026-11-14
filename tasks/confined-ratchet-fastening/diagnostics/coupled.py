"""Bench fixture for physical socket-to-bolt torque transfer, not a robot demo."""
import sys,json,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dataclasses import replace
from ratchet_task.embodiment import RatchetEmbodiment,ROOT
from ratchet_task.physics import Hardware
from ratchet_task.config import *
from inspect_robots import Scene
from inspect_robots.types import Action
from yam_arena.constants import READY
import numpy as np
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
hz=int(sys.argv[4]) if len(sys.argv)>4 else PHYSICS_HZ
p=AXIS+np.array([0,0,.0010]);toolp=p+np.array([0,0,.0305])
class Bench(RatchetEmbodiment):
    def _ensure_sim(self,goal):
        existed=self._env is not None
        super()._ensure_sim(goal)
        if existed:return
        from ratchet_task.physics import Hardware
        Hardware.entry_contact(type('StageOnly',(),{'stage':self._base.sim.stage})(),False)
        from pxr import UsdPhysics,Sdf,Gf
        stage=self._base.sim.stage
        self.bolt_support=UsdPhysics.FixedJoint.Define(stage,'/World/DiagnosticBoltSupport')
        self.bolt_support.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/bolt')]);self.bolt_support.CreateLocalPos0Attr(Gf.Vec3f(*map(float,p)));self.bolt_support.CreateLocalPos1Attr(Gf.Vec3f(0));self.bolt_support.CreateExcludeFromArticulationAttr(True)
        self.input=UsdPhysics.RevoluteJoint.Define(stage,'/World/DiagnosticHandlePin')
        self.input.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/handle')]);self.input.CreateLocalPos0Attr(Gf.Vec3f(*map(float,toolp)));self.input.CreateLocalPos1Attr(Gf.Vec3f(0));self.input.CreateAxisAttr('Z');self.input.CreateExcludeFromArticulationAttr(True)
emb=Bench(task=str(ROOT/'task.py'),device='cpu',top_cam='demo',control_hz=20,cam_width=640,cam_height=360,physics_hz=hz)
emb.task=replace(emb.task,objects=tuple(replace(o,pos=tuple(map(float,p))) if o.name=='bolt' else replace(o,pos=tuple(map(float,toolp))) if o.name in ['handle','drive','pawl'] else o for o in emb.task.objects))
emb.reset(Scene(id='coupled-mechanism-fixture',instruction='Bench calibration only'),seed=17)
from pxr import UsdPhysics
# Release the preparation fixture and flush its removal before making the thread.
emb._base.sim.stage.RemovePrim('/World/DiagnosticBoltSupport')
emb._base.sim.step(render=False)
hardware=Hardware(emb._base.sim.stage,1/hz,pitch_sign=int(sys.argv[2]) if len(sys.argv)>2 else -1,reverse_hinge=bool(int(sys.argv[3])) if len(sys.argv)>3 else False)
b=hardware.pose('bolt');assert np.linalg.norm(b[0]-p)<.001
hardware.capture(b[0],b[2],b[3])
motor=UsdPhysics.DriveAPI.Apply(emb.input.GetPrim(),'angular');motor.CreateTypeAttr('force');motor.CreateStiffnessAttr(20.);motor.CreateDampingAttr(.5);motor.CreateMaxForceAttr(.55)
original=emb._base.sim.step;rows=[]
def step(*args,**kwargs):
    row=hardware.step();result=original(*args,**kwargs);hardware.after_step();rows.append(row);return result
emb._base.sim.step=step
proof=0.;hold_target=None;released=False;invalid_reason=None
for i in range(1600):
    if not released:
        if hold_target is None:
            phase=(i%30)/30
            target=-30*(phase/.5 if phase<.5 else 2-2*phase)
            motor.CreateTargetPositionAttr(float(target))
        f=hardware.telemetry
        good=f.get('socket_engaged',False) and -.0003<=f.get('gap',1)<=.0002 and .4<=hardware.contact_summary(clear=False)['socket_torque_nm']<=.6
        proof=proof+.05 if good else 0.
        if not good:hold_target=None
        if good and hold_target is None:hold_target=target
        if proof>=.5:
            released=True;release_step=i;motor.CreateMaxForceAttr(0.)
    hardware.contact_summary()
    for _ in range(hz//CONTROL_HZ):emb._base.sim.step(render=False)
    measured=hardware.contact_summary(clear=False)
    hardware.telemetry.update(measured)
    if abs(hardware.telemetry.get('interface_torque',0))>1.:
        invalid_reason='over_torque';break
    if i>2 and abs(hardware.telemetry.get('pitch_error_m',0))>.0002:
        invalid_reason='pitch_constraint_error';break
    if i>180 and hardware.telemetry.get('turns',0)<.02:
        invalid_reason='no_thread_advance';break
    if i%60==0:print('COUPLED_PROGRESS',i,json.dumps(hardware.telemetry),flush=True)
    if released and i-release_step>=60:break
f=hardware.telemetry
result=dict(bench_only=True,physics_hz=hz,ratchet_cycles=f.get('ratchet_cycles'),gap_m=f.get('gap'),clockwise_turns=f.get('turns'),proof_seconds=proof,input_torque_removed=released,invalid_reason=invalid_reason,coupled_pass=bool(invalid_reason is None and released and -.0003<=f.get('gap',1)<=.0002 and f.get('turns',0)>=.5 and f.get('ratchet_cycles',0)>=6),maximum_interface_torque=max(r.get('socket_torque_nm',0) for r in rows),maximum_pitch_error_m=max(abs(r['pitch_error_m']) for r in rows),revision=emb.info.environment_revision)
json.dump(result,open(out/'coupled-result.json','w'),indent=2);json.dump(rows,open(out/'coupled-trace.json','w'))
import imageio.v3 as iio
emb.step(Action(data=np.r_[READY,1.,READY,1.]))
for name,im in emb._observation().images.items():iio.imwrite(out/(name+'-end.png'),im)
print('COUPLED_COMPLETE',json.dumps(result),flush=True);emb.close()
