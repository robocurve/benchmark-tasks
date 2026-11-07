"""Isolated calibrated fixtures, never a robot demo or benchmark score.

World pins and motors exist ONLY here to measure passive hardware responses.
"""
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
class Bench(RatchetEmbodiment):
    def _ensure_sim(self,goal):
        existed=self._env is not None
        super()._ensure_sim(goal)
        if existed:return
        from pxr import UsdPhysics,Sdf,Gf
        for name in ['handle','drive']:
            j=UsdPhysics.RevoluteJoint.Define(self._base.sim.stage,'/World/DiagnosticPin'+name)
            j.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/'+name)])
            j.CreateLocalPos0Attr(Gf.Vec3f(.55,0,1.12));j.CreateLocalPos1Attr(Gf.Vec3f(0));j.CreateAxisAttr('Z');j.CreateExcludeFromArticulationAttr(True)
emb=Bench(task=str(ROOT/'task.py'),device='cpu',top_cam='demo',control_hz=20,cam_width=640,cam_height=360,physics_hz=hz)
emb.task=replace(emb.task,objects=tuple(replace(o,pos=(.55,0.,1.12)) if o.name in ['handle','drive','pawl'] else o for o in emb.task.objects))
emb.reset(Scene(id='mechanism-calibration',instruction='Diagnostic only'),seed=17)
from pxr import UsdPhysics
stage=emb._base.sim.stage
hardware=Hardware(stage,1/hz)
assert np.linalg.norm(hardware.pose('handle')[0]-[.55,0,1.12])<.001,'Diagnostic shaft not at declared anchor'
handle=UsdPhysics.RevoluteJoint.Get(stage,'/World/DiagnosticPinhandle')
output=UsdPhysics.RevoluteJoint.Get(stage,'/World/DiagnosticPindrive')
motor=UsdPhysics.DriveAPI.Apply(handle.GetPrim(),'angular');motor.CreateTypeAttr('force');motor.CreateStiffnessAttr(.15);motor.CreateDampingAttr(.006);motor.CreateMaxForceAttr(.8)
brake=UsdPhysics.DriveAPI.Apply(output.GetPrim(),'angular');brake.CreateTypeAttr('force');brake.CreateStiffnessAttr(0.);brake.CreateDampingAttr(.006);brake.CreateMaxForceAttr(.04);brake.CreateTargetVelocityAttr(0.)
original=emb._base.sim.step;rows=[]
def step(*args,**kwargs):
    row=hardware.step()
    result=original(*args,**kwargs)
    rows.append(row)
    return result
emb._base.sim.step=step
for i in range(800):
    phase=(i%80)/80
    target=-30*(phase/.5 if phase<.5 else 2-2*phase)
    motor.CreateTargetPositionAttr(float(target))
    emb.step(Action(data=np.r_[READY,1.,READY,1.]))
clutch_rows=rows.copy();rows.clear()
# Use only actual dynamic angles; report forward/return intervals separately.
summary=[]
for cycle in range(10):
    samples=clutch_rows[cycle*80*(hz//20):(cycle+1)*80*(hz//20)]
    start=samples[0]['angles'];middle=samples[len(samples)//2]['angles'];end=samples[-1]['angles']
    summary.append(dict(forward_drive_deg=math.degrees(middle['drive']-start['drive']),return_backtrack_deg=math.degrees(end['drive']-middle['drive']),handle_forward_deg=math.degrees(middle['handle']-start['handle'])))
result={'fixture':'two pinned shafts with passive clutch and 0.04 N m output brake','physics_hz':hz,'clutch_cycles':summary,'output_brake_nm':.04,'clutch_pass':all(s['forward_drive_deg']<-10 and abs(s['return_backtrack_deg'])<2 for s in summary),'revision':emb.info.environment_revision}
json.dump(result,open(out/'mechanism-result.json','w'),indent=2)
json.dump(clutch_rows,open(out/'clutch-trace.json','w'))
print('MECHANISM_COMPLETE',json.dumps(result),flush=True)
emb.close()
