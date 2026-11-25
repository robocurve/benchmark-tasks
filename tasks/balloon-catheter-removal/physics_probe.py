"""Component fixture tests, not a robot demonstration or benchmark score.

Fixtures initialize measured aligned bodies and support them like a test jig.
After initialization only the receiver's physical joint drive moves its piston.
"""
import argparse
import json
from pathlib import Path
from isaaclab.app import AppLauncher
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--output-dir',type=Path,required=True)
AppLauncher.add_app_launcher_args(p)
args=p.parse_args();args.enable_cameras=False
app=AppLauncher(args).app
import numpy as np
import warp as wp
import omni.usd
from pxr import UsdPhysics,Gf,UsdGeom,Sdf
from isaaclab.sim import SimulationContext,SimulationCfg
import isaaclab.sim.utils.stage as stage_utils
from mechanics import ROOT,BODY_PATHS,receiver_stroke
from scene_runtime import SceneRuntime

out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
report={'kind':'physical component fixtures; not teleoperation/demo','passed':False}
try:
    omni.usd.get_context().open_stage(str(Path(__file__).with_name('scene.usda')))
    for _ in range(15):app.update()
    stage=omni.usd.get_context().get_stage()
    # Detach only the valve's branch in this isolated connector test fixture.
    stage.OverridePrim(ROOT+'/ValveBranchJoint').SetActive(False)
    b=(.264,-.139,.90);v=(.336,-.139,.90)
    for key,pos in [('valve',v),('barrel',b),('plunger',b)]:
        x=UsdGeom.Xform(stage.GetPrimAtPath(BODY_PATHS[key]))
        x.GetOrderedXformOps()[0].Set(Gf.Vec3d(pos[0],pos[1]-.10,pos[2]))
    for key,pos in [('valve',v),('barrel',b)]:
        j=UsdPhysics.FixedJoint.Define(stage,ROOT+'/TestFixture_'+key)
        j.CreateBody1Rel().SetTargets([BODY_PATHS[key]])
        j.CreateLocalPos0Attr(Gf.Vec3f(*pos))
    stage.OverridePrim('/physicsScene').SetActive(False)
    stage_utils._context.stage=stage
    sim=SimulationContext(SimulationCfg(dt=1/120,device=args.device,use_fabric=False,physics_prim_path='/ProbePhysicsScene'))
    sim.reset();runtime=SceneRuntime(stage,sim)
    before=runtime.poses()
    report['initial_connector']=runtime.update(1/120)
    samples=[]
    for i in range(600):
        m=runtime.update(1/120)
        sim.step(render=False)
        if i%60==0:samples.append(m.copy())
    final=runtime.update(1/120)
    report.update(initial_stroke_m=receiver_stroke(before),final=final,samples=samples,
                  authored_balloon_radius_m=float(runtime.balloon.GetRadiusAttr().Get()),
                  measured_stroke_m=receiver_stroke(runtime.poses()))
    report['passed']=bool(final['ever_connected'] and final['collapsed']
                          and report['authored_balloon_radius_m']<.005
                          and report['measured_stroke_m']>.049)
    print(json.dumps(report,indent=2),flush=True)
    assert report['passed'],'Physical receiver coupling did not complete'
except Exception as e:
    report['error']=repr(e)
    import traceback;traceback.print_exc()
finally:
    (out/'physics-probe.json').write_text(json.dumps(report,indent=2)+'\n')
    app.close()
