"""Isolated connector fixture: native-force pull-off, never a robot demo."""
import argparse
import json
from pathlib import Path
from isaaclab.app import AppLauncher
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output-dir',type=Path,required=True)
AppLauncher.add_app_launcher_args(p);a=p.parse_args();a.enable_cameras=False
app=AppLauncher(a).app
import numpy as np
import warp as wp
import omni.usd
from pxr import UsdPhysics,UsdGeom,Gf
from isaaclab.sim import SimulationContext,SimulationCfg
import isaaclab.sim.utils.stage as stage_utils
from mechanics import ROOT,BODY_PATHS,receiver_stroke
from scene_runtime import SceneRuntime
out=a.output_dir;out.mkdir(parents=True,exist_ok=True)
report={'kind':'native-force connector component fixture; not robot demo','passed':False}
try:
    omni.usd.get_context().open_stage(str(Path(__file__).with_name('scene.usda')))
    for _ in range(15):app.update()
    stage=omni.usd.get_context().get_stage();stage.OverridePrim(ROOT+'/ValveBranchJoint').SetActive(False)
    # Fixture initialization only, before the trial: put valve/receiver in
    # alignment and support each so piston return can be measured independently.
    for k,pos in [('valve',(.336,-.139,.90)),('barrel',(.264,-.139,.90)),('plunger',(.264,-.139,.90))]:
        UsdGeom.Xform(stage.GetPrimAtPath(BODY_PATHS[k])).GetOrderedXformOps()[0].Set(Gf.Vec3d(pos[0],pos[1]-.10,pos[2]))
    for k,pos in [('valve',(.336,-.139,.90)),('barrel',(.264,-.139,.90))]:
        j=UsdPhysics.FixedJoint.Define(stage,ROOT+'/Fixture_'+k)
        j.CreateBody1Rel().SetTargets([BODY_PATHS[k]]);j.CreateLocalPos0Attr(Gf.Vec3f(*pos))
    stage.OverridePrim('/physicsScene').SetActive(False);stage_utils._context.stage=stage
    sim=SimulationContext(SimulationCfg(dt=1/120,device=a.device,use_fabric=False,physics_prim_path='/ReleaseProbePhysics'))
    sim.reset();rt=SceneRuntime(stage,sim);samples=[]
    for i in range(720):
        m=rt.update(1/120);sim.step(render=False)
        if i%60==0:samples.append(m.copy())
    before=rt.update(1/120)
    assert before['connected'] and before['collapsed'],'Receiver did not physically return while connected'
    # Remove the test jig support, then apply a real finite external force.
    # There are no body pose/state writes after sim.reset().
    UsdPhysics.Joint(stage.GetPrimAtPath(ROOT+'/Fixture_barrel')).CreateJointEnabledAttr(False)
    force=wp.array(np.array([[-2.,0.,0.]],dtype=np.float32),device=sim.device)
    indices=wp.array([0],dtype=wp.int32,device=sim.device)
    for i in range(60):
        rt.views['barrel'].apply_forces_and_torques_at_position(force,None,None,indices,True)
        m=rt.update(1/120);sim.step(render=False)
        if i%6==0:samples.append(m.copy())
        if not m['connected']:break
    final=rt.update(1/120)
    report.update(before_pull=before,after_pull=final,pull_force_N=2.,samples=samples,
                  measured_receiver_stroke_m=receiver_stroke(rt.poses()),
                  physical_joint_disabled=not bool(UsdPhysics.Joint(stage.GetPrimAtPath(ROOT+'/ReceiverConnection')).GetJointEnabledAttr().Get()))
    report['passed']=bool(before['collapsed'] and not final['connected'] and final['ever_disconnected'] and not final['failed'])
    assert report['passed'],'Measured force-driven connector did not release'
except Exception as e:
    report['error']=repr(e)
    import traceback;traceback.print_exc()
finally:
    (out/'release-probe.json').write_text(json.dumps(report,indent=2)+'\n');app.close()
