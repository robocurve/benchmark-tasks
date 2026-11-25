"""Isolated neck/balloon contact fixture, never a robot demonstration.

A temporary axial test jig applies <=0.3 N to the actual catheter. Chamber
wall/valve effects are isolated; the same neck and balloon colliders are used.
"""
import argparse,json
from pathlib import Path
from isaaclab.app import AppLauncher
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output-dir',type=Path,required=True)
AppLauncher.add_app_launcher_args(p);args=p.parse_args();args.enable_cameras=False
app=AppLauncher(args).app
import numpy as np
import omni.usd
from pxr import UsdPhysics,UsdGeom,Gf
from isaaclab.sim import SimulationContext,SimulationCfg
import isaaclab.sim.utils.stage as stage_utils
from mechanics import ROOT,point

out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
report={'kind':'isolated contact test jig; not grasp/teleop','passed':False}
try:
    omni.usd.get_context().open_stage(str(Path(__file__).with_name('scene.usda')))
    for _ in range(15):app.update()
    stage=omni.usd.get_context().get_stage()
    stage.OverridePrim(ROOT+'/ValveBranchJoint').SetActive(False)
    UsdPhysics.CollisionAPI(stage.GetPrimAtPath(ROOT+'/Phantom/ChamberCollision')).CreateCollisionEnabledAttr(False)
    joint=UsdPhysics.PrismaticJoint.Define(stage,ROOT+'/ClearanceTestJig')
    joint.CreateBody1Rel().SetTargets([ROOT+'/Catheter'])
    joint.CreateLocalPos0Attr(Gf.Vec3f(.375,-.045,.851))
    joint.CreateAxisAttr('Y');joint.CreateLowerLimitAttr(-.20);joint.CreateUpperLimitAttr(.01)
    drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear')
    drive.CreateTargetPositionAttr(-.15);drive.CreateStiffnessAttr(2.);drive.CreateDampingAttr(.5);drive.CreateMaxForceAttr(.3)
    stage.OverridePrim('/physicsScene').SetActive(False);stage_utils._context.stage=stage
    sim=SimulationContext(SimulationCfg(dt=1/120,device=args.device,use_fabric=True,physics_prim_path='/ClearancePhysics'))
    sim.reset();view=sim.physics_manager._view.create_rigid_body_view(ROOT+'/Catheter')
    def center():return point(view.get_transforms().numpy()[0],np.array([0.,.132,0.]))
    for _ in range(300):sim.step(render=False)
    report['inflated_center_m']=center().tolist()
    sphere=UsdGeom.Sphere(stage.GetPrimAtPath(ROOT+'/Catheter/InflatedBalloon'))
    UsdPhysics.CollisionAPI(sphere.GetPrim()).CreateCollisionEnabledAttr(False)
    sphere.CreateVisibilityAttr('invisible')
    collapsed=UsdGeom.Sphere(stage.GetPrimAtPath(ROOT+'/Catheter/CollapsedBalloon'))
    UsdPhysics.CollisionAPI(collapsed.GetPrim()).CreateCollisionEnabledAttr(True)
    collapsed.CreateVisibilityAttr('inherited')
    # Kit's stage-change events must be pumped to deliver a live geometry edit
    # to PhysX, as the real teleop render loop does each frame.
    for _ in range(5):app.update()
    for i in range(300):
        sim.step(render=False)
        if i%12==0:app.update()
    report['deflated_center_m']=center().tolist()
    report['collision_states']=[str(sphere.GetPath()),str(collapsed.GetPath())]
    report['max_test_load_n']=.3
    report['passed']=bool(report['inflated_center_m'][1]>.070 and report['deflated_center_m'][1]<.010)
    print(json.dumps(report,indent=2),flush=True)
    assert report['passed'],'The live collision-radius change did not release the retained catheter'
except Exception as exc:
    report['error']=repr(exc)
    import traceback;traceback.print_exc()
finally:
    (out/'clearance-probe.json').write_text(json.dumps(report,indent=2)+'\n')
    app.close()
