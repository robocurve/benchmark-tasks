"""Isolated native load-path check; not a robot demo or hardware measurement.

Run with the same IsaacLab Python as run_inspect.sh. A temporary revolute
support steadies the handle while known torque is ramped onto it. The socket
uses the same native sliding key to the actual free M5 nut. No support exists in the task scene.
"""
import argparse
from pathlib import Path
import json, math
parser=argparse.ArgumentParser()
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--physics-hz',type=int,choices=[480,960,1440,2400,4800],default=2400)
args=parser.parse_args()
if args.output.exists():raise SystemExit('Refusing existing calibration output')
from isaaclab.app import AppLauncher
app=AppLauncher(headless=True,device='cpu').app
from pxr import Usd,UsdGeom,UsdPhysics,UsdUtils,Gf,PhysxSchema,PhysicsSchemaTools
import omni.physx,carb
import numpy as np
import scene,spec,clamp
spec.PHYSICS_HZ=args.physics_hz
sim=omni.physx.get_physx_simulation_interface()
carb.settings.get_settings().set('/physics/updateToUsd',True)
carb.settings.get_settings().set('/physics/updateVelocitiesToUsd',True)
spec.INITIAL_YAW=0.
axis=Gf.Quatf(math.sqrt(.5),0,-math.sqrt(.5),0)
axis_r=np.array([[0,0,-1],[0,1,0],[1,0,0]])
spec.DRIVER_POS=tuple(np.array(spec.CLAMP)+axis_r@np.array([0.,0.,-.0015]))
results=[]
for load in [3.,3.2,3.5]:
    stage=Usd.Stage.CreateInMemory();scene.build(stage)
    for child in list(stage.GetPrimAtPath('/World').GetChildren()):
        if child.GetName() not in ['physics','contact','pad_contact','axial_contact','bolt','socket','driver','joints']:stage.RemovePrim(child.GetPath())
    for child in list(stage.GetPrimAtPath('/World/joints').GetChildren()):
        if child.GetName() not in ['bolt','torque_spring','socket_key']:stage.RemovePrim(child.GetPath())
    sc=UsdPhysics.Scene(stage.GetPrimAtPath('/World/physics'));sc.GetGravityMagnitudeAttr().Set(0.)
    PhysxSchema.PhysxSceneAPI(sc.GetPrim()).CreateSolverTypeAttr('TGS')
    nut=stage.GetPrimAtPath('/World/bolt');socket=stage.GetPrimAtPath('/World/socket');handle=stage.GetPrimAtPath('/World/driver')
    j=UsdPhysics.RevoluteJoint(stage.GetPrimAtPath('/World/joints/bolt'))
    j.GetBody0Rel().SetTargets([]);j.GetLocalPos0Attr().Set(Gf.Vec3f(*spec.CLAMP));j.GetLocalRot0Attr().Set(axis)
    j.GetLowerLimitAttr().Set(-170.);j.GetUpperLimitAttr().Set(170.)
    nd=UsdPhysics.DriveAPI(j.GetPrim(),'angular');nd.GetMaxForceAttr().Set(3.4)
    nr=clamp.FrictionContact(spec.CONTACT_ANGULAR_STIFFNESS,0.,True)
    anchor=UsdPhysics.RevoluteJoint.Define(stage,'/World/test_support')
    anchor.CreateBody1Rel().SetTargets(['/World/driver']);anchor.CreateAxisAttr('Z')
    anchor.CreateLocalPos0Attr(Gf.Vec3f(*spec.DRIVER_POS));anchor.CreateLocalRot0Attr(axis)
    anchor.CreateLocalPos1Attr(Gf.Vec3f(0.));anchor.CreateLocalRot1Attr(Gf.Quatf(1.))
    sid=UsdUtils.StageCache.Get().Insert(stage);sim.attach_stage(sid.ToLongInt())
    samples=[];hz=spec.PHYSICS_HZ
    for k in range(2*hz):
        sim.flush_changes()
        sim.apply_torque(sid.ToLongInt(),PhysicsSchemaTools.sdfPathToInt('/World/driver'),carb.Float3(load*min(k/(.5*hz),1.),0,0))
        sim.simulate(1/hz,k/hz);sim.fetch_results()
        if k%(hz//spec.CLAMP_UPDATE_HZ):continue
        rotations=[np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0).ExtractRotationMatrix()).T for p in [nut,socket,handle]]
        rn,rs,rh=rotations
        an=math.atan2((axis_r.T@rn)[1,0],(axis_r.T@rn)[0,0]);relative=rs.T@rh
        ac=math.atan2(relative[1,0],relative[0,0])
        nd.GetTargetPositionAttr().Set(math.degrees(nr.update(an,3.4)))
        if k in [hz//2,hz,2*hz-hz//spec.CLAMP_UPDATE_HZ]:
            samples.append({'seconds':(k+1)/hz,'nut_degrees':math.degrees(an),
                            'tool_deflection_deg':math.degrees(ac),
                            'indicated_torque_nm':-ac*spec.DRIVER_SPRING_NM_RAD})
    last=samples[-1]
    held=abs(last['nut_degrees'])<.5
    accurate=abs(last['indicated_torque_nm']-load)<=.07
    passed=accurate and (held if load<3.4 else abs(last['nut_degrees'])>5.)
    results.append({'applied_torque_nm':load,'nut_resistance_nm':3.4,'samples':samples,'passed':passed})
    sim.detach_stage();UsdUtils.StageCache.Get().Erase(sid)
report={'physics_hz':hz,'material_update_hz':spec.CLAMP_UPDATE_HZ,'solver':'TGS',
        'position_iterations':255,'known_load_tests':results,'passed':all(r['passed'] for r in results),
        'scope':'Numerical load-path calibration only; gravity-free supported tool, no robot or physical hardware claim.'}
args.output.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report));app.close()
raise SystemExit(0 if report['passed'] else 1)
