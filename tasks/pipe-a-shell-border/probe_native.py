"""First GPU probe: squeeze a real passive slider with a diagnostic force.

The barrel is clamped ONLY for this constitutive/contact test. This is not a
robot demonstration and cannot be submitted. No Dream API is used.
"""
import argparse
import json
import hashlib
from pathlib import Path
import time
import traceback
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=3.)
    parser.add_argument('--force-n',type=float,default=8.)
    args=parser.parse_args()
    if not 0<args.seconds<=10 or not 0<=args.force_n<=20:parser.error('Probe limits: 10 s, 20 N')
    args.out.mkdir(parents=True,exist_ok=False)
    started=time.monotonic()
    report={'status':'running','robot_demo':False,'diagnostic_force_n':args.force_n,
            'source_sha256':{name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                             for name in ('probe_native.py','native.py','material.py')}}
    app=None;attached=False;stage=None;cache=None;physics=None
    phase='launch'
    try:
        from isaaclab.app import AppLauncher
        app=AppLauncher(headless=True,enable_cameras=False).app
        phase='imports'
        from pxr import Gf,Usd,UsdGeom,UsdPhysics,UsdUtils,PhysxSchema
        from omni.physx import get_physx_simulation_interface
        import omni.physics.tensors as tensors
        import torch
        from native import author_dispenser,Coupling,rotation
        phase='stage creation'
        cache=UsdUtils.StageCache.Get();stage=Usd.Stage.CreateInMemory();cache.Insert(stage)
        stage_id=cache.GetId(stage).ToLongInt()
        UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
        scene=UsdPhysics.Scene.Define(stage,'/physicsScene')
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.,0.,-1.));scene.CreateGravityMagnitudeAttr(9.80665)
        sx=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        sx.CreateEnableGPUDynamicsAttr(False);sx.CreateBroadphaseTypeAttr('MBP')
        sx.CreateSolverTypeAttr('TGS');sx.CreateEnableCCDAttr(True)
        def body(path,position,size,mass=None,quat=(0.,0.,0.,1.)):
            x=UsdGeom.Xform.Define(stage,path);x.AddTranslateOp().Set(Gf.Vec3d(*position))
            x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(quat[3],*quat[:3]))
            if mass:
                UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
                UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(mass)
            c=UsdGeom.Cube.Define(stage,path+'/Shape');c.CreateSizeAttr(1.)
            c.AddScaleOp().Set(Gf.Vec3d(*size));UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        phase='scene authoring'
        body('/World/Table',(.30,0.,.7375),(.6,.5,.02))
        body('/World/Board',(.30,0.,.749),(.20,.16,.003))
        body('/World/Cake',(.30,0.,.7755),(.18,.14,.05))
        # Tilt the nozzle 45 degrees down. The large barrel and nozzle shoulder
        # remain clear of the cake; the lip starts about 0.1 mm above it.
        q=np.array((np.sin(-np.pi/8),0.,0.,np.cos(-np.pi/8)))
        pos=np.array((.235,-.09,.86));axis=rotation(q)[:,1]
        body('/World/Barrel',pos,(.03,.08,.03),.080,q)
        body('/World/Plunger',pos-axis*.060,(.035,.005,.035),.025,q)
        author_dispenser(stage,'/World/Barrel','/World/Plunger',diagnostic_fixture=True)
        coupling=Coupling();coupling.author_surface(stage)
        stage.GetRootLayer().Export(str(args.out/'initial.usda'))
        phase='stage attach'
        physics=get_physx_simulation_interface();physics.attach_stage(stage_id);attached=True
        phase='tensor views'
        sim=tensors.create_simulation_view('torch',stage_id);sim.set_subspace_roots('/')
        barrel=sim.create_rigid_body_view('/World/Barrel')
        plunger=sim.create_rigid_body_view('/World/Plunger')
        if barrel.count!=1 or plunger.count!=1:raise RuntimeError('Missing native dispenser body')
        device=barrel.get_transforms().device
        coupling.prime_boundary(barrel.get_transforms().detach().cpu().numpy()[0])
        trace=[];phase='physics stepping'
        for i in range(int(args.seconds*480)):
            bp=barrel.get_transforms().detach().cpu().numpy()[0]
            pp=plunger.get_transforms().detach().cpu().numpy()[0]
            bv=barrel.get_velocities().detach().cpu().numpy()[0]
            fb,fp=coupling.advance(bp,bv,pp,1/480)
            # Force acts on the ordinary dynamic plunger; no state writes.
            fp+=rotation(bp[3:])[:,1]*(args.force_n if i<args.seconds*240 else 0.)
            for view,force in ((barrel,fb),(plunger,fp)):
                view.apply_forces_and_torques_at_position(force_data=torch.as_tensor(force,dtype=torch.float32,device=device).reshape(1,3),
                    torque_data=(torch.as_tensor(coupling.last_contact_torque,dtype=torch.float32,device=device).reshape(1,3)
                                 if view is barrel else None),
                    position_data=torch.as_tensor(bp[:3] if view is barrel else pp[:3],dtype=torch.float32,device=device).reshape(1,3),
                    indices=torch.tensor([0],dtype=torch.int32,device=device),is_global=True)
            physics.simulate(1/480,i/480);physics.fetch_results()
            if i%24==0:
                coupling.update_surface()
                trace.append({'t':i/480,'q_m':coupling.last_piston_m,
                              'reaction_n':coupling.last_reaction_n,
                              'seal_force_n':coupling.last_seal_force_n,
                              'piston_velocity_m_s':coupling.last_piston_velocity_m_s,
                              'emitted_ml':coupling.material.emitted_m3*1e6})
        coupling.material.save(args.out/'material.npz')
        report.update(status='complete',balance=coupling.material.balance(),trace=trace,
                      note='Interpret contact stability and flow; this test never establishes task feasibility.')
    except BaseException as exc:
        report.update(status='failed',error=f'{type(exc).__name__}: {exc}',error_phase=phase,
                      error_traceback=traceback.format_exc())
        raise
    finally:
        report['wall_seconds']=time.monotonic()-started
        (args.out/'probe.json').write_text(json.dumps(report,indent=2)+'\n')
        if attached:physics.detach_stage()
        if stage and cache:cache.Erase(stage)
        if app:app.close()


if __name__=='__main__':main()
