"""Open the authored USD, capture stills and run an action-free physics smoke test."""
import argparse
import json
from pathlib import Path

from isaaclab.app import AppLauncher

p = argparse.ArgumentParser()
p.add_argument('--output-dir',required=True,type=Path)
AppLauncher.add_app_launcher_args(p)
args = p.parse_args()
args.enable_cameras = True
app = AppLauncher(args).app

import numpy as np
import imageio.v3 as iio
import omni.usd
import carb
carb.settings.get_settings().set_bool('/rtx/raytracing/fractionalCutoutOpacity',True)
from pxr import UsdGeom, UsdPhysics, Gf, Sdf
import asyncio
import time
from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
from isaaclab.sim import SimulationContext, SimulationCfg
import isaaclab.sim.utils.stage as stage_utils

out = args.output_dir
out.mkdir(parents=True,exist_ok=True)
status = {'complete':False,'paid_inference_calls':0,'video_created':False}
try:
    assert omni.usd.get_context().open_stage(str(Path(__file__).parent/'scene.usda'))
    for _ in range(24):
        app.update()
    stage = omni.usd.get_context().get_stage()
    # task.py removes saved runtime graphs offline before Kit ever sees them.
    cameras = {'overview':'/World/envs/env_0/OverviewCam',
               'top':'/World/envs/env_0/TopCam'}
    # Use the source camera's validated optical convention for the detail view.
    # Copying its attributes avoids the YAM camera-axis convention being mixed
    # with a generic USD look-at transform. The calibrated model camera is intact.
    flat=stage.Flatten()
    # Kit edits the session layer by default; CopySpec targets the root layer.
    Sdf.CreatePrimInLayer(stage.GetRootLayer(),Sdf.Path('/World'))
    Sdf.CopySpec(flat,Sdf.Path(cameras['overview']),stage.GetRootLayer(),
                 Sdf.Path('/World/InspectionCamera'))
    detail = UsdGeom.Camera(stage.GetPrimAtPath('/World/InspectionCamera'))
    detail.GetFocalLengthAttr().Set(detail.GetFocalLengthAttr().Get()*1.7)
    cameras['detail'] = str(detail.GetPath())
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError('No headless viewport was initialized')
    viewport.set_texture_resolution((1280,720))
    def capture(path, camera):
        viewport.camera_path = Sdf.Path(camera)
        for _ in range(32):
            app.update()
        handle = capture_viewport_to_file(viewport, str(path))
        future = asyncio.ensure_future(handle.wait_for_result(0))
        for _ in range(400):
            app.update()
            if future.done():
                future.result()
                break
        else:
            raise RuntimeError('Viewport capture did not finish')
        # Capture completion and the asynchronous PNG writer complete separately.
        for _ in range(400):
            app.update()
            try:
                data=iio.imread(path)
                break
            except (FileNotFoundError, OSError, ValueError):
                time.sleep(.025)
        else:
            raise RuntimeError('PNG writer did not finish '+str(path))
        if data.size==0 or data[:,:,:3].std()<1:
            raise RuntimeError('Empty or blank capture '+str(path))
    for key,cam in cameras.items():
        capture(out/(key+'.png'),cam)
    print('STILLS_SAVED',flush=True)
    # Passive settling only: no controller, joint commands or object pose writes.
    witness=UsdGeom.Cube.Define(stage,'/World/PhysicsWitness')
    witness.CreateSizeAttr(.020)
    witness.AddTranslateOp().Set(Gf.Vec3d(1.5,1.5,1.5))
    UsdPhysics.RigidBodyAPI.Apply(witness.GetPrim())
    UsdPhysics.MassAPI.Apply(witness.GetPrim()).CreateMassAttr(.1)
    UsdPhysics.CollisionAPI.Apply(witness.GetPrim())
    # The authored physics scene is in the portable binary sublayer. Lab's
    # constructor deletes/recreates its scene; use an explicit validation scene
    # so it never tries to remove a weaker-layer prim with RemovePrim().
    stage.OverridePrim('/physicsScene').SetActive(False)
    stage_utils._context.stage = stage
    sim = SimulationContext(SimulationCfg(dt=1/120,device=args.device,use_fabric=False,
                                         physics_prim_path='/PhysicsValidationScene'))
    sim.reset()
    from scene_runtime import SceneRuntime
    runtime=SceneRuntime(stage,sim)
    rig={}
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            path=str(prim.GetPath())
            view=sim.physics_manager._view.create_articulation_view(path)
            rig[path]={'count':view.count,'dof_names':list(view.shared_metatype.dof_names),'positions':view.get_dof_positions().numpy().tolist()}
    (out/'rig.json').write_text(json.dumps(rig,indent=2))
    top_path='/World/MedicalStation/Catheter'
    cap_path='/World/MedicalStation/Syringe/Plunger'
    views={path:sim.physics_manager._view.create_rigid_body_view(path)
           for path in [top_path,cap_path,'/World/PhysicsWitness']}
    def position(path):
        transforms=views[path].get_transforms()
        values=transforms.numpy()
        if values.shape[0]!=1:
            raise RuntimeError('Expected one measured rigid body: '+path)
        return [float(x) for x in values[0,:3]]
    witness_before=position('/World/PhysicsWitness')
    before={'catheter':position(top_path),'plunger':position(cap_path)}
    samples=[]
    for frame in range(240):
        runtime.update(1/120)
        sim.step(render=False)
        if frame%12==0:
            samples.append({'frame':frame,'catheter':position(top_path),'plunger':position(cap_path)})
    after={'catheter':position(top_path),'plunger':position(cap_path)}
    witness_after=position('/World/PhysicsWitness')
    sim.pause()
    UsdGeom.Imageable(witness.GetPrim()).MakeInvisible()
    capture(out/'settled-detail.png',cameras['detail'])
    stable=bool(all(np.isfinite(v).all() for v in after.values())
                and all(np.linalg.norm(np.array(after[k])-before[k])<.15 and after[k][2]>.73 for k in after)
                and witness_before[2]-witness_after[2]>.050)
    status.update({'complete':True,'physics_smoke_passed':stable,'physics_steps':240,'physics_dt':1/120,
                   'before':before,'after':after,'samples':samples,
                   'witness_before':witness_before,'witness_after':witness_after,
                   'measurement':'PhysX rigid-body tensor views; not cached USD transforms',
                   'scope':'free-body passive settling only; no grasp or extraction success claimed','mechanics':runtime.last_metrics})
    if not stable:
        raise RuntimeError('Passive stability checks failed: '+str(after))
    print('PHYSICS_SMOKE_PASSED',json.dumps(after),flush=True)
except Exception as exc:
    status['error']=repr(exc)
    import traceback
    traceback.print_exc()
finally:
    (out/'scene-validation.json').write_text(json.dumps(status,indent=2)+'\n')
    app.close()
